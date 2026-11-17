"""step2a_mpc_bench.py — 측정 A: Isaac 시뮬레이션 없이 cuRobo MPC 만 (run_isaac.sh 로 실행)

  사용법: run_isaac.sh step2a_mpc_bench.py [--steps 1200] [--tag NAME]

  - 설정: step2_scene.py (프로토타입 HIGH 조건과 같은 MpcSolverConfig, 후보 400 / horizon 30 / dt 1/60 비균일)
  - 장면: cuboid 5개 (table, wall, 정적 블록 2, 움직이는 mover 1)
  - 루프: 두 목표 사이 왕복 N 스텝. 상태 입력은 직전 명령(js_action) — 시뮬레이터가 없으므로.
          매 스텝 mover 자세를 world_collision.update_obstacle_pose 로 갱신 (시간은 solver 시간과 따로 측정)
  - 기록: 첫 호출 시간(그래프 캡처 포함), 이후 step 평균/p95/최대, torch 최대 메모리, nvidia-smi 프로세스 VRAM
  - 그래프 재캡처 확인: torch.cuda.CUDAGraph.capture_begin 호출 횟수를 단계별로 셈
  - 자세 반영 확인(probe): 같은 상태에서 mover 를 멀리 → EE 위치 → 멀리 로 옮기며
          solver 의 constraint/feasible 이 바뀌는지, 그리고 numpy 로 계산한 구-박스 거리와 일치하는지 확인
"""
import argparse
import datetime
import json
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import torch

import step2_scene as S

ap = argparse.ArgumentParser()
ap.add_argument("--steps", type=int, default=1200)
ap.add_argument("--tag", default="step2a")
ap.add_argument("--scene", default="v1")
args = ap.parse_args()
SCENE_INFO = S.set_scene(args.scene)

OUT = os.path.expanduser("~/effmass_mpc/env_check/step2")
os.makedirs(OUT, exist_ok=True)
STAMP = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
PID = os.getpid()


def log(*a):
    print("[step2a]", *a, flush=True)


log(f"start {datetime.datetime.now().isoformat(timespec='seconds')} pid={PID} steps={args.steps}")
log(f"python {sys.version.split()[0]} {sys.executable}")
log(f"torch {torch.__version__} cuda {torch.version.cuda} device {torch.cuda.get_device_name(0)}")

CAPTURES = S.install_graph_capture_counter(torch)

T_IMPORT0 = time.perf_counter()
import curobo
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util.logger import setup_curobo_logger

setup_curobo_logger("warn")
log(f"curobo {getattr(curobo, '__version__', '?')} from {curobo.__file__} (import {time.perf_counter()-T_IMPORT0:.2f}s)")

torch.manual_seed(S.SEED)
torch.cuda.manual_seed_all(S.SEED)
np.random.seed(S.SEED)
ta = TensorDeviceType()

# ---------------- solver 생성 ----------------
torch.cuda.reset_peak_memory_stats()
t0 = time.perf_counter()
robot_cfg = S.build_robot_cfg()
world_cfg = S.build_world_config()
mpc = S.build_mpc(robot_cfg, world_cfg)
torch.cuda.synchronize()
t_build = time.perf_counter() - t0
info = S.describe_mpc(mpc)
log(f"MpcSolver 생성 {t_build:.2f}s")
log("solver 설정 " + json.dumps({k: v for k, v in info.items() if k != "traj_dt"}))
log("traj_dt " + json.dumps(info["traj_dt"]))

checker = mpc.world_collision
cube_pose_tensor = checker._cube_tensor_list[1]
cube_ptr0 = cube_pose_tensor.data_ptr()
ATTEMPTS = S.count_step_once(mpc)
STEP_ROWS = []
log("scene " + json.dumps(SCENE_INFO))

js0 = JointState.from_position(
    mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0), joint_names=mpc.joint_names
)
goal_idx = 0
goal = Goal(current_state=js0, goal_state=js0, goal_pose=Pose.from_list(S.GOALS_WXYZ[goal_idx], ta))
t0 = time.perf_counter()
gb = mpc.setup_solve_single(goal, 1)
mpc.update_goal(gb)
torch.cuda.synchronize()
t_setup = time.perf_counter() - t0
log(f"setup_solve_single + update_goal {t_setup:.3f}s, 그래프 캡처 수(생성 단계) {len(CAPTURES)}")


def pose_error(state, goal_buffer):
    kin = mpc.rollout_fn.compute_kinematics(state)
    ee = kin.ee_pos_seq.reshape(-1, 3)[0]
    q = kin.ee_quat_seq.reshape(-1, 4)[0]
    gp = goal_buffer.goal_pose.position.reshape(-1, 3)[0]
    gq = goal_buffer.goal_pose.quaternion.reshape(-1, 4)[0]
    pe = float(torch.linalg.norm(ee - gp))
    qe = float(2 * torch.acos(torch.dot(q, gq).abs().clamp(0, 1)))
    spheres = kin.robot_spheres.reshape(-1, 4).detach().cpu().numpy() if kin.robot_spheres is not None else None
    return pe, qe, ee.detach().cpu().numpy(), spheres


current = js0.clone()
step_ms, upd_ms = [], []
arrivals, switches, infeasible = [], 0, 0
leg_start = 0
min_clear_mover = float("inf")
vram_samples = []
first_call_s = None

S.COUNTER_TAG[0] = "step_0"
for i in range(args.steps):
    if i == 1:
        S.COUNTER_TAG[0] = "loop"
    # mover 자세 갱신 (CUDA graph 가 읽는 같은 텐서에 in-place 로 기록됨)
    mp = S.mover_pose(i * S.MODEL_DT)
    u0 = time.perf_counter()
    checker.update_obstacle_pose(S.MOVER_NAME, Pose.from_list(mp, ta))
    torch.cuda.synchronize()
    upd_ms.append((time.perf_counter() - u0) * 1000)

    torch.cuda.synchronize()
    a0 = ATTEMPTS[0]
    s0 = time.perf_counter()
    result = mpc.step(current, max_attempts=S.MAX_ATTEMPTS)
    torch.cuda.synchronize()
    dt = time.perf_counter() - s0
    if i == 0:
        first_call_s = dt
        log(f"첫 mpc.step {dt*1000:.1f} ms, 누적 그래프 캡처 {len(CAPTURES)}")
    else:
        step_ms.append(dt * 1000)

    attempts = ATTEMPTS[0] - a0
    feas = bool(result.metrics.feasible.all())
    if not feas:
        infeasible += 1
    current = result.js_action.get_ordered_joint_state(mpc.joint_names).clone()

    pe, qe, ee, spheres = pose_error(current, gb)
    clr = float("nan")
    if spheres is not None:
        clr = S.sphere_box_clearance(spheres, mp[:3], S.MOVER_DIMS)
        min_clear_mover = min(min_clear_mover, clr)
    reached = pe <= S.POS_TOL_M and qe <= S.ROT_TOL_RAD
    goal_at_step, switched = goal_idx, False
    if reached:
        arrivals.append({"step": i, "goal": goal_idx, "pos_err_m": round(pe, 4), "rot_err_rad": round(qe, 4)})
    if reached or (i - leg_start) >= S.LEG_MAX_STEPS:
        goal_idx = 1 - goal_idx
        gb.goal_pose.copy_(Pose.from_list(S.GOALS_WXYZ[goal_idx], ta))
        mpc.update_goal(gb)
        switches += 1
        leg_start = i
        switched = True
    qv = current.position.reshape(-1).cpu().tolist()
    STEP_ROWS.append([i, round(dt * 1000, 4), int(feas), attempts, 0, goal_at_step, int(switched), mp[0], mp[1], mp[2],
                      float(ee[0]), float(ee[1]), float(ee[2]), round(pe, 5), round(qe, 5), round(clr, 5), 0.0,
                      round(upd_ms[-1] + dt * 1000, 4)] + [round(x, 6) for x in qv])

    if i % 100 == 0 or i == args.steps - 1:
        v = S.proc_vram_mib(PID)
        vram_samples.append(v)
        log(f"step {i:5d}  solver {dt*1000:7.2f} ms  goal {goal_idx}  pos_err {pe:.3f} m  rot_err {qe:.3f}  "
            f"mover_x {mp[0]:.3f}  proc_vram {v} MiB  captures {len(CAPTURES)}")

captures_after_loop = len(CAPTURES)
captures_by_tag = {}
for c in CAPTURES:
    captures_by_tag[c["tag"]] = captures_by_tag.get(c["tag"], 0) + 1

# ---------------- probe: 자세 갱신이 solver 결과에 반영되는지 ----------------
S.COUNTER_TAG[0] = "probe"
probe_state = current.clone()
_, _, ee_probe, spheres_probe = pose_error(probe_state, gb)
probe = []
for phase, pose in [
    ("far_1", S.FAR_POSE),
    ("at_ee", [float(ee_probe[0]), float(ee_probe[1]), float(ee_probe[2]), 1.0, 0.0, 0.0, 0.0]),
    ("far_2", S.FAR_POSE),
]:
    checker.update_obstacle_pose(S.MOVER_NAME, Pose.from_list(pose, ta))
    gt = S.sphere_box_clearance(spheres_probe, pose[:3], S.MOVER_DIMS)
    rows = []
    for k in range(3):
        r = mpc.step(probe_state.clone(), max_attempts=1)
        torch.cuda.synchronize()
        rows.append({
            "feasible": bool(r.metrics.feasible.all()),
            "constraint_sum": float(r.metrics.constraint.sum()),
        })
    tensor_pose = checker._cube_tensor_list[1][0, checker.get_obb_idx(S.MOVER_NAME, 0), :3].cpu().tolist()
    probe.append({"phase": phase, "mover_xyz_set": pose[:3], "mover_xyz_in_gpu_tensor_inverse": tensor_pose,
                  "numpy_min_clearance_m": round(gt, 4), "steps": rows})
    log(f"probe {phase:6s} mover {np.round(pose[:3], 3).tolist()}  numpy 최소 여유 {gt:+.4f} m  "
        f"solver feasible {[x['feasible'] for x in rows]}  constraint {[round(x['constraint_sum'], 2) for x in rows]}")
captures_after_probe = len(CAPTURES)
cube_ptr_same = checker._cube_tensor_list[1].data_ptr() == cube_ptr0

# ---------------- 결과 ----------------
sm = sorted(step_ms)
p95 = sm[max(0, int(round(0.95 * len(sm))) - 1)]
v_end = S.proc_vram_mib(PID)
vram_samples.append(v_end)
res = {
    "tag": args.tag,
    "start": STAMP,
    "steps": args.steps,
    "curobo_file": curobo.__file__,
    "torch": torch.__version__,
    "torch_cuda": torch.version.cuda,
    "solver": info,
    "build_s": round(t_build, 3),
    "setup_s": round(t_setup, 3),
    "first_call_ms": round(first_call_s * 1000, 2),
    "step_ms_mean": round(statistics.mean(step_ms), 3),
    "step_ms_p95": round(p95, 3),
    "step_ms_max": round(sm[-1], 3),
    "step_ms_min": round(sm[0], 3),
    "steps_over_16_7ms": int(sum(1 for x in step_ms if x > 1000 / 60)),
    "obstacle_update_ms_mean": round(statistics.mean(upd_ms), 4),
    "obstacle_update_ms_max": round(max(upd_ms), 4),
    "infeasible_steps": infeasible,
    "goal_switches": switches,
    "arrivals": len(arrivals),
    "arrival_list": arrivals[:20],
    "min_clearance_robot_to_mover_m": round(min_clear_mover, 4),
    "torch_max_allocated_mib": round(torch.cuda.max_memory_allocated() / 2**20, 1),
    "torch_max_reserved_mib": round(torch.cuda.max_memory_reserved() / 2**20, 1),
    "proc_vram_mib_max": max([v for v in vram_samples if v is not None], default=None),
    "proc_vram_mib_samples": vram_samples,
    "graph_captures_by_stage": captures_by_tag,
    "graph_captures_total_after_loop": captures_after_loop,
    "graph_captures_total_after_probe": captures_after_probe,
    "graph_capture_callers": sorted({c["caller"] for c in CAPTURES}),
    "cube_pose_tensor_same_ptr": cube_ptr_same,
    "probe": probe,
}
import csv

steps_path = os.path.join(OUT, f"{args.tag}_steps_{STAMP}.csv")
with open(steps_path, "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(S.STEP_FIELDS)
    wr.writerows(STEP_ROWS)
res["scene"] = SCENE_INFO
res["steps_csv"] = steps_path
log(f"step 기록 {len(STEP_ROWS)}행 → {steps_path} (A 에는 물리 스텝이 없어 physics_ms=0, hold=0, loop_ms=장애물 갱신+solver)")
path = os.path.join(OUT, f"{args.tag}_result_{STAMP}.json")
with open(path, "w") as f:
    json.dump(res, f, indent=2, ensure_ascii=False)

log("===== RESULT =====")
for k in ["build_s", "setup_s", "first_call_ms", "step_ms_mean", "step_ms_p95", "step_ms_max", "steps_over_16_7ms",
          "obstacle_update_ms_mean", "infeasible_steps", "goal_switches", "arrivals", "min_clearance_robot_to_mover_m",
          "torch_max_allocated_mib", "torch_max_reserved_mib", "proc_vram_mib_max", "graph_captures_by_stage",
          "graph_captures_total_after_loop", "graph_captures_total_after_probe", "cube_pose_tensor_same_ptr"]:
    log(f"{k:34s}: {res[k]}")
log("graph capture callers: " + json.dumps(res["graph_capture_callers"]))
log(f"saved {path}")

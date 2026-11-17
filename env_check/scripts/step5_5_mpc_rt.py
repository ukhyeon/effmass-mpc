"""step5_5_mpc_rt.py — 5-5: Isaac 없이 cuRobo MPC 를 60 Hz 실시간 주기로 (ROS_METHOD=min run_isaac_ros.sh 로 실행, SimulationApp 없음)

  - MPC: 측정 A(step2a) 와 같은 설정·장면(v2), 상태 입력 = 직전 명령(js_action), 점검 계산(구-박스 거리) 없음
  - ROS: 매 스텝 JointState 발행 (/step3/joint_states, 3단계와 같음), 사람: step4_human.HumanLink (표시 없음, GPU 텐서 갱신만)
  - 순서: ROS 준비 → 첫 HumanDynamicsState 대기 → 예열 10회 → 측정 루프 N 스텝
  - 실시간: k 번째 스텝 예정 시각 = 루프 시작 + k/60 s. 예정 시각까지 쉬고, 늦었으면 바로 시작 (건너뛰지 않음)
           마감 초과 = (스텝 끝 − 예정 시각) > 1/60 s
  - 출력: step2/<tag>_steps_*.csv, step2/<tag>_result_*.json, <ros-out>_human_rec.npz, <ros-out>_isaac_sent_js.csv, <ros-out>_isaac_meta.json
"""
import argparse, csv, datetime, json, os, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser()
ap.add_argument("--steps", type=int, default=1200)
ap.add_argument("--tag", required=True)
ap.add_argument("--warmup", type=int, default=10)
ap.add_argument("--human-dir", default="~/effmass_mpc/env_check/step5")
ap.add_argument("--ros-out", required=True)
ap.add_argument("--ready-file", required=True)
args = ap.parse_args()
import step3_ros_common as RC
RC.add_ros_paths()
import numpy as np, torch
import step2_scene as S
import step3_msgs as M
log = lambda *a: print("[step5_5]", *a, flush=True)
OUT = os.path.expanduser("~/effmass_mpc/env_check/step2"); STAMP = datetime.datetime.now().strftime("%Y%m%d_%H%M%S"); PID = os.getpid()
SCENE_INFO = S.set_scene("v2")
CAPTURES = S.install_graph_capture_counter(torch)
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util.logger import setup_curobo_logger
setup_curobo_logger("warn")
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState as RosJointState
import step4_human as H4
log(f"python {sys.version.split()[0]} torch {torch.__version__} rclpy {RC.short(rclpy.__file__)} pid {PID}")
torch.manual_seed(S.SEED); torch.cuda.manual_seed_all(S.SEED); np.random.seed(S.SEED)
ta = TensorDeviceType()
t0 = time.perf_counter()
mpc = S.build_mpc(S.build_robot_cfg(), S.build_world_config()); torch.cuda.synchronize()
log(f"MpcSolver 생성 {time.perf_counter() - t0:.2f}s, scene {json.dumps(SCENE_INFO)}")
checker = mpc.world_collision
ATTEMPTS = S.count_step_once(mpc)
js0 = JointState.from_position(mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0), joint_names=mpc.joint_names)
goal_idx = 0
gb = mpc.setup_solve_single(Goal(current_state=js0, goal_state=js0, goal_pose=Pose.from_list(S.GOALS_WXYZ[goal_idx], ta)), 1)
mpc.update_goal(gb); torch.cuda.synchronize()
# ---- ROS ----
rclpy.init()
node = rclpy.create_node("step5_5_mpc")
pub = node.create_publisher(RosJointState, "/step3/joint_states", QoSProfile(depth=10))
HUMAN = H4.HumanLink(node, os.path.join(os.path.expanduser(args.human_dir), "temp_transform.json"), stage=None, display=(), tensor=True, verify_every=0)
ex = SingleThreadedExecutor(); ex.add_node(node); threading.Thread(target=ex.spin, daemon=True).start()
open(args.ready_file, "w").write(str(time.time_ns()))
w0 = time.time()
while HUMAN.n_dyn == 0 and time.time() - w0 < 300:
    time.sleep(0.02)
FIRST_DYN_NS = time.time_ns()
log(f"첫 HumanDynamicsState ({time.time() - w0:.1f}s 대기), HumanPose {len(HUMAN.pose_rec)}개")
# ---- 예열 ----
S.COUNTER_TAG[0] = "warmup"; WARM = []; WS = time.time_ns()
for w in range(args.warmup):
    torch.cuda.synchronize(); _t = time.perf_counter()
    r = mpc.step(js0.clone(), max_attempts=S.MAX_ATTEMPTS); torch.cuda.synchronize()
    WARM.append(round((time.perf_counter() - _t) * 1000, 3))
WE = time.time_ns()
log(f"예열 {args.warmup}회 ms {WARM}, 그래프 캡처 {len(CAPTURES)}")
S.COUNTER_TAG[0] = "loop"
current = js0.clone()
def pose_error(state):
    kin = mpc.rollout_fn.compute_kinematics(state)
    ee = kin.ee_pos_seq.reshape(-1, 3)[0]; q = kin.ee_quat_seq.reshape(-1, 4)[0]
    gp = gb.goal_pose.position.reshape(-1, 3)[0]; gq = gb.goal_pose.quaternion.reshape(-1, 4)[0]
    return float(torch.linalg.norm(ee - gp)), float(2 * torch.acos(torch.dot(q, gq).abs().clamp(0, 1)))
DT = 1.0 / 60.0; ROWS = []; SENT = []; arrivals = 0; leg_start = 0; infeasible = 0
pc = time.perf_counter
LOOP_START_NS = time.time_ns(); T0 = pc()
for k in range(args.steps):
    target = T0 + k * DT
    now = pc()
    if target > now:
        time.sleep(target - now)
    a = pc()
    mp = S.mover_pose(k * S.MODEL_DT)
    checker.update_obstacle_pose(S.MOVER_NAME, Pose.from_list(mp, ta))
    b = pc()
    HT = HUMAN.pump()
    c = pc()
    m = M.fill_js(RosJointState(), k, kind="js", positions=current.position.reshape(-1).tolist(), velocities=current.velocity.reshape(-1).tolist())
    m.name = list(mpc.joint_names); ts = M.stamp_now(m.header); pub.publish(m); SENT.append([k, ts] + list(m.position))
    d = pc()
    torch.cuda.synchronize()
    e = pc()
    a0 = ATTEMPTS[0]
    res = mpc.step(current, max_attempts=S.MAX_ATTEMPTS)
    torch.cuda.synchronize()
    f = pc()
    feas = bool(res.metrics.feasible.all()); infeasible += int(not feas)
    current = res.js_action.get_ordered_joint_state(mpc.joint_names).clone()
    g = pc()
    pe, qe = pose_error(current)
    if (pe <= S.POS_TOL_M and qe <= S.ROT_TOL_RAD) or (k - leg_start) >= S.LEG_MAX_STEPS:
        arrivals += int(pe <= S.POS_TOL_M and qe <= S.ROT_TOL_RAD)
        goal_idx = 1 - goal_idx; gb.goal_pose.copy_(Pose.from_list(S.GOALS_WXYZ[goal_idx], ta)); mpc.update_goal(gb); leg_start = k
    h = pc()
    ROWS.append([k, round((a - target) * 1000, 4), round((h - a) * 1000, 4), round((h - target) * 1000, 4), int((h - target) > DT),
                 round((b - a) * 1000, 4), round((c - b) * 1000, 4), round(HT["pickup"], 4), round(HT["transform"], 4), round(HT["tensor"], 4), HT["new"],
                 round((d - c) * 1000, 4), round((e - d) * 1000, 4), round((f - e) * 1000, 4), round((g - f) * 1000, 4), round((h - g) * 1000, 4),
                 ATTEMPTS[0] - a0, int(feas)])
LOOP_END_NS = time.time_ns(); wall = pc() - T0
fin = HUMAN.final_check()
ex.shutdown(); node.destroy_node(); rclpy.shutdown()
HUMAN.save(args.ros_out)
FIELDS = ["step", "sched_lag_ms", "busy_ms", "late_ms", "deadline_miss", "t_obst_ms", "t_human_ms", "t_h_pickup_ms", "t_h_transform_ms",
          "t_h_tensor_ms", "h_new", "t_js_ms", "t_presync_ms", "t_solver_ms", "t_cmd_ms", "t_goal_ms", "attempts", "feasible"]
sp = os.path.join(OUT, f"{args.tag}_steps_{STAMP}.csv")
with open(sp, "w", newline="") as fo:
    wr = csv.writer(fo); wr.writerow(FIELDS); wr.writerows(ROWS)
with open(args.ros_out + "_isaac_sent_js.csv", "w", newline="") as fo:
    wr = csv.writer(fo); wr.writerow(["seq", "t_send_ns"] + [f"p{i}" for i in range(len(SENT[0]) - 2)]); wr.writerows(SENT)
sol = np.array([r[13] for r in ROWS]); miss = int(sum(r[4] for r in ROWS)); busy = np.array([r[2] for r in ROWS])
out = dict(tag=args.tag, steps=args.steps, wall_s=wall, achieved_hz=args.steps / wall, warmup_ms=WARM,
           solver_ms_mean=float(sol.mean()), solver_ms_p95=float(np.percentile(sol, 95)), solver_ms_max=float(sol.max()),
           deadline_miss=miss, deadline_miss_pct=100 * miss / args.steps, busy_ms_mean=float(busy.mean()), busy_ms_p95=float(np.percentile(busy, 95)),
           late_ms_max=float(max(r[3] for r in ROWS)), infeasible=infeasible, arrivals=arrivals,
           graph_captures_by_stage={t: sum(1 for c in CAPTURES if c["tag"] == t) for t in sorted({c["tag"] for c in CAPTURES})},
           proc_vram_mib=S.proc_vram_mib(PID), human_final=fin, steps_csv=sp)
json.dump(out, open(os.path.join(OUT, f"{args.tag}_result_{STAMP}.json"), "w"), indent=1, default=float)
json.dump(dict(loop_start_ns=LOOP_START_NS, loop_end_ns=LOOP_END_NS, warmup_start_ns=WS, warmup_end_ns=WE, human_first_dyn_ns=FIRST_DYN_NS,
               human_final=fin), open(args.ros_out + "_isaac_meta.json", "w"))
for kk in ["achieved_hz", "solver_ms_mean", "solver_ms_p95", "solver_ms_max", "deadline_miss", "deadline_miss_pct", "busy_ms_mean", "busy_ms_p95",
           "late_ms_max", "infeasible", "arrivals", "graph_captures_by_stage", "proc_vram_mib"]:
    log(f"{kk:24s}: {out[kk]}")

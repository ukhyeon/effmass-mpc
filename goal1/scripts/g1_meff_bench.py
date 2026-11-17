"""g1_meff_bench.py — 1-2d Isaac 없는 MPC 시험 (Isaac Python, SimulationApp 없음; ROS_METHOD=min run_isaac_ros.sh 로 실행)

  장면 g1, fr3v2_1, cuRobo MPC 설정은 측정과 같음 (step2_scene.build_mpc). 상태 입력 = 직전 명령 (step5_5 와 같은 닫힌 루프, 결정적)
  사람: g1_human_seq.npz (사본 노드 핵심으로 계산한 재생 1,262 프레임) 를 2 스텝마다 한 프레임씩 (30 Hz 상당) → to_isaac → MeffInputs.set_human
  로봇 Λ_r⁻¹: 매 스텝 현재 관절 상태로 effmass_robot.RobotInertia (Pinocchio, 회전자 켬) → MeffInputs.set_robot
  --channel off : 채널 없음 (사람 처리 없음 기준), on : MeffChannel (가중치 0)
  기록: 스텝별 solver 시간·명령 관절값 (궤적 비교용), 그래프 캡처 수, 텐서 주소, --dump-steps 에서 디버그 텐서 일부 (CPU 기준 비교용)
"""
import argparse, json, os, sys, time
HOME = os.path.expanduser("~")
sys.path.insert(0, f"{HOME}/effmass_mpc/env_check/scripts"); sys.path.insert(0, f"{HOME}/effmass_mpc/goal1/scripts")
ap = argparse.ArgumentParser()
ap.add_argument("--channel", default="on", choices=["off", "on"])
ap.add_argument("--impl", default="warp")
ap.add_argument("--dir", default="com", choices=["com", "closest"])
ap.add_argument("--motion", default="static", choices=["static", "cv"])
ap.add_argument("--hfront", type=int, default=0)
ap.add_argument("--steps", type=int, default=600)
ap.add_argument("--warmup", type=int, default=10)
ap.add_argument("--dump-steps", default="")
ap.add_argument("--dump-pairs", type=int, default=300, help="덤프마다 저장할 (후보, 시점) 쌍 수 (구는 전부)")
ap.add_argument("--age", type=float, default=0.0, help="cv 시험용 사람 데이터 나이 (s)")
ap.add_argument("--record-input", type=int, default=1, help="1 = 디버그에 rollout q·구 위치도 복사 (CPU 비교용). 시간 측정은 0 과 비교")
ap.add_argument("--paired", type=int, default=0, help="1 = 채널 없는 solver 와 채널 있는 solver 를 한 프로세스에 두고 매 스텝 같은 상태로 번갈아 실행 (GPU 클럭 차를 같이 받게)")
ap.add_argument("--cost-mode", default="off", choices=["off", "uniform", "mass", "iso"])
ap.add_argument("--agg", default="max_all", choices=["max_all", "nearest"])
ap.add_argument("--w", type=float, default=100000.0)
ap.add_argument("--eta0", type=float, default=0.10)
ap.add_argument("--muref", type=float, default=3.0)
ap.add_argument("--p", type=float, default=0.5)
ap.add_argument("--eta-min", type=float, default=0.05)
ap.add_argument("--eta-max", type=float, default=0.20)
ap.add_argument("--switch-step", type=int, default=-1, help="이 스텝에서 η0 를 --switch-eta0 로 바꿈 (재캡처 0 확인용)")
ap.add_argument("--switch-eta0", type=float, default=0.15)
ap.add_argument("--tag", required=True)
a = ap.parse_args()
import step3_ros_common as RC
ADDED = RC.add_ros_paths()
EFF = f"{HOME}/effmass_mpc/ws/install/effmass_robot/lib/python3.10/site-packages"
sys.path.append(EFF)
import numpy as np, torch
os.environ["STEP_ROBOT"] = "fr3v2_1"
import step2_scene as S
SCENE = S.set_scene("g1")
CAPTURES = S.install_graph_capture_counter(torch)
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util.logger import setup_curobo_logger
setup_curobo_logger("warn")
from effmass_robot.robot_inertia import RobotInertia
import g1_meff_gpu as G
log = lambda *x: print("[meff-bench]", *x, flush=True)
OUT = f"{HOME}/effmass_mpc/goal1/logs/meff_bench"; os.makedirs(OUT, exist_ok=True)
torch.manual_seed(S.SEED); torch.cuda.manual_seed_all(S.SEED); np.random.seed(S.SEED)
ta = TensorDeviceType()
robot_cfg = S.build_robot_cfg()
mpc = S.build_mpc(robot_cfg, S.build_world_config()); torch.cuda.synchronize()
info = S.describe_mpc(mpc)
ATTEMPTS = S.count_step_once(mpc)
ls = mpc.kinematics.kinematics_config.link_spheres.cpu().numpy()
valid = ls[:, 3] > 0; NS_ALL = len(ls); SV = int(valid.sum())
assert valid[:SV].all() and not valid[SV:].any(), "유효 구가 앞쪽에 연속하지 않음"
ri = RobotInertia(rotor=True)
assert len(ri.sphere_link) == SV
rng = np.random.default_rng(5); lo = np.array([-2.7, -1.7, -2.8, -3.0, -2.7, 0.6, -3.0]); hi = -lo; hi[3] = -0.2; hi[5] = 4.4
nz = np.zeros(SV)
for _ in range(20):
    nz = np.maximum(nz, np.abs(ri.compute(lo + (hi - lo) * rng.uniform(size=7))[1]).reshape(SV, -1).max(1))
mass_on = (nz > 0).astype(np.int32)
inp = G.MeffInputs(SV, info["traj_dt"], device=str(ta.device))
mpc2 = None
if a.paired:
    assert a.channel == "on"
    mpc2 = mpc                                   # 채널 있는 쪽
    torch.manual_seed(S.SEED); torch.cuda.manual_seed_all(S.SEED)
    mpc = S.build_mpc(robot_cfg, S.build_world_config()); torch.cuda.synchronize()   # 채널 없는 쪽 (루프를 끌고 감)
    ATTEMPTS = S.count_step_once(mpc)
ch = G.MeffChannel(mpc2 if a.paired else mpc, inp, impl=a.impl, direction=a.dir, motion=a.motion, hfront=a.hfront, mass_on=mass_on, record_q=bool(a.record_input)) if a.channel == "on" else None
if ch is not None:
    ch.set_cost(mode=a.cost_mode, agg=a.agg, w=a.w, eta0=a.eta0, muref=a.muref, p=a.p, eta_min=a.eta_min, eta_max=a.eta_max)
log(f"channel {a.channel} impl {a.impl} dir {a.dir} motion {a.motion} hfront {a.hfront}; spheres {SV}/{NS_ALL}, mass_on 0 인 구 {np.nonzero(mass_on == 0)[0].tolist()} "
    f"({[ri.sphere_link[i] for i in np.nonzero(mass_on == 0)[0]]}); traj_dt {info['traj_dt'][:3]}... horizon {info['horizon']}")
seq = np.load(f"{HOME}/effmass_mpc/goal1/logs/g1_human_seq.npz")
tf = json.load(open(f"{HOME}/effmass_mpc/env_check/step4/temp_transform.json")); R_, t_ = tf["R"], tf["t"]
def frame(f):
    return dict(com=seq["com"][f], vel=seq["vel"][f], inv=seq["inv"][f], p0=seq["p0"][f], p1=seq["p1"][f], rad=seq["rad"], seg=seq["seg"])
NF = len(seq["com"])
js0 = JointState.from_position(mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0), joint_names=mpc.joint_names)
goal_idx = 0
gb = mpc.setup_solve_single(Goal(current_state=js0, goal_state=js0, goal_pose=Pose.from_list(S.GOALS_WXYZ[goal_idx], ta)), 1)
mpc.update_goal(gb); torch.cuda.synchronize()
if mpc2 is not None:
    gb2 = mpc2.setup_solve_single(Goal(current_state=js0, goal_state=js0, goal_pose=Pose.from_list(S.GOALS_WXYZ[goal_idx], ta)), 1)
    mpc2.update_goal(gb2); torch.cuda.synchronize()
def feed(step, q_np):
    f = (step // 2) % NF
    if step % 2 == 0 or step < 0:
        inp.set_human(G.to_isaac(frame(f), R_, t_))
    inp.set_age(a.age)
    inp.set_robot(ri.compute(q_np)[1])
    return f
# ---- 예열 (그래프 캡처) ----
S.COUNTER_TAG[0] = "warmup"; WARM = []
q0 = js0.position.reshape(-1).cpu().numpy().astype(float)
for w in range(a.warmup):
    feed(0, q0); torch.cuda.synchronize(); t0 = time.perf_counter()
    mpc.step(js0.clone(), max_attempts=S.MAX_ATTEMPTS); torch.cuda.synchronize(); WARM.append(round((time.perf_counter() - t0) * 1000, 3))
    if mpc2 is not None:
        mpc2.step(js0.clone(), max_attempts=S.MAX_ATTEMPTS); torch.cuda.synchronize()
log(f"예열 ms {WARM}, 캡처 {len(CAPTURES)}")
S.COUNTER_TAG[0] = "loop"
current = js0.clone()
def pose_error(state):
    kin = mpc.rollout_fn.compute_kinematics(state)
    ee = kin.ee_pos_seq.reshape(-1, 3)[0]; q = kin.ee_quat_seq.reshape(-1, 4)[0]
    return float(torch.linalg.norm(ee - gb.goal_pose.position.reshape(-1, 3)[0])), float(2 * torch.acos(torch.dot(q, gb.goal_pose.quaternion.reshape(-1, 4)[0]).abs().clamp(0, 1)))
DUMP = {int(x) for x in a.dump_steps.split(",") if x}
sol, feed_ms, qcmd, feas_l, att_l, sol2, same2 = [], [], [], [], [], [], []
arrivals = 0; leg_start = 0; infeasible = 0; dumps = []
drng = np.random.default_rng(11)
for k in range(a.steps):
    if ch is not None and k == a.switch_step:
        ch.set_cost(eta0=a.switch_eta0); log(f"step {k}: η0 → {a.switch_eta0}, 캡처 {len(CAPTURES)}")
    t0 = time.perf_counter()
    qn = current.position.reshape(-1).cpu().numpy().astype(float)
    f = feed(k, qn)
    torch.cuda.synchronize(); t1 = time.perf_counter()
    a0 = ATTEMPTS[0]
    if mpc2 is not None and k % 2 == 1:          # 홀수 스텝: 채널 쪽 먼저
        r2 = mpc2.step(current, max_attempts=S.MAX_ATTEMPTS); torch.cuda.synchronize(); t1b = time.perf_counter(); sol2.append((t1b - t1) * 1000); t1 = t1b
    res = mpc.step(current, max_attempts=S.MAX_ATTEMPTS)
    torch.cuda.synchronize(); t2 = time.perf_counter()
    if mpc2 is not None and k % 2 == 0:
        r2 = mpc2.step(current, max_attempts=S.MAX_ATTEMPTS); torch.cuda.synchronize(); sol2.append((time.perf_counter() - t2) * 1000)
    if mpc2 is not None:
        same2.append(bool(torch.equal(r2.js_action.position, res.js_action.position)))
    sol.append((t2 - t1) * 1000); feed_ms.append((t1 - t0) * 1000); att_l.append(ATTEMPTS[0] - a0)
    feas = bool(res.metrics.feasible.all()); infeasible += int(not feas); feas_l.append(int(feas))
    current = res.js_action.get_ordered_joint_state(mpc.joint_names).clone()
    qcmd.append(current.position.reshape(-1).cpu().numpy().copy())
    if ch is not None and k in DUMP:
        sn = ch.snapshot(0); B, H = sn["dist"].shape[:2]; Hf = ch.optimizers[0][2]["Hf"]
        bb = drng.integers(0, B, a.dump_pairs); hh = drng.integers(0, Hf, a.dump_pairs)
        np.savez_compressed(f"{OUT}/{a.tag}_dump_{k:04d}.npz", b=bb, h=hh, frame=f, tau=inp.tau.cpu().numpy(), traj_dt=np.asarray(info["traj_dt"]), q_state=qn, inv_r=ri.compute(qn)[1], age=a.age, mass_on=mass_on,
                            calls=sn["calls"], cost_par=ch.par.cpu().numpy(), cost_mode=int(ch.mode[0]), cost_agg=int(ch.agg[0]),
                            **{kk: sn[kk][bb, hh] for kk in ["dist", "shape", "seg", "u", "mh", "mr", "mu", "q", "sph", "cost", "eta", "kc", "cost_bh"] if kk in sn})
        dumps.append(k)
    pe, qe = pose_error(current)
    if (pe <= S.POS_TOL_M and qe <= S.ROT_TOL_RAD) or (k - leg_start) >= S.LEG_MAX_STEPS:
        arrivals += int(pe <= S.POS_TOL_M and qe <= S.ROT_TOL_RAD)
        goal_idx = 1 - goal_idx; gb.goal_pose.copy_(Pose.from_list(S.GOALS_WXYZ[goal_idx], ta)); mpc.update_goal(gb); leg_start = k
sol = np.array(sol)
paired = None
if mpc2 is not None:
    s2 = np.array(sol2); dlt = s2 - sol
    paired = dict(off_mean=float(sol.mean()), on_mean=float(s2.mean()), off_median=float(np.median(sol)), on_median=float(np.median(s2)), off_p95=float(np.percentile(sol, 95)),
                  on_p95=float(np.percentile(s2, 95)), delta_mean=float(dlt.mean()), delta_median=float(np.median(dlt)), delta_p95=float(np.percentile(dlt, 95)),
                  action_equal_all=bool(all(same2)), action_equal_n=int(sum(same2)))
res = dict(cost=ch.cost_state() if ch else None, paired=paired, record_input=a.record_input, tag=a.tag, channel=a.channel, impl=a.impl, dir=a.dir, motion=a.motion, hfront=a.hfront, steps=a.steps, warmup_ms=WARM,
           solver_ms_mean=float(sol.mean()), solver_ms_p95=float(np.percentile(sol, 95)), solver_ms_median=float(np.median(sol)), solver_ms_max=float(sol.max()),
           feed_ms_mean=float(np.mean(feed_ms)), attempts_mean=float(np.mean(att_l)), infeasible=infeasible, arrivals=arrivals,
           graph_captures_by_stage={t: sum(1 for c in CAPTURES if c["tag"] == t) for t in sorted({c["tag"] for c in CAPTURES})},
           inputs_pointers_unchanged=inp.pointers() == inp.addr0, channel_pointers_unchanged=(ch.pointers() == ch.addr0) if ch else None,
           calls=int(ch.snapshot(0)["calls"]) if ch else None, dumps=dumps, mass_on_zero=np.nonzero(mass_on == 0)[0].tolist(),
           torch_max_allocated_mib=round(torch.cuda.max_memory_allocated() / 2**20, 1), proc_vram_mib=S.proc_vram_mib(os.getpid()), ros_paths_added=ADDED)
np.savez(f"{OUT}/{a.tag}_traj.npz", qcmd=np.array(qcmd), solver_ms=sol, solver2_ms=np.array(sol2), feasible=np.array(feas_l), attempts=np.array(att_l))
json.dump(res, open(f"{OUT}/{a.tag}_result.json", "w"), indent=1)
log(json.dumps({k: v for k, v in res.items() if k not in ("warmup_ms", "ros_paths_added")}, ensure_ascii=False))

"""rp2_mpc_rt.py - real_prep2: cuRobo MPC without Isaac (no SimulationApp), wall-clock 60 Hz, joint state in, JointTrajectory out.
Run with Isaac's python and the ROS paths: env_check/scripts/run_isaac_ros.sh (ROS_DOMAIN_ID 30, ROS_LOCALHOST_ONLY 1); see rp2_run.sh.
  input : sensor_msgs/JointState on --js-topic (default /NS_1/franka/joint_states), joints fr3_joint1..7 (matched by name); the latest
          message is the MPC's current state (position, velocity; acceleration 0)
  output: trajectory_msgs/JointTrajectory on /mpc/joint_trajectory every step, joint names fr3_joint1..7, header.stamp 0, K points of the
          horizon (positions, velocities, accelerations, time_from_start = horizon time; as step2b --jt-out). Infeasible step: one point
          at the measured position, zero velocity / acceleration (hold).
  profile: a step2b profile yml (default goal1/config/real_acc10.yml): robot_cfg (acceleration limit), pose_rot_w, sphere_speed_limit.
          Human-related keys (speed_cap*, meff_*, human_*) need the human cost channel, which is not built here (no human); they are
          listed in the log as not applied. sphere_speed_limit > 0: the published first point is moved toward the measured position so
          that no robot sphere moves faster than the limit in one step (as step2b --sphere-speed-limit; its velocity / acceleration
          are scaled by the same factor; points 2..K are not changed).
  scene : no human. Goals alternate between two hand poses (the end points of a scene json, default the live3m corrected path of
          motion 1, gripper down); a goal is switched on arrival (1 cm, 0.05 rad) or after LEG_MAX_STEPS. world = robot table + floor.
          HUMAN_HOOK below is the slot for a human input later (not used).
  timing: step k is scheduled at T0 + k / 60 s (wall clock, time.perf_counter); the loop sleeps until then, a late step starts at
          once (no step is skipped). overrun = the step's own computation (start to publish) > 1/60 s; deadline miss = publish later
          than schedule + 1/60 s.
options for the joint-state input (default off = as before):
  --js-source shm --shm-name rp2_js : read the latest joint state from shared memory written by rp2_js_relay.py (separate process),
          once at the start of each step; no ROS subscription in this process
  --js-qos best_effort : subscribe with the sensor-data QoS (best effort, depth 5) instead of reliable depth 1
  --age-comp 1 : use q + qd * age, age = now - header stamp of the state used (same clock on one computer)
  every step records the age by header stamp (stamp_age) and by receive time (state_age)
acceleration clip (real_prep3; default off = as before):
  --acc-clip 1 [--acc-max 9.5] [--vel-frac 0.9] : last stage before publishing (after the speed limiter): with the two targets sent
          last (p1, p2), v_prev = (p1 - p2) / dt; the first point p is moved so that per joint the differenced acceleration
          ((p - p1) / dt - v_prev) / dt is within +-acc_max and the velocity (p - p1) / dt within +-vel_frac x URDF velocity
          (dt = 1/60 s). The point's velocity becomes (p' - p1) / dt and its acceleration (v' - v_prev) / dt; points 2..K are not
          changed. Before the first step p1 = p2 = the measured position. --clip-feedback 1 (default with --acc-clip 1): the sent
          first point (position, velocity, acceleration) is written into cuRobo's state filter (its "previous command"), so the
          next start state blends the measurement with what was actually sent, not with the unclipped command.
start after warm-up (real_prep3, default on):
  --warm-reset 1 : after the warm-up solves and just before the first real step, cuRobo's state filter (its "previous command") is
          set to the latest measured joint state with zero velocity and acceleration. Without it the warm-up solves advance the filter
          and the first target is tens of mrad away from a resting robot. 0 = as before.
Outputs (--out <prefix>): <prefix>_steps.npz (per step timing, sent points, state age, feasible, limiter scale), <prefix>_summary.json.
"""
import argparse, json, os, sys, threading, time
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts"))
ap = argparse.ArgumentParser()
ap.add_argument("--profile", default="~/effmass_mpc/goal1/config/real_acc10.yml")
ap.add_argument("--js-topic", default="/NS_1/franka/joint_states")
ap.add_argument("--jt-topic", default="/mpc/joint_trajectory")
ap.add_argument("--k", type=int, default=1, help="horizon points per JointTrajectory")
ap.add_argument("--duration", type=float, default=180.0, help="measured loop length, s (wall clock)")
ap.add_argument("--scene-json", default="~/effmass_mpc/goal1/live/live3m/corrected/cal/m1b/scene_AB.json")
ap.add_argument("--warmup", type=int, default=10)
ap.add_argument("--wait-js", type=float, default=60.0, help="s to wait for the first joint state")
ap.add_argument("--out", required=True)
ap.add_argument("--mppi-seed", type=int, default=0)
ap.add_argument("--js-source", default="topic", choices=["topic", "shm"])
ap.add_argument("--shm-name", default="rp2_js")
ap.add_argument("--age-comp", type=int, default=0)
ap.add_argument("--warm-reset", type=int, default=1, help="(real_prep3) 1: reset the solver state filter to the measured state (velocity 0) after the warm-up; 0 = off")
ap.add_argument("--acc-clip", type=int, default=0, help="(real_prep3) 1: clip the differenced acceleration / velocity of the published first point")
ap.add_argument("--acc-max", type=float, default=9.5, help="(real_prep3) rad/s^2, per joint")
ap.add_argument("--vel-frac", type=float, default=0.9, help="(real_prep3) velocity limit = this x URDF velocity limit")
ap.add_argument("--clip-feedback", type=int, default=1, help="(real_prep3) with --acc-clip 1: write the sent point into the solver state filter")
ap.add_argument("--js-qos", default="reliable", choices=["reliable", "best_effort"], help="QoS of the joint-state subscription (best_effort = rclpy qos_profile_sensor_data)")
args = ap.parse_args()
os.environ["STEP_ROBOT"] = "fr3v2_1"
import step3_ros_common as RC
RC.add_ros_paths()
import numpy as np, torch, yaml
import step2_scene as S
log = lambda *a: print("[rp2]", *a, flush=True)
PF = yaml.safe_load(open(os.path.expanduser(args.profile))); SET = PF.get("settings", {})
if SET.get("robot_cfg"):
    S.ROBOT_CFG_OVERRIDE = os.path.expanduser(SET["robot_cfg"])
S.POSE_ROT_W = SET.get("pose_rot_w"); S.MPPI_SEED = args.mppi_seed
SPEED_LIM = float(SET.get("sphere_speed_limit", 0.0))
NOT_APPLIED = sorted(k for k in SET if k.startswith(("speed_cap", "meff", "human")))
S.G2_SCENE_JSON = os.path.expanduser(args.scene_json); SCENE = S.set_scene("g2")
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util.logger import setup_curobo_logger
setup_curobo_logger("warn")
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState as RosJS
from trajectory_msgs.msg import JointTrajectory as RosJT, JointTrajectoryPoint as RosJTP
from builtin_interfaces.msg import Duration as RosDuration
ta = TensorDeviceType(); t0 = time.perf_counter()
mpc = S.build_mpc(S.build_robot_cfg(), S.build_world_config()); torch.cuda.synchronize()
_sb = mpc.rollout_fn.dynamics_model.get_state_bounds()
JLIM = dict(velocity=[round(float(x), 3) for x in _sb.velocity[1].reshape(-1)], acceleration=[round(float(x), 3) for x in _sb.acceleration[1].reshape(-1)])
log(f"MPC built {time.perf_counter() - t0:.1f}s, profile {args.profile}, robot cfg {S.ROBOT_CFG_OVERRIDE or 'default'}, limits {JLIM}, sphere speed limit {SPEED_LIM}, "
    f"profile keys not applied (no human channel): {NOT_APPLIED}")
NJ = len(mpc.joint_names); NAMES = [f"fr3_joint{k + 1}" for k in range(NJ)]; DTF = float(mpc.solver.safety_rollout.dynamics_model.state_filter.dt)
TRAJ_DT = mpc.rollout_fn.dynamics_model.traj_dt.reshape(-1); HT = torch.cumsum(TRAJ_DT, 0); HT_NP = HT.cpu().numpy()
q_start = torch.tensor(SCENE["q_start"], **ta.as_torch_dict()).reshape(1, -1)
js0 = JointState.from_position(q_start.clone(), joint_names=mpc.joint_names)
goal_idx = 1
gb = mpc.setup_solve_single(Goal(current_state=js0, goal_state=js0, goal_pose=Pose.from_list(S.GOALS_WXYZ[goal_idx], ta)), 1)
mpc.update_goal(gb); torch.cuda.synchronize()
# ---- ROS ----
rclpy.init(); node = rclpy.create_node("rp2_mpc_rt"); LAST = [None, 0, 0.0, 0.0]   # (msg arrays, receive perf time, header stamp s, receive wall s)
NRECV = [0]


def on_js(m):
    idx = {n: i for i, n in enumerate(m.name)}
    try:
        q = np.array([m.position[idx[n]] for n in NAMES]); v = np.array([m.velocity[idx[n]] for n in NAMES]) if len(m.velocity) == len(m.name) else np.zeros(NJ)
    except KeyError:
        return
    LAST[0] = (q, v); LAST[1] = time.perf_counter(); LAST[2] = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec; LAST[3] = time.time(); NRECV[0] += 1


SHM = None; SEQ = [-1.0]
if args.js_source == "shm":   # (test a) the joint state comes from rp2_js_relay.py through shared memory
    from multiprocessing import shared_memory, resource_tracker
    w0 = time.time()
    while SHM is None and time.time() - w0 < args.wait_js:
        try:
            SHM = shared_memory.SharedMemory(name=args.shm_name)
        except FileNotFoundError:
            time.sleep(0.05)
    if SHM is None:
        log(f"no shared memory {args.shm_name}"); sys.exit(3)
    resource_tracker.unregister(SHM._name, "shared_memory")   # the relay owns and removes the block
    SB = np.ndarray((17,), np.float64, buffer=SHM.buf)


def read_shm():
    for _ in range(100):
        r = SB.copy()
        if r[0] % 2 == 0 and r[0] == SB[0]:
            break
    if r[0] <= 0:
        return
    if r[0] != SEQ[0]:
        NRECV[0] += int((r[0] - max(SEQ[0], 0)) / 2); SEQ[0] = r[0]
    LAST[0] = (r[3:10].copy(), r[10:17].copy()); LAST[1] = time.perf_counter() - (time.time() - r[2]); LAST[2] = r[1]; LAST[3] = r[2]


if args.js_source == "topic":
    node.create_subscription(RosJS, args.js_topic, on_js, qos_profile_sensor_data if args.js_qos == "best_effort" else QoSProfile(depth=1))
pub = node.create_publisher(RosJT, args.jt_topic, QoSProfile(depth=10))
ex = SingleThreadedExecutor(); ex.add_node(node); threading.Thread(target=ex.spin, daemon=True).start()
w0 = time.time()
while LAST[0] is None and time.time() - w0 < args.wait_js:
    time.sleep(0.01)
    if args.js_source == "shm":
        read_shm()
if LAST[0] is None:
    log(f"no joint state on {args.js_topic} within {args.wait_js} s"); sys.exit(3)
log(f"first joint state after {time.time() - w0:.2f}s on {args.js_topic}")


def HUMAN_HOOK(step):   # slot for the human input (sensing -> human dynamics -> MPC cost); not used in this test
    return None


def state_from_last(comp=False):
    q, v = LAST[0]
    if comp:   # age compensation: move the measured angles forward by velocity x age (header stamp)
        q = q + v * max(time.time() - LAST[2], 0.0)
    return JointState(position=torch.tensor(q, **ta.as_torch_dict()).reshape(1, -1), velocity=torch.tensor(v, **ta.as_torch_dict()).reshape(1, -1),
                      acceleration=torch.zeros(1, NJ, **ta.as_torch_dict()), jerk=torch.zeros(1, NJ, **ta.as_torch_dict()), joint_names=list(mpc.joint_names))


# ---- warm-up (graph capture) ----
WARM = []
for _ in range(args.warmup):
    torch.cuda.synchronize(); _t = time.perf_counter(); mpc.step(state_from_last(), max_attempts=S.MAX_ATTEMPTS); torch.cuda.synchronize(); WARM.append(round((time.perf_counter() - _t) * 1000, 2))
log(f"warm-up ms {WARM}")
DT = 1.0 / 60.0; N = int(round(args.duration * 60)); pc = time.perf_counter
VLIM = np.array(JLIM["velocity"][:NJ]) * args.vel_frac; SFILT = mpc.solver.safety_rollout.dynamics_model.state_filter
CLIP = dict(p1=None, p2=None)   # (real_prep3) the two targets sent last
if args.acc_clip:
    CLIP["p1"] = LAST[0][0].copy(); CLIP["p2"] = LAST[0][0].copy()
    log(f"acceleration clip on: acc_max {args.acc_max} rad/s^2, velocity limits {np.round(VLIM, 3).tolist()} rad/s, feedback to the state filter {args.clip_feedback}")


def acc_clip(q, qd, qdd):
    """returns the clipped first point (q, qd, qdd), the per-joint change of q and whether anything was changed"""
    p1, p2 = CLIP["p1"], CLIP["p2"]; vp = (p1 - p2) / DT; v = (q - p1) / DT
    lo = np.maximum(vp - args.acc_max * DT, -VLIM); hi = np.minimum(vp + args.acc_max * DT, VLIM)
    lo = np.minimum(lo, hi)   # if the previous velocity was above the velocity limit, decelerate as fast as allowed
    v2 = np.clip(v, lo, hi); q2 = p1 + v2 * DT; ch = bool(np.any(np.abs(v2 - v) > 1e-12))
    return (q2, v2, (v2 - vp) / DT, q2 - q, ch) if ch else (q, qd, qdd, np.zeros_like(q), False)
R = {k: np.zeros(N) for k in ("t_sched", "t_start", "t_state", "t_solve0", "t_solve1", "t_pub", "t_end", "state_age", "stamp_age", "lim_scale", "sphere_v_before")}
R["clip_dq"] = np.zeros((N, NJ), np.float32); R["clipped"] = np.zeros(N, np.int8); R["acc_pre"] = np.zeros((N, NJ), np.float32)
R["feasible"] = np.zeros(N, np.int8); R["sent_q"] = np.zeros((N, NJ), np.float32); R["sent_qd"] = np.zeros((N, NJ), np.float32); R["sent_qdd"] = np.zeros((N, NJ), np.float32)
R["meas_q"] = np.zeros((N, NJ), np.float32); R["t_pub_ns"] = np.zeros(N, np.int64); R["goal_idx"] = np.zeros(N, np.int8)
if args.warm_reset:   # (real_prep3) start from the measured state: the warm-up solves have moved the filter's previous command
    if args.js_source == "shm":
        read_shm()
    _q0 = LAST[0][0].copy(); _cs = SFILT.cmd_joint_state
    if _cs is not None:
        _cs.position.copy_(torch.as_tensor(_q0, dtype=_cs.position.dtype, device=_cs.position.device).reshape(_cs.position.shape))
        for _t in (_cs.velocity, _cs.acceleration) + ((_cs.jerk,) if _cs.jerk is not None else ()):
            _t.zero_()
    if args.acc_clip:
        CLIP["p1"] = _q0.copy(); CLIP["p2"] = _q0.copy()
    log(f"warm reset: solver state filter set to the measured joints (velocity 0) {np.round(_q0, 4).tolist()}")
arrivals = 0; leg_start = 0; T0 = pc() + 0.05
for k in range(N):
    sched = T0 + k * DT; now = pc()
    if sched > now:
        time.sleep(sched - now)
    a = pc(); HUMAN_HOOK(k)
    if args.js_source == "shm":
        read_shm()
    sage = time.time() - LAST[2]; cur = state_from_last(bool(args.age_comp)); age = a - LAST[1]; b = pc()
    torch.cuda.synchronize(); s0 = pc()
    res = mpc.step(cur, max_attempts=S.MAX_ATTEMPTS); torch.cuda.synchronize(); s1 = pc()
    feas = bool(res.metrics.feasible.all()); cmd = res.js_action.get_ordered_joint_state(mpc.joint_names)
    jm = RosJT(); jm.joint_names = NAMES; sc = 1.0; vmx = float("nan")
    if feas and torch.isfinite(cmd.position).all() and mpc.solver._init_act_seq is not None:
        a0 = cmd.acceleration.reshape(1, -1); ac = torch.cat([a0, mpc.solver._init_act_seq.reshape(-1, NJ)[:-1]], 0)
        v0 = cmd.velocity.reshape(1, -1) - a0 * DTF; p0 = cmd.position.reshape(1, -1) - cmd.velocity.reshape(1, -1) * DTF   # filtered start state (see step2b)
        hqd = v0 + torch.cumsum(ac * TRAJ_DT[:, None], 0); hq = p0 + torch.cumsum(hqd * TRAJ_DT[:, None], 0)
        HQ, HQD, HQDD = hq.cpu().numpy(), hqd.cpu().numpy(), ac.cpu().numpy()
        if SPEED_LIM > 0:   # hard limit on the first point, measured -> commanded, all robot spheres
            k0 = mpc.rollout_fn.compute_kinematics(cur).robot_spheres.reshape(-1, 4)[:, :3].clone(); j1 = cur.clone(); j1.position = hq[0:1].clone()
            k1 = mpc.rollout_fn.compute_kinematics(j1).robot_spheres.reshape(-1, 4)[:, :3]; vmx = float((k1 - k0).norm(dim=-1).max()) / DT
            if vmx > SPEED_LIM:
                sc = SPEED_LIM / vmx; qm = cur.position.reshape(-1).cpu().numpy(); HQ[0] = qm + sc * (HQ[0] - qm); HQD[0] *= sc; HQDD[0] *= sc
        for i in range(min(args.k, len(HT_NP))):
            p = RosJTP(); p.positions = [float(x) for x in HQ[i]]; p.velocities = [float(x) for x in HQD[i]]; p.accelerations = [float(x) for x in HQDD[i]]
            ns = int(round(float(HT_NP[i]) * 1e9)); p.time_from_start = RosDuration(sec=ns // 1000000000, nanosec=ns % 1000000000); jm.points.append(p)
        sq, sqd, sqdd = HQ[0], HQD[0], HQDD[0]
    else:
        qm = cur.position.reshape(-1).cpu().numpy(); p = RosJTP(); p.positions = [float(x) for x in qm]; p.velocities = [0.0] * NJ; p.accelerations = [0.0] * NJ
        p.time_from_start = RosDuration(sec=0, nanosec=int(DT * 1e9)); jm.points.append(p); sq, sqd, sqdd = qm, np.zeros(NJ), np.zeros(NJ)
    if args.acc_clip:   # (real_prep3) last stage before publishing
        sq = np.asarray(sq, float); sqd = np.asarray(sqd, float); sqdd = np.asarray(sqdd, float)
        R["acc_pre"][k] = ((sq - CLIP["p1"]) / DT - (CLIP["p1"] - CLIP["p2"]) / DT) / DT
        sq, sqd, sqdd, dq, chg = acc_clip(sq, sqd, sqdd)
        if chg:
            p0 = jm.points[0]; p0.positions = [float(x) for x in sq]; p0.velocities = [float(x) for x in sqd]; p0.accelerations = [float(x) for x in sqdd]
            R["clip_dq"][k] = dq; R["clipped"][k] = 1
        if args.clip_feedback and SFILT.cmd_joint_state is not None:   # the solver's "previous command" = what was sent
            _cs = SFILT.cmd_joint_state
            _cs.position.copy_(torch.as_tensor(sq, dtype=_cs.position.dtype, device=_cs.position.device).reshape(_cs.position.shape))
            _cs.velocity.copy_(torch.as_tensor(sqd, dtype=_cs.velocity.dtype, device=_cs.velocity.device).reshape(_cs.velocity.shape))
            _cs.acceleration.copy_(torch.as_tensor(sqdd, dtype=_cs.acceleration.dtype, device=_cs.acceleration.device).reshape(_cs.acceleration.shape))
        CLIP["p2"] = CLIP["p1"]; CLIP["p1"] = np.asarray(sq, float).copy()
    pub.publish(jm); tp = pc(); tpn = time.time_ns()
    # goal switching on the measured state
    kin = mpc.rollout_fn.compute_kinematics(cur); ee = kin.ee_pos_seq.reshape(-1, 3)[0]; qq = kin.ee_quat_seq.reshape(-1, 4)[0]
    pe = float(torch.linalg.norm(ee - gb.goal_pose.position.reshape(-1, 3)[0])); qe = float(2 * torch.acos(torch.dot(qq, gb.goal_pose.quaternion.reshape(-1, 4)[0]).abs().clamp(0, 1)))
    if (pe <= S.POS_TOL_M and qe <= S.ROT_TOL_RAD) or (k - leg_start) >= S.LEG_MAX_STEPS:
        arrivals += int(pe <= S.POS_TOL_M and qe <= S.ROT_TOL_RAD); goal_idx = 1 - goal_idx; gb.goal_pose.copy_(Pose.from_list(S.GOALS_WXYZ[goal_idx], ta)); mpc.update_goal(gb); leg_start = k
    e = pc()
    for key, val in (("t_sched", sched), ("t_start", a), ("t_state", b), ("t_solve0", s0), ("t_solve1", s1), ("t_pub", tp), ("t_end", e), ("state_age", age), ("stamp_age", sage), ("lim_scale", sc), ("sphere_v_before", vmx)):
        R[key][k] = val
    R["feasible"][k] = int(feas); R["sent_q"][k] = sq; R["sent_qd"][k] = sqd; R["sent_qdd"][k] = sqdd; R["meas_q"][k] = cur.position.reshape(-1).cpu().numpy(); R["t_pub_ns"][k] = tpn; R["goal_idx"][k] = goal_idx
ex.shutdown(); node.destroy_node(); rclpy.shutdown()
if SHM is not None:
    del SB; SHM.close()
for kk in [x for x in R if x.startswith("t_") and x != "t_pub_ns"]:
    R[kk] = R[kk] - T0
np.savez(args.out + "_steps.npz", **R, horizon_t=HT_NP, joint_names=np.array(NAMES), k=args.k, profile=args.profile)
solve = (R["t_solve1"] - R["t_solve0"]) * 1000; comp = (R["t_pub"] - R["t_start"]) * 1000; late = (R["t_pub"] - R["t_sched"]) * 1000; per = np.diff(R["t_pub_ns"]) / 1e6
qd2 = np.abs(np.diff(R["sent_q"].astype(float), 2, axis=0)) * 3600.0
S_ = dict(profile=args.profile, js_topic=args.js_topic, jt_topic=args.jt_topic, k=args.k, steps=N, duration_s=args.duration, wall_s=float((R["t_pub_ns"][-1] - R["t_pub_ns"][0]) / 1e9),
          msgs_per_s=float((N - 1) / ((R["t_pub_ns"][-1] - R["t_pub_ns"][0]) / 1e9)), period_ms=dict(mean=float(per.mean()), p99=float(np.percentile(per, 99)), max=float(per.max()), min=float(per.min())),
          solve_ms=dict(mean=float(solve.mean()), p95=float(np.percentile(solve, 95)), p99=float(np.percentile(solve, 99)), max=float(solve.max())),
          step_compute_ms=dict(mean=float(comp.mean()), p95=float(np.percentile(comp, 95)), p99=float(np.percentile(comp, 99)), max=float(comp.max())),
          overrun_steps=int((comp > DT * 1000).sum()), overrun_pct=float(100 * (comp > DT * 1000).mean()), deadline_miss=int((late > DT * 1000).sum()), late_ms_max=float(late.max()),
          sleep_lag_ms=dict(mean=float(((R["t_start"] - R["t_sched"]) * 1000).mean()), p99=float(np.percentile((R["t_start"] - R["t_sched"]) * 1000, 99)), max=float(((R["t_start"] - R["t_sched"]) * 1000).max())),
          state_age_ms=dict(mean=float(R["state_age"].mean() * 1000), p99=float(np.percentile(R["state_age"], 99) * 1000), max=float(R["state_age"].max() * 1000)),
          stamp_age_ms=dict(mean=float(R["stamp_age"].mean() * 1000), std=float(R["stamp_age"].std() * 1000), p99=float(np.percentile(R["stamp_age"], 99) * 1000), max=float(R["stamp_age"].max() * 1000)),
          js_source=args.js_source, age_comp=args.age_comp, js_qos=args.js_qos, js_received=NRECV[0], infeasible=int((R["feasible"] == 0).sum()), arrivals=arrivals,
          diffacc_over10_steps=int((qd2.max(1) > 10).sum()), diffacc_excess_max=float(max(qd2.max() - 10, 0)), diffacc_max_per_joint=qd2.max(0).tolist(),
          limiter_scaled_steps=int((R["lim_scale"] < 1).sum()), acc_clip=args.acc_clip, warm_reset=args.warm_reset, acc_max=args.acc_max, vel_frac=args.vel_frac, clip_feedback=args.clip_feedback,
          clipped_steps=int(R["clipped"].sum()), clip_dq_max_per_joint=np.abs(R["clip_dq"]).max(0).tolist(), warmup_ms=WARM, joint_limits=JLIM, sphere_speed_limit=SPEED_LIM, profile_keys_not_applied=NOT_APPLIED)
json.dump(S_, open(args.out + "_summary.json", "w"), indent=1)
log(json.dumps({k: v for k, v in S_.items() if k not in ("warmup_ms", "joint_limits")}))

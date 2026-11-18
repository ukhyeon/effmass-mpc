"""step2b_mpc_isaac.py — 측정 B: Isaac Sim 안에서 Franka + cuRobo MPC (run_isaac.sh 로 실행)

  사용법: run_isaac.sh step2b_mpc_isaac.py [--gui] [--steps 1200] [--tag NAME] [--hold SEC]
    --hold SEC : GUI 모드에서 측정 후 MPC 를 계속 돌리며 창 유지 (기본 0 = 측정 끝나면 종료, -1 = 창 닫을 때까지)

  참고: curobo/examples/isaac_sim/mpc_example.py (장면 구성, add_robot_to_scene, 명령 적용 방식)
        ref/franka-depth-mppi/experiment/hand_experiment.py (state_input=measured, 위치 명령만, 실패 시 측정 자세 유지)
  - 장면: 바닥 + Franka(cuRobo franka.yml URDF 를 helper 로 import) + step2_scene 의 cuboid 5개 (시각용 VisualCuboid)
  - 매 물리 스텝: world.step → 측정 관절 상태 → mpc.step(max_attempts=2) → 위치 명령 적용
  - mover 는 매 스텝 이동: Isaac 시각 cuboid 와 cuRobo 충돌 모델을 같은 값으로 갱신
  - 기록: solver 시간 평균/p95/최대, 루프 실시간 배율(시뮬레이션 시간 ÷ 실제 시간), 프로세스 VRAM, 도달/실패 횟수
"""
import argparse
import datetime
import json
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

ap = argparse.ArgumentParser()
ap.add_argument("--gui", action="store_true")
ap.add_argument("--steps", type=int, default=1200)
ap.add_argument("--mppi-seed", type=int, default=-1, help="(B1) ≥0: particle_mpc.yml mppi.seed·sample_params.seed 를 이 값으로 덮어씀 (override_particle_file)")
ap.add_argument("--b1-log", type=int, default=0, help="(B1) 1: 스텝별 명령·EE·형상별 최소 간격(로봇 구 전체)·그 쌍의 μ 기록 → {TAG}_b1.npz")
ap.add_argument("--tag", default=None)
ap.add_argument("--hold", type=float, default=0.0)
ap.add_argument("--scene", default="v1")
ap.add_argument("--robot", default=None, choices=[None, "panda", "fr3", "fr3v2_1"], help="(goal1) 지정하면 STEP_ROBOT 대신 사용. 기본 panda, fr3 = v1 USD 비교용, fr3v2_1 = 기준 모델(URDF 변환)")
ap.add_argument("--arm-gains", default=None, help='(goal1) FR3(fr3, fr3v2_1) 팔 관절 구동 이득 "stiffness,damping" (USD 단위) 또는 "usd"(그대로). 기본 625,10 = 환경 점검 Panda 변환 모델 실제 값, 목표 2의 2-3에서 다시 정함')
# 3단계: ROS 2 동시 실행 (기본 꺼짐 → 2단계 동작과 같음). run_isaac_ros.sh (ROS_METHOD=min) 로 실행할 것
ap.add_argument("--ros", action="store_true")
ap.add_argument("--ready-file", default=None)
ap.add_argument("--done-file", default=None)
ap.add_argument("--ros-out", default=None)
# 3-2 재측정: 측정 방법 옵션 (MPC 설정·장면·ROS 처리 방식은 바꾸지 않음)
ap.add_argument("--no-diag", action="store_true",
                help="매 스텝 점검 계산(로봇 구 CPU 복사·numpy 여유·EE/관절 GPU→CPU 복사)과 100스텝마다 nvidia-smi 조회를 끔")
ap.add_argument("--warmup", type=int, default=0, help="측정 루프 전에 같은 장면·같은 상태로 mpc.step 을 N회 호출")
# 4단계: 사람 데이터 (human_dynamics_node 출력) 수신·표시·GPU 텐서. MPC 비용에는 쓰지 않음
ap.add_argument("--human", action="store_true")
ap.add_argument("--human-mode", default="full", choices=["full", "recv"],
                help="full: 표시(PointInstancer)+GPU 텐서 (4-3). recv: 구독·수신 기록만, 표시·GPU 텐서 없음 (4-3 B)")
ap.add_argument("--human-dir", default="~/effmass_mpc/env_check/step4", help="temp_transform.json·mesh_transform.json 위치 (5단계: step5)")
ap.add_argument("--mesh-offset", default="node", choices=["node", "rigid", "node_live"])
ap.add_argument("--human-mesh", default="none", choices=["none", "usd", "rt"],
                help="(4-5) SMPL 메시 (HumanPose.smpl_vertices) 갱신: usd = UsdGeom.Mesh points 한 번에 쓰기, 새 HumanPose 때만")
ap.add_argument("--ros-pose-topic", default="/step3/human_pose", help="3단계 시험 HumanPose 구독 토픽. none 이면 구독·대기 안 함")
# (goal1 1-2d) 사람 데이터 출처·유효질량 채널. 기본값은 이전 동작 그대로 (dyn, off). inertia/on 은 EFFMASS_WS=1 run_isaac_ros.sh 로 실행
ap.add_argument("--human-source", default="dyn", choices=["dyn", "inertia", "precomputed"], help="dyn: 원본 노드 HumanDynamicsState (4단계), inertia: 사본 노드 HumanInertiaState, precomputed: 미리 계산한 프레임을 시뮬레이션 시각으로 (1-3)")
ap.add_argument("--human-pre", default="~/effmass_mpc/goal1/data/human_precomputed_1781524848.npz")
ap.add_argument("--human-frame", default="", help="(live) node world -> Isaac transform file for the human (e.g. goal1/config/live_frame.yml); empty = <human-dir>/temp_transform.json")
ap.add_argument("--human-delay-frames", type=int, default=0)
ap.add_argument("--human-lead-s", type=float, default=0.0, help="(overnight) precomputed: move shapes ahead by file velocity x lead (s); 0 = off")
ap.add_argument("--human-lag-s", type=float, default=0.0, help="(overnight) precomputed: deliver the frame of t_sim - lag (s); 0 = off")
ap.add_argument("--human-jitter-m", type=float, default=0.0, help="(overnight) precomputed: per frame per segment Gaussian position noise std (m); 0 = off")
ap.add_argument("--human-drop-frac", type=float, default=0.0, help="(overnight) precomputed: fraction of frames missing, last frame kept; 0 = off")
ap.add_argument("--human-noise-seed", type=int, default=0, help="(overnight) seed for jitter and dropout")
ap.add_argument("--human-vel-alpha", type=float, default=0.0, help="(overnight) precomputed: EMA factor for the delivered segment velocity (0 = off)")
ap.add_argument("--shapes", default="v1", choices=["v1", "v2"], help="(shapes v2) precomputed source: v2 uses <human-pre>_v2.npz (21 shapes + 3 inactive). v1 = previous results")
ap.add_argument("--human-viz", default="both", choices=["none", "mesh", "shapes", "both"], help="(B2a) precomputed source display: body mesh and/or one-colour capsules")
ap.add_argument("--human-mesh-pre", default="", help="(B2a) display-only vertex file; empty = <human-pre>_verts.npz if it exists")
ap.add_argument("--meff", default="off", choices=["off", "on"], help="on: 가중치 0 유효질량 채널 (g1_meff_gpu.MeffChannel)을 cost_fn 에 끼움, 매 스텝 Λ_r 공급")
ap.add_argument("--meff-impl", default="warp", choices=["pairs", "skip0", "nearest", "warp"])
ap.add_argument("--meff-dir", default="auto", choices=["auto", "com", "closest"], help="auto = from --u-def")
ap.add_argument("--u-def", default="nearest", choices=["com", "nearest"], help="(B2a) u = sphere centre -> segment COM (com, B1/A-stage) or -> nearest point on the shape (nearest, COM fallback within 1 mm of the axis)")
ap.add_argument("--cost-agg", type=int, default=0, help="(B2a) 1: per step, rollout cost aggregation on the GPU (last MPPI iteration): count and cost sum per selected shape, fallback count")
ap.add_argument("--scene-json", default="", help="(B2a) scene g2 json (goals_wxyz, q_start)")
ap.add_argument("--single-pass", type=int, default=0, help="(B2a) 1: start at q_start (goal A), goal = B for the whole run, no goal switching")
ap.add_argument("--meff-motion", default="static", choices=["static", "cv"])
ap.add_argument("--meff-hfront", type=int, default=0)
ap.add_argument("--meff-viz", default="none", choices=["none", "shapes", "lines", "both"], help="GUI 확인용: 사람 형상 반투명 표시, 로봇 구↔가장 가까운 형상 선")
ap.add_argument("--meff-lines", default="topk", choices=["min", "topk", "all"], help="선: min = 가장 가까운 쌍 하나, topk = 가장 가까운 k 쌍 (기본), all = 로봇 구마다")
ap.add_argument("--meff-lines-k", type=int, default=5, help="topk 선 개수")
ap.add_argument("--meff-paired", type=int, default=0, help="1 = 채널 없는 solver 를 하나 더 두고 매 스텝 같은 측정 상태로 번갈아 실행 (solver 증가량 짝 측정; 루프·배율은 이 실행에서 쓰지 않음)")
# (goal1 1-3) 조합 1 비용 (채널 --meff on 일 때): 모드·합산·파라미터는 GPU 텐서 (값 변경에 재캡처 없음)
ap.add_argument("--meff-mode", default="off", choices=["off", "uniform", "mass", "iso", "gain"])
ap.add_argument("--meff-dbase", type=float, default=0.065, help="(V1) gain mode: base clearance d_base (m), also the lower clamp")
ap.add_argument("--meff-gain", type=float, default=0.10, help="(V1) gain mode: k_gain (m) on max(sqrt(mu / mu_ref) - 1, 0)")
ap.add_argument("--meff-agg", default="max_all", choices=["max_all", "nearest"])
ap.add_argument("--meff-w", type=float, default=100000.0, help="w (고정). 기본 = 프로토타입·cuRobo primitive_collision weight")
ap.add_argument("--meff-eta0", type=float, default=0.10)
ap.add_argument("--meff-muref", type=float, default=3.0)
ap.add_argument("--meff-p", type=float, default=0.5)
ap.add_argument("--meff-eta-min", type=float, default=0.05)
ap.add_argument("--meff-eta-max", type=float, default=0.20)
ap.add_argument("--meff-switch-step", type=int, default=-1, help="이 스텝에서 η0 를 --meff-switch-eta0 로 바꿈 (재캡처 0 확인)")
ap.add_argument("--meff-switch-eta0", type=float, default=0.15)
ap.add_argument("--meff-log-near", type=float, default=0.0, help=">0: 매 스텝 실행된 상태의 로봇 구↔형상 쌍 중 거리 < 이 값인 것과 스텝별 최소 거리를 기록 (GPU 동기화 추가, 시간 측정 실행에서는 쓰지 않음)")
ap.add_argument("--loop-prof", type=int, default=0, help="1 = 루프 구간별 벽시계·프로세스 CPU 시간 기록 (time.perf_counter·process_time 만, GPU 동기화 추가 없음)")
ap.add_argument("--speed-cap", default="off", choices=["off", "on"], help="(T1) rollout cost on robot sphere speed near the human (off = not in the graph)")
ap.add_argument("--speed-cap-v", type=float, default=0.5, help="(T1) speed limit m/s")
ap.add_argument("--speed-cap-dist", type=float, default=0.4, help="(T1) penalty only for spheres whose gap to a human shape is <= this (m)")
ap.add_argument("--speed-cap-w", type=float, default=0.0, help="(T1) weight of max(0, v - v_cap)^2 (max over spheres, per rollout step)")
ap.add_argument("--pose-rot-w", type=float, default=None, help="(T1) pose cost orientation weight (default: particle_mpc.yml, 60)")
ap.add_argument("--round-trip", type=int, default=0, help="(R1) 1: start at the scene q_start (A), first goal B, then switch goals on arrival (round trips)")
ap.add_argument("--viz-eta-shell", default="off", choices=["off", "on"], help="(demo) translucent eta shells on the v2 hand shapes (display only)")
ap.add_argument("--viz-trail", default="off", choices=["off", "on"], help="(demo) TCP trail of the last --viz-trail-sec seconds (display only)")
ap.add_argument("--viz-trail-sec", type=float, default=3.0)
ap.add_argument("--viz-hide-com", default="off", choices=["off", "on"], help="(demo) hide the human segment COM markers")
ap.add_argument("--viz-zone-opacity", type=float, default=0.35)
ap.add_argument("--viz-zone-seq", default="", help="(R1) index.json of zone cages along the hand motion (display only)")
ap.add_argument("--viz-zone-seq-cond", default="ours", choices=["ours", "uniform"])
ap.add_argument("--viz-zone-style", default="cage", choices=["cage", "surface"], help="(V2) zone as contour rings (see-through) or translucent surface")
ap.add_argument("--viz-zone", default="", help="(V1) zone surface npz from v1_zone.py (static translucent mesh, display only)")
ap.add_argument("--viz-ghost", default="", help="(demo) conditions whose saved TCP path is drawn in grey, e.g. iso,unim")
ap.add_argument("--viz-ghost-tag", default="", help="(demo) saved run tag with {cond}, e.g. b2a5_front_0.38h1_{cond}_s0")
ap.add_argument("--viz-cam", default="", help="(demo) camera eye,target (6 numbers): GUI viewport and recording camera")
ap.add_argument("--viz-record", default="", help="(demo) save frames of --viz-cam every step to this directory (headless render) and make <dir>.mp4")
ap.add_argument("--viz-record-res", default="1280x720")
ap.add_argument("--profile", default="", help="(T1) tuned profile name: loads ~/effmass_mpc/goal1/config/tuned_<name>.yml (keys = option names with _); explicit options win")
ap.add_argument("--meff-paired-base", default="none", choices=["none", "channel_off", "speedcap_off"], help="(1-3) 짝 측정의 비교 solver: none = 채널 없음, channel_off = 같은 채널(모드 off, 가중치 0)")
ap.add_argument("--meff-dump-steps", default="", help="이 스텝들에서 디버그 텐서 일부 저장 (CPU 기준 비교용, 시간 측정 실행에서는 쓰지 않음)")
args = ap.parse_args()
PROFILE = None
if args.meff_mode == "gain":   # (V1) gain mode: kernel slots eta0 = d_base, p = k_gain, eta_min = d_base; defaults mu_ref 0.53 kg, eta_max 0.25 m unless given
    args.meff_eta0, args.meff_p, args.meff_eta_min = args.meff_dbase, args.meff_gain, args.meff_dbase
    if "--meff-muref" not in sys.argv:
        args.meff_muref = 0.53
    if "--meff-eta-max" not in sys.argv:
        args.meff_eta_max = 0.25
if args.profile:   # (T1) profile values for options not given explicitly on the command line
    import yaml as _yaml
    PROFILE = _yaml.safe_load(open(os.path.expanduser(f"~/effmass_mpc/goal1/config/tuned_{args.profile}.yml")))
    for _k, _v in PROFILE["settings"].items():
        if "--" + _k.replace("_", "-") not in sys.argv:
            setattr(args, _k, _v)
if args.meff_dir == "auto":
    args.meff_dir = "com" if args.u_def == "com" else "closest"
GUI = args.gui
TAG = args.tag or ("step2b_gui" if GUI else "step2b_headless")
T_START = time.perf_counter()
STAMP = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def log(*a):
    print("[step2b]", *a, flush=True)


log(f"start {datetime.datetime.now().isoformat(timespec='seconds')} mode={'GUI' if GUI else 'headless'} steps={args.steps}")

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": not GUI, "width": 1280, "height": 720})
log(f"SimulationApp ready {time.perf_counter() - T_START:.1f}s")

import numpy as np
import torch

if args.ros:
    import threading

    import step3_ros_common as RC

    G1_IMPORT = None
    if args.human_source in ("inertia", "precomputed") or args.meff == "on":
        import warp as _wp0, curobo as _cu0
        _mods = lambda: {m.__name__: (getattr(m, "__file__", None), getattr(m, "__version__", None), id(m)) for m in (np, torch, _wp0, _cu0)}
        _before = _mods()
    _added = RC.add_ros_paths()
    if args.human_source in ("inertia", "precomputed") or args.meff == "on":
        import effmass_msgs.msg as _em0
        import effmass_robot.robot_inertia_np as _er0      # (1-2d 결정 1) numpy Λ_r. Pinocchio 는 Kit 안에서 import 하지 않음 (Assimp·urdfdom 충돌)
        import rclpy as _rc0
        _after = _mods()
        G1_IMPORT = dict(unchanged=_before == _after, modules={k: v[:2] for k, v in _after.items()}, pinocchio_loaded="pinocchio" in sys.modules, effmass_msgs=_em0.__file__,
                         effmass_robot=_er0.__file__, rclpy=_rc0.__file__, added=_added)
        log(f"(1-2d) import 확인: Isaac numpy·torch·warp·curobo 경로·객체 그대로 {G1_IMPORT['unchanged']}, pinocchio 로드 {G1_IMPORT['pinocchio_loaded']}, effmass_msgs {_em0.__file__}, Λ_r {_er0.__file__}")
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile
    from hrc_interfaces.msg import HumanPose
    from sensor_msgs.msg import JointState as RosJointState

    import step3_msgs as M
    log(f"ROS: rclpy {RC.short(rclpy.__file__)}, ROS 경로 {len(_added)}개를 sys.path 뒤에 추가, "
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} ROS_LOCALHOST_ONLY={os.environ.get('ROS_LOCALHOST_ONLY')}")

import step2_scene as S

if args.scene_json:
    S.G2_SCENE_JSON = os.path.expanduser(args.scene_json)
SCENE_INFO = S.set_scene(args.scene)

CAPTURES = S.install_graph_capture_counter(torch)

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.types import ArticulationAction

sys.path.insert(0, os.path.expanduser("~/effmass_mpc/curobo/examples/isaac_sim"))
from helper import add_robot_to_scene  # noqa: E402  (cuRobo 예제 helper, 수정 없이 사용)

import curobo
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.util.logger import setup_curobo_logger

setup_curobo_logger("warn")
log(f"curobo from {curobo.__file__}")
torch.manual_seed(S.SEED)
torch.cuda.manual_seed_all(S.SEED)
np.random.seed(S.SEED)
ta = TensorDeviceType()
PID = os.getpid()

# ---------------- 장면 ----------------
world = World(stage_units_in_meters=1.0)
world.set_simulation_dt(physics_dt=S.PHYSICS_DT, rendering_dt=S.PHYSICS_DT)
stage = world.stage
xform = stage.DefinePrim("/World", "Xform")
stage.SetDefaultPrim(xform)
world.scene.add_default_ground_plane(z_position=S.GROUND_Z)   # g1 장면: 바닥 z -0.9 (베이스 아래 0.9 m)

if args.robot:
    os.environ["STEP_ROBOT"] = args.robot
if args.arm_gains:
    os.environ["STEP_FR3_ARM_GAINS"] = args.arm_gains
robot_cfg = S.build_robot_cfg()
names = robot_cfg["kinematics"]["cspace"]["joint_names"]
q0 = robot_cfg["kinematics"]["cspace"]["retract_config"]
robot, robot_prim_path = S.add_isaac_robot(robot_cfg, world)
log(f"robot {S.robot_name()} prim {robot_prim_path}, cuRobo urdf {robot_cfg['kinematics']['urdf_path']}, FR3 팔 구동 이득 {S.FR3_GAINS_APPLIED or '-'}")

world_cfg = S.build_world_config()
visuals = {}
for c in world_cfg.cuboid:
    if c.name in ("table", "floor"):
        continue  # 바닥면과 겹치는 판·바닥은 시각화 생략 (충돌 모델에는 포함)
    color = np.array([0.9, 0.2, 0.2]) if c.name == S.MOVER_NAME else np.array([0.55, 0.45, 0.35]) if c.name == "robot_table" else np.array([0.5, 0.5, 0.7])
    visuals[c.name] = VisualCuboid(
        f"/World/obstacles/{c.name}", position=np.array(c.pose[:3]), orientation=np.array(c.pose[3:]),
        scale=np.array(c.dims), size=1.0, color=color,
    )
for gi, g in enumerate(S.GOALS_WXYZ):
    VisualCuboid(f"/World/goal{gi}", position=np.array(g[:3]), orientation=np.array(g[3:]), size=0.04,
                 color=np.array([0.2, 0.8, 0.2]))
if GUI:
    from isaacsim.core.utils.viewports import set_camera_view
    _vc = [float(x) for x in args.viz_cam.split(";")[0].split(",")] if args.viz_cam else [1.9, 0.0, 1.2, 0.3, 0.0, 0.4]   # (demo) --viz-cam
    set_camera_view(eye=_vc[:3], target=_vc[3:])
VIZ_RENDER = bool(args.viz_record)   # (demo) headless frames need rendering; the solver and physics do not depend on it
VIZ = None

# ---------------- MPC ----------------
t0 = time.perf_counter()
S.MPPI_SEED = args.mppi_seed if args.mppi_seed >= 0 else None
S.POSE_ROT_W = args.pose_rot_w
mpc = S.build_mpc(robot_cfg, world_cfg)
torch.cuda.synchronize()
t_build = time.perf_counter() - t0
info = S.describe_mpc(mpc)
_o = mpc.solver.optimizers[0]; _sl = _o.sample_lib
MPPI_SEED_INFO = dict(arg=args.mppi_seed, override_file=S.mppi_override_file(), opt_seed=int(getattr(_o, "seed", -1)), sample_params_seed=int(_o.sample_params.seed),
                      halton_gen_seed=int(_sl.halton_sample_lib.halton_generator._seed), knot_gen_seed=int(_sl.knot_halton_sample_lib.halton_generator._seed))
log(f"(B1) MPPI seed: {MPPI_SEED_INFO}")
log(f"MpcSolver 생성 {t_build:.2f}s  설정 " + json.dumps({k: v for k, v in info.items() if k != 'traj_dt'}))
log(f"(T1) profile {args.profile or '-'} {PROFILE['settings'] if PROFILE else ''}; pose cost weights per rollout {S.pose_weights(mpc)}; speed cap {args.speed_cap} v {args.speed_cap_v} dist {args.speed_cap_dist} w {args.speed_cap_w}")
checker = mpc.world_collision
ATTEMPTS = S.count_step_once(mpc)  # _step_once 호출 수 (설정 변경 없음)
# (goal1 1-2d) 유효질량 채널: 첫 mpc.step (그래프 캡처) 전에 끼움
MEFF = MEFF_INP = RI = None
COSTAGG = None
S_ACT_PLATE = 0.025   # (B2a-2) cuRobo primitive collision activation distance (read back below from the solver and checked)
MEFF_SV = 0
MEFF_DUMP = {int(x) for x in args.meff_dump_steps.split(",") if x}
if args.human_source in ("inertia", "precomputed") or args.meff == "on":
    sys.path.insert(0, os.path.expanduser("~/effmass_mpc/goal1/scripts"))
    import g1_meff_gpu as G1M
    _ls = mpc.kinematics.kinematics_config.link_spheres.cpu().numpy()
    MEFF_SV = int((_ls[:, 3] > 0).sum())
    assert (_ls[:MEFF_SV, 3] > 0).all() and not (_ls[MEFF_SV:, 3] > 0).any()
    MEFF_INP = G1M.MeffInputs(MEFF_SV, info["traj_dt"], device=str(ta.device))
if args.meff == "on":
    from effmass_robot.robot_inertia_np import RobotInertiaNP
    RI = RobotInertiaNP(rotor=True)
    assert len(RI.sphere_link) == MEFF_SV
    _rng = np.random.default_rng(5); _lo = np.array([-2.7, -1.7, -2.8, -3.0, -2.7, 0.6, -3.0]); _hi = -_lo; _hi[3] = -0.2; _hi[5] = 4.4
    _nz = np.zeros(MEFF_SV)
    for _ in range(20):
        _nz = np.maximum(_nz, np.abs(RI.compute(_lo + (_hi - _lo) * _rng.uniform(size=7))[1]).reshape(MEFF_SV, -1).max(1))
    MEFF_MASS_ON = (_nz > 0).astype(np.int32)
    MEFF = G1M.MeffChannel(mpc, MEFF_INP, impl=args.meff_impl, direction=args.meff_dir, motion=args.meff_motion, hfront=args.meff_hfront,
                           mass_on=MEFF_MASS_ON, record_q=bool(MEFF_DUMP), speed_cap=args.speed_cap == "on")
    MEFF.set_cost(mode=args.meff_mode, agg=args.meff_agg, w=args.meff_w, eta0=args.meff_eta0, muref=args.meff_muref, p=args.meff_p,
                  eta_min=args.meff_eta_min, eta_max=args.meff_eta_max)
    MEFF.set_speed_cap(v=args.speed_cap_v, dist=args.speed_cap_dist, w=args.speed_cap_w)
    log(f"(1-3) 비용: {MEFF.cost_state()}")
    log(f"(1-2d) 유효질량 채널: impl {args.meff_impl}, u {args.meff_dir}, 사람 시간 {args.meff_motion}, hfront {args.meff_hfront}, 구 {MEFF_SV}, "
        f"Λ_r=0 구 {np.nonzero(MEFF_MASS_ON == 0)[0].tolist()}, optimizers {MEFF.info}, Λ_r 구현 numpy (RobotInertiaNP)")
if MEFF is not None and args.cost_agg:
    COSTAGG = dict(n=torch.zeros(args.steps, 24, device=ta.device), s=torch.zeros(args.steps, 24, device=ta.device), fb=torch.zeros(args.steps, device=ta.device))
    if MEFF.speed_cap:   # (T1) speed-cap cost on the last MPPI iteration: count of (particle, step) with cost > 0 and the sum
        COSTAGG.update(vn=torch.zeros(args.steps, device=ta.device), vs=torch.zeros(args.steps, device=ta.device))
    PLATES = [c for c in world_cfg.cuboid if c.name.startswith("plate")]   # (B2a-2) axis-aligned plates: rollout count of sphere-plate gap < activation
    if PLATES:
        PL_LO = torch.tensor([[c.pose[k] - c.dims[k] / 2 for k in range(3)] for c in PLATES], device=ta.device, dtype=torch.float32)
        PL_HI = torch.tensor([[c.pose[k] + c.dims[k] / 2 for k in range(3)] for c in PLATES], device=ta.device, dtype=torch.float32)
        COSTAGG.update(pn=torch.zeros(args.steps, len(PLATES), device=ta.device), pmin=torch.zeros(args.steps, len(PLATES), device=ta.device))
        assert abs(info["primitive_collision_activation_m"] - S_ACT_PLATE) < 1e-6
        log(f"(B2a-2) plates {[c.name for c in PLATES]}, rollout plate count on (activation {S_ACT_PLATE} m)")
MPC_OFF = None
if args.meff_paired:
    assert MEFF is not None and not S.HAS_MOVER
    torch.manual_seed(S.SEED); torch.cuda.manual_seed_all(S.SEED)
    MPC_OFF = S.build_mpc(robot_cfg, world_cfg); torch.cuda.synchronize()
    if args.meff_paired_base == "channel_off":
        MEFF_OFF = G1M.MeffChannel(MPC_OFF, MEFF_INP, impl=args.meff_impl, direction=args.meff_dir, motion=args.meff_motion, hfront=args.meff_hfront,
                                   mass_on=MEFF_MASS_ON, record_q=False)
        MEFF_OFF.set_cost(mode="off")
    elif args.meff_paired_base == "speedcap_off":   # (T1) same human cost, no speed cap term
        MEFF_OFF = G1M.MeffChannel(MPC_OFF, MEFF_INP, impl=args.meff_impl, direction=args.meff_dir, motion=args.meff_motion, hfront=args.meff_hfront,
                                   mass_on=MEFF_MASS_ON, record_q=False, speed_cap=False)
        MEFF_OFF.set_cost(mode=args.meff_mode, agg=args.meff_agg, w=args.meff_w, eta0=args.meff_eta0, muref=args.meff_muref, p=args.meff_p,
                          eta_min=args.meff_eta_min, eta_max=args.meff_eta_max)
    log(f"(1-2d) 짝 측정: 비교 solver 하나 더 생성 ({args.meff_paired_base}; 로봇은 주 solver 명령으로 움직임)")
STEP_ROWS = []
log("scene " + json.dumps(SCENE_INFO))

js0 = JointState.from_position(mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0),
                               joint_names=mpc.joint_names)
goal_idx = 1 if (args.single_pass or args.round_trip) else 0   # (B2a) single pass: goal B from the start; (R1) round trip: same start
gb = mpc.setup_solve_single(Goal(current_state=js0, goal_state=js0,
                                 goal_pose=Pose.from_list(S.GOALS_WXYZ[goal_idx], ta)), 1)
mpc.update_goal(gb)
if MPC_OFF is not None:
    gb_off = MPC_OFF.setup_solve_single(Goal(current_state=js0, goal_state=js0, goal_pose=Pose.from_list(S.GOALS_WXYZ[goal_idx], ta)), 1)
    MPC_OFF.update_goal(gb_off)

world.reset()
world.play()
_rg = S.apply_arm_gains_runtime(robot)
if _rg:
    log(f"FR3 팔 이득 (실행 중 articulation, view 단위 = USD×180/π): {_rg}")
idx_list = [robot.get_dof_index(n) for n in names]
if args.single_pass or args.round_trip:
    q0 = list(SCENE_INFO["q_start"]) + list(q0[len(SCENE_INFO["q_start"]):])   # (B2a) start at goal A (IK), fingers as retract
robot.set_joint_positions(q0, idx_list)
robot._articulation_view.set_max_efforts(values=np.full(len(idx_list), 5000.0), joint_indices=idx_list)
controller = robot.get_articulation_controller()
for _ in range(10):
    world.step(render=GUI)
cmd_idx = [robot.get_dof_index(n) for n in mpc.joint_names]
log(f"장면 준비 완료 {time.perf_counter() - T_START:.1f}s, 그래프 캡처(생성 단계) {len(CAPTURES)}")


_LAST_ST = [None]
HUMAN = None


def measured_state():
    st = robot.get_joints_state()
    _LAST_ST[0] = st
    pos = ta.to_device(st.positions)[cmd_idx].unsqueeze(0)
    vel = ta.to_device(st.velocities)[cmd_idx].unsqueeze(0)
    return JointState(position=pos, velocity=vel, acceleration=vel * 0.0, jerk=vel * 0.0,
                      joint_names=list(mpc.joint_names))


T_FIELDS = ["t_obst_ms", "t_phys_ms", "t_state_ms", "t_js_ms", "t_presync_ms", "t_solver_ms", "t_cmd_ms",
            "t_goal_ms", "t_diag_ms", "t_book_ms", "t_human_ms", "t_h_pickup_ms", "t_h_transform_ms", "t_h_disp_ms",
            "t_h_tensor_ms", "h_new", "t_h_mesh_ms", "t_h_mesh_xf_ms", "h_mesh_new", "t_meff_r_ms", "t_meff_viz_ms", "t_h_shapes_ms"]
MEFF_QSTATE = [None]
PAIRED = []
NEAR_ROWS, NEAR_MIN = [], []
B1_ROWS = []   # (B1) (step, q_cmd 7, ee 3, feasible, dmin 24, smin 24, mu 24, mh 24, mr 24)
PROF_ON = bool(args.loop_prof)
PROF_MARKS = []          # 이번 스텝의 (구간 끝 이름, perf_counter, process_time)
PROF_ROWS = []
PROF_NAMES = []
def _PM(name):
    if PROF_ON:
        PROF_MARKS.append((name, time.perf_counter(), time.process_time()))
if MEFF_DUMP:
    os.makedirs(os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac"), exist_ok=True)
ROW_FIELDS = S.STEP_FIELDS + T_FIELDS
solver_ms, loop_ms, phys_ms = [], [], []
first_call_ms = None
arrivals, switches, infeasible, holds = [], 0, 0, 0
leg_start = 0
min_clear = float("inf")
vram = []
sim_t0 = world.current_time
S.COUNTER_TAG[0] = "step_0"

if args.ros:
    # 수신: 별도 데몬 스레드의 SingleThreadedExecutor.spin(). 콜백은 가벼운 검사(check_pose full=False) 후 기록만 한다.
    # 발행: 메인 루프(one_step)에서 매 물리 스텝 측정 관절 상태를 publish() 1회. 루프는 ROS 때문에 기다리지 않는다.
    rclpy.init()
    ros_node = rclpy.create_node("step3_mpc_node")
    ROS_RECV, ROS_SENT = [], []

    def _on_pose(msg):
        t = time.time_ns()
        kind, seq = M.parse_frame_id(msg.header.frame_id)
        ROS_RECV.append([kind, seq, M.stamp_ns(msg.header), t, int(M.check_pose(msg, seq, full=False))])

    if args.ros_pose_topic != "none":
        ros_node.create_subscription(HumanPose, args.ros_pose_topic, _on_pose, M.best_effort_qos())
    ros_pub = ros_node.create_publisher(RosJointState, "/step3/joint_states", QoSProfile(depth=10))
    HUMAN = None
    HUMAN_FRAME = os.path.expanduser(args.human_frame) if args.human_frame else os.path.join(os.path.expanduser(args.human_dir), "temp_transform.json")   # (live)
    if args.human and args.human_source == "precomputed":
        import g1_human_link as G1L
        _hp = os.path.expanduser(args.human_pre)
        if args.shapes == "v2" and not _hp.endswith("_v2.npz"):
            _hp = _hp.replace(".npz", "_v2.npz")
        args.human_pre = _hp
        _base = _hp[:-len("_v2.npz")] + ".npz" if _hp.endswith("_v2.npz") else _hp
        _mp = os.path.expanduser(args.human_mesh_pre) if args.human_mesh_pre else _base.replace(".npz", "_verts.npz")
        _mp = _mp if (args.human_viz in ("mesh", "both") and os.path.exists(_mp)) else None
        HUMAN = G1L.PrecomputedHumanLink(os.path.expanduser(args.human_pre), S.PHYSICS_DT, HUMAN_FRAME, MEFF_INP,
                                         delay_frames=args.human_delay_frames, stage=stage, display=("instancer",) if args.human_mode == "full" else (),
                                         shapes_viz=args.meff_viz in ("shapes", "both") or args.human_viz in ("shapes", "both"), mesh_npz=_mp,
                                         lead_s=args.human_lead_s, lag_s=args.human_lag_s, jitter_m=args.human_jitter_m, drop_frac=args.human_drop_frac,
                                         noise_seed=args.human_noise_seed, vel_alpha=args.human_vel_alpha)
        log(f"(overnight) human perturbation: lead {args.human_lead_s} s, lag {args.human_lag_s} s, jitter {args.human_jitter_m} m, drop {args.human_drop_frac}, seed {args.human_noise_seed}, vel alpha {args.human_vel_alpha}")
        log(f"(B2a) human display: viz {args.human_viz}, mesh file {_mp}, capsules {HUMAN.shapes_viz}; shapes {args.shapes}, file {_hp}, active shapes {int((HUMAN.pre_static['rad'] > 0).sum())}")
        HUMAN.pump(-1)      # 예열 전 첫 프레임 (t_sim 0)
        log(f"사람(1-3): 미리 계산한 프레임 {args.human_pre} ({HUMAN.F}개, {HUMAN.rate} Hz), delay_frames {args.human_delay_frames}, 물리 dt {S.PHYSICS_DT}, "
            f"예열 전 프레임 {HUMAN.cur_frame}")
    elif args.human and args.human_source == "inertia":
        import step4_human as H4
        import g1_human_link as G1L
        HUMAN = G1L.HumanInertiaLink(ros_node, HUMAN_FRAME, MEFF_INP, stage=stage,
                                     display=("instancer",) if args.human_mode == "full" else (), shapes_viz=args.meff_viz in ("shapes", "both"))
        if args.human_mesh != "none":
            HUMAN.mesh_link = H4.HumanLink(ros_node, os.path.join(os.path.expanduser(args.human_dir), "temp_transform.json"), stage=stage, display=(), tensor=False,
                                           verify_every=0, mesh=(args.human_mesh,), mesh_transform_json=os.path.join(os.path.expanduser(args.human_dir), "mesh_transform.json"),
                                           mesh_offset=args.mesh_offset, mesh_faces=os.path.expanduser("~/effmass_mpc/env_check/step4/smpl_neutral_faces.npy"), mesh_verify_every=0)
        log(f"사람(1-2d): HumanInertiaState 사본 노드, 처리 방식 {args.human_mode}, 형상 표시 {args.meff_viz}, 메시 {args.human_mesh}, 변환 폴더 {args.human_dir}")
    elif args.human:
        import step4_human as H4
        HUMAN = H4.HumanLink(ros_node, os.path.join(os.path.expanduser(args.human_dir), "temp_transform.json"), stage=stage,
                             display=("instancer",) if args.human_mode == "full" else (),
                             tensor=args.human_mode == "full", verify_every=0,
                             mesh=() if args.human_mesh == "none" else (args.human_mesh,),
                             mesh_transform_json=os.path.join(os.path.expanduser(args.human_dir), "mesh_transform.json"), mesh_offset=args.mesh_offset,
                             mesh_faces=os.path.expanduser("~/effmass_mpc/env_check/step4/smpl_neutral_faces.npy"), mesh_verify_every=0)
        log(f"사람: 처리 방식 {args.human_mode}, 메시 {args.human_mesh}, 변환 폴더 {args.human_dir}, 메시 오프셋 {args.mesh_offset}")
    ros_ex = SingleThreadedExecutor()
    ros_ex.add_node(ros_node)
    ros_th = threading.Thread(target=ros_ex.spin, daemon=True)
    ros_th.start()
    ROS_DOF_NAMES = list(robot.dof_names)
    open(args.ready_file, "w").write(str(time.time_ns()))
    _w0 = time.time()
    if args.ros_pose_topic != "none":
        while (ros_node.count_publishers(args.ros_pose_topic) == 0 or len(ROS_RECV) == 0) and time.time() - _w0 < 180:
            time.sleep(0.05)
        log(f"ROS 준비: HumanPose 발행자 {ros_node.count_publishers(args.ros_pose_topic)}, 수신 {len(ROS_RECV)}개 "
            f"({time.time() - _w0:.1f}s 대기), JointState 관절 {ROS_DOF_NAMES}")
    HUMAN_FIRST_DYN_NS = None
    if HUMAN is not None:
        while HUMAN.n_dyn == 0 and time.time() - _w0 < 240:
            time.sleep(0.02)
        HUMAN_FIRST_DYN_NS = time.time_ns()
        log(f"사람: 첫 HumanDynamicsState 수신 ({time.time() - _w0:.1f}s 대기), HumanPose 수신 {len(HUMAN.pose_rec)}개")

# ---- 예열: 측정 루프 전에 같은 장면·같은 측정 상태로 mpc.step 을 N회 (명령 적용·물리 스텝 없음) ----
WARM = []
WARM_START_NS = WARM_END_NS = None
if args.warmup > 0:
    S.COUNTER_TAG[0] = "warmup"
    WARM_START_NS = time.time_ns()
    for w in range(args.warmup):
        cur = measured_state()
        torch.cuda.synchronize()
        _a0 = ATTEMPTS[0]
        _t = time.perf_counter()
        _r = mpc.step(cur, max_attempts=S.MAX_ATTEMPTS)
        torch.cuda.synchronize()
        if MPC_OFF is not None:
            MPC_OFF.step(cur, max_attempts=S.MAX_ATTEMPTS); torch.cuda.synchronize()
        WARM.append([w, round((time.perf_counter() - _t) * 1000, 4), ATTEMPTS[0] - _a0, int(bool(_r.metrics.feasible.all()))])
    WARM_END_NS = time.time_ns()
    _lim = 1000 / 60
    _stable = next((k for k in range(len(WARM)) if all(x[1] <= _lim for x in WARM[k:])), None)
    log(f"예열 {args.warmup}회: 호출별 ms {[x[1] for x in WARM]}")
    log(f"예열: 16.7 ms 이내로 안정된 첫 호출 번호(0부터) = {_stable}, 예열 그래프 캡처 {len(CAPTURES)}")
    S.COUNTER_TAG[0] = "step_0"
if args.ros:
    ROS_LOOP_START_NS = time.time_ns()
if args.no_diag:
    vram.append(S.proc_vram_mib(PID))  # 루프 밖에서 1회 (루프 안 조회는 끔)

loop_t0 = time.perf_counter()


def one_step(i, timed=True):
    global goal_idx, switches, infeasible, holds, leg_start, first_call_ms, min_clear
    pc = time.perf_counter
    PROF_MARKS.clear(); _PM("start")
    l0 = pc()
    if S.HAS_MOVER:
        mp = S.mover_pose(i * S.PHYSICS_DT)
        checker.update_obstacle_pose(S.MOVER_NAME, Pose.from_list(mp, ta))
        visuals[S.MOVER_NAME].set_world_pose(position=np.array(mp[:3]))
    else:
        mp = [float("nan")] * 7   # g1 장면: 움직이는 장애물 없음
    t1 = pc(); _PM("obst")
    world.step(render=GUI or VIZ_RENDER)
    if VIZ is not None and timed:
        VIZ.capture(i)
    t2 = pc(); _PM("physics")
    p_dt = t2 - t1
    current = measured_state()
    t3 = pc(); _PM("state")
    HT = None
    if args.ros and args.human and HUMAN is not None:
        HT = HUMAN.pump(i) if args.human_source == "precomputed" else HUMAN.pump()
        if getattr(HUMAN, "mesh_link", None) is not None:
            _HM = HUMAN.mesh_link.pump()
            for _k in ("mesh_new", "mesh_xf", "mesh_usd", "mesh_rt", "mesh_verify"):
                HT[_k] = _HM[_k]
    t3b = pc(); _PM("human")
    t_mr = 0.0
    if MEFF is not None:   # (1-2d) 로봇 Λ_r⁻¹: 현재 측정 관절 상태 하나로 Pinocchio → 고정 GPU 텐서 (horizon 동안 고정)
        _q = np.asarray(_LAST_ST[0].positions).reshape(-1)[cmd_idx].astype(np.float64)
        _tm = pc(); _Lr = RI.compute(_q)[1]; _PM("rinv_compute"); MEFF_INP.set_robot(_Lr); t_mr = (pc() - _tm) * 1000; _PM("rinv_upload")
        MEFF_QSTATE[0] = _q
    if args.ros and timed:
        _st = _LAST_ST[0]
        _m = M.fill_js(RosJointState(), i, kind="js", positions=np.asarray(_st.positions).reshape(-1).tolist(),
                       velocities=np.asarray(_st.velocities).reshape(-1).tolist())
        _m.name = ROS_DOF_NAMES
        _t = M.stamp_now(_m.header)
        ros_pub.publish(_m)
        ROS_SENT.append([i, _t] + list(_m.position))
    _PM("js_pub")
    s_off = None
    if MPC_OFF is not None and i % 2 == 1:      # 홀수 스텝: 채널 없는 solver 먼저
        torch.cuda.synchronize(); _so = pc(); MPC_OFF.step(current, max_attempts=S.MAX_ATTEMPTS); torch.cuda.synchronize(); s_off = (pc() - _so) * 1000
    t4 = pc(); _PM("paired_off_odd")
    torch.cuda.synchronize()
    a0 = ATTEMPTS[0]
    s0 = pc(); _PM("presync")
    result = mpc.step(current, max_attempts=S.MAX_ATTEMPTS)
    torch.cuda.synchronize()
    t5 = pc(); _PM("solver")
    s_dt = (t5 - s0) * 1000
    if MPC_OFF is not None and i % 2 == 0:      # 짝수 스텝: 채널 solver 먼저, 그다음 채널 없는 solver
        _so = pc(); MPC_OFF.step(current, max_attempts=S.MAX_ATTEMPTS); torch.cuda.synchronize(); s_off = (pc() - _so) * 1000
    if MPC_OFF is not None and timed and i > 0:
        PAIRED.append((i, s_dt, s_off))
    _PM("paired_off_even")
    if COSTAGG is not None and timed and i < args.steps:   # (B2a) rollout cost aggregation, no host sync
        _d0 = MEFF.optimizers[0][1]; _kc = _d0["kc"].clamp_min(0).long().view(-1); _cv = _d0["cost"].view(-1)
        COSTAGG["n"][i].scatter_add_(0, _kc, (_cv > 0).float()); COSTAGG["s"][i].scatter_add_(0, _kc, _cv); COSTAGG["fb"][i].copy_(_d0["fb"].sum())
        if "vn" in COSTAGG:
            COSTAGG["vn"][i].copy_((_d0["vcost"] > 0).sum()); COSTAGG["vs"][i].copy_(_d0["vcost"].sum())
        if "pn" in COSTAGG:   # (B2a-2) last MPPI iteration rollouts: gap = box distance of sphere centre - radius (radius includes the 0.02 buffer)
            _sp = _d0["sph_ref"]; _c = _sp[..., :3].reshape(-1, 1, 3); _r = _sp[..., 3].reshape(-1, 1); _ok = _r[:, 0] > 0
            _dd = torch.maximum(torch.maximum(PL_LO[None] - _c, _c - PL_HI[None]), torch.zeros((), device=ta.device))
            _ins = torch.minimum(_c - PL_LO[None], PL_HI[None] - _c).amin(-1); _g = torch.where(_ins > 0, -_ins, _dd.norm(dim=-1)) - _r
            _g = torch.where(_ok[:, None], _g, torch.full_like(_g, 1e3))
            COSTAGG["pn"][i].copy_((_g < S_ACT_PLATE).float().sum(0)); COSTAGG["pmin"][i].copy_(_g.amin(0))
    attempts = ATTEMPTS[0] - a0
    feasible = bool(result.metrics.feasible.all())
    cmd = result.js_action.get_ordered_joint_state(mpc.joint_names)
    if not feasible or not bool(torch.isfinite(cmd.position).all()):
        infeasible += 1
        holds += 1
        q_cmd = current.position  # 프로토타입 hold-measured
    else:
        q_cmd = cmd.position
    controller.apply_action(ArticulationAction(q_cmd.reshape(-1).cpu().numpy(), joint_indices=cmd_idx))
    t6 = pc(); _PM("cmd")

    # 목표 판정 (시나리오 동작: 목표 전환에 필요하므로 --no-diag 에서도 유지)
    kin = mpc.rollout_fn.compute_kinematics(current)
    ee = kin.ee_pos_seq.reshape(-1, 3)[0]
    q = kin.ee_quat_seq.reshape(-1, 4)[0]
    pe = float(torch.linalg.norm(ee - gb.goal_pose.position.reshape(-1, 3)[0]))
    qe = float(2 * torch.acos(torch.dot(q, gb.goal_pose.quaternion.reshape(-1, 4)[0]).abs().clamp(0, 1)))
    reached = pe <= S.POS_TOL_M and qe <= S.ROT_TOL_RAD
    goal_at_step = goal_idx
    switched = False
    if reached and not (args.single_pass and arrivals):
        arrivals.append({"step": i, "goal": goal_idx, "pos_err_m": round(pe, 4)})
    if not args.single_pass and (reached or (i - leg_start) >= S.LEG_MAX_STEPS):
        goal_idx = 1 - goal_idx
        gb.goal_pose.copy_(Pose.from_list(S.GOALS_WXYZ[goal_idx], ta))
        mpc.update_goal(gb)
        if MPC_OFF is not None:
            gb_off.goal_pose.copy_(Pose.from_list(S.GOALS_WXYZ[goal_idx], ta)); MPC_OFF.update_goal(gb_off)
        switches += 1
        leg_start = i
        switched = True
    t7 = pc(); _PM("goal")

    if MEFF is not None and timed and i == args.meff_switch_step:
        MEFF.set_cost(eta0=args.meff_switch_eta0); log(f"(1-3) step {i}: η0 → {args.meff_switch_eta0}, 그래프 캡처 누적 {len(CAPTURES)}")
    if MEFF is not None and timed and args.meff_log_near > 0:
        _qa = MEFF.query_all(kin.robot_spheres.reshape(-1, 4)[:MEFF_SV])
        _dd = _qa["dist"].cpu().numpy(); _j = np.unravel_index(int(np.argmin(_dd)), _dd.shape)
        NEAR_MIN.append((i, float(_dd[_j]), int(_j[0]), int(_j[1])))
        _sel = np.argwhere(_dd < args.meff_log_near)
        if len(_sel):
            _mh = _qa["mh"].cpu().numpy(); _mr = _qa["mr"].cpu().numpy(); _mu = _qa["mu"].cpu().numpy(); _u = _qa["u"].cpu().numpy(); _sg = _qa["seg"].cpu().numpy()
            for _s, _k in _sel:
                NEAR_ROWS.append((i, _s, _k, _sg[_s, _k], _dd[_s, _k], _mh[_s, _k], _mr[_s, _k], _mu[_s, _k], *_u[_s, _k]))
    if MEFF is not None and timed and args.b1_log:
        _qb = MEFF.query_all(kin.robot_spheres.reshape(-1, 4)[:MEFF_SV])
        _db = _qb["dist"]; _vb, _sb = _db.min(0)                       # 형상마다 로봇 구 전체 최솟값 (24,)
        _ar = torch.arange(_db.shape[1], device=_db.device)
        B1_ROWS.append(np.concatenate([[i], q_cmd.reshape(-1).cpu().numpy().astype(np.float64), ee.detach().reshape(-1).cpu().numpy().astype(np.float64), [float(feasible)],
                                       _vb.cpu().numpy(), _sb.cpu().numpy(), _qb["mu"][_sb, _ar].cpu().numpy(), _qb["mh"][_sb, _ar].cpu().numpy(), _qb["mr"][_sb, _ar].cpu().numpy(),
                                       [float(_qb["fb"])]]))   # (B2a) last column: executed-state nearest-u fallback count
    # (1-2d) 덤프 (검증 실행만) · GUI 선 표시
    t_mv = 0.0
    if MEFF is not None and timed and i in MEFF_DUMP:
        _sn = MEFF.snapshot(0); _B, _H = _sn["dist"].shape[:2]; _rg = np.random.default_rng(1000 + i)
        _bb = _rg.integers(0, _B, 300); _hh = _rg.integers(0, MEFF.optimizers[0][2]["Hf"], 300)
        _raw = HUMAN.last_raw if HUMAN is not None else None
        np.savez_compressed(os.path.join(os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac"), f"{TAG}_dump_{i:04d}.npz"), b=_bb, h=_hh, q_state=MEFF_QSTATE[0],
                            inv_r=RI.compute(MEFF_QSTATE[0])[1], age=MEFF_INP.age_val, tau=MEFF_INP.tau.cpu().numpy(), traj_dt=np.asarray(info["traj_dt"]), mass_on=MEFF_MASS_ON,
                            calls=_sn["calls"], R=HUMAN.R, t=HUMAN.t, **{f"h_{k}": v for k, v in (_raw or {}).items()}, human_n=HUMAN.last_pumped,
                            cost_par=MEFF.par.cpu().numpy(), cost_mode=int(MEFF.mode[0]), cost_agg=int(MEFF.agg[0]),
                            **{k: _sn[k][_bb, _hh] for k in ["dist", "shape", "seg", "u", "mh", "mr", "mu", "q", "sph", "cost", "eta", "kc", "cost_bh"] if k in _sn},
                            **({"sc_b": _bb[:20], "sc_sph": _sn["sph"][_bb[:20]], "sc_dist": _sn["dist"][_bb[:20]], "sc_vcost": _sn["vcost"][_bb[:20]], "sc_par": MEFF.sc_par.cpu().numpy(),
                                "sc_inv_dt": MEFF.inv_dt.cpu().numpy()} if MEFF.speed_cap and "vcost" in _sn and "sph" in _sn else {}))
    if VIZ is not None:   # (demo) display only, after the solver step
        VIZ.update(i, kin.robot_spheres.reshape(-1, 4)[:MEFF_SV], ee)
    if MEFF is not None and args.meff_viz in ("lines", "both") and HUMAN is not None and HUMAN.last_h is not None and i % 2 == 0:
        _tv = pc()
        _sph = kin.robot_spheres.reshape(-1, 4)[:MEFF_SV]
        _qr = MEFF.query(_sph)
        _d = _qr["dist"].cpu().numpy(); _c = _sph[:, :3].cpu().numpy(); _e = _qr["closest"].cpu().numpy()
        _sel = ([int(np.argmin(_d))] if args.meff_lines == "min" else np.argsort(_d)[: args.meff_lines_k].tolist() if args.meff_lines == "topk"
                else list(range(MEFF_SV)))
        HUMAN.set_lines(_c[_sel], _e[_sel])
        if i % 100 == 0:
            _j = int(np.argmin(_d))
            log(f"(1-2d) 가장 가까운 쌍: 로봇 구 {_j} ({RI.sphere_link[_j]}), 형상 {int(_qr['shape'][_j])}, 분절 {int(_qr['seg'][_j])}, 거리 {_d[_j]:.3f} m, "
                f"m_h {float(_qr['mh'][_j]):.2f} m_r {float(_qr['mr'][_j]):.2f} μ {float(_qr['mu'][_j]):.2f} kg")
        t_mv = (pc() - _tv) * 1000
    # 점검 계산 (--no-diag 이면 끔): 로봇 구 CPU 복사 + numpy 여유, EE 좌표 CPU 복사
    clr = float("nan")
    e = [float("nan")] * 3
    if not args.no_diag:
        if kin.robot_spheres is not None and S.HAS_MOVER:
            clr = S.sphere_box_clearance(kin.robot_spheres.reshape(-1, 4).cpu().numpy(), mp[:3], S.MOVER_DIMS)
            min_clear = min(min_clear, clr)
        e = ee.detach().cpu().tolist()
    t8 = pc()
    # 기록 준비: 관절값은 --no-diag 이면 이미 CPU 에 있는 측정값(_LAST_ST)에서, 아니면 기존처럼 GPU→CPU
    if args.no_diag:
        qv = np.asarray(_LAST_ST[0].positions).reshape(-1)[cmd_idx].tolist()
    else:
        qv = current.position.reshape(-1).cpu().tolist()
    t9 = pc(); _PM("rest")
    if PROF_ON and timed:
        PROF_ROWS.append([i] + [x for k in range(1, len(PROF_MARKS)) for x in ((PROF_MARKS[k][1] - PROF_MARKS[k - 1][1]) * 1000, (PROF_MARKS[k][2] - PROF_MARKS[k - 1][2]) * 1000)])
        if len(PROF_ROWS) == 1:
            PROF_NAMES[:] = [m[0] for m in PROF_MARKS[1:]]
    if timed:
        if i == 0:
            first_call_ms = s_dt
        else:
            solver_ms.append(s_dt)
        phys_ms.append(p_dt * 1000)
        loop_ms.append((t9 - l0) * 1000)
        tms = [(t1 - l0), (t2 - t1), (t3 - t2), (t4 - t3b), (s0 - t4), (t5 - s0), (t6 - t5), (t7 - t6), (t8 - t7), (t9 - t8),
               (t3b - t3)]
        hx = ([HT["pickup"], HT["transform"], HT["disp_instancer"], HT["tensor"], HT["new"],
               HT["mesh_xf"] + HT["mesh_usd"] + HT["mesh_rt"], HT["mesh_xf"], HT["mesh_new"]] if HT else [0.0, 0.0, 0.0, 0.0, 0, 0.0, 0.0, 0])
        hx += [t_mr, t_mv, (HT or {}).get("shapes", 0.0)]
        STEP_ROWS.append([i, round(s_dt, 4), int(feasible), attempts, int(not feasible), goal_at_step, int(switched),
                          mp[0], mp[1], mp[2], e[0], e[1], e[2], round(pe, 5), round(qe, 5), round(clr, 5),
                          round(p_dt * 1000, 4), round(loop_ms[-1], 4)] + [round(x, 6) for x in qv]
                         + [round(x * 1000, 4) for x in tms] + [round(float(x), 4) for x in hx])
    return s_dt, pe, mp


VIZ = None
if args.viz_eta_shell == "on" or args.viz_trail == "on" or args.viz_ghost or args.viz_record or args.viz_hide_com == "on" or args.viz_zone or args.viz_zone_seq:
    sys.path.insert(0, os.path.expanduser("~/effmass_mpc/goal1/scripts")); import g1_viz as G1V
    VIZ = G1V.Viz(world.stage, args, MEFF, HUMAN, TAG, log, MEFF_SV if MEFF is not None else 0)
for i in range(args.steps):
    if i == 1:
        S.COUNTER_TAG[0] = "loop"
    s_dt, pe, mp = one_step(i)
    if i % 100 == 0 or i == args.steps - 1:
        v = None if args.no_diag else S.proc_vram_mib(PID)
        if v is not None:
            vram.append(v)
        log(f"step {i:5d}  solver {s_dt:7.2f} ms  goal {goal_idx}  pos_err {pe:.3f}  mover_x {mp[0]:.3f}  "
            f"arrivals {len(arrivals)}  holds {holds}  proc_vram {v} MiB  captures {len(CAPTURES)}")
loop_wall = time.perf_counter() - loop_t0
if VIZ is not None:
    VIZ_STATS = VIZ.finish(os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac"))
sim_elapsed = world.current_time - sim_t0
if args.ros:
    ROS_LOOP_END_NS = time.time_ns()
    open(args.done_file, "w").write(str(ROS_LOOP_END_NS))
    time.sleep(1.0)
    ros_ex.shutdown()
    ros_node.destroy_node()
    rclpy.shutdown()
    import csv as _csv

    with open(args.ros_out + "_isaac_recv_pose.csv", "w", newline="") as f:
        w = _csv.writer(f); w.writerow(["kind", "seq", "stamp_ns", "t_recv_ns", "ok"]); w.writerows(ROS_RECV)
    with open(args.ros_out + "_isaac_sent_js.csv", "w", newline="") as f:
        w = _csv.writer(f); w.writerow(["seq", "t_send_ns"] + [f"p{k}" for k in range(9)]); w.writerows(ROS_SENT)
    HUMAN_FINAL = None
    if HUMAN is not None:
        HUMAN_FINAL = HUMAN.final_check()
        HUMAN_MESH_FINAL = HUMAN.mesh_final_check()
        log(f"사람: 메시 최종 확인 {HUMAN_MESH_FINAL}, 실시간 메시 오프셋 {getattr(HUMAN, 'mesh_live_info', None)}")
        HUMAN.save(args.ros_out)
        log(f"사람: HumanPose 수신 {len(HUMAN.pose_rec)}, HumanDynamicsState 수신 {len(HUMAN.dyn_rec)}, 처리 {len(HUMAN.upd_rec)}, "
            f"텐서 최종 확인 {HUMAN_FINAL}")
    json.dump(dict(loop_start_ns=ROS_LOOP_START_NS, loop_end_ns=ROS_LOOP_END_NS, warmup_start_ns=WARM_START_NS,
                   warmup_end_ns=WARM_END_NS, human_first_dyn_ns=HUMAN_FIRST_DYN_NS, human_final=HUMAN_FINAL, human_mesh_final=HUMAN_MESH_FINAL if HUMAN is not None else None, js_sent=len(ROS_SENT),
                   pose_recv_total=len(ROS_RECV), executor="SingleThreadedExecutor.spin() in daemon thread",
                   publish="main loop, once per physics step"), open(args.ros_out + "_isaac_meta.json", "w"))
    log(f"ROS 종료: JointState 발행 {len(ROS_SENT)}, HumanPose 수신(전체) {len(ROS_RECV)}")

sm = sorted(solver_ms)
p95 = sm[max(0, int(round(0.95 * len(sm))) - 1)]
lm = sorted(loop_ms)
lp95 = lm[max(0, int(round(0.95 * len(lm))) - 1)]
captures_by_tag = {}
for c in CAPTURES:
    captures_by_tag[c["tag"]] = captures_by_tag.get(c["tag"], 0) + 1
v_end = S.proc_vram_mib(PID)
vram.append(v_end)
res = {
    "tag": TAG, "mode": "GUI" if GUI else "headless", "start": STAMP, "steps": args.steps,
    "solver": info, "build_s": round(t_build, 3),
    "first_call_ms": round(first_call_ms, 2),
    "solver_ms_mean": round(statistics.mean(solver_ms), 3), "solver_ms_p95": round(p95, 3),
    "solver_ms_max": round(sm[-1], 3),
    "solver_steps_over_16_7ms": int(sum(1 for x in solver_ms if x > 1000 / 60)),
    "physics_step_ms_mean": round(statistics.mean(phys_ms), 3),
    "loop_ms_mean": round(statistics.mean(loop_ms), 3), "loop_ms_p95": round(lp95, 3),
    "sim_time_s": round(sim_elapsed, 3), "loop_wall_s": round(loop_wall, 3),
    "realtime_factor": round(sim_elapsed / loop_wall, 3),
    "infeasible_steps": infeasible, "hold_steps": holds, "goal_switches": switches,
    "arrivals": len(arrivals), "arrival_list": arrivals[:20],
    "min_clearance_robot_to_mover_m": None if args.no_diag else round(min_clear, 4),
    "torch_max_allocated_mib": round(torch.cuda.max_memory_allocated() / 2**20, 1),
    "torch_max_reserved_mib": round(torch.cuda.max_memory_reserved() / 2**20, 1),
    "proc_vram_mib_max": max([v for v in vram if v is not None], default=None),
    "proc_vram_mib_samples": vram,
    "graph_captures_by_stage": captures_by_tag,
    "graph_capture_callers": sorted({c["caller"] for c in CAPTURES}),
}
out = os.path.expanduser("~/effmass_mpc/env_check/step2")
os.makedirs(out, exist_ok=True)
import csv

steps_path = os.path.join(out, f"{TAG}_steps_{STAMP}.csv")
with open(steps_path, "w", newline="") as f:
    wr = csv.writer(f)
    wr.writerow(ROW_FIELDS)
    wr.writerows(STEP_ROWS)
res["scene"] = SCENE_INFO
if PROF_ROWS:
    _PR = np.array([r for r in PROF_ROWS if len(r) == 1 + 2 * len(PROF_NAMES)], dtype=float)
    res["loop_prof"] = dict(sections=PROF_NAMES, n=len(_PR), wall_ms_mean={n: float(_PR[:, 1 + 2 * k].mean()) for k, n in enumerate(PROF_NAMES)},
                            cpu_ms_mean={n: float(_PR[:, 2 + 2 * k].mean()) for k, n in enumerate(PROF_NAMES)},
                            wall_ms_p95={n: float(np.percentile(_PR[:, 1 + 2 * k], 95)) for k, n in enumerate(PROF_NAMES)})
    np.save(os.path.expanduser(f"~/effmass_mpc/goal1/logs/meff_isaac/{TAG}_loopprof.npy"), _PR)
    log("(1-2d) loop_prof " + json.dumps(res["loop_prof"], ensure_ascii=False)[:2500])
if args.human_source in ("inertia", "precomputed") or args.meff == "on":
    _tmr = [r[ROW_FIELDS.index("t_meff_r_ms")] for r in STEP_ROWS]
    res["meff"] = dict(meff=args.meff, human_source=args.human_source, impl=args.meff_impl, dir=args.meff_dir, u_def=args.u_def, single_pass=args.single_pass, scene_json=args.scene_json, motion=args.meff_motion, hfront=args.meff_hfront,
                       viz=args.meff_viz, import_check=G1_IMPORT if args.ros else None, spheres=MEFF_SV,
                       calls=int(MEFF.snapshot(0)["calls"]) if MEFF else None, channel_pointers_unchanged=(MEFF.pointers() == MEFF.addr0) if MEFF else None,
                       inputs_pointers_unchanged=MEFF_INP.pointers() == MEFF_INP.addr0, optimizers=MEFF.info if MEFF else None,
                       mass_on_zero=np.nonzero(MEFF_MASS_ON == 0)[0].tolist() if MEFF else None, robot_inv_ms_mean=float(np.mean(_tmr)) if _tmr else None,
                       robot_inv_ms_p95=float(np.percentile(_tmr, 95)) if _tmr else None, human_set=MEFF_INP.n_human, robot_set=MEFF_INP.n_robot,
                       dumps=sorted(MEFF_DUMP), robot_inv_impl="numpy RobotInertiaNP" if MEFF else None)
    if MEFF is not None:
        res["meff"]["cost"] = MEFF.cost_state()
    if COSTAGG is not None and "pn" in COSTAGG:
        _sp = MEFF.optimizers[0][1]["sph_ref"]; log(f"(B2a-2) rollout sphere tensor {tuple(_sp.shape)}, radii {sorted(set(np.round(_sp[0, 0, :, 3].cpu().numpy(), 4).tolist()))[:12]}")
    if COSTAGG is not None:
        np.savez(os.path.expanduser(f"~/effmass_mpc/goal1/logs/meff_isaac/{TAG}_costagg.npz"), n=COSTAGG["n"].cpu().numpy(), s=COSTAGG["s"].cpu().numpy(), fb=COSTAGG["fb"].cpu().numpy(),
                 shape_segment=MEFF_INP.seg.cpu().numpy(), **({"plate_n": COSTAGG["pn"].cpu().numpy(), "plate_min": COSTAGG["pmin"].cpu().numpy(), "plate_names": np.array([c.name for c in PLATES])} if "pn" in COSTAGG else {}),
                 **({"speed_n": COSTAGG["vn"].cpu().numpy(), "speed_sum": COSTAGG["vs"].cpu().numpy()} if "vn" in COSTAGG else {}), note="last MPPI iteration of each step; n = count of (particle, horizon, sphere) with cost > 0 per selected shape, s = cost sum")
    if B1_ROWS:
        np.savez(os.path.expanduser(f"~/effmass_mpc/goal1/logs/meff_isaac/{TAG}_b1.npz"), rows=np.array(B1_ROWS, dtype=np.float64), sphere_link=np.array(RI.sphere_link),
                 cost_par=MEFF.par.cpu().numpy(), cost_mode=int(MEFF.mode[0]), mppi_seed=args.mppi_seed)
    if NEAR_MIN:
        _NM = np.array(NEAR_MIN, dtype=float)
        np.savez(os.path.expanduser(f"~/effmass_mpc/goal1/logs/meff_isaac/{TAG}_near.npz"), near=np.array(NEAR_ROWS, dtype=float).reshape(-1, 11), step_min=_NM,
                 sphere_link=np.array(RI.sphere_link), thr=args.meff_log_near)
        res["meff"]["near"] = dict(min_dist_m=float(_NM[:, 1].min()), min_step=int(_NM[np.argmin(_NM[:, 1]), 0]), rows=len(NEAR_ROWS), thr=args.meff_log_near)
    if PAIRED:
        _P = np.array(PAIRED, dtype=float); _dl = _P[:, 1] - _P[:, 2]
        res["meff"]["paired"] = dict(n=len(_P), on_mean=float(_P[:, 1].mean()), off_mean=float(_P[:, 2].mean()), on_median=float(np.median(_P[:, 1])),
                                     off_median=float(np.median(_P[:, 2])), on_p95=float(np.percentile(_P[:, 1], 95)), off_p95=float(np.percentile(_P[:, 2], 95)),
                                     delta_mean=float(_dl.mean()), delta_median=float(np.median(_dl)), delta_p95=float(np.percentile(_dl, 95)))
        np.save(os.path.expanduser(f"~/effmass_mpc/goal1/logs/meff_isaac/{TAG}_paired.npy"), _P)
    log("(1-2d) meff " + json.dumps(res["meff"], ensure_ascii=False, default=str)[:1500])
res["no_diag"] = bool(args.no_diag)
res["warmup_n"] = args.warmup
res["warmup_calls"] = WARM
res["steps_csv"] = steps_path
res["mppi_seed_info"] = MPPI_SEED_INFO
log(f"step 기록 {len(STEP_ROWS)}행 → {steps_path}")
path = os.path.join(out, f"{TAG}_result_{STAMP}.json")
with open(path, "w") as f:
    json.dump(res, f, indent=2, ensure_ascii=False)
log("===== RESULT =====")
for k in ["mode", "build_s", "first_call_ms", "solver_ms_mean", "solver_ms_p95", "solver_ms_max",
          "solver_steps_over_16_7ms", "physics_step_ms_mean", "loop_ms_mean", "loop_ms_p95", "sim_time_s",
          "loop_wall_s", "realtime_factor", "infeasible_steps", "hold_steps", "goal_switches", "arrivals",
          "min_clearance_robot_to_mover_m", "torch_max_allocated_mib", "torch_max_reserved_mib",
          "proc_vram_mib_max", "graph_captures_by_stage"]:
    log(f"{k:32s}: {res[k]}")
log(f"saved {path}")

if GUI and args.hold != 0:
    log("GUI 유지: " + ("창을 닫을 때까지" if args.hold < 0 else f"{args.hold:.0f}초") + " (MPC 계속 동작)")
    h0 = time.perf_counter()
    i = args.steps
    while simulation_app.is_running() and (args.hold < 0 or time.perf_counter() - h0 < args.hold):
        one_step(i, timed=False)
        i += 1
world.stop()
simulation_app.close()

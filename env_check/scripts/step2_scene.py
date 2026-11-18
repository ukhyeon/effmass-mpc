"""step2_scene.py — 측정 A/B 공통 장면·MPC 설정 (프로토타입 franka-depth-mppi HIGH 조건과 같은 값)

근거 (ref/franka-depth-mppi):
  - experiment/hand_experiment.py 100행: MpcSolverConfig.load_from_robot_config(
        use_cuda_graph=True, use_cuda_graph_metrics=True, use_cuda_graph_full_step=False,
        self_collision_check=True, collision_checker_type=PRIMITIVE(카메라 지도 없을 때)/BLOX,
        use_mppi=True, use_lbfgs=False, use_es=False, store_rollouts=True, step_dt=model_dt)
  - hand_experiment.py 91행: robot_cfg['kinematics']['collision_sphere_buffer']=.02
  - hand_experiment.py 96행: collision_wall.yml 의 cuboid 2개, 첫 번째(table) z -= 0.01
  - hand_experiment.py 298행: mpc.step(current, max_attempts=2), 앞뒤 torch.cuda.synchronize
  - evidence/HIGH/runtime.json: goals_wxyz, physics_dt=1/60, seed 0, 허용오차 0.01 m / 0.05 rad
  - evidence/HIGH/control-settings.json: model_base_dt=1/60, trajectory_dt(30개, 비균일), filter 0.1/0.1
  - evidence/HIGH/cost-connection.json: ParallelMPPI shape [400, 30], environment activation 0.025, weight 1e5
  - 후보 400 / horizon 30 / n_iters 5 / dt_traj_params 는 cuRobo content/configs/task/particle_mpc.yml 기본값
    (프로토타입은 이 파일을 바꾸지 않음: docs/reproduction.md "cuRobo core 로컬 변경은 주석뿐")

이번 측정과 프로토타입의 차이 (보고서에 명시):
  - nvblox(BLOX) 지도 대신 PRIMITIVE(cuboid) 충돌 검사만 사용 (2단계는 nvblox 미빌드)
  - 프로토타입의 target_margin 프록시 cuboid 64개 대신, 정적 cuboid 4개 + 움직이는 cuboid 1개
"""
import os
import math

MODEL_DT = 1.0 / 60.0
PHYSICS_DT = 1.0 / 60.0
SEED = 0
GOALS_WXYZ = [
    [0.4, -0.5, 0.4, 0.0, 1.0, 0.0, 0.0],
    [0.4, 0.5, 0.4, 0.0, 1.0, 0.0, 0.0],
]
POS_TOL_M = 0.01
ROT_TOL_RAD = 0.05
COLLISION_SPHERE_BUFFER = 0.02
MAX_ATTEMPTS = 2
LEG_MAX_STEPS = 400  # 도달하지 못해도 이 스텝 수가 지나면 다음 목표로 (왕복 보장용)

# 움직이는 장애물: 두 목표 사이 경로(x≈0.4, z≈0.4) 앞쪽에서 x 방향으로 왕복
MOVER_NAME = "mover"
MOVER_DIMS = [0.08, 0.08, 0.08]
MOVER_CENTER = [0.65, 0.0, 0.45]
MOVER_AMP_X = 0.15
MOVER_PERIOD_S = 6.0
FAR_POSE = [10.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]


SCENE = "v1"
SCENES = {
    # v1: 1차 측정(4.2절) 장면. 값 변경 없음.
    "v1": dict(MOVER_CENTER=[0.65, 0.0, 0.45], MOVER_AMP_X=0.15, MOVER_PERIOD_S=6.0, MOVER_DIMS=[0.08, 0.08, 0.08]),
    # v2 (2026-11-15, step2_report.md 4.6절): 가장 가까운 지점을 x 0.50 → 0.65 로. 장애물 x 범위 0.61~0.89(크기 포함)가
    # block_low(x 0.50~0.60, z 0~0.30) 위를 지나지 않게 해서, 회피하던 로봇이 block_low 쪽으로 밀려 갇히는 것을 막는다.
    # 정지 자세(두 목표) 최소 여유 +0.298 / +0.390 m, v1 경로 자세와는 −0.044 m(겹침 → 회피 유발 유지). 나머지 값은 v1 과 같다.
    "v2": dict(MOVER_CENTER=[0.75, 0.0, 0.45], MOVER_AMP_X=0.10, MOVER_PERIOD_S=6.0, MOVER_DIMS=[0.08, 0.08, 0.08]),
}


# (goal1 1-2a-2) 사람이 들어갈 장면 "g1": 2단계 table 판·움직이는 장애물·정적 블록 없음.
#   충돌: 로봇 탁자 0.6 x 0.6 x 0.9 m (윗면 z 0 = 베이스, 4단계 시각용 탁자와 같음) + 바닥(윗면 z -0.9). 사람은 MPC 비용·충돌에 넣지 않음.
#   목표 자세: ~/effmass_mpc/goal1/scene_g1.json (g1_goal_search.py 결과)
HAS_MOVER = True
GROUND_Z = 0.0
G1_SCENE_JSON = os.path.expanduser("~/effmass_mpc/goal1/scene_g1.json")
G2_SCENE_JSON = None   # (B2a) set by step2b --scene-json
G1_TABLE = dict(name="robot_table", pose=[0.0, 0.0, -0.45, 1, 0, 0, 0], dims=[0.6, 0.6, 0.9])
G1_FLOOR = dict(name="floor", pose=[0.0, 0.0, -0.95, 1, 0, 0, 0], dims=[4.0, 4.0, 0.1])


def set_scene(name: str):
    """장면 선택. 모듈 전역 MOVER_* 값을 바꾼다. build_world_config() 전에 호출."""
    global SCENE, MOVER_CENTER, MOVER_AMP_X, MOVER_PERIOD_S, MOVER_DIMS, HAS_MOVER, GROUND_Z, GOALS_WXYZ
    if name == "g2":   # (B2a) same table and floor as g1; goals and start configuration from G2_SCENE_JSON
        import json
        SCENE = "g2"; HAS_MOVER = False; GROUND_Z = -0.9
        js = json.load(open(G2_SCENE_JSON)); GOALS_WXYZ = [list(map(float, g)) for g in js["goals_wxyz"]]
        return dict(scene="g2", table=G1_TABLE, floor=G1_FLOOR, goals_file=G2_SCENE_JSON, goals_wxyz=GOALS_WXYZ, q_start=js.get("q_start"), path=js.get("path"))
    if name == "g1":
        import json
        SCENE = "g1"; HAS_MOVER = False; GROUND_Z = -0.9
        info = dict(scene="g1", table=G1_TABLE, floor=G1_FLOOR, goals_file=None)
        if os.path.exists(G1_SCENE_JSON):
            GOALS_WXYZ = [list(map(float, g)) for g in json.load(open(G1_SCENE_JSON))["goals_wxyz"]]
            info["goals_file"] = G1_SCENE_JSON
        info["goals_wxyz"] = GOALS_WXYZ
        return info
    cfg = SCENES[name]
    SCENE = name
    MOVER_CENTER = list(cfg["MOVER_CENTER"])
    MOVER_AMP_X = float(cfg["MOVER_AMP_X"])
    MOVER_PERIOD_S = float(cfg["MOVER_PERIOD_S"])
    MOVER_DIMS = list(cfg["MOVER_DIMS"])
    return dict(scene=SCENE, mover_center=MOVER_CENTER, mover_amp_x=MOVER_AMP_X,
                mover_period_s=MOVER_PERIOD_S, mover_dims=MOVER_DIMS)


def scene_from_argv(argv):
    name = argv[argv.index("--scene") + 1] if "--scene" in argv else "v1"
    return set_scene(name)


def count_step_once(mpc):
    """MpcSolver.step 내부의 _step_once 호출 수를 세는 래퍼 (설정은 바꾸지 않음). 반환: 카운터 리스트."""
    counter = [0]
    orig = mpc._step_once

    def counted(*a, **k):
        counter[0] += 1
        return orig(*a, **k)

    mpc._step_once = counted
    return counter


STEP_FIELDS = ["step", "solver_ms", "feasible", "attempts", "hold", "goal_idx", "goal_switched",
               "mover_x", "mover_y", "mover_z", "ee_x", "ee_y", "ee_z", "pos_err_m", "rot_err_rad",
               "min_clear_mover_m", "physics_ms", "loop_ms",
               "q0", "q1", "q2", "q3", "q4", "q5", "q6"]


def mover_pose(t_s: float):
    x = MOVER_CENTER[0] + MOVER_AMP_X * math.sin(2.0 * math.pi * t_s / MOVER_PERIOD_S)
    return [x, MOVER_CENTER[1], MOVER_CENTER[2], 1.0, 0.0, 0.0, 0.0]


def build_world_config():
    """table + wall (collision_wall.yml, 프로토타입과 동일) + 정적 블록 2개 + mover 1개 = cuboid 5개"""
    from curobo.geom.types import Cuboid, WorldConfig
    from curobo.util_file import get_world_configs_path, join_path, load_yaml

    if SCENE in ("g1", "g2"):
        extra = []
        if SCENE == "g2":   # (B2a-2) optional plates from the scene json: [{"name", "pose" [x y z qw qx qy qz], "dims"}]
            import json
            extra = [Cuboid(name=c["name"], pose=list(map(float, c["pose"])), dims=list(map(float, c["dims"]))) for c in json.load(open(G2_SCENE_JSON)).get("plates", [])]
        return WorldConfig(cuboid=[Cuboid(**G1_TABLE), Cuboid(**G1_FLOOR)] + extra)
    fixed = WorldConfig.from_dict(load_yaml(join_path(get_world_configs_path(), "collision_wall.yml")))
    fixed.cuboid[0].pose[2] -= 0.01
    cuboids = list(fixed.cuboid[:2])
    cuboids += [
        Cuboid(name="block_low", pose=[0.55, 0.0, 0.15, 1, 0, 0, 0], dims=[0.10, 0.10, 0.30]),
        Cuboid(name="block_high", pose=[0.30, 0.0, 0.85, 1, 0, 0, 0], dims=[0.10, 0.40, 0.10]),
        Cuboid(name=MOVER_NAME, pose=mover_pose(0.0), dims=list(MOVER_DIMS)),
    ]
    return WorldConfig(cuboid=cuboids)


FR3_CFG = os.path.expanduser("~/effmass_mpc/goal1/fr3/fr3.yml")
FR3V21_CFG = os.path.expanduser("~/effmass_mpc/goal1/fr3v2_1/fr3v2_1.yml")   # (goal1 모델 결정 2026-11-16) 기준 모델
FR3_USD_REL = "/Isaac/Robots/Franka/FR3/fr3.usd"


def robot_name():
    """(goal1) 환경 변수 STEP_ROBOT=panda|fr3|fr3v2_1 (기본 panda: 환경 점검 단계와 같음). fr3 = v1 USD 비교용, fr3v2_1 = 기준 모델"""
    return os.environ.get("STEP_ROBOT", "panda")


def build_robot_cfg():
    from curobo.util_file import get_robot_configs_path, join_path, load_yaml

    if robot_name() == "fr3":
        robot_cfg = load_yaml(FR3_CFG)["robot_cfg"]
    elif robot_name() == "fr3v2_1":
        robot_cfg = load_yaml(FR3V21_CFG)["robot_cfg"]
    else:
        robot_cfg = load_yaml(join_path(get_robot_configs_path(), "franka.yml"))["robot_cfg"]
    robot_cfg["kinematics"]["collision_sphere_buffer"] = COLLISION_SPHERE_BUFFER
    return robot_cfg


# (goal1 결정 1, 2026-11-16) FR3 팔 관절(joint1~7) 구동 이득: 이 실행의 장면(메모리)에서만 덮어씀, USD 파일은 바꾸지 않음. 손가락 관절은 그대로.
#   기본값 = 환경 점검 Panda 변환 모델(cuRobo helper URDF 변환)의 실제 USD 구동 속성값 stiffness 625, damping 10
#   (USD DriveAPI 단위: 각도 기준, = 35,810 / 573 N·m/rad). 목표 2의 2-3에서 실기 추종에 맞춰 다시 정함.
#   환경 변수 STEP_FR3_ARM_GAINS="stiffness,damping" 로 바꿈 ("usd" 면 덮어쓰지 않고 USD 기본값 60000,6000 사용)
FR3_ARM_GAINS_DEFAULT = "625,10"
FR3_GAINS_APPLIED = {}


def set_fr3_arm_gains(stage, root, prefix="fr3"):
    """prefix_joint1~7 구동 이득을 장면(메모리)에서만 덮어씀. 바꾸기 전 값(변환·USD 가 실제로 넣은 값)을 FR3_GAINS_APPLIED 에 기록"""
    from pxr import UsdPhysics
    g = os.environ.get("STEP_FR3_ARM_GAINS", FR3_ARM_GAINS_DEFAULT)
    FR3_GAINS_APPLIED.clear(); FR3_GAINS_APPLIED["setting"] = g
    kp_kd = None if g == "usd" else [float(x) for x in g.split(",")]
    for i in range(1, 8):
        prim = next((p for p in stage.Traverse() if p.GetPath().pathString.startswith(root) and p.GetName() == f"{prefix}_joint{i}"), None)
        d = UsdPhysics.DriveAPI.Get(prim, "angular")
        before = (d.GetStiffnessAttr().Get(), d.GetDampingAttr().Get(), d.GetMaxForceAttr().Get())
        FR3_GAINS_APPLIED[f"{prefix}_joint{i}"] = dict(before=before, after=tuple(kp_kd) if kp_kd else "그대로")
        if kp_kd:
            d.GetStiffnessAttr().Set(kp_kd[0]); d.GetDampingAttr().Set(kp_kd[1])


def apply_arm_gains_runtime(robot):
    """world.reset() 직후 호출. FR3 계열 팔 7관절 이득을 articulation(실행 중)에 직접 설정 (USD 단위 값 × 180/π = view 단위).
    URDF 변환 경로는 helper 가 로봇을 장면에 넣은 뒤라 USD 속성 덮어쓰기가 물리에 반영되지 않아서 (fr3v2_1 진단, 2026-11-16) 이 방식을 씀."""
    import numpy as np
    if robot_name() not in ("fr3", "fr3v2_1"):
        return None
    g = os.environ.get("STEP_FR3_ARM_GAINS", FR3_ARM_GAINS_DEFAULT)
    pre = robot_name()
    arm = [robot.get_dof_index(f"{pre}_joint{i}") for i in range(1, 8)]
    kp0, kd0 = [np.asarray(x).reshape(-1) for x in robot._articulation_view.get_gains()]
    rec = dict(setting=g, view_before_kp=[round(float(kp0[i]), 3) for i in arm], view_before_kd=[round(float(kd0[i]), 3) for i in arm])
    if g != "usd":
        kp, kd = [float(x) * 180.0 / np.pi for x in g.split(",")]
        robot._articulation_view.set_gains(kps=np.full((1, 7), kp), kds=np.full((1, 7), kd), joint_indices=np.array(arm))
    kp1, kd1 = [np.asarray(x).reshape(-1) for x in robot._articulation_view.get_gains()]
    rec.update(view_after_kp=[round(float(kp1[i]), 3) for i in arm], view_after_kd=[round(float(kd1[i]), 3) for i in arm])
    FR3_GAINS_APPLIED["runtime"] = rec
    return rec


def add_isaac_robot(robot_cfg, world):
    """Isaac 로봇: panda = cuRobo helper 의 URDF 변환 (환경 점검과 같음), fr3 = Isaac 4.5 공식 FR3 USD (/World/fr3)"""
    if robot_name() == "fr3":
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.storage.native import get_assets_root_path
        url = get_assets_root_path() + FR3_USD_REL
        add_reference_to_stage(url, "/World/fr3")
        set_fr3_arm_gains(world.stage, "/World/fr3")
        return world.scene.add(Robot(prim_path="/World/fr3", name="fr3")), "/World/fr3"
    from helper import add_robot_to_scene
    robot, path = add_robot_to_scene(robot_cfg, world)
    if robot_name() == "fr3v2_1":   # cuRobo helper URDF 변환 (Panda 와 같은 방식) 뒤 팔 이득 기록·맞춤 (결정 1)
        set_fr3_arm_gains(world.stage, path, prefix="fr3v2_1")
    return robot, path


POSE_ROT_W = None        # (T1) None = particle_mpc.yml pose_cfg.weight[0] (60). float = orientation weight of the pose cost
MPPI_SEED = None          # (B1) None = particle_mpc.yml 그대로. 정수 = override_particle_file 로 mppi.seed·sample_params.seed 덮어씀
MPPI_OVERRIDE_DIR = "/home/robotics/effmass_mpc/goal1/b1/mppi_override"


MPPI_SIZE = None   # (overnight2) None = particle_mpc.yml; dict(num_particles=, n_iters=) overrides (solve-time study only)


def mppi_override_file():
    if MPPI_SEED is None and not MPPI_SIZE:
        return None
    import os
    import yaml
    os.makedirs(MPPI_OVERRIDE_DIR, exist_ok=True)
    d = {"mppi": {}}
    if MPPI_SEED is not None:
        d["mppi"].update(seed=int(MPPI_SEED), sample_params={"seed": int(MPPI_SEED)})
    if MPPI_SIZE:
        d["mppi"].update({k: int(v) for k, v in MPPI_SIZE.items()})
        if "n_iters" in MPPI_SIZE:
            d["mppi"]["cold_start_n_iters"] = int(MPPI_SIZE["n_iters"])
    nm = f"mppi_seed_{int(MPPI_SEED)}" if MPPI_SEED is not None else "mppi"
    nm += "".join(f"_{k}{int(v)}" for k, v in sorted((MPPI_SIZE or {}).items()))
    f = os.path.join(MPPI_OVERRIDE_DIR, f"{nm}.yml")
    yaml.safe_dump(d, open(f, "w"))
    return f


def build_mpc(robot_cfg, world_cfg):
    from curobo.geom.sdf.world import CollisionCheckerType
    from curobo.wrap.reacher.mpc import MpcSolver, MpcSolverConfig

    cfg = MpcSolverConfig.load_from_robot_config(
        robot_cfg,
        world_cfg,
        use_cuda_graph=True,
        use_cuda_graph_metrics=True,
        use_cuda_graph_full_step=False,
        self_collision_check=True,
        collision_checker_type=CollisionCheckerType.PRIMITIVE,
        use_mppi=True,
        use_lbfgs=False,
        use_es=False,
        store_rollouts=True,
        step_dt=MODEL_DT,
        override_particle_file=mppi_override_file(),   # (B1) None 이면 기존과 같음
    )
    m = MpcSolver(cfg)
    if POSE_ROT_W is not None:   # (T1) pose cost orientation weight (particle_mpc.yml pose_cfg.weight[0], default 60); set in place before any graph capture
        rs = {id(r): r for r in list(m.solver.get_all_rollout_instances()) + [m.rollout_fn]}
        for r in rs.values():
            r.goal_cost.weight[0] = float(POSE_ROT_W); r.goal_cost._weight[0] = float(POSE_ROT_W)
    return m


def pose_weights(m):
    """(T1) pose cost weights [rot, pos, rot alpha, pos alpha] of every rollout instance (to log and check)"""
    rs = {id(r): r for r in list(m.solver.get_all_rollout_instances()) + [m.rollout_fn]}
    return [r.goal_cost.weight.cpu().numpy().round(4).tolist() for r in rs.values()]


def describe_mpc(mpc):
    """실행 중인 solver 에서 실제 값을 읽어 프로토타입 설정과 대조할 수 있게 한다."""
    opt = mpc.solver.optimizers[0]
    dyn = mpc.rollout_fn.dynamics_model
    info = {
        "optimizer_type": type(opt).__name__,
        "num_optimizers": len(mpc.solver.optimizers),
        "total_num_particles": int(opt.total_num_particles),
        "horizon": int(opt.horizon),
        "n_iters": int(opt.n_iters),
        "cold_start_n_iters": int(getattr(opt, "cold_start_n_iters", -1)),
        "use_cuda_graph": bool(opt.use_cuda_graph),
        "traj_dt": [round(float(x), 6) for x in dyn.traj_dt.reshape(-1).cpu().tolist()],
        "joint_names": list(mpc.joint_names),
    }
    try:
        cc = mpc.rollout_fn.primitive_collision_cost
        info["primitive_collision_activation_m"] = float(cc.activation_distance.reshape(-1)[0])
        info["primitive_collision_weight"] = float(cc.weight.reshape(-1)[0])
    except Exception as e:  # noqa
        info["primitive_collision_cost_read_error"] = repr(e)
    return info


def box_signed_distance(points_xyz, box_center, box_dims):
    """축 정렬 박스까지 부호 거리 (numpy, 독립 검증용). 음수 = 내부."""
    import numpy as np

    p = np.asarray(points_xyz, dtype=np.float64) - np.asarray(box_center, dtype=np.float64)
    h = np.asarray(box_dims, dtype=np.float64) / 2.0
    q = np.abs(p) - h
    outside = np.linalg.norm(np.maximum(q, 0.0), axis=-1)
    inside = np.minimum(np.max(q, axis=-1), 0.0)
    return outside + inside


def sphere_box_clearance(spheres_xyzr, box_center, box_dims):
    """로봇 구(반지름>0)와 박스 사이 최소 여유 거리 (음수 = 침투)."""
    import numpy as np

    s = np.asarray(spheres_xyzr, dtype=np.float64).reshape(-1, 4)
    s = s[s[:, 3] > 0]
    d = box_signed_distance(s[:, :3], box_center, box_dims) - s[:, 3]
    return float(d.min())


def install_graph_capture_counter(torch):
    """torch.cuda.CUDAGraph.capture_begin 호출을 모두 기록한다 (cuRobo 내부 어느 graph 든)."""
    log = []
    orig_cls = torch.cuda.CUDAGraph

    class CountingCUDAGraph(orig_cls):
        def capture_begin(self, *a, **k):
            import traceback

            caller = "?"
            for fr in reversed(traceback.extract_stack()[:-1]):
                if "/curobo/" in fr.filename:
                    caller = f"{fr.filename.split('/curobo/')[-1]}:{fr.lineno}:{fr.name}"
                    break
            log.append({"caller": caller, "tag": COUNTER_TAG[0]})
            return super().capture_begin(*a, **k)

    torch.cuda.CUDAGraph = CountingCUDAGraph
    return log


COUNTER_TAG = ["setup"]  # 현재 단계 표시 (setup / step_0 / loop / probe)


def proc_vram_mib(pid):
    """nvidia-smi 기준 해당 프로세스의 VRAM (MiB). 실패 시 None."""
    import subprocess

    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10,
        ).stdout
        for line in out.strip().splitlines():
            p, m = [x.strip() for x in line.split(",")]
            if int(p) == int(pid):
                return int(m)
    except Exception:
        return None
    return None

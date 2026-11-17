"""step2_import_check.py — cuRobo 설치 후 import 확인 (run_isaac.sh 로 실행)

  사용법: run_isaac.sh step2_import_check.py [--with-kit]
    인자 없음  : SimulationApp 없이
    --with-kit : SimulationApp(headless) 를 먼저 띄운 뒤 같은 확인

  출력: 패키지별 버전과 실제 import 된 파일 경로, cuRobo CUDA 확장 5개 로드, warp 초기화,
        torch.compile 사용 여부 (cuRobo 판단 함수 결과와 MpcSolver 생성 중 dynamo/inductor 호출 여부)
"""
import importlib
import importlib.metadata as md
import json
import os
import sys
import time

WITH_KIT = "--with-kit" in sys.argv
print(f"[import_check] mode={'with SimulationApp' if WITH_KIT else 'no SimulationApp'}", flush=True)
if WITH_KIT:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    print("[import_check] SimulationApp ready", flush=True)

HOME = os.path.expanduser("~")

# torch.compile 호출 감시: cuRobo 데코레이터는 import 시점에 정해지므로 cuRobo import 전에 건다
import torch

compile_calls = {"torch.compile": 0}
_orig_compile = torch.compile


def _counting_compile(*a, **k):
    compile_calls["torch.compile"] += 1
    return _orig_compile(*a, **k)


torch.compile = _counting_compile



def where(path):
    if path is None:
        return None
    return path.replace(HOME, "~")


rows = []
PKGS = [
    ("curobo", "nvidia_curobo"),
    ("warp", "warp-lang"),
    ("trimesh", "trimesh"),
    ("tifffile", "tifffile"),
    ("lazy_loader", "lazy_loader"),
    ("skimage", "scikit-image"),
    ("triton", "triton"),
    ("yourdfpy", "yourdfpy"),
    ("torch", "torch"),
    ("numpy", "numpy"),
    ("scipy", "scipy"),
    ("packaging", "packaging"),
    ("imageio", "imageio"),
    ("h11", "h11"),
]
for mod, dist in PKGS:
    r = {"module": mod, "dist": dist}
    try:
        m = importlib.import_module(mod)
        r["__version__"] = str(getattr(m, "__version__", getattr(getattr(m, "config", None), "version", "?")))
        r["__file__"] = where(getattr(m, "__file__", None))
    except Exception as e:
        r["import_error"] = f"{type(e).__name__}: {e}"
    try:
        d = md.distribution(dist)
        r["dist_version"] = d.version
        r["dist_location"] = where(str(d.locate_file("")))
    except Exception as e:
        r["dist_error"] = type(e).__name__
    rows.append(r)
    print("[import_check] " + json.dumps(r, ensure_ascii=False), flush=True)

# 같은 이름이 sys.path 의 여러 곳에 있는지 (먼저 오는 것이 import 됨)
dups = {}
for mod in ["trimesh", "tifffile", "lazy_loader", "warp"]:
    hits = [where(p) for p in sys.path if p and os.path.isdir(os.path.join(p, mod))]
    dups[mod] = hits
print("[import_check] sys.path 상 후보 위치(앞쪽이 우선): " + json.dumps(dups, ensure_ascii=False), flush=True)
bad = [where(p) for p in sys.path if p.startswith(os.path.join(HOME, ".local")) or p.startswith("/opt/ros")]
print(f"[import_check] sys.path 의 ~/.local 또는 /opt/ros: {bad if bad else '없음'}", flush=True)

# cuRobo CUDA 확장 5개
import torch

ext_mods = [
    "curobo.curobolib.kinematics_fused_cu",
    "curobo.curobolib.geom_cu",
    "curobo.curobolib.lbfgs_step_cu",
    "curobo.curobolib.line_search_cu",
    "curobo.curobolib.tensor_step_cu",
]
for em in ext_mods:
    try:
        m = importlib.import_module(em)
        print(f"[import_check] ext OK   {em:42s} {where(m.__file__)}", flush=True)
    except Exception as e:
        print(f"[import_check] ext FAIL {em:42s} {type(e).__name__}: {e}", flush=True)

# warp 초기화 (cuRobo 방식)
try:
    from curobo.types.base import TensorDeviceType
    from curobo.util.warp import init_warp
    import warp as wp

    init_warp(tensor_args=TensorDeviceType())
    print(f"[import_check] warp init OK: version {wp.config.version}, file {where(wp.__file__)}, "
          f"cuda devices {wp.get_cuda_device_count()}", flush=True)
except Exception as e:
    print(f"[import_check] warp init FAIL: {type(e).__name__}: {e}", flush=True)

# torch.compile 사용 여부
from curobo.util import torch_utils

print(f"[import_check] env CUROBO_TORCH_COMPILE_DISABLE={os.environ.get('CUROBO_TORCH_COMPILE_DISABLE')} "
      f"CUROBO_TORCH_COMPILE_FORCE={os.environ.get('CUROBO_TORCH_COMPILE_FORCE')}", flush=True)
print(f"[import_check] is_torch_compile_available() = {torch_utils.is_torch_compile_available()}", flush=True)
deco = torch_utils.get_torch_jit_decorator()
print(f"[import_check] get_torch_jit_decorator() -> {getattr(deco, '__module__', '?')}.{getattr(deco, '__name__', deco)}",
      flush=True)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step2_scene as S
from curobo.rollout.rollout_base import Goal
from curobo.types.math import Pose
from curobo.types.state import JointState

t0 = time.perf_counter()
mpc = S.build_mpc(S.build_robot_cfg(), S.build_world_config())
ta = TensorDeviceType()
js0 = JointState.from_position(mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0),
                               joint_names=mpc.joint_names)
gb = mpc.setup_solve_single(Goal(current_state=js0, goal_state=js0,
                                 goal_pose=Pose.from_list(S.GOALS_WXYZ[0], ta)), 1)
mpc.update_goal(gb)
r = mpc.step(js0, max_attempts=S.MAX_ATTEMPTS)
torch.cuda.synchronize()
dyn_loaded = "torch._dynamo" in sys.modules
ind_loaded = "torch._inductor" in sys.modules
try:
    import torch._dynamo.utils as du

    dyn_counters = {k: dict(v) for k, v in du.counters.items() if v} if dyn_loaded else {}
except Exception:
    dyn_counters = "n/a"
print(f"[import_check] MpcSolver 생성+1 step {time.perf_counter()-t0:.2f}s, feasible={bool(r.metrics.feasible.all())}",
      flush=True)
print(f"[import_check] torch.compile 호출 수 (cuRobo import + MpcSolver 생성 + 1 step 전체): {compile_calls['torch.compile']}", flush=True)
print(f"[import_check] sys.modules: torch._dynamo={dyn_loaded} torch._inductor={ind_loaded} "
      f"triton={'triton' in sys.modules}", flush=True)
print(f"[import_check] dynamo counters: {dyn_counters}", flush=True)
print("[import_check] DONE", flush=True)
if WITH_KIT:
    app.close()

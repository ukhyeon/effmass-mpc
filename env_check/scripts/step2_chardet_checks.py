"""step2_chardet_checks.py — chardet 제거 후 확인 (run_isaac.sh 로 실행)

  사용법: run_isaac.sh step2_chardet_checks.py [--with-kit]

  1) requests 가 문자 인코딩 판별에 어떤 라이브러리를 쓰는지 (requests.compat.chardet)
  2) cuRobo franka.yml 에 연결된 파일(robot yml, URDF, collision sphere yml, URDF 가 참조하는 메시)을
     실제로 읽고: (a) 엄격한 UTF-8 해독 가능 여부 (바이너리 STL 은 바이너리로 분류),
     (b) cuRobo 자체 경로(UrdfKinematicsParser(load_meshes=True) → get_link_mesh → trimesh.load)로 로딩
  3) MpcSolver 생성 + 1 step
"""
import os
import sys
import time
import warnings
import xml.etree.ElementTree as ET

WITH_KIT = "--with-kit" in sys.argv
print(f"[chk] mode={'with SimulationApp' if WITH_KIT else 'no SimulationApp'}", flush=True)
if WITH_KIT:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})

HOME = os.path.expanduser("~")
short = lambda p: str(p).replace(HOME, "~")

# ---------- 1) requests ----------
import importlib.util

print(f"[chk] chardet 설치 여부: {importlib.util.find_spec('chardet') is not None}", flush=True)
with warnings.catch_warnings(record=True) as w:
    warnings.simplefilter("always")
    import requests
    import requests.compat as rc
req_warn = [str(x.message) for x in w if "RequestsDependencyWarning" in type(x.message).__name__
            or "doesn't match a supported version" in str(x.message)]
det = rc.chardet
print(f"[chk] requests {requests.__version__} ({short(requests.__file__)})", flush=True)
print(f"[chk] requests.compat.chardet = {getattr(det, '__name__', det)} "
      f"{getattr(det, '__version__', '?')} ({short(getattr(det, '__file__', '?'))})", flush=True)
print(f"[chk] requests import 시 RequestsDependencyWarning: {req_warn if req_warn else '없음'}", flush=True)

# ---------- 2) cuRobo 파일 ----------
from curobo.util_file import get_assets_path, get_robot_configs_path, join_path, load_yaml
from curobo.cuda_robot_model.urdf_kinematics_parser import UrdfKinematicsParser

robot_yml = join_path(get_robot_configs_path(), "franka.yml")
kin = load_yaml(robot_yml)["robot_cfg"]["kinematics"]
urdf = join_path(get_assets_path(), kin["urdf_path"])
mesh_root = join_path(get_assets_path(), kin["asset_root_path"])
spheres_yml = join_path(get_robot_configs_path(), kin["collision_spheres"])
files = [("robot yml", robot_yml), ("urdf", urdf), ("collision spheres yml", spheres_yml)]
root = ET.parse(urdf).getroot()
mesh_files = sorted({m.get("filename") for m in root.iter("mesh") if m.get("filename")})
for mf in mesh_files:
    p = mf.replace("package://", "")
    p = p if os.path.isabs(p) else join_path(mesh_root, p)
    files.append(("mesh", p))


def classify(path):
    b = open(path, "rb").read()
    if path.lower().endswith(".stl") and not b[:5].lower().startswith(b"solid"):
        return "binary STL (텍스트 해독 대상 아님)", len(b)
    try:
        b.decode("utf-8")
        return "UTF-8 OK", len(b)
    except UnicodeDecodeError as e:
        return f"UTF-8 FAIL ({e})", len(b)


n_fail = 0
for kind, path in files:
    if not os.path.isfile(path):
        print(f"[chk] FILE MISSING {kind:22s} {short(path)}", flush=True)
        n_fail += 1
        continue
    res, size = classify(path)
    if "FAIL" in res:
        n_fail += 1
    print(f"[chk] {kind:22s} {res:36s} {size:>9d} B  {short(path)}", flush=True)
print(f"[chk] 파일 {len(files)}개 (메시 {len(mesh_files)}개), 해독 실패/누락 {n_fail}건", flush=True)

# cuRobo 자체 로딩 경로
t0 = time.perf_counter()
parser = UrdfKinematicsParser(urdf, load_meshes=True, mesh_root=mesh_root, build_scene_graph=True)
# cuRobo 본래 경로와 같게: cuda_robot_generator.py 377~378행이 파서 생성 후 메시 경로를 절대 경로로 바꾼다
parser.add_absolute_path_to_link_meshes(mesh_root)
links_with_mesh = [ln for ln, l in parser._robot.link_map.items() if len(l.visuals) > 0 and l.visuals[0].geometry.mesh is not None]
mesh_ok, mesh_err = 0, []
for ln in links_with_mesh:
    try:
        tm = parser.get_link_mesh(ln).get_trimesh_mesh()
        mesh_ok += 1
        print(f"[chk] cuRobo get_link_mesh {ln:18s} vertices {len(tm.vertices):6d} faces {len(tm.faces):6d}", flush=True)
    except Exception as e:
        mesh_err.append((ln, f"{type(e).__name__}: {e}"))
        print(f"[chk] cuRobo get_link_mesh {ln:18s} FAIL {type(e).__name__}: {e}", flush=True)
print(f"[chk] UrdfKinematicsParser(load_meshes=True) + 링크 메시 {mesh_ok}/{len(links_with_mesh)} 로딩, "
      f"오류 {len(mesh_err)}건, {time.perf_counter()-t0:.2f}s", flush=True)

# ---------- 3) MPC ----------
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step2_scene as S
from curobo.rollout.rollout_base import Goal
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.state import JointState

t0 = time.perf_counter()
mpc = S.build_mpc(S.build_robot_cfg(), S.build_world_config())
ta = TensorDeviceType()
js0 = JointState.from_position(mpc.rollout_fn.dynamics_model.retract_config.clone().unsqueeze(0),
                               joint_names=mpc.joint_names)
gb = mpc.setup_solve_single(Goal(current_state=js0, goal_state=js0, goal_pose=Pose.from_list(S.GOALS_WXYZ[0], ta)), 1)
mpc.update_goal(gb)
r = mpc.step(js0, max_attempts=S.MAX_ATTEMPTS)
torch.cuda.synchronize()
print(f"[chk] MpcSolver 생성 + 1 step {time.perf_counter()-t0:.2f}s, feasible={bool(r.metrics.feasible.all())}, "
      f"action finite={bool(torch.isfinite(r.js_action.position).all())}", flush=True)
# cuRobo API 경로: MPC 안의 로봇 모델(CudaRobotModel)에서 링크 메시를 꺼내 trimesh 로 로딩
try:
    km = mpc.rollout_fn.kinematics
    ml = km.get_robot_link_meshes()
    ok = 0
    for m in ml:
        tm = m.get_trimesh_mesh()
        ok += 1
    print(f"[chk] mpc.rollout_fn.kinematics.get_robot_link_meshes(): {ok}/{len(ml)} 로딩 성공 "
          f"(mesh_link_names {list(km.kinematics_config.mesh_link_names)})", flush=True)
except Exception as e:
    print(f"[chk] mpc.rollout_fn.kinematics.get_robot_link_meshes() FAIL {type(e).__name__}: {e}", flush=True)
print(f"[chk] chardet 이 이 프로세스에서 import 되었나: {'chardet' in sys.modules}", flush=True)
print("[chk] DONE", flush=True)
if WITH_KIT:
    app.close()

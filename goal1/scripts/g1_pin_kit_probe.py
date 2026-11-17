"""g1_pin_kit_probe.py <early|late> — Kit(SimulationApp) 와 Pinocchio 같은 프로세스 시험 (설정 변경 없음, 진단용)
  early: SimulationApp 전에 pinocchio import / late: SimulationApp 후 (1-2d 실패 재현). fr3v2_1 URDF 를 cuRobo helper 로 장면에 넣고 메시 prim 수·DOF·Λ_r 계산 확인"""
import os, sys, time
MODE = sys.argv[1]
HOME = os.path.expanduser("~"); sys.path.insert(0, f"{HOME}/effmass_mpc/env_check/scripts")
import step3_ros_common as RC
res = {"mode": MODE}
def try_pin():
    try:
        import pinocchio as pin
        from effmass_robot.robot_inertia import RobotInertia
        import numpy as np
        ri = RobotInertia(); t = time.perf_counter(); C, L, M = ri.compute(np.array([0, -1.3, 0, -2.5, 0, 1.0, 0.0])); res["pin_ok"] = True
        res["pin_ms"] = (time.perf_counter() - t) * 1000; res["pin_file"] = pin.__file__
    except Exception as e:
        res["pin_ok"] = False; res["pin_err"] = f"{type(e).__name__}: {str(e)[:160]}"
if MODE == "early":
    RC.add_ros_paths(); sys.path.append(f"{HOME}/effmass_mpc/ws/install/effmass_robot/lib/python3.10/site-packages"); try_pin()
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
if MODE == "late":
    RC.add_ros_paths(); sys.path.append(f"{HOME}/effmass_mpc/ws/install/effmass_robot/lib/python3.10/site-packages"); try_pin()
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/curobo/examples/isaac_sim"))
import step2_scene as S
S.set_scene("g1")
from isaacsim.core.api import World
from pxr import UsdGeom
world = World(stage_units_in_meters=1.0); stage = world.stage
xf = stage.DefinePrim("/World", "Xform"); stage.SetDefaultPrim(xf)
try:
    robot, path = S.add_isaac_robot(S.build_robot_cfg(), world); world.reset()
    meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh) and str(p.GetPath()).startswith(path)]
    res.update(robot_ok=True, robot_path=path, dof=len(robot.dof_names), mesh_prims=len(meshes),
               mesh_points_total=int(sum(len(UsdGeom.Mesh(m).GetPointsAttr().Get() or []) for m in meshes)))
    for _ in range(5): world.step(render=False)
    res["joint_pos_ok"] = bool(len(robot.get_joint_positions()) == len(robot.dof_names))
except Exception as e:
    res["robot_ok"] = False; res["robot_err"] = f"{type(e).__name__}: {str(e)[:200]}"
import json; print("[pin-kit]", json.dumps(res, ensure_ascii=False), flush=True)
app.close()

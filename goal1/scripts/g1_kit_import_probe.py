"""g1_kit_import_probe.py <none|before|after> — 1-2d: Isaac Python 에서 rclpy·effmass_msgs·Λ_r numpy 모듈을 SimulationApp 앞/뒤에 import 해도
numpy·torch·warp·curobo 경로가 바뀌지 않고 Kit 오류가 늘지 않는지. (EFFMASS_WS=1 ROS_METHOD=min run_isaac_ros.sh 로 실행) fr3v2_1 장면 로드·5 스텝까지."""
import json, os, sys, time
MODE = sys.argv[1]
HOME = os.path.expanduser("~"); sys.path.insert(0, f"{HOME}/effmass_mpc/env_check/scripts")
import step3_ros_common as RC
res = {"mode": MODE}
def mods():
    import numpy, torch, warp, curobo
    return {m.__name__: [m.__file__, getattr(m, "__version__", None), id(m)] for m in (numpy, torch, warp, curobo)}
def ros_imports():
    t = time.perf_counter(); RC.add_ros_paths()
    import rclpy, effmass_msgs.msg as em, hrc_interfaces.msg as hm
    from effmass_robot.robot_inertia_np import RobotInertiaNP
    import numpy as np
    ri = RobotInertiaNP(); C, L, M = ri.compute(np.array([0, -1.3, 0, -2.5, 0, 1.0, 0.0]))
    m = em.HumanInertiaState(); m.shape_radius = [0.1] * 24
    rclpy.init(); n = rclpy.create_node("g1_kit_import_probe"); pub = n.create_publisher(em.HumanInertiaState, "/g1_probe/hi", 1); pub.publish(m); n.destroy_node(); rclpy.shutdown()
    res.update(ros_ok=True, rclpy=rclpy.__file__, effmass_msgs=em.__file__, robot_np=sys.modules["effmass_robot.robot_inertia_np"].__file__, inv_shape=list(L.shape),
               pinocchio_loaded="pinocchio" in sys.modules, import_s=round(time.perf_counter() - t, 2))
if MODE == "before":   # 프로브가 직접 numpy·torch·warp·curobo 를 먼저 import 하지 않음: ROS import 가 끌어온 것만 기록
    ros_imports(); res["loaded_by_ros_before_app"] = {m: getattr(sys.modules[m], "__file__", None) for m in ("numpy", "torch", "warp", "curobo") if m in sys.modules}
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
res["mods_after_app"] = mods()
if MODE == "after":
    ros_imports(); res["mods_after_ros"] = mods(); res["unchanged_by_ros"] = res["mods_after_app"] == res["mods_after_ros"]
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/curobo/examples/isaac_sim"))
import step2_scene as S
S.set_scene("g1")
from isaacsim.core.api import World
world = World(stage_units_in_meters=1.0); stage = world.stage
xf = stage.DefinePrim("/World", "Xform"); stage.SetDefaultPrim(xf)
robot, path = S.add_isaac_robot(S.build_robot_cfg(), world); world.reset()
for _ in range(5): world.step(render=False)
res.update(robot_dof=len(robot.dof_names), robot_path=path)
REF = {"numpy": "extscache/omni.kit.pip_archive-0.0.0+d02c707b.lx64.cp310/pip_prebundle/numpy", "torch": "exts/omni.isaac.ml_archive/pip_prebundle/torch",
       "warp": "extscache/omni.warp.core-1.5.0+lx64/warp", "curobo": "effmass_mpc/curobo/src/curobo"}       # none 모드(아무것도 먼저 안 함)에서 본 위치
res["same_as_none_mode"] = all(REF[m] in res["mods_after_app"][m][0] for m in REF)
for k in ("mods_after_app", "mods_after_ros"):
    if k in res: res[k] = {m: v[:2] for m, v in res[k].items()}
print("[kit-import]", json.dumps(res, ensure_ascii=False), flush=True)
app.close()

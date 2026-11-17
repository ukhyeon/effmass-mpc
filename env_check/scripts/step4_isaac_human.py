"""step4_isaac_human.py — 4-1/4-2 Isaac 쪽 (ROS_METHOD=min run_isaac_ros.sh 로 실행, MPC 없음)

  --scene empty|franka   empty: 바닥만 (4-1 데이터 경로), franka: 바닥 z=-0.9 + 로봇 아래 탁자(시각용) + Franka (4-2 표시)
  --display none|instancer|prims|both
  --tensor               고정 GPU 텐서 갱신
  --gui --hold SEC       GUI 모드 (hold<0: 창 닫을 때까지, 재생 끝난 뒤에도 마지막 자세 유지)
  --ready-file/--end-file/--out
  --shots N              headless 렌더 캡처 N장 (재생 중 일정 간격, 사람·로봇 위치 확인용)
메인 루프: 60 Hz 벽시계 주기, 매 주기 world.step(render=GUI 또는 캡처 시) + HumanLink.pump() (새 메시지 때만 처리)
수신: 별도 데몬 스레드 SingleThreadedExecutor.spin()
"""
import argparse, json, os, sys, threading, time
ap = argparse.ArgumentParser()
ap.add_argument("--scene", default="empty")
ap.add_argument("--display", default="none")
ap.add_argument("--tensor", action="store_true")
ap.add_argument("--gui", action="store_true")
ap.add_argument("--hold", type=float, default=0)
ap.add_argument("--ready-file", required=True)
ap.add_argument("--end-file", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--shots", type=int, default=0)
ap.add_argument("--timeout", type=float, default=300)
ap.add_argument("--mesh", default="none", choices=["none", "usd", "rt", "both"], help="(4-5) SMPL 메시 갱신 방법")
ap.add_argument("--mesh-offset", default="node", choices=["node", "rigid", "node_live"])
ap.add_argument("--human-dir", default="~/effmass_mpc/env_check/step4")
ap.add_argument("--mesh-verify", type=int, default=1)
a = ap.parse_args()
log = lambda *x: print("[isaac4]", *x, flush=True)
T0 = time.time()
from isaacsim import SimulationApp
app = SimulationApp({"headless": not a.gui, "width": 1280, "height": 720})
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import step3_ros_common as RC
RC.add_ros_paths()
import numpy as np, torch
import rclpy
from rclpy.executors import SingleThreadedExecutor
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
import step4_human as H
log(f"SimulationApp {time.time() - T0:.1f}s, rclpy {RC.short(rclpy.__file__)}, torch {RC.short(torch.__file__)}")

world = World(stage_units_in_meters=1.0)
world.set_simulation_dt(physics_dt=1 / 60, rendering_dt=1 / 60)
stage = world.stage
xf = stage.DefinePrim("/World", "Xform"); stage.SetDefaultPrim(xf)
if a.scene == "franka":
    world.scene.add_default_ground_plane(z_position=-0.9)
    VisualCuboid("/World/robot_table", position=np.array([0.0, 0.0, -0.45]), scale=np.array([0.6, 0.6, 0.9]), size=1.0,
                 color=np.array([0.55, 0.45, 0.35]))
    sys.path.insert(0, os.path.expanduser("~/effmass_mpc/curobo/examples/isaac_sim"))
    from helper import add_robot_to_scene
    import step2_scene as S
    robot_cfg = S.build_robot_cfg()
    robot, robot_path = S.add_isaac_robot(robot_cfg, world)
    log(f"Franka {robot_path}")
else:
    world.scene.add_default_ground_plane()
disp = {"none": (), "instancer": ("instancer",), "prims": ("prims",), "both": ("instancer", "prims")}[a.display]
rclpy.init()
node = rclpy.create_node("step4_isaac_human")
human = H.HumanLink(node, os.path.join(os.path.expanduser(a.human_dir), "temp_transform.json"), stage=stage,
                    display=disp, tensor=a.tensor, verify_every=1,
                    mesh={"none": (), "usd": ("usd",), "rt": ("rt",), "both": ("usd", "rt")}[a.mesh],
                    mesh_transform_json=os.path.join(os.path.expanduser(a.human_dir), "mesh_transform.json"), mesh_offset=a.mesh_offset,
                    mesh_faces=os.path.expanduser("~/effmass_mpc/env_check/step4/smpl_neutral_faces.npy"), mesh_verify_every=a.mesh_verify)
ex = SingleThreadedExecutor(); ex.add_node(node)
threading.Thread(target=ex.spin, daemon=True).start()
world.reset()
if a.scene == "franka":
    q0 = robot_cfg["kinematics"]["cspace"]["retract_config"]
    robot.set_joint_positions(q0, [robot.get_dof_index(n) for n in robot_cfg["kinematics"]["cspace"]["joint_names"]])
for _ in range(10):
    world.step(render=a.gui)
shot_dir = os.path.expanduser("~/effmass_mpc/env_check/step4")
if a.shots:
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[2.6, -1.9, 1.1], target=[0.5, 0.0, 0.0])
if a.gui:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[2.6, -1.9, 1.1], target=[0.5, 0.0, 0.0])
open(a.ready_file, "w").write(str(time.time_ns()))
log(f"READY ({time.time() - T0:.1f}s), display {disp}, tensor {a.tensor}, scene {a.scene}")

period = 1 / 60; k = 0; start = time.perf_counter(); end_seen = None; loop = []; pumps = []; mpumps = []; shots_taken = []
while True:
    target = start + k * period
    now = time.perf_counter()
    if target > now:
        time.sleep(target - now)
    l0 = time.perf_counter()
    render = a.gui or (a.shots and k % 60 == 0)
    world.step(render=bool(render))
    l1 = time.perf_counter()
    Tm = human.pump()
    l2 = time.perf_counter()
    loop.append(((l1 - l0) * 1000, (l2 - l1) * 1000, Tm["new"], Tm["mesh_new"]))
    if Tm["mesh_new"]:
        mpumps.append(Tm)
    if Tm["new"]:
        pumps.append(Tm)
    if a.shots and k % 300 == 150 and len(shots_taken) < a.shots and human.last_pumped > 0:
        fn = f"{shot_dir}/{os.path.basename(a.out)}_shot{len(shots_taken)}.png"
        capture_viewport_to_file(get_active_viewport(), fn); shots_taken.append(fn)
    k += 1
    if end_seen is None and os.path.exists(a.end_file):
        end_seen = time.time()
    if end_seen and time.time() - end_seen > 1.5 and not (a.gui and a.hold != 0):
        break
    if a.gui and a.hold > 0 and end_seen and time.time() - end_seen > a.hold:
        break
    if a.gui and not app.is_running():
        break
    if time.time() - T0 > a.timeout and not (a.gui and a.hold < 0):
        log("시간 초과"); break
fc = human.final_check(); mfc = human.mesh_final_check()
ex.shutdown(); node.destroy_node(); rclpy.shutdown()
human.save(a.out)
arr = np.array([[p[x] for x in ("pickup", "transform", "disp_instancer", "disp_prims", "tensor", "verify")] for p in pumps]) \
    if pumps else np.zeros((0, 6))
q = lambda v: dict(mean=round(float(np.mean(v)), 4), p95=round(float(np.percentile(v, 95)), 4), max=round(float(np.max(v)), 4)) if len(v) else None
res = dict(loop_steps=k, pumps=len(pumps), world_step_ms=q([x[0] for x in loop]), pump_ms_all=q([x[1] for x in loop]),
           pickup_ms=q(arr[:, 0]), transform_ms=q(arr[:, 1]), disp_instancer_ms=q(arr[:, 2]), disp_prims_ms=q(arr[:, 3]),
           tensor_ms=q(arr[:, 4]), verify_ms=q(arr[:, 5]), verify_n=human.verify_n, verify_fail=human.verify_fail,
           final_check=fc, cb_pose_ms=q(human.cb_ms["pose"]), cb_dyn_ms=q(human.cb_ms["dyn"]),
           n_pose=len(human.pose_rec), n_dyn=len(human.dyn_rec), shots=shots_taken,
           mesh=a.mesh, mesh_offset=a.mesh_offset, mesh_updates=len(mpumps), mesh_final=mfc,
           mesh_xf_ms=q([x["mesh_xf"] for x in mpumps]), mesh_usd_ms=q([x["mesh_usd"] for x in mpumps]),
           mesh_rt_ms=q([x["mesh_rt"] for x in mpumps]), mesh_verify_ms=q([x["mesh_verify"] for x in mpumps]),
           mesh_verify_n=human.mesh_verify_n, mesh_verify_fail=human.mesh_verify_fail,
           pump_ms_mesh_steps=q([x[1] for x in loop if x[3]]), world_step_ms_mesh_steps=q([x[0] for x in loop if x[3]]))
json.dump(res, open(a.out + "_isaac_summary.json", "w"), indent=1)
for kk, v in res.items():
    log(f"{kk}: {v}")
app.close()

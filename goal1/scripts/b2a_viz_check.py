"""b2a_viz_check.py - human display check without MPC (Isaac python via run_isaac_ros.sh, EFFMASS_WS=1).
World: ground plane z -0.9, robot at retract (visual reference only, no controller), human from a precomputed file shown with
PrecomputedHumanLink (mesh + one-colour translucent capsules + COM points). Steps the world N times, pumps the human every step,
counts prims under /World/human and records display update time and loop time. --gui --duration S: window closes by itself after S seconds."""
import argparse, json, os, sys, time
ap = argparse.ArgumentParser()
ap.add_argument("--pre", default="~/effmass_mpc/goal1/data/human_b2a_reach.npz")
ap.add_argument("--mesh", default="")
ap.add_argument("--human-dir", default="~/effmass_mpc/goal1/b2a/view")
ap.add_argument("--viz", default="both", choices=["mesh", "shapes", "both"])
ap.add_argument("--steps", type=int, default=120)
ap.add_argument("--gui", action="store_true")
ap.add_argument("--duration", type=float, default=30.0)
ap.add_argument("--out", default="~/effmass_mpc/goal1/logs/meff_isaac/b2a_viz_check.json")
ap.add_argument("--viz-zone", default="", help="(V1) zone surface npz from v1_zone.py, static translucent mesh")
ap.add_argument("--hide-com", action="store_true", help="(V1) hide the human COM markers")
ap.add_argument("--cam", default="", help="(V1) GUI camera eye,target (6 numbers)")
a = ap.parse_args()
from isaacsim import SimulationApp
app = SimulationApp({"headless": not a.gui, "width": 1280, "height": 720})
import numpy as np, torch
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts")); sys.path.insert(0, os.path.expanduser("~/effmass_mpc/goal1/scripts"))
import step3_ros_common as RC
RC.add_ros_paths()
os.environ["STEP_ROBOT"] = "fr3v2_1"
import step2_scene as S
S.set_scene("g1")
from isaacsim.core.api import World
import g1_meff_gpu as G1M, g1_human_link as G1L
world = World(stage_units_in_meters=1.0); world.set_simulation_dt(physics_dt=S.PHYSICS_DT, rendering_dt=S.PHYSICS_DT)
stage = world.stage; stage.SetDefaultPrim(stage.DefinePrim("/World", "Xform")); world.scene.add_default_ground_plane(z_position=S.GROUND_Z)
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/curobo/examples/isaac_sim"))
robot_cfg = S.build_robot_cfg(); robot, _ = S.add_isaac_robot(robot_cfg, world)
inp = G1M.MeffInputs(61, [1 / 60] * 30, device="cuda:0")
pre = os.path.expanduser(a.pre); _base = pre[:-len("_v2.npz")] + ".npz" if pre.endswith("_v2.npz") else pre
mesh = os.path.expanduser(a.mesh) if a.mesh else _base.replace(".npz", "_verts.npz")
H = G1L.PrecomputedHumanLink(pre, S.PHYSICS_DT, os.path.join(os.path.expanduser(a.human_dir), "temp_transform.json"), inp, stage=stage, display=("instancer",),
                             shapes_viz=a.viz in ("shapes", "both"), mesh_npz=mesh if a.viz in ("mesh", "both") else None)
ZONE = None
if a.viz_zone:
    import g1_viz as G1V
    ZONE = G1V.add_zone(stage, os.path.expanduser(a.viz_zone), by_mu="uniform" not in os.path.basename(a.viz_zone))
if a.hide_com:
    from pxr import UsdGeom as _UG
    _pr = stage.GetPrimAtPath("/World/human/instancer")
    if _pr.IsValid():
        _UG.Imageable(_pr).MakeInvisible()
if a.gui:
    from isaacsim.core.utils.viewports import set_camera_view
    c = H.pre["com"][0][18].astype(float) @ H.R.T + H.t
    if a.cam:
        _v = [float(x) for x in a.cam.split(",")]; set_camera_view(eye=_v[:3], target=_v[3:])
    else:
        set_camera_view(eye=list(c + np.array([1.2, -1.6, 0.9])), target=list(c))
world.reset(); world.play()
names = robot_cfg["kinematics"]["cspace"]["joint_names"]; robot.set_joint_positions(robot_cfg["kinematics"]["cspace"]["retract_config"], [robot.get_dof_index(n) for n in names])
pump_ms, loop_ms = [], []
t_end = time.perf_counter() + a.duration
i = 0
while True:
    l0 = time.perf_counter(); world.step(render=a.gui); p0 = time.perf_counter(); T = H.pump(i); p1 = time.perf_counter()
    pump_ms.append((p1 - p0) * 1000); loop_ms.append((p1 - l0) * 1000); i += 1
    if (not a.gui and i >= a.steps) or (a.gui and (time.perf_counter() > t_end or i >= a.steps or not app.is_running())):
        break
from pxr import Usd
kinds = {}
for prim in Usd.PrimRange(stage.GetPrimAtPath("/World/human")):
    kinds[prim.GetTypeName()] = kinds.get(prim.GetTypeName(), 0) + 1
res = dict(pre=pre, mesh=mesh if H.mesh_V is not None else None, viz=a.viz, steps=i, prims_under_world_human=kinds,
           mesh_points=int(len(H.mesh_V[0])) if H.mesh_V is not None else 0, mesh_update_ms=H.mesh_ms, mesh_updates=len(H.mesh_ms),
           pump_ms_first=pump_ms[0], pump_ms_mean_after_first=float(np.mean(pump_ms[1:])) if len(pump_ms) > 1 else None,
           loop_ms_mean=float(np.mean(loop_ms)), loop_ms_p95=float(np.percentile(loop_ms, 95)), frames=H.F, clamped=H.n_clamped, zone=ZONE)
print("[viz-check]", json.dumps(res), flush=True)
json.dump(res, open(os.path.expanduser(a.out), "w"), indent=1)
world.stop(); app.close()

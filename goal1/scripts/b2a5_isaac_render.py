"""b2a5_isaac_render.py <prep_npz> [<prep_npz> ...] [--frames a:b] - Isaac Sim (headless RTX) replay render of saved runs (run_isaac.sh).
No MPC: the robot is set to the executed joint positions of each step (fr3v2_1 cuRobo URDF import, same as the runs), the human is the static B2a
body mesh, a translucent capsule = the hand shape that sets the cost at that step grown by the eta of the closest pair (the cost's active distance),
a thin rod = closest robot sphere -> closest point on that hand shape, a small sphere = that robot sphere. Two cameras (oblique, side).
Frames: goal1/b2a/render/<tag>/<cam>_<step>.png."""
import os, sys, json
from isaacsim import SimulationApp
W, H = 960, 540
app = SimulationApp({"headless": True, "width": W, "height": H})
import numpy as np
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts")); sys.path.insert(0, os.path.expanduser("~/effmass_mpc/curobo/examples/isaac_sim"))
import step2_scene as S
S.G2_SCENE_JSON = os.path.expanduser("~/effmass_mpc/goal1/b2a/g2/scene_b2a4_front_0.38h1.json"); S.set_scene("g2")
from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from pxr import Gf, UsdGeom, UsdShade, Sdf, Vt, UsdLux
import omni.replicator.core as rep
from PIL import Image
args = [a for a in sys.argv[1:] if not a.startswith("--")]; fr = next((a.split("=")[1] for a in sys.argv[1:] if a.startswith("--frames=")), None)
world = World(stage_units_in_meters=1.0); world.set_simulation_dt(physics_dt=1 / 60, rendering_dt=1 / 60)
world.scene.add_default_ground_plane(z_position=-0.9)
robot_cfg = S.build_robot_cfg(); robot, rpath = S.add_isaac_robot(robot_cfg, world)
VisualCuboid("/World/robot_table", position=np.array([0, 0, -0.45]), scale=np.array([0.6, 0.6, 0.9]), size=1.0, color=np.array([0.55, 0.45, 0.35]))
stage = world.stage
UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(900)
dl = UsdLux.DistantLight.Define(stage, "/World/sun"); dl.CreateIntensityAttr(2500); UsdGeom.Xformable(dl).AddRotateXYZOp().Set(Gf.Vec3f(-50, 20, 0))


def material(name, rgb, opacity=1.0, rough=0.5):
    m = UsdShade.Material.Define(stage, f"/World/Looks/{name}"); sh = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/s"); sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb)); sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity); m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface"); return m


z0 = np.load(args[0]); V = z0["verts"]; F = z0["faces"]
mesh = UsdGeom.Mesh.Define(stage, "/World/human/mesh"); P = np.ascontiguousarray(V, np.float32)
mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(P)); mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*P.min(0).tolist()), Gf.Vec3f(*P.max(0).tolist())]))
mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(F))); mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.reshape(-1).astype(np.int32))); mesh.CreateSubdivisionSchemeAttr("none")
mesh.CreateDoubleSidedAttr(True); UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(material("skin", (0.85, 0.66, 0.53), 1.0, 0.7))
COLS = {"iso": (0.16, 0.47, 0.84), "massA": (0.92, 0.41, 0.20), "unim": (0.93, 0.63, 0.0)}
shell = UsdGeom.Capsule.Define(stage, "/World/viz/shell"); shell.CreateAxisAttr("Z"); sx = UsdGeom.Xformable(shell); sh_t, sh_o = sx.AddTranslateOp(), sx.AddOrientOp()
rod = UsdGeom.Cylinder.Define(stage, "/World/viz/rod"); rod.CreateAxisAttr("Z"); rod.CreateRadiusAttr(0.003); rx = UsdGeom.Xformable(rod); rd_t, rd_o = rx.AddTranslateOp(), rx.AddOrientOp()
ball = UsdGeom.Sphere.Define(stage, "/World/viz/ball"); bx = UsdGeom.Xformable(ball); bl_t = bx.AddTranslateOp()
UsdShade.MaterialBindingAPI(rod.GetPrim()).Bind(material("rod", (0.05, 0.05, 0.05)))


def quat_z_to(v):
    v = v / max(np.linalg.norm(v), 1e-12); z = np.array([0, 0, 1.0]); ax = np.cross(z, v); s = np.linalg.norm(ax); c = float(z @ v)
    if s < 1e-9:
        return Gf.Quatf(1, 0, 0, 0) if c > 0 else Gf.Quatf(0, 1, 0, 0)
    a = np.arctan2(s, c); ax = ax / s; return Gf.Quatf(float(np.cos(a / 2)), *[float(x) for x in ax * np.sin(a / 2)])


CAMS = {k: v for k, v in {"oblique": ((0.12, -1.10, 1.08), (0.46, 0.0, 0.47)), "top": ((0.47, -0.30, 1.45), (0.47, 0.0, 0.45)), "side": ((0.46, -1.30, 0.50), (0.46, 0.0, 0.47))}.items() if k in (next((a.split("=")[1] for a in sys.argv[1:] if a.startswith("--cams=")), "oblique,side")).split(",")}
ann = {}
for cn, (eye, tgt) in CAMS.items():
    cp = f"/World/cams/{cn}"; cam = UsdGeom.Camera.Define(stage, cp); cam.CreateFocalLengthAttr(24.0); cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    Mv = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*tgt), Gf.Vec3d(0, 0, 1)).GetInverse()     # camera to world, +z up, no roll
    UsdGeom.Xformable(cam).AddTransformOp().Set(Mv); rp = rep.create.render_product(cp, (W, H))
    a = rep.AnnotatorRegistry.get_annotator("rgb"); a.attach([rp]); ann[cn] = a
world.reset()
names = robot_cfg["kinematics"]["cspace"]["joint_names"]; q_ret = robot_cfg["kinematics"]["cspace"]["retract_config"]
idx = [robot.get_dof_index(n) for n in names]; ctrl = robot.get_articulation_controller()
from isaacsim.core.utils.types import ArticulationAction


def set_q(q7):
    q = np.array(list(q7) + list(q_ret[7:]), float); robot.set_joint_positions(q, idx); ctrl.apply_action(ArticulationAction(joint_positions=q, joint_indices=idx))


for path in args:
    z = np.load(path); tag = str(z["tag"]); cond = str(z["cond"]); out = os.path.expanduser(f"~/effmass_mpc/goal1/b2a/render/{tag}"); os.makedirs(out, exist_ok=True)
    m = material(f"shell_{cond}", COLS.get(cond, (0.5, 0.5, 0.5)), 0.30, 0.3); UsdShade.MaterialBindingAPI(shell.GetPrim()).Bind(m)
    UsdShade.MaterialBindingAPI(ball.GetPrim()).Bind(material(f"ball_{cond}", COLS.get(cond, (0.5, 0.5, 0.5)), 1.0, 0.4))
    n = len(z["q"]); a0, a1 = (0, n) if fr is None else map(int, fr.split(":"))
    for i in range(a0, min(a1, n)):
        set_q(z["q"][i]); k = int(z["k"][i]); p0, p1 = z["p0"][k], z["p1"][k]
        shell.GetRadiusAttr().Set(float(z["shell_r"][i])); shell.GetHeightAttr().Set(float(np.linalg.norm(p1 - p0))); sh_t.Set(Gf.Vec3d(*((p0 + p1) / 2).tolist())); sh_o.Set(quat_z_to(p1 - p0))
        c, qp = z["c"][i], z["qp"][i]; rod.GetHeightAttr().Set(float(np.linalg.norm(qp - c))); rd_t.Set(Gf.Vec3d(*((c + qp) / 2).tolist())); rd_o.Set(quat_z_to(qp - c))
        ball.GetRadiusAttr().Set(float(z["cr"][i])); bl_t.Set(Gf.Vec3d(*c.tolist()))
        for _ in range(3 if i > a0 else 30):     # settle the renderer (first frame: more)
            world.step(render=True); set_q(z["q"][i])
        for cn, a in ann.items():
            d = a.get_data(); Image.fromarray(np.asarray(d)[:, :, :3]).save(f"{out}/{cn}_{i:04d}.png")
    print("rendered", tag, a0, min(a1, n), flush=True)
app.close()

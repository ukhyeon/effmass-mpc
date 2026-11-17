"""g1_fr3v21_usd_check.py — Franka franka_simulation v0.1.0 fr3v2_1 USD (Isaac Sim 6 용) 를 Isaac 4.5 에서 열어 보기만 (장면 성능 측정 없음)
  구조·관절 한계·구동 이득·이름, 그리고 franka_description 2.9.0 fr3v2_1 URDF 의 numpy FK 와 손 위치 비교 (무작위 10 자세)"""
import json, os, sys
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np, omni.usd
from pxr import Usd, UsdPhysics, PhysxSchema, UsdGeom
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.robots import Robot
P = os.path.expanduser("~/effmass_mpc/ref/franka_simulation_release_v0.1.0/fr3v2_1/fr3v2_1.usda")
out = dict(path=P)
st0 = Usd.Stage.Open(P)
out["open_ok"] = st0 is not None
print("[v21] Usd.Stage.Open", out["open_ok"], "defaultPrim", st0.GetDefaultPrim().GetPath() if st0 else None, flush=True)
world = World(stage_units_in_meters=1.0); stage = world.stage; x = stage.DefinePrim("/World", "Xform"); stage.SetDefaultPrim(x)
add_reference_to_stage(P, "/World/fr3v2_1")
art = [p.GetPath().pathString for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print("[v21] 관절 루트", art, flush=True)
J = []
for p in stage.Traverse():
    if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint):
        kind = "angular" if p.IsA(UsdPhysics.RevoluteJoint) else "linear"
        j = UsdPhysics.RevoluteJoint(p) if kind == "angular" else UsdPhysics.PrismaticJoint(p); d = UsdPhysics.DriveAPI.Get(p, kind)
        pj = PhysxSchema.PhysxJointAPI(p)
        J.append(dict(name=p.GetName(), kind=kind, lower=j.GetLowerLimitAttr().Get(), upper=j.GetUpperLimitAttr().Get(),
                      stiffness=d.GetStiffnessAttr().Get() if d else None, damping=d.GetDampingAttr().Get() if d else None,
                      max_force=d.GetMaxForceAttr().Get() if d else None, max_vel=pj.GetMaxJointVelocityAttr().Get() if pj else None))
        print("[v21] 관절", J[-1], flush=True)
links = [p.GetPath().pathString for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
masses = {}
for l in links:
    m = UsdPhysics.MassAPI(stage.GetPrimAtPath(l)); masses[l.split("/")[-1]] = m.GetMassAttr().Get() if m else None
print("[v21] 강체", len(links), masses, flush=True)
out.update(articulation_root=art, joints=J, rigid_bodies=links, masses=masses)
try:
    robot = world.scene.add(Robot(prim_path=art[0], name="v21")); world.reset()
    print("[v21] dof_names", robot.dof_names, flush=True)
    g = {}; src = open(os.path.expanduser("~/effmass_mpc/goal1/scripts/g1_variant_meff.py")).read().split("models = {m: load(m)")[0]; exec(src, g)
    mod = g["load"]("fr3v2_1")
    names = [f"fr3v2_1_joint{i}" for i in range(1, 8)]; idx = [robot.get_dof_index(n) for n in names]
    hand = next(l for l in links if l.endswith("fr3v2_1_hand"))
    rng = np.random.default_rng(0); xf = UsdGeom.XformCache(); errs = []
    lo = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]); hi = -lo; hi[3] = -0.1518; lo[5] = 0.5445; hi[5] = 4.5169
    for k in range(10):
        q = lo + (hi - lo) * rng.uniform(0.1, 0.9, 7)
        robot.set_joint_positions(q, idx); world.step(render=False); qq = robot.get_joint_positions()[idx]
        xf.Clear(); pu = np.array(xf.GetLocalToWorldTransform(stage.GetPrimAtPath(hand)).ExtractTranslation())
        W, _ = g["fk"](mod, qq); errs.append(float(np.linalg.norm(pu - W["fr3v2_1_hand"][:3, 3])))
    out["hand_fk_err_mm"] = [round(e * 1000, 4) for e in errs]
    print(f"[v21] 손 위치 USD ↔ URDF numpy FK 차 최대 {max(errs)*1000:.4f} mm (10 자세)", flush=True)
except Exception as e:
    out["articulation_error"] = repr(e)[:300]; print("[v21] 관절 생성·FK 실패", repr(e)[:300], flush=True)
json.dump(out, open(os.path.expanduser("~/effmass_mpc/goal1/logs/g1_fr3v21_usd_check.json"), "w"), indent=1, default=str)
app.close()

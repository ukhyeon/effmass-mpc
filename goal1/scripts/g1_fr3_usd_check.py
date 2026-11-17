"""g1_fr3_usd_check.py — Isaac 4.5 공식 FR3 USD 확인 (headless): 구조·관절 한계·구동 이득, cuRobo FR3 FK 와 링크 위치 비교"""
import json, os, sys
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np, torch
import omni.usd
from pxr import Usd, UsdPhysics, PhysxSchema, UsdGeom
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.api.robots import Robot
root = get_assets_root_path(); url = root + "/Isaac/Robots/Franka/FR3/fr3.usd"
print("[usd]", url, flush=True)
world = World(stage_units_in_meters=1.0)
add_reference_to_stage(url, "/World/fr3")
stage = omni.usd.get_context().get_stage()
art = [p.GetPath().pathString for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
print("[usd] 관절 루트", art)
J = []
for p in stage.Traverse():
    if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint):
        j = UsdPhysics.RevoluteJoint(p) if p.IsA(UsdPhysics.RevoluteJoint) else UsdPhysics.PrismaticJoint(p)
        kind = "angular" if p.IsA(UsdPhysics.RevoluteJoint) else "linear"
        d = UsdPhysics.DriveAPI.Get(p, kind)
        pj = PhysxSchema.PhysxJointAPI(p)
        mv = pj.GetMaxJointVelocityAttr().Get() if pj else None
        J.append(dict(path=p.GetPath().pathString, kind=kind, lower=j.GetLowerLimitAttr().Get(), upper=j.GetUpperLimitAttr().Get(),
                      stiffness=d.GetStiffnessAttr().Get() if d else None, damping=d.GetDampingAttr().Get() if d else None,
                      max_force=d.GetMaxForceAttr().Get() if d else None, max_vel=mv))
for j in J:
    print("[usd] 관절", j, flush=True)
links = [p.GetPath().pathString for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
print("[usd] 강체", len(links), [l.split("/")[-1] for l in links])
robot = world.scene.add(Robot(prim_path=art[0], name="fr3"))
world.reset()
print("[usd] dof_names", robot.dof_names)
# cuRobo FR3 FK 와 비교
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.util_file import load_yaml
ta = TensorDeviceType()
rc = RobotConfig.from_dict(load_yaml(os.path.expanduser("~/effmass_mpc/goal1/fr3/fr3.yml"))["robot_cfg"], ta)
km = CudaRobotModel(rc.kinematics)
names = [f"fr3_joint{i}" for i in range(1, 8)]
idx = [robot.get_dof_index(n) for n in names]
jl = rc.kinematics.get_joint_limits().position.cpu().numpy()
rng = np.random.default_rng(0)
hand = [l for l in links if l.endswith("fr3_hand")]
xf = UsdGeom.XformCache()
errs = []
for k in range(10):
    q = jl[0] + (jl[1] - jl[0]) * rng.uniform(0.1, 0.9, 7)
    robot.set_joint_positions(q, idx); world.step(render=False)
    qq = robot.get_joint_positions()[idx]
    xf.Clear(); m = xf.GetLocalToWorldTransform(stage.GetPrimAtPath(hand[0]))
    p_usd = np.array(m.ExtractTranslation())
    st = km.get_state(torch.tensor(qq[None], device=ta.device, dtype=ta.dtype))
    errs.append(float(np.linalg.norm(p_usd - st.ee_position[0].cpu().numpy())))
print(f"[usd] 손(fr3_hand) 위치: USD ↔ cuRobo FR3 FK 차 최대 {max(errs)*1000:.3f} mm (10 자세), 링크 경로 {hand}", flush=True)
json.dump(dict(url=url, articulation_root=art, joints=J, rigid_bodies=links, dof_names=list(robot.dof_names), hand_fk_err_mm_max=max(errs) * 1000),
          open(os.path.expanduser("~/effmass_mpc/goal1/logs/g1_fr3_usd_check.json"), "w"), indent=1, default=str)
app.close()

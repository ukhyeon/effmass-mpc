"""g1_drive_probe.py — 로봇 생성 직후 USD 관절 구동 속성값 출력 (읽기만). STEP_ROBOT=panda|fr3"""
import os, sys
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts")); sys.path.insert(0, os.path.expanduser("~/effmass_mpc/curobo/examples/isaac_sim"))
from pxr import UsdPhysics
from isaacsim.core.api import World
import step2_scene as S
w = World(stage_units_in_meters=1.0); st = w.stage; x = st.DefinePrim("/World", "Xform"); st.SetDefaultPrim(x)
cfg = S.build_robot_cfg(); robot, path = S.add_isaac_robot(cfg, w)
for p in st.Traverse():
    if p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint):
        kind = "angular" if p.IsA(UsdPhysics.RevoluteJoint) else "linear"; d = UsdPhysics.DriveAPI.Get(p, kind)
        print("[drive]", S.robot_name(), p.GetPath().pathString.split("/")[-1], kind, d.GetStiffnessAttr().Get(), d.GetDampingAttr().Get(), d.GetMaxForceAttr().Get(), flush=True)
dups = [p.GetPath().pathString for p in st.Traverse() if p.GetName().endswith('_joint1')]
print('[dup]', dups, flush=True)
for q in st.Traverse():
    if q.GetName().endswith('_joint1'):
        print('[dup] stack', q.GetPath().pathString, [str(x.layer.identifier)[-60:] for x in q.GetPrimStack()][:4], flush=True)
w.reset(); S.apply_arm_gains_runtime(robot)
import numpy as np
kp, kd = robot._articulation_view.get_gains()
print("[gains_view]", S.robot_name(), np.round(np.asarray(kp).reshape(-1), 3).tolist(), np.round(np.asarray(kd).reshape(-1), 3).tolist(), flush=True)
app.close()

"""g1_fk_isaac_check.py — STEP_ROBOT 의 cuRobo 설정 FK ↔ Isaac 로봇(step2_scene.add_isaac_robot) 손 위치, 무작위 10 자세. 구동 이득 기록 포함"""
import os, sys, json
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts")); sys.path.insert(0, os.path.expanduser("~/effmass_mpc/curobo/examples/isaac_sim"))
import numpy as np, torch
from pxr import UsdGeom
from isaacsim.core.api import World
import step2_scene as S
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
w = World(stage_units_in_meters=1.0); st = w.stage; x = st.DefinePrim("/World", "Xform"); st.SetDefaultPrim(x); w.scene.add_default_ground_plane()
cfg = S.build_robot_cfg(); robot, path = S.add_isaac_robot(cfg, w)
print("[fk] robot", S.robot_name(), path, "이득", json.dumps(S.FR3_GAINS_APPLIED, default=str), flush=True)
w.reset(); S.apply_arm_gains_runtime(robot)
ta = TensorDeviceType(); rc = RobotConfig.from_dict(cfg, ta); km = CudaRobotModel(rc.kinematics)
names = cfg["kinematics"]["cspace"]["joint_names"][:7]; idx = [robot.get_dof_index(n) for n in names]
ee = cfg["kinematics"]["ee_link"]
hand = next(p.GetPath().pathString for p in st.Traverse() if p.GetName() == ee and p.GetPath().pathString.startswith(path))
jl = rc.kinematics.get_joint_limits().position.cpu().numpy(); rng = np.random.default_rng(0); xf = UsdGeom.XformCache(); errs = []
print("[fk] dof_names", robot.dof_names, "hand prim", hand, flush=True)
for k in range(10):
    q = jl[0] + (jl[1] - jl[0]) * rng.uniform(0.1, 0.9, 7)
    robot.set_joint_positions(q, idx); w.step(render=False); qq = robot.get_joint_positions()[idx]
    xf.Clear(); pu = np.array(xf.GetLocalToWorldTransform(st.GetPrimAtPath(hand)).ExtractTranslation())
    s_ = km.get_state(torch.tensor(qq[None], device=ta.device, dtype=ta.dtype)); errs.append(float(np.linalg.norm(pu - s_.ee_position[0].cpu().numpy())))
print(f"[fk] 손({ee}) 위치 Isaac ↔ cuRobo FK 차 최대 {max(errs)*1000:.4f} mm, 평균 {np.mean(errs)*1000:.4f} mm (10 자세)", flush=True)
json.dump(dict(robot=S.robot_name(), gains=S.FR3_GAINS_APPLIED, dof_names=list(robot.dof_names), hand_err_mm=[e * 1000 for e in errs]),
          open(os.path.expanduser(f"~/effmass_mpc/goal1/logs/g1_fk_isaac_check_{S.robot_name()}.json"), "w"), indent=1, default=str)
app.close()

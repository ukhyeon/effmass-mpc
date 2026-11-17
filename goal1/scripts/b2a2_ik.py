"""b2a2_ik.py - cuRobo IK on the extended front / top TCP lines (y from -0.60 to +0.60 around the line centre, 1 cm), gripper down (Isaac python,
no SimulationApp; world = table + floor). Output goal1/b2a/g2/ik_ext.json (q, success, TCP and hand-link targets per point)."""
import json, os, sys
import numpy as np, torch
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts"))
import step2_scene as S
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
S.set_scene("g1"); ta = TensorDeviceType()
L = json.load(open(os.path.expanduser("~/effmass_mpc/goal1/b2a/g2/lines.json")))
rc = RobotConfig.from_dict(S.build_robot_cfg(), ta)
ik = IKSolver(IKSolverConfig.load_from_robot_config(rc, S.build_world_config(), num_seeds=64, self_collision_check=True, self_collision_opt=True, tensor_args=ta,
                                                    use_cuda_graph=False, position_threshold=0.002, rotation_threshold=0.02))
q4 = [0.0, 1.0, 0.0, 0.0]; off = np.array([0, 0, 0.1034]); out = {}
for nm in ("front", "top"):
    c = np.array(L["lines"][nm]["center"]); ys = np.round(np.arange(-0.60, 0.6001, 0.01), 3); T = np.array([[c[0], c[1] + y, c[2]] for y in ys]); P = T + off
    r = ik.solve_batch(Pose(position=torch.tensor(P, **ta.as_torch_dict()).float(), quaternion=torch.tensor([q4] * len(P), **ta.as_torch_dict()).float()))
    out[nm] = dict(dy=ys.tolist(), tcp=T.tolist(), hand_target=P.tolist(), success=r.success.view(-1).cpu().numpy().tolist(), q=r.solution.view(len(P), -1).cpu().numpy()[:, :7].round(6).tolist())
    print(nm, "IK success", int(np.sum(out[nm]["success"])), "/", len(P), flush=True)
json.dump(out, open(os.path.expanduser("~/effmass_mpc/goal1/b2a/g2/ik_ext.json"), "w"))

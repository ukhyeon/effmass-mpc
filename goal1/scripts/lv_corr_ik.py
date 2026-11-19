"""lv_corr_ik.py <in_json> <out_json> - live3m corrected paths: cuRobo IK (gripper down, wxyz 0 1 0 0, hand link = TCP + 0.1034 m) for a
list of TCP points given in the Franka base frame (Isaac python via run_isaac.sh, no SimulationApp; world = robot table + floor as
scene g2). in_json: {"points": [[x, y, z], ...]}; out_json: {"points", "success", "q" (7 per point)}. Of 16 returned seeds per point the
successful solution nearest to the retract configuration with joint 1 turned toward the point (atan2(y, x)) is kept."""
import json, os, sys
import numpy as np, torch
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts"))
import step2_scene as S
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
S.set_scene("g1"); ta = TensorDeviceType(); D = json.load(open(sys.argv[1])); P = np.array(D["points"], float)
rc = RobotConfig.from_dict(S.build_robot_cfg(), ta)
ik = IKSolver(IKSolverConfig.load_from_robot_config(rc, S.build_world_config(), num_seeds=64, self_collision_check=True, self_collision_opt=True, tensor_args=ta,
                                                    use_cuda_graph=False, position_threshold=0.002, rotation_threshold=0.02))
Pp = P + np.array([0, 0, 0.1034]); q4 = [0.0, 1.0, 0.0, 0.0]
r = ik.solve_batch(Pose(position=torch.tensor(Pp, **ta.as_torch_dict()).float(), quaternion=torch.tensor([q4] * len(Pp), **ta.as_torch_dict()).float()), return_seeds=16)
SO = r.solution.view(len(Pp), 16, -1).cpu().numpy()[:, :, :7]; SU = r.success.view(len(Pp), 16).cpu().numpy().astype(bool)
ret = np.array(S.build_robot_cfg()["kinematics"]["cspace"]["retract_config"][:7], float); ok = SU.any(1); Q = np.zeros((len(Pp), 7))
for i in range(len(Pp)):
    ref = ret.copy(); ref[0] = np.arctan2(P[i, 1], P[i, 0]); d = np.linalg.norm(SO[i] - ref, axis=1); d[~SU[i]] = np.inf; Q[i] = SO[i, int(np.argmin(d))]
json.dump(dict(points=P.tolist(), success=ok.tolist(), q=Q.round(6).tolist()), open(sys.argv[2], "w"))
print("IK ok", int(ok.sum()), "/", len(ok))

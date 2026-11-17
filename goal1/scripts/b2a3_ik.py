"""b2a3_ik.py - cuRobo IK for the B2a-3 hold pose candidates (Isaac python via run_isaac.sh, no SimulationApp; world = table + floor).
TCP at the right-hand COM height (lines.json hand_height_above_base), y = 0, x candidates; gripper down (quaternion 0 1 0 0), hand link = TCP + 0.1034 m in z.
Output goal1/b2a/g2/b2a3_ik.json: q, success, joint margins (URDF limits)."""
import json, os, sys
import numpy as np, torch
import xml.etree.ElementTree as ET
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts"))
import step2_scene as S
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
S.set_scene("g1"); ta = TensorDeviceType(); H = os.path.expanduser("~")
L = json.load(open(f"{H}/effmass_mpc/goal1/b2a/g2/lines.json")); z = float(L["hand_height_above_base"])
root = ET.parse(f"{H}/effmass_mpc/goal1/fr3v2_1/fr3v2_1_curobo_narrow.urdf").getroot()
lim = {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper"))) for j in root.findall("joint") if j.get("type") == "revolute"}
LO = np.array([lim[f"fr3v2_1_joint{i}"][0] for i in range(1, 8)]); HI = np.array([lim[f"fr3v2_1_joint{i}"][1] for i in range(1, 8)])
rc = RobotConfig.from_dict(S.build_robot_cfg(), ta)
ik = IKSolver(IKSolverConfig.load_from_robot_config(rc, S.build_world_config(), num_seeds=64, self_collision_check=True, self_collision_opt=True, tensor_args=ta,
                                                    use_cuda_graph=False, position_threshold=0.002, rotation_threshold=0.02))
xs = [0.40, 0.45, 0.47, 0.50, 0.55]; T = np.array([[x, 0.0, z] for x in xs]); P = T + np.array([0, 0, 0.1034]); q4 = [0.0, 1.0, 0.0, 0.0]
r = ik.solve_batch(Pose(position=torch.tensor(P, **ta.as_torch_dict()).float(), quaternion=torch.tensor([q4] * len(P), **ta.as_torch_dict()).float()))
Q = r.solution.view(len(P), -1).cpu().numpy()[:, :7]; ok = r.success.view(-1).cpu().numpy()
out = []
for x, t, p, q, o in zip(xs, T, P, Q, ok):
    m = np.minimum(q - LO, HI - q); out.append(dict(x=x, tcp=t.tolist(), hand_target=p.tolist(), success=bool(o), q=q.round(6).tolist(), margins=m.round(4).tolist(), min_margin=float(m.min()), worst_joint=int(np.argmin(m)) + 1))
    print(f"x {x:.2f} ok {bool(o)} min margin {m.min():.3f} (j{np.argmin(m)+1}) q {np.round(q,3).tolist()}", flush=True)
json.dump(out, open(f"{H}/effmass_mpc/goal1/b2a/g2/b2a3_ik.json", "w"), indent=1)

"""b2a_ik.py - cuRobo IK along the three TCP lines of scene g2 (Isaac python, no SimulationApp). World = g1 table + floor (the human is not
in the cuRobo world, as in the MPC). Orientation candidates (wxyz of fr3v2_1_hand): down = (0,1,0,0) (as g1, gripper width along world y),
down_rot90 = gripper down with width along world x. Hand-link target = TCP target + R_hand @ (0,0,-0.1034).
Output goal1/b2a/g2/ik.json: per orientation and line, IK success, q and joint-limit margins for 21 points (A..B)."""
import json, os, sys
import numpy as np, torch
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts"))
import step2_scene as S
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
import xml.etree.ElementTree as ET
S.set_scene("g1"); ta = TensorDeviceType()
L = json.load(open(os.path.expanduser("~/effmass_mpc/goal1/b2a/g2/lines.json")))
cfg = S.build_robot_cfg(); rc = RobotConfig.from_dict(cfg, ta)
ik = IKSolver(IKSolverConfig.load_from_robot_config(rc, S.build_world_config(), num_seeds=64, self_collision_check=True, self_collision_opt=True, tensor_args=ta,
                                                    use_cuda_graph=False, position_threshold=0.002, rotation_threshold=0.02))
root = ET.parse(os.path.expanduser("~/effmass_mpc/goal1/fr3v2_1/fr3v2_1_curobo_narrow.urdf")).getroot()
lim = {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper"))) for j in root.findall("joint") if j.get("type") == "revolute"}
LO = np.array([lim[f"fr3v2_1_joint{i}"][0] for i in range(1, 8)]); HI = np.array([lim[f"fr3v2_1_joint{i}"][1] for i in range(1, 8)])


def qmat(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


ORI = {"down": [0.0, 1.0, 0.0, 0.0], "down_rot90": [0.0, 0.7071067811865476, 0.7071067811865476, 0.0]}
out = {}
for on, q4 in ORI.items():
    Rh = qmat(*q4); off = Rh @ np.array([0, 0, -0.1034]); out[on] = {}
    for nm, ln in L["lines"].items():
        A, B = np.array(ln["A"]), np.array(ln["B"]); s = np.linspace(0, 1, 21); T = A + np.outer(s, B - A); P = T + off
        pose = Pose(position=torch.tensor(P, **ta.as_torch_dict()).float(), quaternion=torch.tensor([q4] * len(P), **ta.as_torch_dict()).float())
        r = ik.solve_batch(pose); ok = r.success.view(-1).cpu().numpy(); q = r.solution.view(len(P), -1).cpu().numpy()[:, :7]
        mg = np.minimum(q - LO, HI - q)
        out[on][nm] = dict(success=ok.tolist(), q=q.round(5).tolist(), tcp=T.round(5).tolist(), hand_target=P.round(5).tolist(), min_margin_per_point=mg.min(1).round(4).tolist(),
                           worst_joint_per_point=(mg.argmin(1) + 1).tolist())
        print(on, nm, "success", int(ok.sum()), "/", len(ok), "min margin", float(mg.min().round(4)), "at joint", int(mg.min(0).argmin() + 1), flush=True)
json.dump(dict(orientations=ORI, tcp_offset=0.1034, lines=L["lines"], ik=out, limits=dict(lo=LO.tolist(), hi=HI.tolist())), open(os.path.expanduser("~/effmass_mpc/goal1/b2a/g2/ik.json"), "w"), indent=1)

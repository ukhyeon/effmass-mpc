"""lv_tip_ik2.py <tag> <jobs_json> <out_json> - live cmp tip: IK (gripper down) along candidate tip lines for offset / base shift calibration
(Isaac python via run_isaac.sh, world = robot table + floor). jobs: list of {"motion": name, "offset": m, "shift": [dx, dy], "dz": m};
line centre = tip + offset * axis - shift (+ dz), points centre + s u, s = -0.30 .. +0.30 (1 cm), tip / axis / u from compare_tip/paths.json.
Output: per job the centre, success list and q (7)."""
import json, os, sys
import numpy as np, torch
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts"))
import step2_scene as S
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
S.set_scene("g1"); ta = TensorDeviceType(); T = os.path.expanduser(f"~/effmass_mpc/goal1/live/{sys.argv[1]}/compare_tip")
P = {m["name"]: m for m in json.load(open(f"{T}/paths.json"))["motions"]}; jobs = json.load(open(sys.argv[2]))
rc = RobotConfig.from_dict(S.build_robot_cfg(), ta)
ik = IKSolver(IKSolverConfig.load_from_robot_config(rc, S.build_world_config(), num_seeds=64, self_collision_check=True, self_collision_opt=True, tensor_args=ta,
                                                    use_cuda_graph=False, position_threshold=0.002, rotation_threshold=0.02))
q4 = [0.0, 1.0, 0.0, 0.0]; off = np.array([0, 0, 0.1034]); ss = np.round(np.arange(-0.30, 0.3001, 0.01), 3); out = dict(s=ss.tolist(), jobs=[])
for jb in jobs:
    m = P[jb["motion"]]; c = np.array(m["tip"]) + jb["offset"] * np.array(m["axis"]) - np.array(list(jb["shift"]) + [0.0]) + [0, 0, jb.get("dz", 0.0)]; u = np.array(m["u"])
    Pp = np.array([c + s * u for s in ss]) + off
    r = ik.solve_batch(Pose(position=torch.tensor(Pp, **ta.as_torch_dict()).float(), quaternion=torch.tensor([q4] * len(Pp), **ta.as_torch_dict()).float()))
    ok = r.success.view(-1).cpu().numpy().astype(bool)
    out["jobs"].append(dict(jb, centre=c.tolist(), success=ok.tolist(), q=r.solution.view(len(Pp), -1).cpu().numpy()[:, :7].round(6).tolist()))
json.dump(out, open(sys.argv[3], "w")); print("jobs", len(jobs))

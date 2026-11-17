"""b2a4_ik.py - cuRobo IK grid for the B2a-4 line calibration (Isaac python via run_isaac.sh, no SimulationApp; world = table + floor).
front: line x from 0.20 to 0.50 m (1 cm), z = front line height (fingertip height); top: x = top line x, z from 0.50 to 0.75 m (1 cm).
Each line: y = line centre + (-0.60 .. +0.60, 1 cm), gripper down, hand link = TCP + 0.1034 m. Output goal1/b2a/g2/b2a4_ik.json."""
import json, os, sys
import numpy as np, torch
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts"))
import step2_scene as S
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
S.set_scene("g1"); ta = TensorDeviceType(); H = os.path.expanduser("~")
L = json.load(open(f"{H}/effmass_mpc/goal1/b2a/g2/lines.json"))["lines"]
rc = RobotConfig.from_dict(S.build_robot_cfg(), ta)
ik = IKSolver(IKSolverConfig.load_from_robot_config(rc, S.build_world_config(), num_seeds=64, self_collision_check=True, self_collision_opt=True, tensor_args=ta,
                                                    use_cuda_graph=False, position_threshold=0.002, rotation_threshold=0.02))
q4 = [0.0, 1.0, 0.0, 0.0]; off = np.array([0, 0, 0.1034]); ys = np.round(np.arange(-0.60, 0.6001, 0.01), 3); out = {"dy": ys.tolist(), "front": {}, "top": {}}
cf, ct = np.array(L["front"]["center"]), np.array(L["top"]["center"])
cands = [("front", round(x, 2), np.array([x, cf[1], cf[2]])) for x in np.arange(0.20, 0.5001, 0.01)] + [("top", round(z, 2), np.array([ct[0], ct[1], z])) for z in np.arange(0.50, 0.7501, 0.01)]
if len(sys.argv) > 1:   # extra candidates "front:0.315" ..., merged into the existing json
    old = json.load(open(f"{H}/effmass_mpc/goal1/b2a/g2/b2a4_ik.json")); out = old
    cands = []
    for a in sys.argv[1:]:   # "front:x" | "top:z" | "front:x:z:key" (front line at x with height z, stored under key)
        f = a.split(":")
        if len(f) == 4:
            cands.append((f[0], f[3], np.array([float(f[1]), cf[1], float(f[2])])))
        else:
            cands.append((f[0], float(f[1]), np.array([float(f[1]), cf[1], cf[2]]) if f[0] == "front" else np.array([ct[0], ct[1], float(f[1])])))
for nm, key, c in cands:
    T = np.array([[c[0], c[1] + y, c[2]] for y in ys]); P = T + off
    r = ik.solve_batch(Pose(position=torch.tensor(P, **ta.as_torch_dict()).float(), quaternion=torch.tensor([q4] * len(P), **ta.as_torch_dict()).float()))
    kk = key if isinstance(key, str) else (f"{key:.2f}" if abs(round(key, 2) - key) < 1e-9 else f"{key:.3f}")
    out[nm][kk] = dict(center=c.tolist(), success=r.success.view(-1).cpu().numpy().tolist(), q=r.solution.view(len(P), -1).cpu().numpy()[:, :7].round(6).tolist())
    print(nm, kk, "IK success", int(np.sum(out[nm][kk]["success"])), "/", len(P), flush=True)
json.dump(out, open(f"{H}/effmass_mpc/goal1/b2a/g2/b2a4_ik.json", "w"))

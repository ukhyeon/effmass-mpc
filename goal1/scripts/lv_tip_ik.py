"""lv_tip_ik.py <tag> - live cmp tip: cuRobo IK along each motion path of compare_tip/paths.json (Isaac python via run_isaac.sh, no
SimulationApp; world = robot table + floor as scene g2). Points: centre + s u, s = -0.30 .. +0.30 m (1 cm), heights dz = 0, +-0.025,
+-0.05, +-0.075, +-0.10 m; gripper down (hand link = TCP + 0.1034 m, wxyz 0 1 0 0) as the existing scenes. Optional 2nd argument
"dx,dy" (m, Franka base frame; run_isaac.sh clears the environment): the virtual robot base moved by (dx, dy) = every point moved by
(-dx, -dy) before IK; output then ik_shift<dx>_<dy>.json. Writes compare_tip/ik.json: per motion, per dz: success list and q (7)."""
import json, os, sys
import numpy as np, torch
os.environ["STEP_ROBOT"] = "fr3v2_1"
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts"))
import step2_scene as S
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
S.set_scene("g1"); ta = TensorDeviceType(); _tg, _sub = (sys.argv[1].split("@") + ["compare_tip"])[:2]; T = os.path.expanduser(f"~/effmass_mpc/goal1/live/{_tg}/{_sub}")   # "tag@folder"
P = json.load(open(f"{T}/paths.json")); SH = sys.argv[2] if len(sys.argv) > 2 else "0,0"; sh = np.array([float(v) for v in SH.split(",")] + [0.0])
rc = RobotConfig.from_dict(S.build_robot_cfg(), ta)
ik = IKSolver(IKSolverConfig.load_from_robot_config(rc, S.build_world_config(), num_seeds=64, self_collision_check=True, self_collision_opt=True, tensor_args=ta,
                                                    use_cuda_graph=False, position_threshold=0.002, rotation_threshold=0.02))
q4 = [0.0, 1.0, 0.0, 0.0]; off = np.array([0, 0, 0.1034]); ss = np.round(np.arange(-0.30, 0.3001, 0.01), 3); dzs = [0.0, 0.025, -0.025, 0.05, -0.05, 0.075, -0.075, 0.10, -0.10]
out = dict(s=ss.tolist(), dz=dzs, base_shift=sh[:2].tolist(), motions={})
for m in P["motions"]:
    c, u = np.array(m["centre"]) - sh, np.array(m["u"]); out["motions"][m["name"]] = {}
    for dz in dzs:
        Tp = np.array([c + s * u + [0, 0, dz] for s in ss]); Pp = Tp + off
        r = ik.solve_batch(Pose(position=torch.tensor(Pp, **ta.as_torch_dict()).float(), quaternion=torch.tensor([q4] * len(Pp), **ta.as_torch_dict()).float()))
        ok = r.success.view(-1).cpu().numpy().astype(bool)
        out["motions"][m["name"]][f"{dz:+.3f}"] = dict(success=ok.tolist(), q=r.solution.view(len(Pp), -1).cpu().numpy()[:, :7].round(6).tolist())
        print(m["name"], f"dz {dz:+.3f}", "IK", int(ok.sum()), "/", len(ok), "ok s range", (float(ss[ok].min()), float(ss[ok].max())) if ok.any() else None, flush=True)
json.dump(out, open(f"{T}/ik" + ("" if SH == "0,0" else "_shift" + SH.replace(",", "_")) + ".json", "w"))

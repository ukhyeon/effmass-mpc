"""v2_posture.py - posture zone maps (offline, no MPC; system python via g1_run_human.sh py). Postures from the B2a static pose by changing only the
right elbow (SMPL joint 19, local y axis-angle): 0 (current), 30 and 90 deg requested; human data recomputed with the node code (g13_human_precompute.py
--shapes v2 --pose66-from) and the mesh with the node body model. Reports the actual elbow angle (angle between shoulder->elbow and elbow->wrist) and the
forearm direction; ours zone sections; front protrusion = normal-direction thickness along the arm axis toward the robot from the hand support point."""
import json, os, subprocess, sys
import numpy as np, torch
sys.path.insert(0, os.path.dirname(__file__))
import v2_zone_lib as ZL
H = os.path.expanduser("~"); D_ = f"{H}/effmass_mpc/goal1/data"; OUT = f"{H}/effmass_mpc/goal1/zone"; O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"
base = np.load(f"{D_}/human_b2a_reach_v2.npz"); P66 = base["pose66"].astype(np.float32).reshape(1, 66)
L = json.load(open(f"{ZL.G}/lines.json")); AX = np.array(L["arm_axis"]); AX /= np.linalg.norm(AX)
from effmass_human.human_inertia_core import HumanInertiaCore
from effmass_human import constants
from effmass_human.utils import batch_roteulerSMPL
core = HumanInertiaCore("cpu"); core.set_betas(np.zeros(10, np.float32)); T0 = np.load(f"{H}/hrc_ws/pose_npz/human_state_1781524848.npz")["trans"][0].astype(np.float32)


@torch.inference_mode()
def verts(pose66, trans3):
    pose_world, trans_world = core.aa2euler(torch.as_tensor(pose66).view(1, -1), torch.as_tensor(trans3).view(1, -1))
    full = torch.zeros((1, 72), dtype=torch.float32); full[:, constants.G2Hpose_idx] = pose_world; rot, _ = batch_roteulerSMPL(full)
    return (core.smplh.forward(betas=core.betas, rotmat=rot).vertices[0] + trans_world[0]).numpy().astype(np.float64)


res = {}
for tag, deg in (("e00", 0.0), ("e30", 30.0), ("e90", 90.0)):
    P = P66.copy(); f = f"{D_}/human_posture_{tag}_v2.npz"
    if deg > 0:
        P[0, 19 * 3: 19 * 3 + 3] = np.array([0.0, np.radians(deg), 0.0], np.float32)
        np.savez(f"{D_}/human_posture_{tag}_pose.npz", pose66=P)
        subprocess.run([sys.executable, f"{H}/effmass_mpc/goal1/scripts/g13_human_precompute.py", "--shapes", "v2", "--pose66-from", f"{D_}/human_posture_{tag}_pose.npz", "--out", f], check=True, capture_output=True)
    else:
        f = f"{D_}/human_b2a_reach_v2.npz"
    D = ZL.load(f); V = verts(P.reshape(-1), T0) @ ZL.R.T + ZL.t
    sh, el, wr = D["JP"][14], D["JD"][14], D["JD"][16]; ua, fa = (el - sh) / np.linalg.norm(el - sh), (wr - el) / np.linalg.norm(wr - el)
    ang = float(np.degrees(np.arccos(np.clip(ua @ fa, -1, 1))))
    th = {c: ZL.thick(D, AX, c)[0] for c in ("uniform", "ours")}; thu = {c: ZL.thick(D, np.array([0, 0, 1.0]), c)[0] for c in ("uniform", "ours")}
    res[tag] = dict(requested_deg=deg, file=f, elbow_angle_deg=ang, forearm_dir=fa.round(3).tolist(), upper_arm_dir=ua.round(3).tolist(), hand_com=D["COM"][18].round(3).tolist(),
                    front_thickness_m=th, front_protrusion_vs_uniform_m=th["ours"] - th["uniform"], up_thickness_m=thu)
    np.savez(f"{OUT}/posture_{tag}_data.npz", verts=V, faces=np.asarray(core.smplh.faces, np.int64)); print(tag, json.dumps({k: v for k, v in res[tag].items() if k != "file"}), flush=True)
json.dump(res, open(f"{O}/v2_posture.json", "w"), indent=1)

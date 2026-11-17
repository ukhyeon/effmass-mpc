"""on_wp2_postures.py - WP2 posture set (system python via g1_run_human.sh py; no MPC). Right-arm postures from the B2a static pose: right shoulder
(SMPL 17, 3 axis-angle), right elbow as a hinge (SMPL 19, local y only) and right wrist (SMPL 21, 3 axis-angle, regularised) optimised with the node body
model (HumanInertiaCore) to target upper-arm / forearm directions (and palm-up for the hand-over posture), in the scene g2 (Isaac) frame where the human
faces -x, right = +y, up = +z. Human data recomputed with the node code (g13_human_precompute.py --shapes v2 --pose66-from), base placement unchanged.
Checks: elbow flexion 0-145 deg (from the joint positions), shoulder rotation < 160 deg, wrist < 60 deg, no overlap of right-arm shapes with other shapes
(capsule distance > 0). Outputs goal1/data/human_on_<P>_v2.npz (+ _pose.npz), goal1/data/human_on_<P>_mesh.npz (vertices, display/figures),
logs/meff_isaac/on_wp2_postures.json."""
import json, os, subprocess, sys
import numpy as np, torch
from scipy.optimize import minimize
torch.set_num_threads(1)
sys.path.insert(0, os.path.dirname(__file__))
from effmass_human.human_inertia_core import HumanInertiaCore
from effmass_human import constants
from effmass_human.utils import batch_roteulerSMPL
from effmass_human.human_shapes import ShapeSet
import v2_zone_lib as ZL
H = os.path.expanduser("~"); D_ = f"{H}/effmass_mpc/goal1/data"; O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"
P66 = np.load(f"{D_}/human_b2a_reach_v2.npz")["pose66"].astype(np.float32).reshape(22, 3); T0 = np.load(f"{H}/hrc_ws/pose_npz/human_state_1781524848.npz")["trans"][0].astype(np.float32)
core = HumanInertiaCore(device="cpu"); core.set_betas(np.zeros(10, np.float32))
V2 = json.load(open(f"{D_}/shapes_v2.json")); E = V2["shapes"] + [V2["inactive"]] * (24 - len(V2["shapes"]))
core.shapes = ShapeSet([1] * 24, [e["segment"] for e in E], [e["p0_local"] for e in E], [e["p1_local"] for e in E], [e["radius"] for e in E], core.shapes.distal_local)
RSH, REL, RWR = 17, 19, 21; Ri, ti = ZL.R, ZL.t
f64 = lambda x: x.detach().cpu().numpy().astype(np.float64) if torch.is_tensor(x) else np.asarray(x, np.float64)
F_, S_, U_ = np.array([-1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])
nz = lambda v: v / np.linalg.norm(v)
ang = lambda a, b: float(np.arccos(np.clip(nz(a) @ nz(b), -1, 1)))


def pose_of(x):
    p = P66.copy(); p[RSH] = x[:3]; p[REL] = [0.0, x[3], 0.0]; p[RWR] = x[4:7]; return p


def fk(x):
    o = core.compute(pose_of(x).reshape(-1), T0, prev=None, dt=None); J = f64(o["joints"][0]) @ Ri.T + ti; Rw = f64(o["world_rot"][0]); Dd = np.asarray(o["distal"], float) @ Ri.T + ti
    return dict(sh=J[14], el=J[16], wr=J[18], tip=Dd[18], palm=Ri @ (Rw[18] @ np.array([0, -1.0, 0])), o=o)


g0 = fk(np.r_[P66[RSH], 0.0, P66[RWR]])
deg = np.radians
TARGETS = {   # name: (upper-arm dir, forearm dir, palm target or None, description)
    "P1_reach": None,
    "P2_bend45": (nz(np.cos(deg(45)) * F_ - np.sin(deg(45)) * U_), F_, None, "elbow ~45 deg, forearm forward, upper arm forward-down"),
    "P3_bend90": (-U_, F_, None, "elbow ~90 deg, forearm forward, upper arm down"),
    "P4_side": (S_, S_, None, "arm straight to the right side"),
    "P5_up": (nz(F_ + U_), nz(F_ + U_), None, "arm straight, raised 45 deg forward-up"),
    "P6_handover": (nz(np.cos(deg(60)) * F_ - np.sin(deg(60)) * U_), F_, U_, "hand-over: forearm forward, elbow ~60 deg, palm up"),
    "P7_down": (nz(np.cos(deg(40)) * F_ - np.sin(deg(40)) * U_), nz(np.cos(deg(40)) * F_ - np.sin(deg(40)) * U_), None, "arm straight, forward-down 40 deg toward a work table"),
}


def solve(t):
    ua_t, fa_t, palm_t, _ = t
    def obj(x):
        g = fk(x); c = ang(g["el"] - g["sh"], ua_t) ** 2 + ang(g["wr"] - g["el"], fa_t) ** 2 + 0.002 * np.sum(x[4:7] ** 2)
        c += (0.5 * ang(g["palm"], palm_t) ** 2) if palm_t is not None else 0.05 * ang(g["palm"], g0["palm"]) ** 2
        return c + 10.0 * min(x[3], 0.0) ** 2   # hinge sign: positive local y = flexion (v2_posture.py convention)
    best = None
    for th0 in (0.3, 0.8, 1.2, 1.6):
        x0 = np.r_[P66[RSH], th0, 0, 0, 0].astype(np.float64)
        r = minimize(obj, x0, method="Nelder-Mead", options=dict(xatol=1e-5, fatol=1e-9, maxiter=8000, maxfev=8000))
        if best is None or r.fun < best.fun:
            best = r
    return best.x, best.fun


@torch.inference_mode()
def verts(pose66):
    pw, tw = core.aa2euler(torch.as_tensor(pose66).view(1, -1), torch.as_tensor(T0).view(1, -1))
    full = torch.zeros((1, 72), dtype=torch.float32); full[:, constants.G2Hpose_idx] = pw; rot, _ = batch_roteulerSMPL(full)
    return (core.smplh.forward(betas=core.betas, rotmat=rot).vertices[0] + tw[0]).numpy().astype(np.float64)


def overlaps(o):
    sh = o["shapes"]; P0 = np.asarray(sh["p0"]); P1 = np.asarray(sh["p1"]); rad = np.asarray(sh["radius"]); seg = np.asarray(sh["segment"]); out = []
    arm = [k for k in range(24) if rad[k] > 0 and seg[k] in (14, 16, 18)]; oth = [k for k in range(24) if rad[k] > 0 and seg[k] not in (14, 16, 18, 11)]
    for a in arm:
        for b in oth:
            d = seg_seg(P0[a], P1[a], P0[b], P1[b]) - rad[a] - rad[b]
            out.append((d, a, b))
    return min(out)


def seg_seg(p1, q1, p2, q2):
    ts = np.linspace(0, 1, 41); A = p1[None] + ts[:, None] * (q1 - p1)[None]; B = p2[None] + ts[:, None] * (q2 - p2)[None]
    return float(np.linalg.norm(A[:, None] - B[None], axis=-1).min())


if __name__ == "__main__":
    res = {}; FACES = np.asarray(core.smplh.faces, np.int64)
    for name, t in TARGETS.items():
        if t is None:
            x = np.r_[P66[RSH], 0.0, P66[RWR]].astype(np.float64); P = P66.copy(); fun = 0.0; desc = "B2a static reach pose (reference)"
            f = f"{D_}/human_b2a_reach_v2.npz"
        else:
            x, fun = solve(t); P = pose_of(x); desc = t[3]; f = f"{D_}/human_on_{name}_v2.npz"
            np.savez(f"{D_}/human_on_{name}_pose.npz", pose66=P.reshape(1, 66).astype(np.float32))
            subprocess.run([sys.executable, f"{H}/effmass_mpc/goal1/scripts/g13_human_precompute.py", "--shapes", "v2", "--pose66-from", f"{D_}/human_on_{name}_pose.npz", "--out", f],
                           check=True, capture_output=True)
        g = fk(x); ua, fa = g["el"] - g["sh"], g["wr"] - g["el"]; flex = np.degrees(ang(ua, fa))
        ov = overlaps(g["o"]); sh_rot = float(np.degrees(np.linalg.norm(x[:3]))); wr_rot = float(np.degrees(np.linalg.norm(x[4:7])))
        ok = (0 <= flex <= 145) and x[3] >= -1e-3 and sh_rot < 160 and wr_rot < 60 and ov[0] > 0
        V = verts(P.reshape(-1)) @ Ri.T + ti; np.savez(f"{D_}/human_on_{name}_mesh.npz", verts=V.astype(np.float32), faces=FACES)
        res[name] = dict(description=desc, file=f, fit_cost=float(fun), elbow_flexion_deg=float(flex), elbow_hinge_rad=float(x[3]), shoulder_rot_deg=sh_rot, wrist_rot_deg=wr_rot,
                         upper_arm_dir=nz(ua).round(3).tolist(), forearm_dir=nz(fa).round(3).tolist(), palm_normal=nz(g["palm"]).round(3).tolist(),
                         upper_arm_err_deg=float(np.degrees(ang(ua, t[0]))) if t else 0.0, forearm_err_deg=float(np.degrees(ang(fa, t[1]))) if t else 0.0,
                         palm_up_err_deg=float(np.degrees(ang(g["palm"], U_))) if (t and t[2] is not None) else None,
                         min_arm_to_body_capsule_gap_m=float(ov[0]), closest_pair_shapes=[int(ov[1]), int(ov[2])], plausible=bool(ok),
                         shoulder=g["sh"].round(3).tolist(), elbow=g["el"].round(3).tolist(), wrist=g["wr"].round(3).tolist(), tip=g["tip"].round(3).tolist(), x=x.tolist())
        print(name, json.dumps({k: v for k, v in res[name].items() if k not in ("file", "x")}), flush=True)
    json.dump(res, open(f"{O}/on_wp2_postures.json", "w"), indent=1)

"""b2a_fig_traj.py - EE (TCP) trajectories of mass A seed 0 (solid) and iso seed 0 (dashed) on front and top, orthographic views with the
right hand / forearm mesh and the v2 hand capsule outlines. TCP = hand-link position - 0.1034 m along z (gripper pointing down)."""
import json, os
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon
H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; G = f"{H}/effmass_mpc/goal1/b2a/g2"; OUT = f"{H}/effmass_mpc/goal1/b2a"
tf = json.load(open(f"{G}/temp_transform.json")); Rm, tv = np.array(tf["R"]), np.array(tf["t"]); L = json.load(open(f"{G}/lines.json"))
MV = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_verts.npz"); V = MV["verts"][0].astype(float) @ Rm.T + tv; F = MV["faces"]; LAB = np.load(f"{OUT}/vertex_segment.npy")
Z = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_v2.npz"); SEG = Z["shape_segment"]; RAD = Z["shape_radius"]; P0 = Z["p0"][0] @ Rm.T + tv; P1 = Z["p1"][0] @ Rm.T + tv
keep = np.isin(LAB[F], [16, 18]).all(1); FF = F[keep]


def stadium(a, b, r, n=24):
    d = b - a; Ln = np.linalg.norm(d)
    if Ln < 1e-9:
        t = np.linspace(0, 2 * np.pi, 2 * n); return a + r * np.c_[np.cos(t), np.sin(t)]
    u = d / Ln; phi = np.arctan2(u[1], u[0]); tb = np.linspace(phi + np.pi / 2, phi - np.pi / 2, n); ta = np.linspace(phi - np.pi / 2, phi - 3 * np.pi / 2, n)
    return np.vstack([b + r * np.c_[np.cos(tb), np.sin(tb)], a + r * np.c_[np.cos(ta), np.sin(ta)]])


VIEWS = {"top view (x, y)": (0, 1, 2), "side view (y, z)": (1, 2, 0)}
fig, ax = plt.subplots(2, 2, figsize=(12, 10))
for pi, path in enumerate(("front", "top")):
    for vi, (vn, (ix, iy, iz)) in enumerate(VIEWS.items()):
        a = ax[pi, vi]; T = V[FF][:, :, [ix, iy]]; dep = V[FF][:, :, iz].mean(1); o = np.argsort(dep if iz != 2 else dep)
        a.add_collection(PolyCollection(T[o], facecolors=(0.87, 0.70, 0.58, 1.0), edgecolors=(0.6, 0.45, 0.35, 0.3), lw=0.2))
        for k in range(24):
            if SEG[k] in (16, 18) and RAD[k] > 0:
                a.add_patch(Polygon(stadium(P0[k][[ix, iy]], P1[k][[ix, iy]], RAD[k]), closed=True, facecolor=(0.3, 0.6, 0.95, 0.15), edgecolor=(0.15, 0.4, 0.85, 0.9), lw=0.8))
        ln = L["lines"][path]; A_, B_ = np.array(ln["A"]), np.array(ln["B"]); a.plot([A_[ix], B_[ix]], [A_[iy], B_[iy]], ":", color="gray", lw=1, label="design TCP line")
        for cond, sty, colr in (("massA", "-", "tab:red"), ("iso", "--", "tab:green")):
            f = f"{O}/b2a_{path}_{cond}_s0_b1.npz"
            if os.path.exists(f):
                ee = np.load(f)["rows"][:, 8:11]; tcp = ee - np.array([0, 0, 0.1034]); a.plot(tcp[:, ix], tcp[:, iy], sty, color=colr, lw=1.4, label=f"{cond} seed 0 TCP")
        a.set_aspect("equal"); a.grid(alpha=0.3); a.set_title(f"{path}: {vn}", fontsize=10); a.set_xlabel("xyz"[ix] + " [m]"); a.set_ylabel("xyz"[iy] + " [m]")
        if vi == 0:
            a.set_xlim(0.30, 0.80); a.set_ylim(-0.25, 0.25)
        else:
            a.set_xlim(-0.25, 0.25); a.set_ylim(0.30, 0.70)
ax[0, 0].legend(fontsize=8); fig.suptitle("B2a: TCP trajectories (mass A solid, iso dashed, seed 0) with the right hand"); fig.tight_layout(); fig.savefig(f"{OUT}/b2a_fig2_traj.png", dpi=120)
print("ok")

"""b2a5_fig_avoid.py - avoidance check figure from saved B2a-5 runs (system python via g1_run_human.sh py). Per scene, seed 0, off / iso /
uniform_matched / mass A: TCP path (thin) and the path of the closest robot sphere (thick; per step the MPC sphere closest to the v2 hand shapes,
B1 rows) over the hand mesh and hand capsules, zoomed to +-0.3 m around the hand COM; top (x, y) and side (x, z). Dot = closest moment, label = gap."""
import json, os, sys, csv
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import b2a4_figs as FG
SC = {"front": "front_0.38h1", "top": "top_0.455"}; CONDS = ["off", "iso", "unim", "massA"]; HC = BA.COMS[18]
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
fig, ax = plt.subplots(2, 2, figsize=(13, 12))
for pi, sc in enumerate(SC):
    for vi, (vn, ix, iy, iz) in enumerate((("top view (x, y)", 0, 1, 2), ("side view (x, z)", 0, 2, 1))):
        a = ax[pi, vi]; T = FG.FT.V[FG.FT.FF][:, :, [ix, iy]]; dep = FG.FT.V[FG.FT.FF][:, :, iz].mean(1); o = np.argsort(dep if iz == 2 else -dep)
        a.add_collection(PolyCollection(T[o], facecolors=(0.87, 0.70, 0.58, 1.0), edgecolors=(0.6, 0.45, 0.35, 0.3), lw=0.2))
        for k in BA.HAND:
            a.add_patch(Polygon(FG.FT.stadium(BA.P0[k][[ix, iy]], BA.P1[k][[ix, iy]], BA.RAD[k]), closed=True, facecolor=(0.3, 0.3, 0.3, 0.1), edgecolor=(0.2, 0.2, 0.2, 0.9), lw=0.8))
        for ci, c in enumerate(CONDS):
            tag = f"b2a5_{SC[sc]}_{c}_s0"; r = BA.result(tag); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])
            B = np.load(f"{BA.O}/{tag}_b1.npz")["rows"]; tcp = B[:, 8:11] - np.array([0, 0, 0.1034]); D = B[:, 12:36][:, BA.HAND]
            k = [BA.HAND[j] for j in D.argmin(1)]; s = [int(B[i, 36 + kk]) for i, kk in enumerate(k)]; C = np.array([BA.RI.compute(q)[0][si] for q, si in zip(Q, s)])
            i0 = int(D.min(1).argmin()); g = float(D.min(1)[i0]); col = FG.COL[c]
            near = D.min(1) < 0.25
            a.plot(tcp[:, ix], tcp[:, iy], "-" if c != "off" else "--", color=col, lw=1.6, label=f"{FG.LAB[c]}: TCP; dots = closest sphere; min gap {g*100:.1f} cm")
            a.plot(C[near, ix], C[near, iy], ".", color=col, ms=3.5, alpha=0.8)
            a.plot(C[i0, ix], C[i0, iy], "o", color=col, ms=10, mec="#0b0b0b", mew=1.2)
        a.set_xlim(HC[ix] - 0.3, HC[ix] + 0.3); a.set_ylim(HC[iy] - 0.3, HC[iy] + 0.3); a.set_aspect("equal"); a.grid(alpha=0.25, lw=0.5)
        a.set_title(f"{sc}-cal, seed 0: {vn}"); a.set_xlabel("xyz"[ix] + " [m]"); a.set_ylabel("xyz"[iy] + " [m]"); a.legend(fontsize=8, loc="lower right")
fig.suptitle("B2a-5 seed 0: TCP path (line), closest robot sphere within 0.25 m of the hand (small dots), closest moment (big dot)"); fig.tight_layout()
fig.savefig(f"{BA.OUT}/b2a5_fig5_avoid.png", dpi=120); print("ok")

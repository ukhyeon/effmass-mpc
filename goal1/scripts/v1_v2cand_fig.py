"""v1_v2cand_fig.py - V2 candidate lines over zone sections (system python3): ours (solid, filled by part) and uniform-small (dashed) boundaries.
(1) horizontal section at the front-line height with front candidates x 0.36 / 0.38; (2) horizontal section at the over-hand line height with the
x 0.50 / 0.53 candidates; (3) side section through the arm axis with every candidate as a cross-section marker. Output goal1/zone/v2_candidates.png."""
import os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
sys.path.insert(0, os.path.dirname(__file__))
import v1_zone_field as ZF
OUT = os.path.expanduser("~/effmass_mpc/goal1/zone"); c0 = ZF.COM[18]; HTOP = max(max(ZF.P0[k][2], ZF.P1[k][2]) + ZF.RAD[k] for k in ZF.SH if ZF.SEG[k] == 18)
FRONT = [(0.36, 0.4416), (0.38, 0.4416)]; OVER = [(0.50, 0.5345), (0.53, 0.5345)]; PCM = ListedColormap(["#c8d8ee", "#f3d2c1", "#e7e0b0"])   # hand, forearm, upper arm


def sec(ax_, axis, val, title, marks):
    a1 = np.arange(0.24, 0.98, 0.004); a2 = np.arange((c0[1] - 0.36) if axis == 2 else 0.10, (c0[1] + 0.36) if axis == 2 else 0.80, 0.004); A, Bm = np.meshgrid(a1, a2, indexing="ij")
    P = np.stack([A.ravel(), Bm.ravel() if axis == 2 else np.full(A.size, val), np.full(A.size, val) if axis == 2 else Bm.ravel()], 1)
    fo, _, kb = ZF.field(P, "ours"); fs, _, _ = ZF.field(P, "uniform_small"); part = np.array([{18: 0, 16: 1, 14: 2}[int(ZF.SEG[k])] for k in kb], float)
    ax_.pcolormesh(a1, a2, np.ma.masked_where(fo > 0, part).reshape(A.shape).T, cmap=PCM, vmin=-0.5, vmax=2.5, shading="auto")
    ax_.contour(a1, a2, fo.reshape(A.shape).T, levels=[0], colors="#eb6834", linewidths=2); ax_.contour(a1, a2, fs.reshape(A.shape).T, levels=[0], colors="#2a78d6", linewidths=1.8, linestyles="--")
    Sd = np.min([np.linalg.norm(P - (ZF.P0[k] + np.clip(((P - ZF.P0[k]) @ (ZF.P1[k] - ZF.P0[k])) / max((ZF.P1[k] - ZF.P0[k]) @ (ZF.P1[k] - ZF.P0[k]), 1e-12), 0, 1)[:, None] * (ZF.P1[k] - ZF.P0[k])), axis=1) - ZF.RAD[k] for k in ZF.SH], 0)
    ax_.contourf(a1, a2, (Sd <= 0).reshape(A.shape).T.astype(float), levels=[0.5, 1.5], colors=["#8a5a3a"])
    for m in marks:
        m(ax_)
    ax_.set_aspect("equal"); ax_.set_title(title, fontsize=10); ax_.set_xlabel("x [m] (robot at small x)"); ax_.set_ylabel("y [m]" if axis == 2 else "z [m]")


fig, ax = plt.subplots(1, 3, figsize=(20, 6.8))
sec(ax[0], 2, 0.4416, "(1) horizontal section z 0.442 (front lines, TCP moves +y)", [lambda a, x=x: (a.axvline(x, color="#0b0b0b", lw=1.6), a.text(x + 0.005, -0.30, f"front x {x:.2f}", rotation=90, fontsize=8)) for x, _ in FRONT])
sec(ax[1], 2, 0.5345, "(2) horizontal section z 0.535 (5 cm above the hand top; over-hand lines)", [lambda a, x=x: (a.axvline(x, color="#0b0b0b", lw=1.6), a.text(x + 0.005, -0.30, f"over x {x:.2f}", rotation=90, fontsize=8)) for x, _ in OVER])
sec(ax[2], 1, c0[1], "(3) side section through the arm axis (markers = candidate lines)", [lambda a: [a.plot(x, z, "o", color="#0b0b0b", ms=7) for x, z in FRONT + OVER]])
h = [plt.Line2D([], [], color="#eb6834", lw=2, label="ours boundary"), plt.Line2D([], [], color="#2a78d6", lw=1.8, ls="--", label="uniform-small boundary"),
     plt.Rectangle((0, 0), 1, 1, color="#c8d8ee", label="ours zone from hand"), plt.Rectangle((0, 0), 1, 1, color="#f3d2c1", label="ours zone from forearm"),
     plt.Rectangle((0, 0), 1, 1, color="#e7e0b0", label="ours zone from upper arm"), plt.Rectangle((0, 0), 1, 1, color="#8a5a3a", label="arm shapes (v2)")]
ax[2].legend(handles=h, fontsize=8, loc="lower right"); fig.suptitle("V2 scene candidates on the zone map (point robot at the TCP, robot m_r 4.6 kg assumed)")
fig.savefig(f"{OUT}/v2_candidates.png", dpi=115, bbox_inches="tight"); print("ok")

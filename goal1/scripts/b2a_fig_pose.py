"""b2a_fig_pose.py - orthographic side and top views of the B2a static pose: body mesh (shaded) + capsule outlines (one colour), with right-hand zoom."""
import json, os
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon
H = os.path.expanduser("~"); D = f"{H}/effmass_mpc/goal1/data"; OUT = f"{H}/effmass_mpc/goal1/b2a"
import sys
VER = sys.argv[1] if len(sys.argv) > 1 else "v1"
S = np.load(f"{D}/human_b2a_reach.npz" if VER == "v1" else f"{D}/human_b2a_reach_v2.npz"); M = np.load(f"{D}/human_b2a_reach_verts.npz"); rep = json.load(open(f"{OUT}/pose_report.json"))
V = M["verts"][0].astype(float); F = M["faces"]; P0, P1, R = S["p0"][0], S["p1"][0], S["shape_radius"]
up = np.array([0, 0, 1.0]); fwd = np.array(rep["forward_node_world"]); right = np.cross(fwd, up); right /= np.linalg.norm(right)
o = np.array(rep["shoulder"]); com = np.array(rep["hand_com"])
views = {"side": (fwd, up, right, "forward from right shoulder [m]", "up [m]"), "top": (fwd, -right, up, "forward from right shoulder [m]", "left [m]")}


def stadium(a, b, r, n=24):
    # outline of a capsule projected orthographically: two side lines + semicircle around b (through +u) + semicircle around a (through -u)
    d = b - a; L = np.linalg.norm(d)
    if L < 1e-9:
        t = np.linspace(0, 2 * np.pi, 2 * n); return a + r * np.c_[np.cos(t), np.sin(t)]
    u = d / L; phi = np.arctan2(u[1], u[0])
    tb = np.linspace(phi + np.pi / 2, phi - np.pi / 2, n); ta = np.linspace(phi - np.pi / 2, phi - 3 * np.pi / 2, n)
    return np.vstack([b + r * np.c_[np.cos(tb), np.sin(tb)], a + r * np.c_[np.cos(ta), np.sin(ta)]])


def draw(ax, ex, ey, ez, lim=None):
    X = np.c_[(V - o) @ ex, (V - o) @ ey]; Z = (V - o) @ ez
    T = X[F]; depth = Z[F].mean(1); order = np.argsort(depth)
    n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    shade = 0.45 + 0.55 * np.abs(n @ ez); col = np.c_[0.87 * shade, 0.70 * shade, 0.58 * shade]
    ax.add_collection(PolyCollection(T[order], facecolors=col[order], edgecolors="none"))
    for k in range(len(R)):
        if R[k] <= 0:
            continue
        a2 = np.array([(P0[k] - o) @ ex, (P0[k] - o) @ ey]); b2 = np.array([(P1[k] - o) @ ex, (P1[k] - o) @ ey])
        ax.add_patch(Polygon(stadium(a2, b2, R[k]), closed=True, facecolor=(0.30, 0.60, 0.95, 0.18), edgecolor=(0.15, 0.40, 0.85, 0.9), lw=0.8))
    ax.set_aspect("equal"); ax.grid(alpha=0.3)
    if lim is not None:
        c = np.array([(com - o) @ ex, (com - o) @ ey]); ax.set_xlim(c[0] - lim, c[0] + lim); ax.set_ylim(c[1] - lim, c[1] + lim)
    else:
        ax.set_xlim(X[:, 0].min() - 0.1, X[:, 0].max() + 0.1); ax.set_ylim(X[:, 1].min() - 0.1, X[:, 1].max() + 0.1)


for name, (ex, ey, ez, xl, yl) in views.items():
    fig, ax = plt.subplots(1, 2, figsize=(13, 7), gridspec_kw=dict(width_ratios=[1.4, 1]))
    draw(ax[0], ex, ey, ez); ax[0].set_xlabel(xl); ax[0].set_ylabel(yl); ax[0].set_title(f"B2a static pose, {name} view (mesh + shapes {VER}, {int((R > 0).sum())} capsules)")
    draw(ax[1], ex, ey, ez, lim=0.16); ax[1].set_xlabel(xl); ax[1].set_title(f"right hand zoom (shapes {VER})")
    fig.tight_layout(); fig.savefig(f"{OUT}/b2a_fig_pose_{name}.png" if VER == "v1" else f"{OUT}/shapes_v2_pose_{name}.png", dpi=120); plt.close(fig)
print("ok")

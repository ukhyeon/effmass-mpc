"""b2a4_figs.py - B2a-4 figures from b2a4_runs.json and the B1 rows (system python via g1_run_human.sh py).
fig1: seed 0 TCP trajectories per condition with the right hand / forearm mesh and v2 hand capsules, top view (x, y) and side view (x, z), both scenes.
fig2: closest-moment mu and u-vs-frontal angle per condition and scene (dots = seeds, labelled).
fig3: minimum gap (shape, mesh) and contact-risk index (max, sigma 0.02 m) per condition and scene.
Colours: reference categorical slots 1-4 (validated, light mode); off in grey; marker shape per condition as secondary encoding."""
import json, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon
from types import SimpleNamespace
H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; G = f"{H}/effmass_mpc/goal1/b2a/g2"; OUT = f"{H}/effmass_mpc/goal1/b2a"
_tf = json.load(open(f"{G}/temp_transform.json")); _R, _t = np.array(_tf["R"]), np.array(_tf["t"])      # same loading as b2a_fig_traj.py (not imported: it draws on import)
_MV = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_verts.npz"); _V = _MV["verts"][0].astype(float) @ _R.T + _t; _F = _MV["faces"]; _LAB = np.load(f"{OUT}/vertex_segment.npy")
_Z = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_v2.npz")


def _stadium(a, b, r, n=24):
    d = b - a; Ln = np.linalg.norm(d)
    if Ln < 1e-9:
        t = np.linspace(0, 2 * np.pi, 2 * n); return a + r * np.c_[np.cos(t), np.sin(t)]
    u = d / Ln; phi = np.arctan2(u[1], u[0]); tb = np.linspace(phi + np.pi / 2, phi - np.pi / 2, n); ta = np.linspace(phi - np.pi / 2, phi - 3 * np.pi / 2, n)
    return np.vstack([b + r * np.c_[np.cos(tb), np.sin(tb)], a + r * np.c_[np.cos(ta), np.sin(ta)]])


FT = SimpleNamespace(V=_V, FF=_F[np.isin(_LAB[_F], [16, 18]).all(1)], SEG=_Z["shape_segment"], RAD=_Z["shape_radius"], P0=_Z["p0"][0] @ _R.T + _t, P1=_Z["p1"][0] @ _R.T + _t, stadium=_stadium)
SC = {"front": "front_0.31h1", "top": "top_0.514"}
COL = {"off": "#8a8984", "iso": "#2a78d6", "massA": "#eb6834", "uni10": "#1baf7a", "unim": "#eda100"}
MK = {"off": "x", "iso": "o", "massA": "s", "uni10": "^", "unim": "D"}
LAB = {"off": "off", "iso": "iso", "massA": "mass A", "uni10": "uniform 0.10", "unim": "uniform_matched"}
CONDS = ["off", "iso", "massA", "uni10", "unim"]
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False, "axes.edgecolor": "#52514e", "axes.labelcolor": "#0b0b0b",
                     "xtick.color": "#52514e", "ytick.color": "#52514e"})

if __name__ == "__main__":
    # fig1
    VIEWS = {"top view (x, y)": (0, 1, 2), "side view (x, z)": (0, 2, 1)}
    fig, ax = plt.subplots(2, 2, figsize=(12, 9.5))
    for pi, sc in enumerate(SC):
        js = json.load(open(f"{G}/scene_b2a4_{SC[sc]}.json")); A_, B_ = np.array(js["tcp_A"]), np.array(js["tcp_B"])
        for vi, (vn, (ix, iy, iz)) in enumerate(VIEWS.items()):
            a = ax[pi, vi]; T = FT.V[FT.FF][:, :, [ix, iy]]; dep = FT.V[FT.FF][:, :, iz].mean(1); o = np.argsort(dep if iz == 2 else -dep)
            a.add_collection(PolyCollection(T[o], facecolors=(0.87, 0.70, 0.58, 1.0), edgecolors=(0.6, 0.45, 0.35, 0.3), lw=0.2))
            for k in range(24):
                if FT.SEG[k] in (16, 18) and FT.RAD[k] > 0:
                    a.add_patch(Polygon(FT.stadium(FT.P0[k][[ix, iy]], FT.P1[k][[ix, iy]], FT.RAD[k]), closed=True, facecolor=(0.3, 0.3, 0.3, 0.08), edgecolor=(0.3, 0.3, 0.3, 0.8), lw=0.7))
            a.plot([A_[ix], B_[ix]], [A_[iy], B_[iy]], ":", color="#52514e", lw=1, label="calibrated TCP line")
            for c in CONDS:
                f = f"{O}/b2a4_{SC[sc]}_{c}_s0_b1.npz"
                if os.path.exists(f):
                    tcp = np.load(f)["rows"][:, 8:11] - np.array([0, 0, 0.1034])
                    a.plot(tcp[:, ix], tcp[:, iy], "-" if c != "off" else "--", color=COL[c], lw=2, label=f"{LAB[c]}")
                    a.plot(tcp[::12, ix], tcp[::12, iy], MK[c], color=COL[c], ms=4, mec=(COL[c] if c == "off" else "#fcfcfb"), mew=0.6)
            a.set_aspect("equal"); a.grid(alpha=0.25, lw=0.5); a.set_title(f"{sc}-cal: {vn}"); a.set_xlabel("xyz"[ix] + " [m]"); a.set_ylabel("xyz"[iy] + " [m]")
            if vi == 0:
                a.set_xlim(0.25, 0.80); a.set_ylim(-0.45, 0.45)
            else:
                a.set_xlim(0.25, 0.80); a.set_ylim(0.30, 0.68)
    ax[0, 0].legend(fontsize=8, loc="lower right"); fig.suptitle("B2a-4: seed 0 TCP paths per condition with the right hand (markers every 12 steps)")
    fig.tight_layout(); fig.savefig(f"{OUT}/b2a4_fig1_traj.png", dpi=120); plt.close(fig)

    R = json.load(open(f"{O}/b2a4_runs.json"))


    def dots(ax_, sc, key, scale=1.0, ylab="", hline=None):
        for ci, c in enumerate(CONDS):
            rs = sorted([r for r in R if r["path"] == sc and r["cond"] == c], key=lambda r: r["seed"])
            if not rs:
                continue
            y = np.array([r[key] * scale if r.get(key) is not None else np.nan for r in rs]); xj = ci + np.linspace(-0.22, 0.22, len(rs))
            ax_.plot(xj, y, MK[c], color=COL[c], ms=8, mec=(COL[c] if c == "off" else "#fcfcfb"), mew=1.2 if c != "off" else 2.0, ls="none")
            for xx, yy, r in zip(xj, y, rs):
                ax_.annotate(str(r["seed"]), (xx, yy), textcoords="offset points", xytext=(0, 6), ha="center", fontsize=6, color="#52514e")
            ax_.plot([ci - 0.3, ci + 0.3], [np.nanmean(y)] * 2, color="#0b0b0b", lw=1)
        if hline is not None:
            ax_.axhline(hline, color="#52514e", lw=0.8, ls=":")
        ax_.set_xticks(range(len(CONDS))); ax_.set_xticklabels([LAB[c] for c in CONDS], rotation=15); ax_.set_ylabel(ylab); ax_.grid(axis="y", alpha=0.25, lw=0.5)


    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    for i, sc in enumerate(SC):
        dots(ax[i, 0], sc, "hand_mu", 1, "closest-moment mu [kg]"); ax[i, 0].set_title(f"{sc}-cal: mu at the shape-based minimum")
        dots(ax[i, 1], sc, "hand_shape_u_vs_front_deg", 1, "u vs frontal [deg]", hline=90); ax[i, 1].set_title(f"{sc}-cal: u angle at the minimum (0 = frontal, 90 = from above)")
    fig.suptitle("B2a-4: closest-moment mu and u angle (dots = seeds, bar = mean)"); fig.tight_layout(); fig.savefig(f"{OUT}/b2a4_fig2_mu_u.png", dpi=120); plt.close(fig)

    fig, ax = plt.subplots(2, 3, figsize=(15, 8))
    for i, sc in enumerate(SC):
        dots(ax[i, 0], sc, "hand_shape_gap", 100, "min gap, shape [cm]", hline=0); ax[i, 0].set_title(f"{sc}-cal: minimum hand gap (shape)")
        dots(ax[i, 1], sc, "hand_mesh_gap", 100, "min gap, mesh [cm]", hline=0); ax[i, 1].set_title(f"{sc}-cal: minimum hand gap (mesh)")
        dots(ax[i, 2], sc, "risk_max_s02", 1, "risk index max [m/s]"); ax[i, 2].set_title(f"{sc}-cal: contact-risk index (sigma 0.02 m)")
    fig.suptitle("B2a-4: minimum gap and contact-risk index (dots = seeds, bar = mean)"); fig.tight_layout(); fig.savefig(f"{OUT}/b2a4_fig3_gap_risk.png", dpi=120); plt.close(fig)
    print("ok")

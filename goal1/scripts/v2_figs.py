"""v2_figs.py - V2 figures (system python via g1_run_human.sh py). (a) zone sections (uniform | ours) with the seed 0 paths of the same condition on both
scenes (closest robot sphere centre = dots, TCP = line); (b) gain knob: ours boundaries for k 0 / 0.05 / 0.10 / 0.15 with the front-cal seed 0 paths,
and k vs front hand gap; (c) normal-direction zone thickness (uniform must be 6.5 cm everywhere). Output goal1/zone/v2_*.png, logs v2_thickness.json."""
import json, os, sys, csv
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import v2_zone_lib as ZL
O = BA.O; OUT = os.path.expanduser("~/effmass_mpc/goal1/zone"); D = ZL.load(os.path.expanduser("~/effmass_mpc/goal1/data/human_b2a_reach_v2.npz")); c0 = D["COM"][18]
L = json.load(open(f"{BA.G}/lines.json")); AX = np.array(L["arm_axis"]); AX /= np.linalg.norm(AX)
SC = {"front": "front_0.38h1", "top": "top_0.455"}; R = {(r["scene"], r["cond"], r["seed"]): r for r in json.load(open(f"{O}/v2_runs.json"))}
V = BA.V; FA = BA.F[np.isin(BA.LAB[BA.F], [14, 16, 18]).all(1)]
plt.rcParams.update({"font.size": 9})


def mesh_slice(axis, val):
    T = V[FA]; s = T[:, :, axis] - val; segs = []
    for tri, sv in zip(T, s):
        if (sv > 0).all() or (sv < 0).all():
            continue
        pts = [tri[i] + sv[i] / (sv[i] - sv[j]) * (tri[j] - tri[i]) for i, j in ((0, 1), (1, 2), (2, 0)) if (sv[i] > 0) != (sv[j] > 0)]
        if len(pts) == 2:
            segs.append(np.array(pts)[:, [i for i in range(3) if i != axis]])
    return segs


def path(tag):
    r = BA.result(tag); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))]); B = np.load(f"{O}/{tag}_b1.npz")["rows"]
    Dh = B[:, 12:36][:, BA.HAND]; k = [BA.HAND[j] for j in Dh.argmin(1)]; s = [int(B[i, 36 + kk]) for i, kk in enumerate(k)]
    C = np.array([BA.RI.compute(q)[0][si] for q, si in zip(Q, s)]); n = (r["arrival_list"][0]["step"] + 18) if r["arrival_list"] else len(Q)
    return C[:n], (B[:, 8:11] - np.array([0, 0, 0.1034]))[:n], Dh.min(1)[:n]


def section(ax_, axis, val, cond, k=0.10, fill=True):
    a1 = np.arange(0.24, 0.98, 0.004); a2 = np.arange(c0[1] - 0.36, c0[1] + 0.36, 0.004) if axis == 2 else np.arange(0.12, 0.80, 0.004); A, Bm = np.meshgrid(a1, a2, indexing="ij")
    P = np.stack([A.ravel(), Bm.ravel() if axis == 2 else np.full(A.size, val), np.full(A.size, val) if axis == 2 else Bm.ravel()], 1); f, mu, _ = ZL.field(D, P, cond, k)
    if fill:
        ax_.pcolormesh(a1, a2, np.ma.masked_where(f > 0, mu).reshape(A.shape).T, cmap="coolwarm", vmin=0.5, vmax=3.0, shading="auto", alpha=0.55) if cond == "ours" else \
            ax_.contourf(a1, a2, (f <= 0).reshape(A.shape).T.astype(float), levels=[0.5, 1.5], colors=["#d6dbe3"])
    ax_.add_collection(LineCollection(mesh_slice(axis, val), colors="#7a4a2a", linewidths=0.8)); ax_.set_aspect("equal"); ax_.set_xlim(a1[0], a1[-1]); ax_.set_ylim(a2[0], a2[-1])
    return a1, a2, f.reshape(A.shape)


# (a)
fig, ax = plt.subplots(2, 2, figsize=(15, 12)); COLC = {"u065": "#2a78d6", "gain": "#eb6834"}
for j, (cond, zc, lab) in enumerate((("u065", "uniform", "Uniform clearance (eta 6.5 cm)"), ("gain", "ours", "Directional effective mass (ours, k 0.10)"))):
    for i, (axis, val, vn) in enumerate(((2, c0[2], "top section z = hand COM height"), (1, c0[1], "side section through the arm axis"))):
        a = ax[i, j]; a1, a2, f = section(a, axis, val, zc); a.contour(a1, a2, f.T, levels=[0], colors="#0b0b0b", linewidths=1.5); o = [q for q in range(3) if q != axis]
        for sc, ls in (("front", "-"), ("top", "--")):
            C, T, g = path(f"v1_{SC[sc]}_{cond}_s0"); near = g < 0.25
            a.plot(T[:, o[0]], T[:, o[1]], ls, color=COLC[cond], lw=1.6, label=f"{sc}-cal TCP (min gap {g.min()*100:.1f} cm)"); a.plot(C[near, o[0]], C[near, o[1]], ".", color=COLC[cond], ms=3)
            i0 = int(g.argmin()); a.plot(C[i0, o[0]], C[i0, o[1]], "o", color=COLC[cond], ms=9, mec="#0b0b0b")
        a.set_title(f"{lab}\n{vn}", fontsize=10); a.set_xlabel("xyz"[o[0]] + " [m]"); a.set_ylabel("xyz"[o[1]] + " [m]"); a.legend(fontsize=8, loc="lower right")
fig.suptitle("(a) Zone (black boundary; ours coloured by mu) and seed 0 paths: TCP line, closest robot sphere dots (< 0.25 m), big dot = closest moment\nzone drawn for a point robot, m_r 4.6 kg")
fig.savefig(f"{OUT}/v2_a_paths_zone.png", dpi=110, bbox_inches="tight"); plt.close(fig)

# (b)
KS = (("gk000", 0.0), ("gk005", 0.05), ("gain", 0.10), ("gk015", 0.15)); CK = ["#2a78d6", "#1baf7a", "#eb6834", "#a3341b"]
fig, ax = plt.subplots(1, 3, figsize=(20, 6.3), gridspec_kw=dict(width_ratios=[1.2, 1.2, 0.8]))
for i, (axis, val) in enumerate(((2, c0[2]), (1, c0[1]))):
    a = ax[i]; a1, a2, _ = section(a, axis, val, "ours", 0.10, fill=False); o = [q for q in range(3) if q != axis]
    for (cond, k), col in zip(KS, CK):
        P = np.stack(np.meshgrid(a1, a2, indexing="ij"), -1).reshape(-1, 2); P3 = np.zeros((len(P), 3)); P3[:, o[0]] = P[:, 0]; P3[:, o[1]] = P[:, 1]; P3[:, axis] = val
        f, _, _ = ZL.field(D, P3, "ours", k); a.contour(a1, a2, f.reshape(len(a1), len(a2)).T, levels=[0], colors=[col], linewidths=1.4, linestyles="--")
        C, T, g = path(f"v1_{SC['front']}_{cond}_s0"); a.plot(T[:, o[0]], T[:, o[1]], "-", color=col, lw=1.8, label=f"k {k:.2f}: min gap {g.min()*100:.1f} cm")
    a.set_title(("top" if axis == 2 else "side") + " section, front-cal seed 0 (dashed = zone boundary for that k)"); a.set_xlabel("xyz"[o[0]] + " [m]"); a.set_ylabel("xyz"[o[1]] + " [m]"); a.legend(fontsize=8)
ks = [k for _, k in KS]; gs = [R[("front", c, 0)]["gap_shape"] * 100 for c, _ in KS]; gm = [R[("front", c, 0)]["gap_mesh"] * 100 for c, _ in KS]
ax[2].plot(ks, gs, "o-", color="#0b0b0b", label="shape-based"); ax[2].plot(ks, gm, "s--", color="#52514e", label="mesh-based"); ax[2].set_xlabel("k_gain [m]"); ax[2].set_ylabel("front-cal minimum hand gap [cm]")
ax[2].grid(alpha=0.3); ax[2].legend(); ax[2].set_title("gain knob vs front gap (seed 0)")
fig.suptitle("(b) Gain knob: k_gain 0 / 0.05 / 0.10 / 0.15 (zone drawn for a point robot, m_r 4.6 kg)"); fig.savefig(f"{OUT}/v2_b_gain_knob.png", dpi=110, bbox_inches="tight"); plt.close(fig)

# (c)
DIRS = {"front 0 deg": AX, "45 deg (front-up)": AX + np.array([0, 0, 1.0]), "up 90 deg (back of hand)": np.array([0, 0, 1.0]), "outer side 90 deg (+y)": np.array([0, 1.0, 0]), "down 90 deg (palm)": np.array([0, 0, -1.0])}
FM = [k for k in D["SH"] if D["SEG"][k] == 16]; tab = {}
for cond in ("uniform", "ours"):
    tab[cond] = {nm: ZL.thick(D, d, cond)[0] for nm, d in DIRS.items()}
    tab[cond]["forearm up"] = ZL.thick(D, np.array([0, 0, 1.0]), cond, shapes=FM)[0]; tab[cond]["forearm outer side"] = ZL.thick(D, np.array([0, 1.0, 0]), cond, shapes=FM)[0]
json.dump(dict(thickness_m=tab, note="normal-direction thickness from the support point of the hand (or forearm) shapes along the direction"), open(f"{O}/v2_thickness.json", "w"), indent=1)
fig, a = plt.subplots(figsize=(10, 4.5)); nm = list(tab["uniform"]); xx = np.arange(len(nm))
a.bar(xx - 0.2, [tab["uniform"][n] * 100 for n in nm], 0.38, color="#2a78d6", label="uniform (eta 6.5 cm)"); a.bar(xx + 0.2, [tab["ours"][n] * 100 for n in nm], 0.38, color="#eb6834", label="ours (gain k 0.10)")
for x_, n in zip(xx, nm):
    a.text(x_ - 0.2, tab["uniform"][n] * 100 + 0.3, f"{tab['uniform'][n]*100:.1f}", ha="center", fontsize=8); a.text(x_ + 0.2, tab["ours"][n] * 100 + 0.3, f"{tab['ours'][n]*100:.1f}", ha="center", fontsize=8)
a.set_xticks(xx); a.set_xticklabels(nm, rotation=15, fontsize=8); a.set_ylabel("zone thickness along the surface normal [cm]"); a.legend(); a.grid(axis="y", alpha=0.3)
a.set_title("(c) Zone thickness from the arm surface (point robot, m_r 4.6 kg)"); fig.savefig(f"{OUT}/v2_c_thickness.png", dpi=120, bbox_inches="tight"); plt.close(fig)
for c, v in tab.items():
    print(c, {k: round(x * 100, 1) for k, x in v.items()})

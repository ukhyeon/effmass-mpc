"""v1_zone.py - avoidance zone map (offline, no MPC; system python3 with numpy / matplotlib / scikit-image).
Point p = robot sphere centre (radius 0). For each right-arm v2 shape s (3 hand capsules, forearm, upper arm; static B2a pose, scene g2 frame):
d_s(p) = |p - q_s(p)| - r_s (q_s = closest point on the shape axis), u = (q_s - p)/|q_s - p|, m_h = 1/(u' L_seg u) (segment inverse inertia),
mu = 1/(1/m_h + 1/4.6) (robot side: representative m_r 4.6 kg, isotropic - assumption), eta_s(p) by condition. Zone = {p: min_s (d_s - eta_s) <= 0}
(same meaning as max_all). Conditions: uniform-small (eta = d_base 0.065), uniform-large (eta = mean front gain eta of v1_gain_table, all shapes and
directions), ours (gain: clamp(0.065 + 0.10 max(sqrt(mu/0.53) - 1, 0), 0.065, 0.25)). Grid 5 mm, box +-0.35 m around the hand COM, x extended to the elbow.
Outputs goal1/zone/: zone_slices.png (a, b), zone_3d.png (c), zone_overlay.png (d), zone_<cond>_mesh.npz (marching cubes, world frame, vertex mu),
logs/meff_isaac/v1_zone.json (thickness table)."""
import json, os
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from skimage.measure import marching_cubes
H = os.path.expanduser("~"); G = f"{H}/effmass_mpc/goal1/b2a/g2"; OUT = f"{H}/effmass_mpc/goal1/zone"; O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"
tf = json.load(open(f"{G}/temp_transform.json")); R, t = np.array(tf["R"]), np.array(tf["t"]); L = json.load(open(f"{G}/lines.json")); AX = np.array(L["arm_axis"]); AX /= np.linalg.norm(AX)
Z = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_v2.npz"); SEG = Z["shape_segment"]; RAD = Z["shape_radius"].astype(float)
P0 = Z["p0"][0].astype(float) @ R.T + t; P1 = Z["p1"][0].astype(float) @ R.T + t; COM = Z["com"][0].astype(float) @ R.T + t
LAM = {g: R @ Z["inv"][0][g].astype(float) @ R.T for g in (14, 16, 18)}
SH = [k for k in range(24) if RAD[k] > 0 and SEG[k] in (14, 16, 18)]; HAND = [k for k in SH if SEG[k] == 18]
MV = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_verts.npz"); V = MV["verts"][0].astype(float) @ R.T + t; F = MV["faces"]; LAB = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy")
FA = F[np.isin(LAB[F], [14, 16, 18]).all(1)]
GT = json.load(open(f"{O}/v1_gain_table.json")); ETA_L = float(np.mean([GT["poses"]["front_s0"]["eta_gain"], GT["poses"]["front_s1"]["eta_gain"]]))
DB, KG, MR, EM, MROB = 0.065, 0.10, 0.53, 0.25, 4.6
CONDS = {"uniform_small": "uniform-small (eta 6.5 cm)", "uniform_large": f"uniform-large (eta {ETA_L*100:.1f} cm)", "ours": "ours (gain, directional mu)"}


def field(Pts, cond):
    """Pts (N,3) -> f = min_s (d_s - eta_s), mu of the arg-min shape, index of the arg-min shape"""
    fbest = np.full(len(Pts), np.inf); mub = np.zeros(len(Pts)); kb = np.full(len(Pts), -1)
    for k in SH:
        a, b = P0[k], P1[k]; ab = b - a; tt = np.clip(((Pts - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = a + tt[:, None] * ab; v = q - Pts; n = np.linalg.norm(v, axis=1)
        d = n - RAD[k]; u = v / np.maximum(n, 1e-9)[:, None]; mh = 1.0 / np.maximum(np.einsum("ni,ij,nj->n", u, LAM[SEG[k]], u), 1e-12); mu = 1.0 / (1.0 / mh + 1.0 / MROB)
        e = np.full(len(Pts), DB) if cond == "uniform_small" else np.full(len(Pts), ETA_L) if cond == "uniform_large" else np.clip(DB + KG * np.maximum(np.sqrt(mu / MR) - 1, 0), DB, EM)
        f = d - e; m = f < fbest; fbest[m] = f[m]; mub[m] = mu[m]; kb[m] = k
    return fbest, mub, kb


c0 = COM[18]; lo = c0 - 0.35; hi = c0 + 0.35; hi[0] = max(hi[0], max(P0[k][0] for k in SH if SEG[k] == 16) + 0.05)
xs, ys, zs = [np.arange(lo[i], hi[i] + 1e-9, 0.005) for i in range(3)]
X, Y, Zg = np.meshgrid(xs, ys, zs, indexing="ij"); P = np.stack([X.ravel(), Y.ravel(), Zg.ravel()], 1)
FLD = {}
for c in CONDS:
    f, mu, kb = field(P, c); FLD[c] = (f.reshape(X.shape), mu.reshape(X.shape), kb.reshape(X.shape)); print("field", c, "zone voxels", int((f <= 0).sum()), flush=True)


def mesh_slice(axis, val):
    """triangle-plane intersection segments of the arm mesh with plane coord[axis] = val; returns segments in the other two coords"""
    T = V[FA]; s = T[:, :, axis] - val; segs = []
    for tri, sv in zip(T, s):
        if (sv > 0).all() or (sv < 0).all():
            continue
        pts = []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            if (sv[i] > 0) != (sv[j] > 0):
                w = sv[i] / (sv[i] - sv[j]); pts.append(tri[i] + w * (tri[j] - tri[i]))
        if len(pts) == 2:
            segs.append(np.array(pts))
    o = [i for i in range(3) if i != axis]; return [sg[:, o] for sg in segs]


def section(ax_, cond, axis, val, color_mu, title):
    ia = int(np.argmin(np.abs([xs, ys, zs][axis] - val))); sl = [slice(None)] * 3; sl[axis] = ia; f, mu, _ = [a[tuple(sl)] for a in FLD[cond]]
    o = [i for i in range(3) if i != axis]; A1, A2 = [xs, ys, zs][o[0]], [xs, ys, zs][o[1]]
    if color_mu:
        mm = np.ma.masked_where(f > 0, mu); pc = ax_.pcolormesh(A1, A2, mm.T, cmap="coolwarm", vmin=0.5, vmax=3.0, shading="auto")
    else:
        ax_.contourf(A1, A2, (f <= 0).T.astype(float), levels=[0.5, 1.5], colors=["#c3c2b7"]); pc = None
    ax_.contour(A1, A2, f.T, levels=[0], colors="#0b0b0b", linewidths=1.6)
    ax_.add_collection(LineCollection(mesh_slice(axis, val), colors="#7a4a2a", linewidths=0.8))
    ax_.set_aspect("equal"); ax_.set_xlim(A1[0], A1[-1]); ax_.set_ylim(A2[0], A2[-1]); ax_.set_title(title, fontsize=9); ax_.set_xlabel("xyz"[o[0]] + " [m]"); ax_.set_ylabel("xyz"[o[1]] + " [m]")
    ax_.annotate("robot side", xy=(A1[0] + 0.02, A2[-1] - 0.03), fontsize=8, color="#52514e") if o[0] == 0 else None
    return pc


plt.rcParams.update({"font.size": 9})
fig, ax = plt.subplots(2, 3, figsize=(17, 10.5))
for j, (c, lab) in enumerate(CONDS.items()):
    pc = section(ax[0, j], c, 2, c0[2], c == "ours", f"(a) top section z = {c0[2]:.3f} m (hand COM height)\n{lab}")
    pc2 = section(ax[1, j], c, 1, c0[1], c == "ours", f"(b) side section y = {c0[1]:.3f} m (through the arm axis)\n{lab}")
cb = fig.colorbar(pc, ax=ax[:, 2], fraction=0.03, pad=0.02); cb.set_label("mu in the zone [kg] (robot m_r 4.6 kg isotropic, assumed)")
fig.suptitle("Avoidance zone of the outstretched right arm (point robot; zone = any shape with gap <= eta). Black line = zone boundary, brown = arm mesh section", fontsize=11)
fig.savefig(f"{OUT}/zone_slices.png", dpi=120, bbox_inches="tight"); plt.close(fig)

fig, ax = plt.subplots(1, 2, figsize=(15, 6.5))
for a, axis, val, nm in ((ax[0], 2, c0[2], "top section (horizontal, hand COM height)"), (ax[1], 1, c0[1], "side section (vertical, through the arm axis)")):
    ia = int(np.argmin(np.abs([xs, ys, zs][axis] - val))); sl = [slice(None)] * 3; sl[axis] = ia; o = [i for i in range(3) if i != axis]; A1, A2 = [xs, ys, zs][o[0]], [xs, ys, zs][o[1]]
    for c, col, ls in (("uniform_small", "#2a78d6", "--"), ("ours", "#eb6834", "-")):
        a.contour(A1, A2, FLD[c][0][tuple(sl)].T, levels=[0], colors=col, linewidths=2.2, linestyles=ls)
    a.add_collection(LineCollection(mesh_slice(axis, val), colors="#7a4a2a", linewidths=0.8)); a.set_aspect("equal"); a.set_xlim(A1[0], A1[-1]); a.set_ylim(A2[0], A2[-1])
    a.set_title(nm); a.set_xlabel("xyz"[o[0]] + " [m]"); a.set_ylabel("xyz"[o[1]] + " [m]")
    a.plot([], [], "--", color="#2a78d6", lw=2.2, label="uniform-small (eta 6.5 cm)"); a.plot([], [], "-", color="#eb6834", lw=2.2, label="ours (gain)"); a.legend(fontsize=9, loc="lower right")
fig.suptitle("(d) Zone boundary: uniform-small vs ours (robot side at small x)"); fig.savefig(f"{OUT}/zone_overlay.png", dpi=120, bbox_inches="tight"); plt.close(fig)

from mpl_toolkits.mplot3d.art3d import Poly3DCollection
fig = plt.figure(figsize=(18, 6.5)); cmap = plt.get_cmap("coolwarm")
for j, (c, lab) in enumerate(CONDS.items()):
    f = FLD[c][0]; vt, fc, _, _ = marching_cubes(f, level=0.0, spacing=(0.005, 0.005, 0.005)); vt = vt + lo
    _, mu_v, _ = field(vt, c); np.savez(f"{OUT}/zone_{c}_mesh.npz", verts=vt.astype(np.float32), faces=fc.astype(np.int32), mu=mu_v.astype(np.float32), note="world frame (scene g2), marching cubes of min_s(d_s - eta_s) = 0")
    a = fig.add_subplot(1, 3, j + 1, projection="3d"); tri = vt[fc]; col = cmap(np.clip((mu_v[fc].mean(1) - 0.5) / 2.5, 0, 1)) if c == "ours" else "#9fb8d8"
    a.add_collection3d(Poly3DCollection(tri, facecolors=col, edgecolors="none", alpha=0.55)); arm = V[FA]
    a.add_collection3d(Poly3DCollection(arm[::3], facecolors="#c98a62", edgecolors="none", alpha=0.9))
    a.set_xlim(lo[0], hi[0]); a.set_ylim(lo[1], hi[1]); a.set_zlim(lo[2], hi[2]); a.set_box_aspect((hi - lo)); a.view_init(elev=22, azim=-60); a.set_title(f"(c) {lab}", fontsize=10)
    a.set_xlabel("x"); a.set_ylabel("y"); a.set_zlabel("z"); print("mesh", c, len(vt), "verts", flush=True)
fig.suptitle("Zone surface (marching cubes); ours coloured by mu (blue 0.5 kg -> red 3.0 kg)"); fig.savefig(f"{OUT}/zone_3d.png", dpi=110, bbox_inches="tight"); plt.close(fig)

# thickness table: margin beyond the arm surface along rays
DIRS = {"front 0 deg (toward robot)": AX, "45 deg (front-up)": (AX + np.array([0, 0, 1.0])) / np.linalg.norm(AX + np.array([0, 0, 1.0])), "up 90 deg (back of hand)": np.array([0, 0, 1.0]),
        "outer side 90 deg (+y)": np.array([0, 1.0, 0]), "down 90 deg (palm)": np.array([0, 0, -1.0])}
rr = np.arange(0, 0.6, 0.0005)


def thick(origin, d, cond):
    pts = origin[None] + rr[:, None] * d[None]; f, _, _ = field(pts, cond)
    sd = np.min([np.linalg.norm(pts - (P0[k] + np.clip(((pts - P0[k]) @ (P1[k] - P0[k])) / max((P1[k] - P0[k]) @ (P1[k] - P0[k]), 1e-12), 0, 1)[:, None] * (P1[k] - P0[k])), axis=1) - RAD[k] for k in SH], axis=0)
    i_z = int(np.argmax((f > 0) & (np.arange(len(rr)) > int(np.argmax(sd <= 0)))))   # first point outside the zone after the ray has been on the arm
    ins = np.nonzero(sd[:i_z] <= 0)[0]; i_s = ins[-1] + 1 if len(ins) else 0                # last exit from the arm shapes before that
    return float(rr[i_z] - rr[i_s])


fm = [k for k in SH if SEG[k] == 16][0]; mid = (P0[fm] + P1[fm]) / 2; tab = {}
for c in CONDS:
    tab[c] = {nm: thick(c0, d, c) for nm, d in DIRS.items()}; tab[c]["forearm mid, up"] = thick(mid, np.array([0, 0, 1.0]), c); tab[c]["forearm mid, outer side"] = thick(mid, np.array([0, 1.0, 0]), c)
json.dump(dict(eta_uniform_large=ETA_L, thickness_m=tab, note="margin beyond the arm surface (shape-based): along the ray from the hand COM (or forearm mid), zone exit minus the last exit from the arm shapes before it"), open(f"{O}/v1_zone.json", "w"), indent=1)
for c, v in tab.items():
    print(c, {k: round(x * 100, 1) for k, x in v.items()})

"""lv_remake_paths_zone.py - overnight2 bundle 2: v2_figs.py (a) (zone sections with the seed 0 paths of both V2 scenes) redrawn for uniform vs the
per-part reference rule: right column = per-part zone (eta = clip(0.065 + 0.10 (sqrt(mu (1/m_iso(seg) + 1/m_r)) - 1)+, 0.065, 0.25), point robot
4.6 kg) with the paths of the per-part runs pr_v2_<scene>_part_s0; left column = uniform with v1_<scene>_u065_s0 (as the original). Reads logs only;
writes goal1/public_draft/analysis_remake/figures/static_paths_zone.png (goal1/zone is not touched)."""
import json, os, sys, csv
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import v2_zone_lib as ZL
H = os.path.expanduser("~"); O = BA.O; OUT = f"{H}/effmass_mpc/goal1/public_draft/analysis_remake/figures"; D = ZL.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_v2.npz"); c0 = D["COM"][18]
SC = {"front": "front_0.38h1", "top": "top_0.455"}; V = BA.V; FA = BA.F[np.isin(BA.LAB[BA.F], [14, 16, 18]).all(1)]
ISO_NM = ["pelvis", "left_thigh", "right_thigh", "abdomen", "left_calf", "right_calf", "left_calf", "right_calf", "chest", "neck", "left_shoulder",
          "right_shoulder", "head", "left_upper_arm", "right_upper_arm", "left_lower_arm", "right_lower_arm", "left_hand", "right_hand"]
M_ISO = np.array([json.load(open(f"{H}/effmass_mpc/goal1/data/iso_15066.json"))[n]["m_h_iso"] for n in ISO_NM], float)
plt.rcParams.update({"font.size": 9})


def field(P, cond):
    if cond == "uniform":
        return ZL.field(D, P, "uniform")[:2]
    fb = np.full(len(P), np.inf); mub = np.zeros(len(P))
    for s in D["SH"]:
        a, b = D["P0"][s], D["P1"][s]; ab = b - a; tt = np.clip(((P - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = a + tt[:, None] * ab; v = q - P; n = np.linalg.norm(v, axis=1)
        d = n - D["RAD"][s]; u = v / np.maximum(n, 1e-9)[:, None]; g = D["SEG"][s]
        mh = 1.0 / np.maximum(np.einsum("ni,ij,nj->n", u, D["LAM"][g], u), 1e-12); mu = 1.0 / (1.0 / mh + 1.0 / ZL.MROB)
        eta = np.clip(ZL.DB + 0.10 * np.maximum(np.sqrt(mu * (1.0 / M_ISO[g] + 1.0 / ZL.MROB)) - 1, 0), ZL.DB, ZL.EM)
        f = d - eta; m = f < fb; fb[m] = f[m]; mub[m] = mu[m]
    return fb, mub


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


def section(ax_, axis, val, cond):
    a1 = np.arange(0.24, 0.98, 0.004); a2 = np.arange(c0[1] - 0.36, c0[1] + 0.36, 0.004) if axis == 2 else np.arange(0.12, 0.80, 0.004); A, Bm = np.meshgrid(a1, a2, indexing="ij")
    P = np.stack([A.ravel(), Bm.ravel() if axis == 2 else np.full(A.size, val), np.full(A.size, val) if axis == 2 else Bm.ravel()], 1); f, mu = field(P, cond)
    if cond == "part":
        ax_.pcolormesh(a1, a2, np.ma.masked_where(f > 0, mu).reshape(A.shape).T, cmap="coolwarm", vmin=0.5, vmax=3.0, shading="auto", alpha=0.55)
    else:
        ax_.contourf(a1, a2, (f <= 0).reshape(A.shape).T.astype(float), levels=[0.5, 1.5], colors=["#d6dbe3"])
    ax_.add_collection(LineCollection(mesh_slice(axis, val), colors="#7a4a2a", linewidths=0.8)); ax_.set_aspect("equal"); ax_.set_xlim(a1[0], a1[-1]); ax_.set_ylim(a2[0], a2[-1])
    return a1, a2, f.reshape(A.shape)


fig, ax = plt.subplots(2, 2, figsize=(15, 12)); COLC = {"uniform": "#2a78d6", "part": "#eb6834"}; res = {}
for j, (cond, lab, tagf) in enumerate((("uniform", "Uniform clearance (eta 6.5 cm)", "v1_{sc}_u065_s0"), ("part", "Per-part reference (k 0.10)", "pr_v2_{s}_part_s0"))):
    for i, (axis, val, vn) in enumerate(((2, c0[2], "top section z = hand COM height"), (1, c0[1], "side section through the arm axis"))):
        a = ax[i, j]; a1, a2, f = section(a, axis, val, cond); a.contour(a1, a2, f.T, levels=[0], colors="#0b0b0b", linewidths=1.5); o = [q for q in range(3) if q != axis]
        for sc, ls in (("front", "-"), ("top", "--")):
            C, T, g = path(tagf.format(sc=SC[sc], s=sc)); near = g < 0.25; res[f"{cond}/{sc}"] = round(float(g.min()) * 100, 1)
            a.plot(T[:, o[0]], T[:, o[1]], ls, color=COLC[cond], lw=1.6, label=f"{sc} TCP (min hand gap {g.min()*100:.1f} cm)"); a.plot(C[near, o[0]], C[near, o[1]], ".", color=COLC[cond], ms=3)
            i0 = int(g.argmin()); a.plot(C[i0, o[0]], C[i0, o[1]], "o", color=COLC[cond], ms=9, mec="#0b0b0b")
        a.set_title(f"{lab}\n{vn}", fontsize=10); a.set_xlabel("xyz"[o[0]] + " [m]"); a.set_ylabel("xyz"[o[1]] + " [m]"); a.legend(fontsize=8, loc="lower right")
fig.suptitle("Zone (black boundary; per-part coloured by mu) and seed 0 paths: TCP line, closest robot sphere dots (< 0.25 m), big dot = closest moment\n"
             "zone drawn for a point robot, m_r 4.6 kg; hand gap = b1 log, all hand shapes, whole run (not the pass-section rule)")
os.makedirs(OUT, exist_ok=True); fig.savefig(f"{OUT}/static_paths_zone.png", dpi=110, bbox_inches="tight"); plt.close(fig); print(json.dumps(res))

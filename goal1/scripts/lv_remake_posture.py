"""lv_remake_posture.py - overnight2 bundle 2: v2_posture_fig.py (elbow 0 / 30 / 90 deg zone sections) redrawn with the per-part reference rule
(eta = clip(0.065 + 0.10 (sqrt(mu (1/m_iso(seg) + 1/m_r)) - 1)+, 0.065, 0.25), m_r 4.6 kg point robot) instead of the previous single mu_ref 0.53 kg.
Same postures, sections and mesh outline as the original; reads goal1/zone/posture_*_data.npz and logs/meff_isaac/v2_posture.json (read only).
Writes goal1/public_draft/analysis_remake/figures/elbow_angle_zone.png and elbow_angle_zone.json (does not touch goal1/zone or the logs)."""
import json, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
sys.path.insert(0, os.path.dirname(__file__))
import v2_zone_lib as ZL
H = os.path.expanduser("~"); ZD = f"{H}/effmass_mpc/goal1/zone"; O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; R = json.load(open(f"{O}/v2_posture.json"))
OUT = f"{H}/effmass_mpc/goal1/public_draft/analysis_remake/figures"; LAB = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy")
ISO_NM = ["pelvis", "left_thigh", "right_thigh", "abdomen", "left_calf", "right_calf", "left_calf", "right_calf", "chest", "neck", "left_shoulder",
          "right_shoulder", "head", "left_upper_arm", "right_upper_arm", "left_lower_arm", "right_lower_arm", "left_hand", "right_hand"]
M_ISO = np.array([json.load(open(f"{H}/effmass_mpc/goal1/data/iso_15066.json"))[n]["m_h_iso"] for n in ISO_NM], float)


def field(D, P, cond):
    fb = np.full(len(P), np.inf); mub = np.zeros(len(P))
    for s in D["SH"]:
        a, b = D["P0"][s], D["P1"][s]; ab = b - a; tt = np.clip(((P - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = a + tt[:, None] * ab; v = q - P; n = np.linalg.norm(v, axis=1)
        d = n - D["RAD"][s]; u = v / np.maximum(n, 1e-9)[:, None]; g = D["SEG"][s]
        mh = 1.0 / np.maximum(np.einsum("ni,ij,nj->n", u, D["LAM"][g], u), 1e-12); mu = 1.0 / (1.0 / mh + 1.0 / ZL.MROB)
        eta = np.full(len(P), ZL.DB) if cond == "uniform" else np.clip(ZL.DB + 0.10 * np.maximum(np.sqrt(mu * (1.0 / M_ISO[g] + 1.0 / ZL.MROB)) - 1, 0), ZL.DB, ZL.EM)
        f = d - eta; m = f < fb; fb[m] = f[m]; mub[m] = mu[m]
    return fb, mub


def thick(D, dirv, cond):
    dirv = np.asarray(dirv, float) / np.linalg.norm(dirv); p0, _ = ZL.support(D, dirv, D["HAND"]); rr = np.arange(0, 0.6, 0.0005)
    f, _ = field(D, p0[None] + rr[:, None] * dirv[None], cond); return float(rr[int(np.argmax(f > 0))])


fig, ax = plt.subplots(1, 3, figsize=(20, 7)); res = {}
for j, tag in enumerate(("e00", "e30", "e90")):
    D = ZL.load(R[tag]["file"]); M = np.load(f"{ZD}/posture_{tag}_data.npz"); V, F = M["verts"], M["faces"]; FA = F[np.isin(LAB[F], [14, 16, 18]).all(1)]; c0 = D["COM"][18]
    fa = np.array(R[tag]["forearm_dir"]); fa /= np.linalg.norm(fa)
    th = {c: dict(tip=thick(D, fa, c), robot=thick(D, [-1.0, 0, 0], c)) for c in ("uniform", "part")}
    a1 = np.arange(c0[0] - 0.55, c0[0] + 0.55, 0.004); a2 = np.arange(c0[1] - 0.45, c0[1] + 0.45, 0.004); A, B = np.meshgrid(a1, a2, indexing="ij")
    P = np.stack([A.ravel(), B.ravel(), np.full(A.size, c0[2])], 1); fo, mu = field(D, P, "part"); fu, _ = field(D, P, "uniform")
    a = ax[j]; a.pcolormesh(a1, a2, np.ma.masked_where(fo > 0, mu).reshape(A.shape).T, cmap="coolwarm", vmin=0.5, vmax=3.0, shading="auto", alpha=0.6)
    a.contour(a1, a2, fo.reshape(A.shape).T, levels=[0], colors="#0b0b0b", linewidths=1.6); a.contour(a1, a2, fu.reshape(A.shape).T, levels=[0], colors="#2a78d6", linewidths=1.4, linestyles="--")
    T = V[FA]; s = T[:, :, 2] - c0[2]; segs = []
    for tri, sv in zip(T, s):
        if (sv > 0).all() or (sv < 0).all():
            continue
        pts = [tri[i] + sv[i] / (sv[i] - sv[k]) * (tri[k] - tri[i]) for i, k in ((0, 1), (1, 2), (2, 0)) if (sv[i] > 0) != (sv[k] > 0)]
        if len(pts) == 2:
            segs.append(np.array(pts)[:, :2])
    a.add_collection(LineCollection(segs, colors="#7a4a2a", linewidths=0.8)); a.set_aspect("equal"); a.set_xlim(a1[0], a1[-1]); a.set_ylim(a2[0], a2[-1])
    r = R[tag]; res[tag] = dict(elbow_angle_deg=r["elbow_angle_deg"], thickness_m=th)
    a.set_title(f"elbow {r['elbow_angle_deg']:.0f} deg; zone beyond uniform: toward robot {(th['part']['robot'] - th['uniform']['robot']) * 100:.1f} cm, "
                f"along fingertips {(th['part']['tip'] - th['uniform']['tip']) * 100:.1f} cm", fontsize=9)
    a.set_xlabel("x [m] (robot at small x)"); a.set_ylabel("y [m]")
ax[0].plot([], [], color="#0b0b0b", label="per-part reference boundary"); ax[0].plot([], [], "--", color="#2a78d6", label="uniform 6.5 cm"); ax[0].legend(fontsize=8, loc="lower left")
fig.suptitle("Per-part reference zone for three elbow angles (elbow-only change; horizontal section through the hand COM; point robot, m_r 4.6 kg; colour = mu)")
os.makedirs(OUT, exist_ok=True); fig.savefig(f"{OUT}/elbow_angle_zone.png", dpi=110, bbox_inches="tight"); json.dump(res, open(f"{OUT}/elbow_angle_zone.json", "w"), indent=1)
print(json.dumps(res))

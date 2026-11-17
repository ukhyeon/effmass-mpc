"""v2_posture_fig.py - (d) posture zone maps (system python via g1_run_human.sh py): ours zone (k 0.10) on the horizontal section through the hand COM for
elbow 0 / 30 / 90 deg (elbow-only change); protrusion along the robot-facing arm axis and along the hand's own pointing direction (forearm direction)."""
import json, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
sys.path.insert(0, os.path.dirname(__file__))
import v2_zone_lib as ZL
H = os.path.expanduser("~"); OUT = f"{H}/effmass_mpc/goal1/zone"; O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; R = json.load(open(f"{O}/v2_posture.json"))
LAB = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy")
fig, ax = plt.subplots(1, 3, figsize=(20, 7))
for j, tag in enumerate(("e00", "e30", "e90")):
    D = ZL.load(R[tag]["file"]); M = np.load(f"{OUT}/posture_{tag}_data.npz"); V, F = M["verts"], M["faces"]; FA = F[np.isin(LAB[F], [14, 16, 18]).all(1)]; c0 = D["COM"][18]
    fa = np.array(R[tag]["forearm_dir"]); fa /= np.linalg.norm(fa)
    for cond in ("uniform", "ours"):
        R[tag].setdefault("tip_dir_thickness_m", {})[cond] = ZL.thick(D, fa, cond)[0]
    R[tag]["tip_dir_protrusion_vs_uniform_m"] = R[tag]["tip_dir_thickness_m"]["ours"] - R[tag]["tip_dir_thickness_m"]["uniform"]
    a1 = np.arange(c0[0] - 0.55, c0[0] + 0.55, 0.004); a2 = np.arange(c0[1] - 0.45, c0[1] + 0.45, 0.004); A, B = np.meshgrid(a1, a2, indexing="ij")
    P = np.stack([A.ravel(), B.ravel(), np.full(A.size, c0[2])], 1); fo, mu, _ = ZL.field(D, P, "ours"); fu, _, _ = ZL.field(D, P, "uniform")
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
    r = R[tag]; a.set_title(f"elbow {r['elbow_angle_deg']:.0f} deg (requested {r['requested_deg']:.0f}); forearm dir {np.round(r['forearm_dir'], 2).tolist()}\n"
                            f"protrusion beyond uniform: toward robot {r['front_protrusion_vs_uniform_m']*100:.1f} cm, along fingertips {r['tip_dir_protrusion_vs_uniform_m']*100:.1f} cm", fontsize=9)
    a.set_xlabel("x [m] (robot at small x)"); a.set_ylabel("y [m]")
ax[0].plot([], [], color="#0b0b0b", label="ours boundary"); ax[0].plot([], [], "--", color="#2a78d6", label="uniform 6.5 cm"); ax[0].legend(fontsize=8, loc="lower left")
fig.suptitle("(d) Ours zone for three elbow angles (elbow-only change; horizontal section through the hand COM; point robot, m_r 4.6 kg)")
fig.savefig(f"{OUT}/v2_d_posture.png", dpi=110, bbox_inches="tight"); json.dump(R, open(f"{O}/v2_posture.json", "w"), indent=1)
for tag in R:
    print(tag, round(R[tag]["elbow_angle_deg"], 1), "toward robot", round(R[tag]["front_protrusion_vs_uniform_m"] * 100, 1), "along fingertips", round(R[tag]["tip_dir_protrusion_vs_uniform_m"] * 100, 1), R[tag]["tip_dir_thickness_m"])

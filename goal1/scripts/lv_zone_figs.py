"""lv_zone_figs.py <tag> - overnight2: zone shape figures for the recorded postures (one png per motion, goal1/live/<tag>/zone_figs/).
Posture: the held frame of the motion nearest to its held median (compare_tip/paths.json hold_frames), compare/human_rt_v2.npz, Franka base
frame of the live run (no base shift). Shapes: the arm of the motion (13 / 15 / 17 or 14 / 16 / 18) + torso and head for context.
Zone field as the earlier zone figures (v2_zone_lib / pr_zone_mesh): robot point mass 4.6 kg, mu along the direction to the nearest
point of each capsule from the segment's inverse inertia; uniform: eta = 0.065 m; per-part reference: eta = clip(0.065 + 0.10
(sqrt(mu / mu_iso) - 1)+, 0.065, 0.25), mu_iso = 1 / (1 / m_iso(segment) + 1 / 4.6). Zone = {d - eta < 0} over the arm's capsules.
Section: the plane through the target (fingertip / elbow surface point furthest along the axis) spanned by the arm axis (forearm, or
upper arm for the elbow) and the vertical direction orthogonal to it. Thickness rays: from the target along the axis ("front"); from the body
surface point of the hand (fingertip case) / elbow shapes that is highest, upward ("up").
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g1_human_link as L

GG = os.path.expanduser("~/effmass_mpc/goal1"); MROB, DB, EM, K = 4.6, 0.065, 0.25, 0.10
ISO_NM = ["pelvis", "left_thigh", "right_thigh", "abdomen", "left_calf", "right_calf", "left_calf", "right_calf", "chest", "neck", "left_shoulder",
          "right_shoulder", "head", "left_upper_arm", "right_upper_arm", "left_lower_arm", "right_lower_arm", "left_hand", "right_hand"]
M_ISO = np.array([json.load(open(f"{GG}/data/iso_15066.json"))[n]["m_h_iso"] for n in ISO_NM], float)


def field(P, D, shapes, mode):
    fb = np.full(len(P), np.inf); eb = np.zeros(len(P))
    for s in shapes:
        a, b = D["P0"][s], D["P1"][s]; ab = b - a; tt = np.clip(((P - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = a + tt[:, None] * ab; v = q - P; n = np.linalg.norm(v, axis=1)
        d = n - D["RAD"][s]; u = v / np.maximum(n, 1e-9)[:, None]; g = D["SEG"][s]
        mh = 1.0 / np.maximum(np.einsum("ni,ij,nj->n", u, D["LAM"][g], u), 1e-12); mu = 1.0 / (1.0 / mh + 1.0 / MROB)
        eta = np.full(len(P), DB) if mode == "uniform" else np.clip(DB + K * np.maximum(np.sqrt(mu * (1.0 / M_ISO[g] + 1.0 / MROB)) - 1, 0), DB, EM)
        f = d - eta; m = f < fb; fb[m] = f[m]; eb[m] = eta[m]
    return fb, eb


def inside(P, D, shapes):
    dm = np.full(len(P), np.inf)
    for s in shapes:
        a, b = D["P0"][s], D["P1"][s]; ab = b - a; tt = np.clip(((P - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); dm = np.minimum(dm, np.linalg.norm(P - (a + tt[:, None] * ab), axis=1) - D["RAD"][s])
    return dm


def ray(D, shapes, mode, p, d):
    r = np.arange(0, 0.4, 0.001); f, _ = field(p[None] + r[:, None] * d[None], D, shapes, mode); return float(r[int(np.argmax(f > 0))])


def main(tag):
    C = f"{GG}/live/{tag}/compare"; T = f"{GG}/live/{tag}/compare_tip"; out = f"{GG}/live/{tag}/zone_figs"; os.makedirs(out, exist_ok=True)
    cfg = json.load(open(f"{C}/runs.json")); P = json.load(open(f"{T}/paths.json")); z = np.load(cfg["human"]); F = len(z["p0"]); R, t = L.load_frame(cfg["frame_file"])
    P0a = z["p0"].reshape(F, -1, 3).astype(float) @ R.T + t; P1a = z["p1"].reshape(F, -1, 3).astype(float) @ R.T + t
    rad = z["shape_radius"].astype(float); seg = z["shape_segment"].astype(int); res = []
    for m in P["motions"]:
        hf = np.array(m["hold_frames"]); med0 = np.median(P0a[hf], 0); med1 = np.median(P1a[hf], 0)
        f = int(hf[np.argmin([np.abs(P0a[x] - med0).sum() + np.abs(P1a[x] - med1).sum() for x in hf])])
        D = dict(P0=P0a[f], P1=P1a[f], RAD=rad, SEG=seg, LAM={g: R @ z["inv"][f][g].astype(float) @ R.T for g in range(19)})
        su, sf, sh = (13, 15, 17) if m["side"] == "L" else (14, 16, 18)
        arm = [k for k in range(len(rad)) if rad[k] > 0 and seg[k] in (su, sf, sh)]; ctx = [k for k in range(len(rad)) if rad[k] > 0 and seg[k] in (0, 3, 8, 12, 9)]
        fa = [k for k in arm if seg[k] == sf][0]; ua = [k for k in arm if seg[k] == su][0]
        if m["target"] == "fingertip":
            ax = D["P1"][fa] - D["P0"][fa]; ax /= np.linalg.norm(ax); cand = [D["P1"][k] + rad[k] * ax for k in arm if seg[k] == sh] + [D["P0"][k] - 0 * ax for k in arm if seg[k] == sh]
        else:
            ax = D["P1"][ua] - D["P0"][ua]; ax /= np.linalg.norm(ax); cand = [D["P1"][ua] + rad[ua] * ax, D["P0"][fa] + rad[fa] * ax]
        tip = max(cand, key=lambda c: float(c @ ax))
        e1 = ax; e2 = np.array([0, 0, 1.0]) - ax[2] * ax; e2 /= np.linalg.norm(e2)
        ua_ = np.linspace(-0.55, 0.25, 321); va_ = np.linspace(-0.30, 0.30, 241); U, V = np.meshgrid(ua_, va_)
        G = tip[None] + U.reshape(-1, 1) * e1[None] + V.reshape(-1, 1) * e2[None]
        body = inside(G, D, arm + ctx).reshape(U.shape)
        fig, axs = plt.subplots(1, 2, figsize=(11, 5.0), sharey=True); row = dict(name=m["name"], target=m["target"], frame=f)
        for j, (mode, col, lab) in enumerate((("uniform", "#7f7f7f", "uniform zone (eta 6.5 cm)"), ("part", "#2f6fb0", "per-part reference (k 0.10)"))):
            fz, eta = field(G, D, arm, mode); fz = fz.reshape(U.shape); a = axs[j]
            a.contourf(U * 100, V * 100, fz, levels=[-1, 0], colors=[col], alpha=0.35); a.contour(U * 100, V * 100, fz, levels=[0], colors=[col], linewidths=1.5)
            a.contourf(U * 100, V * 100, body, levels=[-1, 0], colors=["#3a3a3a"], alpha=0.85)
            hs = [k for k in arm if seg[k] == (sh if m["target"] == "fingertip" else su)] + ([fa] if m["target"] != "fingertip" else [])
            top = max([D[e][k] + rad[k] * e2 for k in hs for e in ("P0", "P1")], key=lambda p_: float(p_ @ e2))   # top surface point of the hand / elbow shapes
            tf = ray(D, arm, mode, tip, e1); tu = ray(D, arm, mode, top, e2); h0 = float((top - tip) @ e2)
            a.annotate("", xy=(tf * 100, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="<->", color="k", lw=1)); a.text(tf * 50, -2.6, f"front {tf * 100:.1f} cm", ha="center", fontsize=9)
            x0 = float((top - tip) @ e1) * 100
            a.annotate("", xy=(x0, (h0 + tu) * 100), xytext=(x0, h0 * 100), arrowprops=dict(arrowstyle="<->", color="k", lw=1)); a.text(x0 + 1.0, (h0 + tu) * 100 + 0.8, f"up {tu * 100:.1f} cm", fontsize=9)
            a.plot([m.get("offset_m", 0.03) * 100], [0], marker="x", color="#c0392b", ms=8, mew=2)
            a.set_aspect("equal"); a.set_title(lab, fontsize=10); a.set_xlabel("along the arm axis from the " + m["target"] + " (cm)")
            a.set_xlim(-55, 25); a.set_ylim(-30, 30); row[mode] = dict(front_cm=round(tf * 100, 1), up_cm=round(tu * 100, 1))
        axs[0].set_ylabel("up (cm)")
        fig.suptitle(f"{tag} {m['name']} ({'fingertip' if m['target'] == 'fingertip' else 'elbow'}, elbow bend {m['elbow_bend_deg']:.0f} deg): "
                     f"zone section through the arm axis (dark = body, x = where the front-pass path crosses)", fontsize=10)
        fig.tight_layout(); fp = f"{out}/zone_{m['name'].replace(' ', '')}.png"; fig.savefig(fp, dpi=150); plt.close(fig); row["png"] = fp; res.append(row)
        print(json.dumps(row))
    json.dump(res, open(f"{out}/zone_figs.json", "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1])

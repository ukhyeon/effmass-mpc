"""pr_figs.py - (part) four figures in goal1/part_regen/ from the part_regen summaries (system python3, matplotlib).
  fig_zone_3d.png         zone surface of the B2a reach arm: uniform (eta 6.5 cm) and proposed (per-part reference), colour = eta
  fig_part_thickness.png  direction-wise thickness per part over 7 postures: min-max range, previous (global mu_ref) vs proposed
  fig_static_gaps.png     pass-section min gap per static scene: uniform / previous / proposed (seed mean, seed range as error bar)
  fig_summary_table.png   key numbers of the two modes
"""
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

H = os.path.expanduser("~"); PR = f"{H}/effmass_mpc/goal1/part_regen"; ZD = f"{H}/effmass_mpc/goal1/zone"
C_UNI, C_GLO, C_PART = "#7f7f7f", "#d08c2f", "#2f6fb0"
plt.rcParams.update({"font.size": 9})


def fig_zone():
    fig = plt.figure(figsize=(10, 4.6))
    for i, (f, title) in enumerate(((f"{ZD}/zone_uniform_small_mesh.npz", "uniform (eta 6.5 cm)"), (f"{PR}/zone_part_mesh.npz", "proposed (per-part reference)"))):
        z = np.load(f); V, Fc = z["verts"], z["faces"]
        eta = z["eta"] if "eta" in z.files else np.full(len(V), 0.065)
        ax = fig.add_subplot(1, 2, i + 1, projection="3d")
        tri = V[Fc]; col = plt.cm.viridis((eta[Fc].mean(1) - 0.065) / (0.25 - 0.065))
        pc = Poly3DCollection(tri, facecolors=col, edgecolors="none", alpha=0.55); ax.add_collection3d(pc)
        lo, hi = V.min(0), V.max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
        ax.view_init(elev=22, azim=-60); ax.set_title(title); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
    sm = plt.cm.ScalarMappable(cmap="viridis", norm=plt.Normalize(6.5, 25)); fig.colorbar(sm, ax=fig.axes, shrink=0.6, label="zone thickness eta (cm)")
    fig.suptitle("Zone of the reaching right arm (B2a reach posture, robot point mass 4.6 kg)")
    fig.savefig(f"{PR}/fig_zone_3d.png", dpi=150); plt.close(fig)


def fig_parts():
    Z = json.load(open(f"{PR}/zone_parts.json")); S = Z["summary"]; parts = list(S.keys())
    fig, ax = plt.subplots(figsize=(8.5, 4))
    for j, (mode, col, lab) in enumerate((("global", C_GLO, "previous (mu_ref 0.53 kg)"), ("part", C_PART, "proposed (per-part mu_iso)"))):
        for i, p in enumerate(parts):
            x = i + (j - 0.5) * 0.32; lo = S[p][mode]["min_cm_range"][0]; hi = S[p][mode]["max_cm_range"][1]
            ax.plot([x, x], [lo, hi], color=col, lw=6, solid_capstyle="butt", label=lab if i == 0 else None)
            sh = S[p][mode]["above_base_share_range"]; ax.text(x, hi + 0.6, f"{100*sh[0]:.0f}-{100*sh[1]:.0f}%", ha="center", fontsize=7, color=col)
    ax.axhline(6.5, color=C_UNI, ls="--", lw=1, label="uniform / d_base 6.5 cm"); ax.axhline(25, color="k", ls=":", lw=0.8, label="eta_max 25 cm")
    ax.set_xticks(range(len(parts))); ax.set_xticklabels([p.replace("_", " ") for p in parts]); ax.set_ylabel("direction-wise thickness (cm), min-max over 7 postures")
    ax.set_ylim(0, 29); ax.legend(loc="upper left", fontsize=7, ncol=2)
    ax.set_title("Zone thickness per part (400 directions, own segment(s)); % = share of directions above 6.5 cm")
    fig.tight_layout(); fig.savefig(f"{PR}/fig_part_thickness.png", dpi=150); plt.close(fig)


def fig_static():
    V = json.load(open(f"{PR}/summary_v2.json"))["v2"]; S = json.load(open(f"{PR}/summary_on3.json"))
    rows = [("V2 front", [V["front/uniform"]["gap"], V["front/global"]["gap"], V["front/part"]["gap"]]),
            ("V2 hand top", [V["top/uniform"]["gap"], V["top/global"]["gap"], V["top/part"]["gap"]])]
    for sc, d in S.items():
        rows.append((sc.replace("_", " "), [d["gk000"]["gap_range"], d["gain"]["gap_range"], d["part"]["gap_range"]], [d[c]["gap_mean"] for c in ("gk000", "gain", "part")]))
    fig, ax = plt.subplots(figsize=(10, 4.2))
    for i, row in enumerate(rows):
        for j, col in enumerate((C_UNI, C_GLO, C_PART)):
            vals = row[1][j]
            if len(row) == 3:
                m = row[2][j]; lo, hi = vals
            else:
                m = float(np.mean(vals)); lo, hi = min(vals), max(vals)
            x = i + (j - 1) * 0.27
            ax.bar(x, m * 100, 0.25, color=col, label=["uniform", "previous", "proposed"][j] if i == 0 else None)
            ax.plot([x, x], [lo * 100, hi * 100], color="k", lw=0.8)
    ax.set_xticks(range(len(rows))); ax.set_xticklabels([r[0] for r in rows], rotation=30, ha="right")
    ax.set_ylabel("pass-section min gap (cm), seed 0-2 mean, bar = range"); ax.legend(fontsize=8)
    ax.set_title("Static scenes: robot sphere - hand/forearm shape gap (ends 10 cm left out)")
    fig.tight_layout(); fig.savefig(f"{PR}/fig_static_gaps.png", dpi=150); plt.close(fig)


def fig_table():
    Z = json.load(open(f"{PR}/zone_parts.json"))["summary"]; V = json.load(open(f"{PR}/summary_v2.json"))["v2"]
    R1 = json.load(open(f"{PR}/summary_r1.json")); M = json.load(open(f"{PR}/summary_on4.json")); S = json.load(open(f"{PR}/summary_on3.json"))
    f = lambda xs: f"{100*min(xs):.1f}-{100*max(xs):.1f}"
    rows = [["zone hand (cm)", f"{Z['hand']['global']['min_cm_range'][0]:.1f}-{Z['hand']['global']['max_cm_range'][1]:.1f}", f"{Z['hand']['part']['min_cm_range'][0]:.1f}-{Z['hand']['part']['max_cm_range'][1]:.1f}"],
            ["zone forearm (cm)", f"{Z['forearm']['global']['min_cm_range'][0]:.1f}-{Z['forearm']['global']['max_cm_range'][1]:.1f}", f"{Z['forearm']['part']['min_cm_range'][0]:.1f}-{Z['forearm']['part']['max_cm_range'][1]:.1f}"],
            ["zone upper arm (cm)", f"{Z['upper_arm']['global']['min_cm_range'][0]:.1f}-{Z['upper_arm']['global']['max_cm_range'][1]:.1f}", f"{Z['upper_arm']['part']['min_cm_range'][0]:.1f}-{Z['upper_arm']['part']['max_cm_range'][1]:.1f}"],
            ["zone torso (cm)", f"{Z['torso']['global']['min_cm_range'][0]:.1f}-{Z['torso']['global']['max_cm_range'][1]:.1f}", f"{Z['torso']['part']['min_cm_range'][0]:.1f}-{Z['torso']['part']['max_cm_range'][1]:.1f}"],
            ["zone head (cm)", f"{Z['head']['global']['min_cm_range'][0]:.1f}-{Z['head']['global']['max_cm_range'][1]:.1f}", f"{Z['head']['part']['min_cm_range'][0]:.1f}-{Z['head']['part']['max_cm_range'][1]:.1f}"],
            ["V2 front gap (cm), uniform " + f(V["front/uniform"]["gap"]), f(V["front/global"]["gap"]), f(V["front/part"]["gap"])],
            ["V2 hand top gap (cm), uniform " + f(V["top/uniform"]["gap"]), f(V["top/global"]["gap"]), f(V["top/part"]["gap"])],
            ["forearm side P6 gap (cm), uniform " + f"{100*S['P6_forearmside18']['gk000']['gap_mean']:.1f}", f"{100*S['P6_forearmside18']['gain']['gap_mean']:.1f}", f"{100*S['P6_forearmside18']['part']['gap_mean']:.1f}"]]
    for v in ("v025", "v050", "v100"):
        rows.append([f"moving hand {v} min gap (cm), uniform " + f(R1[f"{v}/gk000"]["min_gap_pass"]), f(R1[f"{v}/gain"]["min_gap_pass"]), f(R1[f"{v}/part"]["min_gap_pass"])])
    for v in ("v025", "v050"):
        rows.append([f"crossing {v} passes, uniform " + "/".join(map(str, M[f"M2/{v}/gk000"]["passes"])), "/".join(map(str, M[f"M2/{v}/gain"]["passes"])), "/".join(map(str, M[f"M2/{v}/part"]["passes"]))])
        rows.append([f"crossing {v} min gap (cm), uniform " + f(M[f"M2/{v}/gk000"]["min_gap_pass"]), f(M[f"M2/{v}/gain"]["min_gap_pass"]), f(M[f"M2/{v}/part"]["min_gap_pass"])])
    pen = lambda d, c: sum(sum(x[f"{c}"]["pen_all"]) for x in [d])
    tot_g = sum(sum(R1[k]["pen_all"]) for k in R1 if k.endswith("/gain")) + sum(sum(M[k]["pen_all"]) for k in M if k.endswith("/gain"))
    tot_p = sum(sum(R1[k]["pen_all"]) for k in R1 if k.endswith("/part")) + sum(sum(M[k]["pen_all"]) for k in M if k.endswith("/part"))
    rows.append(["penetration steps, moving scenes (all seeds)", str(tot_g), str(tot_p)])
    fig, ax = plt.subplots(figsize=(9.5, 0.36 * len(rows) + 0.8)); ax.axis("off")
    t = ax.table(cellText=rows, colLabels=["", "previous (global mu_ref)", "proposed (per-part)"], loc="center", cellLoc="center", colWidths=[0.56, 0.22, 0.22])
    t.auto_set_font_size(False); t.set_fontsize(8); t.scale(1, 1.25)
    ax.set_title("Previous vs proposed: key numbers (seeds 0-2 unless noted)", fontsize=10)
    fig.tight_layout(); fig.savefig(f"{PR}/fig_summary_table.png", dpi=150); plt.close(fig)


if __name__ == "__main__":
    fig_zone(); fig_parts(); fig_static(); fig_table(); print("figures written")

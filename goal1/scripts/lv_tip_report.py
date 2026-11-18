"""lv_tip_report.py <tag> - live cmp tip: table and figure side approach (compare/, the recorded robot path) vs tip pass (compare_tip/),
uniform vs per-part, per motion; writes compare_tip/REPORT.md (an existing "## Interpretation" section is kept) and fig_side_vs_tip.png.
Side approach values: the closest-approach arm gap of the motion (compare/analysis.json, whole motion window, the robot met the forearm side).
Tip pass values: gap to the target (fingertip / elbow) over the held frames (compare_tip/analysis.json), side of the closest pass.
Median [min, max] of seeds 0-2.
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

GG = os.path.expanduser("~/effmass_mpc/goal1")
COL = {"uniform": "#7f7f7f", "part": "#2f6fb0"}


def f1(v, s=100.0, d=1):
    return "-" if not v else f"{v['median'] * s:.{d}f} [{v['lo'] * s:.{d}f}, {v['hi'] * s:.{d}f}]"


def main(tag):
    C = f"{GG}/live/{tag}/compare"; T = f"{GG}/live/{tag}/compare_tip"
    A = json.load(open(f"{C}/analysis.json")); B = json.load(open(f"{T}/analysis.json")); S = {m["name"]: m for m in json.load(open(f"{T}/scenes.json"))["motions"]}
    side = {e["name"]: e["per_mode"] for e in A["intervals"] if e["t0_s"] is not None}
    rows = ["| motion | approach | mode | gap (cm) | part / side | eta at min (cm) | pass s | overlap steps |", "|---|---|---|---|---|---|---|---|"]
    fig_v = {}
    for mot in side:
        for mode in ("uniform", "part"):
            p = side[mot][mode]
            rows.append(f"| {mot} | arm side (recorded path) | {mode} | {f1(p['min_gap_arm'])} | {'/'.join(p['closest'])} | {f1(p['eta_at_min'])} | {f1(p['pass_time_s'], 1, 2)} | {f1(p['overlap_steps'], 1, 0)} |")
            fig_v[(mot, "side", mode)] = p["min_gap_arm"]
        if mot in B:
            tgt = S[mot]["target"]
            for mode in ("uniform", "part"):
                p = B[mot][mode]
                rows.append(f"| {mot} | {tgt} front (new path) | {mode} | {f1(p['target_gap'])} | {tgt} / {'/'.join(p['side'])} | {f1(p['eta_at_min'])} | {f1(p['pass_time_s'], 1, 2)} | {f1(p['overlap_steps'], 1, 0)} |")
                fig_v[(mot, "tip", mode)] = p["target_gap"]
    diff = []
    for mot in side:
        d_side = 100 * (fig_v[(mot, "side", "part")]["median"] - fig_v[(mot, "side", "uniform")]["median"])
        d_tip = 100 * (fig_v[(mot, "tip", "part")]["median"] - fig_v[(mot, "tip", "uniform")]["median"]) if (mot, "tip", "part") in fig_v else None
        diff.append((mot, d_side, d_tip))
    mots = list(side); fig, ax = plt.subplots(figsize=(9, 3.8)); w = 0.2
    for q, mot in enumerate(mots):
        for j, (ap, mode) in enumerate((("side", "uniform"), ("side", "part"), ("tip", "uniform"), ("tip", "part"))):
            v = fig_v.get((mot, ap, mode))
            if not v:
                continue
            x = q + (j - 1.5) * w + (0.05 if ap == "tip" else -0.05)
            ax.bar(x, v["median"] * 100, w * 0.95, color=COL[mode], hatch="//" if ap == "tip" else None, edgecolor="white", lw=0.5,
                   label=f"{'fingertip/elbow front' if ap == 'tip' else 'arm side'} - {'uniform (k 0)' if mode == 'uniform' else 'per-part (k 0.10)'}" if q == 0 else None)
            ax.plot([x, x], [v["lo"] * 100, v["hi"] * 100], color="k", lw=0.8); ax.text(x, max(v["hi"] * 100, 0) + 0.4, f"{v['median'] * 100:.1f}", ha="center", fontsize=7)
    ax.axhline(0, color="k", lw=0.8); ax.set_xticks(range(len(mots))); ax.set_ylim(0, max(v["hi"] for v in fig_v.values()) * 100 + 2.5)
    ax.set_xticklabels([f"{m}\n({'fingertip' if S.get(m, {}).get('target') == 'fingertip' else 'elbow'})" for m in mots])
    ax.set_ylabel("closest-approach gap (cm)"); ax.legend(fontsize=7, ncol=2, loc="upper left")
    ax.set_title(f"{tag}: robot passing the arm side vs in front of the fingertip / elbow (median of seeds 0-2, whisker = range)", fontsize=9)
    fig.tight_layout(); fig.savefig(f"{T}/fig_side_vs_tip.png", dpi=150); plt.close(fig)
    L = [f"# {tag}: fingertip / elbow front pass vs arm side approach", "",
         "Same recording (live3m), replayed at real time with the live settings, seeds 0-2. Arm side = the recorded robot path (compare/, the arm lay across it and "
         "the robot waited beside the forearm). Front pass = a new path per motion: a horizontal line perpendicular to the arm axis just in front of the fingertip "
         "(motions 1, 2) or the elbow (motion 3), from the held posture of the motion (compare_tip/scenes.json; offset calibrated so that the robot without human "
         "cost passes about 3 cm away; virtual robot base moved for reach, see notes). Front-pass values use the frames that hold the posture.", "",
         "Gap = robot sphere surface (incl. 0.02 m buffer) to the shape surface. Arm side: closest arm part over the whole motion. Front pass: the target (fingertip / elbow).", ""] + rows + ["",
         "Per-part minus uniform (median, cm): " + "; ".join(f"{m}: arm side {a:+.1f}, front {('-' if b is None else f'{b:+.1f}')}" for m, a, b in diff) + ".", "",
         "![side vs tip](fig_side_vs_tip.png)", ""]
    notes = []
    for m in S.values():
        notes.append(f"- {m['name']}: {m['target']}, offset {m.get('offset_m', 0.03):.3f} m, base shift {m['base_shift']} m, half length {m.get('half_length_m', 0):.2f} m, "
                     f"held frames {m.get('hold_share', 0) * 100:.0f} %")
    L += ["Paths:"] + notes + [""]
    if os.path.exists(f"{T}/analysis_off.json"):
        Off = json.load(open(f"{T}/analysis_off.json"))
        L += ["Human cost off (seed 0): " + "; ".join(f"{k}: min {100 * v['off']['seeds'][0]['target_gap']:.1f} cm, per pass "
                                                      + ", ".join(f"{100 * x:.1f}" for x in v['off']['seeds'][0].get('pass_gaps', [])) for k, v in Off.items()) + " cm.", ""]
    old = open(f"{T}/REPORT.md").read() if os.path.exists(f"{T}/REPORT.md") else ""
    if "## Interpretation" in old:
        L += [old[old.index("## Interpretation"):].rstrip(), ""]
    open(f"{T}/REPORT.md", "w").write("\n".join(L)); print("\n".join(rows)); print(diff)


if __name__ == "__main__":
    main(sys.argv[1])

"""on_wp3_table.py - WP3 'where does ours help' table as an image (goal1/overnight/wp3_effect_table.png) from on_wp3_summary.json, plus the V2 reach-pose scenes.
1118: pass-section minimum (10 cm at both path ends left out), 'small, uncertain' class, classification rule printed under the table."""
import json, os
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
H = os.path.expanduser("~"); S = json.load(open(f"{H}/effmass_mpc/goal1/logs/meff_isaac/on_wp3_summary.json")); m = lambda x: float(np.mean([v for v in x if v is not None]))
DIRN = {"tipfront": "fingertip front", "top": "over hand", "forearmside": "beside forearm"}
rows = [["P1 reach (V2)", "fingertip front", "0.35", "6.0", "12.0", "+6.0", "-", "ours farther"], ["P1 reach (V2)", "over hand", "0.35", "5.4", "8.2", "+2.8", "-", "ours farther"]]
for sc, r in S.items():
    if "gain" not in r:
        continue
    p, d = sc.split("_"); d0 = d.rstrip("0123456789"); u, o = r["gk000"], r["gain"]; cl = r["class"] + (" (torso cost in ours)" if sum(o["torso_cost_steps"]) else "")
    rows.append([{"P2": "P2 bend45", "P3": "P3 bend90", "P4": "P4 side", "P6": "P6 hand-over"}[p], DIRN[d0], f"{r['torso_min_m']:.2f}", f"{100*u['gap_mean']:.1f}", f"{100*o['gap_mean']:.1f}",
                 f"{100*r['diff_m']:+.1f}", f"{m(o['mu']):.2f}", cl])
rows += [["P2 / P3 / P4", "beside forearm", "-", "-", "-", "-", "-", "excluded (no placement, D3-3)"]]
fig, ax = plt.subplots(figsize=(15, 0.36 * len(rows) + 1.8)); ax.axis("off")
t = ax.table(cellText=rows, colLabels=["posture", "robot passes", "torso min (m)", "uniform gap (cm)", "ours gap (cm)", "ours - uniform", "mu at min (kg)", "class"], cellLoc="center", colWidths=[0.11, 0.11, 0.07, 0.09, 0.08, 0.08, 0.08, 0.30], bbox=[0, 0.2, 1, 0.8])
t.auto_set_font_size(False); t.set_fontsize(9); t.scale(1, 1.35)
for (i, j), c in t.get_celld().items():
    if i > 0 and j == 7:
        c.set_facecolor("#e3f0e3" if rows[i - 1][7].startswith("ours farther") else ("#f2f2f2" if "excluded" in rows[i - 1][7] else ("#fdf6d3" if rows[i - 1][7].startswith("small") else "#fbeee0")))
ax.set_title("WP3: where ours (k 0.10) keeps a larger gap than uniform (shape gap at the pass minimum, 10 cm at both path ends left out;\nmean of 3 seeds; V2 rows from the V2 record)", fontsize=11)
ax.text(0.0, 0.02, "Class rule: d = ours mean - uniform mean (3 seeds), s = larger seed range (max - min) of the two conditions.  ours farther: d > max(1 cm, s);  "
         "small, uncertain: 1 cm < |d| <= s;\nno difference: |d| <= 1 cm ('light' when ours eta at the minimum < 7.5 cm);  ours closer: d < -max(1 cm, s).\n"
         "P2 over hand: d +1.4 cm, s 2.6 cm (ours 6.1-8.7, uniform 5.8-6.3).  P6 beside forearm, forearm only: ours 12.6 vs uniform 5.7 cm.", fontsize=8.5, va="bottom", transform=ax.transAxes)
fig.tight_layout(); fig.savefig(f"{H}/effmass_mpc/goal1/overnight/wp3_effect_table.png", dpi=130)

"""on_wp3_table.py - WP3 'where does ours help' table as an image (goal1/overnight/wp3_effect_table.png) from on_wp3_summary.json, plus the V2 reach-pose scenes."""
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
fig, ax = plt.subplots(figsize=(14, 0.45 * len(rows) + 1.2)); ax.axis("off")
t = ax.table(cellText=rows, colLabels=["posture", "robot passes", "torso min (m)", "uniform gap (cm)", "ours gap (cm)", "ours - uniform", "mu at min (kg)", "class"], loc="center", cellLoc="center")
t.auto_set_font_size(False); t.set_fontsize(9); t.scale(1, 1.35)
for (i, j), c in t.get_celld().items():
    if i > 0 and j == 7:
        c.set_facecolor("#e3f0e3" if rows[i - 1][7].startswith("ours farther") else ("#f2f2f2" if "excluded" in rows[i - 1][7] else "#fbeee0"))
ax.set_title("WP3: where ours (k 0.10) keeps a larger gap than uniform (shape gap, mean of 3 seeds; V2 rows from the V2 record)", fontsize=11)
fig.tight_layout(); fig.savefig(f"{H}/effmass_mpc/goal1/overnight/wp3_effect_table.png", dpi=130)

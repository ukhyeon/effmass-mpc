"""lv_remake_r1_table.py - overnight2 bundle 2: moving-hand table (R1, reach at 0.25 / 0.50 / 1.00 m/s, seeds 0-2) with uniform, previous
(single mu_ref) and per-part reference, from goal1/part_regen/summary_r1.json (pass-section rule of part_regen: TCP within 0.10 m of the
path ends left out). Writes goal1/public_draft/analysis_remake/figures/moving_hand_table.png."""
import json, os
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
H = os.path.expanduser("~"); S = json.load(open(f"{H}/effmass_mpc/goal1/part_regen/summary_r1.json")); OUT = f"{H}/effmass_mpc/goal1/public_draft/analysis_remake/figures"
f = lambda xs: f"{100 * np.mean(xs):.1f} ({100 * min(xs):.1f}-{100 * max(xs):.1f})"
rows = []
for v, lab in (("v025", "0.25 m/s"), ("v050", "0.50 m/s"), ("v100", "1.00 m/s")):
    r = [lab]
    for c in ("gk000", "gain", "part"):
        d = S[f"{v}/{c}"]; r += [f(d["min_gap_pass"]), str(sum(d["pen_all"])), "/".join(map(str, d["passes"]))]
    rows.append(r)
cols = ["hand speed"] + [f"{n}\n{c}" for n in ("uniform", "previous", "per-part") for c in ("min gap cm", "overlap steps", "passes s0/s1/s2")]
fig, ax = plt.subplots(figsize=(13, 1.9)); ax.axis("off")
t = ax.table(cellText=rows, colLabels=cols, loc="center", cellLoc="center"); t.auto_set_font_size(False); t.set_fontsize(8); t.scale(1, 1.9)
ax.set_title("Moving hand (reach toward the path), seeds 0-2: min gap mean (range), overlap steps (all seeds), passes per seed", fontsize=9)
fig.tight_layout(); os.makedirs(OUT, exist_ok=True); fig.savefig(f"{OUT}/moving_hand_table.png", dpi=150); print(rows)

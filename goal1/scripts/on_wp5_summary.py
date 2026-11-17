"""on_wp5_summary.py - WP5 perception imperfection summary (no runs). R1 0.5 m/s scene; base = R1 runs (seed 0 paired; seeds 0-2 spread).
Per (cond, variant, seed): all-pass minimum gap, critical-window (phase in [-1.24, +0.25] s) min gap and max / mean deficit, hold-window mean gap,
penetration steps, joint command acceleration / jerk RMS, realtime factor. Outputs logs/meff_isaac/on_wp5_summary.json, goal1/overnight/wp5_perception.png."""
import json, os
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; OV = f"{H}/effmass_mpc/goal1/overnight"; TR = 1.24
P = [p for p in json.load(open(f"{O}/on_wp1_r1_passes.json")) if p["speed"] == "v050"] + json.load(open(f"{O}/on_wp5_passes.json"))
R = [r for r in json.load(open(f"{O}/on_wp1_r1_runs.json")) if r["speed"] == "v050"] + json.load(open(f"{O}/on_wp5_runs.json"))
ORDER = ["base", "lag050", "lag100", "lag150", "lag300", "jit1", "jit2", "jit4", "drop30", "drop60", "lag100+L01", "lag100+P", "jit2+P", "jit4+P", "lag100+jit2+P", "jit2+P+F", "jit4+P+F"]


def st(ps, r):
    cw = [p for p in ps if p["phase"] is not None and -TR <= p["phase"] <= 0.25]; hw = [p for p in ps if p["phase"] is not None and 0.5 <= p["phase"] <= 1.25]
    return dict(min_gap=float(min(p["gap_shape"] for p in ps)), crit_min_gap=float(min(p["gap_shape"] for p in cw)), crit_max_deficit=float(max(p["deficit"] for p in cw)),
                crit_mean_deficit=float(np.mean([p["deficit"] for p in cw])), hold_mean_gap=float(np.mean([p["gap_shape"] for p in hw])) if hw else None,
                pen_shape=r["pen_shape_hand"], pen_mesh=r["pen_mesh_hand"], acc_rms=r["acc_rms"], jerk_rms=r["jerk_rms"], rtf=r["realtime_factor"], passes=r["passes_completed"],
                v99=r["v_near_p99"], stalls_not_at_goal=r["stalls_not_at_goal"], n_crit=len(cw))


out = {}
for c in ("gk000", "gain"):
    for var in ORDER:
        for s in (0, 1, 2):
            ps = [p for p in P if p["cond"] == c and p["variant"] == var and p["seed"] == s]; rs = [r for r in R if r["cond"] == c and r["variant"] == var and r["seed"] == s]
            if ps and rs:
                out[f"{c}/{var}/s{s}"] = st(ps, rs[0])
json.dump(out, open(f"{O}/on_wp5_summary.json", "w"), indent=1)
for k, x in out.items():
    print(f"{k:22s} min {100*x['min_gap']:5.1f} crit {100*x['crit_min_gap']:5.1f} maxdef {100*x['crit_max_deficit']:5.1f} meandef {100*x['crit_mean_deficit']:5.1f} hold {100*(x['hold_mean_gap'] or np.nan):5.1f} "
          f"pen {x['pen_shape']}/{x['pen_mesh']} acc {x['acc_rms']:.3f} jerk {x['jerk_rms']:.1f} rtf {x['rtf']:.3f} v99 {x['v99']:.3f} passes {x['passes']}")
fig, ax = plt.subplots(1, 3, figsize=(17, 5)); cols = {"gk000": "#2a78d6", "gain": "#d1495b"}
for j, (m, lab) in enumerate((("min_gap", "all-pass minimum gap (cm)"), ("crit_max_deficit", "critical-window max deficit eta - gap (cm)"), ("jerk_rms", "joint command jerk RMS (rad/s^3)"))):
    a = ax[j]
    for i, c in enumerate(("gk000", "gain")):
        xs = []; ys = []
        for n, var in enumerate(ORDER):
            for s in (0, 1, 2):
                k = f"{c}/{var}/s{s}"
                if k in out:
                    xs.append(n + (i - 0.5) * 0.25); ys.append(out[k][m] * (100 if m != "jerk_rms" else 1))
        a.scatter(xs, ys, color=cols[c], s=24, label="uniform" if c == "gk000" else "ours")
    a.set_xticks(range(len(ORDER))); a.set_xticklabels(ORDER, rotation=30); a.set_title(lab, fontsize=10); a.grid(alpha=0.3); a.legend(frameon=False, fontsize=8)
fig.suptitle("WP5 perception imperfection on the R1 0.5 m/s scene (base = R1, seeds as dots)"); fig.tight_layout(); fig.savefig(f"{OV}/wp5_perception.png", dpi=125)

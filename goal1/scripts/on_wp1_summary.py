"""on_wp1_summary.py - WP1 summary from on_analyze outputs (no runs). Passes grouped by crossing phase (t_cross - hold onset, s). Critical window per speed =
[-T_reach, +0.25 s] (robot crosses while the hand reaches or has just arrived); hold window = [+0.5, +1.25 s]. Per (speed, cond, variant): min and mean gap
in the critical window, max and mean deficit (eta - gap at the pass minimum), hold-window mean gap, penetration steps, realtime factor, solver.
Base = R1 runs (seed 0 for paired variant comparisons; seeds 0-2 give the seed spread). Outputs logs/meff_isaac/on_wp1_summary.json and
goal1/overnight/wp1_phase_gap.png."""
import json, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; OV = f"{H}/effmass_mpc/goal1/overnight"
TR = {"v025": 2.47, "v050": 1.24, "v100": 0.62}
P = json.load(open(f"{O}/on_wp1_r1_passes.json")) + (json.load(open(f"{O}/on_wp1_var_passes.json")) if os.path.exists(f"{O}/on_wp1_var_passes.json") else [])
R = json.load(open(f"{O}/on_wp1_r1_runs.json")) + (json.load(open(f"{O}/on_wp1_var_runs.json")) if os.path.exists(f"{O}/on_wp1_var_runs.json") else [])


def stats(ps, rs, v):
    cw = [p for p in ps if p["phase"] is not None and -TR[v] <= p["phase"] <= 0.25]; hw = [p for p in ps if p["phase"] is not None and 0.5 <= p["phase"] <= 1.25]
    g = np.array([p["gap_shape"] for p in cw]); d = np.array([p["deficit"] for p in cw]); gm = [p["gap_mesh"] for p in cw if p["gap_mesh"] is not None]
    return dict(n_crit=len(cw), crit_min_gap=float(g.min()) if len(g) else None, crit_mean_gap=float(g.mean()) if len(g) else None, crit_min_gap_mesh=float(min(gm)) if gm else None,
                crit_max_deficit=float(d.max()) if len(d) else None, crit_mean_deficit=float(d.mean()) if len(d) else None, n_hold=len(hw),
                hold_mean_gap=float(np.mean([p["gap_shape"] for p in hw])) if hw else None, all_min_gap=float(min(p["gap_shape"] for p in ps)),
                pen_shape_hand=int(sum(r["pen_shape_hand"] for r in rs)), pen_mesh_hand=int(sum(r["pen_mesh_hand"] for r in rs)),
                rtf_min=float(min(r["realtime_factor"] for r in rs)), solver_mean=float(np.mean([r["solver_ms_mean"] for r in rs])), solver_p99_max=float(max(r["solver_ms_p99"] for r in rs)),
                recapture=int(sum(r["recapture"] for r in rs)), infeasible=int(sum(r["infeasible"] for r in rs)), v_near_p99_max=float(max(r["v_near_p99"] for r in rs)),
                passes=int(sum(r["passes_completed"] for r in rs)))


out = {"base_seeds": {}, "seed0": {}}
for v in TR:
    for c in ("gk000", "gain"):
        per = []
        for s in (0, 1, 2):
            ps = [p for p in P if p["variant"] == "base" and p["speed"] == v and p["cond"] == c and p["seed"] == s]; rs = [r for r in R if r["variant"] == "base" and r["speed"] == v and r["cond"] == c and r["seed"] == s]
            if ps:
                per.append(stats(ps, rs, v))
        out["base_seeds"][f"{v}/{c}"] = dict(per_seed=per, spread_crit_max_deficit=float(np.ptp([x["crit_max_deficit"] for x in per])) if per else None,
                                             spread_crit_min_gap=float(np.ptp([x["crit_min_gap"] for x in per])) if per else None,
                                             spread_crit_mean_deficit=float(np.ptp([x["crit_mean_deficit"] for x in per])) if per else None)
variants = sorted({p["variant"] for p in P})
FIGV = ["base", "L02", "P", "V100", "P+L02"]
for v in TR:
    for c in ("gk000", "gain"):
        for var in variants:
            ps = [p for p in P if p["variant"] == var and p["speed"] == v and p["cond"] == c and p["seed"] == 0]; rs = [r for r in R if r["variant"] == var and r["speed"] == v and r["cond"] == c and r["seed"] == 0]
            if ps:
                out["seed0"][f"{v}/{c}/{var}"] = stats(ps, rs, v)
json.dump(out, open(f"{O}/on_wp1_summary.json", "w"), indent=1)
for k, x in out["seed0"].items():
    print(f"{k:24s} ncrit {x['n_crit']:3d} critmin {100*x['crit_min_gap']:5.1f} critmean {100*x['crit_mean_gap']:5.1f} maxdef {100*x['crit_max_deficit']:5.1f} meandef {100*x['crit_mean_deficit']:5.1f} "
          f"hold {100*(x['hold_mean_gap'] or np.nan):5.1f} allmin {100*x['all_min_gap']:5.1f} pen {x['pen_shape_hand']} rtf {x['rtf_min']:.3f} sol {x['solver_mean']:.2f}/{x['solver_p99_max']:.2f} v99 {x['v_near_p99_max']:.3f} passes {x['passes']}")
for k, x in out["base_seeds"].items():
    print(k, "seed spread maxdef %.1f cm, critmin %.1f cm, meandef %.1f cm" % (100 * x["spread_crit_max_deficit"], 100 * x["spread_crit_min_gap"], 100 * x["spread_crit_mean_deficit"]))
# figure: gap vs phase, rows speed (v050, v100), columns cond; one binned line per variant (seed 0)
fig, ax = plt.subplots(2, 2, figsize=(15, 9), sharey=True); cm = plt.get_cmap("tab10"); bins = np.arange(-1.5, 2.01, 0.25)
for i, v in enumerate(("v050", "v100")):
    for j, c in enumerate(("gk000", "gain")):
        a = ax[i, j]
        for n_, var in enumerate(FIGV):
            ps = [p for p in P if p["variant"] == var and p["speed"] == v and p["cond"] == c and p["seed"] == 0 and p["phase"] is not None]
            if not ps:
                continue
            ph = np.array([p["phase"] for p in ps]); g = 100 * np.array([p["gap_shape"] for p in ps]); mid = []; mn = []
            for b0 in bins[:-1]:
                m = (ph >= b0) & (ph < b0 + 0.25)
                if m.any():
                    mid.append(b0 + 0.125); mn.append(g[m].min())
            a.plot(mid, mn, "-o", color=cm(n_), ms=3, lw=1.6, label=var); a.scatter(ph, g, s=6, color=cm(n_), alpha=0.25)
        a.axvspan(-TR[v], 0.25, color="#f2c14e", alpha=0.15); a.axhline(0, color="#333", lw=0.8)
        a.set_xlim(-1.5, 2.0); a.set_ylim(-2, 25); a.set_title(f"{v} {'uniform' if c == 'gk000' else 'ours'} (seed 0): min gap per 0.25 s phase bin; shade = critical window")
        a.set_xlabel("crossing phase t_cross - hold onset (s)"); a.set_ylabel("pass minimum hand gap (cm)"); a.grid(alpha=0.3); a.legend(fontsize=8, ncol=3, frameon=False)
fig.tight_layout(); fig.savefig(f"{OV}/wp1_phase_gap.png", dpi=120)

"""on_wp4_summary.py jobs <out.json> | summary - WP4 moving collaboration scenes. 'jobs' writes the on_analyze job list for every on4_* log
(target = hand; M1 reference event = hold onset, M2 = fingertip on the TCP line (cross_out onset); executed-state torso cost share on).
'summary' (no runs): per (motion, speed, cond, variant) over seeds: all-pass minimum gap, critical-window minimum gap and max / mean deficit
(M1: robot crosses within [-T_reach, +0.25 s] of hold onset; M2: within +-T_sweep / 2 of the hand reaching the TCP line), hold-window mean gap (M1,
[+0.5, +2.5 s]), penetration steps, completed passes, stalls in front of the hand (not at a goal: count, time), torso cost steps, RTF, near-human speed p99.
Outputs logs/meff_isaac/on_wp4_summary.json, goal1/overnight/wp4_phase_gap.png."""
import glob, json, os, re, sys
import numpy as np
H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; OV = f"{H}/effmass_mpc/goal1/overnight"; G = f"{H}/effmass_mpc/goal1/b2a/g2/scene_b2a4_front_0.38h1.json"
K = {"gk000": 0.0, "gain": 0.10}
if sys.argv[1] == "jobs":
    jobs = []
    for f in sorted(glob.glob(f"{O}/on4_*.log")):
        t = os.path.basename(f)[:-4]; m = re.match(r"on4_(M\d)_(v\d+)_(gk000|gain)_s(\d)(?:_(.*))?$", t)
        if not m or "_rec" in t:
            continue
        mo, v, c, s, var = m.groups()
        jobs.append(dict(tag=t, human=f"human_on4_{mo}_{v}_v2", k=K[c], scene=G, ref_label="hold" if mo == "M1" else "cross_out", cost_split=True, group="wp4",
                         motion=mo, speed=v, cond=c, seed=int(s), variant=(var or "base").replace("_", "+")))
    json.dump(jobs, open(sys.argv[2], "w")); print(len(jobs), "jobs"); sys.exit(0)
CK = json.load(open(f"{O}/on_wp4_motion_check.json")); P = json.load(open(f"{O}/on_wp4_passes.json")); R = json.load(open(f"{O}/on_wp4_runs.json")); out = {}
for key in sorted({(p["motion"], p["speed"], p["cond"], p["variant"]) for p in P}):
    mo, v, c, var = key; ck = CK[f"{mo}_{v}"]
    lo, hi = ((-ck["reach_time_s"], 0.25) if mo == "M1" else (-ck["sweep_time_s"] / 2, ck["sweep_time_s"] / 2))
    ps = [p for p in P if (p["motion"], p["speed"], p["cond"], p["variant"]) == key]; rs = [r for r in R if (r["motion"], r["speed"], r["cond"], r["variant"]) == key]
    cw = [p for p in ps if p["phase"] is not None and lo <= p["phase"] <= hi]; hw = [p for p in ps if mo == "M1" and p["phase"] is not None and 0.5 <= p["phase"] <= 2.5]
    per_seed = {}
    for r in rs:
        pp = [p for p in ps if p["seed"] == r["seed"]]; cc = [p for p in pp if p in cw]
        per_seed[r["seed"]] = dict(min_gap=r["min_gap_shape"], crit_min=min(p["gap_shape"] for p in cc) if cc else None, crit_max_def=max(p["deficit"] for p in cc) if cc else None)
    out["/".join(key)] = dict(seeds=sorted(r["seed"] for r in rs), min_gap=float(min(r["min_gap_shape"] for r in rs)), min_gap_mesh=float(min(r["min_gap_mesh"] for r in rs if r["min_gap_mesh"] is not None)),
        n_crit=len(cw), crit_min_gap=float(min(p["gap_shape"] for p in cw)) if cw else None, crit_max_deficit=float(max(p["deficit"] for p in cw)) if cw else None,
        crit_mean_deficit=float(np.mean([p["deficit"] for p in cw])) if cw else None, hold_mean_gap=float(np.mean([p["gap_shape"] for p in hw])) if hw else None, n_hold=len(hw),
        pen_shape=int(sum(r["pen_shape_hand"] for r in rs)), pen_mesh=int(sum(r["pen_mesh_hand"] for r in rs)), passes=[r["passes_completed"] for r in rs],
        stalls_front=int(sum(r["stalls_not_at_goal"] for r in rs)), stall_time_front_s=float(sum(r["stall_time_not_at_goal_s"] for r in rs)), torso_cost_steps=[r.get("torso_cost_steps") for r in rs],
        torso_share=[r.get("torso_share_of_step_max") for r in rs], rtf_min=float(min(r["realtime_factor"] for r in rs)), v99_max=float(max(r["v_near_p99"] for r in rs)),
        infeasible=int(sum(r["infeasible"] for r in rs)), recapture=int(sum(r["recapture"] for r in rs)), per_seed=per_seed)
json.dump(out, open(f"{O}/on_wp4_summary.json", "w"), indent=1)
for k, x in out.items():
    print(f"{k:22s} seeds {x['seeds']} min {100*x['min_gap']:5.1f} mesh {100*x['min_gap_mesh']:5.1f} crit n{x['n_crit']} min {100*(x['crit_min_gap'] or np.nan):5.1f} maxdef {100*(x['crit_max_deficit'] or np.nan):5.1f} "
          f"meandef {100*(x['crit_mean_deficit'] or np.nan):5.1f} hold {100*(x['hold_mean_gap'] or np.nan):5.1f} pen {x['pen_shape']}/{x['pen_mesh']} passes {x['passes']} stallsF {x['stalls_front']} {x['stall_time_front_s']:.1f}s "
          f"torso {x['torso_cost_steps']} rtf {x['rtf_min']:.3f} v99 {x['v99_max']:.3f}")
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig, ax = plt.subplots(2, 2, figsize=(15, 9), sharey=True); bins = np.arange(-3.0, 4.01, 0.25)
for i, mo in enumerate(("M1", "M2")):
    for j, v in enumerate(("v025", "v050")):
        a = ax[i, j]
        for c, col in (("gk000", "#2a78d6"), ("gain", "#d1495b")):
            ps = [p for p in P if p["motion"] == mo and p["speed"] == v and p["cond"] == c and p["variant"] == "base" and p["phase"] is not None]
            if not ps:
                continue
            ph = np.array([p["phase"] for p in ps]); g = 100 * np.array([p["gap_shape"] for p in ps]); a.scatter(ph, g, s=8, color=col, alpha=0.35)
            mid = [b + 0.125 for b in bins[:-1] if ((ph >= b) & (ph < b + 0.25)).any()]; mn = [g[(ph >= b) & (ph < b + 0.25)].min() for b in bins[:-1] if ((ph >= b) & (ph < b + 0.25)).any()]
            a.plot(mid, mn, "-o", ms=3, color=col, label="uniform" if c == "gk000" else "ours")
        a.axhline(0, color="#333", lw=0.8); a.set_ylim(-3, 30); a.grid(alpha=0.3); a.legend(frameon=False)
        a.set_title(f"{mo} {'hand-over' if mo == 'M1' else 'crossing'} {v}: pass minimum hand gap vs crossing phase (seeds 0-2)")
        a.set_xlabel("t_cross - " + ("hold onset" if mo == "M1" else "hand on TCP line") + " (s)"); a.set_ylabel("gap (cm)")
fig.tight_layout(); fig.savefig(f"{OV}/wp4_phase_gap.png", dpi=120)

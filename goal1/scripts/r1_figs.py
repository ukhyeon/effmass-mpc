"""r1_figs.py - R1 figures and summary (system python via g1_run_human.sh py; needs r1_runs.json / r1_passes.json from r1_analyze.py).
r1_a_speed_gap.png: per pass, approach speed vs minimum hand gap (shape | mesh), colour = condition, marker = speed step, linear fit per condition (slope in s).
r1_b_timeline.png: per speed step one representative 'reaching' pass of ours seed 0 (approach speed closest to the median): gap and eta, fingertip and TCP speed.
r1_c_table.png: mean minimum gap per speed x condition x hand state (n passes). Writes r1_summary.json (criteria A-C, slopes, phase counts, round-trip ratios)."""
import json, os, sys
import numpy as np
from scipy.stats import linregress
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(__file__))
import r1_analyze as RA
import b2a_analyze as BA
O = RA.O; OUT = os.path.expanduser("~/effmass_mpc/goal1/r1"); os.makedirs(OUT, exist_ok=True)
runs = json.load(open(f"{O}/r1_runs.json")); P = json.load(open(f"{O}/r1_passes.json")); MC = json.load(open(f"{O}/r1_motion_check.json"))
COL = {"gk000": "#2a78d6", "gain": "#eb6834"}; LAB = {"gk000": "uniform (k 0)", "gain": "ours (k 0.10)"}; MK = {"v025": "o", "v050": "s", "v100": "^"}; ST = ["rest", "reaching", "hold", "retracting"]
S = {"criteria": {}, "by_speed_cond_state": {}, "slopes": {}, "phase_counts": {}, "round_trip": {}, "runs": {}}
for v in RA.SPEEDS:
    for c in RA.K:
        pp = [p for p in P if p["speed"] == v and p["cond"] == c]
        S["phase_counts"][f"{v}/{c}"] = {st: sum(p["state"] == st for p in pp) for st in ST}
        S["by_speed_cond_state"][f"{v}/{c}"] = {st: dict(n=sum(p["state"] == st for p in pp), gap_shape=float(np.mean([p["gap_shape"] for p in pp if p["state"] == st])) if any(p["state"] == st for p in pp) else None,
                                                          gap_mesh=float(np.nanmean([p["gap_mesh"] if p["gap_mesh"] is not None else np.nan for p in pp if p["state"] == st])) if any(p["state"] == st for p in pp) else None) for st in ST}
        S["by_speed_cond_state"][f"{v}/{c}"]["all"] = dict(n=len(pp), gap_shape=float(np.mean([p["gap_shape"] for p in pp])), gap_min=float(np.min([p["gap_shape"] for p in pp])))
        rr = [r for r in runs if r["speed"] == v and r["cond"] == c]; S["round_trip"][f"{v}/{c}"] = dict(rt_s=[r["round_trip_s"] for r in rr], period_s=MC["speeds"][v]["period_s"],
                                                                                                         ratio=[MC["speeds"][v]["period_s"] / r["round_trip_s"] for r in rr])
hold = {c: [p["gap_shape"] for p in P if p["cond"] == c and p["state"] == "hold"] for c in RA.K}; ref = {"gk000": 0.060, "gain": 0.120}
S["criteria"]["A"] = {c: dict(mean_hold_gap=float(np.mean(hold[c])) if hold[c] else None, n=len(hold[c]), v2_static=ref[c], ok=bool(hold[c]) and abs(np.mean(hold[c]) - ref[c]) <= 0.015) for c in RA.K}
S["criteria"]["B"] = dict(ok=all(r["realtime_factor"] >= 1.0 and r["recapture"] == 0 and r["infeasible"] == 0 for r in runs), min_rtf=min(r["realtime_factor"] for r in runs),
                          recapture=sum(r["recapture"] for r in runs), infeasible=sum(r["infeasible"] for r in runs))
c025 = [r for r in runs if r["speed"] == "v025"]; reach = {(c, s): [p["gap_shape"] for p in P if p["speed"] == "v025" and p["cond"] == c and p["seed"] == s and p["state"] == "reaching"] for c in RA.K for s in (0, 1, 2)}
S["criteria"]["C"] = dict(pen_shape_hand={r["tag"]: r["pen_shape_hand"] for r in c025}, no_penetration=all(r["pen_shape_hand"] == 0 for r in c025),
                          reaching_mean={f"{c}/s{s}": (float(np.mean(reach[(c, s)])) if reach[(c, s)] else None, len(reach[(c, s)])) for c in RA.K for s in (0, 1, 2)},
                          ours_gt_uniform_each_seed=all(reach[("gain", s)] and reach[("gk000", s)] and np.mean(reach[("gain", s)]) > np.mean(reach[("gk000", s)]) for s in (0, 1, 2)))
S["criteria"]["C"]["ok"] = S["criteria"]["C"]["no_penetration"] and S["criteria"]["C"]["ours_gt_uniform_each_seed"]
S["runs"] = {r["tag"]: {k: r[k] for k in ("realtime_factor", "solver_ms_mean", "solver_ms_p99", "recapture", "infeasible", "passes_completed", "round_trip_s", "pen_shape_hand", "pen_shape_all", "pen_mesh_hand", "v_near_p99", "stalls", "stall_time_s")} for r in runs}
plt.rcParams.update({"font.size": 9}); fig, ax = plt.subplots(1, 2, figsize=(15, 6))
for a, key, nm in ((ax[0], "gap_shape", "shape-based"), (ax[1], "gap_mesh", "mesh-based")):
    for c in RA.K:
        pp = [p for p in P if p["cond"] == c and p[key] is not None]; x = np.array([p["v_approach"] for p in pp]); y = np.array([p[key] for p in pp]) * 100
        for v in RA.SPEEDS:
            m = np.array([p["speed"] == v for p in pp]); a.plot(x[m], y[m], MK[v], color=COL[c], ms=5, alpha=0.6, mec="none", label=f"{LAB[c]}, {v}")
        lr = linregress(x, y / 100); xx = np.linspace(x.min(), x.max(), 10); a.plot(xx, (lr.intercept + lr.slope * xx) * 100, "-", color=COL[c], lw=2)
        S["slopes"][f"{c}/{key}"] = dict(slope_s=float(lr.slope), intercept_m=float(lr.intercept), r=float(lr.rvalue), n=len(x))
        a.text(0.02, 0.97 - 0.06 * list(RA.K).index(c), f"{LAB[c]}: slope {lr.slope:+.3f} s (gap per approach speed), r {lr.rvalue:.2f}, n {len(x)}", transform=a.transAxes, color=COL[c], fontsize=9, va="top")
    a.set_xlabel("fingertip approach speed toward the closest robot sphere at the pass minimum [m/s] (>0 approaching)"); a.set_ylabel(f"minimum hand gap, {nm} [cm]"); a.grid(alpha=0.3)
    a.axhline(0, color="#52514e", lw=0.8, ls=":"); a.set_title(f"per pass, {nm}")
ax[0].legend(fontsize=7, loc="lower left", ncol=2); fig.suptitle("R1: approach speed vs minimum hand gap (all speeds, seeds 0-2)"); fig.savefig(f"{OUT}/r1_a_speed_gap.png", dpi=115, bbox_inches="tight"); plt.close(fig)
fig, ax = plt.subplots(3, 2, figsize=(15, 11))
for i, v in enumerate(RA.SPEEDS):
    pp = [p for p in P if p["speed"] == v and p["cond"] == "gain" and p["seed"] == 0 and p["state"] == "reaching"]
    if not pp:
        pp = [p for p in P if p["speed"] == v and p["cond"] == "gain" and p["seed"] == 0]
    med = np.median([p["v_approach"] for p in pp]); p0 = min(pp, key=lambda p: abs(p["v_approach"] - med)); Z = np.load(f"{O}/{p0['tag']}_r1_series.npz"); D = RA.human(v)
    a_, b_ = max(0, p0["start"] - 30), min(len(Z["gap_shape"]) - 1, p0["end"] + 30); t = (np.arange(a_, b_ + 1) + 1) / 60; fr = Z["frame"][a_:b_ + 1]
    eta = []
    for ii in range(a_, b_ + 1):
        f = Z["frame"][ii]; q = None
    ax[i, 0].plot(t, Z["gap_shape"][a_:b_ + 1] * 100, color="#eb6834", lw=2, label="hand gap, shape (ours)"); ax[i, 0].plot(t, Z["gap_mesh"][a_:b_ + 1] * 100, ".", color="#a3341b", ms=2, label="hand gap, mesh")
    zu = np.load(f"{O}/{p0['tag'].replace('_gain_', '_gk000_')}_r1_series.npz"); ax[i, 0].plot(t, zu["gap_shape"][a_:b_ + 1] * 100, color="#2a78d6", lw=1.5, label="hand gap, shape (uniform, same seed)")
    ax[i, 0].axhline(p0["eta"] * 100, color="#eb6834", ls="--", lw=1, label=f"eta at the minimum (ours) {p0['eta']*100:.1f} cm"); ax[i, 0].axhline(6.5, color="#2a78d6", ls="--", lw=1, label="eta uniform 6.5 cm")
    ax[i, 0].axvline(p0["t_min"], color="#0b0b0b", lw=0.8); ax[i, 0].set_ylabel("cm"); ax[i, 0].set_ylim(-2, 40); ax[i, 0].legend(fontsize=7); ax[i, 0].set_title(f"{v}: pass {p0['pass_idx']} ({p0['state']}, approach {p0['v_approach']:.2f} m/s)")
    tipspeed = np.linalg.norm(D["tipv"][fr], axis=1); tcp = Z["tcp"][a_:b_ + 1]; vt = np.r_[0, np.linalg.norm(np.diff(tcp, axis=0), axis=1) * 60]
    ax[i, 1].plot(t, tipspeed, color="#8a8984", lw=2, label="fingertip speed"); ax[i, 1].plot(t, vt, color="#eb6834", lw=1.5, label="robot TCP speed (ours)"); ax[i, 1].axvline(p0["t_min"], color="#0b0b0b", lw=0.8)
    ax[i, 1].set_ylabel("m/s"); ax[i, 1].legend(fontsize=7); ax[i, 1].grid(alpha=0.3); ax[i, 0].grid(alpha=0.3)
ax[2, 0].set_xlabel("time [s]"); ax[2, 1].set_xlabel("time [s]"); fig.suptitle("R1: representative 'reaching' pass per speed (ours seed 0; uniform seed 0 at the same human time)")
fig.savefig(f"{OUT}/r1_b_timeline.png", dpi=110, bbox_inches="tight"); plt.close(fig)
fig, a = plt.subplots(figsize=(15, 4.2)); a.axis("off"); rowsT = []
for v in RA.SPEEDS:
    for c in RA.K:
        d = S["by_speed_cond_state"][f"{v}/{c}"]; rowsT.append([v, LAB[c]] + [f"{d[st]['gap_shape']*100:.1f} / {d[st]['gap_mesh']*100:.1f} (n {d[st]['n']})" if d[st]["n"] else "-" for st in ST] + [f"{d['all']['gap_shape']*100:.1f} (min {d['all']['gap_min']*100:.1f})"])
tb = a.table(cellText=rowsT, colLabels=["speed", "condition"] + [f"{st}: shape / mesh cm" for st in ST] + ["all passes"], loc="center"); tb.auto_set_font_size(False); tb.set_fontsize(8); tb.scale(1, 1.5)
a.set_title("R1: mean minimum hand gap per pass by speed, condition and hand state (seeds 0-2)"); fig.savefig(f"{OUT}/r1_c_table.png", dpi=120, bbox_inches="tight"); plt.close(fig)
json.dump(S, open(f"{O}/r1_summary.json", "w"), indent=1, default=float); print(json.dumps(S["criteria"], indent=1, default=float)); print(json.dumps(S["slopes"], indent=1)); print(json.dumps(S["phase_counts"]))
print(json.dumps(S["round_trip"], default=float))

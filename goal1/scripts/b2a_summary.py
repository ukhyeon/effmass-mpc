"""b2a_summary.py - statistics and figures for B2a from logs/meff_isaac/b2a_runs.json (system python3)."""
import json, os, sys
import numpy as np
from scipy.stats import binomtest
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; OUT = f"{H}/effmass_mpc/goal1/b2a"
R = json.load(open(f"{O}/b2a_runs.json")); IDX = {(r["path"], r["cond"], r["seed"]): r for r in R}
# extra metrics (defined after the runs; see report): pass window = TCP within +-0.12 m of the hand centre along y; crossing time = first step with TCP y >= 0.17
HAND_Y = json.load(open(f"{H}/effmass_mpc/goal1/b2a/g2/lines.json"))["hand_com"][1]
HANDK = [18, 19, 20]
for r in R:
    z = np.load(f"{O}/{r['tag']}_b1.npz")["rows"]; tcp = z[:, 8:11] - np.array([0, 0, 0.1034]); w = np.abs(tcp[:, 1] - HAND_Y) <= 0.12
    ser = np.load(f"{O}/{r['tag']}_b2a_series.npz")
    r["win_hand_shape_gap"] = float(z[w][:, 12:36][:, HANDK].min()) if w.any() else None
    r["win_hand_mesh_gap"] = float(ser["gmin"][w].min()) if w.any() else None
    cr = np.nonzero(tcp[:, 1] >= 0.17)[0]; r["cross_time_s"] = float((cr[0] + 1) / 60.0) if len(cr) else None
    r["arrived"] = r["pass_time_s"] is not None
    r["start_mesh_gap"] = float(ser["gmin"][0])
    r["win_risk_max_s02"] = float(ser["risk_s02"][w].max()) if w.any() else None
PATHS = ["front", "top"]; CONDS = ["iso", "massA", "uni10", "unim"]; SEEDS = range(8)
MET = ["win_hand_shape_gap", "win_hand_mesh_gap", "cross_time_s", "ee_path_m", "risk_max_s02", "risk_sum_s02", "risk_max_s04", "risk_sum_s04", "hand_shape_gap", "hand_mesh_gap", "pass_time_s",
       "forearm_shape_gap", "torso_shape_gap", "dq_mean", "dq_p95", "infeasible", "fb_exec", "fb_rollout", "loop_ms_mean"]
get = lambda p, c, s, m: IDX[(p, c, s)].get(m) if (p, c, s) in IDX else None


def sign_stats(d):
    d = np.asarray([x for x in d if x is not None], float); n = int((d != 0).sum()); k = int((d > 0).sum())
    return dict(n=int(len(d)), mean=float(d.mean()) if len(d) else None, std=float(d.std(ddof=1)) if len(d) > 1 else None, min=float(d.min()) if len(d) else None,
                max=float(d.max()) if len(d) else None, n_pos=k, n_neg=int((d < 0).sum()), sign_p=float(binomtest(k, n, 0.5).pvalue) if n else None)


S = {"per_cond": {}, "paired": {}, "interaction": {}, "budget": {}, "rollout": {}, "off": {}, "arrivals": {}}
for p in PATHS:
    for c in CONDS + ["off"]:
        S["arrivals"][f"{p}/{c}"] = sum(1 for s in SEEDS if (p, c, s) in IDX and IDX[(p, c, s)]["arrived"])
for p in PATHS:
    for c in CONDS + ["off"]:
        S["per_cond"][f"{p}/{c}"] = {m: (lambda v: dict(mean=float(np.mean(v)), std=float(np.std(v, ddof=1)) if len(v) > 1 else None, min=float(np.min(v)), max=float(np.max(v)), n=len(v)) if v else None)(
            [get(p, c, s, m) for s in SEEDS if get(p, c, s, m) is not None]) for m in MET}
    for b in ("iso", "uni10", "unim"):
        for m in MET[:11]:
            S["paired"][f"{p}: massA-{b}: {m}"] = sign_stats([get(p, "massA", s, m) - get(p, b, s, m) for s in SEEDS if get(p, "massA", s, m) is not None and get(p, b, s, m) is not None])
for m in ("win_hand_shape_gap", "win_hand_mesh_gap", "hand_shape_gap", "hand_mesh_gap", "risk_max_s02", "risk_sum_s02", "cross_time_s"):
    S["interaction"][m] = sign_stats([(get("front", "massA", s, m) - get("front", "iso", s, m)) - (get("top", "massA", s, m) - get("top", "iso", s, m)) for s in SEEDS])
for m in ("win_hand_shape_gap", "win_hand_mesh_gap", "cross_time_s", "risk_sum_s02", "risk_max_s02"):
    S["budget"][m] = sign_stats([(get("front", "massA", s, m) + get("top", "massA", s, m)) - (get("front", "unim", s, m) + get("top", "unim", s, m)) for s in SEEDS])
for p in PATHS:
    for c in CONDS:
        shares = {}
        for s in SEEDS:
            for k, v in (get(p, c, s, "rollout_share_by_segment") or {}).items():
                shares[k] = shares.get(k, 0.0) + v / 8
        S["rollout"][f"{p}/{c}"] = shares
for p in PATHS:
    for s in (0, 1):
        r = IDX.get((p, "off", s)); S["off"][f"{p}/s{s}"] = {m: r.get(m) for m in ("hand_shape_gap", "hand_mesh_gap", "win_hand_shape_gap", "win_hand_mesh_gap", "hand_mesh_steps_negative", "pass_time_s", "cross_time_s", "risk_max_s02", "risk_sum_s02")} if r else None
json.dump(S, open(f"{O}/b2a_summary.json", "w"), indent=1)
# ---- figures
cols = plt.cm.tab10(np.arange(8))
fig, ax = plt.subplots(2, 2, figsize=(12, 8), sharey="row")
for ci, (m, lab) in enumerate((("win_hand_shape_gap", "hand min gap in the pass window, shape-based [cm]"), ("win_hand_mesh_gap", "hand min gap in the pass window, mesh-based [cm]"))):
    for pi, p in enumerate(PATHS):
        a = ax[ci, pi]
        for s in SEEDS:
            y = [get(p, c, s, m) * 100 for c in CONDS]; a.plot(range(4), y, "-o", ms=4, lw=0.8, color=cols[s], label=f"seed {s}")
        for s in (0, 1):
            if get(p, "off", s, m) is not None:
                a.plot([-0.6], [get(p, "off", s, m) * 100], "kx")
        a.set_xticks([-0.6, 0, 1, 2, 3]); a.set_xticklabels(["off", "iso", "mass A", "uni 0.10", "uni matched"], fontsize=8); a.set_title(f"{p}: {lab}", fontsize=9); a.grid(alpha=0.3)
        a.axhline(0, color="k", lw=0.5)
ax[0, 1].legend(fontsize=7, ncol=2); fig.suptitle("B2a: right-hand minimum gap per path and condition (lines = same MPPI seed; x = off seeds 0/1)"); fig.tight_layout()
fig.savefig(f"{OUT}/b2a_fig1_hand_gap.png", dpi=120); plt.close(fig)
fig, ax = plt.subplots(2, 2, figsize=(12, 8), sharey="row")
for ci, (m, lab) in enumerate((("risk_max_s02", "risk max, sigma 0.02 m [m/s, relative]"), ("risk_sum_s02", "risk sum over steps, sigma 0.02 m [m/s, relative]"))):
    for pi, p in enumerate(PATHS):
        a = ax[ci, pi]
        for s in SEEDS:
            a.plot(range(4), [get(p, c, s, m) for c in CONDS], "-o", ms=4, lw=0.8, color=cols[s])
        for s in (0, 1):
            if get(p, "off", s, m) is not None:
                a.plot([-0.6], [get(p, "off", s, m)], "kx")
        a.set_xticks([-0.6, 0, 1, 2, 3]); a.set_xticklabels(["off", "iso", "mass A", "uni 0.10", "uni matched"], fontsize=8); a.set_title(f"{p}: {lab}", fontsize=9); a.grid(alpha=0.3)
        a.set_yscale("symlog", linthresh=1e-5); a.set_ylim(-1e-6, 10)
fig.suptitle("B2a: contact risk = P(mesh gap <= 0) x v_n x sqrt(mu / 2.0) (lines = same seed)"); fig.tight_layout(); fig.savefig(f"{OUT}/b2a_fig3_risk.png", dpi=120); plt.close(fig)
json.dump(R, open(f"{O}/b2a_runs_ext.json", "w"), indent=1)
print(json.dumps({k: S[k] for k in ("arrivals", "interaction", "budget", "rollout", "off")}, indent=1))

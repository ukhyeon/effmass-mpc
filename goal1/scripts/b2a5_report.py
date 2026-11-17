"""b2a5_report.py - B2a-5 presentation numbers (system python): percent changes on front-cal (mass A vs iso, mass A vs uniform_matched; (mean_A - mean_B)
/ mean_B x 100, condition means over seeds 0-7) for gap shape / mesh, closest mu, risk max / sum (sigma 0.02, 0.04); side-by-side paired differences
B2a-4 (no speed cap) vs B2a-5 (profile t1); runs without any rollout human cost. Output logs/meff_isaac/b2a5_report.json."""
import json, os
import numpy as np
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac")
R5 = json.load(open(f"{O}/b2a5_runs.json")); S5 = json.load(open(f"{O}/b2a5_summary.json")); S4 = json.load(open(f"{O}/b2a4_summary.json"))
mean = lambda R, sc, c, k: float(np.mean([r[k] for r in R if r["path"] == sc and r["cond"] == c and r.get(k) is not None]))
K = ["hand_shape_gap", "hand_mesh_gap", "hand_mu", "risk_max_s02", "risk_max_s04", "risk_sum_s02", "risk_sum_s04", "pass_time_s"]
out = {"pct_front": {}, "b2a4_vs_b2a5": {}, "no_cost_runs": []}
for b in ("iso", "unim"):
    out["pct_front"][f"massA_vs_{b}"] = {k: dict(massA=mean(R5, "front", "massA", k), other=mean(R5, "front", b, k), pct=100 * (mean(R5, "front", "massA", k) / mean(R5, "front", b, k) - 1) if mean(R5, "front", b, k) else None) for k in K}
for sc in ("front", "top"):
    for pr in ("massA-iso", "massA-unim"):
        for k in ("hand_shape_gap", "hand_mesh_gap", "hand_mu", "risk_max_s04", "risk_sum_s04", "pass_time_s"):
            a, b = S4["paired"][f"{sc}/{pr}"].get(k), S5["paired"][f"{sc}/{pr}"].get(k)
            out["b2a4_vs_b2a5"][f"{sc}/{pr}/{k}"] = dict(b2a4=(a["mean"], a["same_sign"], a["n_nonzero"], a["p_sign"]) if a else None, b2a5=(b["mean"], b["same_sign"], b["n_nonzero"], b["p_sign"]) if b else None)
    i4, i5 = S4["interaction"], S5["interaction"]
    out["b2a4_vs_b2a5"]["interaction"] = {k: dict(b2a4=(i4[k]["mean"], i4[k]["same_sign"], i4[k]["n_nonzero"], i4[k]["p_sign"]) if i4.get(k) else None,
                                                  b2a5=(i5[k]["mean"], i5[k]["same_sign"], i5[k]["n_nonzero"], i5[k]["p_sign"]) if i5.get(k) else None) for k in ("hand_shape_gap", "hand_mesh_gap", "hand_mu", "risk_max_s04")}
out["no_cost_runs"] = [r["tag"] for r in R5 if r["cond"] != "off" and not r.get("rollout_cost_n")]
json.dump(out, open(f"{O}/b2a5_report.json", "w"), indent=1, default=float)
for k, v in out["pct_front"].items():
    print(k, {kk: round(vv["pct"], 1) if vv["pct"] is not None else None for kk, vv in v.items()})
for k, v in out["b2a4_vs_b2a5"].items():
    print(k, v)
print("no cost runs", out["no_cost_runs"])

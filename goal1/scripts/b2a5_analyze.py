"""b2a5_analyze.py - B2a-5 analysis (profile t1) from saved run records only (system python via g1_run_human.sh py).
Usage: b2a5_analyze.py runs   -> logs/meff_isaac/b2a5_runs.json (per run)
       b2a5_analyze.py summary -> logs/meff_isaac/b2a5_summary.json (paired, sides, interaction, same budget)
Per run: b2a_analyze.run (shape and mesh hand gaps, u / m_h / m_r / mu / eta at the shape-based minimum, contact-risk index with the existing
definition, pass time = arrival at B, EE path, infeasible, rollout cost shares) + b2a4_cal.measure (side at the minimum, TCP heights) and:
shape overlap steps, crossing time (TCP leaves the pass window, y >= hand COM y + 0.12), dwell time in the window, TCP speed in the window (max, mean),
gripper rotation about the vertical vs the goal at the minimum, max distance of the TCP from the line (x-z plane) and in x, sqrt(mu / 2.0)."""
import json, os, sys, csv
import numpy as np
from scipy.stats import binomtest
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import b2a4_cal as CAL
import shape_metrics as SM
O = BA.O; G = BA.G; HY = json.load(open(f"{G}/lines.json"))["hand_com"][1]
SC = {"front": "front_0.38h1", "top": "top_0.455"}; CONDS = ["off", "iso", "massA", "unim"]
RI = BA.RI; HB, HT = RI.body_of["fr3v2_1_hand"]


def line_of(key):
    js = json.load(open(f"{G}/scene_b2a4_{key}.json")); return np.array(js["tcp_A"]), np.array(js["tcp_B"])


def run(scene, cond, seed):
    key = SC[scene]; tag = f"b2a5_{key}_{cond}_s{seed}"; A_, B_ = line_of(key)
    out = BA.run(tag, scene, cond, seed); c = CAL.measure(tag, float(A_[0]))
    out.update(side=c["side"], side_comp=c["comp"], tcp_at_min=c["tcp_at_min"], tcp_max_above_hand_top=c["tcp_max_above_hand_top"], max_dev_x=c["max_dev_x"],
               sqrt_mu=float(np.sqrt(out["hand_mu"] / 2.0)))
    B = np.load(f"{O}/{tag}_b1.npz")["rows"]; tcp = B[:, 8:11] - np.array([0, 0, 0.1034]); w = np.abs(tcp[:, 1] - HY) <= 0.12
    D = B[:, 12:36][:, BA.HAND].min(1); out["hand_shape_steps_negative"] = int((D < 0).sum())
    leave = np.nonzero(tcp[:, 1] >= HY + 0.12)[0]; out["cross_time_s"] = float((leave[0] + 1) / 60.0) if len(leave) else None
    out["dwell_s"] = float(w.sum() / 60.0)
    v = np.linalg.norm(np.gradient(tcp, 1 / 60.0, axis=0), axis=1); out["tcp_speed_window_max"] = float(v[w].max()) if w.any() else None
    out["tcp_speed_window_mean"] = float(v[w].mean()) if w.any() else None
    x = (RI._fk(np.array(c["q_at_min"]))[HB + 1] @ HT)[:3, 0]; out["yaw_err_at_min_deg"] = float(np.degrees(np.arctan2(x[1], x[0])))
    out["max_dev_line_xz"] = float(np.sqrt((tcp[:, 0] - A_[0]) ** 2 + (tcp[:, 2] - A_[2]) ** 2).max())
    out["line_key"] = key
    # (T1 metrics) near-human sphere speed and gripper rotation error at the closest moment (same definitions as t1_metrics.py)
    r = BA.result(tag); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])
    Cs = np.stack([RI.compute(q)[0] for q in Q]); ALL = [k for k in range(24) if BA.RAD[k] > 0]
    gap = np.stack([np.stack([SM.segdist(C, BA.P0[k], BA.P1[k]) - BA.RAD[k] for k in ALL], 1).min(1) - BA.RS for C in Cs])
    vv = (np.linalg.norm(np.diff(Cs, axis=0), axis=-1) * 60.0)[gap[1:] <= 0.4]
    out.update(v_near_p99=float(np.percentile(vv, 99)) if len(vv) else None, v_near_max=float(vv.max()) if len(vv) else None, yaw_err_at_min_abs_deg=abs(out["yaw_err_at_min_deg"]))
    z = np.load(f"{O}/{tag}_costagg.npz")
    if "speed_n" in z.files:
        out.update(speed_cost_steps=int((z["speed_n"] > 0).sum()))
    return out


PAIRS = [("massA", "iso"), ("massA", "unim")]
MET = ["v_near_p99", "v_near_max", "yaw_err_at_min_abs_deg", "dwell_s", "hand_shape_gap", "hand_mesh_gap", "hand_mu", "sqrt_mu", "hand_shape_u_vs_front_deg", "risk_max_s02", "risk_max_s04", "risk_sum_s02", "risk_sum_s04",
       "pass_time_s", "cross_time_s", "ee_path_m", "max_dev_line_xz", "tcp_speed_window_max", "yaw_err_at_min_deg"]


def stats(d):
    d = np.array([x for x in d if x is not None and np.isfinite(x)], float)
    if not len(d):
        return None
    m = float(d.mean()); sgn = np.sign(m) if m != 0 else 1.0; nz = d[d != 0]; k = int((np.sign(nz) == sgn).sum())
    return dict(n=int(len(d)), mean=m, sd=float(d.std(ddof=1)) if len(d) > 1 else 0.0, min=float(d.min()), max=float(d.max()), same_sign=k, n_nonzero=int(len(nz)),
                p_sign=float(binomtest(k, len(nz), 0.5).pvalue) if len(nz) else None, values=d.tolist())


def summary():
    R = json.load(open(f"{O}/b2a5_runs.json")); get = lambda sc, c, s: next((r for r in R if r["path"] == sc and r["cond"] == c and r["seed"] == s), None)
    S = {"by_condition": {}, "sides": {}, "paired": {}, "interaction": {}, "same_budget": {}, "task": {}}
    for sc in SC:
        for c in CONDS:
            rs = [r for r in R if r["path"] == sc and r["cond"] == c]
            if not rs:
                continue
            cnt = {}
            for r in rs:
                cnt[r["side"]] = cnt.get(r["side"], 0) + 1
            S["sides"][f"{sc}/{c}"] = cnt
            S["by_condition"][f"{sc}/{c}"] = {m: stats([r.get(m) for r in rs]) for m in MET + ["hand_m_h", "hand_m_r", "hand_eta", "hand_shape_steps_negative", "hand_mesh_steps_negative",
                                                                                                 "infeasible", "dwell_s", "tcp_speed_window_mean", "tcp_max_above_hand_top", "max_dev_x"]}
            S["task"][f"{sc}/{c}"] = dict(arrived=sum(r["pass_time_s"] is not None for r in rs), n=len(rs), infeasible_total=sum(r["infeasible"] for r in rs),
                                          non_hand_rollout_share_max=max(sum(v for k, v in r.get("rollout_share_by_segment", {}).items() if k != "18") for r in rs),
                                          rollout_segments=sorted({k for r in rs for k in r.get("rollout_share_by_segment", {})}))
        for a, b in PAIRS:
            S["paired"][f"{sc}/{a}-{b}"] = {m: stats([(get(sc, a, s) or {}).get(m) - (get(sc, b, s) or {}).get(m) if get(sc, a, s) and get(sc, b, s)
                                                      and get(sc, a, s).get(m) is not None and get(sc, b, s).get(m) is not None else None for s in range(8)]) for m in MET}
    for m in MET:
        d = []
        for s in range(8):
            f = [get("front", c, s) for c in ("massA", "iso")]; t = [get("top", c, s) for c in ("massA", "iso")]
            ok = all(x is not None and x.get(m) is not None for x in f + t)
            d.append((f[0][m] - f[1][m]) - (t[0][m] - t[1][m]) if ok else None)
        S["interaction"][m] = stats(d)
    for sc, ms in (("front", ["risk_max_s02", "risk_max_s04", "risk_sum_s02", "risk_sum_s04", "hand_mu", "hand_shape_gap", "hand_mesh_gap"]),
                   ("top", ["pass_time_s", "cross_time_s", "ee_path_m", "max_dev_line_xz"])):
        S["same_budget"][sc] = {m: S["paired"][f"{sc}/massA-unim"][m] for m in ms}
    json.dump(S, open(f"{O}/b2a5_summary.json", "w"), indent=1)
    return S


if __name__ == "__main__":
    if sys.argv[1] == "runs":
        from multiprocessing import Pool
        jobs = [(sc, c, s) for sc in SC for c in CONDS for s in range(8) if os.path.exists(f"{O}/b2a5_{SC[sc]}_{c}_s{s}_b1.npz")]
        with Pool(10) as pool:
            allr = pool.starmap(run, jobs)
        for r in allr:
            print(r["path"], r["cond"], r["seed"], r["side"], round(r["hand_shape_gap"] * 100, 2), round(r["hand_mesh_gap"] * 100, 2), round(r["hand_mu"], 2), r["pass_time_s"])
        json.dump(allr, open(f"{O}/b2a5_runs.json", "w"), indent=1, default=float)
    else:
        S = summary(); print(json.dumps(S["sides"])); print(json.dumps(S["task"]))

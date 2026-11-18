"""lv_timing.py <tag> [<tag> ...] - live prep timing summary of lv_live_run.sh runs (after lv_analyze.py --ros).

Per run: realtime factor, solver mean / p95 / p99, loop mean, mean of each per-step time column of the steps csv (t_*_ms,
physics_ms), human data age at the MPC (lv_analyze: wall time of the step - input stamp, median / p95), node compute (perf csv),
process CPU from step4_sys_monitor (mean %), sensing frame rate (sensing perf csv in <tag>_sensing/sensing_perf, if any).
Writes <tag>_lv_timing.json and prints one table.
"""
import csv
import glob
import json
import os
import re
import sys

import numpy as np

O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac")


def one(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read()
    r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))
    rows = list(csv.DictReader(open(r["steps_csv"])))[10:]
    col = lambda k: np.array([float(x[k]) for x in rows if x.get(k, "") not in ("", None)])
    out = dict(tag=tag, steps=len(rows) + 10, realtime_factor=r["realtime_factor"], solver_mean=float(col("solver_ms").mean()),
               solver_p95=float(np.percentile(col("solver_ms"), 95)), solver_p99=float(np.percentile(col("solver_ms"), 99)), loop_mean=float(col("loop_ms").mean()))
    for k in rows[0].keys():
        if (k.startswith("t_") and k.endswith("_ms")) or k == "physics_ms":
            v = col(k)
            if len(v):
                out[k] = float(v.mean())
    a = f"{O}/{tag}_lv_analysis.json"
    if os.path.exists(a):
        d = json.load(open(a))
        out.update(age_ms_median=d.get("age_ms_median"), age_ms_p95=d.get("age_ms_p95"), min_gap_shape=d.get("min_gap_shape"), v_near_p99=d.get("v_near_p99"),
                   passes=d.get("passes_completed"), pen=d.get("pen_shape_all"))
    np_ = f"{O}/{tag}_nodeperf.csv"
    if os.path.exists(np_):
        nr = list(csv.DictReader(open(np_)))
        if nr:
            out["node_total_ms_mean"] = float(np.mean([float(x["total_ms"]) for x in nr])); out["node_msgs"] = len(nr)
    sm = f"{O}/{tag}_sys_monitor.csv"
    if os.path.exists(sm):
        sr = list(csv.DictReader(open(sm)))
        for k in sr[0].keys() if sr else []:
            if k.endswith("cpu_pct"):
                v = [float(x[k]) for x in sr if x[k] not in ("", "nan")]
                if v:
                    out[f"cpu_{k}"] = float(np.mean(v))
    sp = glob.glob(f"{O}/{tag}_sensing/sensing_perf/*.csv")
    for f in sp:
        sr = list(csv.DictReader(open(f)))
        if len(sr) > 2:
            ts = [float(x["t_ros_sec"]) for x in sr if x.get("t_ros_sec")]
            out[f"sensing_{os.path.basename(f)}_rows"] = len(sr)
            if len(ts) > 2:
                out[f"sensing_{os.path.basename(f)}_hz"] = float((len(ts) - 1) / (ts[-1] - ts[0]))
    json.dump(out, open(f"{O}/{tag}_lv_timing.json", "w"), indent=1)
    return out


if __name__ == "__main__":
    R = [one(t) for t in sys.argv[1:]]
    keys = []
    for r in R:
        for k in r:
            if k not in keys:
                keys.append(k)
    for k in keys:
        print(f"{k:42s}" + "".join(f"{(('%.3f' % r[k]) if isinstance(r.get(k), float) else str(r.get(k, '-')))[:22]:>24s}" for r in R))

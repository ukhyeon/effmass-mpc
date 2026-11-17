"""g1_meff_isaac_analyze.py <tag> [...] — g1_meff_run.sh 실행 분석 (시스템 python3 -B)"""
import csv, datetime, json, os, re, sys
import numpy as np
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac")
rows = []
for tag in sys.argv[1:]:
    P = f"{O}/{tag}"; log = open(P + ".log", errors="replace").read()
    rp = re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]; r = json.load(open(rp))
    meta = json.load(open(P + "_isaac_meta.json")); ls, le = meta["loop_start_ns"], meta["loop_end_ns"]
    out = dict(tag=tag, solver_mean=r["solver_ms_mean"], solver_p95=r["solver_ms_p95"], loop_mean=r["loop_ms_mean"], loop_p95=r["loop_ms_p95"], rtf=r["realtime_factor"],
               infeasible=r["infeasible_steps"], arrivals=r["arrivals"], captures=r["graph_captures_by_stage"])
    m = r.get("meff") or {}
    out.update(meff=m.get("meff"), ch_ptr=m.get("channel_pointers_unchanged"), in_ptr=m.get("inputs_pointers_unchanged"), calls=m.get("calls"),
               rinv_mean=m.get("robot_inv_ms_mean"), rinv_p95=m.get("robot_inv_ms_p95"), paired=m.get("paired"), import_unchanged=(m.get("import_check") or {}).get("unchanged"),
               pin_loaded=(m.get("import_check") or {}).get("pinocchio_loaded"))
    H = np.load(P + "_human_inertia_rec.npz"); rc = H["recv"]; t = rc[:, 2]; sel = (t >= ls) & (t <= le)
    iv = np.diff(t[sel]) / 1e6
    out.update(human_hz=float((sel.sum() - 1) / ((t[sel][-1] - t[sel][0]) / 1e9)), human_max_gap_ms=float(iv.max()), human_n=int(sel.sum()),
               node_compute_ms=float(np.mean(rc[sel, 4])), human_lat_ms_p95=float(np.percentile((rc[sel, 2] - rc[sel, 1]) / 1e6, 95)))
    st = [l.split() for l in open(P + "_dmon.txt") if l.strip() and not l.startswith("#")]
    d0 = datetime.datetime.fromtimestamp(ls / 1e9).strftime("%H:%M:%S"); d1 = datetime.datetime.fromtimestamp(le / 1e9).strftime("%H:%M:%S")
    clk = [int(x[6]) for x in st if d0 <= x[0] <= d1 and x[6].isdigit()]
    out.update(sm_clock_median=float(np.median(clk)) if clk else None, sm_clock_min=min(clk) if clk else None, sm_clock_max=max(clk) if clk else None)
    sc = list(csv.DictReader(open(r["steps_csv"])))
    if "t_meff_r_ms" in sc[0]:
        v = np.array([float(x["t_meff_r_ms"]) for x in sc]); out.update(rinv_loop_mean=float(v.mean()), rinv_loop_p95=float(np.percentile(v, 95)))
    sm = [x for x in csv.DictReader(open(P + "_sys_monitor.csv")) if x.get("node_cpu_pct")]
    if sm:
        out.update(node_cpu=float(np.mean([float(x["node_cpu_pct"]) for x in sm[3:-2]])), isaac_cpu=float(np.mean([float(x["isaac_cpu_pct"]) for x in sm[3:-2] if x.get("isaac_cpu_pct")])))
    rows.append(out)
    json.dump(out, open(P + "_analysis.json", "w"), indent=1)
    print(json.dumps(out, ensure_ascii=False))

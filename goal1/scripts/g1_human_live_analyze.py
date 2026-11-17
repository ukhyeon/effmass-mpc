"""g1_human_live_analyze.py <run_dir> — 실제 실행 분석 (시스템 python3 -B)"""
import csv, json, os, sys
import numpy as np
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/env_check/scripts")); import step4_ref_fk as F
O = sys.argv[1]; R = np.load(f"{O}/rec.npz")
z = np.load(os.path.expanduser("~/hrc_ws/pose_npz/human_state_1781524848.npz"))
P, T = z["poses"].astype(np.float32), z["trans"].astype(np.float32)
c = F.rest_constants(np.zeros(10)); ref, _, _ = F.com_world(P, T, c)
fr = R["frame"]; com = R["com"].astype(np.float64)
err = np.linalg.norm(com - ref[fr], axis=2).max(1)
mix_p = F.com_world(P[:-1], T[1:], c)[0]; mix_m = F.com_world(P[1:], T[:-1], c)[0]
nmix = sum(1 for i in np.nonzero(err > 1e-5)[0] if (fr[i] < len(mix_p) and np.linalg.norm(com[i] - mix_p[fr[i]], axis=1).max() < 1e-5)
           or (fr[i] > 0 and np.linalg.norm(com[i] - mix_m[fr[i] - 1], axis=1).max() < 1e-5))
tr = R["t_recv"]; iv = np.diff(tr) / 1e6; dur = (tr[-1] - tr[0]) / 1e9
lat = (tr - R["in_stamp"]) / 1e6
g = np.diff(fr); vt = np.linalg.norm((ref[fr[1:]] - ref[fr[:-1]]) * 30.0 / np.maximum(g, 1)[:, None, None], axis=2); vs = np.linalg.norm(R["vel"][1:], axis=2)
out = dict(n=int(len(fr)), frames_unique=int(len(np.unique(fr))), err_max_mm=float(err.max() * 1000), within_1mm=int((err <= 1e-3).sum()), exact_001mm=int((err <= 1e-5).sum()),
           mixed=int(nmix), rate_hz=float((len(tr) - 1) / dur), max_interval_ms=float(iv.max()), interval_p95_ms=float(np.percentile(iv, 95)),
           latency_ms=dict(mean=float(lat.mean()), p95=float(np.percentile(lat, 95)), max=float(lat.max())),
           compute_ms=dict(mean=float(R["compute_ms"].mean()), p95=float(np.percentile(R["compute_ms"], 95)), max=float(R["compute_ms"].max()), over_33ms=int((R["compute_ms"] > 1000 / 30).sum())),
           gaps={int(k): int((g == k).sum()) for k in np.unique(g)}, shapes=int(R["n_shapes"][0]))
for k in np.unique(g):
    sel = (g == k)[:, None] & (vt > 0.02)
    out[f"vel_ratio_gap{int(k)}"] = dict(median=float(np.median(vs[sel] / vt[sel])), p5=float(np.percentile(vs[sel] / vt[sel], 5)), p95=float(np.percentile(vs[sel] / vt[sel], 95)))
pf = f"{O}/node_perf.csv"
if os.path.exists(pf):
    rows = list(csv.DictReader(open(pf))); out["skipped_inputs"] = int(sum(int(r["skipped_inputs"]) for r in rows))
    out["perf_ms"] = {k: float(np.mean([float(r[k]) for r in rows[1:]])) for k in ["fk_ms", "mass_inv_ms", "inv_vel_shape_ms", "compute_ms", "msg_ms", "total_ms"]}
sm = [r for r in csv.DictReader(open(f"{O}/sys_monitor.csv")) if r.get("node_cpu_pct")]
cpu = [float(r["node_cpu_pct"]) for r in sm][3:-2]
out["cpu_pct"] = dict(mean=float(np.mean(cpu)), max=float(np.max(cpu))); out["threads_max"] = max(int(float(r["node_threads"])) for r in sm if r["node_threads"]); out["rss_mib_max"] = max(float(r["node_rss_mib"]) for r in sm)
out["gpu_vram_node"] = sorted({r["node_vram_mib"] for r in sm})
json.dump(out, open(f"{O}/analysis.json", "w"), indent=1)
print(json.dumps(out, ensure_ascii=False))

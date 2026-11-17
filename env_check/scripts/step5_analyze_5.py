"""step5_analyze_5.py <run_dir> <steps.csv> — 5-5 분석 (측정 루프 구간)"""
import csv, glob, json, os, sys
import numpy as np
O, SF = sys.argv[1], sys.argv[2]
st = lambda v: dict(n=int(len(v)), mean=float(np.mean(v)), p95=float(np.percentile(v, 95)), max=float(np.max(v))) if len(v) else dict(n=0)
fmt = lambda d: (f"n={d['n']} 평균 {d['mean']:.3f} / p95 {d['p95']:.3f} / 최대 {d['max']:.3f}" if d.get("n") else "n=0")
im = json.load(open(f"{O}/run_isaac_meta.json")); L0, L1 = im["loop_start_ns"], im["loop_end_ns"]
tag = os.path.basename(os.path.normpath(O))
res = json.load(open(sorted(glob.glob(os.path.join(os.path.dirname(SF), f"{tag}_result_*.json")))[-1]))
rows = list(csv.DictReader(open(SF)))
K = ["t_obst_ms", "t_human_ms", "t_js_ms", "t_presync_ms", "t_solver_ms", "t_cmd_ms", "t_goal_ms", "busy_ms", "sched_lag_ms", "late_ms"]
out = dict(res=res, items={k: st([float(r[k]) for r in rows]) for k in K}, first_dyn_rel_s=(im["human_first_dyn_ns"] - L0) / 1e9)
print(f"[55] 측정 루프 {(L1 - L0) / 1e9:.2f} s, 달성 {res['achieved_hz']:.2f} Hz, solver 평균 {res['solver_ms_mean']:.3f} / p95 {res['solver_ms_p95']:.3f} / 최대 {res['solver_ms_max']:.3f} ms, "
      f"마감 초과 {res['deadline_miss']}/{res['steps']} ({res['deadline_miss_pct']:.2f}%), 늦음 최대 {res['late_ms_max']:.2f} ms, 그래프 {res['graph_captures_by_stage']}, infeasible {res['infeasible']}")
print("[55] 항목별 평균/p95 (ms): " + "  ".join(f"{k[2:-3] if k.startswith('t_') else k[:-3]} {out['items'][k]['mean']:.3f}/{out['items'][k]['p95']:.3f}" for k in K))
R = np.load(f"{O}/run_human_rec.npz")
P = R["pose"]; w = (P[:, 2] >= L0) & (P[:, 2] <= L1); pr, ps = P[w, 2], P[w, 1]
if len(pr) < 2:
    print(f"[55] !!! 측정 루프 HumanPose {len(pr)}개 → 무효"); json.dump(dict(out, invalid=True), open(f"{O}/analysis.json", "w"), default=float); sys.exit(0)
iv = np.diff(pr) / 1e6
out["pose"] = dict(n=int(w.sum()), rate_hz=float((len(pr) - 1) / ((pr[-1] - pr[0]) / 1e9)), max_interval_ms=float(iv.max()), latency_ms=st((pr - ps) / 1e6),
                   with_vertices=int((P[w, 3] == 20670).sum()))
pf = glob.glob(f"{O}/sensing/sensing_perf/*.csv")
if pf:
    sr = [r for r in csv.DictReader(open(pf[0])) if L0 / 1e9 <= float(r["t_ros_sec"]) <= L1 / 1e9]
    got = int(((ps >= L0) & (ps <= L1)).sum())
    out["sensing"] = {x: st([float(r[x]) for r in sr]) for x in ["capture_ms", "pose_inference_ms", "visualization_ms", "publish_ms", "total_operation_ms"]}
    out["sensing"].update(rows=len(sr), miss=sum(int(r["deadline_miss"]) for r in sr))
    out["pose"].update(sent=len(sr), recv=got, loss_pct=100 * (1 - got / len(sr)) if sr else None)
print(f"[55] HumanPose (MPC 수신): {out['pose']['n']}개, {out['pose']['rate_hz']:.2f} Hz, 최대 간격 {out['pose']['max_interval_ms']:.2f} ms, 지연 {fmt(out['pose']['latency_ms'])} ms, "
      f"정점 있는 메시지 {out['pose']['with_vertices']}, 발행 {out['pose'].get('sent')} ↔ 수신 {out['pose'].get('recv')} (누락 {out['pose'].get('loss_pct')})")
if "sensing" in out:
    print("[55] sensing 메시지당 (평균/p95 ms): " + ", ".join(f"{x[:-3]} {out['sensing'][x]['mean']:.2f}/{out['sensing'][x]['p95']:.2f}"
          for x in ["capture_ms", "pose_inference_ms", "visualization_ms", "publish_ms", "total_operation_ms"]) + f", 33.3 ms 초과 {out['sensing']['miss']}/{out['sensing']['rows']}")
ds, dr = R["dyn_stamp"], R["dyn_recv"]; w2 = (dr >= L0) & (dr <= L1); iv2 = np.diff(dr[w2]) / 1e6
out["dyn"] = dict(n=int(w2.sum()), rate_hz=float((w2.sum() - 1) / ((dr[w2][-1] - dr[w2][0]) / 1e9)), max_interval_ms=float(iv2.max()), latency_ms=st((dr[w2] - ds[w2]) / 1e6))
print(f"[55] HumanDynamicsState: {out['dyn']['n']}개, {out['dyn']['rate_hz']:.2f} Hz, 최대 간격 {out['dyn']['max_interval_ms']:.2f} ms, 지연 {fmt(out['dyn']['latency_ms'])} ms")
npf = glob.glob(f"{O}/nodeperf_paper_human_dynamics_perf.csv")
if npf:
    rr = [r for r in csv.DictReader(open(npf[0])) if L0 / 1e9 <= float(r["t_ros_sec"]) <= L1 / 1e9]
    out["node"] = dict(total_ms=st([float(r["total_operation_ms"]) for r in rr]), miss=sum(int(r["deadline_miss"]) for r in rr), rows=len(rr))
    print(f"[55] 노드 계산: {fmt(out['node']['total_ms'])} ms, 33.3 ms 초과 {out['node']['miss']}/{out['node']['rows']}")
rr = [r for r in csv.DictReader(open(f"{O}/sys_monitor.csv")) if L0 / 1e9 + 1 <= float(r["t"]) <= L1 / 1e9]
fv = lambda c: [float(r[c]) for r in rr if r.get(c) not in (None, "")]
core = np.array([[float(r[c]) for c in rr[0] if c.startswith("core")] for r in rr])
out["sys"] = dict(gpu_util=st(fv("gpu_util_pct")), gpu_mem_max=max(fv("gpu_mem_mib")), core_mean=float(core.mean()))
for l in ("mpc", "node", "sensing"):
    out["sys"][l] = dict(cpu=st(fv(f"{l}_cpu_pct")), vram_max=max(fv(f"{l}_vram_mib") or [0]), threads_max=max(fv(f"{l}_threads") or [0]))
print(f"[55] GPU 사용률 {fmt(out['sys']['gpu_util'])}%, GPU 메모리 최대 {out['sys']['gpu_mem_max']:.0f} MiB, 논리 CPU 평균 {out['sys']['core_mean']:.1f}%")
for l in ("mpc", "node", "sensing"):
    d = out["sys"][l]; print(f"[55]   {l}: CPU% {fmt(d['cpu'])}, VRAM 최대 {d['vram_max']:.0f} MiB, 스레드 최대 {d['threads_max']:.0f}")
jf = f"{O}/sys_recv_js.csv"
if os.path.exists(jf):
    print(f"[55] JointState 수신 {sum(1 for _ in open(jf)) - 1} / 발행 {res['steps']}")
json.dump(out, open(f"{O}/analysis.json", "w"), indent=1, default=float)

"""step5_analyze_2.py <run_dir> <steps.csv> — 5-2 분석 (측정 루프 구간 기준)"""
import csv, glob, json, os, sys
import numpy as np
O, SF = sys.argv[1], sys.argv[2]
st = lambda v: dict(n=int(len(v)), mean=float(np.mean(v)), p95=float(np.percentile(v, 95)), max=float(np.max(v))) if len(v) else dict(n=0)
fmt = lambda d: (f"n={d['n']} 평균 {d['mean']:.3f} / p95 {d['p95']:.3f} / 최대 {d['max']:.3f}" if d.get("n") else "n=0")
im = json.load(open(f"{O}/run_isaac_meta.json")); L0, L1 = im["loop_start_ns"], im["loop_end_ns"]
res = json.load(open(sorted(glob.glob(os.path.dirname(SF) + "/" + os.path.basename(os.path.dirname(O + "/")) + "_result_*.json"))[-1]))
out = dict(loop_s=(L1 - L0) / 1e9, rtf=res["realtime_factor"], solver_p95=res["solver_ms_p95"], loop_mean=res["loop_ms_mean"],
           graph=res.get("graph_captures_by_stage"), first_dyn_rel_s=(im["human_first_dyn_ns"] - L0) / 1e9 if im.get("human_first_dyn_ns") else None)
print(f"[52] 측정 루프 {out['loop_s']:.2f} s, 첫 HumanDynamicsState {out['first_dyn_rel_s']:+.2f} s (루프 시작 기준); 배율 {out['rtf']}, solver p95 {out['solver_p95']}, 루프 평균 {out['loop_mean']}, 그래프 {out['graph']}")
R = np.load(f"{O}/run_human_rec.npz")
P = R["pose"]; w = (P[:, 2] >= L0) & (P[:, 2] <= L1); pr, ps = P[w, 2], P[w, 1]
if len(pr) < 2:
    print(f"[52] !!! 측정 루프 동안 Isaac 이 받은 HumanPose {len(pr)}개 → 측정 무효 (사람 데이터 없음)")
    json.dump(dict(out, invalid="no HumanPose in loop"), open(f"{O}/analysis.json", "w"), indent=1, default=float); sys.exit(0)
iv = np.diff(pr) / 1e6
out["pose"] = dict(n=int(w.sum()), rate_hz=float((w.sum() - 1) / ((pr[-1] - pr[0]) / 1e9)), max_interval_ms=float(iv.max()), interval_ms=st(iv),
                   latency_ms=st((pr - ps) / 1e6), with_vertices=int((P[w, 3] == 20670).sum()))
pf = glob.glob(f"{O}/sensing/sensing_perf/*.csv")
if pf:
    rows = [r for r in csv.DictReader(open(pf[0])) if L0 / 1e9 <= float(r["t_ros_sec"]) <= L1 / 1e9]
    # 발행 수: 루프 구간에 기록된 sensing 줄 (기록 시각 = 발행 직후) ↔ Isaac 이 같은 구간에 받은 수 (stamp 기준)
    sent = len(rows); got = int(((ps >= L0) & (ps <= L1)).sum())
    k = ["capture_ms", "pose_inference_ms", "visualization_ms", "publish_ms", "total_operation_ms"]
    out["sensing"] = {x: st([float(r[x]) for r in rows]) for x in k}
    out["sensing"].update(rows=sent, deadline_miss=sum(int(r["deadline_miss"]) for r in rows), rate_hz=(sent - 1) / ((float(rows[-1]["t_ros_sec"]) - float(rows[0]["t_ros_sec"]))) if sent > 1 else None)
    out["pose"].update(sent_in_loop=sent, recv_by_stamp_in_loop=got, loss_pct=100 * (1 - got / sent) if sent else None)
print(f"[52] HumanPose (Isaac 수신, 측정 루프): {out['pose']['n']}개, 주기 {out['pose']['rate_hz']:.2f} Hz, 최대 간격 {out['pose']['max_interval_ms']:.2f} ms, "
      f"지연(stamp→수신) {fmt(out['pose']['latency_ms'])} ms, 정점 있는 메시지 {out['pose']['with_vertices']}; 발행 {out['pose'].get('sent_in_loop')} ↔ 수신 {out['pose'].get('recv_by_stamp_in_loop')} "
      f"(누락률 {out['pose'].get('loss_pct')})")
if "sensing" in out:
    print(f"[52] sensing (측정 루프): 발행 주기 {out['sensing']['rate_hz']:.2f} Hz, 33.3 ms 초과 {out['sensing']['deadline_miss']}/{out['sensing']['rows']}; " +
          ", ".join(f"{x[:-3]} {out['sensing'][x]['mean']:.2f}/{out['sensing'][x]['p95']:.2f}" for x in ["capture_ms", "pose_inference_ms", "visualization_ms", "publish_ms", "total_operation_ms"]) + " ms (평균/p95)")
ds, dr = R["dyn_stamp"], R["dyn_recv"]; w2 = (dr >= L0) & (dr <= L1); iv2 = np.diff(dr[w2]) / 1e6
out["dyn"] = dict(n=int(w2.sum()), rate_hz=float((w2.sum() - 1) / ((dr[w2][-1] - dr[w2][0]) / 1e9)), max_interval_ms=float(iv2.max()), interval_ms=st(iv2),
                  latency_ms=st((dr[w2] - ds[w2]) / 1e6))
print(f"[52] HumanDynamicsState (측정 루프): {out['dyn']['n']}개, 주기 {out['dyn']['rate_hz']:.2f} Hz, 최대 간격 {out['dyn']['max_interval_ms']:.2f} ms, 지연 {fmt(out['dyn']['latency_ms'])} ms")
npf = glob.glob(f"{O}/nodeperf_paper_human_dynamics_perf.csv")
if npf:
    rr = [r for r in csv.DictReader(open(npf[0])) if L0 / 1e9 <= float(r["t_ros_sec"]) <= L1 / 1e9]
    tot = [float(r["total_operation_ms"]) for r in rr]; dm = sum(int(r["deadline_miss"]) for r in rr)
    out["node"] = dict(total_ms=st(tot), miss=dm, rows=len(rr), miss_pct=100 * dm / len(rr))
    print(f"[52] 노드 계산 (측정 루프): {fmt(out['node']['total_ms'])} ms, 33.3 ms 초과 {dm}/{len(rr)} ({out['node']['miss_pct']:.1f}%)")
rows = [r for r in csv.DictReader(open(SF)) if int(r["step"]) >= 1 and r["feasible"] == "1"]
TF = ["t_obst_ms", "t_phys_ms", "t_state_ms", "t_human_ms", "t_h_mesh_ms", "t_js_ms", "t_presync_ms", "t_solver_ms", "t_cmd_ms", "t_goal_ms", "loop_ms"]
out["items"] = {k: float(np.mean([float(r[k]) for r in rows])) for k in TF}
mn = [r for r in rows if float(r["h_mesh_new"]) >= 1]
out["mesh"] = dict(updates=len(mn), per_update_ms=st([float(r["t_h_mesh_ms"]) for r in mn]), final=im.get("human_mesh_final"))
print("[52] 항목별 평균 (ms): " + "  ".join(f"{k[2:-3] if k.startswith('t_') else k} {v:.3f}" for k, v in out["items"].items()))
print(f"[52] 메시 갱신 {len(mn)}회, 갱신당 {fmt(out['mesh']['per_update_ms'])} ms, 최종 {out['mesh']['final']}")
sm = f"{O}/sys_monitor.csv"
rr = [r for r in csv.DictReader(open(sm)) if L0 / 1e9 + 1 <= float(r["t"]) <= L1 / 1e9]
f = lambda c: [float(r[c]) for r in rr if r.get(c) not in (None, "")]
cc = [c for c in rr[0] if c.startswith("core")]; core = np.array([[float(r[c]) for c in cc] for r in rr])
out["sys"] = dict(gpu_util=st(f("gpu_util_pct")), gpu_mem_max=max(f("gpu_mem_mib")), core_total_mean=float(core.mean()), core_mean=[round(float(x), 1) for x in core.mean(0)],
                  busiest_core_mean=float(core.max(1).mean()))
for l in ("isaac", "node", "sensing"):
    out["sys"][l] = dict(cpu=st(f(f"{l}_cpu_pct")), vram_max=max(f(f"{l}_vram_mib") or [0]), threads_max=max(f(f"{l}_threads") or [0]))
print(f"[52] GPU 사용률 {fmt(out['sys']['gpu_util'])}%, GPU 메모리 최대 {out['sys']['gpu_mem_max']:.0f} MiB; 논리 CPU 평균 {out['sys']['core_total_mean']:.1f}% {out['sys']['core_mean']}")
for l in ("isaac", "node", "sensing"):
    d = out["sys"][l]; print(f"[52]   {l}: CPU% {fmt(d['cpu'])}, VRAM 최대 {d['vram_max']:.0f} MiB, 스레드 최대 {d['threads_max']:.0f}")
jf = f"{O}/sys_recv_js.csv"
if os.path.exists(jf):
    print(f"[52] JointState 수신 {sum(1 for _ in open(jf)) - 1}")
json.dump(out, open(f"{O}/analysis.json", "w"), indent=1, default=float)

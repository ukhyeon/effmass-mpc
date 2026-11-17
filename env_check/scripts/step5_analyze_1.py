"""step5_analyze_1.py <run_dir> — 5-1 분석: HumanPose 주기·간격·지연, sensing 단계별 시간, 프레임 번호 건너뜀, CPU·GPU"""
import csv, glob, json, os, sys
import numpy as np
O = sys.argv[1]
st = lambda v: dict(n=int(len(v)), mean=float(np.mean(v)), p95=float(np.percentile(v, 95)), max=float(np.max(v))) if len(v) else dict(n=0)
fmt = lambda d: (f"n={d['n']} 평균 {d['mean']:.3f} / p95 {d['p95']:.3f} / 최대 {d['max']:.3f}" if d.get("n") else "n=0")
out = {}
R = np.load(f"{O}/rec_pose_rec.npz")
tr, sp = R["t_recv"], R["stamp"]
if len(tr) > 1:
    iv = np.diff(tr) / 1e6; dur = (tr[-1] - tr[0]) / 1e9
    out["pose"] = dict(n=int(len(tr)), dur_s=dur, rate_hz=(len(tr) - 1) / dur, interval_ms=st(iv), gaps_over_50ms=int((iv > 50).sum()),
                       gaps_over_100ms=int((iv > 100).sum()), latency_stamp_to_recv_ms=st((tr - sp) / 1e6),
                       with_vertices=int((R["nv"] == 20670).sum()), betas_first=R["betas"][0].tolist(),
                       betas_const=bool(np.allclose(R["betas"], R["betas"][0])))
    print(f"[5-1] HumanPose 수신 {len(tr)}개 / {dur:.1f} s, 주기 평균 {out['pose']['rate_hz']:.2f} Hz, 간격 {fmt(out['pose']['interval_ms'])} ms, "
          f"50 ms 넘는 간격 {out['pose']['gaps_over_50ms']}, 100 ms 넘는 간격 {out['pose']['gaps_over_100ms']}")
    print(f"[5-1] 지연 (발행 stamp → 수신) {fmt(out['pose']['latency_stamp_to_recv_ms'])} ms, 정점 있는 메시지 {out['pose']['with_vertices']}/{len(tr)}, "
          f"betas 고정 {out['pose']['betas_const']} 첫 값 {np.round(R['betas'][0], 3).tolist()}")
pf = glob.glob(f"{O}/sensing/sensing_perf/*.csv")
if pf:
    rows = list(csv.DictReader(open(pf[0])))
    k = ["capture_ms", "pose_inference_ms", "visualization_ms", "publish_ms", "total_operation_ms"]
    out["perf"] = {x: st([float(r[x]) for r in rows]) for x in k}
    fi = np.array([int(r["frame_index"]) for r in rows]); dm = sum(int(r["deadline_miss"]) for r in rows)
    out["perf"].update(rows=len(rows), deadline_miss=dm, frame_index_span=int(fi[-1] - fi[0] + 1), frame_index_gaps=int((np.diff(fi) > 1).sum()),
                       frames_missing_in_span=int(fi[-1] - fi[0] + 1 - len(fi)))
    print(f"[5-1] sensing 기록 {len(rows)}줄 (발행한 프레임), 타이머 번호 범위 {out['perf']['frame_index_span']} → 발행 안 한 번호 {out['perf']['frames_missing_in_span']} "
          f"(건너뜀 구간 {out['perf']['frame_index_gaps']}), 33.3 ms 초과 {dm}")
    for x in k:
        print(f"[5-1]   {x}: {fmt(out['perf'][x])}")
sm = f"{O}/sys_monitor.csv"
if os.path.exists(sm) and len(tr) > 1:
    rr = [r for r in csv.DictReader(open(sm)) if tr[0] / 1e9 + 1 <= float(r["t"]) <= tr[-1] / 1e9]
    f = lambda c: [float(r[c]) for r in rr if r.get(c) not in (None, "")]
    cc = [c for c in rr[0] if c.startswith("core")]
    core = np.array([[float(r[c]) for c in cc] for r in rr])
    out["sys"] = dict(samples=len(rr), sensing_cpu=st(f("sensing_cpu_pct")), sensing_vram_max=max(f("sensing_vram_mib") or [0]),
                      sensing_threads_max=max(f("sensing_threads") or [0]), gpu_util=st(f("gpu_util_pct")), gpu_mem_max=max(f("gpu_mem_mib")),
                      core_total_mean=float(core.mean()), core_mean=[round(float(x), 1) for x in core.mean(0)])
    print(f"[5-1] sensing CPU% {fmt(out['sys']['sensing_cpu'])}, VRAM 최대 {out['sys']['sensing_vram_max']:.0f} MiB, 스레드 최대 {out['sys']['sensing_threads_max']:.0f}; "
          f"GPU 사용률 {fmt(out['sys']['gpu_util'])}%, GPU 메모리 최대 {out['sys']['gpu_mem_max']:.0f} MiB; 논리 CPU 평균 {out['sys']['core_total_mean']:.1f}% {out['sys']['core_mean']}")
json.dump(out, open(f"{O}/analysis.json", "w"), indent=1, default=float)

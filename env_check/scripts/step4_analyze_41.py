"""step4_analyze_41.py <접두어> [--loop-window] — 4-1/4-2/4-3 데이터 경로 분석 (시스템 python3 -B, numpy)

  HumanPose        : 재생 발행(seq, t_send) ↔ Isaac 수신(seq, stamp, t_recv) → 누락률, 지연(stamp → 수신)
  HumanDynamicsState (수정 기준): 재생 구간 [첫 발행, 마지막 발행] 안에 stamp 가 있는 메시지로
                     수신 주기 평균 Hz, 최대 수신 간격, 지연(노드 stamp → Isaac 수신)
  짝짓기·정확성    : 각 HumanDynamicsState 위치(19×3)를 재생 프레임별 독립 계산값과 비교해 최근접 프레임 → 거리 ≤ 1 mm 이면 짝
                     (i) 완전 독립(모델 파일 numpy), (ii) 노드 코드 상수 + numpy 자세 계산. 두 번째 후보와의 차이도 기록
                     데이터 나이 = Isaac 수신 시각 − 짝지은 HumanPose 발행 시각
  노드 부하        : <접두어>_nodeperf_paper_human_dynamics_perf.csv (노드 자체 단계별 시간), <접두어>_node_monitor.csv
"""
import csv, json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step4_ref_fk as F

P = sys.argv[1]
st = lambda v: dict(n=int(len(v)), mean=float(np.mean(v)), p95=float(np.percentile(v, 95)), max=float(np.max(v))) if len(v) else dict(n=0)
fmt = lambda d: (f"n={d['n']} 평균 {d['mean']:.3f} / p95 {d['p95']:.3f} / 최대 {d['max']:.3f}" if d.get("n") else "n=0")
meta = json.load(open(P + "_replay_meta.json"))
sent = np.array([[int(r["seq"]), int(r["t_send_ns"])] for r in csv.DictReader(open(P + "_replay_sent.csv"))], dtype=np.int64)
t_send = dict(zip(sent[:, 0].tolist(), sent[:, 1].tolist()))
R = np.load(P + "_human_rec.npz")
out = {}
# ---- HumanPose ----
pose = R["pose"]
pose = pose[pose[:, 0] >= 0]
got = {}
for seq, stamp, trecv, nv in pose.tolist():
    got.setdefault(seq, (stamp, trecv, nv))
lat = [(got[s][1] - got[s][0]) / 1e6 for s in t_send if s in got]
loss = 1 - sum(1 for s in t_send if s in got) / len(t_send)
out["pose"] = dict(sent=len(t_send), recv=sum(1 for s in t_send if s in got), dup=int(len(pose) - len(got)), loss_pct=100 * loss,
                   latency_ms=st(lat), verts_per_msg=int(pose[0, 3]) if len(pose) else None)
print(f"[41] HumanPose: 발행 {len(t_send)} 수신 {out['pose']['recv']} 누락률 {100 * loss:.3f}% 중복 {out['pose']['dup']} "
      f"지연(stamp→수신) {fmt(out['pose']['latency_ms'])} ms, 정점 {out['pose']['verts_per_msg']}개/메시지")
# ---- HumanDynamicsState ----
ds, dr, dp = R["dyn_stamp"], R["dyn_recv"], R["dyn_pos"].astype(np.float64)
lo, hi = sent[0, 1], sent[-1, 1]
w = (ds >= lo) & (ds <= hi + 50_000_000)
iv = np.diff(dr[w]) / 1e6
dur = (dr[w][-1] - dr[w][0]) / 1e9 if w.sum() > 1 else float("nan")
out["dyn"] = dict(n_total=int(len(ds)), n_window=int(w.sum()), rate_hz=float((w.sum() - 1) / dur), max_interval_ms=float(iv.max()),
                  interval_ms=st(iv), latency_stamp_to_recv_ms=st((dr[w] - ds[w]) / 1e6))
print(f"[41] HumanDynamicsState: 전체 {len(ds)}개, 재생 구간 {w.sum()}개, 수신 주기 평균 {out['dyn']['rate_hz']:.2f} Hz, "
      f"최대 수신 간격 {iv.max():.2f} ms (간격 {fmt(out['dyn']['interval_ms'])} ms), 지연(노드 stamp→수신) {fmt(out['dyn']['latency_stamp_to_recv_ms'])} ms")
# ---- 짝짓기·정확성 ----
z = np.load(meta["npz"])
c_i = F.rest_constants(np.zeros(10))
c_ii = dict(np.load(os.path.expanduser("~/effmass_mpc/env_check/step4/node_consts_betas0.npz")))
n = meta["n"]
for tag, c in [("i", c_i), ("ii", c_ii)]:
    ref, _, _ = F.com_world(z["poses"][:n], z["trans"][:n], c)          # (N,19,3)
    D = np.linalg.norm(dp[w][:, None, :, :] - ref[None, :, :, :], axis=3).max(axis=2)   # (M,N) 분절 최대 오차
    best = D.argmin(1); d1 = D[np.arange(len(best)), best]
    D2 = D.copy(); D2[np.arange(len(best)), best] = np.inf; d2 = D2.min(1)
    ok = d1 <= 1e-3
    age = [(dr[w][k] - t_send[int(best[k])]) / 1e6 for k in range(len(best)) if ok[k]]
    frames = best[ok]
    uniq = np.unique(frames)
    repeats = int(np.sum(np.diff(frames) == 0))
    out[f"match_{tag}"] = dict(matched=int(ok.sum()), of=int(len(ok)), max_err_mm=float(d1.max() * 1000), err_mm=st(d1 * 1000),
                               second_gap_mm_min=float((d2 - d1).min() * 1000), data_age_ms=st(age),
                               frames_covered=int(len(uniq)), frames_total=int(n), consecutive_same_frame=repeats,
                               frames_skipped=int(n - len(uniq)))
    m = out[f"match_{tag}"]
    print(f"[41] 짝짓기 ({tag}): {m['matched']}/{m['of']} 이 1 mm 이내, 최대 오차 {m['max_err_mm']:.4f} mm (오차 {fmt(m['err_mm'])} mm), "
          f"두번째 후보와 최소 차 {m['second_gap_mm_min']:.3f} mm")
    print(f"[41]   데이터 나이(HumanPose 발행→Isaac 수신) {fmt(m['data_age_ms'])} ms; 재생 프레임 {n}개 중 계산된 프레임 {m['frames_covered']} "
          f"(건너뛴 {m['frames_skipped']}), 같은 프레임 연속 발행 {repeats}회")
# 속도: 같은 프레임 반복 시 속도 0 인지 (노드 동작 확인)
vel = R["dyn_vel"][w]
zero_v = np.all(vel == 0, axis=(1, 2))
out["dyn_zero_velocity_msgs"] = int(zero_v.sum())
print(f"[41] 속도가 모두 0 인 HumanDynamicsState (재생 구간) {zero_v.sum()}개")
# ---- 노드 부하 ----
pf = P + "_nodeperf_paper_human_dynamics_perf.csv"
if os.path.exists(pf):
    rows = list(csv.DictReader(open(pf)))
    tot = np.array([float(r["total_operation_ms"]) for r in rows])
    parts = {k: np.array([float(r[k]) for r in rows]) for k in ["rotation_conversion_ms", "rotation_matrix_ms", "rigid_fk_com_ms",
                                                              "jacobian_ms", "mass_matrix_ms", "state_publish_prep_ms", "publish_ms"]}
    out["node_perf"] = dict(rows=len(rows), first_total_ms=float(tot[0]), total_ms_after_first=st(tot[1:]),
                            deadline_miss_after_first=int(sum(int(r["deadline_miss"]) for r in rows[1:])),
                            parts_mean_after_first={k: float(v[1:].mean()) for k, v in parts.items()})
    print(f"[41] 노드 계산 시간(노드 자체 기록, 첫 계산 제외): {fmt(out['node_perf']['total_ms_after_first'])} ms, 첫 계산 {tot[0]:.1f} ms, "
          f"33.3 ms 초과 {out['node_perf']['deadline_miss_after_first']}회 / {len(rows) - 1}")
    print("[41]   단계별 평균 " + ", ".join(f"{k[:-3]} {v:.3f}" for k, v in out['node_perf']['parts_mean_after_first'].items()) + " ms")
mf = P + "_node_monitor.csv"
if os.path.exists(mf):
    rows = list(csv.DictReader(open(mf)))
    cpu = np.array([float(r["cpu_pct"]) for r in rows]); rss = np.array([float(r["rss_mib"]) for r in rows])
    gm = np.array([float(r["gpu_mem_mib"]) for r in rows if r["gpu_mem_mib"]])
    gu = np.array([float(r["gpu_util_pct"]) for r in rows if r["gpu_util_pct"]])
    act = cpu[len(cpu) // 4: 3 * len(cpu) // 4] if len(cpu) > 8 else cpu
    out["node_monitor"] = dict(samples=len(rows), cpu_pct_mid=st(act), rss_mib_max=float(rss.max()), gpu_mem_mib_max=float(gm.max()) if len(gm) else None,
                               gpu_util_pct=st(gu))
    print(f"[41] 노드 프로세스: CPU% (재생 중간 절반, 1 s 간격) {fmt(out['node_monitor']['cpu_pct_mid'])}, RSS 최대 {rss.max():.0f} MiB, "
          f"GPU 메모리 {out['node_monitor']['gpu_mem_mib_max']} MiB, GPU 전체 사용률 {fmt(out['node_monitor']['gpu_util_pct'])}%")
json.dump(out, open(P + "_analysis41.json", "w"), indent=1, default=float)

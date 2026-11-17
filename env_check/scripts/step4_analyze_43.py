"""step4_analyze_43.py <접두어> <steps.csv> — 4-3/4-5 분석 (측정 루프 구간 기준, 시스템 python3 -B)"""
import csv, glob, json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step4_ref_fk as F
P, SF = sys.argv[1], sys.argv[2]
st = lambda v: dict(n=int(len(v)), mean=float(np.mean(v)), p95=float(np.percentile(v, 95)), max=float(np.max(v))) if len(v) else dict(n=0)
fmt = lambda d: (f"n={d['n']} 평균 {d['mean']:.3f} / p95 {d['p95']:.3f} / 최대 {d['max']:.3f}" if d.get("n") else "n=0")
im = json.load(open(P + "_isaac_meta.json"))
L0, L1 = im["loop_start_ns"], im["loop_end_ns"]
W0, W1 = im.get("warmup_start_ns"), im.get("warmup_end_ns")
meta = json.load(open(P + "_replay_meta.json"))
sent = np.array([[int(r["seq"]), int(r["t_send_ns"])] for r in csv.DictReader(open(P + "_replay_sent.csv"))], dtype=np.int64)
ts = dict(zip(sent[:, 0].tolist(), sent[:, 1].tolist()))
R = np.load(P + "_human_rec.npz")
out = dict(loop_s=(L1 - L0) / 1e9, replay_first_rel_loop_s=(sent[0, 1] - L0) / 1e9, replay_last_rel_loop_s=(sent[-1, 1] - L0) / 1e9,
           first_dyn_rel_loop_s=(im["human_first_dyn_ns"] - L0) / 1e9 if im.get("human_first_dyn_ns") else None,
           warmup_s=((W1 - W0) / 1e9) if W0 else None)
print(f"[43] 측정 루프 {out['loop_s']:.2f} s; 재생 첫 발행 {out['replay_first_rel_loop_s']:+.2f} s, 마지막 {out['replay_last_rel_loop_s']:+.2f} s (루프 시작 기준); "
      f"첫 HumanDynamicsState 수신 {out['first_dyn_rel_loop_s']:+.2f} s; 예열 {out['warmup_s']:.2f} s")
pose = R["pose"]; got = {}
for seq, stamp, trecv, nv in pose.tolist():
    if seq >= 0: got.setdefault(seq, (stamp, trecv, nv))
def pose_win(a, b, name):
    ss = [s for s, t in ts.items() if a <= t <= b]
    r = [s for s in ss if s in got]
    lat = [(got[s][1] - got[s][0]) / 1e6 for s in r]
    d = dict(sent=len(ss), recv=len(r), loss_pct=100 * (1 - len(r) / len(ss)) if ss else None, latency_ms=st(lat))
    print(f"[43] HumanPose ({name}): 발행 {d['sent']} 수신 {d['recv']} 누락률 {d['loss_pct']:.3f}% 지연 {fmt(d['latency_ms'])} ms, 정점 {got[r[0]][2] if r else '-'}개/메시지")
    return d
out["pose_loop"] = pose_win(L0, L1 - 100_000_000, "측정 루프")
if W0:
    out["pose_warmup"] = pose_win(W0, W1, "예열 구간, 측정과 별도")
ds, dr, dp = R["dyn_stamp"], R["dyn_recv"], R["dyn_pos"].astype(np.float64)
w = (dr >= L0) & (dr <= L1)
iv = np.diff(dr[w]) / 1e6
out["dyn_loop"] = dict(n=int(w.sum()), rate_hz=float((w.sum() - 1) / ((dr[w][-1] - dr[w][0]) / 1e9)), max_interval_ms=float(iv.max()),
                       interval_ms=st(iv), latency_ms=st((dr[w] - ds[w]) / 1e6))
print(f"[43] HumanDynamicsState (측정 루프): {w.sum()}개, 수신 주기 평균 {out['dyn_loop']['rate_hz']:.2f} Hz, 최대 간격 {iv.max():.2f} ms, "
      f"지연(노드 stamp→수신) {fmt(out['dyn_loop']['latency_ms'])} ms")
z = np.load(meta["npz"]); c = F.rest_constants(np.zeros(10)); n = meta["n"]
ref, _, _ = F.com_world(z["poses"][:n], z["trans"][:n], c)
D = np.linalg.norm(dp[w][:, None] - ref[None], axis=3).max(2); f = D.argmin(1); e = D[np.arange(len(f)), f]; ok = e <= 1e-3
age = [(dr[w][k] - ts[int(f[k])]) / 1e6 for k in range(len(f)) if ok[k]]
out["match_loop"] = dict(matched=int(ok.sum()), of=int(len(ok)), max_err_mm=float(e[ok].max() * 1000) if ok.any() else None,
                       exact_001mm=int((e <= 1e-5).sum()), data_age_ms=st(age))
# 속도 확인: 짝지은 연속 두 메시지의 프레임 차 g 별로, 노드 속도와 독립 계산 분절 COM 차분 (COM[f2]-COM[f1])*30/g 비교 (과제 2 와 같은 방법)
vv = R["dyn_vel"][w].astype(np.float64); VC = {}
for k in range(1, len(f)):
    if ok[k] and ok[k - 1]:
        g = int(f[k] - f[k - 1])
        true = (ref[f[k]] - ref[f[k - 1]]) * 30.0 / g if g > 0 else np.zeros((19, 3))
        VC.setdefault(g, []).append((float(np.abs(vv[k] - true).max()), float(np.linalg.norm(vv[k], axis=1).mean()),
                                     float(np.linalg.norm(true, axis=1).mean()), bool(np.all(vv[k] == 0))))
out["vel_check"] = {str(g): dict(pairs=len(v), max_abs_diff_ms=max(x[0] for x in v), p95_abs_diff_ms=float(np.percentile([x[0] for x in v], 95)),
                                 ratio_median=float(np.median([x[1] / x[2] for x in v if x[2] > 1e-6])) if any(x[2] > 1e-6 for x in v) else None,
                                 zero_vel=sum(x[3] for x in v)) for g, v in sorted(VC.items())}
print("[43] 속도 확인 (프레임 차별): " + "; ".join(f"차 {g}: 쌍 {d['pairs']}, 노드/독립 비 중앙 {d['ratio_median'] if d['ratio_median'] is None else round(d['ratio_median'], 3)}, "
      f"|노드−독립| 최대 {d['max_abs_diff_ms']:.3f} p95 {d['p95_abs_diff_ms']:.3f} m/s, 속도 0 {d['zero_vel']}" for g, d in out["vel_check"].items()))
print(f"[43] 위치 짝 (측정 루프): 1 mm 이내 {ok.sum()}/{len(ok)} (짝지은 것 최대 오차 {out['match_loop']['max_err_mm']:.4f} mm, 0.01 mm 이내 {out['match_loop']['exact_001mm']}), 데이터 나이 {fmt(out['match_loop']['data_age_ms'])} ms")
rows = list(csv.DictReader(open(SF)))
body = [r for r in rows if int(r["step"]) >= 1]
feas = [r for r in body if r["feasible"] == "1"]
TF = ["t_obst_ms", "t_phys_ms", "t_state_ms", "t_human_ms", "t_js_ms", "t_presync_ms", "t_solver_ms", "t_cmd_ms", "t_goal_ms", "t_diag_ms", "t_book_ms"]
m = {k: float(np.mean([float(r[k]) for r in feas])) for k in TF if k in feas[0]}
print("[43] 항목별 평균 (정상 구간, ms): " + "  ".join(f"{k[2:-3]} {v:.3f}" for k, v in m.items()) + f"  | 루프 {np.mean([float(r['loop_ms']) for r in feas]):.3f}")
new = [r for r in body if float(r.get("h_new", 0)) >= 1]
hk = ["t_human_ms", "t_h_pickup_ms", "t_h_transform_ms", "t_h_disp_ms", "t_h_tensor_ms"]
out["human_per_update"] = {k: st([float(r[k]) for r in new]) for k in hk}
out["human_per_step_mean"] = {k: float(np.mean([float(r[k]) for r in body])) for k in hk}
print(f"[43] 사람 처리: 새 메시지 처리 스텝 {len(new)}/{len(body)}; 처리한 스텝 기준 " +
      ", ".join(f"{k[2:-3]} 평균 {out['human_per_update'][k]['mean']:.3f}/p95 {out['human_per_update'][k]['p95']:.3f}" for k in hk) + " ms")
print(f"[43] 사람 처리: 전체 스텝 평균 " + ", ".join(f"{k[2:-3]} {v:.3f}" for k, v in out['human_per_step_mean'].items()) + " ms")
if "h_mesh_new" in body[0]:
    mn = [r for r in body if float(r["h_mesh_new"]) >= 1]
    out["mesh"] = dict(updates=len(mn), steps=len(body), per_update_ms=st([float(r["t_h_mesh_ms"]) for r in mn]),
                       xf_ms=st([float(r["t_h_mesh_xf_ms"]) for r in mn]),
                       write_ms=st([float(r["t_h_mesh_ms"]) - float(r["t_h_mesh_xf_ms"]) for r in mn]),
                       per_step_mean_ms=float(np.mean([float(r["t_h_mesh_ms"]) for r in body])),
                       human_ms_on_mesh_steps=st([float(r["t_human_ms"]) for r in mn]),
                       loop_ms_mesh_steps=float(np.mean([float(r["loop_ms"]) for r in mn if r["feasible"] == "1"])) if mn else None,
                       loop_ms_other_steps=float(np.mean([float(r["loop_ms"]) for r in body if float(r["h_mesh_new"]) < 1 and r["feasible"] == "1"])),
                       final=im.get("human_mesh_final"))
    if mn:
        print(f"[43] 메시 갱신: {len(mn)}/{len(body)} 스텝, 갱신 스텝 기준 {fmt(out['mesh']['per_update_ms'])} ms (변환 {out['mesh']['xf_ms']['mean']:.3f}, "
              f"쓰기 {out['mesh']['write_ms']['mean']:.3f}), 전체 스텝 평균 {out['mesh']['per_step_mean_ms']:.3f} ms; 루프 평균 갱신 스텝 {out['mesh']['loop_ms_mesh_steps']:.3f} / "
              f"그 밖 {out['mesh']['loop_ms_other_steps']:.3f} ms; 최종 {out['mesh']['final']}")
out["tensor_final"] = im.get("human_final")
print(f"[43] GPU 텐서 최종 확인: {im.get('human_final')}")
res = json.load(open(sorted(glob.glob(os.path.dirname(SF) + "/" + os.path.basename(P) + "_result_*.json"))[-1]))
out["graph_captures"] = res.get("graph_captures_by_stage")
print(f"[43] 그래프 캡처 (단계별): {out['graph_captures']}")
jf = P + "_sys_recv_js.csv"
if os.path.exists(jf):
    jr = {int(r["seq"]): r for r in csv.DictReader(open(jf))}
    js = {int(r["seq"]): r for r in csv.DictReader(open(P + "_isaac_sent_js.csv"))}
    both = [k for k in js if k in jr]
    same = sum(1 for k in both if all(float(js[k][f"p{i}"]) == float(jr[k][f"p{i}"]) for i in range(9)))
    jl = [(int(jr[k]["t_recv_ns"]) - int(jr[k]["stamp_ns"])) / 1e6 for k in both]
    out["js"] = dict(sent=len(js), recv=len(both), same=same, latency_ms=st(jl))
    print(f"[43] JointState (Isaac→시스템): 발행 {len(js)} 수신 {len(both)} 관절값 일치 {same} 지연 {fmt(out['js']['latency_ms'])} ms")
mf = P + "_node_monitor.csv"
if os.path.exists(mf):
    rr = [r for r in csv.DictReader(open(mf)) if L0 / 1e9 <= float(r["t"]) <= L1 / 1e9]
    cpu = [float(r["cpu_pct"]) for r in rr]; gm = [float(r["gpu_mem_mib"]) for r in rr if r["gpu_mem_mib"]]
    gu = [float(r["gpu_util_pct"]) for r in rr if r["gpu_util_pct"]]
    out["node_loop"] = dict(cpu_pct=st(cpu), gpu_mem_mib_max=max(gm) if gm else None, gpu_util_pct=st(gu))
    print(f"[43] 노드 (측정 루프 동안): CPU% {fmt(out['node_loop']['cpu_pct'])}, GPU 메모리 최대 {out['node_loop']['gpu_mem_mib_max']} MiB, "
          f"GPU 전체 사용률 {fmt(out['node_loop']['gpu_util_pct'])}%")
pf = P + "_nodeperf_paper_human_dynamics_perf.csv"
if os.path.exists(pf):
    rr = [r for r in csv.DictReader(open(pf)) if L0 / 1e9 <= float(r["t_ros_sec"]) <= L1 / 1e9]
    tot = [float(r["total_operation_ms"]) for r in rr]; tt = [float(r["t_ros_sec"]) for r in rr]
    dm = int(sum(int(r["deadline_miss"]) for r in rr))
    out["node_perf_loop"] = dict(total_ms=st(tot), deadline_miss=dm, miss_pct=100 * dm / len(rr), rows=len(rr),
                                 node_rate_hz=(len(rr) - 1) / (tt[-1] - tt[0]))
    print(f"[43] 노드 계산 (측정 루프 동안): {fmt(out['node_perf_loop']['total_ms'])} ms, 33.3 ms 초과 {dm}/{len(rr)}회 ({100 * dm / len(rr):.1f}%), "
          f"노드 계산 기록 주기 {out['node_perf_loop']['node_rate_hz']:.2f} Hz")
sf = P + "_sys_monitor.csv"
if os.path.exists(sf):
    rr = [r for r in csv.DictReader(open(sf)) if L0 / 1e9 + 1 <= float(r["t"]) <= L1 / 1e9]
    cols = list(rr[0].keys()); cc = [c for c in cols if c.startswith("core")]
    fl = lambda r, c: float(r[c]) if r.get(c) not in (None, "") else None
    sysd = dict(samples=len(rr), gpu_util_pct=st([fl(r, "gpu_util_pct") for r in rr if fl(r, "gpu_util_pct") is not None]),
                gpu_mem_mib_max=max(fl(r, "gpu_mem_mib") or 0 for r in rr))
    for l in [c[:-8] for c in cols if c.endswith("_cpu_pct")]:
        v = [fl(r, l + "_cpu_pct") for r in rr if fl(r, l + "_cpu_pct") is not None]
        vr = [fl(r, l + "_vram_mib") for r in rr if fl(r, l + "_vram_mib") is not None]
        lc = sorted({r[l + "_last_cpu"] for r in rr if r[l + "_last_cpu"]})
        thr = [fl(r, l + "_threads") for r in rr if r.get(l + "_threads") not in (None, "")]
        sysd[l] = dict(cpu_pct=st(v), vram_mib_max=max(vr) if vr else None, last_cpus=lc, threads_max=max(thr) if thr else None)
    core = np.array([[fl(r, c) for c in cc] for r in rr], dtype=float)
    sysd["core_mean_pct"] = [round(float(x), 1) for x in np.nanmean(core, 0)]
    sysd["core_total_mean_pct"] = float(np.nanmean(core))
    sysd["core_busiest_mean_pct"] = float(np.nanmean(np.nanmax(core, 1)))
    sysd["samples_any_core_ge90"] = int(np.sum(np.nanmax(core, 1) >= 90))
    out["sys_loop"] = sysd
    print(f"[43] 시스템 (측정 루프, 1 s 표본 {len(rr)}): GPU 사용률 {fmt(sysd['gpu_util_pct'])}%, GPU 메모리 최대 {sysd['gpu_mem_mib_max']:.0f} MiB")
    for l in [c[:-8] for c in cols if c.endswith("_cpu_pct")]:
        print(f"[43]   {l}: CPU% {fmt(sysd[l]['cpu_pct'])}, VRAM 최대 {sysd[l]['vram_mib_max']} MiB, 스레드 수 최대 {sysd[l].get('threads_max')}, 실행 코어(표본) {sysd[l]['last_cpus']}")
    print(f"[43]   코어별 평균 사용률 {sysd['core_mean_pct']}; 전체 평균 {sysd['core_total_mean_pct']:.1f}%, 매 표본 최다 사용 코어 평균 {sysd['core_busiest_mean_pct']:.1f}%, "
          f"90% 이상 코어가 있던 표본 {sysd['samples_any_core_ge90']}/{len(rr)}")
wf = P + "_worst_count.csv"
if os.path.exists(wf):
    wt = [int(x) for x in open(wf).read().split()[1:]]
    out["worst_head_meff"] = dict(total=len(wt), in_loop=sum(1 for t in wt if L0 <= t <= L1))
    print(f"[43] /hrc/worst_head_meff 수신: 전체 {len(wt)}개, 측정 루프 중 {out['worst_head_meff']['in_loop']}개")
json.dump(out, open(P + "_analysis43.json", "w"), indent=1, default=float)

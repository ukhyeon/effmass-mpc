"""step4_3d_summary.py — 4-3d (A)·(E)·(F) 각 3회 평균과 범위"""
import json, glob, csv, numpy as np
K = {"A": "(A) 노드 없음", "E": "(E) 노드 CPU", "F": "(F) 노드 CPU 1스레드"}
rows = {}
for c in "AEF":
    R = []
    for r in (1, 2, 3):
        t = f"step4_3d_{c}_run{r}"; a = json.load(open(f"step4/{t}_analysis43.json")); res = json.load(open(sorted(glob.glob(f"step2/{t}_result_*.json"))[-1]))
        f = sorted(glob.glob(f"step2/{t}_steps_*.csv"))[-1]; body = [x for x in csv.DictReader(open(f)) if int(x["step"]) >= 1 and x["feasible"] == "1"]
        it = {k: np.mean([float(x[k]) for x in body]) for k in ["t_obst_ms", "t_phys_ms", "t_state_ms", "t_human_ms", "t_solver_ms", "t_cmd_ms", "t_goal_ms"]}
        s = a["sys_loop"]; nd = s.get("node", {})
        d = dict(loop=res["loop_ms_mean"], rtf=res["realtime_factor"], solver_p95=res["solver_ms_p95"],
                 node_mean=a.get("node_perf_loop", {}).get("total_ms", {}).get("mean"), node_p95=a.get("node_perf_loop", {}).get("total_ms", {}).get("p95"),
                 node_miss=a.get("node_perf_loop", {}).get("miss_pct"), rate=a["dyn_loop"]["rate_hz"],
                 node_cpu=nd.get("cpu_pct", {}).get("mean") if nd.get("cpu_pct", {}).get("n") else None,
                 node_cpu_max=nd.get("cpu_pct", {}).get("max") if nd.get("cpu_pct", {}).get("n") else None, node_thr=nd.get("threads_max"),
                 isaac_cpu=s["isaac"]["cpu_pct"]["mean"], isaac_thr=s["isaac"].get("threads_max"), slack_cpu=s["slack"]["cpu_pct"]["mean"],
                 core_mean=s["core_total_mean_pct"], core_busiest=s["core_busiest_mean_pct"], gpu=s["gpu_util_pct"]["mean"],
                 mixed=a["match_loop"]["of"] - a["match_loop"]["exact_001mm"], n_dyn=a["match_loop"]["of"], **it)
        R.append(d)
    rows[c] = R
def fmt(c, k, p=3):
    v = [r[k] for r in rows[c] if r[k] is not None]
    if not v: return "—"
    return f"{np.mean(v):.{p}f} ({min(v):.{p}f}~{max(v):.{p}f})"
items = [("루프 평균 (ms)", "loop", 3), ("실시간 배율", "rtf", 3), ("solver p95 (ms)", "solver_p95", 3),
         ("장애물 갱신", "t_obst_ms", 3), ("물리", "t_phys_ms", 3), ("상태 읽기", "t_state_ms", 3), ("사람 처리", "t_human_ms", 3),
         ("solver 평균", "t_solver_ms", 3), ("명령 적용", "t_cmd_ms", 3), ("목표 판정", "t_goal_ms", 3),
         ("노드 계산 평균 (ms)", "node_mean", 2), ("노드 계산 p95 (ms)", "node_p95", 2), ("노드 33.3 ms 초과 (%)", "node_miss", 1),
         ("HumanDynamicsState 수신 주기 (Hz)", "rate", 2), ("노드 CPU% 평균", "node_cpu", 1), ("노드 CPU% 1초 최대", "node_cpu_max", 1),
         ("노드 스레드 수 (최대)", "node_thr", 0), ("Isaac CPU% 평균", "isaac_cpu", 0), ("Isaac 스레드 수 (최대)", "isaac_thr", 0),
         ("논리 CPU 12개 평균 사용률 (%)", "core_mean", 1), ("매 표본 가장 바쁜 코어 평균 (%)", "core_busiest", 1),
         ("GPU 사용률 평균 (%)", "gpu", 1), ("Slack CPU% 평균 (측정과 무관)", "slack_cpu", 0),
         ("자세·위치 섞임 수 (0.01 mm 초과) / 측정 루프 메시지", "mixed", 0)]
print("| 항목 (3회 평균, 괄호는 범위) | " + " | ".join(K.values()) + " |"); print("|---" * 4 + "|")
for nm, k, p in items:
    extra = ""
    if k == "mixed":
        print(f"| {nm} | " + " | ".join(", ".join(f"{r['mixed']}/{r['n_dyn']}" for r in rows[c]) for c in "AEF") + " |"); continue
    print(f"| {nm} | " + " | ".join(fmt(c, k, p) for c in "AEF") + " |")
print()
for c in "AEF":
    print(c, "회차별 루프", [round(r["loop"], 3) for r in rows[c]], "배율", [r["rtf"] for r in rows[c]])
la, lf, le = [[r["loop"] for r in rows[c]] for c in "AFE"]
print(f"범위 비교: (A) {min(la):.3f}~{max(la):.3f}, (F) {min(lf):.3f}~{max(lf):.3f}, (E) {min(le):.3f}~{max(le):.3f} ms; (F)−(A) 평균 차 {np.mean(lf) - np.mean(la):+.3f} ms, (E)−(A) {np.mean(le) - np.mean(la):+.3f} ms")
# 코어별 사용률 평균 (3회 평균)
for c in "AEF":
    cm = np.mean([json.load(open(f"step4/step4_3d_{c}_run{r}_analysis43.json"))["sys_loop"]["core_mean_pct"] for r in (1, 2, 3)], 0)
    print(f"{c} 코어별 평균 사용률 (논리 0~11, 3회 평균): {np.round(cm, 1).tolist()}")

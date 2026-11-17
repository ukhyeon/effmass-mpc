"""step5_2_summary.py — 5-2 (G)·(H) 각 3회 평균(범위)과 4-3d (A)·(F) 나란히"""
import csv, glob, json, numpy as np
TF = ["t_obst_ms", "t_phys_ms", "t_state_ms", "t_human_ms", "t_h_mesh_ms", "t_js_ms", "t_presync_ms", "t_solver_ms", "t_cmd_ms", "t_goal_ms", "loop_ms"]
NM = ["장애물 갱신", "물리", "상태 읽기", "사람 처리 (메시 포함)", "└ 메시 갱신", "JointState 발행", "solver 전 동기화", "solver", "명령 적용", "목표 판정", "**루프**"]
def items(steps):
    rows = [r for r in csv.DictReader(open(steps)) if int(r["step"]) >= 1 and r["feasible"] == "1"]
    return {k: (float(np.mean([float(r[k]) for r in rows])) if k in rows[0] else None) for k in TF}
C = {}
for c in "AF":
    L = []
    for r in (1, 2, 3):
        t = f"step4_3d_{c}_run{r}"; a = json.load(open(f"step4/{t}_analysis43.json")); res = json.load(open(sorted(glob.glob(f"step2/{t}_result_*.json"))[-1]))
        s = a["sys_loop"]
        L.append(dict(it=items(sorted(glob.glob(f"step2/{t}_steps_*.csv"))[-1]), rtf=res["realtime_factor"], sp95=res["solver_ms_p95"],
                      pose_rate=None, pose_max=None, pose_lat=a["pose_loop"]["latency_ms"]["p95"], loss=a["pose_loop"]["loss_pct"],
                      dyn_rate=a["dyn_loop"]["rate_hz"], dyn_max=a["dyn_loop"]["max_interval_ms"], dyn_lat=a["dyn_loop"]["latency_ms"]["p95"],
                      node_mean=a.get("node_perf_loop", {}).get("total_ms", {}).get("mean"), node_p95=a.get("node_perf_loop", {}).get("total_ms", {}).get("p95"),
                      node_miss=a.get("node_perf_loop", {}).get("miss_pct"), gpu=s["gpu_util_pct"]["mean"], isaac_cpu=s["isaac"]["cpu_pct"]["mean"],
                      node_cpu=s.get("node", {}).get("cpu_pct", {}).get("mean"), node_vram=s.get("node", {}).get("vram_mib_max"),
                      sens_cpu=None, sens_vram=None, sens_total=None, sens_rate=None, core=s["core_total_mean_pct"]))
    C[{"A": "4-3d (A) 노드 없음·재생", "F": "4-3d (F) 노드 CPU·재생"}[c]] = L
for c, nm in (("G", "5-2 (G) sensing + 노드 CPU"), ("H", "5-2 (H) sensing + 노드 GPU")):
    L = []
    for r in (1, 2, 3):
        d = f"step5/step5_2_{c}_r{r}"; a = json.load(open(f"{d}/analysis.json")); s = a["sys"]
        L.append(dict(it=items(sorted(glob.glob(f"step2/step5_2_{c}_r{r}_steps_*.csv"))[-1]), rtf=a["rtf"], sp95=a["solver_p95"],
                      pose_rate=a["pose"]["rate_hz"], pose_max=a["pose"]["max_interval_ms"], pose_lat=a["pose"]["latency_ms"]["p95"], loss=a["pose"]["loss_pct"],
                      dyn_rate=a["dyn"]["rate_hz"], dyn_max=a["dyn"]["max_interval_ms"], dyn_lat=a["dyn"]["latency_ms"]["p95"],
                      node_mean=a["node"]["total_ms"]["mean"], node_p95=a["node"]["total_ms"]["p95"], node_miss=a["node"]["miss_pct"],
                      gpu=s["gpu_util"]["mean"], isaac_cpu=s["isaac"]["cpu"]["mean"], node_cpu=s["node"]["cpu"]["mean"], node_vram=s["node"]["vram_max"],
                      sens_cpu=s["sensing"]["cpu"]["mean"], sens_vram=s["sensing"]["vram_max"], sens_total=a["sensing"]["total_operation_ms"]["mean"],
                      sens_rate=a["sensing"]["rate_hz"], core=s["core_total_mean"]))
    C[nm] = L
def f(L, g, p=3):
    v = [g(x) for x in L]; v = [x for x in v if x is not None]
    return "—" if not v else f"{np.mean(v):.{p}f} ({min(v):.{p}f}~{max(v):.{p}f})"
H = list(C)
print("| 항목 (3회 평균, 범위) | " + " | ".join(H) + " |"); print("|---" * (len(H) + 1) + "|")
for k, nm in zip(TF, NM):
    print(f"| {nm} | " + " | ".join(f(C[h], lambda x: x["it"][k]) for h in H) + " |")
rows = [("**실시간 배율**", "rtf", 3), ("solver p95 (ms)", "sp95", 3), ("HumanPose 주기 (Isaac 수신, Hz)", "pose_rate", 2), ("HumanPose 최대 수신 간격 (ms)", "pose_max", 1),
        ("HumanPose 지연 p95 (ms)", "pose_lat", 3), ("HumanPose 누락 (%)", "loss", 1), ("HumanDynamicsState 주기 (Hz)", "dyn_rate", 2),
        ("HumanDynamicsState 최대 간격 (ms)", "dyn_max", 1), ("HumanDynamicsState 지연 p95 (ms)", "dyn_lat", 3), ("노드 계산 평균 (ms)", "node_mean", 2),
        ("노드 계산 p95 (ms)", "node_p95", 2), ("노드 33.3 ms 초과 (%)", "node_miss", 1), ("sensing 메시지당 처리 평균 (ms)", "sens_total", 2),
        ("sensing 발행 주기 (자체 기록, Hz)", "sens_rate", 2), ("GPU 사용률 평균 (%)", "gpu", 1), ("Isaac CPU% 평균", "isaac_cpu", 0),
        ("노드 CPU% 평균", "node_cpu", 0), ("노드 VRAM 최대 (MiB)", "node_vram", 0), ("sensing CPU% 평균", "sens_cpu", 0), ("sensing VRAM 최대 (MiB)", "sens_vram", 0),
        ("논리 CPU 12개 평균 (%)", "core", 1)]
for nm, k, p in rows:
    print(f"| {nm} | " + " | ".join(f(C[h], lambda x, k=k: x[k], p) for h in H) + " |")
print()
for h in H:
    print(h, "회차별 배율", [x["rtf"] for x in C[h]], "루프", [round(x["it"]["loop_ms"], 3) for x in C[h]])

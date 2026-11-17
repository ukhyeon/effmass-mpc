"""step5_5_summary.py — 5-5 (I)·(J) 각 3회 평균(범위)과 회차별"""
import csv, glob, json, numpy as np
def load(c, r):
    a = json.load(open(f"step5/step5_5_{c}_r{r}/analysis.json")); res = a["res"]; s = a["sys"]
    return dict(miss=res["deadline_miss_pct"], hz=res["achieved_hz"], sm=res["solver_ms_mean"], sp=res["solver_ms_p95"], sx=res["solver_ms_max"],
                busy=a["items"]["busy_ms"]["mean"], busyp=a["items"]["busy_ms"]["p95"], lat=res["late_ms_max"],
                obst=a["items"]["t_obst_ms"]["mean"], human=a["items"]["t_human_ms"]["mean"], js=a["items"]["t_js_ms"]["mean"], pre=a["items"]["t_presync_ms"]["mean"],
                cmd=a["items"]["t_cmd_ms"]["mean"], goal=a["items"]["t_goal_ms"]["mean"],
                prate=a["pose"]["rate_hz"], pmax=a["pose"]["max_interval_ms"], plat=a["pose"]["latency_ms"]["p95"], loss=a["pose"]["loss_pct"], verts=a["pose"]["with_vertices"],
                drate=a["dyn"]["rate_hz"], dmax=a["dyn"]["max_interval_ms"], dlat=a["dyn"]["latency_ms"]["p95"],
                stot=a["sensing"]["total_operation_ms"]["mean"], scap=a["sensing"]["capture_ms"]["mean"], sinf=a["sensing"]["pose_inference_ms"]["mean"],
                svis=a["sensing"]["visualization_ms"]["mean"], spub=a["sensing"]["publish_ms"]["mean"],
                node=a["node"]["total_ms"]["mean"], nodep=a["node"]["total_ms"]["p95"], gpu=s["gpu_util"]["mean"], core=s["core_mean"],
                mcpu=s["mpc"]["cpu"]["mean"], mvr=s["mpc"]["vram_max"], ncpu=s["node"]["cpu"]["mean"], scpu=s["sensing"]["cpu"]["mean"], svr=s["sensing"]["vram_max"],
                graph=res["graph_captures_by_stage"], inf=res["infeasible"])
D = {c: [load(c, r) for r in (1, 2, 3)] for c in "IJ"}
f = lambda L, k, p=2: f"{np.mean([x[k] for x in L]):.{p}f} ({min(x[k] for x in L):.{p}f}~{max(x[k] for x in L):.{p}f})"
rows = [("**마감 초과 (%)**", "miss", 2), ("달성 주기 (Hz)", "hz", 2), ("solver 평균 (ms)", "sm", 3), ("**solver p95 (ms)**", "sp", 3), ("solver 최대 (ms)", "sx", 2),
        ("스텝 바쁜 시간 평균 / p95 (ms)", "busy", 3), ("", "busyp", 3), ("예정 대비 늦음 최대 (ms)", "lat", 2),
        ("장애물 갱신", "obst", 3), ("사람 처리", "human", 3), ("JointState 발행", "js", 3), ("solver 전 동기화", "pre", 3), ("명령 적용", "cmd", 3), ("목표 판정", "goal", 3),
        ("**HumanPose 주기 (Hz)**", "prate", 2), ("**HumanPose 최대 간격 (ms)**", "pmax", 1), ("HumanPose 지연 p95 (ms)", "plat", 3), ("HumanPose 누락 (%)", "loss", 1),
        ("정점 있는 HumanPose 수", "verts", 0), ("**HumanDynamicsState 주기 (Hz)**", "drate", 2), ("HumanDynamicsState 최대 간격 (ms)", "dmax", 1),
        ("HumanDynamicsState 지연 p95 (ms)", "dlat", 3), ("sensing 메시지당 합계 (ms)", "stot", 2), ("└ 캡처 대기 / 추론 / 정점·준비 / 발행", None, 0),
        ("노드 계산 평균 / p95 (ms)", "node", 2), ("", "nodep", 2), ("GPU 사용률 평균 (%)", "gpu", 1), ("논리 CPU 평균 (%)", "core", 1),
        ("MPC CPU% / VRAM (MiB)", None, 1), ("노드 CPU%", "ncpu", 0), ("sensing CPU% / VRAM (MiB)", None, 2)]
print("| 항목 (3회 평균, 범위) | (I) sensing 정점 계산 유지 | (J) sensing 정점 계산 끔 |"); print("|---|---|---|")
for nm, k, p in rows:
    if k is None:
        if nm.startswith("└"):
            print(f"| {nm} | " + " | ".join(" / ".join(f"{np.mean([x[q] for x in D[c]]):.2f}" for q in ("scap", "sinf", "svis", "spub")) for c in "IJ") + " |")
        elif nm.startswith("MPC"):
            print(f"| {nm} | " + " | ".join(f"{np.mean([x['mcpu'] for x in D[c]]):.0f} / {max(x['mvr'] for x in D[c]):.0f}" for c in "IJ") + " |")
        else:
            print(f"| {nm} | " + " | ".join(f"{np.mean([x['scpu'] for x in D[c]]):.0f} / {max(x['svr'] for x in D[c]):.0f}" for c in "IJ") + " |")
        continue
    print(f"| {nm or '└ p95'} | " + " | ".join(f(D[c], k, p) for c in "IJ") + " |")
print()
print("| 회차 | 마감 초과 (%) | solver p95 / 최대 (ms) | HumanPose (Hz) / 최대 간격 (ms) | HumanDynamicsState (Hz) | GPU (%) | 그래프 | infeasible |"); print("|---|---|---|---|---|---|---|---|")
for c in "IJ":
    for r, x in enumerate(D[c], 1):
        print(f"| ({c}) {r}회차{' (sensing 먼저)' if c == 'I' and r == 1 else ''} | {x['miss']:.2f} | {x['sp']:.2f} / {x['sx']:.2f} | {x['prate']:.2f} / {x['pmax']:.1f} | {x['drate']:.2f} | {x['gpu']:.1f} | {x['graph']} | {x['inf']} |")

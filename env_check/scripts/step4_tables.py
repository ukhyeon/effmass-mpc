"""step4_tables.py — 4-3·4-3b·4-3c·4-5 비교표 (각 경우 2회차). 시스템 python3 -B"""
import json, csv, glob, numpy as np
runs = [('3단계 (b)', 'step3r_b_run2', None), ('4-3', 'step4_3_run2', 'step4_3_run2'), ('(A) 노드 없음', 'step4_3A_run2', 'step4_3A_run2'),
        ('(B) Isaac 처리 끔', 'step4_3B_run2', 'step4_3B_run2'), ('(C) 64방향 끔', 'step4_3C_run2', 'step4_3C_run2'),
        ('(D) C+코어 분리', 'step4_3D_run2', 'step4_3D_run2'), ('(E) C+노드 CPU', 'step4_3E_run2', 'step4_3E_run2'),
        ('4-5 C+메시', 'step4_5_run2', 'step4_5_run2')]
TF = ["t_obst_ms", "t_phys_ms", "t_state_ms", "t_human_ms", "t_h_mesh_ms", "t_js_ms", "t_presync_ms", "t_solver_ms", "t_cmd_ms", "t_goal_ms", "t_diag_ms", "t_book_ms", "loop_ms"]
NM = ["장애물 갱신", "물리 스텝", "상태 읽기", "사람 처리 (메시 포함)", "└ 메시 갱신", "JointState 발행", "solver 전 동기화", "solver", "명령 적용", "목표 판정", "점검 계산 (꺼짐)", "기록 준비", "**루프**"]
M, RS, AN = {}, {}, {}
for n, st, an in runs:
    f = sorted(glob.glob(f'step2/{st}_steps_*.csv'))[-1]
    rows = [r for r in csv.DictReader(open(f)) if int(r['step']) >= 1 and r['feasible'] == '1']
    M[n] = {k: (np.mean([float(r[k]) for r in rows]) if k in rows[0] else None) for k in TF}
    RS[n] = json.load(open(sorted(glob.glob(f'step2/{st}_result_*.json'))[-1]))
    AN[n] = json.load(open(f'step4/{an}_analysis43.json')) if an else None
names = [r[0] for r in runs]
print("| 항목 (정상 구간 평균, ms) | " + " | ".join(names) + " |"); print("|---" * (len(names) + 1) + "|")
for k, nm in zip(TF, NM):
    print(f"| {nm} | " + " | ".join("—" if M[n][k] is None else (f"**{M[n][k]:.3f}**" if k == "loop_ms" else f"{M[n][k]:.3f}") for n in names) + " |")
print("| 루프 − (A) | " + " | ".join(f"{M[n]['loop_ms'] - M['(A) 노드 없음']['loop_ms']:+.3f}" for n in names) + " |")
print("| solver 평균 / p95 | " + " | ".join(f"{RS[n]['solver_ms_mean']:.3f} / {RS[n]['solver_ms_p95']:.3f}" for n in names) + " |")
print("| **실시간 배율** | " + " | ".join(f"**{RS[n]['realtime_factor']:.3f}**" for n in names) + " |")
def g(n, f):
    try: return f(AN[n])
    except Exception: return "—"
R2 = names[1:]
print(); print("| 사람 데이터·노드·자원 (2회차, 측정 루프) | " + " | ".join(R2) + " |"); print("|---" * (len(R2) + 1) + "|")
rowsd = [("HumanPose 누락 / 지연 p95", lambda a: f"{a['pose_loop']['loss_pct']:.1f}% / {a['pose_loop']['latency_ms']['p95']:.3f} ms"),
         ("HumanDynamicsState 수신 주기 / 최대 간격", lambda a: f"{a['dyn_loop']['rate_hz']:.2f} Hz / {a['dyn_loop']['max_interval_ms']:.2f} ms"),
         ("위치 짝 1 mm / 0.01 mm", lambda a: f"{a['match_loop']['matched']}/{a['match_loop']['of']} / {a['match_loop'].get('exact_001mm', '—')}"),
         ("노드 계산 평균 / p95", lambda a: f"{a['node_perf_loop']['total_ms']['mean']:.2f} / {a['node_perf_loop']['total_ms']['p95']:.2f} ms"),
         ("노드 33.3 ms 초과", lambda a: f"{a['node_perf_loop']['deadline_miss']}/{a['node_perf_loop']['rows']} ({a['node_perf_loop']['miss_pct']:.1f}%)"),
         ("노드 발행 주기 (노드 기록)", lambda a: f"{a['node_perf_loop']['node_rate_hz']:.2f} Hz"),
         ("/hrc/worst_head_meff 수신 (전체)", lambda a: f"{a['worst_head_meff']['total']}"),
         ("GPU 전체 사용률 평균", lambda a: f"{a['sys_loop']['gpu_util_pct']['mean']:.1f}%"),
         ("Isaac CPU% 평균", lambda a: f"{a['sys_loop']['isaac']['cpu_pct']['mean']:.0f}%"),
         ("노드 CPU% 평균 / VRAM", lambda a: f"{a['sys_loop']['node']['cpu_pct']['mean']:.0f}% / {a['sys_loop']['node']['vram_mib_max'] or 0:.0f} MiB"),
         ("Slack CPU% 평균 (측정과 무관)", lambda a: f"{a['sys_loop']['slack']['cpu_pct']['mean']:.0f}%"),
         ("논리 CPU 12개 평균 사용률", lambda a: f"{a['sys_loop']['core_total_mean_pct']:.1f}%")]
for nm, f in rowsd:
    print(f"| {nm} | " + " | ".join(g(n, f) for n in R2) + " |")

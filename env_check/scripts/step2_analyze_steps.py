"""step2_analyze_steps.py <steps.csv> — 스텝별 기록을 구간별로 요약 (시스템 python3 로 실행 가능, numpy 불필요)"""
import csv, sys, math
path = sys.argv[1]
rows = list(csv.DictReader(open(path)))
for r in rows:
    for k in r:
        r[k] = float(r[k]) if r[k] not in ("", "nan") else float("nan")
LIM = 1000 / 60

def stats(xs):
    if not xs: return "n=0"
    s = sorted(xs); n = len(s); p95 = s[max(0, int(round(0.95 * n)) - 1)]
    return f"n={n:4d}  mean={sum(s)/n:7.3f}  p95={p95:7.3f}  max={s[-1]:8.3f}  >16.7ms={sum(1 for x in s if x > LIM)}"

body = [r for r in rows if r["step"] >= 1]   # step 0 = 첫 호출(그래프 캡처) 제외
feas = [r for r in body if r["feasible"] == 1]
infe = [r for r in body if r["feasible"] == 0]
print(f"file: {path}")
print(f"첫 호출 step 0: solver {rows[0]['solver_ms']:.2f} ms, attempts {int(rows[0]['attempts'])}, feasible {int(rows[0]['feasible'])}")
print("전체 (step≥1)     ", stats([r["solver_ms"] for r in body]))
print("정상 (feasible)   ", stats([r["solver_ms"] for r in feas]))
print("infeasible        ", stats([r["solver_ms"] for r in infe]))
for name, seg in [("정상", feas), ("infeasible", infe)]:
    at = {}
    for r in seg: at[int(r["attempts"])] = at.get(int(r["attempts"]), 0) + 1
    print(f"시도 횟수 분포 {name}: {dict(sorted(at.items()))}")
print("물리 step ms      ", stats([r["physics_ms"] for r in body]))
print("루프 ms (전체)    ", stats([r["loop_ms"] for r in body]))
print("루프 ms (정상)    ", stats([r["loop_ms"] for r in feas]))
# infeasible 구간(연속 구간) 목록
segs, start = [], None
for r in rows:
    if r["feasible"] == 0 and start is None: start = int(r["step"])
    if r["feasible"] == 1 and start is not None: segs.append((start, int(r["step"]) - 1)); start = None
if start is not None: segs.append((start, int(rows[-1]["step"])))
print(f"infeasible 연속 구간 {len(segs)}개: " + ", ".join(f"{a}-{b}({b-a+1})" for a, b in segs))
for a, b in segs:
    ra = rows[a]
    print(f"   구간 {a}-{b}: 진입 직전 step {a-1} 여유 {rows[a-1]['min_clear_mover_m']:+.4f} m → 진입 step 여유 {ra['min_clear_mover_m']:+.4f} m, "
          f"구간 최소 여유 {min(rows[k]['min_clear_mover_m'] for k in range(a, b+1)):+.4f} m, mover_x {ra['mover_x']:.3f}, goal {int(ra['goal_idx'])}")
# 목표 전환 step
sw = [int(r["step"]) for r in rows if r["goal_switched"] == 1]
print(f"목표 전환 step: {sw}")
# 16.7 ms 초과 정상 구간 step과 최대값 step 의 상황
def context(k):
    r = rows[k]; c = []
    if k == 0: c.append("첫 스텝")
    if k - 1 in sw: c.append("목표 전환 직후(전 스텝에서 전환)")
    if k in sw: c.append("이 스텝에서 목표 전환")
    if r["feasible"] == 0 and k > 0 and rows[k-1]["feasible"] == 1: c.append("infeasible 진입 스텝")
    if r["feasible"] == 1 and k > 0 and rows[k-1]["feasible"] == 0: c.append("infeasible 탈출 직후")
    if k + 1 < len(rows) and rows[k+1]["feasible"] == 0 and r["feasible"] == 1: c.append("다음 스텝 infeasible 진입")
    near = [s for s in sw if 0 < k - s <= 5]
    if near and "목표 전환 직후(전 스텝에서 전환)" not in c: c.append(f"목표 전환 {k-near[-1]} 스텝 후")
    return ", ".join(c) if c else "특이 상황 없음"
mx = max(body, key=lambda r: r["solver_ms"])
k = int(mx["step"])
print(f"최대값: step {k}  solver {mx['solver_ms']:.3f} ms  attempts {int(mx['attempts'])}  feasible {int(mx['feasible'])}  "
      f"physics {mx['physics_ms']:.2f} ms  → {context(k)}")
over_f = [r for r in feas if r["solver_ms"] > LIM]
print(f"정상 구간 16.7 ms 초과 {len(over_f)}개:")
for r in over_f:
    k = int(r["step"]); print(f"   step {k}: solver {r['solver_ms']:.3f} ms attempts {int(r['attempts'])} → {context(k)}")
top = sorted(body, key=lambda r: -r["solver_ms"])[:5]
print("상위 5개 solver 시간: " + ", ".join(f"step {int(r['step'])} {r['solver_ms']:.1f}ms(f={int(r['feasible'])},a={int(r['attempts'])})" for r in top))
sim = len(rows) / 60; wall_loop = sum(r["loop_ms"] for r in rows) / 1000
print(f"참고: 루프 시간 합 {wall_loop:.3f} s (시뮬레이션 {sim:.1f} s) → 루프 기준 배율 {sim/wall_loop:.3f}")
valid = [r for r in rows if r["min_clear_mover_m"] == r["min_clear_mover_m"]]  # nan 제외 (--no-diag 실행)
if valid:
    mr = min(valid, key=lambda r: r["min_clear_mover_m"])
    print(f"로봇 구–mover 최소 여유: {mr['min_clear_mover_m']:+.4f} m (step {int(mr['step'])})")
else:
    print("로봇 구–mover 최소 여유: 실행 중 계산 안 함 (--no-diag) → step3_posthoc_clearance.py 로 계산")
TF = ["t_obst_ms", "t_phys_ms", "t_state_ms", "t_js_ms", "t_presync_ms", "t_solver_ms", "t_cmd_ms", "t_goal_ms", "t_diag_ms", "t_book_ms"]
if TF[0] in rows[0]:
    for name, seg in [("정상", feas), ("infeasible", infe)]:
        if not seg: continue
        m = {k: sum(r[k] for r in seg) / len(seg) for k in TF}
        print(f"항목별 평균 ({name}, n={len(seg)}): " + "  ".join(f"{k[2:-3]} {v:.3f}" for k, v in m.items())
              + f"  | 합 {sum(m.values()):.3f}  루프 {sum(r['loop_ms'] for r in seg)/len(seg):.3f} ms")

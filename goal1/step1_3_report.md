# 목표 1 · 1-3 조합 1 비용 — A단계 보고서

- 작성: 2026-11-16
- 상태: **A단계 완료 (8~12절). B1단계(부위 효과 짝 비교, iso 조건 추가) 완료 (13절).** GUI 없음.

## B2a-4 round 2 summary (English, ASCII) - stopped at the gate, no main runs

- Calibration: front-cal x 0.31 (u <= 40 deg) and top-cal z 0.514 adopted. Seed 1 check: front, gap 3.67 cm (in 2-4 cm), but the finger is nearest (u 22.7 deg) where seed 0 had the gripper body (38.4 deg).
- Gate: front-cal (mass A eta - iso eta) = 3.82 cm < 4 cm on the seed 0 closest pose -> fail, no runs. On the seed 1 pose it would be 5.76 cm. Offline IK: -1 / -2 cm in height gives 4.38 / 5.08 cm (s0), +1 / +2 cm 6.14 / 5.71 cm.
- Hypotheses: not tested.

## B2a-4 summary (English, ASCII) - stopped at the calibration, no main runs

- Calibration: not met. front-cal (x only, 5 off runs): a 2.5-3.5 cm gap always came with the gripper body nearest and u 38-39 deg (> 30); with the fingers nearest the gap was 2.0 cm or less. Cause: the gripper is rotated about 30 deg about the vertical during the off pass. top-cal: the line must go down about 2 cm (now 5.2 cm), but lower lines cannot keep the 0.23 m end rule (max 0.228 m).
- Gate, runs, hypotheses: not done.
- B2a-3 withdrawn (offline gate had passed, +4.93 cm; 4 runs exist, not analysed).

## B2a-2 round 2 summary (English, ASCII) - stopped after the plate check, main runs not done

- (a) free side choice: not measured (main runs not done). In the check runs on free_front, off and uniform 0.10 (seeds 0, 1) all passed "up", but the path bulged 14-15 cm toward the person (TCP x 0.60-0.62 at the hand, line x 0.47), so off went through the hand.
- (b) slot-front paired comparison: not run.
- (c) the plates did change the behaviour (path pulled back to x 0.46-0.48; arrival 124 -> 141 for off s0; off s0 under the hand), but did not force a front route: uniform 0.10 went over the fingertip in 2/2 under the upper plate, with the arm 1.3 cm from it and plate rollouts in 60-208 of 284 steps (plate and human cost mixed).

## B2a-2 summary (English, ASCII) - stopped before runs (slot-top rules cannot all be met)

- (a) Existing B2a records: on the front line every condition with cost passed over the hand (mass A 8/8, uniform 0.10 8/8, uniform_matched 8/8, iso 7/8 up + 1/8 front); only off passed in front. mass A had the most upward u at the closest moment (71 deg from frontal vs iso 56, uniform 58) and the lowest mu (0.82 kg vs iso 1.08, uniform 1.01). On the top line all conditions passed over the hand.
- (b) free scene: not run (stopped at the slot check, see below).
- (c) slot hypotheses: not tested. slot-front satisfies all rules after moving the plates 6 cm behind the fingertip; slot-top cannot: with plates >= 0.22 m from the hand, the front plate leaves a 0.22 m corridor in front of the fingertip (robot on the front route stays 14.5 cm from it); blocking needs the plate within about 6 cm of the fingertip.

## B2a results summary (English, ASCII)

Gate (u to the nearest point, set A, gripper down; closest IK pose of each line):

| path | closest robot sphere | u vs arm axis (deg) | m_h (kg) | m_r (kg) | mu (kg) | eta mass A (m) | iso mu / eta |
|---|---|---|---|---|---|---|---|
| front | 59 fr3v2_1_rightfinger | 6.1 | 6.018 | 4.623 | 2.614 | 0.1143 | 0.531 / 0.0515 |
| top | 59 fr3v2_1_rightfinger | 90.2 | 0.950 | 5.606 | 0.812 | 0.0637 | 0.542 / 0.0521 |

- Gate: front (mass A eta - iso eta) = 6.28 cm >= 4 cm: pass. Recorded: front eta - top eta = 5.06 cm. No COM fallback; no eta at the limits.
- Hypothesis 1 (front mass A - iso > 0 and larger than top): front > 0 holds (pass-window gap shape +0.89 (8+/0-, p 0.008), mesh +1.95 (8+/0-, p 0.008) cm); larger than top only mesh-based and not significant (interaction mesh +0.77 cm, 7/8 positive, p 0.070; shape +0.17 cm, 5/8, p 0.727).
- Same budget (uniform_matched) - does mass A lower contact risk on front and save time on top? **No on front, partly on top**: on front mass A is closer (pass-window shape -2.18 (0+/8-, p 0.008) cm) and its risk is higher (sigma 0.02 max +0.000054 (7+/1-, p 0.070) m/s; sigma 0.04 max +0.00260 (8+/0-, p 0.008) m/s); on top mass A reaches B earlier (-0.363 (0+/8-, p 0.008) s) but passes the hand at the same time (crossing time +0.004 (2+/2-, p 1.000) s).
- Main caveats: (1) on front the MPC goes over the hand (TCP up to ~12 cm above the design line); the closest moment has u about 71 deg from the frontal direction, where mass A eta is small (0.064 m), so the frontal mass-A advantage is not exercised; (2) uniform 0.10 and uniform_matched on front never reached B (0/8 each; B lies inside their eta shell because the gripper body at B is close to the hand), and uniform 0.10 on top reached B 1/8; (3) the start pose sets the run-wide minimum in several conditions (half length 0.20 m is short for the gripper width plus eta), so pass-window gaps are used as the main gap metric (definition below).

## B2a gate summary (English, ASCII) - stopped before MPC runs

Gate table (scene g2, shapes v2, u to the hand COM, set A; gripper pointing down as in g1; values at the closest IK pose of each line):

| path | closest robot sphere | MPC sphere gap to hand shape (cm) | u vs arm axis (deg) | m_h (kg) | m_r (kg) | mu (kg) | eta set A (m) | iso mu / eta |
|---|---|---|---|---|---|---|---|---|
| front | 59 fr3v2_1_rightfinger | -0.03 | 10.3 | 4.578 | 4.567 | 2.286 | 0.1069 | 0.530 / 0.0515 |
| top | 59 fr3v2_1_rightfinger | 0.10 | 58.1 | 1.196 | 7.246 | 1.027 | 0.0716 | 0.554 / 0.0526 |
| side | 56 fr3v2_1_hand | -8.55 | 30.1 | 1.581 | 5.326 | 1.219 | 0.0781 | 0.539 / 0.0519 |

- Delta eta: front - top = 3.5 cm, front - side = 2.9 cm (gate >= 6 cm): **both fail -> no MPC runs (as instructed)**. No eta at eta_min/eta_max. iso: front - top -0.11 cm, front - side -0.04 cm.
- Hypothesis not tested (no runs). Main reasons: u goes to the hand COM, so at the front pass u is 10 deg off the arm axis (m_h 4.6 kg, not 7.9) and at the top pass 58 deg (COM 5 cm behind the finger row); mu is capped by m_r (4.6-7.2 kg); eta ~ sqrt(mu). The side pass does not test the side direction with this gripper orientation (the gripper body, not the TCP, is nearest; section B2a item 4).
- Only 'u to the closest point + set B (p 0.7)' passes both (6.7 / 6.7 cm); information only, not applied.

## Shapes v2 summary (English, ASCII)

- Shape count: v1 24 -> v2 21 active (+3 inactive padding to keep the GPU size 24). Kernel and cost unchanged; only shape -> segment mapping and geometry.
- Performance: paired solver increase over the 1-2d channel (mass A, max_all, precomputed replay, 1200 steps, 2 runs each, SM clock 1965 MHz): v1 0.707 ms, v2 0.694 ms (<= 1 ms, pass); v2 - v1 = -0.013 ms.
- Fit rule result: under-cover had priority where both targets could not be met (torso, legs, hands); those shapes became bulky (torso capsules r 0.14-0.17 m, nearly spheres; finger row r 0.046 m). See the over-cover p95 column and the figures. Needs the user's decision (section Shapes v2, item 8).

Under-cover p95 (cm, union of all shapes) / over-cover median and p95 (cm, all surface samples, inside the mesh = 0). static = B2a pose; replay max = worst of replay frames 0/300/600/900/1261.

| segment | v1 under static / replay max | v2 under static / replay max | v1 over median / p95 static | v2 over median / p95 static | v2 over median replay max |
|---|---|---|---|---|---|
| pelvis | 0.0 / 0.0 | 1.7 / 2.6 | 5.1 / 12.3 | 0.2 / 3.6 | 0.2 |
| left_thigh | 0.0 / 0.0 | 1.9 / 2.7 | 2.3 / 6.6 | 1.0 / 3.3 | 1.1 |
| right_thigh | 0.0 / 0.0 | 1.7 / 2.9 | 2.4 / 6.7 | 0.9 / 3.7 | 1.0 |
| abdomen | 0.0 / 1.3 | 1.8 / 2.3 | 2.6 / 8.9 | 0.0 / 2.5 | 0.0 |
| left_calf | 0.0 / 0.0 | 2.1 / 2.1 | 3.6 / 6.6 | 2.2 / 5.1 | 2.2 |
| right_calf | 0.0 / 0.1 | 1.7 / 2.2 | 3.5 / 6.6 | 2.1 / 5.2 | 2.2 |
| left_foot | 0.2 / 0.2 | 2.0 / 2.0 | 4.5 / 6.8 | 1.6 / 4.1 | 1.6 |
| right_foot | 0.0 / 0.0 | 2.1 / 2.1 | 3.7 / 5.9 | 0.7 / 3.4 | 0.8 |
| chest | 0.0 / 0.0 | 1.0 / 0.9 | 3.0 / 10.4 | 0.9 / 6.3 | 0.8 |
| neck | 0.0 / 0.0 | 0.8 / 1.4 | 0.0 / 1.6 | 0.0 / 0.6 | 0.0 |
| left_shoulder | 0.0 / 0.0 | 2.6 / 2.9 | 0.2 / 5.6 | - / - | - |
| right_shoulder | 0.0 / 0.0 | 2.2 / 2.8 | 0.2 / 5.0 | - / - | - |
| head | 0.0 / 0.0 | 0.0 / 0.0 | 1.7 / 3.8 | 1.7 / 3.8 | 1.9 |
| left_upper_arm | 0.0 / 0.0 | 1.7 / 1.8 | 2.2 / 4.5 | 0.0 / 1.8 | 0.0 |
| right_upper_arm | 0.0 / 0.0 | 2.1 / 1.6 | 2.5 / 4.1 | 0.2 / 1.7 | 0.1 |
| left_lower_arm | 0.0 / 0.0 | 1.6 / 2.1 | 2.0 / 3.0 | 0.5 / 1.3 | 0.4 |
| right_lower_arm | 0.0 / 0.0 | 1.3 / 1.6 | 1.7 / 2.8 | 0.6 / 1.6 | 0.5 |
| left_hand | 0.0 / 0.1 | 2.0 / 2.0 | 3.1 / 5.8 | 0.0 / 3.3 | 0.0 |
| right_hand | 0.2 / 0.3 | 2.0 / 2.0 | 1.6 / 4.5 | 0.0 / 3.2 | 0.0 |

- Targets: under p95 <= 2.0 cm, over median <= 0.5 cm. v2 shoulders have no own shape (covered by the chest capsule and upper arms).

## B2a.1 summary (display and shape check; written in English on request)

- Mesh-capsule alignment: body mesh from the node body model (SMPL-H, betas 0, same pose/orientation/translation/camera-to-world) sits on the capsules; per-segment p95 of vertex-outside-capsule distance <= 0.65 cm on the static pose (replay frames 0/300/600/900: <= 0.89 cm, last frame 1261: 2.56 cm). No global shift.
- Right hand over-cover by direction (capsule surface minus real hand surface): tip -0.4 cm, back of hand (top) +3.8 cm (local +4.3 cm), outward side +0.1 cm (local +0.8 cm). A path 3 cm from the capsule surface is about 2.6 cm from the real fingertips but 7.3 cm from the real back of the hand.

## B2a 맨 앞 요약 (정지 자세까지, 사용자 확인 대기)

- 정지 자세: 오른팔 앞 수평, 팔꿈치 실제 0.1°, 어깨 굴곡 86.8°. 손 형상 ↔ 몸통·어깨/쇄골 형상 최소 간격 **0.310 m → 통과** (사용자가 기준을 0.4 → 0.25 m 로 고침). precomputed 저장, GUI 확인 명령 준비. 사용자 확인 뒤 3번(장면 g2)부터.
- 관문 표는 m_r(대표 자세)이 장면 배치 뒤에 정해져 아직 없음. 사람 쪽 m_h: 정면 7.92 kg, 위 0.95 kg, 옆 0.58 kg. m_r 4.9 kg 가정이면 η 정면 − 위 = 6.0 cm (관문 6 cm 경계) [추정: m_r 가정].

## B1 맨 앞 요약

| 질문 | 결과 |
|---|---|
| 가설 1: 손 간격 mass A < uniform_matched | **반대.** mass A − uniform_matched = **+2.84 cm**, 같은 부호(+) seed **7 / 8**, 부호 검정 p 0.070. 가설 방향(−)은 1 / 8 |
| 가설 2: 몸통 간격 mass A > uniform_matched | **가설대로.** **+3.44 cm**, 같은 부호(+) seed **8 / 8**, p 0.008 |
| mass A 와 iso 의 부위군별 간격 차가 seed 간 흩어짐을 넘는가 | **넘지 않는다.** 6개 부위군 모두 평균 차 −1.4~+0.5 cm, 같은 부호 seed 최대 7 / 8 (다리, p 0.070), 나머지 p ≥ 0.29. seed 범위가 모든 부위군에서 겹침 |
| off 배율 확인 (A단계 1.147 vs 1-2d 1.32) | 비교 대상 차이다. 1.32 는 채널 없음, 1.147 은 채널 켬·모드 off. 같은 조건 재측정(모두 1,350 MHz): 채널 없음 1.287 → 채널 켬 1.140 (−0.15), 근접 기록 켬 1.053 (−0.09), precomputed 는 ROS 경로보다 +0.08~0.10 (1.224 / 1.385). 클럭은 원인 아님 |

- mass A 는 손을 포함해 **모든 부위군에서** uniform_matched 보다 멀다 (손 +2.8, 아래팔 +4.3, 위팔 +3.8, 몸통 +3.4, 다리 +3.1 cm). 부위 선택 효과라기보다 전체가 더 보수적이다 (13.(5) 해석).
- mass A·iso 는 seed 8개 중 3·4개에서 관절 4 하한(−3.042 rad)에 걸려 끝까지 멈춤 (infeasible 평균 120·146 스텝). off 는 8개 모두 사람 형상과 겹침 (−6~−16 cm, 45~109 스텝).
- 측정 전 남은 프로세스 0 (모든 실행에서 실행 스크립트가 시작 시 확인. 시작 전 /dev/shm 의 지난 Kit 세마포어 1개 정리, 8.(5) 참고)

## B1 후속 맨 앞 요약 (저장 기록만, 14절)

| 질문 | 결과 |
|---|---|
| (a) 선택성 C (몸통 − 손 최소 간격) 가 mass A 에서 uniform 대비 커지는가 | **아니다.** mass A − uniform_matched = +0.60 cm, 같은 부호 4/8 (p 1.00). mass A − uniform 0.10 = +1.31 cm, 6/8 (p 0.29). C2·C3 도 같은 부호 최대 6/8, p ≥ 0.29 |
| (b) 손이 최근접인 순간 비용을 정한 것은 손인가 몸통인가 | **둘 다 아니다.** mass A·iso 에서 그 순간 그 로봇 구의 비용은 100% 0 (실행 상태 기준). 손 간격 − η 중앙 +18.8 cm, 몸통은 그 구 0.3 m 안에 있을 때(23%) 간격 − η 중앙 +10.1 cm |
| (c) η 상한 포화 비율 (0.3 m 안 쌍) | mass A 6.7% (몸통 10.9%, 다리 77%), iso 10.4% (몸통 13.9%, 다리 91%). 손·팔은 0% |

- 사람 자세 자체: 손 캡슐과 가장 가까운 몸통 캡슐의 표면 간격은 재생 1,262 프레임에서 중앙 −8~−9 cm (겹침). 손이 몸통에서 η_max(0.20 m) 이상 떨어진 프레임은 왼손 0%, 오른손 1.3%.
- 몸통 쌍의 μ 는 로봇 쪽 m_r 이 정한다 (μ/m_r 중앙 0.83, 모든 몸통 쌍에서 μ/m_r > 0.5). 손 쌍은 사람 쪽이 정한다 (μ/m_h 중앙 0.90).
- 현재 η_matched 0.1272 대신 "max_all 비용을 낸 형상의 η" 평균으로 정하면 **0.189 m**.

## 맨 앞 요약

**(1) precomputed 검증 [확인]** — 노드 핵심(`HumanInertiaCore`, 설치본 import)으로 재생 파일 1,262 프레임 전부를 미리 계산 (`goal1/data/human_precomputed_1781524848.npz`)

| 대조 대상 (실제 노드 메시지) | 같은 입력 프레임 수 | COM | 역관성 | 형상 p0·p1·반지름 | 속도 |
|---|---|---|---|---|---|
| live 기록 `g1_live_nodrop` (끊김 없음) | 1,262 | 최대 차 0 (비트 동일) | 0 | 기록 없음 | 노드와 같은 dt 를 넣으면 0. 미리 계산 정의(dt = 1/30)로는 최대 7.3e-3 m/s (상대 1.4e-3) |
| live 기록 `g1_live_drop7` (7프레임마다 끊김) | 1,082 | 0 | 0 | 기록 없음 | 끊김 뒤 노드 dt 가 2프레임 간격이라 다름 (정의 차이) |
| Isaac 덤프 h_* (1-2d 실행 3개, 15개) | 15 | 0 | 0 | 0 (형상 종류·분절 같음) | dt 1/30 과 2e-7~6e-4 m/s |

- 속도 차는 노드 dt (입력 stamp 차, 재생 시스템 시계 지터 −0.22~+0.21 ms) 와 미리 계산 dt (재생 일정 k/30 의 차 = 1/30) 의 차이다. 계산 코드 차이는 0.
- 회전·관절 위치는 대조 기록이 없다 [미확인]. 같은 compute 결과에서 꺼내므로 같은 경로.

**(2) seed 재현성: 통과 [확인]** — precomputed, uniform η0 0.10, 같은 seed 두 번 (`g13p_uni10_a`, `g13p_uni10_b`)
- MPC 명령(측정 관절값) 1,200 스텝 비트 동일, 스텝별 사람 프레임 열 동일, 근접 쌍 기록 67,150행 동일.
- 같은 precomputed 의 off 와 η0 0.05 실행은 1-2d 기준 `g1d_off_1` 과도 비트 동일.

**(3) 성능 기준: 통과 [확인]** — 지금 경로 (재생 → 노드 → ROS → Isaac), 후보 A

| 기준 | 결과 | 판정 |
|---|---|---|
| mass/max_all 실시간 배율 ≥ 1.0 (따로 실행) | 1.114 | 통과 |
| 1-2d 채널 대비 solver 추가 ≤ 1 ms (짝, 평균) | mass/max_all +0.79 ms (중앙 +0.79, p95 +2.20) | 통과 |
| 참고: mass/nearest 짝 | +0.03 ms (중앙 −0.02, p95 +1.53) | 기록 |
| 재캡처 | 0 (예열 캡처만, 짝 2 / 따로 1), 텐서 주소 불변 | 통과 |

**(4) 파라미터 후보** (η = clamp(η0 (μ/μref)^p, η_min, η_max)). Δη 는 실행 경로 근접 쌍 μ 의 중앙값끼리 (precomputed off / 저장해 둔 ROS off). 기준: 노트 17 의 3.5 cm 의 2~3배 = 7~10.5 cm

| 세트 | η0 | μref | p | η_min | η_max | 손 vs 가슴 | 손 vs 배 | 아래팔 vs 가슴 | 상완 vs 가슴 | 오른손 안 방향 최대 vs 최소 | 가슴 안 방향 p95 vs p5 | η_min / η_max 에 걸린 쌍 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A (중간) | 0.10 | 2.0 | 0.5 | 0.05 | 0.20 | 8.3 / 8.5 cm | 9.4 / 9.6 | 4.6 / 4.6 | 2.6 / 2.6 | 1.5 / 1.6 | 5.3 / 5.2 | 0.1% / 5% |
| B (강함) | 0.10 | 2.0 | 0.7 | 0.04 | 0.20 | 11.6 / 11.8 | 13.4 / 13.5 | 6.8 / 7.0 | 4.1 / 4.1 | 1.7 / 1.8 | 6.4 / 6.4 | 4% / 13% |
| C (손 기준) | 0.05 | 0.66 | 0.5 | 0.05 | 0.15 | 7.2 / 7.4 | 8.2 / 8.3 | 4.0 / 4.0 | 2.3 / 2.3 | 0.8 / 0.9 | 4.2 / 4.2 | 21% / 10% |

- 손 vs 몸통은 세 세트 모두 7 cm 이상 (A 2.4~2.7배, B 3.3~3.8배, C 2.1~2.4배). 상완 vs 몸통, 손 안 방향 차는 세 세트 모두 3.5 cm 이하.
- mass 짧은 실행은 A 로 함 (9절).

**(5) 가장 가까운 거리가 생기는 곳 (비용 없음, precomputed off):** 두 번의 접근(스텝 309~313, 766~768)에서 로봇 손(fr3v2_1_hand 구 51) ↔ 사람 오른손 엄지(형상 23) 0.086 m, 로봇 link6 (구 31) ↔ 배(형상 5) 0.10 m 미만. ROS 경로 off 는 같은 두 구간에서 link6 ↔ 배가 최소 (0.085 m).

## 이전 요약 (ROS 경로, seed 재현성 불통과로 멈췄던 때)


| 항목 | 결과 | 판정 |
|---|---|---|
| **seed 재현성** | 채널 off: 같은 seed 실행끼리 명령 비트 동일 (1-2d 포함 10회 이상). **uniform η0 0.10 두 실행: 스텝 281부터 다름 (관절 최대 차 0.50 rad, 1,200 중 919 스텝)** | **불통과 → 멈춤** |
| 원인 [확인] | 같은 스텝에 쓰인 사람 HumanPose 순번이 두 실행에서 636/1,200 스텝 다름(대부분 1프레임). 비용이 rollout에서 켜지는 순간(실행 경로 최소 거리 0.214 m, rollout은 약 0.68 s 앞까지 봄)부터 갈라짐. MPPI 잡음은 결정적 | 기록 |
| off 모드 = 1-2d | 측정 관절값이 g1d_off_1(1-2d)과 같음 (Isaac). Isaac 없는 시험에서도 명령 비트 동일 | 통과 |
| 비용 식 | cuRobo `scale_eta_metric` 형태 = common14 `eta_cost` (별도 세션 확인) | 기록 |
| GPU ↔ CPU (합성 근접 366,000개 × 3설정) | 비용을 고른 형상 일치 100%, η 상대 차 ≤ 1.1e-5, 비용(가중 ≥ 1) 상대 차 ≤ 6.5e-5 | 통과 |
| GPU ↔ CPU (rollout 덤프 73,200개, mass) | 형상 100%, μ ≤ 1.4e-6, η ≤ 6e-7. 비용 > 0 인 531개 중 가중 ≥ 1 상대 차 최대 1.24e-4 (η − d 가 수 mm 인 float32 상쇄) | 기록 |
| 파라미터 변경 재캡처 | 실행 중 η0 0.10 → 0.15 바꿔도 캡처 1회(예열)뿐, 주소 불변 (Isaac 없는 시험) | 통과 |
| uniform η0 0.05 / 0.10 / 0.15 | 도달 5 / 5 / 5, infeasible 0. 최소 이격(MPC 구 기준) 0.085 / 0.090 / 0.126 m (off 0.085) | 기록 |

## 1. 비용 식과 출처

- **비용 형태 [확인: 코드]:** cuRobo `curobolib/cpp/sphere_obb_kernel.cu` `scale_eta_metric` (프로토타입이 쓰는 `PrimitiveCollisionCost` 의 sphere–OBB 커널)
  - d = 로봇 구 표면 ~ 형상 표면 거리 (커널 안에서는 구 반지름에 η 를 더해 sph_dist = η − d 로 다룸)
  - c(d, η) = 0 (d ≥ η), (η − d)² / (2η) (0 ≤ d < η), η/2 − d (d < 0). 가중치 w 를 곱함
  - **common14 `eta_cost` 와 동일함을 별도 세션에서 확인** (사용자 확인: CPU 30만 점 최대 차 0.0)
- **이번 구현 (`goal1/scripts/g1_meff_gpu.py` warp 커널 `meff_kernel`, 비용 부분):**
  - 형상 k 마다 d_k (1-2d 와 같은 계산), u_k = 로봇 구 중심 → 형상 k 의 분절 COM 단위벡터, m_h = 1/(u_kᵀΛ_h u_k), m_r = 1/(u_kᵀΛ_r u_k), μ_k = 1/(u_kᵀΛ_h u_k + u_kᵀΛ_r u_k) (Λ_r = 0 구는 μ = m_h)
  - η_k = clamp(η0 (μ_k/μref)^p, η_min, η_max) (mass), η_k = η0 (uniform)
  - 구별 비용 = w · max_k c(d_k, η_k) (max_all, 기본) 또는 w · c(d_k*, η_k*) (nearest: 가장 가까운 형상 하나)
  - (b, h) 비용 = Σ_구 구별 비용 → cuRobo 원래 비용에 더함. off 면 0 을 더함 (1-2d 와 같은 값)
  - 모드(off / uniform / mass), 합산(max_all / nearest), [w, η0, μref, p, η_min, η_max] 는 GPU 텐서. `set_cost()` 가 pinned → copy_ 로 바꿈
  - 계산 줄임: d_k ≥ η 상한(mass 는 η_max, uniform 은 η0)인 형상은 질량 계산 생략 (비용 0 이므로 결과 같음)
  - 사람 시간 τ_h 는 1-2d 정의(Σ_{k=1..h} traj_dt_k) 그대로. 기본은 정지
- **프로토타입·cuRobo 환경 충돌 비용과 다른 점 [확인: 코드]:**

| 항목 | cuRobo 환경 충돌 / 프로토타입 TargetCostChannel | 이번 조합 1 |
|---|---|---|
| 한 구에서 장애물(형상) 여러 개 | 합 (`sum_collisions=True`) | **최댓값** (max_all, 사용자 결정. nearest 는 비교용) |
| 구 여러 개 | 합 (`sum_distance=True`, `weight_distance`) | 합 (같음) |
| η | 장애물 전체 하나 (프로토타입은 질량 가정값으로 정한 하나) | 형상·구·시점마다 μ 로 다름 |
| 시간 보간 | `use_sweep: True, sweep_steps: 4` (시점 사이 구 위치 보간) | 없음 (시점별 구 위치만) |
| w | 100000 (`particle_mpc.yml primitive_collision_cfg.weight`, 프로토타입 대상 비용도 100000, alpha 1) | 100000 (기본, 고정) |
| 형상 | 상자(OBB, 프로토타입 손 패치) | 캡슐·구 24개 |

## 2. 검증

**(1) GPU ↔ CPU 기준** (`g1_meff_cpu_check.py` 확장: 모든 형상 η_k·c_k 를 numpy float64 로 따로 계산해 구별 최댓값·선택 형상·η·(b,h) 합 비교)

| 자료 | 설정 | 비용 > 0 | 비용 고른 형상 일치 | η 상대 차 최대 | 비용 상대 차 (가중 ≥ 1) 최대 / p99 | 상대 차 > 1e-4 |
|---|---|---|---|---|---|---|
| 합성 근접 (실제 사람 프레임 형상 표면 −0.10~+0.15 m, 366,000개) | mass / max_all | 356,023 | 100% | 7.0e-6 | 6.5e-5 / 2.5e-6 | 82개, 모두 가중 비용 ≤ 0.66 |
| 같음 | mass / nearest | 352,440 | 100% | 1.1e-5 | 6.5e-5 / 2.9e-6 | 93개, ≤ 0.66 |
| 같음 | uniform / max_all | 355,240 | 100% | 0 | 5.9e-5 / 3.0e-6 | 108개, ≤ 0.37 |
| rollout 덤프 (Isaac 없는 MPC, 4회 × 300쌍 × 구 61 = 73,200) | mass / max_all (η0 0.10, 0.15) | 531 | 100% | 6.1e-7 | 1.24e-4 / 6.9e-5 | 10개 |

- 상대 차가 1e-4 를 넘는 것은 모두 d 가 η 바로 안쪽이라 (η − d)² 가 아주 작은 경우다 (float32 에서 η − d 상쇄). 가중 비용 1 = (η − d) 약 1.4~1.7 mm.
- 1-2d 출력(가장 가까운 형상·거리·u·m_h·m_r·μ)은 같은 덤프에서 그대로 일치 (형상 100%, μ 상대 차 ≤ 1.4e-6).

**(2) off 모드 = 1-2d [확인]:**
- Isaac 없는 시험: 채널 켬·모드 off 600 스텝 명령이 1-2d 의 채널 끔 실행과 비트 동일.
- Isaac g1 (F): `g13_off_near` (모드 off, 근접 기록 켬) 측정 관절값이 `g1d_off_1`(1-2d 기준) 과 같음.

**(3) 파라미터 변경 재캡처 0 [확인]:** Isaac 없는 시험 mass 모드에서 스텝 300 에 η0 0.10 → 0.15. 캡처 1 (예열) 그대로, 입력·채널 텐서 주소 불변, infeasible 0, 도달 4.

## 3. seed 재현성 (ROS 경로) — 불통과 [확인], 멈춤 → 1안 승인, 8절에서 해결

**MPPI 잡음 seed 위치 [확인: 코드]:**
- `curobo/content/configs/task/particle_mpc.yml` `mppi.seed: 0`, `sample_params: {fixed_samples: True, seed: 0, sample_ratio: halton 0.3, halton-knot 0.7}`
- `opt/particle/parallel_mppi.py`: `SampleLib(self.sample_params)` (110행) 로 잡음 집합을 만들고, `fixed_samples=True` 라 실행 중 바뀌지 않음. 평균 샘플용 `HaltonSampleLib` seed 2567 (152행). 측정 스크립트는 `torch.manual_seed(S.SEED=0)`
- 결정성: 채널 off 실행들(g1d_off_1·off_2·on_1·on_2(가중치 0)·prof·noprof·g13_off_near)의 측정 관절값이 모두 같음 → MPPI·Isaac 물리는 같은 입력이면 같은 출력

**uniform η0 0.10 두 번 (`g13_uni10_a`, `g13_uni10_b`, 같은 seed·같은 재생 파일):**
- 측정 관절값이 스텝 281 부터 다름, 1,200 중 919 스텝, 관절 최대 차 0.496 rad. 도달 5 / 5, infeasible 0 / 0, 최소 이격 0.090 / 0.090 m 는 같음.
- 원인 [확인]:
  - 스텝마다 쓰인 사람 데이터(HumanPose 순번, Isaac 쪽 갱신 기록과 루프 시각으로 맞춤)가 두 실행에서 **636 / 1,200 스텝 다름**, 대부분 1 프레임 차이 (예: 스텝 1 에서 a 36번, b 37번).
  - 사람 데이터는 재생(30 Hz, 시스템 시계) → 사본 노드 → ROS → Isaac 수신 순서로 오고, Isaac 루프(시뮬레이션 스텝)와 시계가 따로라 어느 프레임이 어느 스텝에 닿는지가 실행마다 달라진다.
  - 비용이 0 이면 사람 위치가 명령에 영향을 주지 않아 같았고, rollout 에서 비용이 켜지는 시점부터 갈라졌다. 스텝 281 의 실행 경로 최소 거리는 0.214 m (MPC 구 기준) 지만 rollout 은 약 0.68 s 앞까지 보므로 η 안으로 들어가는 후보가 생긴다 [추정: 시점 대응]. 실행 경로가 η0 0.10 안으로 처음 들어간 스텝은 299.
- **B단계(짝 비교) 영향:** 같은 seed 짝이라도 사람 프레임 도착 시점 차로 경로가 갈라진다. 이 차이가 질량 효과와 섞일 수 있다.
- **선택지 (사용자 결정, 실행 안 함):**

| 안 | 내용 | 영향 |
|---|---|---|
| 1 | 결정적 재생 모드: Isaac 스크립트가 재생 자료(사본 노드 핵심으로 미리 계산한 프레임)를 시뮬레이션 시각으로 직접 읽음 (스텝 k → 프레임 ⌊k·dt·30⌋). ROS·노드 경로는 실시간 측정용으로 따로 둠 | 같은 seed → 같은 명령 기대. B단계 비교에 적합. 실시간 경로(노드·ROS 지연) 효과는 빠짐 |
| 2 | 재생을 Isaac 루프에 맞춤: Isaac 이 스텝마다 "다음 프레임 요청"을 보내고 재생·노드가 응답할 때까지 기다림 | ROS 경로 유지, 결정성 기대. 루프가 노드 계산(약 5 ms)만큼 기다림 |
| 3 | 그대로 두고 반복 수로 통계 처리 (짝 비교 대신 분포 비교) | 구현 변경 없음. 실행 수 많이 필요 |

## 4. uniform 동작 (장면 g1, (F) 설정, 근접 쌍 기록 켬 → 시간 판정에 쓰지 않음)

| 실행 | η0 | 도달 | infeasible | 최소 이격 MPC 구 기준 (버퍼 0.02 제외) | 그때 로봇 구 / 사람 형상 | 10 cm 미만 스텝 | 실시간 배율 (기록 켬) |
|---|---|---|---|---|---|---|---|
| g13_off_near | off | 5 | 0 | 0.085 m (0.105) | link6 / 5번 (배) | 12 | 1.075 |
| g13_uni05 | 0.05 | 5 | 0 | 0.085 m (0.105) | link6 / 5번 (배) | 16 | 1.030 |
| g13_uni10_a | 0.10 | 5 | 0 | 0.090 m (0.110) | hand / 23번 (오른손 엄지) | 16 | 1.025 |
| g13_uni10_b | 0.10 | 5 | 0 | 0.090 m (0.110) | hand / 23번 | 12 | 1.026 |
| g13_uni15 | 0.15 | 5 | 0 | 0.126 m (0.146) | link6 / 5번 | 0 | 1.030 |

- 세 η0 모두 목표 도달을 막지 않았다 (목표점은 사람 표면에서 0.150 m). 장면은 바꾸지 않음.
- 최소 이격은 "MPC 로봇 구(yml 반지름 + 버퍼 0.02) 표면 ~ 사람 형상 표면" 거리다. 괄호는 버퍼 0.02 를 뺀 yml 반지름 기준.
- η0 0.05 는 off 와 최소 이격이 같다. 이 재생에서 로봇이 사람 형상 0.085 m 안으로 들어오지 않아 η0 0.05 비용이 실행 경로 근처에서 거의 켜지지 않은 것으로 보인다 [추정].
- 모든 실행: 그래프 재캡처 0 (예열 1), 입력·채널 텐서 주소 불변, 사람 데이터 30 Hz.

## 5. (이전) 하지 않은 것 — 8절 이후에서 모두 함

- μ 분포·분절별 방향 범위 (근접 기록 `g13_off_near_near.npz` 등은 저장됨, 분석 스크립트 `g13_analyze.py mu` 준비됨)
- 파라미터 후보 (μref, p, η_min, η_max) 와 Δη 표 (`g13_analyze.py eta` 준비됨)
- mass 모드 Isaac 짧은 실행
- 성능: 짝 측정 (off 채널 vs mass/max_all, mass/nearest — `--meff-paired-base channel_off` 준비됨), 따로 실행 off / uniform / mass

## 6. 실행 중 발견·정리한 것

- **사용자 GUI 실행(g1d_gui, 17:09) 뒤 남은 것:** 감시(`step4_sys_monitor`)·수신기(`step4_js_sink`)·`vram_monitor.sh` 와 /dev/shm fastrtps 5개. 실행 스크립트 정리 단계 전에 끝난 것으로 보인다. 이번 측정 동안 계속 돌았다 (1 Hz 감시, `/step3/joint_states` 수신). `g1d_gui.stop` 을 만들어 정상 종료시키고 vram 감시는 종료, /dev/shm 정리. ~/hrc_ws 는 g1d_gui 시작 대비 변경 0.
- **내가 남긴 것:** 1-2d 클럭 재측정 첫 시도(15:08)의 `nvidia-smi dmon` 하나가 백그라운드 subshell 의 `kill` 이 dmon 을 못 끄는 바람에 계속 돌았다 (1 Hz GPU 조회, `meff_bench/rep_dmon.txt` 에 기록). 17:5x 에 종료. 그 사이 측정(1-2d 후반, 1-3 실행)에 같이 돌았다.

## 7. 파일

- 코드: `goal1/scripts/g1_meff_gpu.py` (커널·채널 비용, 백업 `logs/g1_meff_gpu.before_1-3.py`), `env_check/scripts/step2b_mpc_isaac.py` (옵션 `--meff-mode/-agg/-w/-eta0/-muref/-p/-eta-min/-eta-max/-switch-step/-switch-eta0/-log-near/-paired-base`, 백업 `logs/step2b_mpc_isaac.before_1-3.py`), `goal1/scripts/g1_meff_bench.py`, `g1_meff_near.py`, `g1_meff_cpu_check.py`, `g13_analyze.py`, `g13_campaign_a.sh`
- 로그: `goal1/logs/meff_bench/c13_*`, `near_com_static_{mass,uniform}_*`, `near_cost_*_check.json`, `goal1/logs/meff_isaac/g13_*` (근접 기록 `*_near.npz`)

## 8. 결정적 재생 경로 (1안, 사용자 승인)

**(1) 사람 프레임 미리 계산** — `goal1/scripts/g13_human_precompute.py` (실행: `g1_run_human.sh py`, CPU 1스레드)
- `effmass_human.human_inertia_core.HumanInertiaCore` 를 설치본 그대로 import (새 구현 없음. 설치본 7개 파일이 src 와 같음 확인). 메시지 채우기는 `human_inertia_node.compute_callback` 과 같은 순서·float32 변환.
- betas = 재생이 보내는 값 (0 × 10, `replay_meta`), 첫 입력에서 `set_betas` 한 번 (노드와 같음).
- 입력 시각 t_k = k/30 (재생 일정). 속도 dt = t_k − t_{k−1} (노드 정의 "입력 stamp 차" 를 재생 일정에 적용 → 1/30). 첫 프레임 속도 0·무효 (노드와 같음).
- 저장 `goal1/data/human_precomputed_1781524848.npz`: input_time, frame, com·vel·rot (19), mass, inv (19×3×3), joint 근위·원위, 형상 24 (종류·분절·p0·p1·반지름), vdt, vvalid. 1,262 프레임, 6.1 s (프레임당 4.8 ms).
- 대조용 `goal1/data/check_vdt_nodrop.npz`: live 기록의 노드 dt 를 그대로 넣어 계산 (계산 코드 동일성 확인용).
- 검증 `g13_precompute_check.py` → `logs/g13_precompute_check.json` (결과는 맨 앞 요약 (1)).

**(2) Isaac `--human-source precomputed`** (`step2b_mpc_isaac.py`, `g1_human_link.PrecomputedHumanLink`)
- 스텝 i 의 시각 t_sim = (i+1)·dt_phys (그 스텝 `world.step` 뒤 측정 상태의 시각). 프레임 = clip(⌊t_sim·30 + 1e-6⌋ + delay_frames, 0, 1261). 스텝 번호에서 계산하므로 실시간 배율과 무관.
- `--human-delay-frames` 기본 0. 지시 식대로 더한다 (음수면 늦은 프레임).
- 새 프레임일 때만 ROS 경로와 같은 `to_isaac` → `MeffInputs.set_human`. age = t_sim − 프레임 입력 시각 (매 스텝 갱신. ROS 경로는 "지금 − 입력 stamp"). 등속 예측·τ_h 정의 그대로 (기본 정지).
- 첫 프레임(t_sim 0 → 0번)은 예열 전에 넣는다. 1,200 스텝은 프레임 0~600 을 쓴다 (끝 넘김 0).
- 메시 표시 없음 (선택 사항, headless). COM 19 표시는 ROS 경로와 같게 켬.
- 실행 스크립트 `g13_pre_run.sh`: 재생·노드·JointState 수신 없음, 나머지 Isaac 인자는 `g1_meff_run.sh` 와 같음.
- 백업: `logs/g1_human_link.before_pre.py`, `logs/step2b_mpc_isaac.before_pre.py`.

**(3) ROS 경로와 사람 시간축 차이 [확인]:** ROS 경로는 재생이 Isaac 예열 중에 시작해 첫 스텝이 프레임 35 근처다. 스텝별 프레임 − precomputed 프레임 = uniform ROS 실행에서 +20~+36 (중앙 +25~+27, 약 0.9 s 앞섬). 짝 측정 실행은 루프가 느려(배율 0.76) +47~+242. 그래서 precomputed 와 ROS 경로의 경로·최소 이격은 같은 장면이라도 다르다.

**(4) ROS 경로 예열 때 사람 텐서 [확인: 코드]:** `inertia` 경로는 예열 10회 동안 `pump()` 를 부르지 않아 사람 입력 텐서가 초기값(형상 끝점 0, 반지름 0)이다. 1-3 ROS 경로 uniform·mass 실행의 예열 비용에 원점의 반지름 0 형상이 들어갔을 수 있다 [미확인: 영향]. 측정 루프에는 해당 없음. precomputed 는 예열 전에 0번 프레임을 넣는다.

## 9. A단계 나머지 결과

**(1) uniform (precomputed vs ROS 경로, 장면 g1, 1,200 스텝, 근접 기록 켬)**

| η0 | precomputed 도달 / infeasible | precomputed 최소 이격 (버퍼 제외) | 그때 로봇 구 / 사람 형상 | 10 cm 미만 스텝 | ROS 경로 도달 / infeasible | ROS 경로 최소 이격 |
|---|---|---|---|---|---|---|
| off | 5 / 0 | 0.086 m (0.106) | hand 51 / 23 오른손 엄지 | 8 | 5 / 0 | 0.085 m |
| 0.05 | 5 / 0 | 0.086 m (0.106) | hand 51 / 23 | 8 | 5 / 0 | 0.085 m |
| 0.10 (a, b) | 5 / 0, 5 / 0 | 0.099 m (0.119) | hand 51 / 5 배 | 2 | 5 / 0, 5 / 0 | 0.090 m |
| 0.15 | 5 / 0 | 0.128 m (0.148) | link6 31 / 5 배 | 0 | 5 / 0 | 0.126 m |

- η0 0.05 precomputed 실행은 off 와 명령 비트 동일 → 실행 rollout 안에서 0.05 m 안으로 들어간 후보가 결과를 바꾸지 않았다 [확인: 명령 동일]. ROS 경로 η0 0.05 는 off 와 달랐다 (사람 시간축이 다름).
- 세 η0 모두 도달을 막지 않음. 장면 변경 없음.

**(2) μ 분포** (실행 경로, 사람 형상 0.3 m 안 쌍, precomputed off `g13p_off` / ROS off `g13_off_near`). μ 단위 kg. 방향 비 = 분절 안 최대/최소

| 분절 | 쌍 수 | μ 최소 | μ 중앙 | μ 최대 | μ 최대/최소 | m_h 중앙 | m_h 최대/최소 (관측 방향) | 가장 가까운 거리 |
|---|---|---|---|---|---|---|---|---|
| 오른손 | 7,958 / 10,514 | 0.540 / 0.541 | 0.702 / 0.661 | 0.888 / 0.922 | 1.65 / 1.71 | 0.78 / 0.73 | 1.80 / 1.82 | 0.086 / 0.098 m |
| 왼손 | 11,240 / 11,270 | 0.485 / 0.482 | 0.564 / 0.561 | 0.736 / 0.668 | 1.52 / 1.38 | 0.64 / 0.64 | 1.60 / 1.26 | 0.180 / 0.185 |
| 오른 아래팔 | 6,057 / 7,745 | 1.297 / 1.311 | 1.871 / 1.846 | 2.495 / 2.552 | 1.92 / 1.95 | 2.63 / 2.62 | 1.38 / 1.41 | 0.138 / 0.132 |
| 왼 아래팔 | 1,249 / 1,612 | 1.430 / 1.444 | 2.026 / 2.040 | 2.287 / 2.264 | 1.60 / 1.57 | 2.76 / 2.74 | 1.53 / 1.43 | 0.204 / 0.210 |
| 오른 상완 | 5,853 / 7,516 | 1.926 / 1.924 | 2.691 / 2.691 | 4.758 / 4.802 | 2.47 / 2.50 | 6.01 / 5.99 | 1.52 / 1.53 | 0.197 / 0.187 |
| 오른 어깨 | 2,253 / 2,602 | 2.147 / 2.039 | 3.376 / 3.418 | 4.632 / 4.595 | 2.16 / 2.25 | 8.83 / 8.78 | 1.48 / 1.42 | 0.206 / 0.201 |
| 가슴 | 8,424 / 9,412 | 2.241 / 2.235 | 4.051 / 4.058 | 8.754 / 8.758 | 3.91 / 3.92 | 22.6 / 22.4 | 1.71 / 1.71 | 0.138 / 0.124 |
| 배 | 13,477 / 14,132 | 2.497 / 2.496 | 4.722 / 4.688 | 12.91 / 12.90 | 5.17 / 5.17 | 33.9 / 32.3 | 1.46 / 1.47 | 0.097 / 0.085 |
| 골반 | 10,689 / 11,563 | 2.619 / 2.604 | 4.994 / 4.967 | 13.32 / 13.35 | 5.08 / 5.13 | 27.3 / 27.0 | 1.51 / 1.57 | 0.129 / 0.123 |
| 허벅지 (오른) | 635 / 771 | 4.578 / 4.432 | 9.433 / 9.333 | 14.70 / 14.65 | 3.21 / 3.31 | 47.5 / 47.5 | 1.78 / 1.89 | 0.221 / 0.218 |

- 머리·왼 상완은 ROS 경로에서만 0.3 m 안 쌍이 2~3개, precomputed 에서는 0개. 왼 어깨·왼 허벅지는 쌍이 적어 표에서 뺌 (json 에 있음).
- μ 가 몸통에서 m_h 보다 훨씬 작다: 로봇 쪽 m_r 이 수 kg 이라 μ = m_h m_r/(m_h + m_r) 가 m_r 쪽으로 눌린다. μ 최대/최소 비는 로봇 구·자세 변화가 섞인 값이다. 사람 쪽만의 방향 비는 m_h 최대/최소 (관측 방향 1.3~1.9).
- 참고 (재생 전 프레임 Λ_h 고유값, 모든 방향): m_h 최대/최소 중앙값 오른손 7.3, 오른 아래팔 6.1, 상완 2.2, 가슴 2.9, 배 1.9. 실행 경로의 u (로봇 구 → 분절 COM) 는 이 범위의 일부만 지난다.
- 파일: `logs/meff_isaac/g13_mu_g13p_off.json`, `g13_mu_g13_off_near.json`.

**(3) 파라미터 후보 Δη 전체** (`g13_cands.json` → `g13_cands_result_g13p_off.json`, `g13_cands_result_g13_off_near.json`). 앞 / 뒤 = precomputed / ROS. 단위 cm

| 쌍 | A | B | C |
|---|---|---|---|
| 손(오른) vs 가슴 | 8.3 / 8.5 | 11.6 / 11.8 | 7.2 / 7.4 |
| 손(오른) vs 배 | 9.4 / 9.6 | 13.4 / 13.5 | 8.2 / 8.3 |
| 아래팔(오른) vs 가슴 | 4.6 / 4.6 | 6.8 / 7.0 | 4.0 / 4.0 |
| 상완(오른) vs 가슴 | 2.6 / 2.6 | 4.1 / 4.1 | 2.3 / 2.3 |
| 머리 vs 가슴 | 쌍 없음 / 1.0 | 쌍 없음 / 1.5 | 쌍 없음 / 0.8 |
| 오른손 안 최대 vs 최소 (p95 vs p5) | 1.5 (1.1) / 1.6 (1.1) | 1.7 (1.2) / 1.8 (1.2) | 0.8 (0.5) / 0.9 (0.5) |
| 오른 아래팔 안 | 3.1 (1.7) / 3.2 (1.7) | 4.3 (2.4) / 4.4 (2.3) | 2.7 (1.5) / 2.8 (1.5) |
| 오른 상완 안 | 5.6 (2.7) / 5.7 (2.6) | 8.6 (4.0) / 8.7 (3.8) | 4.9 (2.3) / 5.0 (2.2) |
| 가슴 안 | 9.4 (5.3) / 9.4 (5.2) | 9.2 (6.4) / 9.2 (6.4) | 5.8 (4.2) / 5.8 (4.2) |
| 배 안 | 8.8 (7.2) / 8.8 (7.2) | 8.3 (5.8) / 8.3 (5.9) | 5.3 (3.8) / 5.3 (3.9) |
| μ p5 / p50 / p95 에서 η (precomputed) | 0.052 / 0.127 / 0.200 m | 0.040 / 0.140 / 0.200 | 0.050 / 0.111 / 0.150 |

- precomputed 와 ROS 근접 기록의 Δη 차는 0.4 cm 이하. 저장해 둔 ROS 기록으로 만든 후보 표가 precomputed 기록에서도 같다 [확인].
- 몸통 안 방향 차가 큰 것은 대부분 μ 가 η_max 에 걸리거나 로봇 쪽 m_r 변화가 섞인 결과다 (위 (2)).

**(4) mass 모드 짧은 실행** (precomputed, 후보 A, max_all, 600 스텝, `g13p_massA600`)

| 실행 (600 스텝까지) | 도달 (스텝) | infeasible | 최소 이격 (버퍼 제외) | 그때 로봇 구 / 사람 형상 | 10 cm 미만 스텝 |
|---|---|---|---|---|---|
| off | 3 (128, 286, 598) | 0 | 0.086 m (0.106) | hand 51 / 23 오른손 엄지 | 5 |
| uniform 0.10 | 2 (128, 286; 3번째 603) | 0 | 0.099 m (0.119) | hand 51 / 23 | 1 |
| uniform 0.15 | 3 (128, 282, 585) | 0 | 0.138 m (0.158) | link6 31 / 5 배 | 0 |
| **mass A** | **2 (128, 281)** | **0** | **0.137 m (0.157)** | rightfinger 59 / 22 오른손 손바닥 | 0 |

- mass A 는 600 스텝 안에서 세 번째 도달이 없었다. 600 스텝 뒤에 도달하는지는 이 실행으로 모름 [미확인].
- 최소 이격 지점이 오른손 형상인데도 0.137 m: 손 μ 는 η ≈ 0.06 m 지만, 같은 로봇 구가 배(η ≈ 0.15) 비용도 받는 경로로 바뀌어 손에서도 멀어진 것으로 보인다 [추정].

## 10. 성능 (지금 경로: 재생 → 노드 → ROS → Isaac, 1-2d 최종과 같은 방식, `g13_campaign_perf.sh`)

| 실행 | 설정 | solver 평균 / p95 (ms) | loop 평균 (ms) | 실시간 배율 | 도달 / infeasible | 재캡처 | GPU 클럭 중앙 (MHz) | 사람 Hz |
|---|---|---|---|---|---|---|---|---|
| g13_pair_maxall | 짝: 1-2d 채널 ↔ mass A max_all | 8.80 / 10.11 (짝 기준 8.00) | 21.98 | 0.755 (두 solver) | 4 / 0 | 0 | 1,935 | 30.00 |
| g13_pair_nearest | 짝: 1-2d 채널 ↔ mass A nearest | 8.29 / 9.57 (짝 기준 8.26) | 21.73 | 0.764 (두 solver) | 4 / 0 | 0 | 1,755 | 30.00 |
| g13_sep_off | off (= 1-2d 채널) | 9.38 / 11.02 | 14.45 | 1.147 | 5 / 0 | 0 | 1,350 | 30.00 |
| g13_sep_uni | uniform 0.10 | 9.97 / 11.68 | 15.05 | 1.100 | 5 / 0 | 0 | 1,545 | 30.03 |
| g13_sep_mass | mass A max_all | 9.81 / 11.30 | 14.87 | 1.114 | 5 / 0 | 0 | 1,560 | 30.00 |

**짝 측정 (같은 측정 상태, 같은 스텝 안에서 번갈아, 1,199 스텝)**

| 비교 | 추가 평균 | 중앙 | p95 | 기준 ≤ 1 ms (평균) |
|---|---|---|---|---|
| mass/max_all − 1-2d 채널 | +0.79 ms | +0.79 | +2.20 | 통과 |
| mass/nearest − 1-2d 채널 | +0.03 ms | −0.02 | +1.53 | (참고) |

- 따로 실행 사이 GPU 클럭이 1,350~1,560 MHz 로 달라서 따로 실행 solver 차(off 대비 +0.4~0.6 ms)는 짝 결과보다 거칠다.
- 짝 실행은 한 스텝에 solver 두 개라 배율 0.76 이고, 그만큼 사람이 스텝당 더 많이 움직여 도달 4 다. 판정에는 따로 실행 배율을 씀.
- 모든 실행: Λ_r 루프 평균 0.44~0.46 ms, 사람 노드 계산 평균 5.5 ms, 노드 CPU 15%, import 그대로, Pinocchio 로드 없음, ~/hrc_ws 변경 0, ~/.ros 새 파일 0, /dev/shm fastrtps 0, warp 캐시 ~/.cache/warp 33 → 33 (변화 없음), 남은 프로세스 0.

## 11. 이번에 발견·정리한 것

- **측정 전 점검 (지시 0):** 시작 때 Isaac·ROS·재생·노드·nvidia-smi 프로세스 0. /dev/shm 에 `sem.carbonite-sharedmemory` (32바이트, 11-15 20:53 생성) 1개. Isaac(Kit) 이 실행마다 만드는 세마포어로, 내 환경 점검 실행(11-15 20:53, `env_check` 호환 점검) 때 처음 생겼다. 지금까지 실행 스크립트는 fastrtps 만 세어서 못 봤다. 지움.
  - 이후 실행에서 다시 생겨서, `g13_pre_run.sh` 와 `g1_meff_run.sh` 끝에 "관련 프로세스 0 일 때만 지움" 을 넣었다.
- **남은 프로세스 셈 고침:** 실행 스크립트의 `pgrep -fc` 가 검사 문자열을 담은 바깥 셸 자신을 세어 1 이 나온 적이 있다 (첫 precomputed 두 실행). 명령 이름이 bash·sh·grep 인 것을 빼고 세는 `left()` 로 바꿈. 이후 모든 실행 앞뒤 0.
- **~/hrc_ws git 상태 (읽기만) [확인]:** `git status` 에 수정 2·삭제 1 이 보인다 (`human_dynamics_node.py` 9-24, `plot_demo_metrics.py` 7-20 등). 모두 이 작업 전 날짜이고, 이번 실행들의 시작·끝 스냅샷 비교는 변경 0. 원본 노드 src 작업 파일(md5 c5af1d089010)은 설치본·HEAD(3788917c7d93)와 다르다. 사본 노드는 실제로 실행되는 설치본(= HEAD) 기준이다. 아무것도 바꾸지 않음.
- 이번 측정 중 다른 무거운 계산 없음 (실행마다 `top` 상위 기록, `g13_campaign_*.txt`).

## 12. 파일 (이번 추가)

- 데이터: `goal1/data/human_precomputed_1781524848.npz`, `goal1/data/check_vdt_nodrop.npz`
- 코드: `goal1/scripts/g13_human_precompute.py`, `g13_precompute_check.py`, `g13_pre_run.sh`, `g13_campaign_p.sh`, `g13_campaign_perf.sh`, `g1_human_link.py` (`PrecomputedHumanLink`), `env_check/scripts/step2b_mpc_isaac.py` (`--human-source precomputed`, `--human-pre`, `--human-delay-frames`), `g13_analyze.py` (p95/p5 쌍), `g1_meff_run.sh` (`left()`, Kit 세마포어 정리)
- 로그: `goal1/logs/g13_precompute_check.json`, `goal1/logs/meff_isaac/g13p_*` (precomputed), `g13_pair_*`, `g13_sep_*` (성능), `g13_mu_*.json`, `g13_cands*.json`, `g13_campaign_p.txt`, `g13_campaign_perf.txt`
- 보고서 직전본: `goal1/logs/step1_3_report.before_precomputed.md`

## 13. B1 — 부위 효과 짝 비교 (precomputed, 장면 g1, 세트 A, 무게중심 방향, max_all)

### (0) 시작 전 확인

- 측정 전 남은 프로세스 0, /dev/shm 0 (20:03).
- **off 배율 확인 [확인]** (여유 시간 산정용, 판정 아님. 같은 시간대 연속 1회씩, `--loop-prof 1`, ROS 쪽 THREAD_SNAP 켬)

| 실행 | 경로 | 채널 | 근접 기록 | 실시간 배율 | solver 평균 (ms) | loop 평균 (ms) | SM 클럭 중앙 (MHz) |
|---|---|---|---|---|---|---|---|
| b1r_ros_chanoff | ROS | 없음 (1-2d 최종 off 와 같음) | 끔 | 1.287 | 8.29 | 12.86 | 1,350 |
| b1r_ros_modeoff | ROS | 켬, 모드 off (A단계 sep_off 와 같음) | 끔 | 1.140 | 9.34 | 14.53 | 1,350 |
| b1r_ros_modeoff_near | ROS | 켬, 모드 off | 0.3 m | 1.053 | 9.53 | 15.71 | 1,350 |
| b1r_pre_modeoff | precomputed | 켬, 모드 off | 끔 | 1.224 | 8.46 | 13.52 | 1,350 |
| b1r_pre_chanoff | precomputed | 없음 | 끔 | 1.385 | 7.62 | 11.95 | 1,350 |

- 결론: A단계 1.147 과 1-2d 1.32 의 차이는 채널 유무 차이다 (1-2d 채널 켬은 1.18 / 1.19). 덤프는 어느 실행에서도 꺼져 있음.

### (1) seed 여러 개

- **방법:** cuRobo `MpcSolverConfig.load_from_robot_config(override_particle_file=...)` (cuRobo 가 제공하는 덮어쓰기 인자). `--mppi-seed N` 이면 `goal1/b1/mppi_override/mppi_seed_N.yml` (`mppi.seed`, `mppi.sample_params.seed`) 을 만들어 넘김. cuRobo 설치 파일 수정 없음. `-1`(기본) 이면 인자 None → 기존과 같음.
- **실제 적용 확인 [확인]:** 실행 로그에 optimizer seed, sample_params.seed, Halton 생성기 seed(halton·knot) 가 모두 N. 잡음이 seed 에 따라 바뀌는 곳은 `HaltonGenerator` 의 버퍼 색인용 `torch.Generator(seed)` 다 (Halton 수열 자체는 scramble=False 라 seed 무관). 평균 샘플용 `HaltonSampleLib` seed 2567 은 코드 상수라 그대로.
- seed 0 덮어쓰기 = 덮어쓰기 없음: `b1_uni10_s0` 첫 1,200 스텝이 A단계 `g13p_uni10_a` 와 비트 동일. seed 1 은 스텝 1 부터 다름.
- **같은 seed 2회:** mass A seed 0, seed 5 각각 2,523 스텝 명령·스텝별 기록 비트 동일.
- 한 Isaac 프로세스 안에서 이어 돌리기는 쓰지 않음 (따로 실행 1개 약 1분, 전체 시간이 감당 가능해서).
- 길이: 2,523 스텝 (마지막 스텝 t_sim 42.05 s → 프레임 1,261, 끝 넘김 0).

### (2) 조건

| 조건 | 설정 |
|---|---|
| off | 채널 켬, 모드 off (명령은 채널 없음과 같음, A단계 확인) |
| uniform 0.10 | η = 0.10 |
| uniform η_matched | η = **0.1272 m** |
| iso (추가) | m_h = ISO 15066 부위별 고정, 나머지 mass A 와 같음 (13.(4)) |
| mass A | η = clamp(0.10 (μ/2.0)^0.5, 0.05, 0.20) |

- **η_matched 계산 [확인]:** mass A seed 0 근접 기록 (실행 상태, 로봇 구–형상 간격 < 0.3 m, 2,128 스텝). (스텝, 로봇 구) 마다 가장 가까운 형상 하나를 활성 형상으로 보고 그 쌍의 μ 로 η_A 를 계산, (스텝, 로봇 구) 58,986개 평균 = 0.12719 m → 0.1272. 중앙 0.142, p5~p95 0.052~0.200, η_max 에 걸린 비율 6.7%. 참고: 비용이 켜진 쌍(d < η)만 평균하면 0.186 (3,867쌍). 부위군별 평균 손 0.054, 아래팔 0.094, 위팔 0.120, 몸통 0.162.
- 실행: 조건 5 × seed 8 = 40 + 같은 seed 반복 2 + 확인용. 모두 precomputed, `--b1-log 1` (스텝별 명령·EE·형상 24개 각각의 로봇 구 전체 최소 간격과 그 쌍의 μ·m_h·m_r), mass A·iso 는 근접 기록 0.3 m 도 켬.
- 간격은 MPC 로봇 구(yml 반지름 + 버퍼 0.02) 표면 ~ 사람 형상 표면. 버퍼 제외 간격은 +2 cm.
- 부위군: 손 = 형상 20~23 (좌우, 엄지 포함), 아래팔 18·19, 위팔 16·17, 몸통 0·1·4·5·10·11 (골반·배·가슴), 머리 15, 다리 2·3·6~9. 목·어깨(12~14) 는 따로 "목·어깨" 로 기록.

### (3) 결과
### 짝 차이 (cm, 같은 seed끼리, n = 8)

| 부위군 | 비교 | 평균 | 표준편차 | 양(+) seed / 8 | 음(−) seed / 8 | 부호 검정 p (양측) |
|---|---|---|---|---|---|---|
| 손 | mass A-uniform_matched | +2.84 | 1.56 | 7 | 1 | 0.070 |
| 손 | mass A-uniform 0.10 | +4.23 | 1.24 | 8 | 0 | 0.008 |
| 손 | mass A-iso | +0.48 | 2.47 | 3 | 5 | 0.727 |
| 손 | iso-uniform_matched | +2.36 | 2.86 | 7 | 1 | 0.070 |
| 아래팔 | mass A-uniform_matched | +4.34 | 1.12 | 8 | 0 | 0.008 |
| 아래팔 | mass A-uniform 0.10 | +6.09 | 1.76 | 8 | 0 | 0.008 |
| 아래팔 | mass A-iso | -0.08 | 1.23 | 2 | 6 | 0.289 |
| 아래팔 | iso-uniform_matched | +4.42 | 1.52 | 8 | 0 | 0.008 |
| 위팔 | mass A-uniform_matched | +3.77 | 0.80 | 8 | 0 | 0.008 |
| 위팔 | mass A-uniform 0.10 | +6.00 | 1.18 | 8 | 0 | 0.008 |
| 위팔 | mass A-iso | -0.10 | 1.28 | 3 | 5 | 0.727 |
| 위팔 | iso-uniform_matched | +3.87 | 1.34 | 8 | 0 | 0.008 |
| 몸통 | mass A-uniform_matched | +3.44 | 2.16 | 8 | 0 | 0.008 |
| 몸통 | mass A-uniform 0.10 | +5.54 | 1.91 | 8 | 0 | 0.008 |
| 몸통 | mass A-iso | -0.53 | 1.33 | 2 | 6 | 0.289 |
| 몸통 | iso-uniform_matched | +3.97 | 1.48 | 8 | 0 | 0.008 |
| 머리 | mass A-uniform_matched | +3.02 | 5.95 | 6 | 2 | 0.289 |
| 머리 | mass A-uniform 0.10 | +3.45 | 5.26 | 6 | 2 | 0.289 |
| 머리 | mass A-iso | -1.38 | 6.00 | 2 | 6 | 0.289 |
| 머리 | iso-uniform_matched | +4.40 | 7.60 | 5 | 3 | 0.727 |
| 다리 | mass A-uniform_matched | +3.11 | 1.54 | 8 | 0 | 0.008 |
| 다리 | mass A-uniform 0.10 | +4.16 | 1.50 | 8 | 0 | 0.008 |
| 다리 | mass A-iso | -0.81 | 1.17 | 1 | 7 | 0.070 |
| 다리 | iso-uniform_matched | +3.92 | 1.57 | 8 | 0 | 0.008 |
| 목·어깨 | mass A-uniform_matched | +3.16 | 3.99 | 6 | 2 | 0.289 |
| 목·어깨 | mass A-uniform 0.10 | +4.93 | 5.17 | 7 | 1 | 0.070 |
| 목·어깨 | mass A-iso | -1.44 | 4.03 | 3 | 5 | 0.727 |
| 목·어깨 | iso-uniform_matched | +4.60 | 4.34 | 6 | 2 | 0.289 |

### seed별 범위 (최소~최대, cm) 와 겹침

| 부위군 | off | uniform 0.10 | uniform η_matched | iso | mass A | mass A vs uniform_matched | mass A vs iso |
|---|---|---|---|---|---|---|---|
| 손 | -1.5~9.2 | 7.9~12.0 | 8.4~12.5 | 8.5~17.4 | 11.2~16.3 | 겹침 | 겹침 |
| 아래팔 | -10.4~1.6 | 4.6~9.9 | 6.5~10.3 | 11.6~15.2 | 12.3~14.9 | 안 겹침 | 겹침 |
| 위팔 | -11.6~-1.8 | 4.1~6.5 | 6.7~8.2 | 9.0~13.1 | 9.8~12.7 | 안 겹침 | 겹침 |
| 몸통 | -15.7~-6.4 | 3.3~7.0 | 5.5~9.3 | 10.9~13.4 | 9.4~12.7 | 안 겹침 | 겹침 |
| 머리 | 14.4~21.7 | 13.0~26.0 | 14.0~27.1 | 16.4~32.0 | 16.8~29.1 | 겹침 | 겹침 |
| 다리 | 7.2~11.3 | 18.5~21.1 | 18.1~22.1 | 23.1~26.4 | 22.0~26.1 | 겹침 | 겹침 |
| 목·어깨 | 0.4~4.4 | 4.9~10.1 | 7.2~13.8 | 10.9~18.6 | 7.9~17.7 | 겹침 | 겹침 |

### 최소 간격 순간 (seed 8개: 중앙값, 가장 많은 로봇 구 링크)

| 부위군 | 조건 | 최소 간격 중앙 (cm) | 시각 중앙 (s) | μ 중앙 (kg) | η 중앙 (m) | 로봇 구 (빈도) |
|---|---|---|---|---|---|---|
| 손 | off | 6.4 | 6.1 | 0.58 | — | hand 4, rightfinger 4 |
| 손 | uniform 0.10 | 8.6 | 6.2 | 0.57 | 0.100 | hand 6, leftfinger 2 |
| 손 | uniform η_matched | 11.2 | 22.8 | 0.62 | 0.127 | hand 4, link6 3 |
| 손 | iso | 13.8 | 17.7 | 0.74 | 0.052 | link7 3, rightfinger 3 |
| 손 | mass A | 13.9 | 6.3 | 0.61 | 0.055 | rightfinger 5, link7 2 |
| 아래팔 | off | -1.3 | 37.2 | 4.54 | — | link6 5, link7 2 |
| 아래팔 | uniform 0.10 | 7.8 | 38.8 | 2.37 | 0.100 | hand 4, link7 3 |
| 아래팔 | uniform η_matched | 10.1 | 38.8 | 2.15 | 0.127 | link7 4, hand 3 |
| 아래팔 | iso | 14.6 | 38.4 | 2.21 | 0.093 | link7 5, link6 2 |
| 아래팔 | mass A | 14.3 | 38.4 | 2.17 | 0.104 | link7 5, hand 2 |
| 위팔 | off | -5.2 | 37.2 | 3.65 | — | link6 6, link7 2 |
| 위팔 | uniform 0.10 | 5.6 | 38.6 | 3.28 | 0.100 | link7 5, link6 3 |
| 위팔 | uniform η_matched | 7.9 | 38.8 | 3.23 | 0.127 | link7 6, link6 2 |
| 위팔 | iso | 11.9 | 38.4 | 3.47 | 0.105 | link6 5, link7 2 |
| 위팔 | mass A | 11.8 | 38.4 | 3.49 | 0.132 | link6 4, link7 2 |
| 몸통 | off | -10.5 | 37.1 | 11.74 | — | link6 8 |
| 몸통 | uniform 0.10 | 6.2 | 33.5 | 5.95 | 0.100 | link6 4, link7 3 |
| 몸통 | uniform η_matched | 8.0 | 37.1 | 5.63 | 0.127 | link7 5, link6 3 |
| 몸통 | iso | 12.0 | 29.4 | 6.10 | 0.177 | link6 5, hand 2 |
| 몸통 | mass A | 11.8 | 25.6 | 6.08 | 0.174 | link6 5, hand 3 |
| 머리 | off | 17.3 | 25.8 | 3.24 | — | link6 8 |
| 머리 | uniform 0.10 | 17.4 | 38.3 | 4.26 | 0.100 | link6 6, link5 2 |
| 머리 | uniform η_matched | 18.8 | 33.1 | 3.35 | 0.127 | link6 6, hand 1 |
| 머리 | iso | 23.7 | 25.1 | 3.38 | 0.117 | link6 7, link4 1 |
| 머리 | mass A | 21.7 | 27.4 | 3.12 | 0.125 | link6 7, link7 1 |
| 다리 | off | 8.7 | 37.1 | 9.33 | — | hand 8 |
| 다리 | uniform 0.10 | 19.3 | 35.7 | 13.55 | 0.100 | hand 7, leftfinger 1 |
| 다리 | uniform η_matched | 20.6 | 33.4 | 13.77 | 0.127 | hand 6, leftfinger 2 |
| 다리 | iso | 24.1 | 26.9 | 10.90 | 0.200 | hand 7, leftfinger 1 |
| 다리 | mass A | 23.0 | 30.1 | 12.30 | 0.200 | hand 8 |

### 과제 지표 (조건별 seed 8개 평균, 괄호 = off 대비)

| 조건 | 도달 | EE 경로 (m) | 명령 변화 평균 / p95 (rad) | infeasible | 겹침 스텝 | solver 평균 / p95 (ms) |
|---|---|---|---|---|---|---|
| off | 11.25 | 8.54 | 0.0117 / 0.0353 | 0.0 | 66.0 | 8.80 / 9.82 |
| uniform 0.10 | 10.38 (-0.88) | 8.05 (-0.49) | 0.0114 / 0.0346 | 0.0 (+0.0) | 0.0 (-66.0) | 9.55 / 10.61 |
| uniform η_matched | 9.00 (-2.25) | 7.80 (-0.74) | 0.0113 / 0.0342 | 0.0 (+0.0) | 0.0 (-66.0) | 9.53 / 10.47 |
| iso | 7.62 (-3.62) | 6.83 (-1.71) | 0.0101 / 0.0332 | 146.0 (+146.0) | 0.0 (-66.0) | 10.53 / 14.93 |
| mass A | 7.50 (-3.75) | 7.11 (-1.44) | 0.0104 / 0.0335 | 119.8 (+119.8) | 0.0 (-66.0) | 10.29 / 14.19 |

### 도달 시각 (목표 순서별 평균 s, 그 순번에 도달한 seed만)

| 조건 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| off | 2.3 (8) | 5.6 (8) | 10.1 (8) | 13.1 (8) | 17.5 (8) | 20.5 (8) | 24.8 (8) | 27.8 (8) |
| uniform 0.10 | 2.3 (8) | 5.6 (8) | 10.2 (8) | 13.2 (8) | 17.6 (8) | 20.7 (8) | 25.2 (8) | 28.1 (8) |
| uniform η_matched | 2.3 (8) | 5.6 (8) | 10.1 (8) | 13.2 (8) | 18.0 (8) | 21.1 (8) | 25.7 (8) | 28.9 (8) |
| iso | 2.3 (8) | 5.7 (8) | 10.6 (8) | 13.6 (8) | 18.5 (8) | 21.7 (8) | 31.5 (8) | 30.5 (4) |
| mass A | 2.3 (8) | 5.5 (8) | 10.7 (8) | 13.8 (8) | 18.9 (8) | 22.1 (8) | 32.0 (8) | 37.3 (4) |

### seed별 infeasible·도달·겹침

| 조건 | s0 | s1 | s2 | s3 | s4 | s5 | s6 | s7 |
|---|---|---|---|---|---|---|---|---|
| off | 0 / 11 / 46 | 0 / 11 / 64 | 0 / 11 / 54 | 0 / 10 / 45 | 0 / 11 / 65 | 0 / 13 / 95 | 0 / 10 / 50 | 0 / 13 / 109 |
| uniform 0.10 | 0 / 10 / 0 | 0 / 10 / 0 | 0 / 10 / 0 | 0 / 10 / 0 | 0 / 11 / 0 | 0 / 11 / 0 | 0 / 9 / 0 | 0 / 12 / 0 |
| uniform η_matched | 0 / 9 / 0 | 0 / 9 / 0 | 0 / 9 / 0 | 0 / 8 / 0 | 0 / 9 / 0 | 0 / 9 / 0 | 0 / 9 / 0 | 0 / 10 / 0 |
| iso | 0 / 8 / 0 | 0 / 9 / 0 | 117 / 7 / 0 | 0 / 7 / 0 | 411 / 8 / 0 | 192 / 7 / 0 | 0 / 7 / 0 | 448 / 8 / 0 |
| mass A | 234 / 7 / 0 | 0 / 8 / 0 | 0 / 8 / 0 | 0 / 7 / 0 | 0 / 8 / 0 | 186 / 7 / 0 | 0 / 7 / 0 | 538 / 8 / 0 |

### 해석: 최소 간격 순간 μ_mass vs μ_iso (같은 분절·같은 로봇 구 m_r, seed 중앙)

| 부위군 | 분절 | m_h 방향 (mass A 실행) | m_h_iso | μ_mass | μ_iso | 방향 m_h 가 ISO 보다 |
|---|---|---|---|---|---|---|
| 손 | [17, 18] | 0.69 | 0.6 | 0.61 | 0.54 | 큼 |
| 아래팔 | [15] | 2.82 | 2.0 | 2.17 | 1.72 | 큼 |
| 위팔 | [13] | 6.03 | 3.0 | 3.49 | 2.10 | 큼 |
| 몸통 | [0, 3, 8] | 31.98 | 40.0 | 6.08 | 6.37 | 작음 |
| 머리 | [12] | 6.24 | 4.4 | 3.12 | 2.60 | 큼 |
| 다리 | [1] | 32.81 | 75.0 | 12.30 | 16.10 | 작음 |
| 목·어깨 | [10] | 8.45 | 40.0 | 4.02 | 6.40 | 작음 |

### 해석: 분절별 방향 m_h vs ISO (mass A 8 seed 근접 기록 0.3 m)

| 분절 | 쌍 수 | m_h 중앙 (p5~p95) | m_h_iso | m_h > ISO 비율 | μ_mass / μ_iso 중앙 | η_mass / η_iso 중앙 (m) |
|---|---|---|---|---|---|---|
| pelvis | 323,441 | 26.02 (23.84~32.35) | 40.0 | 0% | 4.99 / 5.32 | 0.158 / 0.163 |
| left_thigh | 20,312 | 22.34 (19.69~43.05) | 75.0 | 0% | 9.38 / 13.14 | 0.200 / 0.200 |
| right_thigh | 1,301 | 44.96 (36.65~51.98) | 75.0 | 0% | 11.01 / 12.18 | 0.200 / 0.200 |
| abdomen | 407,399 | 31.93 (29.34~40.47) | 40.0 | 6% | 4.53 / 4.64 | 0.151 / 0.152 |
| chest | 329,885 | 21.87 (20.31~24.77) | 40.0 | 0% | 3.98 / 4.31 | 0.141 / 0.147 |
| neck | 2,126 | 10.60 (9.57~11.98) | 1.2 | 100% | 3.80 / 0.99 | 0.138 / 0.070 |
| left_shoulder | 104,187 | 9.17 (7.31~11.59) | 40.0 | 0% | 4.50 / 7.55 | 0.150 / 0.194 |
| right_shoulder | 25,132 | 8.53 (7.16~9.19) | 40.0 | 0% | 3.40 / 4.96 | 0.130 / 0.157 |
| head | 6,730 | 6.18 (5.75~9.50) | 4.4 | 100% | 3.24 / 2.60 | 0.127 / 0.114 |
| left_upper_arm | 197,198 | 7.66 (5.96~8.78) | 3.0 | 100% | 2.76 / 1.78 | 0.117 / 0.094 |
| right_upper_arm | 64,703 | 5.99 (5.53~7.00) | 3.0 | 100% | 2.70 / 1.86 | 0.116 / 0.096 |
| left_lower_arm | 170,920 | 2.71 (2.42~3.38) | 2.0 | 100% | 1.85 / 1.49 | 0.096 / 0.086 |
| right_lower_arm | 69,409 | 2.62 (2.35~2.81) | 2.0 | 100% | 1.84 / 1.53 | 0.096 / 0.087 |
| left_hand | 157,818 | 0.67 (0.61~1.49) | 0.6 | 98% | 0.60 / 0.55 | 0.055 / 0.052 |
| right_hand | 96,007 | 0.72 (0.60~0.92) | 0.6 | 96% | 0.66 / 0.55 | 0.057 / 0.053 |


- "최소 간격 순간" 표의 μ 는 기록된 방향 μ (mass 정의). iso 조건의 η 는 같은 쌍의 μ_iso 로 계산한 값.
- 그림 1: `goal1/b1/b1_fig1_group_min_gap.png` (부위군별 최소 간격, 조건별 점, 같은 seed 선). 그림 2: `goal1/b1/b1_fig2_massA_s0_gap_eta.png` (mass A seed 0, 최근접 간격과 그 쌍의 활성 η, 아래는 간격 − η 를 부위군 색으로).

### (4) iso 모드 (사용자 추가 지시)

- **ISO 값 출처:** `~/indy_ws/src/indy_control_cpp/ISO15066/iso_15066.json` (읽기만) → `goal1/data/iso_15066.json` 복사 (md5 48370e24…, 원본·indy_ws HEAD 와 같음). indy_ws HEAD `cd6ce6c34956797386f0a0f292fc1c2a9e506ee3` (2026-09-22), 이 파일 마지막 커밋 `f8fe8a6fb886…` (2026-06-19).
- **분절 대응:** `collision_assessment_node.cpp` `initHumanBodyIndexMap()` (작업 사본은 HEAD 와 내용이 다르지만 이 함수의 대응은 HEAD 와 같음 [확인]) 과 effmass_human `SEGMENT_NAMES` 를 번호별로 대조: **19개 모두 같음**.

| 번호 | indy_ws 부위 | effmass_human 분절 | m_h_iso (kg) |
|---|---|---|---|
| 0 | pelvis | pelvis | 40 |
| 1 / 2 | left_thigh / right_thigh | 같음 | 75 / 75 |
| 3 | abdomen | abdomen | 40 |
| 4 / 5 | left_calf / right_calf | 같음 | 75 / 75 |
| 6 / 7 | left_foot / right_foot | 같음 | **75 / 75 (가정: JSON 에 발 없음 → 종아리 값)** |
| 8 | chest | chest | 40 |
| 9 | neck | neck | 1.2 |
| 10 / 11 | left_shoulder / right_shoulder | 같음 | 40 / 40 |
| 12 | head | head | 4.4 |
| 13 / 14 | left_upper_arm / right_upper_arm | 같음 | 3.0 / 3.0 |
| 15 / 16 | left_lower_arm / right_lower_arm | 같음 | 2.0 / 2.0 |
| 17 / 18 | left_hand / right_hand | 같음 | 0.6 / 0.6 |

  JSON 의 `face` (4.4) 는 대응 분절이 없어 안 씀. 형상 24개는 속한 분절 값 (엄지 → 손).
- **구현:** 커널 모드 3. `ahk = 1/m_h_iso[분절]` (방향 무관), 로봇 쪽 m_r(u), μ, η 매핑(세트 A), max_all 은 mass 와 같은 코드. `inv_iso` 는 GPU 텐서 (19,), 모드는 기존 GPU 텐서 값만 바꿈. 백업 `logs/*.before_iso.py`.
- **검증 [확인]:**
  - 합성 근접 366,000개 (iso/max_all): 비용 형상 일치 100%, η 상대 차 ≤ 4.3e-6, 가중 비용 ≥ 1 상대 차 ≤ 5.7e-5 (mass 와 같은 수준). `logs/meff_bench/near_cost_iso_max_all_check.json`
  - Isaac rollout 덤프 (iso, 6 스텝 × 300쌍 × 61 = 109,800): 형상 100%, 비용 > 0 693개, η ≤ 3.4e-7, 가중 비용 ≥ 1 상대 차 최대 1.6e-4 (η − d 수 mm 의 float32 상쇄, A단계 1.24e-4 와 같은 종류), (b,h) 합 ≤ 2.9e-4. `logs/meff_isaac/b1iso_dump_check.json`
  - 패치 뒤 mass·uniform 합성 출력이 패치 전 덤프와 모든 항목 비트 동일 (같은 입력 seed 1).
  - 패치 뒤 off seed 0·mass A seed 0 (2,523 스텝) 명령·스텝별 기록이 이전 실행과 비트 동일. 재캡처 0 (예열 1).
  - 첫 시도에서 커널 줄 끝 한글 주석이 warp 생성 CUDA 소스에 들어가 ROS 실행 환경(ASCII)에서 컴파일 오류로 3개 실행이 시작 직후 죽었다. 주석을 영문으로 바꾸고 다시 돌림 (결과 무관).

### (5) 해석 [추정 표시]

- **손 가설이 반대로 나온 이유 [추정]:** max_all 이라 한 로봇 구가 손(η ≈ 0.05)과 몸통(η ≈ 0.15~0.20)을 함께 보면 큰 η 쪽 비용이 그 구를 밀어낸다. 손 최소 간격 순간에도 로봇 구가 몸통 가까이 있으면 손 간격이 같이 커진다. 그림 2 에서 손이 최근접일 때 간격 − η 는 +10~20 cm 로 손 비용은 거의 꺼져 있다. uniform_matched 의 0.127 은 "최근접 쌍 평균" 이라 몸통 근처에서 mass A 가 실제로 쓰는 η (0.16~0.20) 보다 작다. 그래서 mass A 가 전체적으로 더 멀다.
- **mass A ≈ iso [확인: 수치]:** μ = m_h m_r/(m_h + m_r) 에서 로봇 쪽 m_r 이 수 kg 이라, 몸통처럼 m_h 가 큰 곳은 m_h 가 32 kg 이든 40 kg 이든 μ 가 거의 같다 (μ_mass 6.08 vs μ_iso 6.37). 손·팔은 방향 m_h 가 ISO 보다 크지만 (손 0.69 vs 0.6, 아래팔 2.8 vs 2.0, 위팔 6.0 vs 3.0) η 차는 손 0.4 cm, 아래팔 1 cm, 위팔 2 cm 정도다 (분절별 표). 차이가 큰 곳은 목 (방향 10.6 kg vs ISO 1.2 kg → η 0.138 vs 0.070) 과 어깨 (방향 8.5~9.2 kg vs ISO 40 kg → η 0.13~0.15 vs 0.16~0.19) 인데, 이 장면에서 최소 간격이 거기서 생기지 않는다.
- **개인 방향 질량이 ISO 보다 큰 곳:** 목(100%), 머리(100%), 위팔(100%), 아래팔(≈100%), 손(96~98%). **작은 곳:** 골반·가슴(100%), 배(93%), 허벅지(100%), 어깨(100%).
- **infeasible:** mass A 3/8, iso 4/8 seed 에서 관절 4 가 하한 −3.042 rad 에 닿은 뒤 끝까지 hold (최초 mass A seed 0 스텝 2289 에서 여유 0.016 rad). 팔이 몸통 큰 η 를 피해 접히다가 한계에 걸린 것으로 보인다 [추정]. uniform 은 0. 설정은 바꾸지 않음.
- **off 겹침:** 재생 후반(프레임 600 이후, A단계는 0~600만 씀)에 사람이 로봇 쪽으로 들어와 off 는 모든 seed 에서 겹친다 (몸통 −6~−16 cm, 시각 약 37 s). 비용이 있는 조건은 모두 겹침 0.
- 노트 17 기준: mass A vs uniform_matched 는 아래팔·위팔·몸통에서 seed 범위가 안 겹침 (손·머리·다리는 겹침). mass A vs iso 는 모든 부위군에서 겹침.

### (6) GUI 명령 (실행 안 함)

같은 표시 옵션으로 headless 200 스텝 시험 2개: 오류 없음, 재캡처 0, 명령이 B1 seed 0 실행 첫 200 스텝과 비트 동일. 재생 끝(2,523 스텝) 뒤 스스로 종료 (`--hold` 기본 0, `--hold -1` 안 씀). 선 = 로봇 구 → 가장 가까운 형상 상위 5쌍(빨강), 형상 = 사람 캡슐 반투명.

```bash
cd ~/effmass_mpc/goal1
# mass A, seed 0
RUN_ISAAC_NO_DISPLAY=0 scripts/g13_pre_run.sh b1gui_massA_s0 --meff on --meff-mode mass --meff-eta0 0.10 --meff-muref 2.0 --meff-p 0.5 \
  --meff-eta-min 0.05 --meff-eta-max 0.20 --meff-agg max_all --mppi-seed 0 --steps 2523 --meff-viz both --meff-lines topk --meff-lines-k 5 --gui
# uniform η_matched, seed 0
RUN_ISAAC_NO_DISPLAY=0 scripts/g13_pre_run.sh b1gui_unim_s0 --meff on --meff-mode uniform --meff-eta0 0.1272 --meff-agg max_all \
  --mppi-seed 0 --steps 2523 --meff-viz both --meff-lines topk --meff-lines-k 5 --gui
```

- η 껍질 표시는 없다 (기존 선·형상 표시만). GUI 에서 렌더링이 결과를 바꾸는지는 확인 안 함 [미확인].
- 실행 스크립트는 이제 측정 전 남은 프로세스나 /dev/shm 항목이 있으면 시작하지 않는다 (아래).

### (7) 이번에 발견·정리한 것

- **/dev/shm fastrtps 6개:** 위 한글 주석 오류로 죽은 Isaac 3개(20:50~20:51)가 남김. 실행 스크립트가 "측정 전 /dev/shm 6" 을 기록만 하고 멈추지 않아 **iso 8개 실행과 패치 뒤 확인 실행이 이 항목이 남은 채로 돌았다.** 결과 영향 없음 [확인: 같은 상태에서 돈 off·mass A 확인 실행이 이전과 비트 동일]. 쓰는 프로세스 없음 확인 후 지움. `g13_pre_run.sh`·`g1_meff_run.sh` 에 "남은 프로세스 또는 /dev/shm(ad_ 제외) 이 0 이 아니면 종료" 를 넣음.
- 합성 근접 시험의 이전 덤프는 입력 seed 1 로 만든 것이었다 (보고서에 seed 가 안 적혀 있었음). seed 0 으로 처음 돌려 "다름" 이 나왔고, 입력 프레임이 달라서였다. seed 1 로 다시 돌려 비트 동일 확인.
- 보호 폴더: ~/hrc_ws 변경 0 (모든 실행 시작·끝 스냅샷), ~/indy_ws 는 읽기·`git show`·`git log` 만, /opt/ros·~/.local·~/.ros 변경 없음.

### (8) 파일

- 코드: `env_check/scripts/step2_scene.py` (`MPPI_SEED`, `mppi_override_file`, 백업 `logs/step2_scene.before_b1.py`), `step2b_mpc_isaac.py` (`--mppi-seed`, `--b1-log`, `--meff-mode iso`), `goal1/scripts/g1_meff_gpu.py` (iso 모드), `g1_meff_near.py`, `g1_meff_cpu_check.py`, `g1_meff_bench.py`, `b1_analyze.py`, `b1_campaign.sh`, `b1_campaign_rtf.sh`, `g13_pre_run.sh`·`g1_meff_run.sh` (시작 전 점검), 패치 스크립트 `goal1/b1/apply_b1_patch.py`, `apply_iso_patch.py`
- 데이터: `goal1/data/iso_15066.json`, `goal1/b1/mppi_override/`
- 로그: `goal1/logs/meff_isaac/b1_*` (실행별 `_b1.npz`, mass A·iso `_near.npz`), `b1_summary.json`, `b1_eta_matched.json`, `b1_campaign.txt`, `b1_campaign_rtf.txt`, `b1iso_dump_check.json`, `logs/meff_bench/near_cost_iso_max_all_check.json`
- 그림: `goal1/b1/b1_fig1_group_min_gap.png`, `b1_fig2_massA_s0_gap_eta.png`

## 14. B1 후속 분석 (새 Isaac 실행 없음, 저장 기록만)

- 기록: 실행별 `_b1.npz` (스텝마다 형상 24개 각각의 로봇 구 전체 최소 간격·그 구·μ·m_h·m_r), mass A·iso 의 `_near.npz` (0.3 m 안 모든 (구, 형상) 쌍). off·uniform 은 근접 기록이 없어 '같은 로봇 구의 다른 형상' 이 필요한 항목은 **기록 없음**. 모든 값은 실행된 상태 기준이다 (rollout 비용은 기록 없음).
- 스크립트 `goal1/scripts/b1_followup.py` → `logs/meff_isaac/b1_followup.json`.

### (1) 선택성 (전체 보수성과 분리)

C = 몸통 − 손, C2 = 몸통 − 아래팔, C3 = 몸통 − 위팔 (최소 간격, cm). 클수록 몸통을 손·팔보다 더 멀리한다.

| 조건 | C 평균 | C2 평균 | C3 평균 |
|---|---|---|---|
| off | -16.01 | -7.27 | -4.46 |
| uniform 0.10 | -3.39 | -1.86 | +0.42 |
| uniform η_matched | -2.68 | -1.52 | +0.29 |
| iso | -1.07 | -1.97 | +0.39 |
| mass A | -2.08 | -2.42 | -0.04 |

| 짝 차이 | 지표 | 평균 (cm) | 표준편차 | + seed / 8 | − seed / 8 | 부호 검정 p |
|---|---|---|---|---|---|---|
| mass A-uniform_matched | C | +0.60 | 2.99 | 4 | 4 | 1.000 |
| mass A-uniform_matched | C2 | -0.90 | 2.94 | 5 | 3 | 0.727 |
| mass A-uniform_matched | C3 | -0.33 | 2.11 | 4 | 4 | 1.000 |
| mass A-uniform 0.10 | C | +1.31 | 2.05 | 6 | 2 | 0.289 |
| mass A-uniform 0.10 | C2 | -0.56 | 2.54 | 3 | 5 | 0.727 |
| mass A-uniform 0.10 | C3 | -0.46 | 1.66 | 2 | 6 | 0.289 |
| mass A-iso | C | -1.01 | 2.90 | 5 | 3 | 0.727 |
| mass A-iso | C2 | -0.45 | 0.90 | 3 | 5 | 0.727 |
| mass A-iso | C3 | -0.43 | 0.71 | 2 | 6 | 0.289 |
| iso-uniform_matched | C | +1.61 | 3.07 | 6 | 2 | 0.289 |
| iso-uniform_matched | C2 | -0.45 | 2.63 | 4 | 4 | 1.000 |
| iso-uniform_matched | C3 | +0.10 | 2.11 | 5 | 3 | 0.727 |

- 어느 짝에서도 C·C2·C3 의 차가 seed 흩어짐(표준편차 0.7~3.1 cm)을 넘지 않는다. 13절의 "mass A 가 몸통에서 더 멀다" 는 손·팔도 같이 멀어진 결과이고, 몸통만 골라 더 멀리하는 선택성은 보이지 않는다 [확인: 수치].

### (2) 손이 최근접일 때 무엇이 비용을 정했나

손 형상(20~23)이 전체 최근접 형상인 스텝에서, 그 손에 가장 가까운 로봇 구의 0.3 m 안 형상 모두에 대해 c(d, η) 를 다시 계산해 max_all 비용을 낸 형상의 부위군을 셈.

| 조건 | 손 최근접 스텝 / 전체 | 비용을 낸 부위군 | 손 간격 − η (cm, p5 / 중앙 / p95) | 같은 구의 몸통 간격 − η (cm) | 몸통이 그 구 0.3 m 밖 |
|---|---|---|---|---|---|
| off | 5705 / 20184 | 기록 없음 (비용 없음) | — | 기록 없음 | 기록 없음 |
| uniform 0.10 | 5857 / 20184 | 비용 없음 99.6%, 손 0.4% | 5.9 / 13.8 / 15.6 | 기록 없음 | 기록 없음 |
| uniform η_matched | 5857 / 20184 | 비용 없음 98.7%, 손 1.3% | 3.0 / 11.3 / 13.1 | 기록 없음 | 기록 없음 |
| iso | 5906 / 20184 | 비용 없음 100.0% | 11.3 / 18.9 / 20.5 | 5.3 / 10.1 / 13.3 (n 1394) | 4474 / 5868 |
| mass A | 6048 / 20184 | 비용 없음 100.0% | 10.7 / 18.8 / 20.2 | 5.1 / 10.1 / 13.5 (n 1375) | 4538 / 5913 |

- uniform 은 η 가 같아 최근접(손)이 비용을 낼 유일한 후보다. 그 경우에도 비용이 켜진 스텝은 0.4% / 1.3% 뿐이다.
- mass A·iso 는 손 최근접 순간 그 로봇 구의 비용이 모두 0 이다. 실행 상태는 모든 η 껍질 밖에 있고, 비용은 rollout 앞쪽 시점에서만 작동한다. rollout 안에서 어느 형상이 비용을 냈는지는 **기록 없음** (덤프는 B1 실행에서 끔).

**사람 자세 자체: 손 캡슐 ↔ 가장 가까운 몸통 캡슐 표면 간격** (재생 1,262 프레임, 형상 기준, 음수 = 겹침)

| 손 | 최소 | p5 | 중앙 | p95 | ≥ 0.20 m (η_max) | ≥ 0.10 m | < 0.05 m |
|---|---|---|---|---|---|---|---|
| 왼손 | -19.5 cm | -15.4 | -8.4 | 9.0 | 0.0% | 2.1% | 54.4% |
| 오른손 | -17.0 cm | -14.9 | -9.2 | 11.8 | 1.3% | 5.1% | 94.8% |
| 가까운 쪽 손 | -19.5 cm | -15.6 | -11.5 | 4.4 | 0.0% | 2.1% | 95.1% |

- 이 재생에서 손 형상은 대부분 몸통 형상과 겹쳐 있다 (손을 몸 앞에 붙인 자세로 보인다 [추정: 자세 해석]). 손 근처 로봇 구는 거의 항상 몸통 η 껍질(0.15~0.20 m) 안쪽 거리에도 있으므로, max_all 에서는 몸통 비용이 손 근처의 회피를 정한다 [추정]. 손만 따로 가까이 가게 할 공간이 이 재생에는 거의 없다.

### (3) 포화 (0.3 m 안 쌍, seed 8개 합)

| 부위군 | 쌍 수 (mass A) | η_max 비율 mass A / iso | η_min 비율 mass A / iso | μ/m_h 중앙 (p5~p95) mass A | μ/m_r 중앙 (p5~p95) mass A | μ/m_r > 0.5 (로봇 쪽이 정함) mass A / iso |
|---|---|---|---|---|---|---|
| 손 | 253,825 | 0.0% / 0.0% | 0.3% / 3.0% | 0.89 (0.83~0.95) | 0.11 (0.05~0.17) | 0% / 0% |
| 아래팔 | 240,329 | 0.0% / 0.0% | 0.0% / 0.0% | 0.70 (0.56~0.84) | 0.30 (0.16~0.44) | 0% / 0% |
| 위팔 | 261,901 | 0.0% / 0.0% | 0.0% / 0.0% | 0.40 (0.27~0.67) | 0.60 (0.33~0.73) | 69% / 6% |
| 몸통 | 1,060,725 | 10.9% / 13.9% | 0.0% / 0.0% | 0.17 (0.10~0.36) | 0.83 (0.64~0.90) | 100% / 100% |
| 머리 | 6,730 | 0.0% / 0.0% | 0.0% / 0.0% | 0.50 (0.33~0.84) | 0.50 (0.16~0.67) | 51% / 7% |
| 다리 | 21,613 | 77.4% / 91.1% | 0.0% / 0.0% | 0.41 (0.17~0.48) | 0.59 (0.52~0.83) | 97% / 100% |
| 목·어깨 | 131,445 | 0.8% / 35.2% | 0.0% / 0.0% | 0.47 (0.29~0.70) | 0.53 (0.30~0.71) | 57% / 98% |
| 전체 | 1,976,568 | 6.7% / 10.4% | 0.04% / 0.38% | | | |

- iso 의 μ/m_h 는 m_h = m_h_iso 로 계산. η 는 실행 상태 쌍의 μ 로 계산 (커널과 같은 식).
- 몸통·다리 쌍은 μ 가 로봇 쪽 m_r 에 묶인다 (μ ≈ 0.8 m_r). 사람 쪽 질량 차(방향 m_h 대 ISO 40 kg)가 η 에 거의 안 닿는 이유다. 손·아래팔은 사람 쪽이 정한다.

**로봇 링크별** (스텝 중 그 링크의 구가 하나라도 사람 형상 0.3 m 안에 있었던 비율, 그 쌍들의 m_r 중앙)

| 링크 | mass A 0.3 m 안 스텝 | iso | m_r 중앙 mass A (kg) |
|---|---|---|---|
| link2 | 0.0% | 0.1% | 86.5 |
| link3 | 2.3% | 1.0% | 20.8 |
| link4 | 22.5% | 21.2% | 18.0 |
| link5 | 58.0% | 58.5% | 13.4 |
| link6 | 86.4% | 85.9% | 8.2 |
| link7 | 86.0% | 85.5% | 5.8 |
| hand | 86.4% | 85.9% | 4.9 |
| leftfinger | 66.8% | 66.0% | 3.7 |
| rightfinger | 43.9% | 43.8% | 3.8 |

- link0·link1 은 0.3 m 안에 들어온 적 없음. m_r 이 큰 link3 (21 kg)·link4 (18 kg) 는 2%·22% 스텝, link5 (13 kg) 58%. 사람 가까이 가는 것은 주로 m_r 4~8 kg 인 link6·link7·손(86%).
- off·uniform 의 링크별 비율은 **기록 없음** (근접 기록 없음).

### (4) infeasible·겹침

| 조건 | seed | 시작 스텝 | 사람 프레임 | infeasible 스텝 (구간 수) | 걸린 관절 / 한계 | 그때 여유 (rad) | 최근접 부위 / 간격 | 같은 seed off 겹침 (±5 프레임) |
|---|---|---|---|---|---|---|---|---|
| mass A | 0 | 2289 | 1145 | 234 (1) | 관절 4 / -3.042 | 0.0161 | abdomen / 16.2 cm | 아님 |
| mass A | 5 | 1850 | 925 | 186 (13) | 관절 4 / -3.042 | -0.0001 | abdomen / 18.3 cm | 아님 |
| mass A | 7 | 1985 | 993 | 538 (1) | 관절 4 / -3.042 | 0.0243 | abdomen / 18.9 cm | 아님 |
| iso | 2 | 1736 | 868 | 117 (30) | 관절 4 / -3.042 | 0.0009 | abdomen / 18.5 cm | 겹침 [866, 876] |
| iso | 4 | 2112 | 1056 | 411 (1) | 관절 4 / -3.042 | 0.0419 | abdomen / 18.0 cm | 아님 |
| iso | 5 | 1785 | 893 | 192 (26) | 관절 4 / -3.042 | 0.0026 | abdomen / 18.5 cm | 아님 |
| iso | 7 | 1980 | 990 | 448 (8) | 관절 4 / -3.042 | -0.0000 | abdomen / 17.9 cm | 아님 |

**off 겹침 구간** (seed 별, 프레임 = 재생 프레임)

| seed | 겹침 구간 프레임 (겹친 부위군, 최소 간격 cm) |
|---|---|
| 0 | 769~773 (몸통, -5.2); 839~845 (몸통, -4.7); 995~998 (몸통, -3.5); 1075~1083 (몸통, 위팔, -6.4) |
| 1 | 768~773 (몸통, -5.6); 861~871 (몸통, 위팔, -10.0); 992~994 (몸통, -1.7); 1099~1111 (아래팔, 몸통, 위팔, -10.4) |
| 2 | 776~779 (몸통, -4.0); 866~876 (몸통, 위팔, -8.0); 1014~1016 (몸통, -1.5); 1097~1106 (아래팔, 몸통, 위팔, -8.3) |
| 3 | 811~812 (몸통, -0.4); 924~932 (몸통, 위팔, -6.8); 1060~1064 (몸통, -3.2); 1159~1167 (아래팔, 손, 몸통, 위팔, -13.9) |
| 4 | 770~773 (몸통, -2.4); 859~871 (몸통, 위팔, -9.5); 993~994 (몸통, -0.5); 1081~1095 (아래팔, 몸통, 위팔, -9.7) |
| 5 | 771~785 (몸통, 위팔, -10.2); 876~881 (몸통, -5.4); 975~986 (아래팔, 몸통, 위팔, -8.1); 1061~1067 (아래팔, 몸통, 위팔, -6.6); 1157~1167 (아래팔, 몸통, 위팔, -15.3) |
| 6 | 814~816 (몸통, -1.8); 906~915 (몸통, 위팔, -7.2); 1064~1068 (몸통, -4.4); 1156~1165 (아래팔, 몸통, 위팔, -15.7) |
| 7 | 843~844 (몸통, -0.5); 933~950 (아래팔, 몸통, 위팔, -8.7); 1040~1043 (몸통, -1.9); 1122~1154 (아래팔, 몸통, 위팔, -10.6) |

- infeasible 7건 모두 **관절 4 하한 (−3.042 rad)**, 그때 최근접은 **배 (0.16~0.19 m)**. 사람 프레임 868~1145.
- off 겹침은 모든 seed 에서 프레임 768~1167 사이 4~5 구간, 주로 몸통 (후반 구간은 위팔·아래팔·손 포함).
- 같은 구간인가: infeasible 시작은 off 겹침이 생기는 시기(프레임 770~1170) 안에 있지만, 같은 seed 의 off 겹침 구간과 ±5 프레임 안에서 맞는 것은 7건 중 1건 (iso seed 2, 866~876) 뿐이다. 조건마다 로봇 경로가 달라 구간이 어긋난다. 사람이 로봇 쪽으로 들어오는 재생 후반에 off 는 겹치고, mass A·iso 는 몸통(η 0.15~0.20)을 피해 팔을 접다가 관절 4 한계에 걸린다 [추정: 공통 원인].

### (5) η_matched 정의 점검

- 현재: (스텝, 로봇 구)마다 가장 가까운 형상의 η_A 평균 = 0.1272 m (58,986개).
- 대안: 비용 > 0 인 (스텝, 로봇 구)마다 max_all 비용을 낸 형상의 η_A 평균 = **0.1892 m** (중앙 0.1895, 3,072개). mass A seed 0, 실행 상태 기준. (13절의 "비용이 켜진 쌍 평균 0.186" 은 쌍 단위 평균이라 다른 값.)

### (6) 발견·정리

- 분석 뒤 점검에서 /dev/shm 에 내가 띄우지 않은 항목 5개가 있었다: fastrtps_0a2c8e15bbf69974 (549408 B,  21:33:07), fastrtps_0a2c8e15bbf69974_el (0 B,  21:33:07), fastrtps_port14911 (52400 B,  21:33:07), fastrtps_port14911_el (0 B,  21:33:07), sem.fastrtps_port14911_mutex (32 B,  21:33:07). 21:33:07 에 생겼다. 쓰는 프로세스 없음 (`fuser` 확인). 목록을 기록하고 지움. (B2a 15절에서 원인 확인: 사용자 GUI 실행 `b1gui_massA_s0` 의 ready 시각이 21:33:07 이고, 2,523 스텝 예정이 약 1분 만에 끝남. 창을 닫아 Isaac 이 ROS 정리 없이 끝나며 남긴 것으로 보인다 [추정: 종료 경위])

### (7) 파일

- `goal1/scripts/b1_followup.py`, `goal1/logs/meff_isaac/b1_followup.json`. 보고서 직전본 `goal1/logs/step1_3_report.before_b1f.md`.

## 15. B2a — 뻗은 팔 (정지 자세까지, 사용자 확인 대기)

- 측정 전 남은 프로세스 0, /dev/shm 0.
- **정지 자세** (`goal1/scripts/b2a_pose.py`, effmass_human `HumanInertiaCore` 설치본 그대로, betas 0). 기록 재생 첫 프레임 선 자세에서 오른팔만 바꿈.
  - 1단계: 오른 어깨(SMPL 17) 축-각 3개를 최적화. 목표 = 위팔·아래팔이 골반 정면의 수평 투영 방향, 손바닥 법선(손 분절 −y)이 아래(−z). 팔꿈치·손목 0.
  - 축-각 0 인 팔꿈치는 SMPL 정지 자세의 굽힘 때문에 실제 5.4° 였다. 2단계로 어깨 3 + 팔꿈치 3 을 함께 최적화해 실제 팔꿈치 각도를 0° 로 맞춤 (목적 함수에 팔꿈치 각도² × 4 추가). 손목(21) = 0 (중립).
  - 최종 목적 함숫값 4.04e-04 (rad² 합).

**최종 SMPL 포즈 (축-각, rad). 굵게 = 바꾼 관절**

| # | 관절 | 첫 프레임 | 최종 (쓴 값) |
|---|---|---|---|
| 0 | pelvis(전역, 카메라 좌표) | -2.7267, -0.0560, 0.2588 | 같음 |
| 1 | L_hip | -0.0346, -0.0056, 0.0296 | 같음 |
| 2 | R_hip | -0.0412, 0.0295, -0.0355 | 같음 |
| 3 | spine1 | -0.1574, 0.0176, -0.0386 | 같음 |
| 4 | L_knee | -0.0877, 0.0074, -0.0196 | 같음 |
| 5 | R_knee | 0.0252, 0.1296, 0.0501 | 같음 |
| 6 | spine2 | 0.1848, -0.0696, 0.0930 | 같음 |
| 7 | L_ankle | -0.2442, 0.2966, 0.1814 | 같음 |
| 8 | R_ankle | -0.3528, -0.2062, -0.0533 | 같음 |
| 9 | spine3 | 0.0310, -0.0541, 0.0092 | 같음 |
| 10 | L_foot | 0.0282, 0.0094, -0.0141 | 같음 |
| 11 | R_foot | -0.0025, 0.0142, 0.0204 | 같음 |
| 12 | neck | -0.0386, -0.2755, 0.0711 | 같음 |
| 13 | L_collar | 0.1588, -0.0581, -0.2538 | 같음 |
| 14 | R_collar | 0.0922, 0.1622, 0.3593 | 같음 |
| 15 | head | 0.1657, -0.2611, 0.4197 | 같음 |
| 16 | L_shoulder | 0.3690, -0.4399, -0.8172 | 같음 |
| 17 | **R_shoulder** | 0.3731, 0.2854, 0.8265 | **0.2350, 1.5234, -0.5160** |
| 18 | L_elbow | 0.1990, -0.4793, 0.1454 | 같음 |
| 19 | **R_elbow** | 0.1440, 0.5437, -0.1840 | **-0.0588, -0.0575, 0.0712** |
| 20 | L_wrist | -0.0979, -0.1299, 0.1117 | 같음 |
| 21 | **R_wrist** | -0.2771, 0.1025, -0.1728 | **0.0000, 0.0000, 0.0000** |

- 위치(trans, 카메라 좌표) -0.5917, -0.0412, 2.7642 와 betas 0 은 첫 프레임 그대로.

| 확인 항목 | 값 |
|---|---|
| 팔꿈치 실제 각도 (위팔·아래팔 벡터 사이) | 0.10° (축-각 0 일 때 5.45°) |
| 어깨 굴곡 (위팔 ↔ 몸통 아래 방향) | 86.8° |
| 위팔·아래팔 수평 기울기 | 0.41° / 0.51° |
| 위팔·아래팔 ↔ 정면 | 0.41° / 0.51° |
| 손바닥 법선 ↔ 아래 | 0.92° |
| **손 ↔ 몸통·어깨/쇄골 형상 최소 간격** | **0.310 m (기준 ≥ 0.25 m, 통과. 처음 기준 0.4 m 는 사용자가 고침)** |
| 손 ↔ 머리 / 목 | 0.379 / 0.402 m |

형상별 간격 (손바닥·손가락 캡슐 22 반지름 0.061 m, 엄지 23 반지름 0.023 m):

| 형상 | 손바닥·손가락 (22) | 엄지 (23) |
|---|---|---|
| 골반 왼 (0) | 0.435 m | 0.482 m |
| 골반 오른 (1) | 0.384 m | 0.442 m |
| 배 왼 (4) | 0.388 m | 0.438 m |
| 배 오른 (5) | 0.310 m | 0.373 m |
| 가슴 왼 (10) | 0.376 m | 0.433 m |
| 가슴 오른 (11) | 0.323 m | 0.396 m |
| 왼 어깨(쇄골) (13) | 0.429 m | 0.488 m |
| 오른 어깨(쇄골) (14) | 0.373 m | 0.451 m |
| 목 (12) | 0.402 m | 0.473 m |
| 머리 (15) | 0.379 m | 0.454 m |

- 가장 가까운 것: 손바닥·손가락 캡슐 ↔ 배 오른쪽 캡슐 0.310 m.
- **precomputed 저장:** `goal1/data/human_b2a_reach.npz` (A단계 precomputed 와 같은 키, 1 프레임, 속도 0·무효, `pose66` 추가). Isaac `--human-source precomputed` 로 headless 120 스텝 읽기 시험 통과 (프레임 1개, 텐서 주소 불변).

**참고: 사람 쪽만 본 오른손 m_h(u)** (u = 로봇 → 손 COM. 정면 = −팔 축(팔꿈치→손목), 위 = 아래 방향, 옆 = 수평 직각). η 는 세트 A. m_r 은 장면 배치 뒤에 계산하므로 두 경우만: m_r = ∞ (μ = m_h), m_r = 4.9 kg (B1 손 링크 구 m_r 중앙값을 가정)

| 평면 | 각도 | m_h (kg) | η (m_r = ∞) | η (m_r = 4.9 가정) |
|---|---|---|---|---|
| 정면→위 | 0° | 7.915 | 0.199 | 0.123 |
| 정면→위 | 15° | 5.100 | 0.160 | 0.112 |
| 정면→위 | 30° | 2.703 | 0.116 | 0.093 |
| 정면→위 | 45° | 1.663 | 0.091 | 0.079 |
| 정면→위 | 60° | 1.205 | 0.078 | 0.070 |
| 정면→위 | 75° | 1.005 | 0.071 | 0.065 |
| 정면→위 | 90° | 0.950 | 0.069 | 0.063 |
| 정면→옆 | 0° | 7.915 | 0.199 | 0.123 |
| 정면→옆 | 15° | 3.393 | 0.130 | 0.100 |
| 정면→옆 | 30° | 1.584 | 0.089 | 0.077 |
| 정면→옆 | 45° | 0.956 | 0.069 | 0.063 |
| 정면→옆 | 60° | 0.700 | 0.059 | 0.055 |
| 정면→옆 | 75° | 0.597 | 0.055 | 0.052 |
| 정면→옆 | 90° | 0.582 | 0.054 | 0.051 |

- 오른손 m_h 전체 범위 (Λ_h 고유값) 0.58 ~ 8.58 kg.
- 정면 − 위 η 차: m_r = ∞ 이면 13.0 cm, m_r = 4.9 kg 가정이면 6.0 cm (관문 경계). 실제 관문은 장면 g2 배치·IK 로 대표 자세 m_r 을 구한 뒤 [미확인].
- η 하한(0.05)·상한(0.20): m_r = ∞ 정면 0.199 (상한 직전), 옆 90° 0.054 (하한 근처). m_r = 4.9 가정에서는 안 걸림.

**정지 자세 GUI 확인 명령 (실행 안 함)**

- 보기용 임시 배치 `goal1/b2a/view/temp_transform.json`: g1 변환에서 사람을 등 쪽(수평)으로 0.6 m 옮김 (g1 그대로면 손끝이 로봇 베이스 수평 0.20 m 안까지 들어옴). 손끝은 베이스에서 수평 0.80 m, 높이 z 0.435 m. 장면 g2 배치가 아니다.
- 표시: 사람 형상 캡슐 24개 반투명 + 분절 COM 19개 (메시는 precomputed 경로에 없음). 로봇은 g1 두 목표를 오가고 비용 off. 1,800 스텝(약 30 s) 뒤 스스로 종료.
- 같은 옵션 headless 120 스텝 시험: 오류 없음.

```bash
cd ~/effmass_mpc/goal1
RUN_ISAAC_NO_DISPLAY=0 scripts/g13_pre_run.sh b2a_pose_gui --human-pre ~/effmass_mpc/goal1/data/human_b2a_reach.npz \
  --human-dir ~/effmass_mpc/goal1/b2a/view --meff on --meff-mode off --meff-viz shapes --steps 1800 --gui
```

**이번에 발견·정리한 것**

- 보기용 변환 json 의 설명 문자열에 한글을 넣었더니 Isaac(ROS 실행 환경, ASCII) 이 json 을 못 읽어 첫 headless 시험이 시작 직후 죽었다. 설명을 영문으로 바꿈. 죽은 실행이 남긴 /dev/shm fastrtps 5개(21:44:27, `fastrtps_port14911` 포함)를 지움. 14절의 21:33:07 항목은 사용자 GUI 실행 `b1gui_massA_s0` 에서 생김 (15.1 참고).
- 추가 지시 (rollout 안 비용 형상 집계)는 MPC 실행 단계(4번)에서 넣는다. 아직 코드 변경 없음.

**다음 (사용자 확인 뒤):** 3번 장면 g2 (사람 마주 보기 배치, front·top 경로, 각 경로 직선과 몸통·어깨·머리 형상 ≥ 0.35 m 확인, IK·관절 여유) → 2번 관문의 m_r·μ·η 표 → 관문 통과 시 4번 실행.

### 15.1 B2a display and shape check (English)

Rule for this round: collision shapes (24 capsules) and the cost are unchanged; only display and checks.

**0. Pre-check.** At the start /dev/shm had 5 fastrtps entries (21:46:45). They came from the user's GUI run `b2a_pose_gui` (ready 21:46:45, stopped after about 15 s), and the 21:33:07 entries in section 14 came from the user's GUI run `b1gui_massA_s0` (ready 21:33:07, stopped after about 1 min of 2,523 steps). Isaac exits without ROS cleanup when the window is closed early [inferred]. No process was using them; removed. `g13_pre_run.sh` and `g1_meff_run.sh` now remove fastrtps entries newer than the run's own start marker when no related process is left. Processes 0, /dev/shm 0 after that.

**1. Mesh.** `goal1/scripts/b2a_mesh.py` (run with `g1_run_human.sh py`). Vertices = `HumanInertiaCore.smplh.forward(betas, rotmat_world).vertices + trans_world`, with `rotmat_world` from the node path (`aa2euler` -> `G2Hpose_idx` -> `batch_roteulerSMPL`), betas 0, same pose, global orientation, translation and camera-to-world transform as the node. Sensing `smpl_vertices` not used. Faces = `core.smplh.faces` (13,776). Display-only files, separate from the cost path:
- static pose: `goal1/data/human_b2a_reach_verts.npz` (1 frame, computed once)
- replay: `goal1/data/human_precomputed_1781524848_verts.npz` (1,262 frames)

Alignment (vertex -> nearest capsule surface of its own segment; positive = outside; static pose):

| segment | vertices | p95 (cm) | median (cm) | outside | centroid - capsule-axis midpoint (m) |
|---|---|---|---|---|---|
| pelvis | 399 | 0.17 | -7.53 | 5.5% | [0.002, -0.0142, -0.0498] |
| left_thigh | 237 | 0.10 | -3.30 | 5.9% | [-0.0127, -0.0061, 0.0231] |
| right_thigh | 234 | 0.13 | -3.16 | 6.4% | [0.0091, -0.0066, 0.0228] |
| abdomen | 200 | -0.31 | -5.51 | 2.0% | [-0.0012, 0.0003, -0.0204] |
| left_calf | 199 | 0.02 | -3.55 | 5.5% | [0.0033, 0.0255, 0.029] |
| right_calf | 199 | -0.28 | -3.39 | 3.0% | [0.0027, 0.0217, 0.0366] |
| left_foot | 253 | 0.21 | -2.99 | 6.7% | [0.0165, -0.0407, -0.0136] |
| right_foot | 253 | -0.04 | -2.79 | 4.7% | [-0.0159, -0.0327, -0.0126] |
| chest | 808 | 0.07 | -4.37 | 5.7% | [0.0009, -0.0005, -0.0195] |
| neck | 118 | 0.14 | -0.94 | 8.5% | [0.0042, 0.0026, -0.0038] |
| left_shoulder | 114 | 0.65 | -2.48 | 14.9% | [0.0361, 0.0196, 0.0244] |
| right_shoulder | 113 | 0.57 | -2.79 | 9.7% | [-0.0229, 0.0315, 0.0196] |
| head | 1203 | 0.02 | -2.77 | 5.2% | [-0.0157, -0.0288, -0.0152] |
| left_upper_arm | 271 | -0.47 | -2.32 | 1.8% | [-0.0055, 0.0131, 0.0011] |
| right_upper_arm | 272 | -0.81 | -2.77 | 0.0% | [-0.0065, 0.0205, -0.0131] |
| left_lower_arm | 230 | 0.15 | -2.07 | 7.0% | [-0.0053, -0.0006, 0.0075] |
| right_lower_arm | 230 | -0.38 | -1.79 | 1.7% | [0.0021, 0.0061, 0.0026] |
| left_hand | 778 | -0.18 | -3.12 | 2.8% | [0.0209, -0.0254, -0.0449] |
| right_hand | 778 | 0.20 | -1.75 | 7.1% | [-0.0343, -0.0478, 0.0023] |

- Replay frames 0 / 300 / 600 / 900 / 1261: max over segments of p95 = 0.63 / 0.72 / 0.68 / 0.89 / 2.56 cm. The centroid offsets are a few cm in different directions per segment (capsule fit geometry), not one common shift. One vertex has no segment label in the node partition (ignored).

**2. Display** (`g1_human_link.py`, `step2b_mpc_isaac.py`; backups `logs/*.before_b2a_viz.py`)
- Capsules: one colour (0.30, 0.60, 0.95), opacity 0.30 (this also applies to the ROS-path capsule display). Mesh: opaque skin colour, points/topology/extent set at creation, points updated only when the frame changes.
- `--human-viz none|mesh|shapes|both` (default both) for `--human-source precomputed`; `--human-mesh-pre` (default `<human-pre>_verts.npz`). Closest-pair lines and COM points use the existing options. `b1_campaign.sh` now passes `--human-viz none` so B1-type runs keep the same prims as before.
- First headless test showed one Hydra message `Skipping invalid mesh ... invalid points buffer` (points were set after the topology). Fixed by setting points and extent at creation; not seen in the following runs.

**3. Shape check (static pose, report only)** `goal1/scripts/b2a_shape_check.py`. Mesh is closed (Euler characteristic 2; winding number 1.0 inside the pelvis, 0.0 far away).
- Under-cover = mesh vertices outside the segment's capsules (fraction, p95 of the outside distance).
- Over-cover = 1,500 area-uniform samples per capsule surface (samples inside another capsule of the same segment removed); samples outside the mesh (winding < 0.5) -> exact distance to the mesh surface (median, p95), and the fraction of samples outside.

| segment | capsules (radius m) | under: outside / p95 (cm) | over: outside / median / p95 (cm) |
|---|---|---|---|
| pelvis | [0, 1] (0.1935, 0.1862) | 5.5% / 1.33 | 80% / 7.0 / 12.7 |
| left_thigh | [2] (0.1145) | 5.9% / 0.62 | 68% / 3.4 / 6.8 |
| right_thigh | [3] (0.1131) | 6.4% / 0.92 | 66% / 3.4 / 6.8 |
| abdomen | [4, 5] (0.1628, 0.1609) | 2.0% / 0.14 | 69% / 4.6 / 9.2 |
| left_calf | [6] (0.0877) | 5.5% / 0.58 | 91% / 3.9 / 6.6 |
| right_calf | [7] (0.0882) | 3.0% / 0.15 | 95% / 3.8 / 6.6 |
| left_foot | [8] (0.0905) | 6.7% / 1.35 | 97% / 4.4 / 6.8 |
| right_foot | [9] (0.0808) | 4.7% / 1.09 | 98% / 3.7 / 5.8 |
| chest | [10, 11] (0.1479, 0.1463) | 5.7% / 1.57 | 79% / 4.2 / 11.4 |
| neck | [12] (0.0695) | 8.5% / 0.56 | 39% / 0.9 / 1.8 |
| left_shoulder | [13] (0.1044) | 14.9% / 1.30 | 50% / 3.3 / 5.9 |
| right_shoulder | [14] (0.0989) | 9.7% / 1.30 | 49% / 2.7 / 5.1 |
| head | [15] (0.1177) | 5.2% / 1.47 | 80% / 2.2 / 3.9 |
| left_upper_arm | [16] (0.0756) | 1.8% / 0.27 | 76% / 2.9 / 4.7 |
| right_upper_arm | [17] (0.0769) | 0.0% / 0.00 | 81% / 2.7 / 4.1 |
| left_lower_arm | [18] (0.0548) | 7.0% / 0.67 | 89% / 2.1 / 3.0 |
| right_lower_arm | [19] (0.0544) | 1.7% / 0.15 | 91% / 1.8 / 2.8 |
| left_hand | [20, 21] (0.0746, 0.0229) | 2.8% / 0.61 | 98% / 2.9 / 5.8 |
| right_hand | [22, 23] (0.0609, 0.0227) | 7.1% / 0.87 | 97% / 1.7 / 4.5 |

**Right hand by direction** (support of the capsule union 22+23 minus support of the right-hand mesh vertices; local = within +-2 cm of the hand COM along the arm axis, where a top path would pass)

| direction | over-cover, whole hand (cm) | over-cover, local (cm) | real gap for a path 3 cm from the capsule surface (cm) |
|---|---|---|---|
| tip (arm axis, elbow -> wrist) | -0.4 | - | 2.6 |
| top (+z, back of hand) | +3.8 | +4.3 | 7.3 |
| outward side (person's right, horizontal) | +0.1 | +0.8 | 3.8 |

- The palm/finger capsule (radius 0.061 m) is set by the hand width; the hand is flat (palm down), so the capsule rises about 4 cm above the back of the hand. Along the arm axis the fingertips reach 0.4 cm beyond the capsule cap. With equal capsule gaps, the B2a top path would be about 4.5 cm farther from the real hand than the front path [checked: numbers].

**Torso layout proposals (report only, nothing changed)** `goal1/scripts/b2a_torso_alt.py`. Over-cover of the current torso capsules is large (pelvis median 7.0 cm). Proposals with the same or fewer capsules, same fit rule (axis through the vertex centroid, end points moved in by r, r = p95):
- alt1: 1 horizontal capsule per segment along the body's left-right axis (3 capsules).
- alt2: 2 horizontal capsules per segment, vertices split at the median height (6).
- alt3: alt2 for chest and abdomen, alt1 for pelvis (5).

| layout | capsules | segment | radius (m) / length (m) | under: outside / p95 (cm) | over: outside / median / p95 (cm) |
|---|---|---|---|---|---|
| current | 6 | pelvis | 0.1935, 0.1862 / 0.1205, 0.1205 | 5.5% / 1.33 | 79% / 6.8 / 12.9 |
| current | 6 | abdomen | 0.1628, 0.1609 / 0.1448, 0.1448 | 2.0% / 0.14 | 70% / 4.7 / 9.2 |
| current | 6 | chest | 0.1479, 0.1463 / 0.2788, 0.2788 | 5.7% / 1.57 | 80% / 4.2 / 11.0 |
| alt1 | 3 | pelvis | 0.1973 / 0.0 | 5.0% / 1.14 | 65% / 2.9 / 6.7 |
| alt1 | 3 | abdomen | 0.1703 / 0.0 | 5.0% / 0.33 | 46% / 2.1 / 5.0 |
| alt1 | 3 | chest | 0.214 / 0.0 | 5.1% / 2.23 | 74% / 3.6 / 9.6 |
| alt2 | 6 | pelvis | 0.1685, 0.1754 / 0.0163, 0.0272 | 4.5% / 0.57 | 65% / 2.0 / 5.5 |
| alt2 | 6 | abdomen | 0.1642, 0.1739 / 0.0, 0.0 | 2.0% / 0.25 | 56% / 3.0 / 5.8 |
| alt2 | 6 | chest | 0.2046, 0.1973 / 0.0, 0.0296 | 4.6% / 0.85 | 80% / 4.6 / 9.7 |
| alt3 | 5 | pelvis | 0.1973 / 0.0 | 5.0% / 1.14 | 65% / 2.9 / 6.7 |
| alt3 | 5 | abdomen | 0.1642, 0.1739 / 0.0, 0.0 | 2.0% / 0.25 | 56% / 3.0 / 5.8 |
| alt3 | 5 | chest | 0.2046, 0.1973 / 0.0, 0.0296 | 4.6% / 0.85 | 80% / 4.6 / 9.7 |

- The horizontal fits collapse to length about 0 (spheres): the p95 radius (0.16-0.21 m) is about half the body width, so a left-right capsule cannot be longer than a point. alt1 lowers the over-cover median for pelvis (7.0 -> 2.9 cm) and abdomen (4.6 -> 2.1 cm) with 3 instead of 6 capsules, but chest under-cover p95 rises (1.6 -> 2.2 cm). alt2/alt3 give no clear gain over alt1. A real reduction of torso over-cover would need non-round cross-sections or more capsules, which is outside this rule. Decision is the user's.

**4. Display check without MPC** `goal1/scripts/b2a_viz_check.py` via `b2a_viz_run.sh` (same environment as `g13_pre_run.sh`; pre-check 0/0; ~/hrc_ws changes 0; /dev/shm 0 after; processes 0 after). Timing recorded only.

| run | prims under /World/human | mesh updates | mesh update (ms) | pump mean after first (ms) | loop mean / p95 (ms, headless, no render) |
|---|---|---|---|---|---|
| static pose, 120 steps | {'': 2, 'PointInstancer': 1, 'Sphere': 1, 'Mesh': 1, 'Capsule': 24} | 1 | 0.57 | 0.044 | 1.51 / 2.13 |
| replay (g1 placement), 600 steps | {'': 2, 'PointInstancer': 1, 'Sphere': 1, 'Mesh': 1, 'Capsule': 24} | 301 | 0.80 (p95 0.84) | 5.119 | 5.62 / 10.74 |

- One mesh and one capsule set (24) are created; the 2 untyped prims are the /World/human and /World/human/shapes containers, PointInstancer + Sphere are the 19 COM points. The replay pump mean (5.1 ms) includes the capsule transform updates and the GPU input copy on new frames.

**5. Figures**: `goal1/b2a/b2a_fig_pose_side.png`, `goal1/b2a/b2a_fig_pose_top.png` (orthographic, mesh shaded + capsule outlines, right-hand zoom).

**Static-pose GUI command (not run).** No MPC; robot at retract for scale; viewing placement `goal1/b2a/view` (g1 transform moved 0.6 m to the human's back, not scene g2); mesh + one-colour translucent capsules + COM points; camera aimed at the right hand; closes by itself after 30 s.

```bash
cd ~/effmass_mpc/goal1
RUN_ISAAC_NO_DISPLAY=0 scripts/b2a_viz_run.sh b2a_pose_gui2 --gui --duration 30
```

- The GUI path of `b2a_viz_check.py` (render=True, window close) was not run [not checked]; the same script ran headless without errors.
- Rollout cost-shape aggregation will be added at step 4 (MPC runs). Step 3 (scene g2, paths) waits for the user's GUI check.

## Shapes v2 (English, ASCII)

Principle (user): shapes slightly smaller than the SMPL mesh, natural look first; collision margin is designed with eta. A-stage and B1 keep v1 (files and results preserved); B2a onward uses v2.

**0. Pre-check**: processes 0, /dev/shm 0.

**1. Fit** (`goal1/scripts/shapes_v2_fit.py`, run with `g1_run_human.sh py`; output `goal1/data/shapes_v2.json`)
- Capsules in segment-local coordinates (p_world = J_s + R_s p_local), the node pose path is unchanged. Fit cloud = vertices of the B2a static pose and replay frames 0/300/600/900/1261 mapped into each segment frame.
- Layout: pelvis / abdomen / chest one left-right capsule each (chest x range taken from chest + both shoulder vertices, so its end caps are the shoulders; no shoulder shapes); hands: palm row + finger row along the palm width + thumb; head = v1 PCA capsule unchanged; thighs, calves, upper/lower arms, neck = v1 joint axis with both end points moved in by r; feet = v1 direction, axis moved to pass through the foot vertex centroid (changed: the v1 axis through the ankle needs a radius down to the sole), ends moved in by r.
- Radius rule: r_under = smallest r with under p95 <= 2 cm (union of all current shapes, all fit frames); r_over = largest r with over median <= 0.5 cm on the static mesh, where a surface sample inside the mesh but in another segment's region counts as over (distance to the nearest own vertex) so that a shape cannot grow into its neighbours (a first try without this made the neck a 0.10 m sphere inside chest/head). r = r_over if r_over >= r_under, else r_under (under-cover priority). Two passes so each shape sees its fitted neighbours.
- Hands: grid over (palm row position, r_palm, r_finger), smallest r_palm + r_finger with hand under p95 <= 2 cm and fingertip under p95 <= 1 cm (non-thumb vertices within 2 cm of the most distal point); finger row placed so its distal cap reaches the fingertips; thumb = v1 thumb axis, smallest r with thumb under p95 <= 2 cm.

**Shape list (v2)**

| # | segment | kind | radius (m) | length (m) | r_over / r_under (m) |
|---|---|---|---|---|---|
| 0 | 0 pelvis | torso left-right | 0.1707 | 0.023 | 0.142 / 0.171 |
| 1 | 1 left_thigh | limb-inset | 0.0956 | 0.200 | 0.086 / 0.096 |
| 2 | 2 right_thigh | limb-inset | 0.0979 | 0.204 | 0.083 / 0.098 |
| 3 | 3 abdomen | torso left-right | 0.1429 | 0.043 | 0.096 / 0.143 |
| 4 | 4 left_calf | limb-inset | 0.0717 | 0.275 | 0.046 / 0.072 |
| 5 | 5 right_calf | limb-inset | 0.0718 | 0.276 | 0.045 / 0.072 |
| 6 | 6 left_foot | limb-inset (foot axis through vertex centroid) | 0.0534 | 0.150 | 0.036 / 0.053 |
| 7 | 7 right_foot | limb-inset (foot axis through vertex centroid) | 0.0429 | 0.176 | 0.038 / 0.043 |
| 8 | 8 chest | torso left-right (to shoulder width) | 0.1709 | 0.081 | 0.000 / 0.171 |
| 9 | 9 neck | limb-inset | 0.0633 | 0.000 | 0.063 / 0.048 |
| 10 | 12 head | v1 head | 0.1177 | 0.014 | - |
| 11 | 13 left_upper_arm | limb-inset | 0.0504 | 0.171 | 0.050 / 0.043 |
| 12 | 14 right_upper_arm | limb-inset | 0.0528 | 0.159 | 0.053 / 0.046 |
| 13 | 15 left_lower_arm | limb-inset | 0.0403 | 0.183 | 0.040 / 0.033 |
| 14 | 16 right_lower_arm | limb-inset | 0.0434 | 0.181 | 0.043 / 0.033 |
| 15 | 17 left_hand | hand palm row + finger row + thumb | 0.0175 | 0.055 | - |
| 16 | 17 left_hand | hand palm row + finger row + thumb | 0.0455 | 0.021 | - |
| 17 | 17 left_hand | hand palm row + finger row + thumb | 0.0060 | 0.080 | - |
| 18 | 18 right_hand | hand palm row + finger row + thumb | 0.0150 | 0.063 | - |
| 19 | 18 right_hand | hand palm row + finger row + thumb | 0.0455 | 0.021 | - |
| 20 | 18 right_hand | hand palm row + finger row + thumb | 0.0060 | 0.081 | - |
| 21-23 | 0 (inactive) | padding, 1 km away, radius 0, not drawn | 0 | 0 | - |

- left_hand: palm row at 5 cm from the wrist along the hand axis (knuckles at 10.6 cm, fingertips at 19.9 cm), r_palm 0.0175, r_finger 0.0455, r_thumb 0.006 m.

- right_hand: palm row at 6 cm from the wrist along the hand axis (knuckles at 10.3 cm, fingertips at 19.8 cm), r_palm 0.0150, r_finger 0.0455, r_thumb 0.006 m.

**2. Per-frame values (v2)**: under p95 (cm) / over median (cm)

| segment | static | replay0 | replay300 | replay600 | replay900 | replay1261 |
|---|---|---|---|---|---|---|
| pelvis | 1.7 / 0.2 | 1.8 / 0.2 | 1.7 / 0.2 | 1.7 / 0.2 | 1.5 / 0.0 | 2.6 / 0.0 |
| left_thigh | 1.9 / 1.0 | 1.8 / 1.1 | 1.9 / 1.0 | 1.9 / 1.0 | 1.9 / 0.0 | 2.7 / 0.6 |
| right_thigh | 1.7 / 0.9 | 1.7 / 1.0 | 1.7 / 0.9 | 1.7 / 0.9 | 1.7 / 0.0 | 2.9 / 0.4 |
| abdomen | 1.8 / 0.0 | 1.9 / 0.0 | 1.5 / 0.0 | 1.6 / 0.0 | 1.8 / 0.0 | 2.3 / 0.0 |
| left_calf | 2.1 / 2.2 | 2.1 / 2.2 | 2.0 / 2.1 | 2.0 / 2.1 | 1.8 / 1.9 | 1.6 / 1.4 |
| right_calf | 1.7 / 2.1 | 2.0 / 2.1 | 2.0 / 2.2 | 2.2 / 2.2 | 1.9 / 1.5 | 1.5 / 1.7 |
| left_foot | 2.0 / 1.6 | 2.0 / 1.6 | 2.0 / 1.5 | 2.0 / 1.5 | 2.0 / 1.4 | 1.9 / 1.6 |
| right_foot | 2.1 / 0.7 | 2.1 / 0.7 | 2.0 / 0.7 | 2.0 / 0.8 | 1.7 / 0.7 | 1.9 / 0.8 |
| chest | 1.0 / 0.9 | 0.9 / 0.7 | 0.9 / 0.8 | 0.9 / 0.6 | 0.8 / 0.8 | 0.9 / 0.6 |
| neck | 0.8 / 0.0 | 1.1 / 0.0 | 1.1 / 0.0 | 1.1 / 0.0 | 1.4 / 0.0 | 1.2 / 0.0 |
| left_shoulder | 2.6 / - | 2.8 / - | 2.9 / - | 2.7 / - | 2.8 / - | 2.6 / - |
| right_shoulder | 2.2 / - | 2.3 / - | 2.6 / - | 2.5 / - | 2.8 / - | 2.7 / - |
| head | 0.0 / 1.7 | 0.0 / 1.7 | 0.0 / 1.9 | 0.0 / 1.9 | 0.0 / 1.8 | 0.0 / 1.8 |
| left_upper_arm | 1.7 / 0.0 | 1.8 / 0.0 | 1.3 / 0.0 | 1.3 / 0.0 | 1.8 / 0.0 | 1.4 / 0.0 |
| right_upper_arm | 2.1 / 0.2 | 1.4 / 0.1 | 1.2 / 0.0 | 1.3 / 0.0 | 1.6 / 0.1 | 1.6 / 0.1 |
| left_lower_arm | 1.6 / 0.5 | 1.6 / 0.4 | 1.6 / 0.1 | 1.6 / 0.1 | 2.1 / 0.4 | 1.4 / 0.2 |
| right_lower_arm | 1.3 / 0.6 | 1.4 / 0.5 | 1.6 / 0.4 | 1.6 / 0.4 | 1.4 / 0.3 | 1.2 / 0.4 |
| left_hand | 2.0 / 0.0 | 2.0 / 0.0 | 2.0 / 0.0 | 2.0 / 0.0 | 1.9 / 0.0 | 2.0 / 0.0 |
| right_hand | 2.0 / 0.0 | 2.0 / 0.0 | 2.0 / 0.0 | 2.0 / 0.0 | 1.7 / 0.0 | 2.0 / 0.0 |

Same for v1:

| segment | static | replay0 | replay300 | replay600 | replay900 | replay1261 |
|---|---|---|---|---|---|---|
| pelvis | 0.0 / 5.1 | 0.0 / 3.7 | 0.0 / 3.9 | 0.0 / 3.9 | 0.0 / 4.3 | 0.0 / 3.8 |
| left_thigh | 0.0 / 2.3 | 0.0 / 2.3 | 0.0 / 2.1 | 0.0 / 2.1 | 0.0 / 1.0 | 0.0 / 2.5 |
| right_thigh | 0.0 / 2.4 | 0.0 / 2.4 | 0.0 / 2.1 | 0.0 / 2.1 | 0.0 / 0.8 | 0.0 / 2.1 |
| abdomen | 0.0 / 2.6 | 0.0 / 2.0 | 0.0 / 1.9 | 0.0 / 1.8 | 0.0 / 1.5 | 1.3 / 1.5 |
| left_calf | 0.0 / 3.6 | 0.0 / 3.6 | 0.0 / 3.2 | 0.0 / 3.0 | 0.0 / 2.5 | 0.0 / 3.1 |
| right_calf | 0.0 / 3.5 | 0.0 / 3.5 | 0.0 / 3.0 | 0.1 / 2.9 | 0.0 / 2.6 | 0.0 / 3.3 |
| left_foot | 0.2 / 4.5 | 0.2 / 4.5 | 0.2 / 3.9 | 0.2 / 3.9 | 0.2 / 3.8 | 0.0 / 4.5 |
| right_foot | 0.0 / 3.7 | 0.0 / 3.7 | 0.0 / 3.4 | 0.0 / 3.4 | 0.0 / 3.2 | 0.0 / 3.7 |
| chest | 0.0 / 3.0 | 0.0 / 2.8 | 0.0 / 2.8 | 0.0 / 2.8 | 0.0 / 3.1 | 0.0 / 3.0 |
| neck | 0.0 / 0.0 | 0.0 / 0.0 | 0.0 / 0.0 | 0.0 / 0.0 | 0.0 / 0.0 | 0.0 / 0.0 |
| left_shoulder | 0.0 / 0.2 | 0.0 / 0.0 | 0.0 / 0.3 | 0.0 / 0.4 | 0.0 / 0.1 | 0.0 / 0.3 |
| right_shoulder | 0.0 / 0.2 | 0.0 / 0.0 | 0.0 / 0.1 | 0.0 / 0.1 | 0.0 / 0.0 | 0.0 / 0.2 |
| head | 0.0 / 1.7 | 0.0 / 1.7 | 0.0 / 1.9 | 0.0 / 1.9 | 0.0 / 1.8 | 0.0 / 1.8 |
| left_upper_arm | 0.0 / 2.2 | 0.0 / 2.1 | 0.0 / 2.2 | 0.0 / 2.2 | 0.0 / 1.4 | 0.0 / 2.0 |
| right_upper_arm | 0.0 / 2.5 | 0.0 / 2.5 | 0.0 / 2.3 | 0.0 / 2.3 | 0.0 / 2.3 | 0.0 / 2.6 |
| left_lower_arm | 0.0 / 2.0 | 0.0 / 1.9 | 0.0 / 1.7 | 0.0 / 1.8 | 0.0 / 1.6 | 0.0 / 1.7 |
| right_lower_arm | 0.0 / 1.7 | 0.0 / 1.7 | 0.0 / 1.6 | 0.0 / 1.5 | 0.0 / 1.2 | 0.0 / 1.5 |
| left_hand | 0.0 / 3.1 | 0.0 / 3.0 | 0.0 / 3.0 | 0.0 / 2.9 | 0.0 / 1.2 | 0.1 / 2.3 |
| right_hand | 0.2 / 1.6 | 0.0 / 1.5 | 0.0 / 1.5 | 0.0 / 1.5 | 0.0 / 0.0 | 0.3 / 1.4 |

- Metric definitions (`goal1/scripts/shape_metrics.py`, `shapes_eval.py`): under = p95 over the segment's vertices of max(0, distance outside the union of all shapes); over = 800 area-uniform samples per shape (internal parts inside another shape of the same segment removed), distance outside the mesh (exact point-triangle, 0 inside by winding number), median / p95 over all samples. The v1 numbers here use the same definitions, so they differ from section 15.1 (which used the segment's own shapes for under-cover and outside-only samples for over-cover).
- Under-cover target missed: shoulders 2.2-2.9 cm (no own shape), calves up to 2.2 cm, and replay frame 1261 pelvis/thighs/abdomen 2.3-2.9 cm (a pose outside the static/early-replay range). Over-cover median target missed: calves 1.4-2.2 cm, feet 0.7-1.6 cm, thighs ~1.0 cm (radius set by under-cover), head 1.7-1.9 cm (v1 head kept).

**3. Connection** (`g13_human_precompute.py --shapes v2`, `step2b_mpc_isaac.py --shapes v1|v2`, `g1_human_link.py`)
- v2 precomputed files: `goal1/data/human_precomputed_1781524848_v2.npz` (replay) and `goal1/data/human_b2a_reach_v2.npz` (static). Only the shape arrays differ from v1: com, vel, inv, rot, joints are bit-equal to the v1 files [checked]. Re-generating the v1 replay file reproduces it bit-equal [checked].
- `--shapes v2` selects `<human-pre>_v2.npz`; the display mesh file is taken from the v1 name. Inactive padding shapes (radius 0) are not drawn. Default `--shapes v1`.
- v1 reproduction: B1 mass A seed 0 re-run with the new code (`b1_massA_s0_v1check`): commands 2,523 steps and per-step B1 record bit-equal to the original run [checked].
- GPU vs CPU (Isaac dumps, v2, mass A, 6 steps x 300 pairs x 61 = 109,800): nearest-shape match 100%, distance abs diff <= 3.4e-07 m, mu rel diff <= 1.0e-06; cost > 0 in 642: cost-shape match 100%, eta rel diff <= 3.6e-07, weighted cost >= 1 rel diff <= 1.6e-04; (b,h) sum rel diff max 4.9e-03 at a reference sum of 0.0012 (float32 cancellation at tiny cost), <= 7.1e-05 where the sum >= 10. Same class as the A-stage dumps (A-stage iso dump: sum max 2.9e-4 at a sum of 0.27).

**4. Mesh-based gap tool** (`goal1/scripts/mesh_gap.py <tag>`, offline): robot sphere centres from the measured joint values (RobotInertiaNP, same 61 spheres), mesh of the frame used at that step, run's temp_transform; gap = distance to the body-part mesh surface minus the MPC sphere radius (yml + 0.02; also without buffer), negative inside (winding). Pipeline check: the same code with capsules instead of the mesh reproduces the GPU-logged shape gaps of `b1_massA_s0` within 6.9e-7 m (51 steps).

| run | group | shape-based min gap (cm) | mesh-based min gap (cm) |
|---|---|---|---|
| b1_massA_s0 (v1 shapes) | hand | 13.7 | 14.3 |
| b1_massA_s0 (v1 shapes) | forearm | 13.7 | 16.5 |
| b1_massA_s0 (v1 shapes) | upperarm | 11.6 | 16.4 |
| b1_massA_s0 (v1 shapes) | torso | 12.4 | 18.6 |
| b1_massA_s0 (v1 shapes) | legs | 25.2 | 25.2 |

- In B1 (v1 shapes) the real body was several cm farther than the shape-based gaps (seed 0: torso 12.4 -> 18.6 cm, upper arm 11.6 -> 16.4 cm). From now on result tables give both columns.

**Right hand, static pose: where a path 3 cm from the shape surface really is** (support of the right-hand shapes minus support of the right-hand mesh, surface samples; local = within +-2 cm of the hand COM along the arm axis)

| direction | v1: real gap (cm), whole / local | v2: real gap (cm), whole / local |
|---|---|---|
| tip | 2.6 / - | 2.6 / - |
| top | 6.8 / 7.3 | 4.2 / 4.2 |
| side | 3.1 / 3.8 | 2.8 / 2.5 |

- v2 top is still about 1.2 cm above the back of the hand because the finger row radius (4.55 cm) is set by the under-cover target over the finger length.

**5. Performance** (paired, same measured state, alternating with the 1-2d channel solver; precomputed replay, mass A max_all, 1200 steps; order v1, v2, v2, v1)

| run | solver with cost (ms) | 1-2d channel solver (ms) | increase mean / median / p95 (ms) |
|---|---|---|---|
| v2perf_v1_1 | 8.406 | 7.690 | 0.717 / 0.755 / 1.740 |
| v2perf_v2_1 | 8.355 | 7.679 | 0.676 / 0.687 / 1.483 |
| v2perf_v2_2 | 8.250 | 7.537 | 0.713 / 0.708 / 1.529 |
| v2perf_v1_2 | 8.421 | 7.724 | 0.697 / 0.707 / 1.712 |

- Mean increase v1 0.707 ms, v2 0.694 ms: v2 keeps <= 1 ms (pass). Recaptures 0 (warm-up captures 2 = two solvers). SM clock median 1965 MHz in all four runs.

**6. Display and checks** (headless, no MPC, `b2a_viz_run.sh`)

| run | prims under /World/human | mesh updates / mean (ms) | loop mean / p95 (ms) |
|---|---|---|---|
| static v2, 120 steps | {'': 2, 'PointInstancer': 1, 'Sphere': 1, 'Mesh': 1, 'Capsule': 21} | 1 / 0.57 | 1.48 / 2.28 |
| replay v2, 600 steps | {'': 2, 'PointInstancer': 1, 'Sphere': 1, 'Mesh': 1, 'Capsule': 21} | 301 / 0.80 | 4.91 / 9.56 |

- One shape set (21 capsules, one colour, translucent) + one opaque mesh; no errors. Figures: `goal1/b2a/shapes_v2_pose_side.png`, `goal1/b2a/shapes_v2_pose_top.png`.

**7. GUI commands (not run)**

```bash
cd ~/effmass_mpc/goal1
# static pose, mesh + v2 shapes, no MPC, robot at retract for scale, viewing placement (b2a/view), closes after 30 s
RUN_ISAAC_NO_DISPLAY=0 scripts/b2a_viz_run.sh v2_pose_gui --pre ~/effmass_mpc/goal1/data/human_b2a_reach_v2.npz --gui --duration 30 --steps 100000
# replay, mesh + v2 shapes, no MPC, g1 placement, closes after 2,523 steps (42 s of replay) or 120 s
RUN_ISAAC_NO_DISPLAY=0 scripts/b2a_viz_run.sh v2_replay_gui --pre ~/effmass_mpc/goal1/data/human_precomputed_1781524848_v2.npz \
  --human-dir ~/effmass_mpc/env_check/step4 --gui --steps 2523 --duration 120
```

- The GUI path of `b2a_viz_check.py` has not been run [not checked]; headless runs of the same script are clean. In the replay command the human frame follows the step count (frame = floor((i+1)/2)), so playback speed follows the render rate.

**8. Open point for the user**: with under-cover priority, the torso capsules are close to spheres (r 0.14-0.17 m) and bulge in front of and behind the body in the side view (static over p95 pelvis 3.6, abdomen 2.5, chest 6.3 cm), and the finger row is a 4.6 cm ball at the fingertips. Over-cover medians stay small because most of those surfaces lie inside the body (inside arms/neck/other torso parts). If natural look should win over the 2 cm under-cover target for these parts, the alternative is r = r_over (pelvis 0.142, abdomen 0.096; chest r_over was 0 under the intrusion rule, so the chest would need its own rule) with under-cover reported; not done.

**9. Files**: `goal1/data/shapes_v2.json`, `human_precomputed_1781524848_v2.npz`, `human_b2a_reach_v2.npz`; scripts `shapes_v2_fit.py`, `shape_metrics.py`, `shapes_eval.py`, `mesh_gap.py`, `g13_human_precompute.py --shapes`, `g1_meff_cpu_check.py` (sum detail); logs `goal1/b2a/shapes_eval.json`, `hand_dir_v1_v2.json`, `goal1/logs/meff_isaac/v2_*`, `b1_massA_s0_v1check*`, `*_mesh_gap.json`, `v2perf_summary.json`; backups `goal1/logs/*.before_v2.py`, first fit try `goal1/logs/shapes_v2_fit.try1.py`.

## B2a - scene g2, lines, IK and offline gate (English, ASCII). Stopped at the gate.

Settings: shapes v2, u to the segment COM (kernel 'com'), set A (eta0 0.10, muref 2.0, p 0.5, eta 0.05-0.20), max_all, human = static precomputed `human_b2a_reach_v2.npz`.

**0. Pre-check**: processes 0, /dev/shm 0.

**1. Placement** (`goal1/scripts/b2a_scene_design.py` -> `goal1/b2a/g2/temp_transform.json`, `lines.json`)
- g1 transform rotated about z (yaw 280.9 deg) so the human faces -x (the robot base at the origin), z offset as g1 (feet on the floor at z -0.9), shifted so the right fingertip (support of the v2 right-hand shapes along the arm axis) is at x 0.50, y 0.00. Robot table and floor as g1.
- Fingertip - base horizontal distance 0.500 m; hand COM height above the base 0.436 m (fingertip z 0.441); shoulder at x 1.230. Arm axis (elbow -> wrist) = [-1.0, -0.0, 0.009] (toward the robot), person's right = +y, so the right thumb is on the -y side and the outward (pinky) side is +y.
- No position adjustment was needed (IK below).

**2. Lines** (gripper TCP = hand link + 0.1034 m along its z; the cuRobo goal link is `fr3v2_1_hand`, so goals are TCP targets moved back by 0.1034 m. Using the flange would put the downward fingers 10 cm into the hand.) Half length 0.20 m each (0.25 m made the top end of the side line unreachable).

| path | travel | line centre (m) | A -> B | gap to v2 hand shapes (m) | nearest direction vs intended (deg) | gap to forearm shape (m, >= 0.08) | gap to torso/neck/head shapes (m, >= 0.35) |
|---|---|---|---|---|---|---|---|
| front | +y | [0.47, -0.0, 0.442] | [0.47, -0.2, 0.442] -> [0.47, 0.2, 0.442] | 0.030 | 1.5 | 0.228 | 0.575 |
| top | +y | [0.546, 0.01, 0.514] | [0.546, -0.19, 0.514] -> [0.546, 0.21, 0.514] | 0.030 | 0.7 | 0.162 | 0.507 |
| side | top -> bottom (-z) | [0.547, 0.096, 0.438] | [0.547, 0.096, 0.638] -> [0.547, 0.096, 0.238] | 0.030 | 0.1 | 0.174 | 0.547 |

- side travels vertically: a line parallel to the common travel direction (y) cannot have its nearest direction along y, so it passes the outward end of the hand from top to bottom.
- Mesh-based gap of the lines to the real hand: front 2.6 cm, top 4.2 cm, side 2.6-2.8 cm (from the Shapes v2 right-hand direction table, same static pose).

**3. IK** (`goal1/scripts/b2a_ik.py`, cuRobo IK, world = table + floor, 21 points per line, narrow joint limits)

| orientation | front success / min margin (rad) | top | side |
|---|---|---|---|
| down [0.0, 1.0, 0.0, 0.0] | 21/21, 0.95 | 21/21, 1.02 | 21/21, 0.64 |
| down_rot90 [0.0, 0.7071067811865476, 0.7071067811865476, 0.0] | 21/21, 0.95 | 21/21, 1.02 | 21/21, 0.64 |

- 'down' = g1 goal orientation (gripper down, gripper width along y); 'down_rot90' = gripper down, width along x (information).

**4. Offline gate** (`goal1/scripts/b2a_gate.py`; along the 21 IK poses of each line, MPC spheres (yml + 0.02) vs v2 right-hand shapes; the closest moment and sphere; u = sphere centre -> hand COM; m_h from the static-pose hand inverse inertia, m_r from that pose)

Orientation down:

| path | IK point | sphere | MPC sphere gap (cm) | TCP gap (cm) | nearest dir vs arm axis (deg) | u vs arm axis (deg) | m_h | m_r | mu | eta A | iso mu | iso eta |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| front | 8 | 59 fr3v2_1_rightfinger | -0.03 | 4.00 | 6.1 | 10.3 | 4.578 | 4.567 | 2.286 | 0.1069 | 0.530 | 0.0515 |
| top | 7 | 59 fr3v2_1_rightfinger | 0.10 | 4.49 | 89.8 | 58.1 | 1.196 | 7.246 | 1.027 | 0.0716 | 0.554 | 0.0526 |
| side | 7 | 56 fr3v2_1_hand | -8.55 | 5.10 | 83.9 | 30.1 | 1.581 | 5.326 | 1.219 | 0.0781 | 0.539 | 0.0519 |

- front - top 3.5 cm (fail), front - side 2.9 cm (fail). No eta at the limits.

Orientation down_rot90:

| path | IK point | sphere | MPC sphere gap (cm) | TCP gap (cm) | nearest dir vs arm axis (deg) | u vs arm axis (deg) | m_h | m_r | mu | eta A | iso mu | iso eta |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| front | 10 | 57 fr3v2_1_leftfinger | -5.06 | 3.00 | 9.8 | 10.2 | 4.723 | 5.257 | 2.488 | 0.1115 | 0.539 | 0.0519 |
| top | 9 | 59 fr3v2_1_rightfinger | 1.57 | 3.06 | 57.3 | 38.4 | 1.987 | 6.485 | 1.521 | 0.0872 | 0.549 | 0.0524 |
| side | 0 | 33 fr3v2_1_link6 | -3.53 | 16.83 | 87.6 | 64.4 | 0.660 | 5.891 | 0.593 | 0.0545 | 0.545 | 0.0522 |

- front - top 2.4 cm (fail), front - side 5.7 cm (fail). No eta at the limits.

- With the g1 orientation ('down'), the front and top passes behave as designed (the nearest robot part is the right finger sphere at the TCP, gap ~0 to the hand shape for the MPC sphere, TCP 4.0 / 4.5 cm). On the side pass the nearest part is a gripper-body sphere (link `fr3v2_1_hand`, sphere 56) that overlaps the top of the hand by 8.6 cm: the gripper spans +-8 cm along y above the TCP, so when the TCP passes the outward side at hand height, the body is over the hand. The side pass therefore does not test the side direction. With 'down_rot90' the side pass is nearest at link6 at the bottom end of the line instead, and the front pass is nearest at a finger sphere 5 cm into the hand shape. A single orientation that keeps the TCP nearest on all three passes was not found [checked: two orientations].

**Alternatives (information only, not applied; same poses and spheres, orientation 'down')** `goal1/scripts/b2a_gate_alt.py`

| u definition / set | front m_h, mu, eta | top m_h, mu, eta | side m_h, mu, eta | front - top (cm) | front - side (cm) |
|---|---|---|---|---|---|
| com/A | 4.58, 2.29, 0.107 | 1.20, 1.03, 0.072 | 1.58, 1.22, 0.078 | 3.5 | 2.9 |
| com/B | 4.58, 2.29, 0.110 | 1.20, 1.03, 0.063 | 1.58, 1.22, 0.071 | 4.7 | 3.9 |
| closest/A | 6.02, 2.61, 0.114 | 0.95, 0.81, 0.064 | 0.95, 0.81, 0.064 | 5.1 | 5.1 |
| closest/B | 6.02, 2.61, 0.121 | 0.95, 0.81, 0.053 | 0.95, 0.81, 0.053 | 6.7 | 6.7 |

- Set B = eta0 0.10, muref 2.0, p 0.7, eta 0.04-0.20 (A-stage candidate B). 'closest' = u to the closest point of the shape (existing kernel switch `--meff-dir closest`).

**5. Decision needed (user)**
- (a) keep the current definition and set A: gate fails (3.5 / 2.9 cm); run anyway with a lower gate, or stop B2a.
- (b) set B with u to the COM: 4.7 / 3.9 cm (fails 6 cm).
- (c) u to the closest point + set B: 6.7 / 6.7 cm (passes), but changes the agreed definition.
- Side pass geometry: needs either a different shared gripper orientation, a different side path, or dropping the side pass (front + top only).

**6. Files**: `goal1/b2a/g2/` (`temp_transform.json`, `lines.json`, `ik.json`, `gate.json`, `gate_alt.json`); scripts `b2a_scene_design.py`, `b2a_ik.py`, `b2a_gate.py`, `b2a_gate_alt.py`. No Isaac MPC run, no GUI command (no runs to show).

## B2a - runs (English, ASCII)

Settings: scene g2 (previous design), lines front / top only (side dropped), TCP lines, half length 0.20 m, travel +y; shapes v2; u = nearest point (new default `--u-def nearest`, COM fallback within 1 mm of the shape axis); set A; max_all; static human `human_b2a_reach_v2.npz`. One run = single pass A -> B (`--scene g2 --scene-json ... --single-pass 1`: start at the IK of A, goal B, no switching). Steps fixed at 138 = 2 x off arrival time (off seed 0 reached B at step 68 on both paths -> 69/60 s = 1.15 s). Pre-check 0/0 before every run (run script refuses otherwise).

**1. u definition change and checks**
- Kernel (`g1_meff_gpu.py`): with `use_com == 0` the nearest-shape u and every per-shape cost u go to the closest axis point (same direction as the closest surface point of a capsule); if that point is within 1 mm of the sphere centre, the segment COM is used and a per-thread fallback count is written (`o_fb`). Torch executed-state queries use the same rule and return a count. m_h = 1/(u' Lambda_seg u) with the shape's segment, m_r, mu, eta unchanged.
- `--u-def com|nearest` (default nearest). `b1_campaign.sh` and `g1_meff_run.sh` now pass `--u-def com`. B1 mass A seed 0 with `--u-def com` and the new code: commands (2,523 steps) and the B1 record bit-equal to the original [checked].
- CPU reference (`g1_meff_cpu_check.py --dir closest`) has the same fallback. Synthetic near-contact set (366,000 samples, mass and iso): shape match 100%, u abs diff <= 3.7e-5, m_h rel diff <= 3.0e-5, weighted cost >= 1 rel diff <= 6.0e-5 (mass) / 9.6e-5 (iso). Isaac rollout dumps (front, mass A, 6 steps, 109,800 samples, 1,406 with cost): shape 100%, u <= 2.4e-6, m_h <= 6.5e-6, eta <= 1.4e-6, weighted cost >= 1 rel diff <= 3.1e-4 (float32 cancellation at eta - d of a few mm); (b,h) sums >= 10: <= 1.2e-4.
- Paired timing (precomputed replay v2, mass A, 1,200 steps, order nearest/com/com/nearest, SM clock 1965 MHz): increase over the 1-2d channel nearest 0.748 ms, com 0.773 ms, nearest - com -0.025 ms (record only).
- Rollout cost aggregation (`--cost-agg 1`): after each MPC step, on the GPU without host sync, the last MPPI iteration's (particle, horizon, sphere) entries with cost > 0 are counted and summed per selected shape (scatter_add), plus the fallback count. Same-seed repeats: counts bit-equal, cost sums differ by <= 1.8e-6 relative (float atomic add order) [checked]; commands unaffected.
- Same-seed repeat (mass A seed 0): front and top commands and B1 records bit-equal [checked].

**2. Conditions**: iso, mass A, uniform 0.10, uniform_matched seeds 0-7 per path; off seeds 0, 1.
- uniform_matched eta = (mass A eta front + mass A eta top) / 2 from the gate table = (0.1143 + 0.0637) / 2 = 0.0890 -> 0.0890 m, same value on both paths.

**3. Metric definitions**
- Shape-based gap: B1 record (MPC sphere surface to v2 shape surface). Mesh-based gap: static node SMPL-H mesh in the scene-g2 placement, MPC sphere radius (yml + 0.02), signed (inside negative).
- Pass window (added because the start pose binds the run-wide minimum in several conditions): steps with the TCP within +-0.12 m of the hand COM along y. Crossing time (added because uniform runs on front did not reach B): first step with TCP y >= 0.17 m. Arrival time = first step within the goal tolerance at B (spec metric).
- Contact risk (relative, m/s): risk_t = P_t * v_n * sqrt(mu / 2.0); P_t = P(mesh gap <= 0) with the hand mesh rigidly translated by an isotropic Gaussian (sigma 0.02 / 0.04 m, 1000 samples, same samples for the same seed number in every condition); v_n = approach speed of the mesh-nearest robot sphere along u (to the closest mesh point), positive only when approaching; mu = nearest-definition mu at that sphere and step. Max and sum over the run. P uses the first-order gap model g_s - n_s . delta per sphere (all spheres within 0.2 m); check against exact sampling on the 8 highest-P steps: off front linear 0.85-0.98 vs exact 0.74-0.92, iso front 0.005-0.007 vs 0.001-0.004, off top 0.027-0.056 vs 0.020-0.054 (the linear model over-estimates; same model in all conditions). No ISO F_max or k is used.

**4. Results per condition** (mean over seeds; [min, max])

| path | cond | arrived | pass-window gap shape (cm) | pass-window gap mesh (cm) | run-wide gap shape / mesh (cm) | crossing time (s) | arrival time (s) | EE path (m) | risk max s0.02 / s0.04 | risk sum s0.02 / s0.04 |
|---|---|---|---|---|---|---|---|---|---|---|
| front | off | 2/2 | -4.43 [-4.53, -4.33] | -2.75 [-3.32, -2.18] | -4.43 / -2.75 | 0.358 [0.300, 0.417] | 1.367 [1.150, 1.583] | 0.431 | 9.16e-01 / 8.69e-01 | 2.63e+00 / 2.85e+00 |
| front | iso | 8/8 | 3.66 [3.24, 3.91] | 5.34 [4.46, 5.86] | 3.66 / 5.34 | 0.454 [0.400, 0.600] | 1.456 [1.183, 1.817] | 0.486 | 7.18e-04 / 2.08e-02 | 2.61e-03 / 1.23e-01 |
| front | massA | 8/8 | 4.55 [4.23, 4.93] | 7.28 [6.96, 7.52] | 4.54 / 6.56 | 0.415 [0.367, 0.450] | 1.581 [1.417, 1.783] | 0.532 | 6.71e-05 / 5.88e-03 | 1.15e-04 / 4.65e-02 |
| front | uni10 | 0/8 | 7.75 [7.31, 8.15] | 9.58 [9.32, 9.77] | 6.67 / 6.60 | 0.450 [0.417, 0.483] | - | 0.562 | 0.00e+00 / 1.50e-03 | 0.00e+00 / 5.58e-03 |
| front | unim | 0/8 | 6.72 [6.52, 6.92] | 8.62 [8.01, 9.03] | 6.64 / 6.60 | 0.444 [0.383, 0.483] | - | 0.543 | 1.34e-05 / 3.28e-03 | 1.34e-05 / 1.29e-02 |
| top | off | 2/2 | 0.97 [0.51, 1.43] | 3.26 [3.17, 3.35] | 0.97 / 3.26 | 0.367 [0.317, 0.417] | 1.375 [1.150, 1.600] | 0.452 | 9.05e-03 / 7.41e-02 | 2.58e-02 / 2.23e-01 |
| top | iso | 8/8 | 3.75 [3.42, 4.14] | 6.03 [5.59, 6.37] | 3.75 / 6.03 | 0.369 [0.333, 0.400] | 1.421 [1.117, 1.733] | 0.475 | 3.25e-04 / 1.25e-02 | 5.43e-04 / 5.01e-02 |
| top | massA | 8/8 | 4.47 [4.10, 4.65] | 7.21 [6.84, 7.54] | 4.47 / 7.19 | 0.396 [0.350, 0.467] | 1.344 [1.117, 1.583] | 0.471 | 8.20e-05 / 6.15e-03 | 8.45e-05 / 2.87e-02 |
| top | uni10 | 1/8 | 7.37 [7.11, 7.73] | 9.75 [9.09, 10.22] | 7.37 / 7.39 | 0.408 [0.367, 0.433] | 1.800 [1.800, 1.800] | 0.508 | 0.00e+00 / 1.20e-03 | 0.00e+00 / 5.62e-03 |
| top | unim | 8/8 | 6.38 [5.87, 6.90] | 8.76 [8.44, 9.22] | 6.38 / 7.39 | 0.392 [0.350, 0.417] | 1.706 [1.300, 2.133] | 0.497 | 6.86e-07 / 2.67e-03 | 6.86e-07 / 1.20e-02 |

Other per-condition values (mean): forearm / torso shape gaps, command change, infeasible, fallbacks, loop time

| path | cond | forearm gap (cm) | torso gap (cm) | dq mean / p95 (rad) | infeasible | nearest-u fallback executed / rollout | loop mean (ms) |
|---|---|---|---|---|---|---|---|
| front | off | 13.7 | 49.3 | 0.0076 / 0.0298 | 0 | 0 / 0 | 17.92 |
| front | iso | 19.4 | 52.8 | 0.0086 / 0.0310 | 0 | 0 / 0 | 17.83 |
| front | massA | 18.1 | 52.6 | 0.0090 / 0.0313 | 0 | 0 / 0 | 18.03 |
| front | uni10 | 21.5 | 52.8 | 0.0098 / 0.0333 | 0 | 0 / 0 | 17.61 |
| front | unim | 21.2 | 52.8 | 0.0095 / 0.0330 | 0 | 0 / 0 | 17.39 |
| top | off | 10.1 | 44.4 | 0.0072 / 0.0250 | 0 | 0 / 0 | 18.57 |
| top | iso | 11.1 | 45.4 | 0.0074 / 0.0250 | 0 | 0 / 0 | 17.50 |
| top | massA | 12.3 | 46.7 | 0.0076 / 0.0251 | 0 | 0 / 0 | 17.86 |
| top | uni10 | 13.0 | 46.4 | 0.0081 / 0.0246 | 0 | 0 / 0 | 17.75 |
| top | unim | 12.4 | 46.2 | 0.0078 / 0.0250 | 0 | 0 / 0 | 17.73 |

- Forearm and torso shapes stay far (no shadowing). No infeasible step, no nearest-u fallback in any run. Loop means include the cost aggregation (record only).

**Values at the shape-based hand minimum** (mean over seeds)

| path | cond | robot sphere | u vs frontal (deg) | m_h | m_r | mu | eta |
|---|---|---|---|---|---|---|---|
| front | off | {'leftfinger': 2} | 24.5 | 4.25 | 6.62 | 2.23 | - |
| front | iso | {'leftfinger': 8} | 55.8 | 1.26 | 7.80 | 1.08 | 0.053 |
| front | massA | {'leftfinger': 8} | 71.1 | 0.95 | 5.83 | 0.82 | 0.064 |
| front | uni10 | {'hand': 8} | 60.2 | 0.87 | 5.48 | 0.75 | 0.100 |
| front | unim | {'hand': 5, 'leftfinger': 3} | 59.9 | 0.97 | 6.20 | 0.84 | 0.089 |
| top | off | {'rightfinger': 1, 'leftfinger': 1} | 107.6 | 1.06 | 4.62 | 0.85 | - |
| top | iso | {'leftfinger': 8} | 108.1 | 1.05 | 4.11 | 0.83 | 0.051 |
| top | massA | {'leftfinger': 8} | 103.7 | 0.99 | 4.34 | 0.81 | 0.064 |
| top | uni10 | {'leftfinger': 8} | 105.9 | 1.01 | 3.98 | 0.80 | 0.100 |
| top | unim | {'leftfinger': 8} | 106.5 | 1.02 | 4.02 | 0.81 | 0.089 |

**5. Paired differences within a path** (mean, positive / negative seeds of 8, two-sided sign test p). Gaps in cm, times in s, risk in relative m/s.

| path | metric | mass A - iso | mass A - uniform 0.10 | mass A - uniform_matched |
|---|---|---|---|---|
| front | win_hand_shape_gap | +0.89 (8+/0-, p 0.008) | -3.20 (0+/8-, p 0.008) | -2.18 (0+/8-, p 0.008) |
| front | win_hand_mesh_gap | +1.95 (8+/0-, p 0.008) | -2.30 (0+/8-, p 0.008) | -1.33 (0+/8-, p 0.008) |
| front | hand_shape_gap | +0.88 (8+/0-, p 0.008) | -2.13 (0+/8-, p 0.008) | -2.10 (0+/8-, p 0.008) |
| front | hand_mesh_gap | +1.22 (8+/0-, p 0.008) | -0.04 (0+/1-, p 1.000) | -0.04 (0+/1-, p 1.000) |
| front | cross_time_s | -0.040 (1+/6-, p 0.125) | -0.035 (0+/7-, p 0.016) | -0.029 (0+/6-, p 0.031) |
| front | pass_time_s | +0.125 (6+/2-, p 0.289) | n/a | n/a |
| front | ee_path_m | +0.046 (8+/0-, p 0.008) | -0.030 (0+/8-, p 0.008) | -0.011 (2+/6-, p 0.289) |
| front | risk_max_s02 | -6.50e-04 (0+/8-, p 0.008) | +6.71e-05 (8+/0-, p 0.008) | +5.38e-05 (7+/1-, p 0.070) |
| front | risk_sum_s02 | -2.49e-03 (0+/8-, p 0.008) | +1.15e-04 (8+/0-, p 0.008) | +1.02e-04 (7+/1-, p 0.070) |
| front | risk_max_s04 | -1.49e-02 (0+/8-, p 0.008) | +4.39e-03 (8+/0-, p 0.008) | +2.60e-03 (8+/0-, p 0.008) |
| front | risk_sum_s04 | -7.64e-02 (0+/8-, p 0.008) | +4.09e-02 (8+/0-, p 0.008) | +3.36e-02 (8+/0-, p 0.008) |
| top | win_hand_shape_gap | +0.72 (8+/0-, p 0.008) | -2.90 (0+/8-, p 0.008) | -1.91 (0+/8-, p 0.008) |
| top | win_hand_mesh_gap | +1.18 (8+/0-, p 0.008) | -2.54 (0+/8-, p 0.008) | -1.54 (0+/8-, p 0.008) |
| top | hand_shape_gap | +0.72 (8+/0-, p 0.008) | -2.90 (0+/8-, p 0.008) | -1.91 (0+/8-, p 0.008) |
| top | hand_mesh_gap | +1.16 (8+/0-, p 0.008) | -0.20 (0+/7-, p 0.016) | -0.20 (0+/7-, p 0.016) |
| top | cross_time_s | +0.027 (7+/0-, p 0.016) | -0.012 (1+/6-, p 0.125) | +0.004 (2+/2-, p 1.000) |
| top | pass_time_s | -0.077 (3+/5-, p 0.727) | -0.300 (0+/1-, p 1.000) | -0.363 (0+/8-, p 0.008) |
| top | ee_path_m | -0.004 (3+/5-, p 0.727) | -0.037 (0+/8-, p 0.008) | -0.026 (1+/7-, p 0.070) |
| top | risk_max_s02 | -2.43e-04 (1+/7-, p 0.070) | +8.20e-05 (8+/0-, p 0.008) | +8.13e-05 (7+/1-, p 0.070) |
| top | risk_sum_s02 | -4.58e-04 (0+/8-, p 0.008) | +8.45e-05 (8+/0-, p 0.008) | +8.38e-05 (7+/1-, p 0.070) |
| top | risk_max_s04 | -6.33e-03 (0+/8-, p 0.008) | +4.95e-03 (8+/0-, p 0.008) | +3.47e-03 (8+/0-, p 0.008) |
| top | risk_sum_s04 | -2.14e-02 (0+/8-, p 0.008) | +2.31e-02 (8+/0-, p 0.008) | +1.68e-02 (8+/0-, p 0.008) |

- pass_time_s (arrival) n/a: the uniform runs did not reach B on front (and 7/8 on top for uniform 0.10).

**6. Interaction** (front mass A - iso) - (top mass A - iso), same seed number paired

| metric | mean | min .. max | positive / negative of 8 | sign p |
|---|---|---|---|---|
| win_hand_shape_gap | +0.17 cm | -0.30 cm .. +0.59 cm | 5 / 3 | 0.727 |
| win_hand_mesh_gap | +0.77 cm | -0.24 cm .. +1.60 cm | 7 / 1 | 0.070 |
| hand_shape_gap | +0.16 cm | -0.30 cm .. +0.59 cm | 5 / 3 | 0.727 |
| hand_mesh_gap | +0.06 cm | -0.62 cm .. +1.24 cm | 4 / 4 | 1.000 |
| cross_time_s | -0.067 s | -0.217 s .. +0.017 s | 1 / 7 | 0.070 |
| risk_max_s02 | -4.08e-04 | -2.07e-03 .. +7.30e-04 | 2 / 6 | 0.289 |
| risk_sum_s02 | -2.03e-03 | -8.94e-03 .. +3.91e-04 | 1 / 7 | 0.070 |

**7. Budget comparison** (front + top) of mass A minus the same of uniform_matched, per seed

| metric | mean | min .. max | positive / negative of 8 | sign p |
|---|---|---|---|---|
| win_hand_shape_gap | -4.09 cm | -4.63 cm .. -3.71 cm | 0 / 8 | 0.008 |
| win_hand_mesh_gap | -2.88 cm | -3.53 cm .. -2.41 cm | 0 / 8 | 0.008 |
| cross_time_s | -0.025 s | -0.067 s .. +0.017 s | 1 / 6 | 0.125 |
| risk_max_s02 | +1.35e-04 | -7.94e-05 .. +3.68e-04 | 7 / 1 | 0.070 |
| risk_sum_s02 | +1.86e-04 | -6.86e-05 .. +4.41e-04 | 7 / 1 | 0.070 |

- Arrival-time budget not computed: uniform_matched never reached B on front.

**8. Rollout cost shapes**: in every path and condition 100% of the (particle, horizon, sphere) entries with cost > 0 selected a right-hand shape (segment 18); no other segment produced cost.

**9. Interpretation [inferred unless marked]**
- The MPC avoids the hand by going over it on both paths (fig 2: TCP rises ~5-12 cm above the line). On front the closest moment is therefore not head-on (u ~71 deg from frontal for mass A, ~56 deg for iso [checked]), where the personal hand mass along u is ~1 kg and the mass-A eta (~0.064 m) is below uniform_matched (0.089 m); this explains why mass A is closer than uniform_matched on front and why the risk ordering is the reverse of the hypothesis.
- mass A vs iso differ consistently on both paths (mass A farther, 8/8 each) because mass A's eta at u ~70-110 deg (m_h ~1 kg) is still above iso's (m_h 0.6 kg); the difference is not specific to the frontal direction in this geometry (interaction weak).
- uniform 0.10 / uniform_matched fail to settle at B on front: B (and A) put the gripper body within ~5-7 cm of the hand, inside a 0.089-0.10 m shell in every direction. mass A and iso settle because their eta is ~0.05-0.06 m sideways. A longer line (larger half length) would remove this confound; not done (settings unchanged).

**10. Figures**: `goal1/b2a/b2a_fig1_hand_gap.png` (pass-window hand gap, shape / mesh, per path and condition, seed lines), `goal1/b2a/b2a_fig2_traj.png` (TCP of mass A and iso seed 0 with the hand, top and side views), `goal1/b2a/b2a_fig3_risk.png` (risk max and sum, sigma 0.02 m, symlog scale).

**11. GUI commands (not run)**: mesh + v2 shapes + top-5 closest-pair lines, closes after 138 steps + 5 s hold. Same options ran headless for front mass A seed 0 without errors and with commands bit-equal to the campaign run.

```bash
cd ~/effmass_mpc/goal1; G=~/effmass_mpc/goal1/b2a/g2
SA="--meff-eta0 0.10 --meff-muref 2.0 --meff-p 0.5 --meff-eta-min 0.05 --meff-eta-max 0.20 --meff-agg max_all"
for P in front top; do for C in mass iso; do
  RUN_ISAAC_NO_DISPLAY=0 scripts/g13_pre_run.sh b2agui_${P}_${C}_s0 --scene g2 --scene-json $G/scene_g2_${P}.json --single-pass 1 --shapes v2 \
    --human-pre ~/effmass_mpc/goal1/data/human_b2a_reach.npz --human-dir $G --human-viz both --meff-viz lines --meff-lines topk --meff-lines-k 5 \
    --u-def nearest --meff on --meff-mode $C $SA --mppi-seed 0 --steps 138 --hold 5 --gui
done; done
```

**12. Files**: scripts `b2a_campaign.sh`, `b2a_analyze.py`, `b2a_summary.py`, `b2a_fig_traj.py`, `b2a_gate.py` (nearest), code changes in `g1_meff_gpu.py`, `g1_meff_cpu_check.py`, `g1_meff_near.py`, `step2b_mpc_isaac.py` (`--u-def`, `--cost-agg`, `--scene-json`, `--single-pass`), `step2_scene.py` (scene g2); data `goal1/b2a/g2/scene_g2_{front,top}.json`, `gate_nearest.json`; logs `goal1/logs/meff_isaac/b2a_*` (`_b1.npz`, `_costagg.npz`, `_b2a_series.npz`), `b2a_runs.json`, `b2a_runs_ext.json`, `b2a_summary.json`, `udperf_*`; backups `goal1/logs/*.before_b2a_unear.py`.

## B2a-2 - longer lines, free and slot scenes (English, ASCII). Stopped before runs.

Settings unchanged: shapes v2, u nearest, set A, max_all, static human, seeds 0-7, iso / mass A / uniform 0.10 / uniform_matched (0.0890 m), off seeds 0, 1.

**0. Pre-check**: processes 0, /dev/shm 0.

**Existing B2a records: which side the robot passed** (`goal1/scripts/b2a_side.py`; at the shape-based hand minimum inside the pass window; side = largest component of the direction from the closest hand-shape surface point to the robot sphere centre: up / down / front (arm axis toward the robot) / side (|y|))

| path | cond | n | side at the closest moment | u vs frontal, mean [min, max] (deg) | mu, mean [min, max] (kg) | TCP max above hand top (cm) |
|---|---|---|---|---|---|---|
| front | off | 2 | front 2 | 24.5 [12.4, 36.7] | 2.23 [1.63, 2.83] | -3.3 |
| front | iso | 8 | front 1, up 7 | 55.8 [46.4, 65.4] | 1.08 [0.90, 1.32] | 4.8 |
| front | massA | 8 | up 8 | 71.3 [65.6, 79.4] | 0.82 [0.74, 0.89] | 7.5 |
| front | uni10 | 8 | up 8 | 57.7 [50.7, 63.6] | 1.02 [0.92, 1.17] | 8.9 |
| front | unim | 8 | up 8 | 58.0 [47.7, 66.1] | 1.01 [0.88, 1.24] | 8.1 |
| top | off | 2 | up 2 | 107.6 [101.7, 113.6] | 0.85 [0.83, 0.88] | 4.0 |
| top | iso | 8 | up 8 | 108.1 [102.4, 115.9] | 0.83 [0.81, 0.88] | 7.0 |
| top | massA | 8 | up 8 | 103.7 [99.9, 107.9] | 0.81 [0.80, 0.81] | 8.5 |
| top | uni10 | 8 | up 8 | 105.9 [99.5, 112.0] | 0.80 [0.78, 0.84] | 11.4 |
| top | unim | 8 | up 8 | 106.6 [101.0, 111.3] | 0.81 [0.79, 0.84] | 10.2 |

- Not only mass A went over the hand: every condition with cost did on front. mass A went over with the most upward u and the lowest mu at the closest moment.

**1. Line length** (`goal1/scripts/b2a2_ik.py`, `b2a2_halflen.py`; IK every 1 cm on the extended lines; rule: robot (all MPC spheres) >= 0.23 m from every v2 hand shape at both end poses, all IK points between the ends succeed)
- front: half length 0.42 m (symmetric) satisfies the rule.
- top: no symmetric half length does: IK fails beyond +0.41 m (dy) and the clearance at |dy| = 0.36-0.41 is 0.19-0.23 m and not monotonic (IK branch changes). The ends dy -0.42 / +0.39 (TCP y -0.41 / +0.40) satisfy the rule, so the top line is asymmetric (total length 0.81 m vs front 0.84 m).

| line | end | TCP (m) | gap to hand shapes (m) | gap to all human shapes (m) | min joint margin (rad, joint) | min joint margin on the line (rad) |
|---|---|---|---|---|---|---|
| front | A | [0.47, -0.42, 0.442] | 0.235 | 0.235 | 1.16 (j6) | 0.95 |
| front | B | [0.47, 0.42, 0.442] | 0.247 | 0.247 | 1.16 (j6) | 0.95 |
| top | A | [0.546, -0.41, 0.514] | 0.231 | 0.231 | 0.60 (j4) | 0.60 |
| top | B | [0.546, 0.4, 0.514] | 0.239 | 0.239 | 0.68 (j4) | 0.60 |

- Pass window (TCP within +-0.12 m of the hand COM along y) and crossing time (TCP y >= 0.17 m) definitions are unchanged.

**2. Slot plates** (`goal1/scripts/b2a2_slots.py` -> `goal1/b2a/g2/slots.json`; cuRobo cuboids, 1 cm thick, axis aligned; only the cuRobo collision cost sees them, not the human cost). Hand: top z 0.4845, bottom z 0.3921, fingertip x 0.500, outward edge y +0.066, back x 0.661.

| scene | plate | centre (m) | size (m) | gap to hand shapes (m, >= 0.22) | gap to all human shapes (m, > 0) | robot gap on the intended line (m, >= 0.025) | robot gap on the route to block (m, < 0 = blocked) |
|---|---|---|---|---|---|---|---|
| slot_front | plate_up | [0.735, -0.012, 0.709] | [0.35, 0.6, 0.01] | 0.2204 | 0.1553 | 0.0292 | -0.0484 |
| slot_front | plate_down | [0.735, -0.012, 0.167] | [0.35, 0.6, 0.01] | 0.2203 | 0.1293 | 0.2550 | 0.3136 |
| slot_top | plate_front | [0.274, -0.012, 0.313] | [0.01, 0.6, 0.282] | 0.2209 | 0.2209 | 0.0337 | 0.1455 |
| slot_top | plate_side | [0.47, 0.291, 0.313] | [0.381, 0.01, 0.282] | 0.2200 | 0.2200 | 0.0310 | 0.0556 |

- Route to block = IK poses of the other line inside the pass window (slot-front: the top line over the hand; slot-top: the front line in front of the fingertip).
- slot-front: with the plates starting at the fingertip, the arm above the TCP on the front line touched the upper plate (-2.8 cm). Scan of the upper-plate start (behind the fingertip): +0.00 -> -0.028 / -0.070 (front line / over-hand route), +0.04 -> +0.010 / -0.068, **+0.06 -> +0.029 / -0.048 (used)**, +0.10 -> +0.069 / -0.008, +0.12 -> +0.089 / +0.012. With +0.06 all rules hold and the over-hand route at TCP 3 cm above the hand is blocked (checked only with those IK poses).
- slot-top: all three rules hold, but the plates do not block the front route (robot on the front line stays 14.5 cm from the front plate and 5.6 cm from the side plate). Scan of the front-plate distance from the fingertip: 0.22 -> +0.145 (front route) / +0.034 (top line), 0.15 -> +0.086 / +0.076, 0.10 -> +0.039 / +0.116, 0.08 -> +0.019 / +0.110, 0.06 -> -0.001 / +0.092. Blocking the front route needs the plate within about 6 cm of the fingertip, which conflicts with the >= 0.22 m rule. **Per instruction: no runs.**

**3. Decision needed (user)**
- slot-top: (a) allow the front plate within ~6 cm of the fingertip (inside the eta shell; then the plate and the hand cost overlap), (b) drop slot-top and use free-top as the top case (in the old records all conditions already passed over the hand on top), or (c) another blocking geometry.
- Ready to run once decided: free-front / free-top (ends above), slot-front (plates above). Steps per scene will be 2 x the off arrival time from an off probe.

**4. Files**: `goal1/scripts/b2a_side.py`, `b2a2_ik.py`, `b2a2_halflen.py`, `b2a2_slots.py`; `goal1/b2a/g2/ik_ext.json`, `slots.json`, `ends_b2a2.json`; `goal1/logs/meff_isaac/b2a_side_old.json`. No Isaac MPC run in this round.

## B2a-2 round 2 - probes and plate check (English, ASCII). Stopped before the main runs.

User decision: slot-top dropped; scenes free-front, free-top, slot-front; ends (front +-0.42 m, top -0.42 / +0.39 m) and slot-front plates (from 6 cm behind the fingertip) approved.

**Code changes** (backups `goal1/logs/*.before_b2a2.py`)
- `env_check/scripts/step2_scene.py`: scene g2 reads an optional `plates` list from the scene json and adds them as cuRobo cuboids (no plates = unchanged world).
- `goal1/scripts/g1_meff_gpu.py`: the cost hook keeps a Python reference to the rollout sphere tensor (no GPU op).
- `env_check/scripts/step2b_mpc_isaac.py`: with `--cost-agg 1` and plates, per step on the last MPPI iteration: count of (particle, horizon, sphere) with sphere-plate gap < 0.025 m (cuRobo activation distance, read back from the solver and asserted) and the minimum gap, per plate; saved in `<tag>_costagg.npz` (`plate_n`, `plate_min`). Rollout sphere radii equal yml + 0.02 (checked).
- New: `goal1/scripts/b2a2_scenes.py` (scene jsons `goal1/b2a/g2/scene_b2a2_{free_front,free_top,slot_front}.json`), `b2a2_campaign.sh`, `b2a2_platecheck.py`.
- Check: slot_front uni10 seed 0 with and without the plate count: joint commands bit-equal (284 steps). Probe and run of the same seed: bit-equal on the common steps.

**Probes (off, seed 0, 600 steps)**: arrival step free_front 124, free_top 115, slot_front 141 -> steps 250 / 232 / 284 (2 x (arrival + 1)). Pre-check before every run: processes 0, /dev/shm 0.

**Plate check** (slot_front vs free_front, off and uniform 0.10, seeds 0 and 1; `goal1/logs/meff_isaac/b2a2_platecheck.json`; side as in `b2a_side.py`; plate gaps for free_front use the slot_front plate boxes as if present)

| run | arrival step | side | u vs frontal (deg) | mu (kg) | TCP at hand minimum (m) | TCP max above hand top (cm) | executed gap to plate_up (m) | rollout steps with plate_up gap < 0.025 m |
|---|---|---|---|---|---|---|---|---|
| free_front off s0 | 124 | up | 108.0 | 0.82 | [0.621, 0.006, 0.450] | -3.2 | -0.058 (no plate) | - |
| free_front off s1 | 167 | up | 125.5 | 1.06 | [0.612, -0.040, 0.471] | -1.3 | -0.069 (no plate) | - |
| free_front uni10 s0 | 137 | up | 113.5 | 0.90 | [0.610, -0.047, 0.573] | 8.8 | -0.070 (no plate) | - |
| free_front uni10 s1 | 154 | up | 105.8 | 0.84 | [0.596, -0.043, 0.570] | 8.6 | -0.070 (no plate) | - |
| slot_front off s0 | 141 | front (TCP under the hand) | 23.8 | 2.20 | [0.627, -0.018, 0.360] | -12.4 | +0.024 | 52 |
| slot_front off s1 | 144 | up | 63.5 | 0.99 | [0.480, 0.014, 0.540] | 5.9 | +0.016 | 188 |
| slot_front uni10 s0 | 148 | up | 70.6 | 0.90 | [0.464, 0.036, 0.606] | 12.3 | +0.013 | 60 |
| slot_front uni10 s1 | 164 | up | 72.5 | 0.87 | [0.460, 0.043, 0.612] | 13.0 | +0.013 | 208 |

**TCP deviation from the line** (max |x - line x| over the run; TCP x inside the pass window)

| run | max deviation (cm) | TCP x in the pass window (m) |
|---|---|---|
| B2a front off s0 / s1 (half length 0.20) | 3.4 / 3.8 | 0.491-0.504 / 0.493-0.508 |
| B2a top off s0 | 2.5 | 0.561-0.571 |
| B2a-2 free_front off s0 / s1 (0.42) | 15.1 / 14.3 | 0.610-0.621 / 0.600-0.613 |
| B2a-2 free_front uni10 s0 | 14.3 | 0.600-0.613 |
| B2a-2 slot_front off s0 | 15.7 | 0.619-0.627 |
| B2a-2 free_top off s0 (probe) | 12.5 | 0.663-0.671 |

Line x: front 0.470, top 0.546. Fingertip x 0.500, hand COM x 0.596.

**Findings (unexpected; stopped per the standing rule)**
1. With the longer lines the MPC path bulges about 14-15 cm toward the person (+x). On free_front the off TCP crosses at x 0.61, at hand height, inside the hand (shape gap -3.1 / -1.9 cm). The "front of the fingertip" route of the design (line x 0.47) is no longer what the robot does; free_front becomes "through the hand unless lifted". With uniform 0.10 the robot goes over the hand at x 0.60-0.61 (8.7 cm above the hand top).
2. The slot_front plates change the behaviour, but do not make a front route: the upper plate (from x 0.56) pushes the path back to x 0.46-0.48, and from there uniform 0.10 (2/2) and off s1 go over the fingertip with the TCP 6-13 cm above the hand top, under the plate front edge. off s0 went under the hand. The executed arm stays 1.3-2.4 cm from the upper plate (inside the 2.5 cm activation), and rollouts touch the upper plate in 52-208 of 284 steps: the plate cost and the human cost act together on the same pass.
3. With the plates >= 0.22 m from the hand, there is room for the arm to pass over the fingertip below the upper plate (TCP up to 13 cm above the hand top). The offline check blocked only the top-line IK poses (TCP 3 cm above the hand at x 0.546), not this route.

**Not run**: main runs (3 scenes x 4 conditions x 8 seeds + off). Done: 3 probes, 8 plate-check runs, 1 recording check.

**Decision needed (user)**
- free scenes: run as is (the robot's own path, which on front goes over the hand body at x 0.60), or keep the TCP near the line (for example an intermediate goal or a shorter line; the latter conflicts with the 0.23 m end rule).
- slot_front: as is (plate and human cost mixed; front route not forced), drop it, or change the plates (for example extend the upper plate forward past the fingertip, which needs a smaller plate-hand gap than 0.22 m to block the over-fingertip route).

## B2a-3 - approaching hand, robot holding a pose (English, ASCII). Withdrawn by the user during the runs.

- Done before withdrawal: hold pose IK (`goal1/scripts/b2a3_ik.py` -> `b2a/g2/b2a3_ik.json`; TCP (0.47, 0.00, 0.436), gripper down, min joint margin 0.94 rad at j4), design and offline gate (`b2a3_design.py` -> `b2a/g2/b2a3_design.json`, `scene_b2a3.json`), precomputed moving-hand files (`data/human_b2a3_{front,side}_v2.npz`, `_verts.npz`; rigid translation, 0.2 m/s, finite-difference speed vs file velocity <= 3.5e-6 m/s), `b2a3_campaign.sh` (constant-velocity human prediction).
- Offline gate (gap 3 cm before the stop): front mass A eta 0.1026 vs iso 0.0533 (+4.93 cm, pass); side 0.0590 vs 0.0514; eta_matched would have been 0.0808 m.
- Runs: 1 test run (front mass A seed 0) and 3 campaign runs (front off s0, s1, iso s0, the last one stopped) exist in `logs/meff_isaac/b2a3_*`; not analysed. The stopped Isaac left 3 /dev/shm entries (own process, removed). No change to the core code in B2a-3.

## B2a-4 - calibrated round-trip lines (English, ASCII). Stopped at the calibration.

Settings: shapes v2, u nearest, set A, max_all, static B2a pose, end rule (all MPC spheres >= 0.23 m from the hand shapes at both end poses), motion +y, single pass A -> B.

**Tools**: `goal1/scripts/b2a4_ik.py` (IK grid: front lines x 0.20-0.50 at the front line height z 0.4416, plus x 0.315; top lines at x 0.546, z 0.50-0.75; y = centre +-0.60, 1 cm) -> `b2a/g2/b2a4_ik.json`; `b2a4_scene.py` (ends by the 0.23 m rule, smallest symmetric half length, else per side) -> `scene_b2a4_<path>_<key>.json`; `b2a4_campaign.sh`; `b2a4_cal.py` (measures from the B1 rows: minimum shape-based hand gap over the run, side at that step, u nearest vs frontal, mu, TCP height over the hand top in the pass window) -> `logs/meff_isaac/b2a4_cal/`; `b2a4_switch.py` (offline straight-line check). Pre-check before every run: processes 0, /dev/shm 0.

**front-cal (off, seed 0, 400 steps; target: front side, min gap 3.0 +- 0.5 cm, u <= 30 deg)**

| try | line x (m) | half length (m) | end gaps (m) | min gap (cm) | nearest sphere | side | u vs frontal (deg) | mu (kg) | TCP max over hand top (cm) | TCP x in pass window (m) | max x deviation (cm) | arrival step |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 0.35 | 0.35 | 0.252 / 0.236 | -2.34 | leftfinger | front | 19.5 | 2.34 | -2.9 | 0.464-0.475 | 12.5 | 101 |
| 2 | 0.30 | 0.29 | 0.241 / 0.230 | 5.21 | hand | front | 31.5 | 2.04 | -3.3 | 0.378-0.392 | 9.2 | 94 |
| 3 | 0.31 | 0.31 | 0.247 / 0.235 | 2.99 | hand | front | 38.4 | 1.67 | -3.3 | 0.402-0.418 | 10.8 | 103 |
| 4 | 0.32 | 0.32 | 0.248 / 0.234 | 2.00 | leftfinger | front | 12.0 | 2.95 | -2.9 | 0.418-0.429 | 10.9 | 103 |
| 5 | 0.315 | 0.31 | 0.244 / 0.231 | 3.62 | hand | front | 38.8 | 1.63 | -2.7 | 0.404-0.414 | 9.9 | 98 |

- Not met: when the minimum gap is in 2.5-3.5 cm (tries 3, 5) the nearest robot part is the gripper body (sphere on `fr3v2_1_hand`, above the fingers) and u is 38-39 deg; when the fingers are nearest (tries 1, 4) u is 12-20 deg but the gap is 2.0 cm or negative. The gap is not monotonic in x (0.31 -> 3.0 cm, 0.315 -> 3.6 cm, 0.32 -> 2.0 cm): x changes the start IK and the half length, and the path with them.
- Cause (checked): at the closest moment of the off pass the gripper is rotated 29-33 deg about the vertical from the goal orientation (tilt from vertical only 1.8-2.6 deg). On the static IK poses of straight lines (goal orientation) the fingers are nearest down to a 0.9 cm gap with u 1-3 deg (`b2a4_switch.py`, TCP x 0.39-0.46). The orientation error during the fast off pass (TCP about 1.4 m/s at the hand) brings the gripper body toward the hand.
- Seed 1 re-check, arrival and gate: not done.

**top-cal (target: over the back of the hand, min gap 3.0 +- 0.5 cm, u >= 70 deg)**
- Existing B2a-2 top line (x 0.546, z 0.514, ends dy -0.42 / +0.39, end gaps 0.231 / 0.239) off probe seed 0: min gap 5.19 cm, side up, u 96.9 deg, mu 0.84, TCP x 0.663-0.671 at the hand -> gap 2.2 cm too large; the line must go down by about 2 cm.
- Lower lines do not meet the end rule: on the IK grid at x 0.546 the reachable ends give at most 0.228 m (z 0.51) and 0.223 m (z 0.50); z 0.52-0.60 also stay below 0.23 m (0.20-0.22 m). The end gap is set by the gripper sphere at |dy| about 0.4 m and by link 4 near the reach limit; IK fails beyond |dy| 0.41-0.45 m.
- No top-cal run.

**Decision needed (user)**
- front-cal: (a) accept the gripper-body contact with u <= 40 deg (try 3: x 0.31, 3.0 cm, 38 deg); (b) accept the finger contact with a 2.0 cm gap (try 4: x 0.32, 12 deg); (c) also adjust the line height or slow the pass; (d) another approach.
- top-cal: (a) relax the end rule for top (about 0.22 m); (b) allow x as well as height; (c) keep z 0.514 (gap 5.2 cm).

## B2a-4 round 2 - calibration decision, seed 1 check, gate (English, ASCII). Stopped at the gate.

User decision: front-cal = try 3 (x 0.31 m), u criterion relaxed to <= 40 deg; top-cal = existing line z 0.514 m (ends dy -0.42 / +0.39, end gaps 0.231 / 0.239 m; scene identical to `scene_b2a2_free_top.json`).

**Seed 1 check (front-cal, off, 400 steps)**: side front, min gap 3.67 cm (within 2-4 cm), nearest sphere leftfinger (seed 0: gripper body), u 22.7 deg (seed 0: 38.4), mu 2.43 kg (seed 0: 1.67), TCP at the minimum (0.413, 0.044, 0.480) (seed 0: (0.412, 0.067, 0.451)), arrival step 135 (seed 0: 103), infeasible 0.

**Gate** (`goal1/scripts/b2a4_gate.py` -> `b2a/g2/b2a4_gate.json`; executed q at the closest moment of the seed 0 off run, nearest robot sphere, u nearest)

| scene | sphere | gap (cm) | u vs frontal (deg) | m_h (kg) | m_r (kg) | mu (kg) | sqrt(mu/2) | eta mass A (m) | eta iso (m) | mass A - iso (cm) |
|---|---|---|---|---|---|---|---|---|---|---|
| front-cal (x 0.31, s0) | fr3v2_1_hand | 2.99 | 38.4 | 2.02 | 9.50 | 1.667 | 0.913 | 0.0913 | 0.0531 | **3.82 (< 4, fail)** |
| top-cal (z 0.514, s0) | rightfinger | 5.19 | 96.9 | 0.96 | 6.47 | 0.839 | 0.648 | 0.0648 | 0.0524 | 1.24 |

- uniform_matched would be 0.0780 m (mean of the two mass A eta). **Per instruction: no runs.**

**Offline IK only: line height +-2 cm at x 0.31** (`b2a4_height.py` -> `b2a/g2/b2a4_height.json`). Assumption (not run): the off pass moves rigidly with the line height, so the closest-moment hand-link pose is shifted by dz; damped least squares IK from the executed q (pose error <= 8e-6 m / 8e-6 rad). "executed" keeps the executed orientation (yaw error about 32 deg in s0, 35 deg in s1); "goal" uses the goal orientation (gripper down, no yaw error).

| seed | orientation | dz (cm) | gap (cm) | nearest sphere | u (deg) | m_h | m_r | mu | eta A (m) | eta iso (m) | A - iso (cm) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | executed | -2 | 1.85 | hand | 30.0 | 2.74 | 10.30 | 2.163 | 0.1040 | 0.0532 | 5.08 |
| 0 | executed | -1 | 2.39 | hand | 34.4 | 2.32 | 10.00 | 1.882 | 0.0970 | 0.0532 | 4.38 |
| 0 | executed | 0 | 2.99 | hand | 38.4 | 2.02 | 9.50 | 1.667 | 0.0913 | 0.0531 | 3.82 |
| 0 | executed | +1 | 3.47 | leftfinger | 14.5 | 4.50 | 6.05 | 2.581 | 0.1136 | 0.0522 | 6.14 |
| 0 | executed | +2 | 3.75 | leftfinger | 19.2 | 3.75 | 6.66 | 2.400 | 0.1096 | 0.0525 | 5.71 |
| 0 | goal | -2 | 5.60 | hand | 20.8 | 4.20 | 8.03 | 2.759 | 0.1174 | 0.0528 | 6.46 |
| 0 | goal | -1 | 5.82 | leftfinger | 5.3 | 6.17 | 4.55 | 2.618 | 0.1144 | 0.0515 | 6.29 |
| 0 | goal | 0 | 5.89 | leftfinger | 7.7 | 5.73 | 4.87 | 2.634 | 0.1148 | 0.0517 | 6.31 |
| 0 | goal | +1 | 6.03 | leftfinger | 11.2 | 5.11 | 5.23 | 2.583 | 0.1136 | 0.0519 | 6.18 |
| 0 | goal | +2 | 6.24 | leftfinger | 14.9 | 4.44 | 5.59 | 2.476 | 0.1113 | 0.0521 | 5.92 |
| 1 | executed | -2 | 3.04 | leftfinger | 12.9 | 5.68 | 6.02 | 2.923 | 0.1209 | 0.0522 | 6.87 |
| 1 | executed | -1 | 3.32 | leftfinger | 17.9 | 4.53 | 6.65 | 2.696 | 0.1161 | 0.0525 | 6.36 |
| 1 | executed | 0 | 3.67 | leftfinger | 22.7 | 3.66 | 7.24 | 2.430 | 0.1102 | 0.0526 | 5.76 |
| 1 | executed | +1 | 4.10 | leftfinger | 27.2 | 3.02 | 7.73 | 2.174 | 0.1043 | 0.0528 | 5.15 |
| 1 | executed | +2 | 4.60 | leftfinger | 31.4 | 2.57 | 8.09 | 1.950 | 0.0987 | 0.0528 | 4.59 |

- The gate fails only for the seed 0 pose with the gripper body nearest; the same line passes it on the seed 1 pose (5.76 cm). eta iso stays 0.052-0.053 m (mu_iso is set by the 0.6 kg hand); the difference follows m_h, which drops when u tilts upward (gripper body above the hand).
- In s0, lowering by 1-2 cm keeps the gripper body nearest but u 30-34 deg and passes (4.4-5.1 cm), with the gap 1.9-2.4 cm; raising by 1-2 cm switches to the finger (6.1 / 5.7 cm). Without the yaw error every case passes (5.3-7.0 cm).

**Recorded (no criterion)**: TCP speed inside the pass window: front-cal s0 max 1.74 / mean 1.68 m/s, s1 1.80 / 1.73; top-cal s0 (B2a-2 probe) 2.14 / 2.08. Gripper rotation about the vertical vs the goal at the closest moment: 31.7 deg (front s0), 34.7 deg (front s1), 22.9 deg (top s0).

**Not done**: arrival-based step counts, main runs, analysis, figures, GUI command.

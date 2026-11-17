# Overnight report (2026-11-17 to 2026-11-18)

## 첫 쪽 (05:40 작성)

### 1. 한 것과 못 한 것
| 묶음 | 상태 | 실행 | 시각 |
|---|---|---|---|
| 0 준비 | 완료. 기준 실행이 V2 기록과 비트 단위로 같음 | 7 (비트 재확인 4 포함) | 23:18~23:25 |
| 1 빠른 손 원인 | 완료 | 28 | 23:25~00:20 |
| 2 자세별 구역 지도 | 완료 (자세 7개) | 0 | 00:16~00:32 |
| 3 정지 장면 모음 | 부분 완료: 12개 중 9개, 3개 제외 (D3-1~3) | 65 | 00:30~01:10 |
| 4 움직이는 협업 장면 | 완료 (건네기, 가로지르기) | 28 | 01:10~01:52 |
| 5 인지 불완전성 | 완료 (지연, 떨림, 누락 + 확장 단계) | 26 | 01:52~02:26 |
| 6 튜닝 | 완료. 규칙상 쓸 만한 후보 없음 | 24 | 02:25~02:38 |
| 7 정리 | 완료 (영상 3개, 문서, 커밋) | 표시용 8 | 03:39~05:40 |
| 예비 1 seed 2 추가 | 묶음 3·4 안에서 함 | - | |
| 예비 2 이득 손잡이 | 완료 | 12 | 02:38~02:44 |
| 예비 3 굽힌 팔 M3 | 완료 | 6 | 02:44~02:55 |
| 예비 4 ROS 재생 경로 | 하지 않음 (기존 스크립트로는 R1 동작을 재생할 수 없음) | 0 | |
| 후속 F1~F8 | 완료 | 64 | 02:57~05:26 |
합계 268회 실행, 시간 초과와 재시도 0. 분석한 실행 253회(표시용 녹화·묶음 0 제외) 모두 실시간 비율 1.0 이상이다. 최저는 묶음 3 단일 패스 1회의 1.02(P2_tipfront 균일 seed 2)이고, 나머지는 1.136 이상이다. infeasible 0, recapture 0. 침투가 생긴 실행은 V100 진단 변형, 떨림 + 무필터 예측, 지연 + 떨림 1.0 m/s 정지뿐이다(아래 표).

### 2. 핵심 결과
1. **빠른 손의 간격 부족은 예측이 없어서다.** 1.0 m/s 에서 손이 뻗는 중 로봇이 지나갈 때 제안의 최대 간격 부족(η − 간격)은 11.7~13.8 cm 다. 등속 예측(--meff-motion cv)을 쓰면 4.3~5.0 cm 로 줄었다(seed 3개). 속도 상한을 0.75 / 1.0 m/s 로 올려도 12.3 cm 그대로였다. 근거: `logs/meff_isaac/on_wp1_summary.json`, `goal1/overnight/wp1_phase_gap.png`.
2. **제안이 더 멀리 가는 곳은 손끝 앞과 전완 옆이다.** 정지 장면 9개(seed 3개)에서 손끝 앞 4개 자세 모두 +2.7~+5.6 cm, 옆 팔 자세의 손 위 +3.7 cm, 전완 옆(P6) +5.6 cm 였다. 굽힌 팔의 손 위 3개는 차이가 없었다. 그 방향 μ 가 0.54~0.59 kg 로 μ_ref 근처라 정상이다. 경로가 바뀐 장면과 도달 실패는 없다. 근거: `on_wp3_summary.json`, `wp3_effect_table.png`.
3. **협업 동작에서도 제안이 더 멀리 간다.** 건네기 머묾 간격은 10.1 대 5.8~5.9 cm, 최소 간격은 seed 마다 제안이 위다(seed 3개, 0.25·0.5 m/s). 가로지르기 0.5 m/s 최소 간격은 3.8~6.7 대 0.0~1.2 cm 다. 대신 0.25 m/s 에서 제안은 쉬고 있는 손 앞에서 더 기다려 통과 수가 12~14 대 16~18 이다. 침투는 0 이다. 근거: `on_wp4_summary.json`.
4. **인지 불완전성은 지연이 가장 문제다.** 지연 300 ms, 떨림 4 cm, 누락 60% 까지 침투는 0 이었다. 지연 100 ms 부터 제안의 간격 부족이 커졌다(10.6~11.1 대 기준 6.1~7.8 cm, seed 2개). 떨림은 주로 명령 저크를 키웠다. 근거: `on_wp5_summary.json`.
5. **실시간 Kinect 설정 후보: 등속 예측 + 속도 필터(EMA 0.2) + 속도 상한 반경 0.5 m (t2a_c2).** 필터 없는 예측은 떨림이 있으면 위험하다. 떨림 4 cm 에서 균일이 손을 62 스텝 침투했다. 필터를 넣으면 침투가 0 이 된다. 지연 100 ms + 떨림 2 cm 에서 이 후보는 정지보다 간격이 크고(0.5 m/s 제안 7.2~7.7 대 3.9~4.8 cm), 사람 근처 속도 p99 는 0.467~0.478 m/s 다. 깨끗한 데이터와 협업 동작에서도 손해가 없었다(seed 2개). 근거: REPORT 의 F2~F8 절, `on_wp1_var_runs.json`, `on_wp5_runs.json`.

### 3. 스스로 내린 결정
| 번호 | 원래 기준 | 판정 | 바꾼 내용 | 이유 |
|---|---|---|---|---|
| D1-1 | 위상 = 로봇이 손 앞을 지나는 시각 − 손이 다 뻗은 시각 | - | 로봇 TCP 가 머묾 자세 손끝 위치를 지나는 시각으로 정의 | 간격 자체에 기대지 않는 정의 |
| D1-2 | 변형 L·P·V, 합친 변형 | L02 가 사람 근처 속도 상한을 넘김 | P+L02 조합과 P 의 seed 1 추가 | 단일 seed 결과 보강 |
| D2-1 | 형상끼리 겹치지 않을 것 | 기준 자세까지 모두 불통과 | 어깨 붙은 위팔-가슴 쌍 제외, 손목을 비틂·굽힘으로 나눔, P3·P6 위팔 14도 벌림 | 측정 방법의 문제 (나) |
| D2-1 | P6 손바닥 위 | 30도 남음 | 가중치 2회 시도 후 0.5 유지 | 더 올리면 팔꿈치가 펴짐 |
| D3-1~3 | 몸통 거리 0.35 m, 12장면 | 5장면만 가능 | 사용자 지시 0.25 m, 추가 규칙에 따라 0.18 m 까지 완화, 사람이 경로 바깥쪽에 서는 조건 추가 | 전완 옆 방향 확보. 몸통 비용은 오프라인으로 확인(P6 손 위만 있음) |
| D4-1, D4-2 | (묶음 4 에 몸통 기준 없음, 내 출발값 0.35 m) | 건네기·가로지르기 불가 | 0.25 m, 가로지르기 각도 40도 | 동작 2개 결과 확보 |
| D5-1 | 지연 50~150 ms, 떨림 1~2 cm, 누락 30% | 모두 견딤 (하한만 앎) | 지연 300 ms, 떨림 4 cm, 누락 60% 추가 | 견디는 수준 확보 |
| D6-0, D6-1 | 개선 폭 > seed 차이 | c1 은 0.015 < 0.031 이라 불통과 | tuned_t2a.yml 로 저장, 채택 안 함 | 규칙 그대로 적용 |
| D7-1 | V2 카메라 | M1 에서 사람이 로봇을 가림 | M1 만 반대편 카메라로 다시 녹화 | 보이는 영상 |
| F1~F8 | - | - | 재생 옵션 추가: 앞당김, 지연, 떨림, 누락, 속도 필터. 모두 기본 꺼짐이고 기준 실행 비트 일치를 3번 확인 | 후속 질문 |

### 4. 내 결정이 필요한 것
- tuned_t2a.yml(속도 상한 가중치 1e6)의 채택 여부. 규칙상 개선은 아니지만 must 위반 2건이 0건이 됐다.
- Kinect 단계 설정: 등속 예측 + 속도 필터 + 반경 0.5 m(t2a_c2)를 쓸지. 필터 계수 0.2 는 재생 데이터 기준이라 실데이터로 다시 정해야 한다.
- 묶음 3 의 몸통 거리 완화(0.25 m 는 지시, 0.18 m 는 내 판단)를 받아들일지. 전완 옆은 P6 한 자세뿐이다.
- 예비 4(ROS 경로로 R1 동작 재생)를 하려면 g1_meff_run.sh 의 재생 파일과 장면 인자를 바꿔야 한다. 하지 않았다.
- 가로지르기 0.25 m/s 에서 제안이 손 앞에서 더 기다리는 것(통과 수 −25%)을 허용할지.

### 5. 눈으로 확인할 것 (순서대로)
1. `goal1/overnight/postures_preview.png`
2. `goal1/overnight/wp3_effect_table.png`, `goal1/overnight/wp3_scene_gap.png`
3. `goal1/b2a/demo/overnight/on_static_P4_tipfront.mp4` (8.5 s, 한 번 지나감)
4. `goal1/b2a/demo/overnight/on_M1_handover.mp4` (21.5 s)
5. `goal1/b2a/demo/overnight/on_M2_crossing.mp4` (21.5 s)
6. `goal1/overnight/wp1_phase_gap.png`
7. `goal1/overnight/wp4_phase_gap.png`
8. `goal1/overnight/wp5_perception.png`
9. `goal1/overnight/wp2_zone_thickness.png`

### 6. 미해결과 (해석)
- 미해결: 1.0 m/s 손 + 지연 100 ms 에서 제안이 균일보다 낫다는 것은 확인되지 않았다(최소 간격 1.9~5.0 대 0.6~6.2 cm, seed 2개). 간격과 속도 상한을 함께 지키는 설정도 찾지 못했다.
- 미해결: 전완 옆 장면은 P6 하나뿐이다. 굽힌 팔 P2·P3 와 옆 팔 P4 의 전완 옆은 직선 경로에서 몸통이 0.09 m 안까지 와서 만들지 못했다.
- 미해결: 단일 패스에서 균일의 사람 근처 속도 p99 가 0.546 / 0.518 m/s 로 한도를 넘은 2건. 왕복에서는 넘지 않았다.
- (해석) 예측 후 남는 1.0 m/s 간격 부족 약 5 cm 는 머묾에서도 보이는 정상 상태 부족(간격/η 약 0.85)과 0.5 m/s 로 비키는 시간 때문으로 보인다.
- (해석) 전완 옆에서 제안의 간격이 η 보다 큰 것(간격/η 1.48)은 더 무거운 손의 구역이 먼저 로봇을 막기 때문으로 보인다.
- (해석) 가로지르기 0.25 m/s 에서 제안이 더 기다리는 것은 손이 경로 안에 쉬고 있어 큰 구역이 비켜 지나가기를 막기 때문으로 보인다.
- (해석) 지연 + 떨림 + 예측에서 속도 p99 가 오르는 것은 속도 상한의 거리 판정이 실제 손과 다른 예측 형상을 쓰기 때문으로 보인다.
- 단일 seed 결과: 묶음 5 의 대부분 변형, F3, F5.


---
# 묶음별 상세 (English, written as each package finished)


## WP0 Preparation (23:18-23:25)
- Deleted the 6 frame / recording folders (r1/rec, r1/frames_r1_main, r1/frames_r1_fast, v2/rec, v2/frames_v2_main, v2/frames_v2_gain_knob, about 5.1 GB) after
  ffprobe (r1_main/r1_fast 645 frames, v2_main 236, v2_gain_knob 172, all 1920x1080 h264) and the existing bit checks (r1_video_bitcheck.json, v2_video_bitcheck.json, all true).
- Local commit 2927641 "r1: moving hand replay" (R1 scripts, R1 summary json, R1 figures, .gitignore widened for R1 code, summary json, figures and overnight notes).
- Regression baseline: tonight's runs reproduce the V2 records bit for bit (uniform s0 and ours s0, 304 steps), and an R1 60 s round trip bit for bit (3600 steps).
  Joint commands do not depend on B1 logging or cost aggregation (V2 records had both on, tonight's runs have both off).
- Wall time: single pass 27.4 s, 60 s round trip 75.6 s. All planned run counts fit; seeds were not reduced (STATUS.md).

## WP1 Why the zone collapses for a fast hand (23:25-00:20, 28 runs, all RTF >= 1.149, recapture 0, infeasible 0)
Question: at 1.0 m/s ours dipped to ~3.5 cm while eta was 16.6 cm. (a) no human motion prediction, (b) the 0.5 m/s near-human speed cap, (c) other.

Method: R1 passes re-grouped by crossing phase = time the TCP passes the fingertip line minus hold onset (D1-1). Critical window = robot crosses
while the hand is still reaching or has just arrived ([-T_reach, +0.25 s]). Deficit = eta - gap at the pass minimum. Variants on the R1 scene, seed 0,
0.5 and 1.0 m/s, uniform and ours: L01 / L02 (shapes moved ahead by file velocity x 0.1 / 0.2 s, replay side), P (existing --meff-motion cv,
constant-velocity shapes over the horizon), V075 / V100 (diagnostic profiles tuned_d1v075 / tuned_d1v100, speed cap 0.75 / 1.0 m/s), P+L02; P repeated with seed 1.

R1 by phase (seed 0-2): the worst gaps sit in the critical window. At 1.0 m/s ours max deficit 11.7-13.8 cm (3 seeds), uniform 4.9-6.8 cm.
In the hold window the gap is steady (uniform 5.7-6.0, ours 10.5-11.6 cm). Seed spread of the critical metrics: 0.1-3.3 cm.

Critical-window results (cm; min gap / max deficit; seed 0 unless noted):
| speed / cond | base (s0-s2) | L01 | L02 | P (s0, s1) | V075 | V100 | P+L02 |
|---|---|---|---|---|---|---|---|
| 0.5 uniform | 1.6-3.7 / 2.8-4.9 | 4.2 / 2.3 | 5.7 / 0.8 | 6.6, 6.5 / -0.1, -0.0 | 3.5 / 3.0 | -0.1 / 6.6 | 8.0 / -1.5 |
| 0.5 ours | 5.4-6.4 / 6.1-7.8 | 8.4 / 4.7 | 8.7 / 3.1 | 9.3, 8.8 / 3.3, 2.9 | 6.0 / 7.2 | 4.2 / 8.0 | 10.5 / 2.2 |
| 1.0 uniform | -0.3-1.6 / 4.9-6.8 | 3.0 / 3.5 | 6.5 / 0.0 | 4.7, 5.6 / 1.8, 0.9 | 0.6 / 5.9 | -0.4 / 6.9 | 6.4 / 0.1 |
| 1.0 ours | 2.0-2.2 / 11.7-13.8 | 4.7 / 9.1 | 7.5 / 3.5 | 6.1, 6.1 / 5.0, 4.7 | 1.9 / 12.3 | 0.9 / 12.3 | 7.8 / 0.8 |
Near-human robot speed p99 (true human): base, L01, P 0.47-0.48 m/s; L02 at 1.0 m/s 0.66 / 0.76; P+L02 at 1.0 m/s 1.04 / 0.95; V075 0.73; V100 0.98-1.00.
Hold-window gap unchanged by P (ours 10.5-10.6, uniform 5.7-5.9). Shape penetration only with V100 (uniform 1 step at 0.5 m/s, 6 steps at 1.0 m/s).

Conclusion:
- (a) Missing prediction is the main cause. P lowers ours max deficit at 1.0 m/s from 11.7-13.8 to 4.7-5.0 cm (2 seeds, both beyond the seed spread)
  and raises the critical min gap from 2.0-2.2 to 6.1 cm, with the speed cap intact. Lead L02 helps as much but breaks the speed cap at 1.0 m/s.
- (b) The speed cap is not the cause: raising it to 0.75 / 1.0 m/s does not reduce the deficit (12.3 cm) and lowers the minimum gaps.
- (c) Remaining deficit with P at 1.0 m/s: ~5 cm for ours, ~1-2 cm for uniform. (interpretation) the residual is the steady-state shortfall seen
  in the hold window (ours gap/eta ~0.85) plus the time the robot needs to move away at <= 0.5 m/s; not checked.
Required extra margin (= critical-window max deficit, cm), base -> P: uniform 0.5 m/s 2.8-4.9 -> ~0, 1.0 m/s 4.9-6.8 -> 0.9-1.8;
ours 0.5 m/s 6.1-7.8 -> 2.9-3.3, 1.0 m/s 11.7-13.8 -> 4.7-5.0. (0.25 m/s base only: uniform 2.4-3.3, ours 5.1-7.0; P not run.)
Candidate for the live Kinect stage: --meff-motion cv (P), lead 0, speed cap 0.5 m/s kept. Lead (L) is not recommended because it lets the robot
exceed the speed cap near the true hand at 1.0 m/s.
Figure: goal1/overnight/wp1_phase_gap.png. Numbers: logs/meff_isaac/on_wp1_summary.json, on_wp1_r1_passes.json, on_wp1_var_passes.json, on_wp1_var_runs.json.

## WP2 Zone maps by posture and direction (00:16-00:32, no MPC)
Seven right-arm postures from the B2a pose (shoulder 3 + elbow hinge + wrist 3 optimised with the node body model; shapes and inertia recomputed
with g13_human_precompute.py). All pass the checks after the fixes in D2-1. Preview: goal1/overnight/postures_preview.png (first figure to look at).
Ours zone thickness by direction (cm; uniform is 6.5-10.2 cm everywhere, the upper value where shapes overlap); mu ranges (kg, robot 4.6 kg isotropic):
| posture | elbow deg | tip front | hand back | palm | hand inner | hand outer | forearm top | forearm side | elbow | upper-arm side | hand mu | forearm mu | upper-arm mu |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| P1_reach (ref) | 5 | 20.3 | 9.1 | 8.8 | 13.3 | 6.5 | 14.8 | 12.7 | 24.6 | 20.6 | 0.52-2.99 | 1.36-2.87 | 2.90-3.28 |
| P2_bend45 | 42 | 12.4 | 7.3 | 7.2 | 11.3 | 6.5 | 17.2 | 13.4 | 23.4 | 18.2 | 0.51-1.55 | 1.45-2.88 | 2.48-3.30 |
| P3_bend90 | 89 | 9.5 | 7.0 | 8.1 | 6.6 | 10.8 | 37.3 | 13.9 | 21.9 | 19.1 | 0.50-1.08 | 1.46-3.06 | 2.35-3.27 |
| P4_side | 4 | 21.1 | 9.3 | 8.8 | 6.5 | 13.4 | 15.9 | 14.1 | 24.9 | 19.1 | 0.52-3.72 | 1.35-3.30 | 2.70-3.76 |
| P5_up | 5 | 17.5 | 9.5 | 9.3 | 13.6 | 6.5 | 14.7 | 14.5 | 23.9 | 21.8 | 0.52-3.13 | 1.37-3.16 | 2.77-3.53 |
| P6_handover | 48 | 9.1 | 8.9 | 7.6 | 7.2 | 13.8 | 17.8 | 13.9 | 23.2 | 17.9 | 0.51-1.28 | 1.44-3.10 | 2.41-3.39 |
| P7_down | 5 | 13.4 | 7.8 | 8.0 | 11.8 | 6.5 | 13.5 | 13.9 | 24.1 | 21.8 | 0.52-1.71 | 1.39-2.97 | 2.75-3.43 |
- The fingertip-front thickness follows the arm: straight arms 17.5-21.1 cm, bent arms 9.1-12.4 cm (with a bent elbow the hand points along a light
  direction: hand mu max 1.08-1.55 kg vs 2.99-3.72 kg straight).
- Back of hand and palm stay near uniform (7.0-9.5 cm) in every posture. One hand side is always at the uniform value (6.5 cm).
- Elbow and upper arm are thick in every posture (17.9-24.9 cm); no direction reaches eta_max (25 cm) at the surface, the elbow comes closest (24.9).
- forearm_top of P3 (37.3 cm) is the hanging upper arm's zone above the forearm, not the forearm's own.
- Limits: P6_handover palm is 30 deg from up and the elbow 48 deg (D2-1). Single static frame per posture; thickness is along one ray per direction.
- Picked for WP3 (D2-2): P3_bend90, P6_handover, P4_side, P2_bend45.
Files: logs/meff_isaac/on_wp2_postures.json, on_wp2_zone.json; figures goal1/overnight/postures_preview.png, wp2_zone_thickness.png.

## WP3 Static scene matrix (00:30-01:10, 65 runs, all wall-time ok, RTF 1.02-1.25 (1.02 = one run, P2_tipfront uniform s2; next lowest 1.142), infeasible 0, recapture 0)
Robot path fixed (front scene, single pass, 420 steps); each WP2 posture placed by a rigid yaw + translation so that the off path passes 2.5 cm from the
target part on the intended side (search: on_wp3_scenes.py). Calibration on off and uniform only; every built scene passed on the first try
(off gap 2.5 cm, uniform arrives, infeasible 0) except P4 forearmside18 (D3-3). Uniform and ours, seeds 0-2 (seed 2 = reserve item 1).
Scene count: original rule (torso >= 0.35 m) built 5 of 12. User instruction (0.25 m) added P3 top, P6 fingertip-front. Added rules (relax first):
0.18 m added P6 top and P6 beside-forearm. Excluded: P2 / P3 beside-forearm (torso <= 0.09 m from the path), P4 beside-forearm (no human-side
placement) = 3 of 12. Purpose check: all 3 approach directions present; bent-arm postures P2 (42 deg), P3 (89 deg), P6 (48 deg).

| scene | torso min (m) | uniform gap mean (range) cm | ours gap mean (range) cm | mesh gap u / o cm | side | u vs front deg | mu kg | ours eta cm | gap/eta u / o | pass time o/u | v99 max m/s | ours torso-cost steps | class |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| P1 reach front (V2) | 0.35 | 6.0 | 12.0 | - | front | - | - | - | - | - | - | - | ours farther |
| P1 reach over hand (V2) | 0.35 | 5.4 | 8.2 | - | up | - | - | - | - | - | - | - | ours farther |
| P2_tipfront | 0.35 | 6.0 (5.3-6.5) | 9.9 (9.3-10.4) | 5.9 / 9.9 | front | 43 | 1.03 | 10.4 | 0.92 / 0.95 | 0.96 | 0.503 | 0 | ours farther |
| P2_top | 0.35 | 6.0 (5.8-6.3) | 7.5 (6.1-8.7) | 5.9 / 7.8 | up | 92 | 0.59 | 7.0 | 0.93 / 1.06 | 0.92 | 0.472 | 0 | no difference (light) |
| P3_tipfront | 0.35 | 5.9 (5.6-6.1) | 8.5 (8.2-8.8) | 6.2 / 8.8 | front | 12 | 0.92 | 9.7 | 0.90 / 0.88 | 1.06 | 0.518 | 0 | ours farther |
| P3_top25 | 0.25 | 5.9 (5.7-6.2) | 6.0 (5.6-6.2) | 7.9 / 8.1 | up | 78 | 0.54 | 6.6 | 0.91 / 0.91 | 1.00 | 0.496 | 0 | no difference (light) |
| P4_tipfront | 0.35 | 5.9 (5.2-6.5) | 11.5 (11.0-11.9) | 5.8 / 12.5 | front | 42 | 1.30 | 12.2 | 0.91 / 0.94 | 1.06 | 0.507 | 0 | ours farther |
| P4_top | 0.35 | 5.9 (5.6-6.1) | 9.5 (8.9-10.0) | 8.6 / 11.7 | up | 73 | 0.85 | 9.2 | 0.90 / 1.04 | 0.98 | 0.493 | 0 | ours farther |
| P6_tipfront25 | 0.25 | 6.0 (5.8-6.1) | 10.2 (9.8-10.6) | 5.8 / 9.7 | front | 15 | 1.08 | 10.8 | 0.92 / 0.94 | 1.00 | 0.546 | 0 | ours farther |
| P6_top18 | 0.18 | 6.2 (5.4-6.6) | 6.6 (6.1-6.9) | 6.0 / 6.4 | up | 66 | 0.57 | 6.8 | 0.95 / 0.96 | 1.05 | 0.503 | 97 | no difference (light), torso cost in ours |
| P6_forearmside18 | 0.18 | 5.6 (5.5-5.8) | 11.2 (11.2-11.2) | 5.9 / 11.1 | front | 20 | 0.65 | 7.6 | 0.87 / 1.48 | 1.09 | 0.498 | 0 | ours farther |
Classification rule written before the runs (on_wp3_summary.py): threshold = max(1 cm, seed range); "light" = ours eta at the minimum < 7.5 cm.
- Ours keeps more distance when the robot crosses in front of the fingertip (all 4 postures, +2.7 to +5.6 cm, 3 seeds each beyond the seed range),
  and over the hand of the side-arm posture P4 (+3.7 cm). Over the hand of bent-arm postures (P2, P3, P6) there is no difference: mu there is
  0.54-0.59 kg (near mu_ref 0.53), so ours eta stays 6.6-7.0 cm. This is the intended direction dependence, not a failure.
- Beside the forearm (P6 only): ours 11.2 vs uniform 5.6 cm although ours eta at the forearm minimum is 7.6 cm (gap/eta 1.48).
  (interpretation) the heavier hand zone keeps the robot out before it reaches the forearm; not checked.
- Torso: torso cost appeared only in P6_top18 for ours (28-35 steps per run, torso gives 86-100 % of the per-step maximum cost on those steps);
  the gap there equals uniform's, so the scene says nothing about the arm. Uniform never had torso cost; no other scene had torso cost in either condition.
- Same passing side in every seed and condition; no path change; all arrived; pass time ours / uniform 0.92-1.09.
- Near-human speed p99 slightly above 0.5 m/s in single passes: max 0.546 (P6_tipfront25), 0.518 (P3_tipfront). Not seen in the R1 round trips.
Figures: goal1/overnight/wp3_effect_table.png, wp3_scene_gap.png. Numbers: logs/meff_isaac/on_wp3_summary.json, on_wp3_runs.json, on_wp3_passes.json,
on_wp3_scenes.json.

## WP4 Moving collaboration scenes (01:10-01:52, 28 runs, all wall-time ok, RTF >= 1.159, infeasible 0, recapture 0)
Motions (on_wp4_motion.py, node code, D4-1, D4-2): M1 hand-over = P6_handover held 3 s, reach / retract 0.38 m along the reach line (min-jerk,
peak 0.253 / 0.500 m/s); M2 crossing = P7_down fingertip swept 0.40 m sideways through the robot TCP line at 40 deg (peak 0.250 / 0.500 m/s),
resting 1.5 s on each side (on the far side the hand sits inside the off-path sweep). Front robot path, round trips, 60 s, uniform and ours,
seeds 0-2 (seed 2 = reserve item 1); M1 0.5 m/s also with P (--meff-motion cv), seeds 0-1. Analysis by crossing phase (WP1 method).
| motion / speed | cond | min gap per seed (cm) | critical-window max deficit per seed (cm) | hold-window mean gap (cm) | penetration (shape / mesh steps) | passes per run | stalls in front of the hand (count, s, 3 runs) |
|---|---|---|---|---|---|---|---|
| M1 0.25 | uniform | 5.3 / 2.8 / 4.1 | 1.2 / 3.7 / 2.4 | 5.8 | 0 / 0 | 27 / 26 / 27 | 0 |
| M1 0.25 | ours | 7.5 / 6.6 / 7.6 | 4.1 / 4.6 / 2.9 | 10.1 | 0 / 0 | 27 / 25 / 27 | 0 |
| M1 0.5 | uniform | 3.0 / 3.5 / 3.9 | 3.5 / 3.0 / 2.6 | 5.9 | 0 / 0 | 28 / 26 / 27 | 0 |
| M1 0.5 | ours | 4.7 / 7.7 / 6.5 | 6.7 / 3.8 / 4.5 | 10.1 | 0 / 0 | 27 / 25 / 27 | 0 |
| M1 0.5 P | uniform | 5.6 / 5.5 (s0, s1) | 0.7 / 0.9 | 6.2 | 0 / 0 | 28 / 26 | 0 |
| M1 0.5 P | ours | 9.0 / 10.0 (s0, s1) | 0.9 / 0.6 | 10.0 | 0 / 0 | 26 / 25 | 0 |
| M2 0.25 | uniform | 2.6 / 2.5 / 3.3 | 3.9 / 4.0 / 3.2 | - | 0 / 0 | 18 / 16 / 18 | 13, 32.5 s |
| M2 0.25 | ours | 3.6 / 4.2 / 4.3 | 3.1 / 2.8 / 3.0 | - | 0 / 0 | 14 / 12 / 14 | 20, 38.9 s |
| M2 0.5 | uniform | 0.6 / 1.2 / 0.0 | 5.9 / 5.3 / 6.5 | - | 0 / 0 | 21 / 21 / 21 | 17, 13.7 s |
| M2 0.5 | ours | 6.4 / 6.7 / 3.8 | - / - / 3.1 | - | 0 / 0 | 18 / 19 / 18 | 3, 5.0 s |
(critical window: M1 robot crosses within [-T_reach, +0.25 s] of hold onset; M2 within +-T_sweep/2 of the hand reaching the TCP line. "-" = no pass
of that seed fell in the window.) Near-human speed p99 max 0.472-0.493 m/s in every group.
- M1 hand-over: ours keeps the larger distance at both speeds (min gap mean +3.2 cm at 0.25 m/s, +2.9 cm at 0.5 m/s; every ours seed above every uniform seed; hold gap 10.1 vs 5.8-5.9 cm)
  with no loss of throughput (25-27 vs 26-28 passes). With P the critical deficit drops to <= 0.9 cm in both conditions and ours' min gap rises to 9.0-10.0 cm (2 seeds).
- M1 torso: the hand-over placement has the torso 0.26 m from the sweep; ours had torso cost on 0-55 steps per run, 0-1.4 % of the summed per-step
  maximum cost. Small; the hand dominates.
- M2 crossing: no penetration anywhere. At 0.5 m/s uniform came to 0.0-1.2 cm, ours stayed at 3.8-6.7 cm. At 0.25 m/s ours waits more in front of the
  hand (20 stalls, 38.9 s over 3 runs vs 13, 32.5 s) and completes fewer passes (12-14 vs 16-18 per 60 s); at 0.5 m/s ours waits less (3, 5.0 s vs
  17, 13.7 s) and completes 18-19 vs 21. (interpretation) with the hand resting inside the path the larger zone makes ours wait for the hand to leave
  rather than squeeze past; not checked frame by frame.
Figure: goal1/overnight/wp4_phase_gap.png. Numbers: logs/meff_isaac/on_wp4_summary.json, on_wp4_runs.json, on_wp4_passes.json, on_wp4_motion_check.json.

## WP5 Perception imperfection rehearsal (01:52-02:26, 26 runs, all RTF >= 1.160, infeasible 0, recapture 0)
R1 0.5 m/s scene (60 s round trips), seed 0 (worst two also seed 1). Replay options (all 0 = bit-equal, checked in WP0): lag (frame of t - lag),
jitter (per frame, per segment Gaussian offset of COM and shapes, velocity consistent, noise seed 7 for both conditions), dropout (frames missing,
last frame kept). Base = R1 runs, seeds 0-2. Measured against the true human. Extension levels lag 300 ms, jitter 4 cm, drop 60 % added (D5-1).
| variant | uniform min gap (cm) | uniform crit max deficit | ours min gap | ours crit max deficit | ours hold gap | jerk RMS u / o (rad/s^3) | penetration |
|---|---|---|---|---|---|---|---|
| base (s0-s2) | 1.6-3.7 | 2.8-4.9 | 5.4-6.4 | 6.1-7.8 | 10.3-11.8 | 21.4-22.7 / 22.3-23.6 | 0 |
| lag 50 ms | 2.8 | 3.7 | 5.6 | 7.9 | 10.7 | 22.6 / 23.0 | 0 |
| lag 100 ms (s0, s1) | 3.5, 1.7 | 3.0, 4.8 | 5.4, 4.6 | 11.1, 10.6 | 11.0, 11.4 | 24.0, 22.5 / 22.3, 22.1 | 0 |
| lag 150 ms | 2.1 | 4.4 | 5.2 | 6.4 | 10.7 | 22.4 / 25.0 | 0 |
| lag 300 ms | 3.3 | 3.2 | 3.2 | 14.8 | 10.4 | 22.5 / 25.0 | 0 |
| jitter 1 cm | 2.1 | 4.4 | 6.0 | 6.9 | 11.3 | 22.9 / 24.1 | 0 |
| jitter 2 cm (s0, s1) | 1.8, 2.6 | 4.7, 3.9 | 4.9, 6.6 | 8.2, 7.6 | 10.2, 11.4 | 26.7, 34.8 / 25.3, 24.3 | 0 |
| jitter 4 cm | 2.0 | 4.5 | 5.4 | 8.9 | 10.1 | 33.4 / 37.6 | 0 |
| drop 30 % | 1.6 | 4.9 | 6.5 | 7.4 | 10.8 | 22.4 / 23.8 | 0 |
| drop 60 % | 1.4 | 5.1 | 6.2 | 8.1 | 10.4 | 22.3 / 23.5 | 0 |
| lag 100 + lead 0.1 s | 2.1 | 4.4 | 7.7 | 7.2 | 10.8 | 22.0 / 24.5 | 0 |
| lag 100 + P (cv) | 5.6 | -0.5 | 8.3 | 3.9 | 10.9 | 24.8 / 27.1 | 0 |
Near-human speed p99 0.464-0.493 m/s in every run.
Tolerated levels (no penetration, min gap within or near the base seed range):
- Lag: up to 50 ms no measurable change. At 100 ms ours' critical deficit grows to 10.6-11.1 cm (2 seeds, base 6.1-7.8) while the min gap stays
  4.6-5.4 cm; at 300 ms ours' min gap drops to 3.2 cm (deficit 14.8). Uniform shows no clear trend (its base range is wide, 1.6-3.7 cm).
  lag 150 ms (single seed) did not follow the trend (deficit 6.4); treat single-seed lag values as +-2 cm.
- Jitter: gaps stay in range up to 4 cm (ours min 4.9-6.6); the joint command jerk rises from ~22 to 25-35 (2 cm) and 33-38 (4 cm) rad/s^3.
- Dropout: up to 60 % (effective 12 fps with holds) no change beyond the seed range.
- P (--meff-motion cv) with 100 ms lag: ours min gap 8.3 cm, deficit 3.9 cm, better than the lag-free base (1 seed); uniform 5.6 cm.
Notes for the live Kinect stage: keep the end-to-end delay of the human data <= ~50-100 ms or use P; P absorbs 100 ms here. Filter the skeleton
(jitter mostly costs command smoothness, not distance). Frame drops up to 60 % with hold-last are harmless at 0.5 m/s. Single seeds except lag100 / jit2.
Figure: goal1/overnight/wp5_perception.png. Numbers: logs/meff_isaac/on_wp5_summary.json, on_wp5_runs.json, on_wp5_passes.json.

## WP6 Tuning (02:25-02:38, 24 runs <= 25, RTF 1.161-1.221)
Goals and split written before tuning (D6-0). Problem: near-human speed p99 above the 0.52 m/s must-limit in 2 of 54 WP3 single passes (uniform
P6_tipfront25 s1 0.546, P3_tipfront s2 0.518). gap/eta was not tuned: seed-0 means already at target, seed spread 0.06-0.09 larger than any expected effect.
Shared settings only, one change each; tune set P2_tipfront, P4_top, P6_tipfront25, P3_top25 (seed 1); check set P3_tipfront, P4_tipfront, P2_top, P6_forearmside18 (seed 2).
| candidate | set | cond | p99 max t1 -> cand (m/s) | gap mean t1 -> cand (cm) | gap/eta t1 -> cand | pass time t1 -> cand (s) | must items | side unchanged |
|---|---|---|---|---|---|---|---|---|
| c1 speed_cap_w 1e6 | tune | uniform | 0.546 -> 0.511 | 6.11 -> 6.29 | 0.940 -> 0.967 | 2.34 -> 2.44 | all met | yes |
| c1 | tune | ours | 0.492 -> 0.482 | 9.16 -> 9.30 | 0.974 -> 1.004 | 2.25 -> 2.25 | all met | yes |
| c1 | check | uniform | 0.518 -> 0.499 | 5.51 -> 5.46 | 0.847 -> 0.840 | 2.25 -> 2.28 | all met | yes |
| c1 | check | ours | 0.503 -> 0.481 | 9.34 -> 9.09 | 1.034 -> 1.010 | 2.36 -> 2.31 | all met | yes |
| c2 speed_cap_dist 0.5 m | tune | uniform / ours | 0.546 -> 0.546 / 0.492 -> 0.492 | 6.11 -> 6.1 / 9.16 -> 9.2 | 0.940 -> 0.943 / 0.974 -> 0.974 | 2.34 -> 2.33 / 2.25 -> 2.29 | uniform fails (0.546) | yes |
Verdict: no candidate passes the pre-set improvement rule. c1 is the only usable candidate: it removes both must-limit violations and lowers p99 in
15 of 16 paired runs, but its mean p99 reduction (0.015 m/s) is smaller than the WP3 seed spread (0.031 m/s). Saved as goal1/config/tuned_t2a.yml
(header states this); not adopted. Numbers: logs/meff_isaac/on_wp6_summary.json, on_wp6_runs.json; config files tuned_t2a_c1.yml, tuned_t2a_c2.yml.

## Reserve 2 Gain knob on two WP3 scenes (02:38-02:44, 12 runs + WP3 runs for k 0 / 0.10)
| scene | k 0 | k 0.05 | k 0.10 | k 0.15 |
|---|---|---|---|---|
| P4_tipfront gap mean (seeds) cm | 5.9 (6.0 / 6.5 / 5.2) | 9.8 (9.5 / 10.3 / 9.5) | 11.5 (11.0 / 11.5 / 11.9) | 13.4 (12.8 / 14.2 / 13.3) |
| P4_tipfront eta at the minimum cm | 6.5 | 10.9-11.3 | 11.8-12.4 | 12.8-14.6 |
| P4_tipfront side | front x3 | front x3 | front x3 | up / front / front |
| P2_top gap mean (seeds) cm | 6.0 (6.3 / 6.0 / 5.8) | 6.4 (7.0 / 6.5 / 5.7) | 7.5 (8.7 / 7.6 / 6.1) | 8.1 (8.6 / 8.3 / 7.4) |
| P2_top eta at the minimum cm | 6.5 | 6.6-6.8 | 6.9-7.2 | 7.0-7.3 |
- P4_tipfront: the gap follows eta (gap/eta 0.88-0.97) over the whole range; the step 0 -> 0.05 is the largest (+3.9 cm), then +1.7 and +1.9 cm.
  At k 0.15 one seed of three passes over the hand (side 'up'), the same path change V2 saw at k 0.15 on the reach pose. Proportional range: k 0 to 0.10.
- P2_top (light direction, mu ~0.6 kg): eta moves only 6.5 -> 7.3 cm, the gap 6.0 -> 8.1 cm. Raising k does not create a zone where mu is near mu_ref.
- All arrived, infeasible 0, near-human speed p99 <= 0.507 m/s. Numbers: logs/meff_isaac/on_r2_gainknob.json (from on_wp3_passes.json / on_wp3_runs.json).

## Reserve 3 M3 bent-arm forearm task (02:44-02:55, 6 runs, RTF >= 1.136, infeasible 0, recapture 0)
P6_handover (elbow 48 deg) at the P6_forearmside18 placement (DR3-1), forearm stroke 10 cm along its axis at 0.25 m/s, robot passes beside the forearm.
| cond | min forearm/hand gap per seed (cm) | mesh min (cm) | critical-window max deficit (cm) | penetration | passes per 60 s | stalls in front of the hand |
|---|---|---|---|---|---|---|
| uniform | 5.1 / 4.8 / 4.4 | 4.6 | 1.7 | 0 / 0 | 25 / 25 / 25 | 0 |
| ours | 8.6 / 7.8 / 9.0 | 7.7 | 1.5 | 0 / 0 | 24 / 24 / 23 | 0 |
- Ours keeps +3.4 to +4.6 cm more (every ours seed above every uniform seed) at a cost of about one pass per minute. Torso cost in ours on 0-8 steps
  per run, about 0 % of the cost. Near-human speed p99 <= 0.482 m/s. Numbers: logs/meff_isaac/on_wp4_summary.json (M3 rows), on_wp4_motion_check.json.

## Follow-up F1 P (cv) in the crossing task M2 (02:57-03:12, 12 runs)
| speed / cond | min gap per run, base -> P (cm, 3 seeds pooled) | passes per run base -> P | stalls in front of the hand (3 runs) base -> P |
|---|---|---|---|
| 0.25 uniform | 2.5 -> 4.1 | 16-18 -> 20-22 | 13, 32.5 s -> 8, 18.8 s |
| 0.25 ours | 3.6 -> 5.2 | 12-14 -> 12-14 | 20, 38.9 s -> 21, 42.5 s |
| 0.5 uniform | 0.0 -> 2.7 | 21 -> 21 | 17, 13.7 s -> 10, 7.3 s |
| 0.5 ours | 3.8 -> 5.1 | 18-19 -> 19 | 3, 5.0 s -> 3, 5.5 s |
- P raises the minimum gap in all four groups (+1.3 to +2.7 cm) without penetration. Uniform also gets through more often at 0.25 m/s.
  Ours' waiting in front of the resting hand at 0.25 m/s is unchanged (as expected in DF1: the hand rests inside the path, velocity 0, P = static).

## Follow-up F2 P (cv) under jitter and lag; R1 1.0 m/s P seed 2 (03:12-03:25, 8 runs)
| R1 0.5 m/s, seed 0 | uniform min gap (cm) | uniform penetration shape / mesh steps | ours min gap | jerk RMS u / o | passes u / o | v99 max |
|---|---|---|---|---|---|---|
| base (s0) | 3.7 | 0 / 0 | 5.9 | 22.7 / 22.4 | 27 / 27 | 0.474 |
| jitter 2 cm, static | 1.8 | 0 / 0 | 4.9 | 26.7 / 25.3 | 26 / 26 | 0.478 |
| jitter 2 cm + P | 1.1 | 0 / 0 | 12.1 | 60.4 / 100.5 | 23 / 16 | 0.518 |
| jitter 4 cm + P | -4.3 | 62 / 34 | 13.1 | 89.9 / 137.9 | 20 / 9 | 0.515 |
| lag 100 ms + jitter 2 cm + P | -1.1 | 22 / 10 | 12.1 | 71.2 / 106.8 | 22 / 13 | 0.518 |
- P with a jittery skeleton is harmful: the frame-to-frame velocity of a 2-4 cm jitter at 30 Hz (~0.6-1.2 m/s) is extrapolated over the horizon.
  Uniform penetrates the hand (up to 62 shape steps, 34 mesh steps, min -4.3 cm); ours stays far but its commands get 4-6x more jerky and it
  completes only 9-16 passes per minute (27 without jitter). Without P the same jitter caused no penetration (WP5).
- R1 1.0 m/s with P, seed 2: uniform critical min gap 4.9, max deficit 1.6 cm; ours 7.9 / 4.3 cm. With seeds 0-1: P result holds over 3 seeds
  (ours max deficit 4.3-5.0 vs base 11.7-13.8 cm; uniform 0.9-1.8 vs 4.9-6.8 cm).
- Consequence for the Kinect stage: P only with a filtered human velocity (or a smoothed skeleton). Static is the safer default for raw skeletons.

## Follow-up F3 Velocity filter for P under jitter (03:26-03:38, 6 runs + 2 baseline bit-check runs)
New replay option --human-vel-alpha (EMA of the delivered segment velocity, 0 = off). V2 baselines bit-equal after the change (on_base_*_s0_p2).
| R1 scene, seed 0 | uniform min gap / penetration | ours min gap | ours crit max deficit | jerk RMS u / o | passes u / o | v99 max |
|---|---|---|---|---|---|---|
| 0.5 m/s jitter 2 cm + P | 1.1 / 0 | 12.1 | 3.4 | 60.4 / 100.5 | 23 / 16 | 0.518 |
| 0.5 m/s jitter 2 cm + P + filter 0.2 | 3.6 / 0 | 8.3 | 0.7 | 36.7 / 40.9 | 26 / 26 | 0.482 |
| 0.5 m/s jitter 4 cm + P | -4.3 / 62 shape, 34 mesh steps | 13.1 | 5.4 | 89.9 / 137.9 | 20 / 9 | 0.515 |
| 0.5 m/s jitter 4 cm + P + filter 0.2 | 3.8 / 0 | 6.8 | 3.1 | 45.5 / 54.5 | 26 / 24 | 0.520 |
| 1.0 m/s no jitter, P | 4.7 / 0 (crit min) | 6.1 (crit min) | 5.0 | - | 56 / 54 | 0.477 |
| 1.0 m/s no jitter, P + filter 0.2 | 3.2 / 0 (crit min) | 5.3 (crit min) | 6.1 | - | 56 / 56 | 0.478 |
| 1.0 m/s base (static) | 0.9 / 0 (crit min) | 2.0 (crit min) | 13.3 | - | 56 / 55 | 0.478 |
- The filter removes the penetration and most of the extra jerk (still 1.6-2.4x the no-jitter 22) and restores throughput (24-26 passes).
- It keeps most of P's fast-hand benefit: at 1.0 m/s ours' max deficit 6.1 cm with filter vs 5.0 without and 13.3 static (single seed).
- Kinect candidate revised: P with a filtered human velocity (EMA 0.2 here; to be tuned on real data), not P on raw frame differences. Single seeds.

## Follow-up F4 Lag 100 ms + jitter 2 cm: static vs P + filter (04:07-04:28, 12 runs)
| scene | cond | static: min gap / crit max deficit / penetration / jerk | P + filter 0.2: min gap / crit max deficit / penetration / jerk | v99 static -> P+F |
|---|---|---|---|---|
| R1 0.5 m/s s0 | uniform | 3.7 / 2.8 / 0 / 27.1 | 4.7 / 1.7 / 0 / 44.5 | 0.478 -> 0.499 |
| R1 0.5 m/s s1 | uniform | 1.3 / 5.2 / 0 / 23.1 | 2.9 / 1.4 / 0 / 40.4 | 0.463 -> 0.493 |
| R1 0.5 m/s s0 | ours | 4.8 / 10.2 / 0 / 26.9 | 7.2 / 3.2 / 0 / 51.6 | 0.477 -> 0.503 |
| R1 0.5 m/s s1 | ours | 3.9 / 11.4 / 0 / 33.5 | 7.7 / 5.1 / 0 / 44.6 | 0.475 -> 0.498 |
| R1 1.0 m/s s0 | uniform | -0.4 / 6.9 / 4 shape, 2 mesh steps / 32.6 | 1.6 / 4.9 / 0 / 52.7 | 0.479 -> 0.623 |
| R1 1.0 m/s s0 | ours | 2.3 / 15.5 / 0 / 26.3 | 5.0 / 4.6 / 0 / 52.4 | 0.478 -> 0.564 |
(min gap = all passes; crit = critical window; jerk = joint command jerk RMS rad/s^3; 1.0 m/s ours P+F had only 2 passes in the critical window.)
- With the realistic Kinect combination, P + filter is better than static on every gap measure (2 seeds at 0.5 m/s, 1 seed at 1.0 m/s) and removes the
  only penetration (uniform, 1.0 m/s static). Costs: jerk about 1.7-2x, and at 1.0 m/s the near-human speed p99 rises to 0.56-0.62 m/s, above the
  0.52 limit. (interpretation) with lag the controller sees the hand where it was; the speed cap's distance test uses the predicted shapes, which lag
  less than static but still differ from the true hand, and with the faster commands the robot passes the true hand above 0.5 m/s; not checked.

## Follow-up F5 Speed cap settings under lag + jitter + P + filter at 1.0 m/s (04:29-04:45, 4 runs, seed 0)
| profile | cond | all-pass min gap (cm) | crit max deficit (cm) | near-human speed p99 (m/s) | jerk RMS | passes |
|---|---|---|---|---|---|---|
| t1 | uniform | 1.6 | 4.9 | 0.623 | 52.7 | 54 |
| t2a_c1 (speed_cap_w 1e6) | uniform | 1.3 | 5.2 | 0.541 | 48.8 | 53 |
| t2a_c2 (speed_cap_dist 0.5 m) | uniform | 0.6 | 5.9 | 0.477 | 35.0 | 52 |
| t1 | ours | 5.0 | 4.6 | 0.564 | 52.4 | 49 |
| t2a_c1 | ours | 2.5 | 10.4 | 0.616 | 57.4 | 50 |
| t2a_c2 | ours | 3.7 | 8.9 | 0.478 | 40.5 | 49 |
- The larger speed-cap radius (c2) brings p99 back under 0.52 m/s and lowers jerk; the heavier weight (c1) does not. In this single seed both lower
  the minimum gaps (ours 5.0 -> 2.5 / 3.7 cm); the critical window holds only 2-9 passes, so the gap changes are within single-seed noise
  (WP1 spread up to 3.3 cm). No penetration in any of the 4 runs. Unresolved: one setting that keeps both the gap and the speed limit for a 1.0 m/s
  hand under 100 ms lag; not settled tonight.

## Follow-up F6 Seed 1 and 0.5 m/s check of the speed-cap radius under lag + jitter + P + filter (04:49-05:02, 6 runs)
| scene | cond | t1: min gap / crit max deficit / v99 / jerk | t2a_c2 (radius 0.5 m): min gap / crit max deficit / v99 / jerk |
|---|---|---|---|
| 1.0 m/s s0 | uniform | 1.6 / 4.9 / 0.623 / 52.7 | 0.6 / 5.9 / 0.477 / 35.0 |
| 1.0 m/s s1 | uniform | 2.3 / 4.2 / 0.628 / 50.4 | 6.2 / 0.3 / 0.467 / 29.5 |
| 1.0 m/s s0 | ours | 5.0 / 4.6 / 0.564 / 52.4 | 3.7 / 8.9 / 0.478 / 40.5 |
| 1.0 m/s s1 | ours | 1.9 / 12.0 / 0.539 / 48.3 | 3.8 / 9.4 / 0.473 / 36.1 |
| 0.5 m/s s0 | uniform | 4.7 / 1.7 / 0.499 / 44.5 | 5.1 / 1.4 / 0.475 / 36.0 |
| 0.5 m/s s0 | ours | 7.2 / 3.2 / 0.503 / 51.6 | 6.7 / 6.1 / 0.474 / 40.6 |
- The 0.5 m radius keeps the near-human speed p99 at 0.467-0.478 m/s in all 6 runs (t1: up to 0.628) and lowers jerk by 20-40 %. Its effect on the
  gap changes sign between seeds (uniform 1.0 m/s: -1.0 cm in s0, +3.9 cm in s1), i.e. no systematic gap cost. No penetration in any run.
- Ours at 1.0 m/s under lag + jitter is not consistently better than uniform (min gap 1.9-5.0 vs 0.6-6.2 cm over the four runs); the zone advantage
  seen without perturbation (WP1, F2) is not established under this combination. (interpretation) 100 ms of lag at 1 m/s is a 10 cm position error,
  larger than the ours - uniform zone difference in the critical window; not checked.

## Follow-up F7 Full Kinect candidate on clean R1 data (05:03-05:14, 8 runs)
Candidate = P (--meff-motion cv) + velocity filter EMA 0.2 (--human-vel-alpha 0.2) + profile t2a_c2 (t1 with speed_cap_dist 0.5 m). Seeds 0-1.
| speed / cond | base (t1, static) min gap / crit max deficit / hold gap | candidate min gap / crit max deficit / hold gap | jerk base -> cand | passes base -> cand |
|---|---|---|---|---|
| 0.25 uniform | 3.3, 3.2 / 3.2, 3.3 / 6.0, 5.9 | 5.6, 5.3 / 0.5, 1.2 / 5.7, 5.8 | 23.0, 23.3 -> 22.4, 20.2 | 27, 26 -> 27, 24 |
| 0.25 ours | 8.9, 8.5 / 7.0, 5.1 / 11.6, 11.8 | 9.4, 8.4 / 3.4, 3.0 / 11.0, 10.9 | 22.1, 23.4 -> 22.0, 20.7 | 26, 24 -> 26, 24 |
| 0.5 uniform | 3.7, 1.6 / 2.8, 4.9 / 5.9, 6.1 | 5.5, 5.5 / 0.1, -0.5 / 6.0, 6.0 | 22.7, 21.4 -> 22.6, 20.8 | 27, 24 -> 26, 25 |
| 0.5 ours | 5.9, 5.4 / 7.8, 7.2 / 10.8, 11.8 | 8.9, 8.4 / 3.7, 3.9 / 10.6, 10.6 | 22.4, 22.3 -> 21.5, 21.8 | 27, 24 -> 27, 25 |
(values = seed 0, seed 1; near-human speed p99 0.467-0.475 m/s everywhere; no penetration.)
- On clean data the candidate does not cost anything: minimum gaps equal or larger, critical deficits halved, hold gaps and jerk unchanged.

## Follow-up F8 Full Kinect candidate on the collaboration motions at 0.5 m/s (05:15-05:26, 8 runs, seeds 0-1)
| motion / cond | base (t1, static) min gap per seed (cm) | candidate (P + filter + radius 0.5) min gap per seed | crit max deficit base -> cand | passes base -> cand |
|---|---|---|---|---|
| M1 hand-over uniform | 3.0 / 3.5 / 3.9 | 5.7 / 5.5 | 3.5 -> -0.1 | 26-28 -> 24-26 |
| M1 hand-over ours | 4.7 / 7.7 / 6.5 | 9.0 / 9.9 | 6.7 -> 1.8 | 25-27 -> 24-26 |
| M2 crossing uniform | 0.6 / 1.2 / 0.0 | 2.6 / 2.5 | 6.5 -> 4.0 | 21 -> 21 |
| M2 crossing ours | 6.4 / 6.7 / 3.8 | 6.5 / 6.4 | 3.1 -> 1.2 | 18-19 -> 19 |
- Same picture as P alone (WP4, F1): larger gaps, no penetration, near-human speed p99 <= 0.478 m/s, hold gap unchanged (M1 ours 10.5 cm).


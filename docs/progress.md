# 단계별 진행 기록

수치는 모두 로그 파일에서 읽었다. 근거로 든 로그는 모두 이 저장소에 들어 있다. 비교는 균일 대 제안, 같은 설정으로만 쓴다.

실행 노드에는 R1 용 --round-trip 옵션이 들어 있다 (`env_check/scripts/step2b_mpc_isaac.py`).

## 1-2 유효질량을 MPC에 공급 (완료)

- 확인: 사람 동역학 사본 노드의 분절 역관성과 위치가 원본 노드와 같은지, 로봇 구별 역관성을 MPC 루프에 넣을 수 있는지.
- 수치: m_h(u) 6,156개에서 사본 대 원본 상대 차 최대 7.8e-7, 위치 차 최대 0 m (1,082 프레임). 로봇 충돌 구 61개.
- 근거: `goal1/logs/g1_human_validate.json`, `goal1/logs/g1_robot_validate.json`, 기록 `goal1/step1_2_report.md`.

## 1-3 비용 구현 (완료)

- 확인: GPU 비용 채널(warp 커널, CUDA graph 캡처 안)을 켰을 때 실시간성과 solver 증가량.
- 수치: 질량 모드 단독 실행 실시간 배율 1.114, infeasible 0. 같은 입력 짝 비교에서 solver 증가 평균 +0.79 ms (n 1,199 스텝).
- 근거: `goal1/logs/meff_isaac/g13_sep_mass_analysis.json`, `g13_pair_maxall_analysis.json`, 기록 `goal1/step1_3_report.md`.

## B2a 정지 자세에서 방향 효과 확인 (완료, 최종 회차 B2a-5)

- 확인: 뻗은 팔 정지 자세에서 질량 비용(세트 A: η = clamp(0.10·(μ/2.0)^0.5, 0.05, 0.20))과 질량 비용의 두 장면 평균 η(정면 10.3, 위 6.9 cm)에 맞춘 균일 비용(η 8.61 cm)을 정면·위 장면에서 비교. seed 8개, 프로파일 t1.
- 수치 (손 최소 간격, 형상 기준, 평균): 정면 질량 8.2 cm 대 균일 7.8 cm, 위 질량 5.7 cm 대 균일 7.4 cm.
- 방향 상호작용 (정면의 질량−균일) − (위의 질량−균일) = +2.46 cm, seed 8개 모두 같은 부호.
- 근거: `goal1/logs/meff_isaac/b2a5_summary.json` (`by_condition`, `paired`, `interaction`), `b2a5_runs.json`, `b2a5_gate.json`, 그림 `goal1/b2a/b2a5_fig*.png`.

## T 사람 근처 속도 상한 0.5 m/s 와 조정 (완료)

- 확인: 사람 근처(형상 기준 0.4 m 안) 로봇 구 속도 비용과 자세 가중치 조합 4가지 중 속도·그리퍼 회전 기준을 모두 만족하는 설정.
- 수치: 선택 s4 (속도 가중치 3e5, 자세 회전 가중치 200): 사람 근처 속도 p99 최대 0.499 m/s, 최소 간격 순간 회전 최대 5.4°, 모두 도달, infeasible 0.
- 속도 상한을 켰을 때 solver 증가 평균 +0.74 ms (짝 비교, n 1,199), 실시간 배율 1.099.
- 근거: `goal1/logs/meff_isaac/t1_summary.json`, 설정 `goal1/config/tuned_t1.yml`.

## V1 기본 간격 + 질량 이득 식 (완료)

- 확인: 균일 η 를 바꾸며 실제 간격이 η 를 얼마나 따라가는지 보정하고, 식 η = clamp(d_base + k_gain·max(sqrt(μ/μ_ref) − 1, 0), d_base, η_max) 를 정함.
- 수치: 균일 η 6.5~20 cm 에서 간격/η 비 정면 0.915~0.972, 위 0.825~0.895. 기본값 d_base 0.065 m, k_gain 0.10 m, μ_ref 0.53 kg, η_max 0.25 m.
- 근거: `goal1/logs/meff_isaac/v1_calib.json`, `v1_gain_table.json`, `v1_zone.json`.

## V2 정적 시연과 질량 이득 조절 (완료)

- 확인: 균일(η 6.5 cm) 대 제안(k_gain 0.10), 같은 설정, 정면·위 장면, seed 3개. 질량 이득 k_gain 0/0.05/0.10/0.15 (정면, seed 0).
- 수치: 손 간격(형상) 정면 균일 6.0 대 제안 12.0 cm, 방향 대비(정면 − 위) 균일 +0.5 대 제안 +3.8 cm (메시 기준 +1.2 대 +2.2 cm).
- k_gain 별 손 간격 6.3 / 9.7 / 12.5 / 13.5 cm, 0.15 에서 손 위로 넘어가는 길. 정면 구역 두께 균일 6.6 대 제안 20.0 cm.
- 근거: `goal1/logs/meff_isaac/v2_summary.json`, `v2_runs.json`, `v2_thickness.json`, `v2_posture.json`, 영상 `media/v2_main.gif`, `media/v2_gain_knob.gif`.

## R1 움직이는 손 재생 (완료)

- 확인: 정면 장면에서 손이 뻗기 → 머묾 1.5 s → 빼기 → 쉼을 반복할 때(손끝 최고 속도 0.25 / 0.5 / 1.0 m/s, 최소 저크) 균일(k_gain 0) 대 제안(k_gain 0.10). 로봇은 왕복, seed 3개, 프로파일 t1, 사람 움직임 예측 정지.
- 수치 (손 최소 간격, 형상 기준): 0.25 m/s 균일 3.2 대 제안 7.3 cm, 0.5 m/s 1.6 대 5.4 cm, 1.0 m/s −0.3 대 2.0 cm. 머묾 간격 균일 5.8 대 제안 10.8 cm.
- 실시간 배율 1.114~1.196 (rollout 비용 집계 끔), recapture 0, infeasible 0. 형상 침투는 1.0 m/s 균일 seed 1 의 3 스텝뿐 (메시 침투 0).
- 기준 C("뻗는 중" 통과 평균이 seed 3개 모두 제안 > 균일)는 seed 1 에서 불통과. 이 평균은 손과 만나는 위상에 좌우돼 회피 성능을 재지 못한다고 판단해 overnight WP1 에서 위상 기준으로 다시 분석.
- 근거: `goal1/logs/meff_isaac/r1_summary.json`, `r1_runs.json`, `r1_passes.json`, 그림 `goal1/r1/r1_*.png`.

## Overnight 2026-11-17/18 자율 진행 (완료, 상세 `goal1/overnight/REPORT.md`)

- WP1 빠른 손: 1.0 m/s 에서 제안의 간격 부족(η − 간격, 손이 뻗는 중 지나는 통과)은 사람 움직임 예측이 없어서다. 등속 예측(--meff-motion cv)으로 최대 부족이 11.7~13.8 → 4.3~5.0 cm (seed 3개). 속도 상한을 0.75 / 1.0 m/s 로 올려도 줄지 않았다.
- WP2 자세 7개의 방향별 구역 두께: 손끝 앞은 팔을 편 자세 17.5~21.1 cm, 굽힌 자세 9.1~12.4 cm. 손등·손바닥은 모든 자세에서 7.0~9.5 cm.
- WP3 정지 장면 9개 (자세 4개 × 방향, 몸통 거리 0.35 / 0.25 / 0.18 m 세 무리): 손끝 앞 장면은 모두 제안이 +2.7~+5.6 cm 더 멀리, 굽힌 팔의 손 위 장면은 차이 없음(μ ≈ μ_ref), 전완 옆(P6)은 +5.6 cm.
- WP4 협업 동작: 건네기에서 제안이 머묾 간격 10.1 대 5.8~5.9 cm, 가로지르기에서 0.5 m/s 최소 간격 3.8~6.7 대 0.0~1.2 cm. 침투 0.
- WP5 인지 불완전성: 지연 300 ms, 떨림 4 cm, 누락 60% 까지 침투 0. 지연 100 ms 부터 제안의 간격 부족이 커진다. 떨림은 명령 저크를 키운다.
- 후속: 등속 예측은 떨리는 골격에서 해롭다(균일 침투 62 스텝). 속도 필터(EMA 0.2)를 붙이면 해결되고, 지연 100 ms + 떨림 2 cm 에서 정지보다 간격이 크다. 단 1.0 m/s 에서 사람 근처 속도 p99 가 0.56~0.62 m/s 로 올라간다.
- WP6 튜닝: 속도 상한 가중치 1e6 후보(tuned_t2a.yml)는 must 기준을 모두 지켰지만 개선 폭이 seed 차이보다 작아 규칙상 개선 아님. 채택하지 않음.
- 근거: `goal1/logs/meff_isaac/on_wp*_summary.json`, `on_wp*_runs.json`, `on_wp*_passes.json`, 그림 `goal1/overnight/*.png`.

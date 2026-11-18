> **English summary.** Human-avoidance MPC for a Franka FR3 in which the avoidance zone around each body part changes shape with the directional effective mass of the human (body shape, posture, collision direction), compared against uniform avoidance under the same settings.
> Pipeline: Azure Kinect + real-time SMPL → per-segment directional effective mass → reduced mass μ → GPU MPPI cost → FR3 in Isaac Sim 4.5.
> The zone rule now compares μ with a per-body-part reference mass (ISO/TS 15066 body-region masses). Results: simulation with replayed human data (static arms, moving hand, hand-over, crossing, synthetic camera faults) and a real Azure Kinect recording of three arm motions replayed at real time. The real FR3 is not tested yet. Analyses: [analysis/README.md](analysis/README.md), [analysis/live/README.md](analysis/live/README.md).

# effmass-mpc: 유효질량으로 회피 구역을 바꾸는 사람 회피 MPC

사람의 체형·자세·충돌 방향에 따라 달라지는 유효질량을 이용해 회피 구역의 모양을 바꾸는 MPC를 개발한다. 현재 Isaac Sim 의 Franka FR3 에서 검증하고 있으며, 실물 FR3 적용을 목표로 한다.

![실제 Kinect 녹화: 손끝 앞을 지날 때 왼쪽 균일, 오른쪽 부위별 기준](analysis/live/media/front_pass_differs.gif)

*실제 Kinect 로 녹화한 편 팔을 제 속도로 다시 틀고, 로봇이 손끝 앞을 지나가게 했다. 왼쪽 균일 회피(6.5 cm 고정), 오른쪽 부위별 기준(k_gain 0.10). 반투명 몸 = SMPL 메시, 팔 둘레 = 회피 구역, seed 0. 컬러 영상과 함께 본 원본: [analysis/live/media/front_motion1.mp4](analysis/live/media/front_motion1.mp4)*

> ### [실제 Kinect 녹화로 본 손·팔꿈치 회피 → analysis/live/README.md](analysis/live/README.md)
> 손끝 앞을 지날 때는 부위별 기준이 더 멀리 지나가고(+0.9~1.5 cm), 팔 옆으로 다가올 때는 두 방식이 거의 같다(+0.2~0.5 cm). 구역 모양 그림과 세 화면 영상(컬러·균일·부위별)을 함께 실었다.
>
> ### [시뮬레이션 결과 분석 전체 → analysis/README.md](analysis/README.md)
> 자세·방향별 구역, 정지한 팔 9개 장면, 움직이는 손, 건네기·가로지르기, 빠른 손과 움직임 예측, 카메라 지연·떨림·누락. 일부 그림·영상은 이전 식 결과이며, 각 항목에 표시했다.

> **이전 연구**
> - 석사 연구 [human-robot-safety-system](https://github.com/ukhyeon/human-robot-safety-system): SMPL 기반 개인별·방향별 사람 유효질량을 실시간으로 추정해 Indy7 의 충돌 위험 평가와 속도 조절에 적용했다. 이 저장소의 사람 유효질량 계산은 이 연구에서 이어진다.
> - [franka-depth-mppi](https://github.com/ukhyeon/franka-depth-mppi): Isaac Sim Franka 의 cuRobo MPPI 에 손 ROI 근접 비용을 더하고, 가정한 손 유효질량에 따라 활성 거리를 바꿔 비교했다. 이 저장소의 비용 채널 구조는 이 프로젝트에서 이어진다.

## 1. 목표

- 사람의 체형, 자세, 충돌 방향에 따라 달라지는 유효질량으로 회피 구역의 모양을 바꾸는 사람 회피 MPC를 Isaac Sim에서 설계하고, 실물 Franka FR3에 적용한다.
- 비교 기준은 **균일 회피**(사람의 질량 정보를 쓰지 않고 모든 부위·방향에 같은 간격을 두는 방식)이며, 제안 방법과 같은 설정에서 비교한다.
- 논문용 통계보다 "회피 구역의 모양 차이를 한눈에 보이게" 하는 것을 우선한다.

## 2. 핵심 아이디어

**설계 질문: 로봇이 사람의 어느 부위에 어느 방향으로 접근하느냐에 따라 충돌 시 유효질량이 다를 때, 그 방향으로 얼마만큼의 간격을 두어야 하는가?**

충돌 방향 u 는 로봇 구 중심에서 사람 형상의 가장 가까운 점을 향하는 방향이다. 이 방향으로 사람 분절의 유효질량 m_h, 로봇 구 위치의 유효질량 m_r 을 구하고, 둘을 묶은 환산질량 μ 로 그 쌍의 간격 η 를 정한다.

```
m_h = 1 / (uᵀ Λ_h⁻¹ u)        Λ_h⁻¹ : 사람 분절의 3x3 작업공간 역관성 (SMPL 체형·자세에서 계산)
m_r = 1 / (uᵀ Λ_r⁻¹ u)        Λ_r⁻¹ : 로봇 구 중심의 3x3 작업공간 역관성 (FR3 모델)
μ   = 1 / (uᵀ Λ_h⁻¹ u + uᵀ Λ_r⁻¹ u)
μ_iso = 1 / (1 / m_iso(부위) + 1 / m_r)      m_iso : ISO/TS 15066 의 부위별 기준 질량

η = clamp( d_base + k_gain · max( sqrt(μ / μ_iso) − 1, 0 ),  d_base,  η_max )
```

| 매개변수 | 기본값 | 뜻 |
|---|---|---|
| d_base | 0.065 m | 기본 간격 (균일 회피의 간격과 같음) |
| k_gain | 0.10 m | 질량 이득. **k_gain = 0 이면 균일 회피와 같다** |
| μ_iso | 부위별 | 그 부위가 혼자 부딪힐 때의 환산질량. 이보다 가벼운 충돌은 기본 간격만 둔다 |
| η_max | 0.25 m | 간격 상한 |

이전 식은 모든 부위에 같은 기준 μ_ref = 0.53 kg 을 썼다. 손은 두 식이 거의 같고, 아래팔·위팔·몸통·머리는 지금 식에서 구역이 얇다(팔 옆이나 몸통 쪽에서 불필요하게 멀리 돌지 않는다). 바뀐 숫자는 [goal1/public_draft/changed_numbers.md](goal1/public_draft/changed_numbers.md) 에 모았다.

같은 손이라도 방향에 따라 구역 두께가 달라진다. 녹화된 세 자세의 단면(점 로봇 4.6 kg):

| 자세 | 손끝(팔꿈치) 앞, 균일 | 부위별 | 위쪽, 균일 | 부위별 |
|---|---|---|---|---|
| 편 팔 | 6.6 cm | 16.7 cm | 6.6 cm | 7.4 cm |
| 굽힌 손 | 6.6 cm | 11.2 cm | 6.6 cm | 7.5 cm |
| 팔꿈치 내밀기 | 6.6 cm | 10.0 cm | 6.6 cm | 7.8 cm |

![편 팔의 구역 단면: 균일 대 부위별](analysis/live/figures/zone_motion1.png)

![구역 표면: 균일 대 부위별](analysis/figures/zone_3d.png)

## 3. 현재 단계 (갱신: 2026-11-19)

| 단계 | 내용 | 상태 |
|---|---|---|
| 1-2 | 유효질량을 MPC에 공급 | 완료 |
| 1-3 | 비용 구현 | 완료 |
| B2a | 정지 자세에서 방향 효과 확인 | 완료 |
| T | 사람 근처 속도 상한(0.5 m/s) 적용과 가중치 조정 | 완료 |
| V1, V2 | 기본 간격 + 질량 이득 식, 정적 시연 | 완료 |
| R1 | 움직이는 손 재생 | 완료 |
| 시험 | 자세 7개·방향별 구역, 정지 장면 9개, 협업 동작, 인지 불완전성(합성) | 완료 (이전 식) |
| 부위별 기준 | 구역 식의 기준을 부위별 ISO 질량으로 변경, 정지·움직이는 손·협업 일부 재실행 | 완료 |
| R2 | 실시간 Kinect + 가상 Franka (실시간 실행, 녹화, 제 속도 다시 틀기 비교) | 완료 (손·팔꿈치 세 동작) |
| R3 | 실물 연결 준비: 계산 시간, 60 → 1000 Hz 잇기, 안전 장치 | 진행 중 |
| G2 | 실물 FR3 적용 | 예정 |

단계별 확인 내용과 근거 로그는 [docs/progress.md](docs/progress.md) 에 있다.

## 4. 결과 요약

- **실제 녹화, 손끝 앞 통과:** 부위별 기준이 균일보다 0.9 / 1.5 / 1.2 cm 더 멀리 지나갔다(편 팔 / 굽힌 손 / 팔꿈치, seed 3개 중앙값).
- **실제 녹화, 팔 옆 접근:** 0.2 / 0.5 / 0.4 cm 로 거의 같다. 팔 옆 방향은 움직이는 질량이 작아 부위별 기준에서도 구역이 얇기 때문이며, 뜻한 대로의 결과다.
- **정지 장면 V2 정면(앞으로 뻗은 팔의 손끝 앞):** 균일 5.9 cm 대 부위별 10.7 cm(seed 3개 평균, 통과 구간 규칙).
- **아래팔 옆 장면:** 이전 식은 12.1 cm 를 두었지만 부위별 기준은 6.5 cm(균일 5.6)로, 가벼운 방향에서 불필요한 우회를 줄였다.

## 5. 한계와 주의

1. **실제 녹화 비교의 길은 녹화 뒤에 정했다.** 녹화된 사람은 로봇에 반응하지 않고, 비교 영상의 Isaac 화면은 실행 기록으로 다시 그린 것이다. 실시간 실행 당시에는 화면이 실제의 약 0.6배로 흘렀다.
2. **빠른 손 + 지연은 미해결.** 손이 1.0 m/s 로 움직이고 지연 100 ms 가 겹치면, 제안이 균일보다 낫다는 것이 확인되지 않았다(이전 식 결과).
3. **가로지르기 0.5 m/s 에서 부위별 기준의 이점이 없다.** 아래팔 쪽에서 다가오는 장면이라 구역이 균일과 비슷하다(최소 간격 0.2~1.2 대 0.0~1.2 cm).
4. **카메라 결함 결과는 이전 식, 대부분 seed 1개, 합성 잡음 기준이다.**
5. **속도 상한 0.5 m/s 는 설계값이며 규격 적합 주장이 아니다.** 충돌력 측정과 규격 검증은 범위 밖이다.
6. **실물 로봇 결과는 아직 없다.** 사람 한 명의 녹화 한 번이다.
7. **구역이 두꺼워도 실제 간격은 그보다 작을 수 있다.** 로봇은 구역이 얇은 쪽(비스듬한 쪽, 위쪽)으로 비켜 지나간다. 실제 녹화에서도 가장 가까운 방향이 손 위쪽이어서 차이가 정지 장면보다 작았다.
8. **구역 그림과 두께는 로봇을 4.6 kg 의 점으로 놓고 계산했다.**

## 6. 시스템 구성

```mermaid
flowchart LR
    K[Azure Kinect] --> S[실시간 SMPL<br/>체형·자세]
    S --> H[사람 동역학 노드<br/>분절별 Λ_h⁻¹, 캡슐 형상]
    R[FR3 동역학 모델<br/>numpy 구현] --> RR[로봇 구별 Λ_r⁻¹]
    H -->|ROS 2| C
    RR --> C
    subgraph GPU["GPU (RTX 2080 Ti)"]
      C[유효질량 비용 채널<br/>u, m_h, m_r, μ, η] --> M[cuRobo MPPI]
    end
    M -->|관절 위치 명령| I[Isaac Sim 4.5<br/>FR3]
    I -->|관절 상태| M
```

## 7. 실행 환경과 필요한 것

(현재 README 7절과 같음.)

## 8. 저장소 구조

(현재 README 8절에 다음 두 줄을 더함.)

| 경로 | 설명 |
|---|---|
| `analysis/live/` | 실제 Kinect 녹화 비교 글, 그림, 세 화면 영상 |
| `goal1/live/live3m/` | 녹화 live3m 의 비교 결과(표 json, 그림, 영상). 원본 컬러 프레임과 사람 자세 자료는 미포함 |

## 9. 문서

- 연구일지: https://app.notion.com/p/MPC-Franka-41afcf2876fa80ceb6fffa02d7205a02
- 전체 계획: https://app.notion.com/p/419fcf2876fa806593b9d49d7d07a0ce
- Franka ROS 2 세팅: https://app.notion.com/p/Franka-ROS-2-41bfcf2876fa805f9f9fc1dea7914fd1

## 10. 미포함 항목과 이유

(현재 README 10절과 같음. "사람 데이터" 줄에 "Kinect 원본 컬러 프레임"을 더함.)

## 11. 참고문헌

(현재 README 11절과 같음.)

# 목표 1 · 1-2 사람·로봇 질량 공급 — 보고서

- 작성: 2026-11-16 (진행 중)
- 상태: **1-2d 완료 (headless 기준 통과). GUI 확인 직전 멈춤: 사용자가 형상(머리 주성분 캡슐, 손 2캡슐, 몸통)·가장 가까운 쌍 선 확인.**

## 요약

| 항목 | 결과 | 판정 |
|---|---|---|
| FR3 모델 출처 | franka_description **2.9.0** (commit `7aeeddc`, 2026-08-17), `robots/fr3/fr3.urdf.xacro`(기본 FR3, `hand:=true ee_id:=franka_hand`), Apache-2.0. `~/effmass_mpc/ref/franka_description` | 기록 |
| cuRobo FR3 설정 | `goal1/fr3/fr3.yml` (franka.yml 템플릿, 링크 이름만 FR3), 관절·속도 한계는 URDF 값, 충돌 구는 Panda 구를 FR3 링크로 옮김 | 완료 |
| 충돌 구 방법 | 옮긴 구가 FR3 메시를 Panda 구가 Panda 메시를 덮는 것과 같거나 더 잘 덮음. cuRobo 맞춤 도구(같은 개수)는 덜 덮음 | 옮김 채택 |
| FK 확인 | cuRobo FR3 ↔ Panda: 공통 관절 범위 50 자세에서 손 위치 차 0.0000 mm. cuRobo FR3 ↔ Isaac FR3 USD: 10 자세 손 위치 차 최대 0.001 mm | 통과 |
| Isaac FR3 | Isaac 4.5 공식 `Isaac/Robots/Franka/FR3/fr3.usd` (17.5 MB, 2025-01-30, Apache-2.0) | 사용 |
| **USD 관절 한계 ≠ URDF 2.9.0** | USD는 franka_description이 2025-09-23(`919c533`)에 FR3 한계를 넓히기 **전** 값(예: 관절 1 ±2.7437 vs ±2.9007 rad) | 기록, 실제 로봇 버전 확인 필요 |
| 측정 A (Isaac 없음, v2) | Panda: solver 6.12 / p95 6.80 ms, infeasible 0, 도달 8. **FR3: 6.22 / 7.12 ms, infeasible 0, 도달 9** | MPC 설정 정상 |
| **측정 B (Isaac, v2) FR3 2회** | solver 14.9 / p95 17.9 ms, **배율 0.81, infeasible 974, 도달 1** (Panda 2단계: 9.3~9.6 / 11.4~11.5 ms, 배율 1.006~1.028, infeasible 16, 도달 6) | **예상 밖 → 멈춤** |
| 원인 (진단) | Isaac 위치 명령 추종: FR3 USD 평균 지연 0.09 s, 정착 0.30 s / Panda URDF 변환 0.009 s, 0.017 s. USD 구동 이득 stiffness 60,000, damping 6,000 (Panda 변환 모델 실제 USD 값 625, 10 — 아래 8절 정정) | 확인 |
| **정정: Panda 변환 모델의 실제 구동 이득** | 앞 보고의 1,047.2 / 52.4는 cuRobo helper 변환기 설정값이었다. Isaac 장면에 실제 들어간 값은 **stiffness 625, damping 10**(USD, 도 기준 = 35,810 / 573 N·m/rad) | 정정 |
| 결정 1: FR3 팔 구동 이득 | 측정 스크립트에서 이번 실행에만 덮어씀(기본 625 / 10 = Panda 변환 모델 실제 값, 인자로 변경 가능). 진단: **지연 0.009 s, 정착 0.017 s → Panda 수준**. 1,047.2 / 52.4면 0.042 s, USD 기본값이면 0.092 s | 적용·확인 |
| 결정 2: cuRobo 관절 한계 | 좁은 FR3 한계(`fr3_curobo_narrow.urdf`), 2.9.0 한계는 `fr3_curobo.urdf`·`fr3_urdf290.yml`에 남김 | 적용 |
| 조사: fr3 / fr3v2 / fr3v2_1 | 관절 원점·한계·속도 같음. 링크 질량 합(link0~7) 17.89 / 15.91 / 15.93 kg (라벨 18.3 kg). COM 최대 113 mm, 관성 대각합 비 0.26~6.0 차이. 손 끝 유효질량 fr3v2_1이 fr3보다 평균 21% 낮음 | 기록 |
| 조사: Franka fr3v2_1 USD | `frankarobotics/franka_simulation` 릴리스 v0.1.0(Isaac Sim 6.0 용), **라이선스 표시 없음**. Isaac 4.5에서 파일은 열리지만 PhysX 관절 생성 실패(강체 중첩), 손 없음, 구동 이득 0. USD 관절 프레임 FK는 URDF와 0.0001 mm 일치 | 4.5에서 그대로는 못 씀 |
| 조사: 구 재사용 | 지금 옮긴 구가 fr3v2_1 형상을 fr3와 비슷하게 덮음(p95 차 ≤ 3 mm) | 다시 옮길 필요 없음 |
| **모델 결정 (사용자)** | **fr3v2_1** (franka_description 2.9.0) 하나를 cuRobo·Isaac·1-2c 로봇 유효질량의 기준으로. 관절 위치 한계는 좁은 FR3 값 | 적용 |
| fr3v2_1 Isaac (URDF 변환) | 변환이 넣은 팔 이득 stiffness 625, **damping 0.003**(URDF `dynamics damping`) → 실행 중 articulation에서 625 / 10으로 맞춤 | 적용 |
| fr3v2_1 FK | cuRobo ↔ Isaac 변환 모델 손 위치 차 최대 0.0005 mm (10 자세) | 통과 |
| fr3v2_1 추종 지연 | 625 / 10: 0.010 / 0.009 / 0.011 s, 정착 0.017 s (Panda 0.009 s) | 통과 |
| **fr3v2_1 측정 B 2회** | solver 8.05 / 8.02 ms, p95 9.29 / 8.69 ms, **배율 1.150 / 1.159, infeasible 0 / 0, 도달 5 / 5** | **Panda 범위 안 → GUI 직전 멈춤** |
| Franka fr3v2_1 USD | **미사용, 라이선스 미확인, Isaac 6용**. 받은 파일은 그대로 둠 | 기록 |
| 1-2a-1 GUI | 사용자 확인: FR3 손 포함 정상, 목표 왕복·장애물 회피 | 통과 |
| **1-2a-2 장면 g1** | 로봇 탁자 0.6×0.6×0.9 m + 바닥(z −0.9), 2단계 table 판·움직이는 장애물 없음. 목표 (0.35, −0.20, 0.20), (0.50, 0.40, 0.30), 손 아래 방향. 둘 다 사람 표면에서 0.150 m(전 프레임 최소) | 완료 |
| 1-2a-2 측정 (fr3v2_1) | 사람 없이: 도달 5, infeasible 0, 배율 1.359. 4-3d (F) 설정(재생·노드 CPU·사람 표시·GPU 텐서) 2회: **배율 1.325 / 1.306, 루프 12.5 / 12.7 ms, 도달 5, infeasible 0**, HumanDynamicsState 30 Hz | 통과 |
| 1-2a-3 사람 쪽 | 조합 1 분절별 3×3 역관성 = `J_p[s] · Minv_contact · J_p[s]ᵀ`. 필요한 값이 노드 안에 다 있음(코드 위치 정리) | 조사 |
| 1-2a-3 로봇 쪽 | Pinocchio 3.9.0(시스템, Isaac Python에서도 import 됨) CPU 상태당 수십 µs. torch GPU 일괄 12,000 상태 20.9 ms(eager). **회전자 반사 관성 N²Jm을 넣으면 손 끝 m_r 평균 +250%** | 조사 |
| 1-2a-3 MPC 쪽 | 400×30×61×19 거리: (H) 4.4 ms, (R) 20.9 ms(eager), 추가 메모리 270 MiB. 60 Hz 예산 안에 넣으려면 계산 범위를 줄이거나 그래프 캡처 구현 필요 | 조사 |
| **1-2b 사본 노드** | `~/effmass_mpc/ws` colcon (시스템 Humble만). `effmass_msgs/HumanInertiaState`, `effmass_human` 노드. 수정 4가지(잠금+스냅샷, stamp 차 속도·새 입력 없으면 계산 안 함, 64방향 제거, CPU 1스레드) | 완료 |
| 1-2b 위치 | 사본 ↔ 원본 노드 차 0 m, 사본 ↔ 독립 계산 최대 0.0008 mm (재생 1,262 / 1,082 프레임 모두 1 mm 이내). 자세·위치 혼합 0건 | 통과 |
| 1-2b 속도 (7번째마다 건너뜀) | 프레임 차 2에서 사본/실제 중앙 **1.000** (p5 0.980, p95 1.017). 원본 노드는 2.000 | 통과 |
| 1-2b 역관성 | m_h(u) 6,156개: 사본 ↔ 원본 `compute_meff_constrained_from_common`(ROM 잠금 끔, +2 kg 없음) 상대 차 최대 7.8e-7. ROM 켬과 비교: 16.9%가 달라짐, 평균 +2.65%, 최대 +270% (기록만) | 통과 |
| 1-2b 성능 (ROS 실제 실행) | 30.03 Hz (1,262/1,262 수신), 메시지당 계산 평균 5.18 ms, p95 5.50 ms (첫 메시지 42 ms = 체형·형상 맞춤 포함), CPU 평균 17%, GPU 사용 없음 | 통과 |
| 1-2b 형상 | 22개 (몸통 3분절 × 캡슐 2, 머리 구, 손·발 끝 캡슐, 나머지 캡슐). 반지름 = 정점 거리 p95 → 밖 정점 약 5%. 밖 거리 p95 최대: **머리 33.6 mm**, 위팔 15.0 mm, 오른손 14.2 mm. 손 캡슐 반지름 0.10 m | 기록 (머리·손 확인 필요) |
| **1-2c 로봇 모듈** | `effmass_robot/robot_inertia.py` (Pinocchio 3.9.0 /opt/ros, 새 설치 없음). fr3v2_1 좁은 한계 URDF, 손가락 0.04 고정, 충돌 구 61개마다 3×3 Λ⁻¹, 회전자 스위치(기본 켬), link0 Λ⁻¹ = 0 | 완료 |
| 1-2c 정확도 | numpy 독립 구현과 Λ⁻¹ 상대 차 3.2e-14, 구 중심 ↔ cuRobo 4e-7 m (Isaac Python). indy_ws 식을 같은 점에 적용하면 차 8e-16 | 통과 |
| 1-2c 회전자 켬/끔 | 손 끝 m_r 중앙 **1.11 → 4.15 kg (×3.6)**, 팔꿈치(link4 원점) 12.4 → 36.9 kg (×2.7) | 기록 |
| 1-2c 시간 | 구 61개 전체, 한 상태: 0.20 ms (CPU, Python) | 기록 |
| 1-2c Isaac Python | min 경로 + ws 경로를 sys.path 맨 뒤에 추가. 두 import 순서 모두 numpy 1.26.0·torch 2.5.1+cu118·warp 1.5.0·curobo 0.7.7 위치·객체 그대로 | 통과 |
| **1-2d 형상** | 머리 위아래 캡슐 = 길이 0(높이 ≈ 깊이), p95 33.6 → 25.2 mm. 손 캡슐 2개(손바닥·엄지) p95 8.1 / 7.4 mm. 형상 24개 | 적용 |
| **1-2d GPU 채널 (Isaac 없음)** | 가중치 0 warp 채널: 궤적 비트 동일, 재캡처 0, 주소 불변. GPU↔CPU 219,600개 × 3설정 상대 차 ≤ 5e-6 (동률 1개). solver 짝 측정 +0.56~0.58 ms | 통과 |
| 1-2d Isaac (1차) | Kit 안 `import pinocchio` 실패 (Kit Assimp 5.4.3·urdfdom ↔ hpp-fcl). → 안 C(numpy) 결정 | 해결 |
| **1-2d numpy Λ_r** | Pinocchio와 상대 차 1.9e-15 (201 상태), 171.8 µs/상태 (FK 32, M 71, 역행렬 11, 점 야코비안 46, 역관성 11 µs). 시뮬·실기 공통, Pinocchio는 대조 기준 | 통과 |
| 1-2d 머리 형상 (2차) | 주성분 축 캡슐: 길이 0.014 m, r 0.118 m, p95 12.9 mm, 머리 회전 따라감(오차 2e-16) | 적용 |
| 1-2d Isaac 실시간 (1차, 1안·2안 전) | 짝 측정 solver +0.98~1.01 ms. 따로 실행 끔→켬 +2.2~2.9 ms, 배율 0.93~0.99. 재캡처 0, 주소 불변, 사람 30 Hz, 명령 동일, 덤프 201,300개 상대 차 ≤ 5e-6 | 불통과 → 원인 확인 후 해결(아래 최종) |
| **1-2d 원인** | Λ_r의 `np.linalg.inv`(7×7)가 Isaac numpy OpenBLAS 스레드 6개를 바쁜 대기로 만듦 (프로세스 CPU 146% → 727%). 물리 step +0.96 ms 등 모든 CPU 구간이 느려짐. 클럭 탓 아님 | 확인 |
| **1-2d 최종 (1안 Cholesky + 2안 OPENBLAS/OMP 1스레드)** | 짝 측정 solver +1.01 / +0.99 ms (p95 +2.09 / +2.05, 기록만). 따로 실행 끔 8.13 / 8.02 → 켬 9.09 / 9.02 ms (같은 1,350 MHz). **채널 켬 실시간 배율 1.178 / 1.191**. 바쁜 대기 스레드 없음, Isaac CPU 727% → 193~199%. 명령 동일, 재캡처 0, 주소 불변, 사람 30 Hz | **통과** |
| 1-2d GUI 준비 | topk(기본 5) 가장 가까운 쌍 선, 형상 반투명, 메시. headless 시험 통과 | GUI 직전 멈춤 |
| 1-2c 손 끝 μ | 회전자 켬이면 μ/m_h: 손 0.57 → **0.84**, 아래팔 0.26 → 0.58, 머리 0.14 → 0.37, 가슴 0.04 → 0.14 | 기록 |

---

## 1-2a-1. FR3 모델 전환

### 1. 모델 출처와 URDF

- `git clone https://github.com/frankaemika/franka_description.git` → `~/effmass_mpc/ref/franka_description`.
  - HEAD `7aeeddc449edf8d62b594f9e36a81da53e7796f9` (2026-08-17, "chore: bump version for release 2.9.0"), `package.xml` version 2.9.0, LICENSE Apache-2.0.
  - `robots/`에는 `fr3`, `fr3v2`, `fr3v2_1` 등이 있다. 지시대로 기본 `fr3`를 썼다. 연구실 FR3의 Arm·시스템 이미지 버전은 아직 모른다.
- **URDF 생성:** 시스템 ROS 2 `xacro`를 썼다. `$(find franka_description)`을 위해 `~/effmass_mpc/goal1/fr3/ament/`에 임시 ament 색인(저장소 심볼릭 링크)을 만들었고, `AMENT_PREFIX_PATH`는 이 명령에서만 지정했다.
  - `xacro robots/fr3/fr3.urdf.xacro hand:=true ee_id:=franka_hand` → `goal1/fr3/fr3_xacro.urdf`(링크 26개, 가속도계 프레임 포함, 메시 19개).
  - cuRobo·Isaac용으로 `package://franka_description/`를 상대 경로로 바꿨다 → `goal1/fr3/fr3_curobo.urdf`. 메시는 `goal1/fr3/meshes`(저장소 `meshes`로 연결)다. 모든 메시 파일의 존재를 확인했다.
- `/opt/ros`, `~/hrc_ws`, `~/indy_ws`는 바꾸지 않았다.

### 2. cuRobo FR3 설정 (`~/effmass_mpc/goal1/fr3/fr3.yml`)

- **방법:** cuRobo `content/configs/robot/franka.yml`(Panda) 템플릿을 그대로 따르고, 링크·관절 이름만 `panda_` → `fr3_`로 바꿨다.
  - 관절·속도 한계: URDF에서 읽는다(FR3 값).
  - 가속도 15, 저크 500: franka.yml 값. FR3 URDF에는 이 한계가 없다.
  - 손가락: FR3 URDF에서 `fr3_finger_joint2`가 `fr3_finger_joint1`의 mimic이므로 `fr3_finger_joint1`만 0.04로 고정했다. cspace는 7관절 + finger1이다.
  - `self_collision_ignore`·`self_collision_buffer`·`attached_object`는 franka.yml과 같다(이름만 변경).
  - Isaac(Kit) 안에서는 YAML을 ASCII로 읽어 한국어 주석에서 실패했다. 그래서 설명은 영어 주석과 `fr3_yml_notes_ko.md`로 나눴다.
- **충돌 구 — Panda 구를 FR3 링크로 옮김** (`goal1/fr3/spheres/fr3_port.yml`, 비교 `goal1/scripts/g1_sphere_eval.py`, `goal1/logs/g1_sphere_eval.json`)
  - **이유:**
    - FR3와 Panda는 관절 원점(링크 길이)이 같아 링크 좌표계가 같다.
    - 링크별 충돌 메시 크기 차는 최대 6 mm다(link5 길이 316.7 vs 310.6 mm).
    - 아래 표처럼 옮긴 구가 FR3 형상을 덮는 정도가 Panda 구가 Panda 형상을 덮는 정도와 같거나 낫다.
    - cuRobo `fit_spheres_to_mesh`(같은 개수, VOXEL_VOLUME_SAMPLE_SURFACE)는 모든 링크에서 덜 덮었다.
  - **오른쪽 손가락:** Panda는 관절 회전 없이 메시를 Rz(π) 회전하고, FR3는 관절이 Rz(π) 회전한다. 그래서 구 중심에 Rz(π)를 적용했다.
  - 덮는 정도: 링크 충돌 형상 표면 4,000점 중 구 밖에 있는 비율, 구 밖 거리 p95.

| 링크 | 구 수 | Panda 구 → Panda 형상 | 옮긴 구 → FR3 형상 | cuRobo 맞춤 → FR3 형상 |
|---|---|---|---|---|
| link0 | 2 | 97.7%, 84.7 mm | 97.9%, 83.8 mm | 99.2%, 148.7 mm |
| link1 | 4 | 44.2%, 26.4 mm | 22.8%, 14.4 mm | 99.2%, 51.4 mm |
| link2 | 4 | 72.9%, 29.8 mm | 56.9%, 18.5 mm | 99.7%, 51.5 mm |
| link3 | 4 | 67.2%, 24.4 mm | 37.4%, 8.0 mm | 99.4%, 50.0 mm |
| link4 | 4 | 76.7%, 24.7 mm | 52.0%, 8.2 mm | 99.0%, 93.7 mm |
| link5 | 13 | 85.4%, 47.1 mm | 63.5%, 12.3 mm | 97.9%, 37.8 mm |
| link6 | 3 | 88.7%, 81.7 mm | 78.2%, 80.5 mm | 98.6%, 78.4 mm |
| link7 | 5 | 57.8%, 6.1 mm | 34.2%, 3.6 mm | 95.8%, 29.7 mm |
| hand | 18 | 86.8%, 10.0 mm | 86.8%, 10.0 mm (같은 메시) | 93.6%, 28.2 mm |
| left/right finger | 2 / 2 | 80.7%, 10.3 mm | 65.4 / 65.0%, 9.4 mm | 87.3 / 82.8% |

  - link0·link6은 원래 Panda 설정에서도 크게 덜 덮는다(cuRobo가 베이스 구를 0.03 m로 줄여 둔 설정, 주석 "#0.07"). 이번에는 Panda와 같게 두었다.

### 3. Panda ↔ FR3 차이

| 항목 | Panda (cuRobo `franka.yml`, `franka_panda.urdf`) | FR3 (franka_description 2.9.0) |
|---|---|---|
| 관절 원점 (링크 길이) | j1 z 0.333, j3 y −0.316, j4 x 0.0825, j5 x −0.0825 y 0.384, j7 x 0.088, j8 z 0.107 | **같음** (cuRobo FK 50 자세 차 0) |
| 관절 1 / 3 위치 | ±2.8973 | ±2.9007 |
| 관절 2 | ±1.7628 | ±1.8361 |
| 관절 4 | −3.0718 ~ −0.0698 | −3.0770 ~ −0.1169 |
| 관절 5 | ±2.8973 | ±2.8763 |
| **관절 6** | **−0.0175 ~ 3.7525** | **0.4398 ~ 4.6216** |
| 관절 7 | ±2.8973 | ±3.0508 |
| 속도 한계 (rad/s) | 2.175 ×4, 2.61 ×3 | 2.62 ×4, **5.26, 4.18, 5.26** |
| 토크 한계 (N·m) | 87 ×4, 12 ×3 | 같음 |
| 손가락 | 독립 2관절, 오른쪽 메시 Rz(π) | finger2 = finger1 mimic, 오른쪽 관절 Rz(π), 충돌 형상은 박스 4개 |
| 충돌 구 수·크기 | 65개(반지름>0 61개), 평균 반지름 0.0379 m | 같음 (옮김) |
| 링크 충돌 메시 크기 | — | 링크별 차 ≤ 6 mm |
| FR3 position-based velocity limits | 없음 | URDF에 태그 있음 (cuRobo는 읽지 않음, 이번에 쓰지 않음) |

### 4. Isaac FR3 (`goal1/scripts/g1_fr3_usd_check.py`, `goal1/logs/g1_fr3_usd_check.{log,json}`)

- Isaac 4.5 자산 서버에 공식 FR3 USD가 있다: `Assets/Isaac/4.5/Isaac/Robots/Franka/FR3/fr3.usd`(17,462,793 B, 2025-01-30). 같은 폴더의 `franka-LICENSE.txt`는 Apache-2.0이다.
- 구조: 관절 루트 `/World/fr3`, 강체 13개(`fr3_link0~8, fr3_hand, fr3_leftfinger, fr3_rightfinger, fr3_hand_tcp`), DOF 9(`fr3_joint1~7, fr3_finger_joint1·2`).
- **[확인] FK:** 무작위 10 자세에서 USD `fr3_hand` 위치 ↔ cuRobo FR3 FK 차 최대 0.001 mm.
- **[확인] USD 관절 한계는 URDF 2.9.0과 다르다.** franka_description 이력에서 `919c533`(2025-09-23 "Extend the joint limits") 이전의 FR3 값과 같다.

| 관절 | USD (도 → rad) | franka_description 919c533 이전 | URDF 2.9.0 (cuRobo 사용) |
|---|---|---|---|
| 1 | ±157.20° = ±2.7437 | ±2.7437 | ±2.9007 |
| 2 | ±102.20° = ±1.7837 | ±1.7837 | ±1.8361 |
| 3 | ±166.20° = ±2.9007 | ±2.9007 | ±2.9007 |
| 4 | −174.30 ~ −8.70° = −3.0421 ~ −0.1518 | −3.0421 ~ −0.1518 | −3.0770 ~ −0.1169 |
| 5 | ±160.80° = ±2.8065 | ±2.8065 | ±2.8763 |
| 6 | 31.20 ~ 258.80° = 0.5445 ~ 4.5169 | 0.5445 ~ 4.5169 | 0.4398 ~ 4.6216 |
| 7 | ±172.80° = ±3.0159 | ±3.0159 | ±3.0508 |

  - 속도 한계(최대 관절 속도)는 USD와 URDF가 같다(2.62 / 5.26 / 4.18 rad/s).
  - cuRobo가 USD 한계 밖을 명령하면 Isaac이 막는다. 연구실 FR3에 어느 한계가 맞는지는 시스템 이미지 버전에 달렸다 [미확인].
- **USD 관절 구동 이득:** 팔 관절 stiffness 60,000, damping 6,000, maxForce 87/12. finger2는 이득 0(mimic).
  - Panda는 cuRobo helper의 URDF 변환을 쓰고, `default_drive_strength` 1,047.2, damping 52.4다.
  - 측정 스크립트는 두 로봇 모두 max effort를 5,000으로 올린다(환경 점검 때와 같음).

### 5. 측정 스크립트 변경 (기본 동작은 Panda 그대로)

- `env_check/scripts/step2_scene.py`: `STEP_ROBOT=panda|fr3`(기본 panda). FR3면 cuRobo `goal1/fr3/fr3.yml`을 쓰고, Isaac은 공식 FR3 USD를 `/World/fr3`에 참조해 `Robot`으로 감싼다(`add_isaac_robot`).
- `step2b_mpc_isaac.py`: `--robot` 인자를 추가했고, 로봇 생성을 `S.add_isaac_robot`으로 바꿨다(Panda는 helper 그대로). `step4_isaac_human.py`도 같다.
- `run_isaac.sh`, `run_isaac_ros.sh`: `env -i` 목록에 `STEP_ROBOT`을 넘기게 했다.
  - 이 변경 전 첫 측정 A "FR3"는 실제로 Panda로 돌았다(`env -i`가 변수를 지움). 그래서 버리고 다시 쟀다.
- 변경 전 사본: `env_check/step4/*.before_goal1*`.

### 6. 측정 결과 (v2 장면, 2단계와 같은 옵션: 예열 없음, 점검 계산 켬, ROS 없음)

**측정 B (Isaac + MPC, headless)** (`goal1/logs/g1_b_v2_*.log`, `env_check/step2/g1_b_v2_*`)

| 실행 | solver 평균 | solver p95 | solver 최대 | 실시간 배율 | infeasible | 도달 | 루프 평균 | 그래프 캡처 |
|---|---|---|---|---|---|---|---|---|
| 2단계 Panda 1회차 | 9.331 | 11.350 | 199.9 | 1.028 | 16 | 6 | 15.90 | step_0 1 |
| 2단계 Panda 2회차 | 9.593 | 11.541 | 203.1 | 1.006 | 16 | 6 | 16.26 | step_0 1 |
| 오늘 Panda 참고 (같은 스크립트) | 8.218 | 9.245 | 186.3 | 1.192 | 16 | 6 | 13.74 | step_0 1 |
| **FR3 1회차** | **14.979** | **17.912** | 187.3 | **0.805** | **974** | **1** | 20.46 | step_0 1 |
| **FR3 2회차** | **14.907** | **17.882** | 189.4 | **0.810** | **974** | **1** | 20.33 | step_0 1 |

- 오늘 Panda가 2단계보다 빠른 것은 [추정] 2단계 때 Slack이 논리 CPU 약 1.7개를 쓰고 있었기 때문이다(4·5단계 기록). Slack은 지금 꺼져 있다.
- **[확인] FR3 경과:**
  - step 226부터 로봇 충돌 구가 움직이는 장애물(mover)에 들어갔다(여유 −6 mm → 최소 −70 mm, 여유 < 0인 스텝 397).
  - 그 뒤 infeasible이 이어지며 "측정 자세 다시 명령"(hold)이 974번 반복되었고, 로봇이 그 자리에 머물렀다.
  - 첫 226스텝 동안 목표까지 거리는 0.61 → 0.57 m로 거의 줄지 않았다(Panda는 step 229에 0.026 m로 도달).
  - 첫 infeasible 전 관절 속도 최대는 FR3 [1.39, 1.12, 0.93, 0.47, 0.50, 0.94, 0.74], Panda [1.90, 1.51, 1.81, 0.56, 0.92, 1.64, 0.90] rad/s였다.

**측정 A (Isaac 없음, cuRobo MPC만, 진단)** (`goal1/logs/g1_a_v2_{panda,fr3}.log`)

| 로봇 | solver 평균 / p95 | infeasible | 도달 | mover 최소 여유 |
|---|---|---|---|---|
| Panda | 6.121 / 6.804 ms | 0 | 8 | 0.0046 m |
| FR3 | 6.216 / 7.120 ms | 0 | 9 | 0.0012 m |

- **[확인]** Isaac 없이 상태를 직전 명령으로 넣으면 FR3 MPC는 Panda처럼 두 목표를 오간다. cuRobo FR3 설정 자체는 정상이다.

**Isaac 위치 명령 추종 (진단, `goal1/scripts/g1_track_check.py`)**
- 조건: step2b와 같은 로봇 생성·max effort 5,000·60 Hz 물리. 관절 1·2·4에 1 / 0.5 / 0.5 rad/s 경사 명령을 1 s 준 뒤 정지한다.

| 로봇 (Isaac 모델) | 경사 중 명령−측정 평균 차 (rad) | 시간 지연 환산 | 5 mrad 이내 정착 |
|---|---|---|---|
| Panda (URDF 변환, stiffness 1,047, damping 52) | 0.0087 / 0.0042 / 0.0055 | 0.009 / 0.008 / 0.011 s | 0.017 s |
| **FR3 (공식 USD, stiffness 60,000, damping 6,000)** | **0.092 / 0.045 / 0.046** | **0.092 / 0.090 / 0.093 s** | **0.300 s** |

- **[확인] 원인:** FR3 USD는 위치 명령을 약 0.09 s 늦게 따라간다. damping/stiffness 비 0.1 s와 같은 크기이고, Panda 변환 모델의 약 10배다.
- **[추정]** MPC가 계획한 궤적이 늦게 실행되면서 실제 로봇이 움직이는 장애물 경로에 머무는 시간이 길어졌다. 충돌 상태(infeasible)가 되면 hold로 멈춰 빠져나오지 못했다.

### 7. 선택지 (사용자 결정 대기, 실행 안 함)

1. **공식 USD를 쓰되, 측정 스크립트에서 이 실행에만 팔 관절 구동 이득을 Panda 변환 모델과 같은 값(1,047.2 / 52.4)으로 맞춘다.** USD 파일은 바꾸지 않는다. 측정 B를 Panda와 같은 시뮬레이션 추종 조건에서 비교할 수 있다.
2. **FR3도 URDF 변환을 쓴다** (`fr3_curobo.urdf`, cuRobo helper, Panda와 같은 importer 설정). 관절 한계가 cuRobo와 같아진다(USD 한계 차 문제도 없어짐). 지시의 "공식 USD 우선"과는 다르다.
3. **공식 USD 이득 그대로 측정 B를 결과로 남긴다.** 실제 FR3는 Franka 내부 제어기가 추종하므로, 시뮬레이션 이득은 실기 추종을 나타내지 않는다.
4. 명령에 속도 목표도 함께 넣는다(MPC `js_action` 속도). 명령 방식이 환경 점검 측정과 달라진다.

- 관절 한계는 USD 한계(구 FR3 값)와 URDF 2.9.0 한계 중 어느 것을 cuRobo에 쓸지도 정해야 한다. 연구실 FR3의 시스템 이미지 버전 확인이 필요하다.

### 8. 결정 반영 (2026-11-16 13:2x)

**결정 1 — Isaac FR3 팔 관절 구동 이득**
- **[확인] 정정:** 앞 보고서(4·7절)에 적은 "Panda 변환 모델 stiffness 1,047.2, damping 52.4"는 cuRobo helper의 `import_config.default_drive_strength`·`default_position_drive_damping` 설정값이다. Isaac 4.5 장면에 실제 기록된 Panda 관절 구동 속성은 다음과 같다(`goal1/scripts/g1_drive_probe.py`).
  - 팔 7관절 모두 stiffness **625**, damping **10**(USD DriveAPI, 각도 기준 단위), maxForce 87/12.
  - articulation view 기준으로는 35,809.9 / 572.96 N·m/rad(= 625·180/π, 10·180/π).
  - 같은 방식으로 FR3 USD 기본값은 60,000 / 6,000(view 3,437,747 / 343,775)이다.
- 지시의 의도("Panda 변환 모델과 같은 값")에 맞춰 **기본값을 625 / 10으로 했다.** 지시에 적힌 1,047.2 / 52.4도 진단으로 함께 쟀다.
- **구현** (`env_check/scripts/step2_scene.py` `set_fr3_arm_gains`):
  - FR3 USD를 참조한 직후, 장면(메모리) 안의 `fr3_joint1~7` DriveAPI stiffness·damping만 덮어쓴다. USD 파일과 손가락 관절은 그대로다.
  - 실행 인자 `step2b_mpc_isaac.py --arm-gains "stiffness,damping"`(또는 `usd` = 덮어쓰지 않음), 환경 변수 `STEP_FR3_ARM_GAINS`(run_isaac*.sh가 넘김)로 바꿀 수 있다.
  - 출처 기록: "환경 점검 Panda 변환 모델과 같은 값(실제 USD 구동 속성 625/10), 목표 2의 2-3에서 실기 추종에 맞춰 다시 정함". 실행 로그에 바꾸기 전후 값을 남긴다.
- **진단 재실행** (`g1_track_check.py`, 지금 FR3 v1 USD, 측정 B는 돌리지 않음):

| FR3 팔 구동 이득 (USD 단위) | 경사 추종 지연 (관절 1 / 2 / 4) | 5 mrad 정착 | 정지 후 오차 |
|---|---|---|---|
| **625 / 10 (기본, Panda 변환 모델 실제 값)** | **0.009 / 0.008 / 0.010 s** | **0.017 s** | ≤ 0.0005 rad |
| 1,047.2 / 52.4 (지시 문구 값) | 0.042 / 0.042 / 0.043 s | 0.117 s | ≤ 0.0006 rad |
| USD 기본 60,000 / 6,000 | 0.092 / 0.090 / 0.093 s | 0.300 s | ≤ 0.0011 rad |
| 참고: Panda 변환 모델 | 0.009 / 0.008 / 0.011 s | 0.017 s | ≤ 0.0012 rad |

  - **[확인]** 625 / 10이면 FR3 추종 지연이 Panda 수준(약 0.01 s)이 된다.

**결정 2 — cuRobo 관절 한계 (좁은 쪽)**
- `goal1/fr3/fr3_curobo_narrow.urdf`: `fr3_curobo.urdf`에서 7관절 위치 한계만 FCI 문서 FR3 값(= Isaac 4.5 USD = franka_description 919c533 이전)으로 바꿨다. 속도·토크 한계는 그대로다.
  - j1 ±2.7437, j2 ±1.7837, j3 ±2.9007, j4 −3.0421 ~ −0.1518, j5 ±2.8065, j6 0.5445 ~ 4.5169, j7 ±3.0159.
- `goal1/fr3/fr3.yml`은 이 URDF를 가리키고, 주석에 "연구실 FR3 시스템 이미지 버전 확인 전까지 보수적으로 좁은 한계 사용"을 남겼다.
- URDF 2.9.0 한계는 `fr3_curobo.urdf`와 `fr3_urdf290.yml`(참고용, 쓰지 않음)로 남겼다.
- retract 자세 [0, −1.3, 0, −2.5, 0, 1.0, 0]은 좁은 한계 안에 있다.

---

## 1-2a-1 조사: Arm v2.2에 맞는 모델 (바꾸지 않음)

**연구실 로봇 (사용자 확인):** Component FR3 Arm, Version 2.2, 생산 2026-07, 팔 질량 18.3 kg, 도달 855 mm. Franka 데이터시트 R02212 2.6(2026-07)은 Arm v2.1과 v2.2를 같은 사양으로 묶는다. Desk arm type·시스템 이미지 버전은 확인 예정이다.

### 조사 1. franka_description 2.9.0 fr3 / fr3v2 / fr3v2_1 비교

- **방법:** 세 모델 모두 `robots/<model>/<model>.urdf.xacro hand:=true ee_id:=franka_hand`로 URDF를 만들었다(`goal1/fr3/variants/`, `goal1/logs/g1_variants_urdf.json`).

| 항목 | fr3 | fr3v2 | fr3v2_1 |
|---|---|---|---|
| 관절 원점 (`kinematics.yaml`) | 같음 | 같음 | 같음 (세 파일 차이 없음) |
| xacro (이름 외) | — | 같음 | 같음 |
| 관절 위치 한계 | 2.9.0 넓은 값 (j1 ±2.9007 …) | fr3와 같음 (소수 자리만 더 김) | fr3v2와 같음 |
| 속도 / 토크 한계 | 2.62 ×4, 5.26/4.18/5.26 / 87·12 | 같음 | 같음 |
| 링크 질량 link0~7 합 | **17.891 kg** | **15.908 kg** | **15.928 kg** |
| 손(hand + 손가락) | 0.713 kg | 0.713 kg | 0.713 kg |
| 전체 (팔 + 손) | 18.604 kg | 16.620 kg | 16.640 kg |
| 라벨 18.3 kg 대비 (link0~7) | −0.41 kg | −2.39 kg | −2.37 kg |
| 충돌 메시 크기 (링크별, fr3 대비) | — | 차 ≤ 4 mm | fr3v2와 같음 |
| 손 장착 | link8 → hand, rpy (0, 0, −π/4) | 같음 | 같음 |
| 구동 설정 (`dynamics.yaml`) | 관절별 마찰·감쇠 | 별도 파일 | 별도 파일 |

**링크별 질량·COM·관성 (fr3 ↔ fr3v2_1)**

| 링크 | 질량 fr3 / fr3v2 / fr3v2_1 (kg) | COM fr3 → fr3v2_1 차 (mm) | 관성 대각 fr3 (ixx, iyy, izz, kg·m²) | fr3v2_1 | 대각 합 비 (v2_1/fr3) |
|---|---|---|---|---|---|
| link0 | 2.397 / 2.397 / 2.397 | 0.0 | [0.009, 0.0115, 0.0085] | [0.009, 0.0115, 0.0085] | 1.00 |
| link1 | 2.927 / 2.438 / 2.438 | 51.0 | [0.0239, 0.0225, 0.0064] | [0.0232, 0.0215, 0.0055] | 0.95 |
| link2 | 2.936 / 2.237 / 2.237 | 112.9 | [0.0419, 0.0251, 0.0617] | [0.019, 0.0057, 0.0171] | 0.32 |
| link3 | 2.245 / 2.193 / 2.196 | 96.1 | [0.0241, 0.0197, 0.019] | [0.0102, 0.0124, 0.0069] | 0.47 |
| link4 | 2.616 / 2.182 / 2.186 | 53.3 | [0.0345, 0.0289, 0.0413] | [0.0089, 0.0076, 0.0103] | 0.26 |
| link5 | 2.327 / 2.187 / 2.191 | 45.8 | [0.0516, 0.0479, 0.0164] | [0.0319, 0.0297, 0.0048] | 0.57 |
| link6 | 1.817 / 1.580 / 1.584 | 37.7 | [0.0054, 0.0141, 0.0161] | [0.0029, 0.0039, 0.0045] | 0.32 |
| link7 | 0.627 / 0.695 / 0.699 | 84.2 | [0.0002, 0.0002, 0.0001] | [0.0009, 0.0011, 0.0007] | 6.03 |


- **[확인]** fr3v2·fr3v2_1은 질량 분포가 크게 다르다(changelog "identify fr3v2 inertials"). link1~6이 가볍고, COM이 37~113 mm 옮겨졌고, 관성 대각합이 0.26~0.57배다. link7만 무겁다(0.627 → 0.699 kg, 관성 6배).
- **라벨 18.3 kg과의 비교:** fr3 link0~7 합(17.89)이 가장 가깝다. fr3v2_1(15.93)은 2.4 kg 적다. 라벨 질량에 무엇이 포함되는지(베이스 케이블, 플랜지 등)는 모른다. 그래서 어느 모델 관성이 실제와 맞는지 이 비교만으로는 판단할 수 없다 [미확인].
- **1-2c 로봇 유효질량에 주는 영향** (`goal1/scripts/g1_variant_meff.py`, `goal1/logs/g1_variant_meff.json`):
  - 계산: numpy로 URDF 링크 질량·관성에서 관절 공간 질량행렬 M(q)를 만들고, m_r(u) = 1/(uᵀ J_p M⁻¹ J_pᵀ u)를 구했다. 자세 6개(retract + 좁은 한계 안 무작위 5), 방향 26개(±x ±y ±z + 무작위 20)다.
  - **검증:** M(q)는 시스템 ROS의 Pinocchio 3.9.0(`/opt/ros/humble`, 새 설치 없음) `crba`와 2 자세에서 최대 차 9e-16이다.

| 점 | fr3 m_r 범위 (중앙) | fr3v2 − fr3 | fr3v2_1 − fr3 |
|---|---|---|---|
| 손 TCP (`hand_tcp`) | 0.90 ~ 8.03 kg (1.55) | 평균 −21.5% (−39.3 ~ −2.3%) | 평균 **−21.2%** (−37.6 ~ −1.4%) |
| 팔꿈치 (`link4` 원점) | 5.95 ~ 557.9 kg (12.91) | 평균 −5.6% (−45.0 ~ +4.2%) | 평균 −5.6% (−47.4 ~ +6.5%) |

  - **[확인]** 손 끝에서는 모델 선택에 따라 로봇 유효질량이 평균 약 21%, 최대 38% 달라진다. retract, +x 방향으로는 fr3 0.90 kg, fr3v2_1 0.70 kg이다.
  - [추정] 환산질량 μ = m_h m_r / (m_h + m_r)에서 m_r이 m_h보다 작은 손 끝 접촉은 μ가 m_r에 거의 비례한다. 그래서 이 차이가 허용 속도 계산에 그대로 들어간다. 팔꿈치처럼 m_r이 큰 곳은 μ가 m_h에 가까워져 영향이 작다.

### 조사 2. Franka 공개 fr3v2_1 USD

- **위치:** GitHub `frankarobotics/franka_simulation` (franka.de "Models & Simulation" 페이지가 가리키는 곳). HEAD `4981709bc0ea495ca8849c76b7fb2b86f8d6f4a8`(2026-08-12).
  - 릴리스 **v0.1.0**(2026-08-12) `fr3v2_1-isaacsim6.tar.gz`(5,560,164 B, sha256 `b46fba7b…d51d18c`)다. 압축 안 `SHA256SUMS` 10개 모두 일치했다.
  - 받은 위치: `~/effmass_mpc/ref/franka_simulation/`(README만), `~/effmass_mpc/ref/franka_simulation_release_v0.1.0/fr3v2_1/`.
- **라이선스:** 저장소에 LICENSE 파일이 없고, README·압축 안 README에도 라이선스 문구가 없다(GitHub API license 없음). **사용 조건 미확인**이다. franka_description은 Apache-2.0이다.
- **버전:** 압축 README에 "Isaac Sim version: 6.0.0-rc.59"가 적혀 있다. 생성 도구는 "URDF USD Converter v0.1.2"이고, franka_ros2·franka_description을 SHA 고정해 만들었다고 한다(README Provenance).
- **Isaac 4.5에서 열기** (`goal1/scripts/g1_fr3v21_usd_check.py`, `g1_fr3v21_open_stage.py`, 측정 없음):
  - `Usd.Stage.Open`은 성공한다. 참조로 넣거나 루트 스테이지로 열어도 파일은 읽힌다.
  - **PhysX 관절 생성은 실패한다:** "CreateJoint - no bodies defined at body0 and body1", "Rigid Body … missing xformstack reset when child of rigid body … in hierarchy". 링크 강체가 서로 안에 중첩된 구조(`Geometry/base/link0/link1/…`)라 Isaac 4.5 PhysX가 관절 루트를 만들지 못한다. 그래서 `Robot`(관절 제어) 생성이 안 된다.
  - **손이 없다:** 강체는 `fr3v2_1_link0~7`(+ link8 프레임)뿐이다("FR3 v2.1 arm, fixed base").
  - **관절 구동 이득:** 팔 7관절 stiffness **0**, damping 0.003, maxForce 87/12, 최대 속도 150.1 / 301.4 / 239.5 °/s(= 2.62 / 5.26 / 4.18 rad/s). 위치 제어용 이득이 아니다.
  - **관절 한계:** j1 ±166.2° (= 2.9007), j2 ±105.2° (= 1.8361), j3 ±166.2°, j4 −176.3 ~ −6.7° (= −3.0770 ~ −0.1169), j5 ±164.8° (= 2.8763), j6 25.2 ~ 264.8° (= 0.4398 ~ 4.6216), j7 ±174.8° (= 3.0508). franka_description 2.9.0의 넓은 한계와 같다.
  - **이름:** 관절 `fr3v2_1_joint1~7`, `fr3v2_1_base_joint`(고정). 링크 `fr3v2_1_link0~8`. 관절 루트 `/fr3v2_1/Geometry/base`.
  - **링크 질량:** URDF fr3v2_1과 같다(link1 2.4377 … link7 0.6987 kg).
- **FK 일치** (`g1_fr3v21_usd_fk.py`): PhysX 관절을 만들 수 없어서, USD 관절의 body0/body1·localPos/Rot·축으로 numpy FK를 계산했다. franka_description fr3v2_1 URDF FK와 비교하면, 무작위 10 자세에서 link7 원점 위치 차 최대 **0.0001 mm**, 회전 성분 차 1.8e-7이다(손이 없어 link7로 비교).

### 조사 3. fr3v2_1로 바꿀 때 바꿀 파일

| 대상 | 바꿀 내용 |
|---|---|
| URDF | `robots/fr3v2_1/fr3v2_1.urdf.xacro hand:=true ee_id:=franka_hand` → `goal1/fr3v2_1/fr3v2_1_curobo.urdf`(+ 좁은 한계면 `_narrow`). 링크·관절 이름은 `fr3v2_1_` 접두어(xacro `arm_prefix`는 앞에 붙기만 함) |
| cuRobo 설정 | `goal1/fr3v2_1/fr3v2_1.yml`: fr3.yml에서 이름 `fr3_` → `fr3v2_1_`, urdf 경로. 나머지(손가락 mimic, self-collision 표, retract)는 같음 |
| 충돌 구 | `fr3_port.yml`의 키 이름만 바꿔 재사용. 같은 방법 평가에서 fr3v2_1 형상 덮는 정도가 fr3와 비슷함(아래). 다시 옮길 필요 없음 |
| Isaac 로봇 생성 (`step2_scene.add_isaac_robot`) | **Franka fr3v2_1 USD는 Isaac 4.5에서 관절을 못 만들고 손이 없음.** 4.5에서는 (a) fr3v2_1 URDF(손 포함)를 cuRobo helper URDF 변환으로 넣거나, (b) Isaac Sim 6으로 올리고 손을 따로 붙이는 수밖에 없다. (a)면 구동 이득은 변환기 결과(Panda와 같은 625/10 예상, 확인 필요)다 |
| 측정·사람 스크립트 | `step2_scene.robot_name` 분기에 fr3v2_1 추가, `step2b --robot` 선택지, `run_isaac*.sh`(변수 전달은 이미 됨), `step4_isaac_human.py`, `step5_5_mpc_rt.py`(설정 경로만) |
| 1-2c 로봇 유효질량 | 관성 출처를 fr3v2_1 URDF inertial로 바꿈. 위 비교처럼 손 끝 m_r이 평균 21% 낮아짐 |
| 관절 한계 | fr3v2_1 원본은 넓은 한계. 결정 2(좁은 한계)를 유지하면 fr3v2_1에도 같은 좁은 URDF를 만듦 |

**지금 옮긴 구가 각 모델 형상을 덮는 정도** (`goal1/scripts/g1_variant_spheres.py`, `goal1/logs/g1_variant_spheres.json`, 표면 4,000점 중 구 밖 비율 / p95 / 최대)

| 링크 | fr3 | fr3v2_1 (fr3v2와 같음) |
|---|---|---|
| link0 | 97.9% / 83.8 / 96.8 mm | 98.2% / 83.2 / 99.3 mm |
| link1 | 22.8% / 14.4 / 27.3 | 27.5% / 17.1 / 30.5 |
| link2 | 56.9% / 18.5 / 31.1 | 53.7% / 20.2 / 32.5 |
| link3 | 37.4% / 8.0 / 19.6 | 57.9% / 10.8 / 20.6 |
| link4 | 52.0% / 8.2 / 20.4 | 71.0% / 9.1 / 16.8 |
| link5 | 63.5% / 12.3 / 18.8 | 48.0% / 12.2 / 18.3 |
| link6 | 78.2% / 80.5 / 89.6 | 81.8% / 80.6 / 89.8 |
| link7 | 34.2% / 3.6 / 11.8 | 39.3% / 4.7 / 12.3 |
| hand, fingers | 같은 메시 | 같음 |

- p95 거리 차는 3 mm 이내이고, 모든 링크에서 Panda 구가 Panda 형상을 덮는 기준(2장 표)보다 좁거나 같다.

---

## 모델 결정 반영: fr3v2_1 (2026-11-16 13:5x)

**결정과 이유 (사용자)**
- franka_description 2.9.0의 **fr3v2_1** URDF 하나를 cuRobo, Isaac, 1-2c 로봇 유효질량 계산의 기준으로 쓴다.
- 이유:
  - 세 모델(fr3, fr3v2, fr3v2_1)은 기구학·한계·충돌 형상이 같고 관성만 다르다. 손 끝 로봇 유효질량이 v1 대비 평균 21% 달라 환산질량에 직접 들어간다.
  - 연구실 로봇은 Arm v2.2이고, Franka가 v2 계열 관성을 새로 식별해 넣었다.
- 라벨 18.3 kg과 fr3v2_1 링크 합(15.93 kg)의 차이는 미확인으로 남긴다. 목표 2에서 libfranka `franka::Model` 질량행렬과 URDF M(q)를 같은 자세에서 비교해 확인한다(아래 목표 2 과제).
- Franka 공개 fr3v2_1 USD(`ref/franka_simulation_release_v0.1.0/fr3v2_1/`)는 **미사용, 라이선스 미확인, Isaac 6용**이다. 받아 둔 파일은 그대로 두었다.

**적용** (`~/effmass_mpc/goal1/fr3v2_1/`)
1. **URDF:** `xacro robots/fr3v2_1/fr3v2_1.urdf.xacro hand:=true ee_id:=franka_hand` → `fr3v2_1_xacro.urdf` → 상대 경로 `fr3v2_1_curobo.urdf`(원래 넓은 한계, 참고용).
   - 관절 위치 한계만 좁은 FR3 FCI 문서 값으로 바꾼 **`fr3v2_1_curobo_narrow.urdf`가 기본**이다.
   - 메시 19개 모두 있음을 확인했다. 링크는 `fr3v2_1_link0~8, fr3v2_1_hand, fr3v2_1_hand_tcp, fr3v2_1_leftfinger, fr3v2_1_rightfinger`다.
2. **cuRobo:** `fr3v2_1.yml`은 `goal1/fr3/fr3.yml`에서 이름(`fr3_` → `fr3v2_1_`)과 경로만 바꿨다(ASCII). 충돌 구 `spheres/fr3v2_1_port.yml`은 `fr3_port.yml`의 키 이름만 바꿨다(같은 구).
3. **Isaac:** cuRobo helper `add_robot_to_scene`로 URDF를 변환했다(Panda와 같은 방식). 변환 결과는 `goal1/fr3v2_1/fr3v2_1_curobo_narrow_temp*.usd`다.
   - **[확인] 변환이 장면에 넣은 팔 관절 구동 이득:** 7관절 모두 stiffness **625**, damping **0.003**(USD 단위), maxForce 87/12다. 실행 중 articulation 기준으로는 kp 35,809.9, kd 0.172다.
     - damping은 fr3v2_1 URDF 관절 `<dynamics damping="0.003" …>`에서 온 것으로 보인다 [추정]. Panda URDF는 `damping="10.0"`이다.
   - **625 / 10이 아니어서 결정 1에 따라 이번 실행에서만 625 / 10으로 맞췄다.**
   - **구현 변경 [확인]:** 처음에는 FR3 USD 때처럼 USD DriveAPI 속성을 덮어썼다. 그러나 URDF 변환 경로에서는 helper가 로봇을 장면에 넣은 뒤라, 속성값만 바뀌고 물리(articulation)에는 반영되지 않았다(kd 0.172 그대로, 추종 결과 동일).
     - 그래서 `step2_scene.apply_arm_gains_runtime(robot)`을 추가했다. `world.reset()` 직후 articulation에 직접 설정하고(view 단위 = USD × 180/π), 전후 값을 로그에 남긴다.
     - FR3 v1 USD도 같은 함수를 거치며 결과는 그대로였다(kd 572.958 유지, 지연 0.009 s).
4. **FK** (`goal1/scripts/g1_fk_isaac_check.py`): cuRobo fr3v2_1 ↔ Isaac 변환 모델의 손(`fr3v2_1_hand`) 위치 차가 무작위 10 자세에서 최대 0.0005 mm, 평균 0.0004 mm다.
5. **추종 지연** (`g1_track_check.py`):

| 로봇·팔 이득 | articulation kd (view) | 경사 추종 지연 (관절 1 / 2 / 4) | 5 mrad 정착 |
|---|---|---|---|
| **fr3v2_1, 625 / 10 (적용)** | 572.958 | **0.010 / 0.009 / 0.011 s** | **0.017 s** |
| fr3v2_1, 변환 그대로 (625 / 0.003) | 0.172 | 0.006 / 0.007 / 0.005 s | 0.017 s |
| fr3 v1 USD, 625 / 10 | 572.958 | 0.009 / 0.008 / 0.010 s | 0.017 s |
| fr3 v1 USD, USD 기본 | 343,775 | 0.092 / 0.090 / 0.093 s | 0.300 s |
| Panda 변환 모델 | 572.958 | 0.009 / 0.008 / 0.011 s | 0.017 s |

6. **측정 스크립트:** `step2b_mpc_isaac.py --robot` 선택지는 `panda`(기본), `fr3`(v1 USD, 비교용), `fr3v2_1`(기준)이다. `STEP_ROBOT`도 같다.

**측정 B (v2, headless, 2단계와 같은 옵션)** (`goal1/logs/g1_b_v2_fr3v21_run{1,2}.log`, `env_check/step2/g1_b_v2_fr3v21_run*`)

| 실행 | solver 평균 | solver p95 | solver 최대 | 실시간 배율 | infeasible | 도달 | 루프 평균 | mover 최소 여유 |
|---|---|---|---|---|---|---|---|---|
| 2단계 Panda 1회차 | 9.331 | 11.350 | 199.9 | 1.028 | 16 | 6 | 15.90 | — |
| 2단계 Panda 2회차 | 9.593 | 11.541 | 203.1 | 1.006 | 16 | 6 | 16.26 | — |
| 오늘 Panda 참고 | 8.218 | 9.245 | 186.3 | 1.192 | 16 | 6 | 13.74 | — |
| **fr3v2_1 1회차** | **8.052** | **9.285** | 188.8 | **1.150** | **0** | **5** | 14.25 | 0.0003 m |
| **fr3v2_1 2회차** | **8.023** | **8.690** | 188.1 | **1.159** | **0** | **5** | 14.15 | 0.0003 m |
| (참고) fr3 v1 USD 기본 이득 | 14.9 | 17.9 | 189 | 0.81 | 974 | 1 | 20.4 | −0.070 m |

- 그래프 캡처는 모두 첫 호출 1회, 루프 중 0회다. 프로세스 VRAM은 2,198 MiB다.
- **판정:** Panda와 같은 범위(infeasible 수십 이하, 도달 5회 이상, 배율 ≥ 1.0)에 들어왔다. → **GUI 확인 직전에서 멈춤.**
- 도달 5회는 Panda(6회)보다 1회 적다. 좁은 관절 한계(관절 6 하한 0.5445 등)와 질량 차이 중 어느 것 때문인지는 확인하지 않았다 [미확인].

**GUI 확인 명령 (실행 안 함)**

```bash
cd ~/effmass_mpc/env_check
scripts/run_isaac.sh $PWD/scripts/step2b_mpc_isaac.py --gui --scene v2 --robot fr3v2_1 --hold -1 --tag g1_b_v2_fr3v21_gui
```

- 확인할 것은 세 가지다:
  - FR3(v2.1) 모양으로 보이는지(손 포함).
  - 두 목표를 오가는지.
  - 움직이는 장애물을 피하는지.

---

## 1-2a-2. 장면 재설계 (사람이 들어갈 장면 g1, 2026-11-16 14:0x)

**장면** (`env_check/scripts/step2_scene.py` `set_scene("g1")`, `step2b --scene g1`)
- cuRobo 충돌 cuboid 두 개만 둔다.
  - `robot_table`: 0.6 × 0.6 × 0.9 m, 중심 (0, 0, −0.45), 윗면 z 0 = 로봇 베이스. 4단계 시각용 탁자와 같다.
  - `floor`: 4 × 4 × 0.1 m, 윗면 z −0.9.
- **없앤 것:** 2단계 table 판(2.2 × 2.2 m), wall, 정적 블록 2개, 움직이는 장애물.
- Isaac: 바닥면 z −0.9, 로봇 탁자 시각 cuboid(갈색), 목표 표시 상자.
- **사람:** 4단계 재생(`human_state_1781524848.npz`)을 4단계 임시 변환(로봇 앞 0.9 m, 바닥 −0.9)으로 놓는다. 사람은 MPC 비용·충돌에 넣지 않았다(표시·GPU 텐서만).
- `step2b_mpc_isaac.py`: 움직이는 장애물 갱신·점검 거리 계산을 장면에 장애물이 있을 때만 하게 바꿨다(v1·v2 동작은 그대로). 변경 전 사본은 `env_check/step4/*.before_g1scene.py`다.

**목표 자세** (`goal1/scripts/g1_goal_search.py`, `goal1/scene_g1.json`, `goal1/logs/g1_goal_search.json`)
- **방법:**
  - 후보는 x 0.30~0.80, y −0.60~0.60, z −0.30~0.70의 0.05 m 격자(5,775점)다. 손(`fr3v2_1_hand`) 방향은 2단계와 같은 (0, 1, 0, 0), 즉 손이 아래를 본다.
  - 사람 조건: 재생 1,262 전 프레임에서 19분절 표면(분절 COM, 4단계 표시 구 반지름)까지 최소 거리 ≥ 0.15 m. 2,228점이 남았다.
  - 로봇 조건: cuRobo IK(fr3v2_1 좁은 한계, 자기충돌·탁자·바닥 충돌 포함, 시드 32)가 성공해야 한다. 1,328점이 남았다.
  - 선택: 오른쪽(y < −0.15)과 왼쪽(y > 0.15)에서 각각 사람 표면 최소 거리가 가장 작은 점(같으면 x가 큰 점)을 골랐다. "로봇이 사람 쪽으로 다가가야 오갈 수 있는 위치"를 이 기준으로 정했다.

| 목표 | 위치 (m) | 방향 wxyz | 사람 표면 최소 거리 (전 프레임) | 가장 가까운 분절 | IK 관절값 (rad) |
|---|---|---|---|---|---|
| 0 (오른쪽) | [0.35, -0.2, 0.2] | (0, 1, 0, 0) | 0.1504 m | L_lowerarm | [-0.06620000302791595, 0.0925000011920929, -0.4287000000476837, -2.7147998809814453, 0.11389999836683273, 2.7969000339508057, 0.18479999899864197] |
| 1 (왼쪽) | [0.5, 0.4, 0.3] | (0, 1, 0, 0) | 0.1502 m | head | [0.6848999857902527, 0.4187000095844269, -0.011099999770522118, -1.8431999683380127, 0.005799999926239252, 2.261899948120117, 1.4564000368118286] |

- **이유:** 재생 사람은 x 방향으로 걷고(골반 x 0.73~1.27 m) 왼팔을 앞으로 뻗어(왼손 최소 x 0.518 m) 앞쪽 공간이 좌우 비대칭이다. 그래서 각 쪽에서 사람에 가장 가까운 허용점이 비대칭으로 나왔다.
- 두 목표 사이를 오갈 때 로봇은 사람 쪽(x 0.35~0.50 m, 사람 앞)을 가로지른다. 목표 자체는 규칙대로 0.15 m 경계에 있다(경계값 0.1504 / 0.1502 m).
- 측정 B와 같은 손 방향만 시험했다. 다른 손 방향은 찾지 않았다.

**측정 (fr3v2_1, 팔 이득 625 / 10, headless)**

| 실행 | 설정 | solver p95 | 루프 평균 | 실시간 배율 | 도달 | infeasible | 사람 데이터 |
|---|---|---|---|---|---|---|---|
| 사람 없이 (`g1_scene_plain`) | 2단계 측정 B와 같은 옵션 | 8.06 ms | 12.03 ms | 1.359 | 5 | 0 | — |
| **(F) 1회차** (`g1_scene_F_run1`) | 4-3d (F): 재생 + 노드 CPU 1스레드·64방향 끔 + ROS + 사람 표시·GPU 텐서, 예열 10, 점검 끔 | 9.43 ms | **12.51 ms** | **1.325** | 5 | 0 | HumanPose 누락 0%·p95 2.56 ms, HumanDynamicsState 30.00 Hz·최대 간격 38.5 ms, 노드 계산 5.4 ms |
| **(F) 2회차** | 같음 | 9.65 ms | **12.69 ms** | **1.306** | 5 | 0 | 누락 0%·p95 2.31 ms, 30.00 Hz·38.9 ms, 노드 5.6 ms |
| 참고: 4-3d (F) (v2 장면, Panda, 3회 평균) | 같은 설정 | 11.95 ms | 16.83 ms | 0.985 | — | — | — |

- **(F) 1회차 루프 항목 (ms):** 장애물 갱신 0.001(4-3d 2.28), 물리 2.69, 상태 읽기 0.46, 사람 처리 0.14, JointState 0.18, solver 8.14, 명령 적용 0.49, 목표 판정 0.35.
- **[확인]** v2 장면 대비 빨라진 주원인은 움직이는 장애물 갱신(스텝당 약 2.2 ms)이 없어진 것이다.
- `~/hrc_ws` 변경 0, `~/.ros` 새 파일 0, `/dev/shm` 0, 남은 프로세스 0이었다.

---

## 1-2a-3. 조사 (1-2b~d 설계용, 구현 안 함)

### 조사 1. 사람 동역학 사본 노드 (설치본 `hrc_body_tracking/human_dynamics_node.py`, `physcis.py` 기준 행)

**분절별 작업공간 역관성 Λ_s⁻¹ = J_p,s M⁻¹ J_p,sᵀ (구조 잠금 반영, ROM 방향 잠금 제외 = 조합 1)**
- **필요한 값과 위치:**
  - 일반화 좌표 D = 60이다. 바닥 위치 3(`base_dofs`)과 19관절 × 오일러 3이다.
  - `analytic_Jp_19` (B, 19, 3, 60): 노드 339행 `physics.compute_J_p_19`(physcis 1403행). 분절 COM 선형 야코비안이다.
  - `Mq_19` (B, 60, 60): 노드 344행이다. 선형 `Mq_linear_19` + 회전 `Mq_angular_19`(342~343행)이고, 관성은 `world_inerita` 338행, 질량·몸 관성은 `mass_inertia()` 524~535행에서 첫 메시지 때 한 번 계산한다.
  - `Minv_contact`: 노드 345~354행 `prepare_meff_constrained_common`(physcis 1788행)이 만든다.
    1. `Minv = (Mq + 1e-6 I)⁻¹` (Cholesky).
    2. **구조 잠금:** `build_structural_locked_cols`(physcis 361행)로 19분절 10·11번 관절(양 어깨)의 3축 전부와 ROM 설정(`/home/robotics/python/physpt/models/joint_ROM.json`, 노드 197행)의 `structural_lock_axes`를 잠근다. `apply_joint_lock_projector_from_cols`(physcis 384·419행 두 번 정의, 뒤 것이 쓰임)가 `Minv − Minv[:,L](Minv[L,L] + λI)⁻¹Minv[L,:]`을 계산한다(잠근 자유도 가속 0).
    3. **접촉:** `apply_contact_projector_from_indices`(physcis 304행, `contact_J_index=[6, 7]` 양발)가 `Minv − Minv Jcᵀ(Jc Minv Jcᵀ + λI)⁻¹ Jc Minv`를 계산한다(발 고정).
  - 조합 1의 분절별 3×3: `Λ_s⁻¹ = J_p[:, s] @ Minv_contact @ J_p[:, s]ᵀ` (B, 19, 3, 3). 19 × (3×60·60×60·60×3) 정도의 작은 계산이다. 노드에는 이 값을 만드는 줄이 없고, 같은 계산이 방향 질의 안(`compute_meff_constrained_from_common` physcis 1839행)에서 ROM 잠금을 더한 `Minv_eff`로 대상 분절 하나에 대해서만 이뤄진다.
  - ROM 방향 잠금은 `apply_sequential_rom_projector_qddot_contact_free`(physcis 66행)가 대상 분절·방향 u·현재 관절각에 따라 순차로 정한다. 조합 1에서는 제외한다.
- **자세·위치 혼합 경합 최소 수정:**
  - 원인 위치: 노드 `MultiThreadedExecutor(3)`(835행), `pose_callback`(553~568행)이 `pose_torch_camera`·`trans_torch_camera`·`betas`를 차례로 덮어쓴다. `timer_callback`(298행~)의 `aa2euler`(576행~)가 583행에서 자세, 595행에서 위치를 따로 읽는다.
  - 수정 1: `pose_callback`에서 한 메시지의 (자세, 위치, betas, stamp, 순번)를 하나의 튜플로 만들어 `threading.Lock` 안에서 통째로 바꾼다.
  - 수정 2: `timer_callback` 시작에서 잠금 안에서 그 튜플을 받아 `has_new_pose = False`로 되돌린다. 계산 전체(`aa2euler`, 속도, 발행 stamp)에 그 튜플만 쓴다.
- **속도 계산 최소 수정:**
  - `calculate_vel_19_body`(499~508행)가 `constants.dt = 1/30`(constants.py 167행)으로 나눈다.
  - 수정: 입력 HumanPose `header.stamp`(sensing 발행 시각) 차이로 dt를 정하고, dt ≤ 0이거나 같은 입력이면 계산·발행하지 않는다(새 입력일 때만 발행). 5-5에서 sensing 주기는 25~30 Hz였으므로 고정 dt는 맞지 않는다.
- **64방향 계산 제거:** 392~399행 `if self.find_worst_vis_enabled:` 분기, `publish_worst_target_meff_direction`(438행~), 설정 읽기(150~170행), `worst_head_pub` 생성을 지운다. 4-3b (C)에서 래퍼로 끈 것과 같은 효과다(노드 계산 31 → 24 ms, GPU 기준).
- **새 메시지 후보 필드** (분절 19개 기준):

| 필드 | 크기 | 노드 내부 출처 |
|---|---|---|
| 입력 순번, 입력 stamp(HumanPose), 계산 stamp | 3 | 새 (수정 1의 튜플) |
| 분절 COM 위치·속도 | 19×3 ×2 | `com_world`(335행), 속도는 수정된 dt |
| 관절 위치 (캡슐 양 끝) | 19×3 (+ 손끝·머리끝 등 말단점) | `joints_19_world`(329행) |
| 분절 회전 | 19×(3×3 또는 쿼터니언 4) | `world_rot`(328행) |
| 분절별 3×3 역관성 Λ_s⁻¹ (구조 잠금·발 접촉 반영) | 19×6 (대칭 성분) | 위 계산 |
| 캡슐 반지름 | 19 | 아래 방법으로 체형마다 한 번 |
| (선택) 분절 질량 | 19 | `mass_19_body` |

**캡슐 반지름을 정하는 방법 (예시 계산, `goal1/logs/g1_capsule_radius.txt`)**
- **방법:** 체형(betas)이 정해지면 한 번 계산한다.
  1. 정지 자세 SMPL-H 정점을 physics.pkl 부위 분할(노드 COM 계산과 같은 분할)로 나눈다.
  2. 분절 축은 부모 관절 → 자식 관절 선분으로 둔다.
  3. 축 선분까지 정점 거리의 백분위(예: p90)를 반지름으로 쓴다.
  4. 정지 자세에서 한 번 구한 값을 분절 강체로 옮겨 쓴다.
- 예시 (SMPL-H male, betas 0, numpy):

| 분절 | 축 (SMPL 관절) | 축 길이 (mm) | 정점 수 | 축 선분까지 거리 p50 / p90 / 최대 (mm) |
|---|---|---|---|---|
| L_thigh | 1→4 | 391 | 237 | 82 / 112 / 118 |
| R_thigh | 2→5 | 400 | 234 | 81 / 109 / 121 |
| L_calf | 4→7 | 419 | 199 | 54 / 82 / 94 |
| R_calf | 5→8 | 419 | 199 | 54 / 85 / 94 |
| L_upper_arm | 16→18 | 272 | 271 | 53 / 70 / 94 |
| R_upper_arm | 17→19 | 265 | 272 | 52 / 71 / 94 |
| L_lower_arm | 18→20 | 263 | 230 | 35 / 51 / 61 |
| R_lower_arm | 19→21 | 268 | 230 | 37 / 49 / 60 |
| L_hand | 20→22 | 91 | 778 | 64 / 103 / 115 |
| R_hand | 21→23 | 91 | 778 | 63 / 100 / 115 |
| L_foot | 7→10 | 141 | 253 | 50 / 77 / 89 |
| R_foot | 8→11 | 143 | 253 | 49 / 72 / 87 |
| abdomen | 3→6 | 145 | 200 | 160 / 175 / 183 |
| chest | 6→12 | 279 | 808 | 155 / 187 / 224 |
| neck | 12→15 | 87 | 118 | 61 / 68 / 73 |
| head | 12→15 | 87 | 1203 | 114 / 146 / 217 |
| pelvis | 0→3 | 120 | 399 | 160 / 191 / 212 |

- **[확인] 예시에서 본 점:**
  - 팔다리는 캡슐에 잘 맞는다(p90: 위팔 70, 아래팔 50, 허벅지 110, 종아리 83 mm).
  - **손**은 축(손목 → SMPL 손 관절)이 91 mm로 짧고 손가락이 그 밖으로 뻗어 p90이 103 mm다. 축 끝을 손끝(정점 최원점)까지 늘리는 쪽이 낫다 [추정].
  - **몸통**(배·가슴·골반)은 넓고 납작해 p90 175~191 mm로 크다. 캡슐 하나보다 캡슐 2개(좌우)나 상자가 맞을 수 있다 [추정].

### 조사 2. 로봇 유효질량 m_r = 1 / (uᵀ J M⁻¹ Jᵀ u)

**관성 출처:** franka_description 2.9.0 `robots/fr3v2_1/inertials.yaml` → fr3v2_1 URDF `<inertial>`(링크 0~7, 손 0.654 kg, 손가락 0.029 kg). URDF 관절 `<dynamics>`에 기어비(120 ×4, 80 ×3)와 모터 관성(4.21e-5 ×2, 3.21e-5 ×5 kg·m²)이 있다.

| 방법 | 위치 | 확인한 시간·결과 | 비고 |
|---|---|---|---|
| (a) Pinocchio 3.9.0 (`/opt/ros/humble`, apt `ros-humble-pinocchio`, 새 설치 없음) | MPC 프로세스 밖(시스템 Python·C++ 노드) 또는 안(Isaac Python에서 import·crba 됨 [확인], Kit 안에서는 미시험) | CPU 1스레드: crba 2.4 µs/상태, 점 1개 m_r 전체 약 34 µs/상태(Python 호출 포함). 12,000 상태면 약 0.4 s | numpy 구현과 M(q) 최대 차 9e-16 [확인]. 현재 상태(또는 몇 개)에는 충분, rollout 전체에는 느림 |
| (b) torch GPU 일괄 (질량행렬·야코비안 직접 구현, `g1_mpc_cost_bench.py`) | MPC 프로세스 안, rollout 텐서와 같은 GPU | 12,000 상태 × 구 61개 역관성 (B, 61, 3, 3): eager 20.9 ms(fk 1.5, M 4.6, 구 야코비안·풀이 14.6). 현재 상태 1개 1.9 ms. horizon 앞 1 / 5 / 10스텝만 2.5 / 5.4 / 9.3 ms(사람 쪽 포함). 추가 메모리 270 MiB | M(q) fp32 ↔ numpy 차 3e-7, 구 중심 ↔ cuRobo `link_spheres_tensor` 0.0003 mm [확인]. 내 구현은 CUDA graph 캡처에 실패(원인 미확인). cuRobo 자체 커널 수준 최적화는 안 함 |
| (c) 기타 | — | — | ① 현재 상태 하나로 구별 역관성을 구해 horizon 전체에 재사용(가장 쌈, 0.675 s 안의 자세 변화는 무시). ② cuRobo 기구학 출력(링크 자세)에 야코비안·M만 덧붙이는 맞춤 CUDA 커널. ③ 실기에서는 libfranka `franka::Model::mass()`로 현재 상태 M(q)를 받아 검증(목표 2) |

- **[확인] 회전자 반사 관성의 영향 (`goal1/logs/g1_rotor_inertia.txt`):** M(q) 대각에 N²·Jm(0.606 / 0.606 / 0.463 / 0.463 / 0.206 / 0.206 / 0.206 kg·m²)을 더하면 m_r이 크게 오른다(자세 6 × 방향 26).
  - 손 TCP: 중앙 1.22 → 4.27 kg, 평균 **+250%**(+45 ~ +603%).
  - link4 원점: 중앙 12.3 → 37.9 kg, 평균 +294%.
  - 링크 관성 모델 차(fr3 ↔ fr3v2_1, 손 끝 평균 21%)보다 훨씬 크다. 1-2c에서 회전자 관성을 넣을지(충돌 순간 관절 탄성 K=7000 때문에 회전자가 얼마나 함께 움직이는지)를 정해야 한다 [미확인].
- **indy_ws Pinocchio 로봇 유효질량 코드 (읽기만)** (`~/indy_ws/src/indy_control_cpp/src/robot_dynamics_node.cpp`):
  - **점:** 링크 1~6 BODY 프레임의 링크 COM(`model.inertias[parentJoint].lever()`)이다. 선형 야코비안 `Jv_com = Jv_o − [R·lever]× Jω`(`getFrameJacobian`, LOCAL_WORLD_ALIGNED)이다.
  - **질량행렬:** `pinocchio::crba` 결과 대각에 기어 반사 관성 `λ = N²·Jm`(기어비 121·121·101·101·101·101, 회전자 관성 "assumed values" 4.75e-5 / 2.52e-5 / 1.41e-5)을 더하고 LDLT로 푼다(`computeMeffWithGear` 296행~, `computeDirectionalMeffForTarget` 553행~).
  - **식:** `A = Jv_com M_aug⁻¹ Jv_comᵀ`, `m = 1/(uᵀ A u)`(`effectiveMassAlong` 242행).
  - **방향:** `collision_assessment_node.cpp`가 사람 분절 COM마다 가장 가까운 로봇 링크 COM을 찾아 `u = (사람 − 로봇)/|·|`로 둔다(1001행 근처). 사람 노드에는 u, 로봇 노드에는 −u로 질의한다(427·435행, 토픽 `/hrc/directional_meff_query`, `/indy/directional_meff_query`).
  - 성능 CSV를 `~/hrc_ws/analysis/realtime_perf/paper_robot_meff_query_perf.csv`에 쓴다(99행). 실행하지 않았다.

### 조사 3. MPC 쪽 계산 (franka-depth-mppi `TargetCostChannel` 방식)

- **TargetCostChannel** (`ref/franka-depth-mppi/experiment/base_target_cost.py`):
  - MPC solver의 각 ParallelMPPI optimizer에 대해 **CUDA graph 캡처 전에** `rollout_fn.cost_fn`을 감싼다.
  - 원래 비용에 `α × PrimitiveCollisionCost(state.robot_spheres)`(사람 패치 cuboid 슬롯만 있는 별도 충돌 세계)를 더한다.
  - 사람 위치는 고정 개수 슬롯의 GPU 텐서를 `update_obb_pose`로 제자리 갱신하고, 주소 불변을 검사한다.
  - 같은 자리(감싼 `cost_fn`)에서 `state.robot_spheres`(400, 30, S, 4)와 `state.state_seq.position`(400, 30, 7)을 받아 아래 항을 계산할 수 있다.
- **텐서 크기** (후보 400 × horizon 30 × 로봇 구 61(반지름 > 0) × 분절 19):
  - 거리 쌍 13.9 M개. 거리만 저장하면 fp32 56 MB, 차 벡터까지 두면 167 MB.
  - 가장 가까운 분절만 남기면 (400, 30, 61) = 0.73 M개다. u 3개, m_h, m_r, μ를 더해 약 18 MB다.
  - 사람 고정 텐서: 위치 19×3, Λ_s⁻¹ 19×3×3(또는 6 성분).
  - 로봇: 상태별 M(q) 12,000 × 7×7, 구 야코비안 12,000 × 61 × 3 × 7(fp32 약 61 MB), 역관성 12,000 × 61 × 3×3.
- **[확인] 측정** (`g1_mpc_cost_bench.py`, `goal1/logs/g1_mpc_cost_bench.json`, eager torch, 같은 GPU, 다른 프로그램 없이):
  - 사람 쪽 (cdist → 최근접 분절 → u → m_h(u), m_r(−u), μ): **4.4 ms**.
  - 로봇 쪽: **20.9 ms**.
  - 추가 메모리 최대 270 MiB(첫 구현에서 큰 블록 행렬을 만들었을 때 1.8 GB).
- **[추정] 60 Hz 예산과의 관계:**
  - 지금 MPC 루프는 g1 장면 (F) 설정에서 12.5 ms다(solver 8.1 ms). 남은 시간은 약 4 ms다.
  - 위 eager 구현을 rollout 전체에 매 스텝 더하면 예산을 넘는다. MPPI 반복 5회마다 비용을 다시 계산하면 그만큼 더 커진다.
  - 줄이는 방법 후보: (1) 로봇 역관성은 현재 상태 하나로 계산해 horizon 전체에 쓰고(1.9 ms eager, 그래프 캡처 시 더 작을 것), 사람 쪽 거리·u·μ만 rollout 전체로 계산(4.4 ms). (2) horizon 앞 몇 스텝만 계산. (3) 맞춤 CUDA 커널로 fk·야코비안·역관성을 한 번에 계산(cuRobo 기구학 커널처럼).
  - 어느 경우든 cuRobo solver의 CUDA graph에 들어가야 하므로 캡처 가능한 구현이어야 한다. 이번 시험 구현은 캡처에 실패했다.
- **기타 [확인]:**
  - link0 구 2개는 야코비안이 0이라 m_r이 무한대가 된다(고정 베이스). 비용에서 제외하거나 상한을 둬야 한다.
  - 다른 구의 m_r은 0.29 kg ~ 무한대(중앙 3.95 kg, 회전자 관성 제외)다.

## 1-2b. 사람 동역학 사본 노드 (2026-11-16, 결정 1-2a-3 반영)

### 1. 만든 것 (`~/effmass_mpc/ws`, 시스템 ROS 2 Humble만 source 해서 colcon 빌드)

| 패키지 | 내용 |
|---|---|
| `effmass_msgs` | `HumanInertiaState.msg`: 입력 순번·stamp·frame_id, 계산 완료 시각(header.stamp), compute_ms, 속도 dt·유효 여부, 분절 19개(이름, 질량, COM 위치·속도, 회전 3×3, 관절 양 끝, **3×3 역관성 Λ_s⁻¹**), 형상 목록(종류, 소속 분절, p0·p1, 반지름) |
| `effmass_human` | `human_inertia_core.py`(ROS 없는 계산), `human_shapes.py`(형상), `human_inertia_node.py`(노드). `physcis.py`·`utils.py`·`constants.py`는 설치본을 그대로 복사(내용 안 바꿈, md5는 `COPIED_FROM.md`) |

- 실행: `goal1/scripts/g1_run_human.sh node` (env -i, `CUDA_VISIBLE_DEVICES=`, OMP·MKL 1, `torch.set_num_threads(1)`, `PYTHONDONTWRITEBYTECODE=1`, `ROS_LOG_DIR`=env_check/ros_log, domain 30, localhost).
- 구독 `/hrc/human_pose`(원본과 같은 QoS), 발행 `/effmass/human_inertia_state`.
- 계산 수식·상수·순서는 원본 `timer_callback`과 같음. 역관성은 조합 1: `prepare_meff_constrained_common`의 `Minv_contact`(구조 잠금 10·11 + ROM json 구조 잠금 축 + 발 6·7 접촉)로 `J_p[s] Minv_contact J_p[s]ᵀ`. ROM 방향 잠금 없음.

수정 4가지:

| 수정 | 방법 |
|---|---|
| 자세·위치 혼합 | `pose_callback`이 (순번, 자세, 위치, betas, stamp, frame_id) numpy 복사본 튜플 하나를 잠금 안에서 통째로 바꿈. 계산은 잠금 안에서 튜플을 꺼내 그것만 씀 |
| 속도 | dt = 입력 HumanPose stamp 차이. 60 Hz로 새 입력을 확인하고, 순번이 같으면 계산·발행하지 않음 |
| 64방향 계산 | 없음 (방향 질의·worst head·~/hrc_ws 성능 CSV도 없음) |
| CPU 1스레드 | device cpu, torch 1스레드 (환경 점검 (F)) |

### 2. 검증 A: 한 프로세스에서 원본 노드와 같은 입력 (`g1_human_validate.py`, `logs/g1_human_validate.{log,json}`)

- 입력: 4단계 재생 파일 `human_state_1781524848.npz`, betas 0, 7번째 프레임마다 건너뜀(1,082 / 1,262).
- 원본: 설치본 `DynamicsNode`를 고치지 않고 만들어 `pose_callback → timer_callback`을 직접 부름. 64방향 계산만 인스턴스 속성으로 끔. 성능 CSV는 goal1/logs로 돌렸고 ~/hrc_ws 쓰기는 감사 훅으로 막음(막힌 쓰기 0건).

| 항목 | 결과 | 기준 |
|---|---|---|
| 위치: 사본 ↔ 원본 | 최대 차 0 m [확인] | |
| 위치: 사본 ↔ 독립 계산(step4_ref_fk, numpy) | 최대 0.0008 mm, 1,082/1,082 1 mm 이내 [확인] | 1 mm 통과 |
| 속도 비 (사본 / 실제 COM 차분), 프레임 차 1 | 중앙 1.000 (p5 0.987, p95 1.011) | |
| 속도 비, 프레임 차 2 (건너뜀 180쌍) | **사본 1.000** (p5 0.980, p95 1.017), **원본 2.000** [확인] | 1.0 통과 |
| m_h(u) 사본 ↔ 원본 함수 (ROM 끔 = `tol_deg −1e9`, +2 kg 없음) | 6,156개(108프레임 × 19분절 × 3방향), 상대 차 최대 7.8e-7, p95 2.4e-7. 값 범위 0.57 ~ 96,600 kg [확인] | 일치 |
| ROM 켬(`tol_deg 5.0`, 원본 질의와 같음) − 끔 | 16.9%가 달라짐, 평균 +2.65%, 범위 −1.7% ~ +269.7% | 기록만 |
| 계산 시간 (CPU 1스레드) | 사본 5.09 / p95 5.14 ms, 원본 timer_callback 4.92 / p95 5.42 ms | |

- 원본 함수의 분모에 1e-6이 더해져 있어 사본도 같은 식으로 비교했다. 차는 float32 반올림 수준이다 [확인].

### 3. 검증 B: ROS 실제 실행 (`g1_human_live.sh`, `logs/g1_live_{nodrop,drop7}/`)

재생(30 Hz) → 사본 노드 → 기록기. `step4_replay.py`에 `--drop-every N`을 더했다(기본 0 = 이전 동작, 백업 `env_check/step4/step4_replay.before_g1.py`).

| 항목 | 건너뜀 없음 | 7번째마다 건너뜀 |
|---|---|---|
| 수신 / 발행 | 1,262 / 1,262 | 1,082 / 1,082 |
| 위치 오차 최대 (독립 계산) | 0.0008 mm | 0.0008 mm |
| 자세·위치 혼합 (이웃 프레임 조합과 일치) | 0 | 0 |
| 출력 주기 | **30.03 Hz**, 간격 p95 34.4 ms, 최대 50.4 ms | 25.74 Hz (입력 30 × 6/7 = 25.7), 최대 68.8 ms |
| 속도 비 프레임 차 2 | | 중앙 1.000 (p5 0.980, p95 1.017) |
| 못 처리한 입력 | 0 | 0 |
| 입력 stamp → 수신 지연 | 평균 7.0, p95 8.7 ms | 평균 22.1 ms (stamp가 건너뜀 없이 흐르는 재생 방식 때문) |
| 메시지당 계산 | 평균 5.18, p95 5.50 ms. 첫 메시지 42.4 ms (체형·형상 맞춤 포함, 그다음 최대 8.4 ms) | 5.15 / 5.55 ms |
| 계산 내역 (평균) | 자세→FK 3.35, 질량행렬·Minv 1.27, 역관성·속도·형상 0.29, 메시지 0.20 ms | 같음 |
| CPU (노드) | 평균 17.1%, 최대 20.2% | 15.3%, 18.3% |
| 스레드 / RSS / GPU | 13 / 510 MiB / 없음 | 같음 |

- 실행 전후 ~/hrc_ws 변경 0줄, ~/.ros 새 파일 0, /dev/shm fastrtps 0, 남은 프로세스 0 [확인].
- 혼합 0건은 이 재생(30 Hz 균일)에서의 결과다. 원본에서 보였던 경합을 재현한 시험은 아니다. 구조상 한 튜플만 쓰므로 섞일 경로가 없다 [확인: 코드].

### 4. 형상 (결정 3)

- 체형이 정해질 때(첫 입력 betas) 정지 자세 SMPL-H male 메시 정점으로 한 번 맞춤. 정점 분할은 노드 COM 계산과 같은 `physics.pkl bodypart_vertexid` + `merge_joint_map`.
- 반지름 = 축(구는 중심)까지 정점 거리의 95 백분위. 그래서 밖 정점 비율은 분절마다 약 5%다.
- 형상은 분절 좌표로 저장하고 매 계산마다 분절 회전·관절 위치로 옮김.

| 분절 | 형상 | 반지름 (m) | 밖 거리 p95 / 최대 (mm) |
|---|---|---|---|
| pelvis | 캡슐 2 | 0.194, 0.186 | 10.2 / 12.0 |
| abdomen | 캡슐 2 | 0.163, 0.161 | 5.8 / 5.9 |
| chest | 캡슐 2 | 0.148, 0.146 | 13.4 / 22.9 |
| neck | 캡슐 | 0.070 | 3.2 / 3.4 |
| head | 구 | 0.132 | **33.6 / 35.9** |
| L/R shoulder | 캡슐 | 0.104 / 0.099 | 6.6 / 4.6 |
| L/R upper_arm | 캡슐 | 0.076 / 0.077 | 15.0 / 13.3 |
| L/R lower_arm | 캡슐 | 0.055 / 0.054 | 6.2 / 5.3 |
| L/R hand (손목→손끝) | 캡슐 | **0.104 / 0.100** | 10.1 / 14.2 |
| L/R thigh | 캡슐 | 0.115 / 0.113 | 3.6 / 7.2 |
| L/R calf | 캡슐 | 0.088 / 0.088 | 6.1 / 5.7 |
| L/R foot (발목→발끝) | 캡슐 | 0.091 / 0.081 | 11.3 / 11.9 |

- 머리 구는 밖 거리가 다른 분절보다 크다. 머리 정점이 한 방향으로 길게 퍼져 있기 때문으로 보인다 [추정]. 결정 3의 "분절당 1~2개" 안에서 캡슐로 바꿀 수 있다.
- 손 캡슐 반지름 0.10 m는 손 두께보다 크다. 정지 자세에서 손가락·엄지가 펴져 있어 축에서 먼 정점이 많기 때문으로 보인다 [추정]. 다음 회차 전에 GUI로 확인하는 것이 좋다.

## 1-2c. 로봇 유효질량 (2026-11-16, 결정 1·2 반영)

### 1. 모듈 (`~/effmass_mpc/ws/src/effmass_robot/effmass_robot/robot_inertia.py`)

- 시스템 Pinocchio 3.9.0 (`/opt/ros/humble/lib/python3.10/site-packages/pinocchio`). 새 설치 없음.
- 모델: `fr3v2_1_curobo_narrow.urdf`. 손가락 2관절을 0.04로 고정한 축소 모델(nv 7).
- 회전자: URDF `<dynamics gear_ratio motor_inertia>`. N = 120 ×4, 80 ×3. Jm = 4.206e-5 ×2, 3.212e-5 ×5. N²Jm = 0.606, 0.606, 0.462, 0.462, 0.206, 0.206, 0.206 kg·m²를 M(q) 대각에 더함. `RobotInertia(rotor=True)` 기본, `rotor=False`로 비교.
- 계산 단위: 충돌 구 61개(`fr3v2_1_port.yml`, cuRobo와 같은 순서, attached_object 4개 제외). 구마다 Λ⁻¹ = J_p M⁻¹ J_pᵀ (3×3).
- link0 구 2개: 야코비안 0 → Λ⁻¹ = 0 (μ = m_h).
- link1 구 4개 중 2개 (0, 0, −0.12), (0, 0, −0.17)는 관절 1 축 위에 있어 역시 Λ⁻¹ = 0 [확인].
- `compute(q)` → (구 중심, Λ⁻¹ (61, 3, 3), M). MPC 스텝마다 현재 상태 하나로 부르고 horizon 동안 고정하는 것은 1-2d에서 호출 쪽이 함.

### 2. 검증 (`g1_robot_validate.py`, `logs/g1_robot_validate.{log,json}`; 상태 201개 = retract + 좁은 한계 안 무작위 200)

| 항목 | 결과 |
|---|---|
| numpy 독립 구현 (URDF XML 직접 파싱, 회전축 야코비안), 50 상태 × 회전자 켬/끔 | 구 중심 4.4e-16 m, M 상대 7.8e-16, **Λ⁻¹ 상대 3.2e-14**, link0 Λ⁻¹ 정확히 0 [확인] |
| cuRobo `link_spheres_tensor` (Isaac Python, 31 상태) | 구 중심 최대 4.1e-7 m (fp32), 순서 같음. 반지름 차 0.004 m = yml `collision_sphere_buffer` (cuRobo가 더함, 모듈은 yml 반지름) [확인] |
| 시간: 구 61개 전체, 한 상태 | **0.20 ms** (회전자 켬 200 µs, 끔 196 µs). 처음 0.52 ms였고, 프레임별 야코비안 한 번 + 구는 벡터화로 줄임 |
| 시간 내역 | crba 1.6 µs, 관절 야코비안·FK 1.2, M 조립 14.7, 7×7 풀이 약 7, 프레임 야코비안 11개 5.0, 프레임 위치 읽기 18.5 µs. 나머지 약 150 µs는 작은 배열 numpy 연산 [추정: 뺄셈] |

회전자 켬/끔 m_r (방향 30개 × 상태 201개):

| 점 | 끔 중앙 (p5~p95) | 켬 중앙 (p5~p95) | 켬/끔 중앙 (최소) |
|---|---|---|---|
| 손 끝 `fr3v2_1_hand_tcp` | 1.11 (0.66~4.81) kg | **4.15** (2.83~12.2) kg | ×3.60 (×1.25) |
| 팔꿈치 (link4 원점) | 12.4 (6.9~90.8) kg | 36.9 (16.3~351) kg | ×2.66 (×1.41) |

링크별 구 중앙값 (상태 51개 × 방향 10개; link1은 무한대 1,020/2,040 제외):

| 링크 | link1 | link2 | link3 | link4 | link5 | link6 | link7 | hand | 손가락 L/R |
|---|---|---|---|---|---|---|---|---|---|
| 끔 (kg) | 562 | 105 | 15.7 | 12.2 | 6.10 | 4.43 | 2.11 | 1.54 | 0.92 / 0.84 |
| 켬 (kg) | 2,961 | 567 | 56.5 | 33.6 | 13.4 | 9.02 | 6.96 | 5.62 | 4.30 / 4.20 |
| 켬/끔 | ×4.7 | ×4.2 | ×3.3 | ×2.6 | ×2.4 | ×2.0 | ×3.5 | ×3.8 | ×4.7 / ×5.1 |

- 1-2a-3 조사 때의 손 끝 1.22 → 4.27 kg은 상태 6개 기준이었고, 이번은 201개 기준이다.

### 3. indy_ws 계산 방식과 비교 (`~/indy_ws/src/indy_control_cpp/src/robot_dynamics_node.cpp`, 읽기만)

| 항목 | indy_ws (`computeMeffWithGear` 296행, `computeDirectionalMeffForTarget` 553행) | 이 모듈 |
|---|---|---|
| 점 | link1~6 BODY 프레임의 부모 관절 관성 COM (`inertias[parentJoint].lever`) | 충돌 구 중심 61개 |
| 점 야코비안 | `getFrameJacobian(LOCAL_WORLD_ALIGNED)`, Jv = Jv_o − [R·lever]× Jω | 같음 (프레임 원점 → 구 중심) |
| 질량행렬 | crba + diag(N²Jm), LDLT | crba + diag(N²Jm), 일반 풀이 |
| 회전자 값 | 가정값 N 121, 121, 101 ×4, Jm 4.75e-5 ×2, 2.52e-5 ×2, 1.41e-5 ×2 (Indy 6축) | URDF 값, 스위치 |
| m | 1/(uᵀAu), \|분모\| < 1e-12 → 무한대 | 같음 (≤ 1e-12 → 무한대) |
| 방향 | collision_assessment: 가장 가까운 로봇 링크 COM → 사람 분절 COM. 사람은 u, 로봇은 −u | 1-2d에서 정함. 부호는 결과에 영향 없음 |

- 같은 점(부모 관절 COM)에 indy 식을 Python으로 그대로 옮겨 이 모듈과 비교했다. link1~7·hand에서 상대 차 최대 8e-16이다. 정의가 같다 [확인].
- indy 식은 프레임 원점이 부모 관절 원점과 같을 때만 COM 점이 맞다. indy가 쓰는 link1~6 BODY 프레임은 같다 [확인: 코드]. fr3v2_1_hand 프레임에 그대로 쓰면 0.107 m 어긋난다 [확인: 측정]. 이 모듈은 구 중심을 쓰므로 해당 없다.

### 4. Isaac Python import (`g1_robot_in_isaac.py`, `logs/g1_robot_in_isaac_{isaac_first,paths_first}.{log,json}`)

- `ROS_METHOD=min run_isaac_ros.sh`. 3단계 min 경로(`STEP3_ROS_PYPATHS`)와 `~/effmass_mpc/ws/install/effmass_robot/lib/python3.10/site-packages`를 sys.path **맨 뒤**에 붙임. run_isaac_ros.sh는 안 바꿈.

| import 순서 | numpy | torch | warp | curobo | 결과 |
|---|---|---|---|---|---|
| Isaac 먼저 → 경로 추가 → pinocchio | 1.26.0 (Isaac pip_prebundle) | 2.5.1+cu118 (ml_archive) | 1.5.0 (kit python) | 0.7.7 (`~/effmass_mpc/curobo/src`) | 파일·모듈 객체 그대로 [확인] |
| 경로 먼저 → Isaac 모듈 | 같은 위치 | 같은 위치 | 같은 위치 | 같은 위치 | 가려지지 않음 [확인] |

- pinocchio 3.9.0은 /opt/ros에서 import 되고 Isaac numpy 1.26.0과 함께 동작했다 [확인]. SimulationApp(Kit)을 띄운 상태의 import는 시험하지 않았다 [미확인].

### 5. 손 끝 μ = m_r m_h / (m_r + m_h)

- m_h: 사본 노드 실제 출력(`g1_live_nodrop/rec.npz`, 재생, betas 0)의 Λ_s⁻¹.
- m_r: 손 끝, 상태 201개.
- 표본마다 프레임·상태·방향 u를 무작위로 고르고, 같은 u로 4,000개를 계산해 중앙값을 냈다.

| 사람 분절 | m_h 중앙 (kg) | μ 회전자 끔 (kg) | μ 회전자 켬 (kg) | μ/m_h 끔 | μ/m_h 켬 |
|---|---|---|---|---|---|
| 오른손 | 0.78 | 0.48 | 0.66 | 0.57 | **0.84** |
| 왼손 | 0.78 | 0.47 | 0.66 | 0.57 | 0.84 |
| 오른 아래팔 | 2.91 | 0.82 | 1.78 | 0.26 | 0.58 |
| 머리 | 6.64 | 0.97 | 2.62 | 0.14 | 0.37 |
| 가슴 | 24.6 | 1.07 | 3.62 | 0.041 | 0.14 |
| 배 | 36.7 | 1.07 | 3.70 | 0.029 | 0.10 |
| 골반 | 30.2 | 1.05 | 3.68 | 0.034 | 0.12 |

- 회전자를 켜면 손 끝 μ가 사람 쪽으로 가까워진다. 손·아래팔은 μ/m_h가 0.5 이상이 된다. 몸통은 여전히 μ ≈ m_r (로봇 쪽이 정함) [확인: 수치].

## 1-2d 준비 (정리만, 실행 안 함)

### A. 사람 쪽 계산 4.4 ms를 줄일 방법

- 4.4 ms 출처: `g1_mpc_cost_bench.py` (H). 12,000 상태 × 구 61 × COM 19 거리(cdist) + 가장 가까운 분절 + 2차 형식 2개. eager, 무작위 Λ_h [확인].
- 결정 2로 로봇 Λ⁻¹는 MPC 스텝마다 현재 상태 하나에서 CPU 0.20 ms로 구하고 horizon 동안 고정한다. 그래서 벤치의 (R) 20.9 ms(12,000 상태 질량행렬·야코비안)는 MPC 안에서 필요 없어진다. GPU에는 (61, 3, 3) 고정 버퍼 복사만 남는다 [확인: 결정 + 이번 측정].

| 방법 | 내용 | 효과 (추정) |
|---|---|---|
| 1. 사람 근처에 올 수 없는 구 생략 | link0 2개·link1 축 위 2개는 Λ⁻¹ = 0이라 μ = m_h로 로봇과 무관. 그 밖에 현재 상태에서 사람 형상까지 거리가 (최대 속도 × horizon 0.5 s + 작동 거리)보다 먼 구는 이번 스텝에서 뺀다. CUDA graph용으로는 고정 개수 K개를 고르는 방식(top-K 인덱스 버퍼) | 계산량 ∝ K/61 [추정] |
| 2. 가장 가까운 분절만 질량 계산 | 이미 argmin 뒤 1개만 2차 형식을 계산한다. 더 줄이려면 거리 후보 자체를 스텝마다 구별 가까운 형상 몇 개(고정 개수)로 미리 좁힌다. 사람 쪽 Λ_h는 노드가 계산해 주므로 MPC 안에서는 gather와 3×3 2차 형식뿐 | 19(→ 형상 22) 대신 후보 수에 비례 [추정] |
| 3. warp 커널 | (rollout, step, 구) 하나당 스레드 하나로 거리 → 최소 → u → uᵀΛu 2개 → μ를 한 커널에서. (B, S, 19) 중간 텐서와 cdist가 없어진다. Isaac warp 1.5.0이 이미 있다 | 메모리 대폭 감소, 시간 감소 [추정, 미측정] |
| 4. horizon 앞쪽만 | 벤치: 앞 1 / 5 / 10스텝만 2.5 / 5.4 / 9.3 ms (로봇 쪽 포함 값) [확인]. 로봇 쪽을 고정하면 더 작아진다 | [추정] |

- 사람 노드 쪽 5.2 ms(그중 자세→FK 3.35 ms)는 별도 프로세스이므로 MPC 예산과 무관하다 [확인: 구조].

### B. CUDA graph 캡처 실패 원인 후보 (`g1_mpc_cost_bench.py`)

| 후보 | 근거 | 가능성 |
|---|---|---|
| 1. `fk()` 안 `ax = T(j["axis"])` (33행) | 호출마다 numpy → CUDA 새 텐서를 만든다. 캡처 중 pageable 메모리에서 GPU로 복사하는 것은 허용되지 않는다. 캡처는 fk에서 실패했다. 이 값은 쓰이지도 않는다 [확인: 코드] | 가장 높음 [추정] |
| 2. 캡처 밖에서 만든 텐서를 바꿔 넣음 | cuRobo solver에 넣을 때 사람 Λ_h·형상 위치·로봇 Λ⁻¹를 새 텐서로 바꾸면 그래프가 옛 주소를 읽는다. 고정 버퍼에 `copy_`해야 한다 | 통합 때 주의 [추정] |
| 3. 동기화가 필요한 연산 | `.item()`, boolean mask, `nonzero`, `torch.linalg.cholesky`(오류 검사). 벤치 함수 안에는 없다(cholesky_ex 사용) [확인: grep] | 낮음 |
| 4. cuBLAS·cdist 작업 공간 | 캡처 중 처음 쓰는 cuBLAS 핸들·작업 공간 할당. 예열 3회 뒤 캡처해서 가능성 낮음 | 낮음 [추정] |

- 지금 벤치 파일은 부분 캡처 시험(`_try`) 앞에서 `sys.exit(0)`으로 끝난다. 다음 회차에 그 줄과 1번 줄을 지우고 부분별로 다시 돌리면 1번을 확인할 수 있다.

## 1-2d. MPC 쪽 공급 (2026-11-16, 진행 중 → Isaac 안 Pinocchio import 실패로 멈춤)

### 0. 결정 반영 (1-2b·1-2c 확인 후)

**형상 (`effmass_human/human_shapes.py`, 형상 22 → 24개, 메시지 상수 `N_SHAPES=24`, GPU 텐서도 24)** — `logs/g1_shape_fit_1-2d.json`

| 분절 | 형상 | 반지름 (m) | 밖 비율 | 밖 거리 p95 / 최대 (mm) |
|---|---|---|---|---|
| 머리 (전: 구) | 구 0.132 | | 5.1% | 33.6 / 35.9 |
| **머리 (결정 1: 위아래 축 캡슐)** | 캡슐, 축 = 정지 자세 SMPL +y, 정점 중심 지남. **길이 0.00 m** | 0.132 | 5.1% | **25.2 / 27.5** |
| 왼손 (전: 캡슐 1) | 0.104 | | 5.0% | 10.1 / 10.9 |
| **왼손 (결정 2: 캡슐 2)** | 손바닥·손가락 + 엄지 | 0.0746, 0.0229 | 4.9% | **8.1 / 8.8** |
| 오른손 (전) | 0.100 | | 5.0% | 14.2 / 15.0 |
| **오른손 (결정 2)** | 손바닥·손가락 + 엄지 | 0.0609, 0.0227 | 5.0% | **7.4 / 8.3** |

- 엄지 정점: SMPL-H(`male.npz`, 노드와 같은 파일, 읽기만) 스키닝 가중치가 가장 큰 관절이 엄지 사슬(왼 34~36, 오른 49~51)인 정점, 손마다 106개.
  - 엄지 사슬 확인: 다섯 손가락 사슬 첫 관절 중 엄지1이 손목에 가장 가까움(56 mm, 나머지 99~114 mm) [확인].
- **머리 캡슐이 길이 0이 됨 [확인]:**
  - 이 체형(betas 0) 정지 자세 머리 정점은 높이 0.24 m, 깊이(앞뒤) 0.24 m, 폭 0.19 m다.
  - 반지름(축까지 거리 p95)이 0.132 m로 높이의 절반보다 커서, 위아래 캡슐이 구와 같아진다.
  - 그래도 중심이 높이 중간으로 옮겨져 p95가 33.6 → 25.2 mm로 줄었다.
- 비교용으로만 잰 다른 축이다 (적용 안 함):
  - 관절 12→15 방향(앞으로 약 38° 기욺): 36.1 mm.
  - 정점 주성분 긴 축(앞뒤): 길이 0.014 m, 반지름 0.118 m, 12.9 mm.
- 손바닥 캡슐 반지름의 좌우 차(0.0746 / 0.0609 m)는 원인을 확인하지 않았다 [미확인]. GUI에서 볼 항목이다.

**ROM 방향 잠금 (결정 3):** 1-2d도 끈 상태(조합 1)다. 1-2b 기록(ROM 켬이면 m_h 표본 16.9%가 달라지고 평균 +2.65%, 최대 +270%)은 **1-4 근거**로 1-2b 2절 표와 목표 1 과제에 남겨 둔다.

### 1. 구현

| 파일 | 내용 |
|---|---|
| `goal1/scripts/g1_meff_gpu.py` | `MeffInputs`: 고정 GPU 입력(분절 COM·속도·Λ_h⁻¹ 19, 형상 24, 로봇 구 Λ_r⁻¹ 61, 나이, τ) + pinned 스테이징 → `copy_(non_blocking)`. `MeffChannel`: 프로토타입 `base_target_cost.TargetCostChannel`과 같이 각 ParallelMPPI optimizer의 `rollout_fn.cost_fn`을 **캡처 전에** 감쌈. **가중치 0 = 원래 비용을 그대로 반환**, 결과는 고정 디버그 텐서(거리, 형상, 분절, u, m_h, m_r, μ, 호출 수)에 기록만 |
| `goal1/scripts/g1_human_link.py` | Isaac 쪽 HumanInertiaState 수신 → Isaac 좌표(p' = Rp + t, Λ' = RΛRᵀ) → MeffInputs. COM 19개 표시(4단계와 같음). 선택: 형상 반투명 표시, 가장 가까운 쌍 선 |
| `env_check/scripts/step2b_mpc_isaac.py` | 옵션 추가 `--human-source inertia`, `--meff on`, `--meff-impl/-dir/-motion/-hfront/-viz/-lines/-dump-steps`. 기본값은 이전 동작(백업 `env_check/step4/step2b_mpc_isaac.before_g1d.py`) |
| `env_check/scripts/run_isaac_ros.sh` | `EFFMASS_WS=1`일 때만 ws의 effmass_msgs·effmass_robot 경로 추가. 기본 환경은 이전과 md5 같음 [확인] (백업 `run_isaac_ros.before_g1d.sh`) |
| `goal1/scripts/g1_meff_bench.py`, `g1_meff_cpu_check.py`, `g1_meff_near.py`, `g1_meff_micro.py`, `g1_human_seq.py`, `g1_meff_run.sh` | Isaac 없는 MPC 시험, CPU 기준 비교, 근접 합성 시험, 커널 단독 시간, 사람 시퀀스, Isaac 실시간 실행 |

계산 정의:
- **거리** = |구 중심 − 형상 축 위 가장 가까운 점| − 형상 반지름 − 로봇 구 반지름. 음수 = 겹침.
  - 로봇 구 반지름은 MPC가 쓰는 값이다. `step2_scene.COLLISION_SPHERE_BUFFER` 0.02로 yml 0.004를 덮어쓴, 프로토타입과 같은 값.
- **가장 가까운 형상** = argmin 거리 (같으면 앞 번호), 분절 = 그 형상 소속.
- **u** 기본: 로봇 구 중심 → 소속 분절 COM. 스위치 `closest`: → 형상 위 가장 가까운 점.
- **질량:** m_h = 1/(uᵀΛ_hu), m_r = 1/(uᵀΛ_ru), μ = 1/(uᵀΛ_hu + uᵀΛ_ru).
  - Λ_r = 0이면 m_r = ∞, μ = m_h.
  - Λ_r = 0 구는 시작 때 Pinocchio 무작위 자세 20개로 정함: 0, 1(link0), 4, 5(link1 축 위) [확인].
- **사람 시간:** `static`(기본)은 현재 위치. `cv`는 분절 COM 속도로 τ_h = 나이 + Σ_{k≤h} traj_dt_k만큼 평행 이동.
  - MPC traj_dt는 균일하지 않다: 1/60 s × 16 → 0.04 s까지 증가, horizon 합 약 0.68 s [확인].
  - 상태 h가 시각 Σ_{k≤h}에 해당한다는 대응은 코드로 확인하지 않았다 [추정].
- **CUDA graph:** 캡처 안 함수에서 호스트 데이터로 텐서를 만들지 않음. `.item()`, `.cpu()`, boolean mask 없음, 크기 고정.
  - warp 구현은 중간 텐서도 없음. torch 구현의 중간 결과는 그래프 메모리 풀에 잡힌다.
- **Λ_r:** MPC 스텝마다 측정 관절 상태로 1-2c 모듈 → 고정 텐서. **Isaac 안에서 import 실패(4절).**

### 2. 검증 (Isaac 없는 MPC 시험, `logs/meff_bench/`)

장면 g1, fr3v2_1, 측정과 같은 MPC 설정, 상태 입력 = 직전 명령(결정적), 사람 = 사본 노드 핵심으로 계산한 재생 1,262 프레임(2 스텝마다 한 프레임), Λ_r = Pinocchio 매 스텝.

**(1) MPC 궤적 = 사람 처리 없음 [확인]:** 채널 끔 2회와 채널 켬 4가지 구현(pairs, skip0, nearest, warp) 600 스텝 명령 관절값이 **비트 단위로 같음**(최대 차 0). 그래프 캡처는 예열 1회뿐(루프 0), 입력·디버그 텐서 주소 불변, 호출 수 3,065(스텝당 5 = n_iters).

**(2) GPU ↔ CPU 기준 (warp 구현):**
- CPU 기준은 GPU 코드를 쓰지 않고 따로 구현했다 (`g1_meff_cpu_check.py`). 구성:
  - 구 중심: rollout 관절값 q[b,h]로 Pinocchio FK.
  - Λ_h: 사본 노드 핵심 출력을 R, t로 옮김.
  - Λ_r: 현재 상태로 Pinocchio.
  - 연산: numpy float64.
- 덤프 12개 × (후보, 시점) 300쌍 × 구 61 = 설정마다 **219,600개**.

| 설정 | 형상·분절 일치 | 거리 abs 최대 | 거리 상대 최대 | u abs 최대 | m_h 상대 최대 | m_r 상대 최대 | μ 상대 최대 |
|---|---|---|---|---|---|---|---|
| u = COM, 정지 | 99.9995% (1개 다름) | 3.7e-7 m | 2.8e-6 | 1.4e-6 | 9.2e-7 | 3.5e-6 | 1.1e-6 |
| u = 가장 가까운 점, 정지 | 99.9995% (1개) | 3.7e-7 m | 2.8e-6 | 1.8e-6 | 1.4e-6 | 5.0e-6 | 1.1e-6 |
| u = COM, 등속 (나이 0.05 s) | 100% | 3.5e-7 m | 2.5e-6 | 1.4e-6 | 2.0e-6 | 2.1e-6 | 1.6e-6 |

- 구 중심 cuRobo ↔ Pinocchio 최대 4.5e-7 m. m_r = ∞ 14,400개는 GPU·CPU 모두 ∞ (불일치 0).
- 다른 1개는 두 형상(골반 4번·배 0번 형상) 거리 차가 3.8e-8 m인 동률이다 (float32 반올림) [확인].
- rollout 표본의 거리는 0.08 m 이상뿐이었다. 그래서 겹침·근접은 합성 시험으로 따로 확인했다 (`g1_meff_near.py`).
  - 실제 사람 프레임 형상 표면 −0.10 ~ +0.15 m에 구를 두었다.
  - 설정마다 366,000개, 그중 겹침 279,051개, 1 cm 미만 23,399개.

| 합성 근접 | 형상 일치 | 거리 abs 최대 | m_h / m_r / μ 상대 최대 |
|---|---|---|---|
| COM, 정지 | 100% | 9.2e-8 m | 1.3e-5 / 8.1e-5 / 2.3e-5 |
| COM, 등속 | 100% | 1.4e-7 m | 8.6e-6 / 2.5e-5 / 1.2e-5 |
| 가장 가까운 점, 정지 | 100% | 9.2e-8 m | **9.1e-4 / 2.0e-3 / 6.0e-4** |

- 가장 가까운 점 방향에서 1e-4를 넘은 것은 21개(0.006%)다. 모두 구 중심이 형상 축에서 0.26 mm 안에 있어 방향이 거의 정해지지 않는 경우다.
  - 축에서 1 mm 이상이면 m_h 상대 차 최대 3.0e-5 [확인].
  - COM 방향(기본)은 모든 표본이 1e-4 이하다.

### 3. 시간 (Isaac 없는 MPC 시험, solver 증가량)

| 단계 | 구현 | solver 평균 (ms) | 끔 대비 | 추가 메모리 |
|---|---|---|---|---|
| 끔 (2회) | — | 6.15 / 6.10 | — | — |
| 0 | pairs: 모든 (구, 형상) 쌍에서 u·질량 계산 후 선택 (torch) | 264.0 | +258 | 1.4 GB |
| 1 | + skip0: Λ_r = 0 구는 로봇 항 생략 (torch) | 268.3 | +262 | 1.6 GB |
| 2 | + nearest: 가장 가까운 형상 하나만 질량 계산 (torch) | 53.5 | +47 | 0.7 GB |
| 3 | + warp 커널 하나 | 8.48 (v1) | +2.35 (따로 실행, 아래 참고) | 0.1 GB |
| 3 (짝 측정) | warp v2 (형상 상수 미리 계산) | 끔 7.53 / 켬 8.11, 끔 7.62 / 켬 8.18 | **+0.58 / +0.56** (중앙 +0.62 / +0.60, p95 +1.21) | 0.1 GB |
| 4 | horizon 앞쪽만 | 쓰지 않음 | | |

- 단계 1 해석: 지시는 "거리만 계산"이다. 결정 2(μ = m_h)를 지키려고 m_h는 계산하고 로봇 항만 생략했다. 4개 구뿐이라 시간 차는 없다.
- torch 단계 0~2가 느린 원인은 따로 분석하지 않았다. 텐서 (400, 30, 61, 24, 3) 여러 개와 einsum으로 보인다 [추정].
- warp 커널 단독(그래프 재생): 호출당 0.226 ms(v1) → 0.204 ms(v2). 스텝당 5회.
- **GPU 클럭 때문에 따로 실행한 프로세스끼리 비교는 믿기 어렵다 [확인].**
  - 같은 설정 끔 실행이 6.1 ms일 때와 8.2 ms일 때가 있었다. 실행 중 SM 클럭이 1,350 MHz(대부분)와 1,710~1,965 MHz 사이를 오갔다 (`nvidia-smi dmon`).
  - 클럭 설정은 바꾸지 않았다.
  - 그래서 한 프로세스에 채널 없는 solver와 있는 solver를 두고 매 스텝 같은 상태로 번갈아 재는 짝 측정을 넣었다 (`--paired 1`).
  - 짝 측정의 두 solver는 난수 흐름이 달라 행동 비교 대상이 아니다. 궤적 동일성은 (1)의 따로 실행 비교로 본다.
- **결론 [확인]:** 단계 3(warp)까지로 Isaac 없는 시험에서 +0.56~0.58 ms다. 단계 4는 쓰지 않았다. 기준(+2 ms 이하)의 최종 판정은 Isaac (F) 실측으로 해야 하는데 아직 못 했다 (4절).
- warp 커널 캐시가 `~/.cache/warp/1.5.0`에 2개 생겼다: `wp_g1_meff_gpu_*`, v1·v2. cuRobo 커널과 같은 위치이고 다른 항목은 그대로다.

### 4. **멈춘 이유: Isaac(Kit) 안에서 Pinocchio import 실패 [확인]**

- 짧은 시험 실행(`g1d_smoke`, 300 스텝, 채널 켬)에서 SimulationApp 뒤 `import pinocchio`가 실패했다:
  `ImportError: /opt/ros/humble/lib/x86_64-linux-gnu/libhpp-fcl.so: undefined symbol: _ZNK6Assimp8IOSystem16CurrentDirectoryB5cxx11Ev`
- 원인 [확인]:
  - Kit 확장 `omni.kit.asset_converter`가 Assimp 5.4.3(`extscache/.../libassimp.so.5`)을 먼저 올린다.
  - Pinocchio의 hpp-fcl은 시스템 Assimp 5.2.0(`/lib/x86_64-linux-gnu/libassimp.so.5`)이 필요하다.
  - soname이 같아 이미 올라온 5.4.3을 쓰다가 기호가 없다.
- 1-2c 4절의 "SimulationApp 안 import [미확인]"이 실제 문제로 확인된 것이다.
- 진단 시험 (`g1_pin_kit_probe.py`, 설정 변경 없음):

| 시험 | Pinocchio | 로봇(fr3v2_1) 로드·DOF | Kit 오류 |
|---|---|---|---|
| SimulationApp **뒤** import (지금 방식) | 실패 (위 오류) | 정상, DOF 9 | 0 |
| SimulationApp **앞** import | 정상, Λ_r 0.69 ms | 정상, DOF 9 | **23**: lula 확장(`liblula_kinematics.so: undefined symbol _ZN4urdf9parseURDFERKSs`, urdfdom 충돌)이 깨져 `omni.isaac.franka`·`omni.isaac.universal_robots` 시작 실패 (우리 스크립트는 안 씀) |

- numpy 독립 구현(1-2c 검증용, Pinocchio와 1e-14 일치)은 지금 형태로 한 상태 12.6 ms다. 매 스텝 쓰기에는 느리다.
- 그래서 하지 못한 것:
  - Isaac (F) 실시간 측정: 채널 없음 기준, 채널 2회.
  - Isaac 안 rclpy·effmass_msgs import 확인. import 블록이 pinocchio에서 먼저 멈췄다.
  - Isaac 실행 덤프 검증, GUI 표시 시험. 코드는 작성했고 Isaac에서 돌려 보지 않았다.
- 시험 실행 뒤 ~/hrc_ws 변경 0, ~/.ros 새 파일 0, /dev/shm 0, 남은 프로세스 0 [확인].

**Λ_r 공급 방법 선택지 (사용자 결정):**

| 안 | 방법 | 장점 | 단점 |
|---|---|---|---|
| A | Isaac 스크립트에서 SimulationApp 전에 Pinocchio import | 같은 프로세스, 현재 상태 그대로, 1-2c 모듈 그대로 | Kit lula 확장 등 시작 실패(23 오류). Assimp·urdfdom 충돌이 다른 Kit 기능(에셋 변환 등)에 주는 영향 미확인 |
| B | Λ_r 별도 프로세스: 시스템 Python + 1-2c 모듈. Isaac이 이미 매 스텝 보내는 `/step3/joint_states`를 받아 Λ_r(61×3×3)을 보냄 | Pinocchio를 검증한 환경 그대로, 충돌 없음 | Isaac은 보통 직전 스텝 상태의 Λ_r을 씀 (1스텝 16.7 ms 늦음 추정). 노드 하나 더, 지연 측정 필요 |
| C | Isaac 안 numpy 구현 (Pinocchio 없이 같은 정의, 벡터화) | 같은 프로세스, 현재 상태 그대로, 충돌 없음. 매 실행 Pinocchio와 오프라인 대조 가능 | 새 구현·최적화 필요 (지금 12.6 ms, 1 ms 이하 가능 추정 [추정]) |

### 5. 결정 반영 (2차): Λ_r = numpy (안 C), 머리 = 주성분 축 캡슐

**Isaac(Kit) 안 Pinocchio 충돌 기록 (환경 기록):**
- Kit 확장이 올린 Assimp 5.4.3·urdfdom과 시스템 Pinocchio(hpp-fcl → Assimp 5.2.0, urdfdom)의 같은 이름 라이브러리가 충돌한다 [확인: 오류 기호·라이브러리 경로].
- 오류 기호(`...B5cxx11Ev`, `_ZN4urdf9parseURDFERKSs`)로 보아 C++ 문자열 ABI 차이(Isaac 4.5 옛 ABI ↔ ROS Humble/Pinocchio 새 ABI)도 겹친 것으로 추정한다 [추정].
- NVIDIA 포럼에도 같은 증상 사례가 있으나 확정 해결책은 없다 (사용자 제공 정보). 사례는 lula `liblula_kinematics.so`의 `_ZN4urdf9parseURDFERKSs`, Pinocchio .so 오류, Pinocchio 로봇 생성 시 세그폴트다.
- 선택지와 이유:
  - A(SimulationApp 전 import): Kit 오류 23개, lula·franka·UR 확장 시작 실패라 쓰지 않음.
  - B(별도 프로세스): 1스텝 지연이 생겨 예비로 둠.
  - **C(numpy) 채택**: 같은 프로세스·현재 상태 그대로, 충돌 없음.

**numpy Λ_r (`effmass_robot/robot_inertia_np.py`, Pinocchio import 없음)** — `logs/g1_robot_np_validate.json`
- 정의는 1-2c와 같다:
  - fr3v2_1 좁은 한계 URDF, 손가락 0.04 고정, 회전자 N²Jm 기본 켬(스위치 유지).
  - M = Σ 링크(m JvᵀJv + JωᵀIJω). 7자유도라 링크 야코비안 합으로 구하며 CRBA와 같은 M이다.
  - 구 61개 점 야코비안을 관절 축에서 한 번에 구하고, Λ⁻¹ = J_p M⁻¹ J_pᵀ.
- 1-2c와 같은 상태 201개에서 Pinocchio와 비교 [확인]:

| 항목 | 회전자 켬 | 회전자 끔 |
|---|---|---|
| 역관성 상대 차 최대 | **1.9e-15** | 7.6e-14 |
| 구 중심 / M 상대 차 | 4.4e-16 m / 7.6e-16 | 4.4e-16 m / 1.1e-15 |
| Λ = 0 구 | 0, 1, 4, 5 (Pinocchio와 같음) | 같음 |

| 단계 (시스템 Python, 중앙값) | FK | 질량행렬 | 역행렬 | 점 야코비안 | 역관성 | 합계 |
|---|---|---|---|---|---|---|
| µs | 32.5 | 71.3 | 10.5 | 46.2 | 10.7 | **171.8** (p99 299.7) |

- 목표 1 ms 이하를 만족해 C를 쓴다. B로 바꾸지 않았다. Pinocchio 모듈은 같은 조건에서 194.5 µs다.
- **이 numpy 구현을 시뮬레이션(Isaac)과 실기(목표 2)에서 공통으로 쓰고, Pinocchio(`robot_inertia.py`)는 오프라인 대조 기준으로 둔다.**
- Isaac 스크립트에서 Pinocchio import를 뺐다. 실행 로그에 "pinocchio 로드 False"로 나온다 [확인].

**머리 형상 (결정 2):**
- 정점 주성분 긴 축(정지 자세에서 앞뒤) 캡슐로 바꿨다. 길이 0.014 m, 반지름 0.118 m, 밖 비율 5.1%, **밖 거리 p95 12.9 mm**(최대 13.8)다. 이전 값은 구 33.6 mm, 위아래 축 25.2 mm.
- 축은 정지 자세(머리 분절 좌표)에 저장된다. 재생 64 프레임에서 축 = world_rot[머리] · 정지 축, 오차 2e-16으로 머리 회전을 따라감을 확인했다 [확인].
- 형상은 24개 그대로다.

**등속 예측 시각 대응 [확인, 고침]:**
- cuRobo `curobolib/cpp/tensor_step_kernel.cu`의 `acceleration_loop(_idx)(_rk2)_kernel`은 `out_pos[0] = start_position`(상태 0 = 현재)이다.
- h ≥ 1에서 `dt[h]`로 적분한다. 그래서 상태 h 시각은 **Σ_{k=1..h} traj_dt_k**다. 이전 정의 Σ_{k≤h}는 1/60 s 길었다.
- `MeffInputs.tau = cumsum(traj_dt) − traj_dt[0]`로 고쳤고, CPU 기준도 같이 고쳤다. 기본(정지) 동작에는 영향 없다.

### 6. Isaac 작업

**(1) import 확인 (`g1_kit_import_probe.py`, fr3v2_1 장면 로드·5 스텝 포함):**

| 모드 | Kit [Error] | numpy·torch·warp·curobo 위치 | 비고 |
|---|---|---|---|
| 아무것도 안 함 | 0 | Isaac 위치 (기준) | |
| SimulationApp **앞**에서 rclpy·effmass_msgs·Λ_r numpy·hrc_interfaces import | 0 | 기준과 같음 | ROS 쪽 import가 끌어온 것은 Isaac numpy(같은 파일)뿐. Pinocchio 미로드 |
| SimulationApp **뒤**에서 같은 import | 0 | 기준과 같음, import 전후 같은 객체 | Pinocchio 미로드 |

- 참고 [확인]: warp를 SimulationApp 전에 직접 import하면 `kit/python/.../warp`, 뒤에 import하면 `extscache/omni.warp.core-1.5.0`(둘 다 1.5.0)이 잡힌다. 측정 스크립트는 뒤에서 import한다.

**(2) 실시간 측정 (장면 g1, fr3v2_1, (F) 설정 + 가중치 0 채널 warp v2, u = COM, 사람 정지, headless)** — `logs/meff_isaac/`

- 사람 쪽: 재생 30 Hz → 사본 노드(CPU 1스레드) → Isaac HumanInertiaState 수신. (F)와 같게 Isaac도 HumanPose를 구독(기록만)한다.
- 실행 스크립트 `g1_meff_run.sh`는 nvidia-smi dmon으로 SM 클럭을 기록한다.

| 실행 | 채널 | solver 평균 / p95 (ms) | 루프 평균 (ms) | 실시간 배율 | SM 클럭 평균 (1,350 MHz 비율) | Isaac CPU 중앙값 |
|---|---|---|---|---|---|---|
| g1d_off_1 | 끔 | 8.10 / 8.67 | 12.47 | 1.329 | 1,350 (100%) | 146% |
| g1d_on_1 | 켬 | 10.38 / 12.39 | 16.90 | **0.980** | 1,541 (52%) | 727% |
| g1d_off_2 | 끔 | 8.32 / 9.74 | 13.34 | 1.242 | 1,754 (29%) | 145% |
| g1d_on_2 | 켬 | 11.04 / 12.19 | 17.87 | **0.927** | 1,350 (100%) | 729% |
| g1d_prof_off (계측) | 끔 | 8.17 / 8.84 | 12.66 | 1.308 | 1,350 (100%) | |
| g1d_prof_on (계측) | 켬 | 10.34 / 11.87 | 16.78 | **0.986** | 1,449 (67%) | |
| g1d_noprof_off (계측 스위치 끔) | 끔 | 8.11 | | 1.323 | 1,350 (100%) | |
| g1d_dump (덤프 11회) | 켬 | 10.85 / 11.80 | 17.69 | 0.937 | 1,350 (100%) | |

**짝 측정 (한 프로세스, 채널 없는 solver와 있는 solver를 같은 측정 상태로 번갈아, 로봇은 채널 solver 명령):**

| 측정 | 채널 없음 평균 / p95 | 채널 있음 평균 / p95 | 증가 평균 / 중앙 / p95 | 클럭 평균 |
|---|---|---|---|---|
| Isaac 없는 시험 1 | 7.53 / 8.90 | 8.11 / 9.44 | +0.58 / +0.62 / +1.21 | |
| Isaac 없는 시험 2 | 7.62 / 8.82 | 8.18 / 9.44 | +0.56 / +0.60 / +1.22 | |
| Isaac g1d_pair_1 | 9.62 / 10.50 | 10.63 / 11.52 | **+1.01 / +1.02 / +2.21** | 1,372 |
| Isaac g1d_pair_2 | 9.69 / 10.60 | 10.70 / 11.77 | **+1.01 / +0.99 / +2.32** | 1,350 |
| Isaac g1d_prof_pair (계측) | 9.53 / 10.43 | 10.51 / 11.43 | **+0.98 / +0.98 / +2.16** | 1,435 |

- 짝 실행은 solver 두 개를 돌려 루프 27 ms, 배율 0.61이다. 배율·루프 판정에는 쓰지 않는다.

**판정:**

| 기준 | 결과 | 판정 |
|---|---|---|
| solver 증가 ≤ 2 ms | 짝 측정 +0.98~1.01 ms. 따로 실행한 끔↔켬은 +2.2~2.9 ms (같은 1,350 MHz 100% 끼리 8.10 → 11.04) | **짝 통과, 실제 실행 불통과** |
| 실시간 배율 ≥ 1.0 | 채널 켬 0.927~0.986 | **불통과** |
| 그래프 재캡처 0 | 모든 실행 루프 중 0 (예열 때만, 짝 2) | 통과 |
| 텐서 주소 불변 | 입력·채널 디버그 텐서 모두 불변 | 통과 |
| 사람 데이터 30 Hz | HumanInertiaState 30.00 Hz, 최대 간격 37~43 ms | 통과 |
| MPC 명령 = 사람 처리 없음 | 측정 관절값이 끔·켬·짝·덤프·계측 켬/끔 모두 g1d_off_1과 같음 (CSV 1e-6 단위) | 통과 |

**(3) 원인 분석 [확인]: 채널 켬일 때 Λ_r numpy의 `np.linalg.inv`가 OpenBLAS 스레드 6개를 계속 바쁜 대기 상태로 만든다**

- 루프 계측(`--loop-prof 1`)은 구간마다 `perf_counter`와 `process_time`만 쓴다. GPU 동기화 추가는 없다. 계측 켬/끔 모두 명령이 같다.

| 구간 (평균 ms) | 끔 벽시계 | 끔 프로세스 CPU | 켬 벽시계 | 켬 프로세스 CPU | 벽시계 증가 |
|---|---|---|---|---|---|
| Isaac 물리 step | 2.71 | 7.62 | 3.67 | 29.37 | **+0.96** |
| 상태 읽기 | 0.41 | 0.46 | 0.55 | 3.91 | +0.14 |
| 사람 메시지 처리 (변환·MeffInputs 복사·표시) | 0.24 | 0.26 | 0.36 | 2.51 | +0.11 |
| Λ_r 계산 (numpy) | — | — | 0.48 | 3.41 | +0.48 |
| Λ_r 업로드 (pinned → GPU copy_) | — | — | 0.04 | 0.28 | +0.04 |
| JointState 발행 | 0.18 | 0.21 | 0.22 | 1.60 | +0.04 |
| 동기화 대기 (solver 앞) | 0.07 | 0.08 | 0.07 | 0.51 | 0 |
| solver | 8.17 | 9.01 | 10.34 | 73.72 | **+2.17** |
| 명령 적용 | 0.49 | 0.54 | 0.60 | 4.28 | +0.11 |
| 목표 판정 | 0.34 | 0.38 | 0.42 | 2.95 | +0.07 |
| 기타 | 0.03 | 0.03 | 0.03 | 0.19 | 0 |

- 채널을 켜면 모든 구간에서 프로세스 CPU가 벽시계의 약 7배가 된다. 스레드 6개가 계속 100%라는 뜻이다.
- 스레드 스냅샷(`top -H`, 루프 중) [확인]: 켬 실행에서 이름 `python3`, 번호가 이어진 스레드 6개가 각 99.5~99.9%였다. 끔 실행에는 없다.
- 진단(`g1_spin_probe.py`, Isaac Python, Kit 없음, 60 Hz 호출 사이 sleep) [확인]:
  - `np.linalg.inv(7×7)`만 불러도 프로세스 CPU/벽시계 6.0, `RobotInertiaNP.compute`도 6.02다.
  - batched matmul·einsum은 0.0, `A@A.T`(7×30)는 0.52, `np.linalg.solve`는 6.0, `np.linalg.cholesky`는 0.62다.
  - Isaac numpy 1.26.0 = OpenBLAS64 0.3.23 (pthreads, `USE_OPENMP=` 빈 값), `OPENBLAS_NUM_THREADS`·`OMP_NUM_THREADS` 미설정, 코어 12개다.
  - 시스템 Python(numpy 1.21)에서는 같은 계산이 171.8 µs였고, Isaac 안에서는 0.48 ms였다.
- 해석:
  - **채널 자체(GPU warp 커널)의 solver 증가는 짝 측정 +1.0 ms**다. 짝의 두 solver는 같은 CPU 경합을 받는다. Isaac 없는 시험 +0.57 ms보다 큰 이유는 CPU 경합에 따른 실행 지연으로 추정한다 [추정].
  - 따로 실행한 끔↔켬의 나머지(+1.2~1.9 ms)와 "원인 미상 1.5 ms"(물리 +0.96, 상태·사람·명령·목표 +0.43)는 OpenBLAS 바쁜 대기 스레드가 CPU를 차지해 생긴 실제 비용이다 [확인: 계측·스냅샷·진단]. GPU 클럭 탓이 아니다.
- **클럭 [확인]:** 같은 1,350 MHz 100% 실행끼리(off_1·prof_off·noprof_off 8.10~8.17 ↔ on_2 11.04)도 +2.9 ms다. off_2는 평균 1,754 MHz인데도 8.32 ms로 off_1과 비슷하다. 이 장면의 Isaac 측정에서 solver 시간은 클럭보다 CPU 쪽에 묶여 있다 [추정].
- **루프 안 GPU 동기화 호출 (우리 코드, 측정 실행 기준):**

| 위치 | 호출 | 스텝당 | 끔/켬 |
|---|---|---|---|
| `measured_state` | `ta.to_device(numpy)` ×2 (pageable → GPU) | 2 | 같음 |
| solver 앞·뒤 | `torch.cuda.synchronize()` | 2 | 같음 |
| solver 뒤 | `bool(feasible.all())`, `bool(isfinite.all())`, `q_cmd.cpu().numpy()` | 3 | 같음 |
| 목표 판정 | `float(pe)`, `float(qe)` | 2 | 같음 |
| MeffInputs | pinned → `copy_(non_blocking=True)` (Λ_r 1·age 1 매 스텝, 사람 10 새 메시지마다) | 0 (동기화 아님) | 켬만 |
| 짝 실행만 | `torch.cuda.synchronize()` | +2 | |
| 덤프·선 표시 | `snapshot()`(synchronize + `.cpu()`), `query().cpu()` | 측정 실행에서는 꺼짐 | |

- 측정 실행(on_1·on_2·prof_on)에서 덤프·디버그 텐서 CPU 읽기·선 표시는 꺼져 있었다 [확인]:
  - 코드상 `record_q = bool(MEFF_DUMP)`이고 덤프는 `--meff-dump-steps`가 있을 때만이다.
  - `snapshot()`은 실행 끝 결과 기록 때 1회뿐이다.
  - 로그에 "형상 표시 none", 덤프 인자 없음이 찍혀 있다.
  - 덤프는 g1d_dump에서만 11회(100 스텝마다) 했다.

**(4) Isaac 실행 덤프 검증 (`g1d_dump`, 덤프 11개, CPU 기준 = numpy float64 + Pinocchio Λ_r, Isaac 밖) [확인]:**
- 표본 201,300개, 형상·분절 일치 100%, 거리 abs 최대 3.7e-7 m, 거리 상대 최대 4.9e-6이다.
- u abs 최대 1.6e-6, m_h / m_r / μ 상대 최대 1.5e-6 / 3.0e-6 / 1.5e-6, m_r = ∞ 13,200개 일치다.
- Isaac 안 numpy Λ_r ↔ Pinocchio 상대 차 1.0e-15다.
- 이 실행 표본의 거리는 0.072 m 이상이었다. 근접·겹침은 2절 합성 시험으로 확인했다.

**(5) 함께 확인:**
- g1d_off_2 노드 종료 코드 1 [확인]: 종료 SIGINT 처리 중 `spin()`이 `RCLError: failed to initialize wait set: the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called`로 끝났다. `main`이 `KeyboardInterrupt`만 잡아서 생긴 것이고 측정 중이 아니다. 고치지 않았다.
- 측정 중 다른 무거운 계산 [확인: 실행 전 `top` 기록]: 계측 실행 전 상위 프로세스는 claude(최대 43.8% 순간), chrome(6.7~18.8%), Xorg(12.5%), anydesk(12.5%), gnome-shell(6.2%)이다. 노트북·학습 계산은 없었다. 앞 8회 측정 중에도 이쪽에서는 다른 계산을 돌리지 않았다.
- 시각화 headless 시험(g1d_viz, 형상·선 모두, 300 스텝): 오류 없음. 구마다 선을 그리면 루프 30.9 ms라 GUI 확인 전용이다.

### 7. 멈춤: 기준 불통과 (실시간 배율 < 1.0, 실제 실행 solver 증가 > 2 ms). 줄이는 방법은 승인 후

| 안 | 내용 | 근거 (진단, 측정 설정에 적용 안 함) | 영향 범위 |
|---|---|---|---|
| 1 | Λ_r의 7×7 역행렬을 LAPACK 없이 numpy로 (Cholesky, 반복 7회) | CPU/벽시계 0.01, 83 µs/호출, `inv`와 상대 차 3e-16. Λ_r 전체 약 0.25 ms 예상 [추정] | Λ_r 모듈만. Pinocchio 대조 다시 함 |
| 2 | Isaac 실행 환경에 `OPENBLAS_NUM_THREADS=1` (run 스크립트 env만) | 미시험 | Isaac 안 모든 numpy BLAS가 1스레드. 시스템 설정 아님 |
| 3 | B: Λ_r 별도 프로세스 | 시스템 Python Pinocchio 0.19 ms | 1스텝 지연 |

### 8. 1안·2안 적용과 최종 측정 (1-2d 완료, 2026-11-16 17:0x)

**결론: 최종 설정(1안 + 2안)에서 채널 추가 solver 증가는 평균 +0.96~1.01 ms(≤ 2 ms)이고, 채널 켬 실시간 배율은 1.178 / 1.191(≥ 1.0)이다. 통과 [확인].**

**1안 (Λ_r 모듈, `robot_inertia_np.py`):**
- `np.linalg.inv`를 LAPACK을 쓰지 않는 numpy Cholesky `spd_inv`로 바꿨다. L Lᵀ → L⁻¹ → L⁻ᵀL⁻¹, 반복 7회씩. 백업은 `logs/robot_inertia_np.before_chol.py`.
- 같은 M에서 `np.linalg.inv`와의 상대 차 최대: 회전자 켬 5.6e-16, 끔 5.6e-15.
- Pinocchio와 비교(201 상태): 역관성 상대 차 1.9e-15 / 7.7e-14, Λ = 0 구 0·1·4·5 같음 [확인].
- 시간(시스템 Python, 중앙값): FK 32.1, 질량행렬 71.0, **역행렬 63.9**(전 10.5), 점 야코비안 46.8, 역관성 10.8, **합계 226 µs**다. Isaac 루프 안에서는 0.44 ms다.

**루프 경로 LAPACK 호출 목록 (Isaac 쪽 스크립트, g1_* 모듈, Λ_r 모듈) [확인: grep]:**

| 위치 | 호출 | 루프 안 | LAPACK | 조치 |
|---|---|---|---|---|
| `robot_inertia_np.py` compute | `np.linalg.inv(M)` 7×7 | 매 스텝 | 예 | **1안으로 교체** |
| `robot_inertia_np.py` point_inv_inertia | `np.linalg.solve` | 아님 (비교용 함수) | 예 | 그대로 (목록만) |
| `g1_human_link.py` 형상 표시 | `np.linalg.norm` (벡터) | GUI 형상 표시 켤 때만 | 아님 | 그대로 |
| `step2_scene.py` sphere_box_clearance | `np.linalg.norm` | 점검 계산(측정에서는 꺼짐) | 아님 | 그대로 |
| `g1_meff_gpu.py`, `step2b_mpc_isaac.py`, `step4_human.py`, `step3_msgs.py` | 없음 (`torch.linalg.norm`은 GPU) | | | |

- `scipy`는 루프 경로에서 쓰지 않는다.
- 참고: `@`(BLAS gemm)는 LAPACK이 아니다. 진단에서 7×30 `A@A.T`가 CPU/벽시계 0.52였다. 2안(1스레드)이 이것까지 막는다.

**2안 (`env_check/scripts/run_isaac.sh`, `run_isaac_ros.sh`만):**
- `OPENBLAS_NUM_THREADS=1`, `OMP_NUM_THREADS=1`을 `env -i` 실행 환경에 넣었다.
- `EFFMASS_THREADS1=0`이면 넣지 않는다 (1안만 비교용).
- 시스템·~/.bashrc는 그대로다. 백업은 `env_check/step4/run_isaac*.before_g1d_threads.sh`.
- `OMP_NUM_THREADS=1`이라 Isaac 안 torch CPU 스레드도 1이 된다. cuRobo 계산은 GPU다.

**warp 커널 캐시:**
- 같은 두 스크립트에서 `WARP_CACHE_PATH=~/effmass_mpc/cache/warp`를 지정했다.
- 새 위치에는 `wp_curobo.geom.transform_935b195`, `wp_curobo.rollout.cost.bound_cost_bfe87a4`, `wp_curobo.rollout.cost.dist_cost_cbb0cd1`, `wp_g1_meff_gpu_a1e1209`가 생겼다 [확인].
- 기존 `~/.cache/warp/1.5.0`은 지우지 않았고, 이후 바뀌지 않았다 (항목 11개, 마지막 16:21:54) [확인].
- 그 11개 항목: `~/.cache/warp`는 2026-11-15 21:00:34에 생겼고, 항목 시각은 모두 이 프로젝트 실행 시각(11-15 22:14 ~ 11-16 16:21)이다.

| 항목 | 시각 | 만든 것 |
|---|---|---|
| wp_g1_meff_gpu_a868ab1, _3aca050, _a1e1209 | 11-16 14:57, 15:07, 16:21 | **1-2d 채널 커널 [확인: 모듈 이름]** |
| wp_curobo.* 6개 (geom.transform 2, bound_cost 3, dist_cost 2 중 6) | 11-15 22:14~22:15, 11-16 13:48 | cuRobo 실행 (환경 점검·목표 1) [추정: 시각] |
| wp_isaacsim.core.utils.fabric_034658a | 11-15 23:48 | Isaac 실행 [추정: 시각] |

**최종 측정 (장면 g1, fr3v2_1, (F) 설정, 예열 10, 시각화·덤프 끔, 루프 계측 켬, 스레드 스냅샷)** — `logs/meff_isaac/campaign3.txt`, `campaign3_analysis.jsonl`

| 실행 | 설정 | solver 평균 / p95 (ms) | 루프 평균 / p95 (ms) | 실시간 배율 | SM 클럭 평균 (1,350 비율) | Isaac CPU 중앙값 |
|---|---|---|---|---|---|---|
| g1d_c1_on (600 스텝) | **1안만**, 켬 | 9.10 / 9.65 | 14.00 / 15.13 | 1.183 | 1,350 (100%) | **172%** (전 727%) |
| g1d_f_off_1 | 최종, 끔 | 8.13 / 8.69 | 12.55 / 13.37 | 1.319 | 1,350 (100%) | 205% |
| g1d_f_on_1 | 최종, 켬 | 9.09 / 9.72 | 14.05 / 15.68 | **1.178** | 1,350 (100%) | 193% |
| g1d_f_off_2 | 최종, 끔 | 8.02 / 8.55 | 12.44 / 13.32 | 1.331 | 1,350 (100%) | 200% |
| g1d_f_on_2 | 최종, 켬 | 9.02 / 9.59 | 13.90 / 14.83 | **1.191** | 1,350 (100%) | 199% |

| 짝 측정 (최종 설정) | 채널 없음 평균 / p95 | 채널 있음 평균 / p95 | 증가 평균 (판정) | 증가 p95 (기록만) | 클럭 평균 |
|---|---|---|---|---|---|
| g1d_f_pair_1 | 7.85 / 8.54 | 8.87 / 9.54 | **+1.01** | +2.09 | 1,369 |
| g1d_f_pair_2 | 7.79 / 8.48 | 8.78 / 9.51 | **+0.99** | +2.05 | 1,385 |

- **1안만으로 바쁜 대기 스레드가 사라졌다 [확인]:** 루프 중 `top -H`에 100% `python3` 스레드가 없다. 주 스레드 98.5%, carb.tasking 각 3% 안팎이다. 최종 설정 켬 실행도 같다.
- **2안이 채널 끔 쪽을 나쁘게 하지 않았다 [확인]:** 같은 1,350 MHz 100% 끔 실행끼리 비교했다.
  - 2안 전: off_1 8.10, prof_off 8.17, noprof_off 8.11 ms, 배율 1.329 / 1.308 / 1.323.
  - 2안 후: f_off 8.13 / 8.02 ms, 배율 1.319 / 1.331.
- 루프 항목별 (f_off_1 → f_on_1, 벽시계/프로세스 CPU ms):
  - solver 8.13/8.9 → 9.09/10.0 (+0.96)
  - Λ_r 계산 0.44/0.5, 업로드 0.04/0.0
  - 물리 2.67/7.5 → 2.70/7.5
  - 상태 0.41 → 0.41, 사람 처리 0.23 → 0.26, 명령 0.49 → 0.50, 목표 0.34 → 0.34
  - CPU/벽시계 비가 끔과 같아졌다 (전 켬 7배).
- 그 밖 [확인]:
  - MPC 명령(측정 관절값)이 모든 실행에서 g1d_off_1(계측·1안·2안 전)과 같다 (CSV 1e-6 단위).
  - 루프 중 그래프 재캡처 0 (예열 때만, 짝 2), 입력·채널 텐서 주소 불변.
  - infeasible 0, 도달 5 (1안만 600 스텝은 3), 사람 데이터 30.00 Hz.
- 실행 전 상위 프로세스는 claude·chrome·Xorg·anydesk·gnome-shell이었고, 무거운 계산은 없었다 (campaign3.txt 기록).
- g1d_f_on_2 노드 종료 코드 1도 같은 SIGINT 처리 중 `RCLError`다 (측정 뒤, 5절과 같음).

### 9. GUI 확인 준비 (headless 시험 통과, GUI 직전 멈춤)

- 선 표시 옵션 `--meff-lines topk --meff-lines-k K`(기본 topk, K = 5)를 넣었다. 매 2 스텝, 현재 로봇 구 61개 중 거리가 가장 짧은 K쌍을 그린다. 선은 로봇 구 중심 → 소속 형상 축 위 가장 가까운 점이다.
  - `min`(1쌍)과 `all`(61쌍)도 남겼다. 100 스텝마다 가장 가까운 쌍(구, 형상, 분절, 거리, m_h, m_r, μ)을 로그에 쓴다.
- headless 시험(`g1d_guiprep_headless`, 메시 usd·형상·topk 5 선, 300 스텝): 오류 없음, 메시 최종 확인 `usd_equal True`, 재캡처 0.
  - 루프는 형상·선 갱신 때문에 길어진다 (g1d_f_viz 20.8 ms). GUI 확인 전용이다.
- 확인할 것: 사람 메시 위의 형상, 가장 가까운 쌍 선, 손바닥 캡슐 좌우 반지름 차.
  - 머리 주성분 축 캡슐, 손 2캡슐(손바닥·손가락, 엄지), 몸통 캡슐 2개씩.
  - 손바닥 캡슐 반지름 0.0746 / 0.0609 m.

```bash
cd ~/effmass_mpc/goal1
RUN_ISAAC_NO_DISPLAY=0 REPLAY_ARGS=--vertices scripts/g1_meff_run.sh g1d_gui \
  --meff on --meff-viz both --meff-lines topk --meff-lines-k 5 --human-mesh usd --mesh-offset node --gui --hold -1
```

- 재생은 42 s(1,262 프레임) 뒤 끝난다. 그 뒤 사람은 마지막 자세로 멈추고, 창을 닫을 때까지 MPC는 계속 돈다.

## 목표 2 과제 (1-2에서 발견)

- **실기 FR3의 libfranka `franka::Model` 질량행렬과 URDF fr3v2_1 M(q) 비교로 관성 출처 검증.** 라벨 팔 질량 18.3 kg과 fr3v2_1 링크 합 15.93 kg의 차이(2.4 kg)를 확인한다.
- (목표 1 과제와 연결) 실기 추종 지연을 재고 Isaac 팔 이득(지금 625 / 10)을 2-3에서 다시 정한다.

## 목표 1 과제 (1-2에서 발견)

- **Isaac 관절 추종 지연이 MPC 회피 성능을 크게 바꾼다.**
  - USD 기본 이득(추종 지연 약 0.09 s)에서 측정 B는 infeasible 974/1,200, 도달 1이었다. 움직이는 장애물에 들어간 뒤 hold(측정 자세 재명령)에 갇혔다.
  - 이득을 Panda 변환 모델 수준(지연 0.009 s)으로 맞추는 것은 시뮬레이션 비교용 임시 조치다.
  - 노트 21(지연)과 목표 2의 2-3(실기 추종·지연 맞춤)과 연결된다. 실기 FR3의 명령 → 실제 위치 지연을 재고, 그 값으로 시뮬레이션 이득(또는 MPC 지연 보상)을 다시 정해야 한다.
  - hold 방식이 충돌 상태에서 빠져나오지 못하는 문제(환경 점검 2단계 과제 "제자리 유지 중 처짐 누적"과 관련)도 함께 본다.
- **모델 관성 출처:** fr3(v1)과 fr3v2_1 관성이 크게 달라 손 끝 로봇 유효질량이 평균 21% 달라진다. Arm v2.2에 맞는 관성 출처를 정해야 1-2c 결과를 믿을 수 있다.
- **(기록 유지) 측정 A 첫 실행:** `run_isaac.sh`의 `env -i`가 `STEP_ROBOT`을 지워 "FR3" 측정 A가 Panda로 돌았다. `run_isaac.sh`·`run_isaac_ros.sh`에 `STEP_ROBOT`(이후 `STEP_FR3_ARM_GAINS`도) 전달을 추가하고 다시 쟀다(6절).
- **(1-2b) 원본 노드 속도:** 원본 `human_dynamics_node`는 고정 dt(1/30)로 나눠, 입력이 한 프레임 건너뛰면 속도가 2배가 된다(검증 A에서 2.000). 사본은 stamp 차로 고쳤다. ~/hrc_ws 원본은 안 바꿈.
- **(1-2b) ROM 방향 잠금 (1-4 근거로 유지):** 켜면 m_h 표본 16.9%가 달라지고 평균 +2.65%, 최대 +270%까지 커진다. 1-2d까지는 끈 상태(조합 1, 사용자 결정). 1-4에서 이 수치를 근거로 다시 본다.
- **(1-2b) 형상:** 머리 구의 밖 거리 p95 33.6 mm, 손 캡슐 반지름 0.10 m. GUI 확인과 형상 재조정(머리 캡슐 등)이 필요한지 정해야 한다.
- **(1-2c→1-2d) SimulationApp 안 import:** 1-2d에서 실패 확인 (1-2d 4절). Λ_r 공급 방법 결정 필요.
- **GPU 클럭 변동(1,350 ↔ 1,700~1,965 MHz)으로 따로 실행한 측정끼리 solver가 약 2 ms 달라짐 → 이후 비교는 짝 측정 또는 클럭 기록 필수.**
  - Isaac 없는 시험에서 관찰했다. Isaac g1 (F) 측정에서는 클럭보다 CPU 경합이 더 컸다 (1-2d 6절).
- **(1-2d) Isaac numpy(OpenBLAS64 0.3.23 pthreads)의 LAPACK 호출(`inv`, `solve`)은 호출 뒤 스레드 6개가 바쁜 대기를 한다.** 같은 프로세스의 Isaac 물리·cuRobo CPU 쪽이 느려진다. Isaac 안 numpy 코드(목표 2 실기 포함)에서 LAPACK 호출을 피하거나 스레드 수를 제한해야 한다.
- **(환경 기록) Isaac(Kit) 안 Pinocchio 충돌:** Kit Assimp 5.4.3·urdfdom ↔ 시스템 Pinocchio(hpp-fcl, Assimp 5.2.0)의 같은 이름 라이브러리 충돌, C++ 문자열 ABI 차이도 추정. NVIDIA 포럼에 같은 증상 사례가 있으나 확정 해결책은 없다.

### 9. 파일

- FR3 모델: `~/effmass_mpc/goal1/fr3/` (`fr3_xacro.urdf`, `fr3_curobo.urdf`, `fr3.yml`, `fr3_yml_notes_ko.md`, `spheres/fr3_port.yml`, `spheres/fr3_fit.yml`(비교용), `meshes`, `ament/`)
- 스크립트: `~/effmass_mpc/goal1/scripts/` (`g1_sphere_eval.py`, `g1_fr3_check.py`, `g1_fr3_usd_check.py`, `g1_track_check.py`, `g1_drive_probe.py`, `g1_variant_spheres.py`, `g1_variant_meff.py`, `g1_fr3v21_usd_check.py`, `g1_fr3v21_usd_fk.py`, `g1_fr3v21_open_stage.py`)
- FR3 변형 URDF: `~/effmass_mpc/goal1/fr3/variants/{fr3,fr3v2,fr3v2_1}.urdf`, 좁은 한계 `goal1/fr3/fr3_curobo_narrow.urdf`
- **기준 모델 fr3v2_1:** `~/effmass_mpc/goal1/fr3v2_1/` (`fr3v2_1_xacro.urdf`, `fr3v2_1_curobo.urdf`(넓은 한계, 참고), `fr3v2_1_curobo_narrow.urdf`(기본), `fr3v2_1.yml`, `spheres/fr3v2_1_port.yml`, `meshes`, Isaac 변환 `*_temp*.usd`)
- 추가 스크립트: `g1_fk_isaac_check.py`, `g1_goal_search.py`, `g1_mpc_cost_bench.py`, `g1_pin_in_isaac.py`
- 장면 g1 목표: `~/effmass_mpc/goal1/scene_g1.json`
- 로그: `~/effmass_mpc/goal1/logs/`
- 참고 저장소: `~/effmass_mpc/ref/franka_description` (Apache-2.0), `~/effmass_mpc/ref/franka_simulation` (README만, 라이선스 표시 없음), `~/effmass_mpc/ref/franka_simulation_release_v0.1.0/fr3v2_1/` (릴리스 압축 풀어 둠)
- **1-2b·1-2c 작업공간:** `~/effmass_mpc/ws/` (`src/effmass_msgs`, `src/effmass_human`, `src/effmass_robot`, colcon `build/ install/ log/`)
- 1-2b·1-2c 스크립트: `goal1/scripts/g1_run_human.sh`(실행 환경), `g1_human_validate.py`, `g1_human_live.sh`, `g1_human_rec.py`, `g1_human_live_analyze.py`, `g1_robot_validate.py`, `g1_robot_in_isaac.py`. `env_check/scripts/step4_replay.py`에 `--drop-every` 추가(백업 `env_check/step4/step4_replay.before_g1.py`)
- 1-2b·1-2c 로그: `goal1/logs/g1_human_validate.{log,json}`, `goal1/logs/g1_live_{nodrop,drop7}/`(rec.npz, node_perf.csv, sys_monitor.csv, analysis.json), `goal1/logs/g1_robot_validate.{log,json}`, `goal1/logs/g1_robot_in_isaac_*.{log,json}`. 이 보고서 수정 전 백업 `goal1/logs/step1_2_report.before_1-2bc.md`

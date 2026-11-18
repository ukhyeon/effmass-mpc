# 공개 자료 처리표 (analysis/ 의 그림·GIF·영상)

공개 저장소의 `analysis/` 는 이전 식(모든 부위에 같은 기준 환산질량 μ_ref 0.53 kg)으로 만든 결과다. 지금 식은 부위별 기준(그 부위의 ISO/TS 15066 기준 질량과 비교)이다.
아래는 파일마다 "그대로 둠 / 문구만 고침 / 부위별 기준으로 다시 만듦" 중 하나로 정한 것이다. 다시 만든 파일은 `goal1/public_draft/analysis_remake/` 에 있다.
`analysis/` 자체는 바꾸지 않았다.

판단 기준:
- 사람 자세나 장면만 보여 주고 구역 식과 무관하면 그대로 둔다.
- 구역 식에 따라 그림이 바뀌고, 부위별 기준으로 같은 내용을 이미 계산했거나 계산이 짧으면 다시 만든다.
- 다시 돌리려면 실행 묶음 전체(수십 회)가 필요한 경우는 문구만 고친다. 그림·영상이 이전 식 결과라고 밝힌다.

| 파일 | 내용 | 처리 | 이유 | 새 파일 |
|---|---|---|---|---|
| figures/zone_3d.png | 손의 구역 표면: 균일 6.5 cm, 균일 16.5 cm, 이전 식 | 다시 만듦 | 구역 모양이 식에 따라 바뀜. 부위별 기준 구역 표면을 이미 계산함 | analysis_remake/figures/zone_3d.png (균일 대 부위별) |
| figures/postures.png | 자세 7개 미리보기 | 그대로 둠 | 사람 자세만 보여 줌. 식과 무관 | - |
| figures/zone_thickness_by_posture.png | 자세·방향별 구역 두께 | 다시 만듦 | 두께가 식에 따라 바뀜. 부위별 측정으로 다시 계산해 둠(표시 단위가 방향별에서 부위별로 바뀜) | analysis_remake/figures/zone_thickness_by_part.png |
| figures/elbow_angle_zone.png | 팔꿈치 각도 0·30·90도의 구역 단면 | 다시 만듦 | 짧은 계산으로 같은 그림을 다시 그릴 수 있음 | analysis_remake/figures/elbow_angle_zone.png |
| figures/static_paths_zone.png | 정면·위 장면의 구역과 로봇 경로 | 다시 만듦 | 부위별 기준 실행(V2 정면·위 seed 0)의 경로 기록이 있어, 원본 그림 코드를 부위별 식으로 옮겨 다시 그림 | analysis_remake/figures/static_paths_zone.png |
| figures/static_scene_gap.png | 정지 장면별 간격 막대 | 다시 만듦 | 부위별 기준으로 같은 장면을 다시 돌려 둠(균일·이전·부위별 세 막대) | analysis_remake/figures/static_scene_gap.png |
| figures/static_scene_table.png | 장면별 효과 표 | 다시 만듦 | 부위별 기준의 주요 숫자 표로 교체(내용이 요약표로 바뀜) | analysis_remake/figures/summary_table.png |
| figures/gain_knob.png | k_gain 0~0.15 에 따른 구역과 간격 | 문구만 고침 | 부위별 기준의 k_gain 표(V2 정면 seed 0: 6.2 / 8.9 / 10.9 / 13.3 cm)는 있으나 그림은 없음. 경향은 같음 | - |
| figures/moving_hand_table.png | 움직이는 손, 속도별 평균 간격 표 | 다시 만듦 | 부위별 기준으로 다시 돌려 둠(균일·이전·부위별) | analysis_remake/figures/moving_hand_table.png |
| figures/collaboration_phase_gap.png | 건네기·가로지르기 통과 시점별 간격 | 문구만 고침 | 이전 식 결과. 부위별 기준으로는 가로지르기 일부만 다시 돌림 | - |
| figures/fast_hand_phase_gap.png | 빠른 손과 움직임 예측 | 문구만 고침 | 움직임 예측의 효과를 본 실험. 이전 식 결과임을 밝힘 | - |
| figures/camera_imperfection.png | 지연·떨림·누락 조합 | 문구만 고침 | 카메라 결함과 예측·필터의 효과를 본 실험. 이전 식 결과임을 밝힘 | - |
| media/static_two_directions.gif, .mp4 | 정지한 팔, 정면·위 장면, 균일 대 이전 식 | 다시 만듦 | 같은 장면의 부위별 기준 영상이 있음(정면·위 x 균일·부위별, seed 0) | analysis_remake/media/static_two_directions.gif (6.4 MB), .mp4 (2.2 MB) |
| media/static_fingertip_front.gif, .mp4 | 옆으로 뻗은 팔의 손끝 앞 통과 | 문구만 고침 | 이전 식 결과. 실제 녹화의 손끝 앞 통과 영상(analysis_live)이 같은 질문을 보여 줌 | - |
| media/gain_knob.gif, .mp4 | k_gain 4가지 | 문구만 고침 | 위 gain_knob.png 와 같음 | - |
| media/moving_hand.gif, .mp4 | 움직이는 손 0.25 m/s | 문구만 고침 | 영상은 이전 식. 숫자는 새 표로 고침 | - |
| media/handover.gif, .mp4 | 건네기 | 문구만 고침 | 이전 식 결과 | - |
| media/crossing.gif, .mp4 | 가로지르기 | 문구만 고침 | 이전 식 결과 | - |

그림 12개와 GIF·mp4 짝 6개(12 파일), 모두 24 파일을 18줄로 정리했다. 다시 만듦 8줄, 문구만 고침 9줄, 그대로 둠 1줄, 미완 0줄.

문구만 고치는 항목에 붙일 문장(초안): "이 그림(영상)은 이전 식(모든 부위에 같은 기준 질량 0.53 kg)으로 만들었다. 손의 구역은 지금 식(부위별 기준)과 거의 같지만, 아래팔·위팔·몸통·머리의 구역은 지금 식에서 더 얇다. 지금 식의 결과는 [부위별 기준 비교](../goal1/part_regen/REPORT.md)를 본다."

README 의 그림(`goal1/zone/zone_3d.png`, `v2_c_thickness.png`, `zone_overlay.png`, `media/v2_main.gif`)도 이전 식이다. README 초안(README_draft.md)에서는 대표 영상을 실제 녹화 비교로 바꾸고, 위 그림들은 부위별 기준 그림으로 바꾸었다.

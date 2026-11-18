# 올리기 전 점검 (2026-11-19 00:16, push 하지 않음)

범위: `origin/main`(10aaa58) 이후 로컬 커밋 18개, 더하거나 바뀐 파일 195개(약 68 MB, 그림·영상 포함). 이 초안 폴더를 커밋하면 더 늘어난다.

| 검사 | 방법 | 결과 |
|---|---|---|
| 비공개 대상 파일 | 경로 이름 검사: ws/, iso_15066.json, shapes_v2.json, LICENSE, 원본 컬러 프레임(vis_output, color/), 사람 자세(pose_npz, .npz), 메시 정점(_verts), SMPL 모델(.pkl), 로봇 자산(.usd, .urdf, .stl, .obj, .dae), 타사 코드(curobo, isaacsim) | 범위 안: 해당 자료 없음. 이름에 걸린 것은 스크립트 두 개(`goal1/scripts/lv_mesh_verts.py` = 정점을 만드는 코드, `shapes_v2_fit.py` = 이미 공개됨)이며 자료가 아니다. 저장소 전체(HEAD)에서도 같다 |
| 비밀 정보 | 내용 검사: password, passwd, ghp_, github_pat_, api_key, secret_key, 개인 키 머리말, AWS 키 모양, token = '...' | 없음 (텍스트 파일 164개와 범위의 커밋 기록 전체) |
| 메일 주소 | 내용 검사 | 없음 (커밋 서명의 noreply 제외) |
| 내부 IP 주소 | 192.168.x, 10.x, 172.16-31.x | 없음 |
| 파일 크기 | 영상 20 MB, GIF 10 MB 한도 | 넘는 파일 없음. 가장 큰 파일 `goal1/live/live3m/compare/live3m_compare.mp4` 10.6 MB |

공개 전에 사용자가 확인할 것:

- **영상 속 사람**: live3m 영상(컬러 화면)에 사용자 얼굴과 연구실 내부가 나온다. 창문의 형체는 사용자의 유리 반사로 판단했다(처음·중간·끝 프레임 확인). 시험 녹화(lp2_*)의 영상은 커밋하지 않았다.
- **사람 자세에서 나온 숫자**: `goal1/live/live3m/compare_tip/paths.json`, `scenes.json` 에 대표 자세의 손끝 위치·팔 방향 몇 개(로봇 기준 좌표)와 유지 프레임 번호가 들어 있다. 원본 자세 자료는 아니다.
- **개발 환경 경로**: 로그 json 과 스크립트에 `/home/robotics/...` 경로가 남아 있다(기존 공개 자료와 같음, README 7절에 적혀 있음).
- **커밋 범위의 성격**: 범위에는 실시간 Kinect 준비, 부위별 기준, 이번 밤 작업이 모두 들어 있다. 공개 README 와 analysis/ 는 이 초안을 반영하기 전까지 이전 식 기준이다.

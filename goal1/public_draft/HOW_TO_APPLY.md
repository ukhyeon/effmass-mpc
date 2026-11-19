# 초안 적용 방법 (사용자가 "푸시 진행" 전에 할 일)

> **적용 완료 (2026-11-19).** 아래 순서로 README.md, analysis/, analysis/live/ 에 반영했다. README_draft.md 는 최종본(README.md)을 가리키게 바꿨고, analysis_live/README.md 는 analysis/live/README.md 와 같게 맞췄다.

공개 저장소의 README 와 analysis/ 는 이번 작업에서 바꾸지 않았다. 아래 순서로 옮기면 된다.

1. `goal1/public_draft/analysis_live/` 를 `analysis/live/` 로 복사한다(README.md, figures/, media/).
2. `goal1/public_draft/analysis_remake/figures/*.png`, `media/*` 를 `analysis/figures/`, `analysis/media/` 에 덮어쓴다.
   - `zone_thickness_by_part.png` 는 이름이 다르다. analysis/README.md 2-2 절의 그림 링크를 이 이름으로 바꾸거나, 파일 이름을 `zone_thickness_by_posture.png` 로 바꿔 덮어쓴다.
   - `summary_table.png` 는 `static_scene_table.png` 자리에 쓴다.
3. `file_table.md` 의 "문구만 고침" 항목 설명 아래에 표에 적은 한 문장을 붙인다.
4. `changed_numbers.md` 의 새 숫자로 analysis/README.md 와 README.md 의 숫자를 고친다.
5. `README_draft.md` 를 `README.md` 로 바꾸고, "(현재 README N절과 같음)" 표시는 현재 README 의 해당 절로 채운다.
6. `prepush_check.md` 를 다시 돌린다: `git diff --name-only --diff-filter=AM origin/main..HEAD` 로 범위를 보고 같은 검사를 한다.

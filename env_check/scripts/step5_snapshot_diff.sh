#!/usr/bin/env bash
# step5_snapshot_diff.sh <before_prefix> <after_prefix>
B="$1"; A="$2"
echo "--- ~/hrc_ws 파일 목록 차이: $(diff ${B}_hrc_files.tsv ${A}_hrc_files.tsv | grep -c '^[<>]')줄"
diff -q ${B}_hrc_summary.txt ${A}_hrc_summary.txt > /dev/null && echo "--- 노드 성능 CSV·__pycache__ 요약 동일" || diff ${B}_hrc_summary.txt ${A}_hrc_summary.txt
diff -q ${B}_sensing_dirs.txt ${A}_sensing_dirs.txt > /dev/null && echo "--- vis_output·body_crop·pose_npz 수, sensing CSV, ~/.ros 파일 수 동일" || diff ${B}_sensing_dirs.txt ${A}_sensing_dirs.txt
# 측정과 무관하게 실행 중인 브라우저(google-chrome) 프로필 파일은 따로 셈
D=$(diff ${B}_home_files.tsv ${A}_home_files.tsv | grep '^[<>]')
echo "--- ~/.config ~/.cache ~/.nv 차이: 전체 $(printf '%s' "$D" | grep -c .)줄, 그중 google-chrome $(printf '%s' "$D" | grep -c 'google-chrome')줄, 그 밖 $(printf '%s' "$D" | grep -vc 'google-chrome')줄"
printf '%s\n' "$D" | grep -v 'google-chrome' | grep . | head -10

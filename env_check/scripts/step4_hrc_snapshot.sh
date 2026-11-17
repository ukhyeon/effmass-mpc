#!/usr/bin/env bash
# step4_hrc_snapshot.sh <out_prefix> — ~/hrc_ws 전체 파일 목록(경로·크기·수정시각), 성능 CSV 2개 줄수, install __pycache__ 목록
set -u
O="${1:?prefix}"
find "$HOME/hrc_ws" -printf '%p\t%s\t%T@\n' 2>/dev/null | sort > "${O}_files.tsv"
{
  for f in paper_human_dynamics_perf.csv paper_human_meff_query_perf.csv; do
    p="$HOME/hrc_ws/analysis/realtime_perf/$f"
    echo -e "$f\tsize=$(stat -c %s "$p")\tlines=$(wc -l < "$p")\tmtime=$(stat -c %y "$p")"
  done
  echo -e "install_pycache_dirs\t$(find "$HOME/hrc_ws/install" -name __pycache__ -type d | wc -l)"
  echo -e "install_pyc_files\t$(find "$HOME/hrc_ws/install" -name '*.pyc' | wc -l)"
  echo -e "hrc_ws_entries\t$(wc -l < "${O}_files.tsv")"
} > "${O}_summary.txt"
cat "${O}_summary.txt"

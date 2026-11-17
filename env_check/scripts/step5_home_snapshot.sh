#!/usr/bin/env bash
# step5_home_snapshot.sh <prefix> — ~/.config ~/.cache ~/.nv 파일 목록(경로·크기·수정시각) + ~/hrc_ws 스냅샷 (step4_hrc_snapshot.sh) + sensing 쓰기 위치 파일 수
P="${1:?prefix}"
find $HOME/.config $HOME/.cache $HOME/.nv -type f -printf '%p\t%s\t%T@\n' 2>/dev/null | sort > "${P}_home_files.tsv"
$HOME/effmass_mpc/env_check/scripts/step4_hrc_snapshot.sh "${P}_hrc" > /dev/null
{ for d in vis_output body_crop pose_npz; do echo "$d $(find $HOME/hrc_ws/$d -type f | wc -l)"; done
  stat -c 'sensing_csv size=%s mtime=%y' $HOME/hrc_ws/analysis/realtime_perf/paper_human_sensing_perf.csv
  echo "ros_files $(find $HOME/.ros -type f | wc -l)"; } > "${P}_sensing_dirs.txt"

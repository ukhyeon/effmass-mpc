#!/usr/bin/env bash
# g1_human_live.sh <tag> [drop_every] — 재생(30 Hz) → 사본 노드(CPU 1스레드) → 기록기, 1초 감시. ~/hrc_ws·~/.ros 전후 확인
set -u
TAG="${1:?tag}"; DROP="${2:-0}"
E=$HOME/effmass_mpc/env_check; S=$E/scripts; G=$HOME/effmass_mpc/goal1; O=$G/logs/$TAG; mkdir -p $O; rm -f $O/stop $O/ready $O/end
$S/step4_hrc_snapshot.sh $O/hrc_before > /dev/null; touch $O/t0
/usr/bin/python3 -B $S/step4_sys_monitor.py --proc node=human_inertia_node --out $O/sys_monitor.csv --stop-file $O/stop &
SM=$!
$G/scripts/g1_run_human.sh py $G/scripts/g1_human_rec.py --out $O/rec.npz --stop-file $O/stop > $O/rec.log 2>&1 &
RC=$!
$G/scripts/g1_run_human.sh node -p perf_csv:=$O/node_perf.csv > $O/node.log 2>&1 &
NP=$!
touch $O/ready
$S/run_sys_ros.sh -B $S/step4_replay.py --npz $HOME/hrc_ws/pose_npz/human_state_1781524848.npz --min-subs 1 --ready-file $O/ready --end-file $O/end --out $O/replay --start-wait 8 --drop-every $DROP > $O/replay.log 2>&1
echo "replay exit=$?"
kill -INT $NP; wait $NP; echo "node exit=$?"; touch $O/stop; wait $SM $RC 2>/dev/null
$S/step4_hrc_snapshot.sh $O/hrc_after > /dev/null
echo "--- ~/hrc_ws 변경: $(diff $O/hrc_before_files.tsv $O/hrc_after_files.tsv | grep -c '^[<>]')줄, ~/.ros 새 파일: $(find $HOME/.ros -type f -newer $O/t0 | wc -l), /dev/shm: $(ls /dev/shm | grep -c fastrtps), 남은 프로세스: $(pgrep -fc 'human_inertia_node|g1_human_rec|step4_replay|step4_sys_monitor')"
grep -a "^\[rec\]\|^\[replay\]" $O/rec.log $O/replay.log | cut -c1-200; grep -a "형상 맞춤\|device\|Traceback" $O/node.log | cut -c1-200 | head -3

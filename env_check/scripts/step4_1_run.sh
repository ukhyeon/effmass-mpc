#!/usr/bin/env bash
# step4_1_run.sh <tag> [Isaac 인자...] — 재생 → human_dynamics_node → Isaac (MPC 없음). 4-1·4-2 공용
set -u
TAG="${1:?tag}"; shift
D=$HOME/effmass_mpc/env_check; S=$D/scripts; O=$D/step4/$TAG
NPZ=$HOME/hrc_ws/pose_npz/human_state_1781524848.npz
rm -f $O.ready $O.end $O.stop
$S/step4_hrc_snapshot.sh $D/step4/${TAG}_hrc_before > /dev/null
touch $D/step4/${TAG}_t0
RUN_ISAAC_NO_DISPLAY=${RUN_ISAAC_NO_DISPLAY:-1} ROS_METHOD=min $S/run_isaac_ros.sh $S/step4_isaac_human.py --ready-file $O.ready --end-file $O.end --out $O "$@" > $D/logs/${TAG}_isaac.log 2>&1 &
IP=$!
$S/step4_run_dyn_node.sh > $D/logs/${TAG}_node.log 2>&1 &
NP=$!
sleep 1
/usr/bin/python3 -B $S/step4_proc_monitor.py --pid $NP --out $D/step4/${TAG}_node_monitor.csv --stop-file $O.stop &
MP=$!
$S/run_sys_ros.sh -B $S/step4_replay.py --npz $NPZ ${REPLAY_ARGS:-} --ready-file $O.ready --end-file $O.end --out $O --start-wait 8 > $D/logs/${TAG}_replay.log 2>&1
echo "replay exit=$?"
wait $IP; echo "isaac exit=$?"
kill -INT $NP; wait $NP; echo "node exit=$?"
touch $O.stop; wait $MP 2>/dev/null
for f in $D/step4/node_perf/*.csv; do [ -f "$f" ] && mv -n "$f" "$D/step4/${TAG}_nodeperf_$(basename $f)"; done
$S/step4_hrc_snapshot.sh $D/step4/${TAG}_hrc_after > /dev/null
echo "--- ~/hrc_ws 변경 (경로·크기·수정시각 목록 diff 줄 수): $(diff $D/step4/${TAG}_hrc_before_files.tsv $D/step4/${TAG}_hrc_after_files.tsv | grep -c '^[<>]')"
diff $D/step4/${TAG}_hrc_before_summary.txt $D/step4/${TAG}_hrc_after_summary.txt && echo "--- 성능 CSV·__pycache__ 요약 동일"
cat $D/step4/${TAG}_hrc_after_summary.txt
echo "--- ~/.ros 새 파일: $(find $HOME/.ros -type f -newer $D/step4/${TAG}_t0 | wc -l)"
grep -a "^\[wrapper\]" $D/logs/${TAG}_node.log
grep -a "^\[replay\]" $D/logs/${TAG}_replay.log
grep -a "^\[isaac4\]" $D/logs/${TAG}_isaac.log | cut -c1-300
grep -aE "Traceback" -A5 $D/logs/${TAG}_isaac.log $D/logs/${TAG}_node.log $D/logs/${TAG}_replay.log | head -20

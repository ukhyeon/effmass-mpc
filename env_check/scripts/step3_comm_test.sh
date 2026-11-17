#!/usr/bin/env bash
# step3_comm_test.sh <tag> — 3-1 통신 시험: Isaac 쪽(배경) → READY → 시스템 쪽(전경) → 분석
set -u
TAG="${1:?tag}"; D=$HOME/effmass_mpc/env_check; O=$D/step3/$TAG; S=$D/scripts
rm -f $O.ready $O.end
RUN_ISAAC_NO_DISPLAY=1 ROS_METHOD=min $S/run_isaac_ros.sh $S/step3_isaac_comm.py --ready-file $O.ready --end-file $O.end --out $O > $D/logs/${TAG}_isaac.log 2>&1 &
IP=$!
$S/run_sys_ros.sh $S/step3_sys_node.py --plan "pose30:30:20,pose60:60:20,dyn30:30:20" --ready-file $O.ready --end-file $O.end --js-check rule --out $O > $D/logs/${TAG}_sys.log 2>&1
SR=$?
wait $IP; IR=$?
echo "sys exit=$SR isaac exit=$IR"
grep -a "^\[isaac\]" $D/logs/${TAG}_isaac.log; grep -a "^\[sys\]" $D/logs/${TAG}_sys.log
python3 $S/step3_analyze_comm.py $O | tee $D/logs/${TAG}_analysis.txt

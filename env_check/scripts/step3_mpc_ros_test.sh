#!/usr/bin/env bash
# step3_mpc_ros_test.sh <tag> — 3-2: 측정 B v2 + ROS (Isaac 쪽 배경, 시스템 쪽 HumanPose 30 Hz 발행·JointState 수신)
set -u
TAG="${1:?tag}"; D=$HOME/effmass_mpc/env_check; O=$D/step3/$TAG; S=$D/scripts
rm -f $O.ready $O.done
$S/vram_monitor.sh $D/logs/${TAG}_vram.csv & MON=$!
sleep 2
RUN_ISAAC_NO_DISPLAY=1 ROS_METHOD=min $S/run_isaac_ros.sh $S/step2b_mpc_isaac.py --scene v2 --tag $TAG --ros \
  --ready-file $O.ready --done-file $O.done --ros-out $O > $D/logs/${TAG}.log 2>&1 &
IP=$!
$S/run_sys_ros.sh $S/step3_sys_node.py --plan "pose30:30:600" --until-file $O.done --ready-file $O.ready \
  --end-file $O.sysend --js-check log --out $O > $D/logs/${TAG}_sys.log 2>&1
SR=$?
wait $IP; IR=$?
sleep 2; kill $MON 2>/dev/null; wait $MON 2>/dev/null
echo "sys exit=$SR isaac exit=$IR"
grep -aE "^\[step2b\] (ROS|scene|first_call|solver_ms|solver_steps|realtime|loop_ms|physics|infeasible|arrivals|min_clear|proc_vram|graph|step 기록)" $D/logs/${TAG}.log | cut -c1-240
grep -a "^\[sys\]" $D/logs/${TAG}_sys.log
grep -aE "Traceback" -A4 $D/logs/${TAG}.log $D/logs/${TAG}_sys.log | head -10
$S/vram_summary.sh $D/logs/${TAG}_vram.csv
F=$(ls -t $D/step2/${TAG}_steps_*.csv 2>/dev/null | head -1); [ -n "$F" ] && python3 $S/step2_analyze_steps.py $F | grep -E "정상|infeasible  |루프 ms \(전체\)|물리|최대값|참고" 
python3 $S/step3_analyze_mpc.py $O | tee $D/logs/${TAG}_ros_analysis.txt

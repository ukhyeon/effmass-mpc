#!/usr/bin/env bash
# step3r_run.sh <tag> <ros:0|1> [step2b 추가 인자...]
#   3-2 재측정용. 공통: --scene v2 --no-diag --warmup 10 (+ 추가 인자). VRAM 은 1초 간격 외부 기록.
#   ros=0: run_isaac.sh 로 실행 (ROS 없음)
#   ros=1: run_isaac_ros.sh(ROS_METHOD=min) + --ros, 시스템 노드가 HumanPose 30 Hz 발행·JointState 수신 (3-2 와 같음)
set -u
TAG="${1:?tag}"; ROS="${2:?ros 0|1}"; shift 2
D=$HOME/effmass_mpc/env_check; O=$D/step3/$TAG; S=$D/scripts
COMMON=(--scene v2 --no-diag --warmup 10 --tag "$TAG" "$@")
rm -f $O.ready $O.done $O.sysend
$S/vram_monitor.sh $D/logs/${TAG}_vram.csv & MON=$!
sleep 2
if [[ "$ROS" == "1" ]]; then
  RUN_ISAAC_NO_DISPLAY=1 ROS_METHOD=min $S/run_isaac_ros.sh $S/step2b_mpc_isaac.py "${COMMON[@]}" --ros \
    --ready-file $O.ready --done-file $O.done --ros-out $O > $D/logs/${TAG}.log 2>&1 &
  IP=$!
  $S/run_sys_ros.sh $S/step3_sys_node.py --plan "pose30:30:600" --until-file $O.done --ready-file $O.ready \
    --end-file $O.sysend --js-check log --out $O > $D/logs/${TAG}_sys.log 2>&1
  echo "sys exit=$?"
  wait $IP; echo "isaac exit=$?"
else
  RUN_ISAAC_NO_DISPLAY=1 $S/run_isaac.sh $S/step2b_mpc_isaac.py "${COMMON[@]}" > $D/logs/${TAG}.log 2>&1
  echo "isaac exit=$?"
fi
sleep 2; kill $MON 2>/dev/null; wait $MON 2>/dev/null
grep -aE "^\[step2b\] (예열|ROS|first_call|solver_ms|solver_steps|realtime|loop_ms|physics|infeasible|arrivals|proc_vram|graph)" $D/logs/${TAG}.log | cut -c1-300
[[ "$ROS" == "1" ]] && grep -a "^\[sys\]" $D/logs/${TAG}_sys.log | tail -3
grep -aE "Traceback" -A4 $D/logs/${TAG}.log | head -8
$S/vram_summary.sh $D/logs/${TAG}_vram.csv
F=$(ls -t $D/step2/${TAG}_steps_*.csv 2>/dev/null | head -1)
[[ -n "$F" ]] && python3 $S/step2_analyze_steps.py $F | grep -E "^(정상|infeasible  |시도|물리|루프 ms \(전체|최대값|참고|항목별)"
[[ "$ROS" == "1" ]] && python3 $S/step3_analyze_mpc.py $O | tee $D/logs/${TAG}_ros_analysis.txt
exit 0

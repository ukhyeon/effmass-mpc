#!/usr/bin/env bash
# step4_3_run.sh <tag> [step2b 추가 인자] — 4-3: 3단계 (b) 설정 + 사람 (재생 → human_dynamics_node → Isaac 표시·GPU 텐서)
set -u
TAG="${1:?tag}"; shift
D=$HOME/effmass_mpc/env_check; S=$D/scripts; O=$D/step4/$TAG
NPZ=$HOME/hrc_ws/pose_npz/human_state_1781524848.npz
rm -f $O.ready $O.done $O.end $O.stop
$S/step4_hrc_snapshot.sh $D/step4/${TAG}_hrc_before > /dev/null
touch $D/step4/${TAG}_t0
$S/vram_monitor.sh $D/logs/${TAG}_vram.csv & VM=$!
$S/run_sys_ros.sh -B $S/step4_js_sink.py --out ${O}_sys_recv_js.csv --stop-file $O.stop > $D/logs/${TAG}_sink.log 2>&1 &
SK=$!
RUN_ISAAC_NO_DISPLAY=${RUN_ISAAC_NO_DISPLAY:-1} ROS_METHOD=min $S/run_isaac_ros.sh $S/step2b_mpc_isaac.py --scene v2 --no-diag --warmup 10 \
  --tag $TAG --ros --ros-pose-topic none --human --ready-file $O.ready --done-file $O.done --ros-out $O "$@" > $D/logs/${TAG}.log 2>&1 &
IP=$!
$S/step4_run_dyn_node.sh > $D/logs/${TAG}_node.log 2>&1 &
NP=$!
sleep 1
/usr/bin/python3 -B $S/step4_proc_monitor.py --pid $NP --out $D/step4/${TAG}_node_monitor.csv --stop-file $O.stop &
MP=$!
$S/run_sys_ros.sh -B $S/step4_replay.py --npz $NPZ --ready-file $O.ready --end-file $O.end --out $O --start-wait 8 > $D/logs/${TAG}_replay.log 2>&1
echo "replay exit=$?"
wait $IP; echo "isaac exit=$?"
kill -INT $NP; wait $NP; echo "node exit=$? (SIGINT 후 노드 main 의 rclpy.shutdown 중복 호출로 1 이 나올 수 있음)"
touch $O.stop; wait $MP $SK 2>/dev/null
sleep 1; kill $VM 2>/dev/null; wait $VM 2>/dev/null
for f in $D/step4/node_perf/*.csv; do [ -f "$f" ] && mv -n "$f" "$D/step4/${TAG}_nodeperf_$(basename $f)"; done
$S/step4_hrc_snapshot.sh $D/step4/${TAG}_hrc_after > /dev/null
echo "--- ~/hrc_ws 변경: $(diff $D/step4/${TAG}_hrc_before_files.tsv $D/step4/${TAG}_hrc_after_files.tsv | grep -c '^[<>]')줄"
diff -q $D/step4/${TAG}_hrc_before_summary.txt $D/step4/${TAG}_hrc_after_summary.txt > /dev/null && echo "--- 성능 CSV·__pycache__ 요약 동일"
echo "--- ~/.ros 새 파일: $(find $HOME/.ros -type f -newer $D/step4/${TAG}_t0 | wc -l)"
grep -a "^\[wrapper\] 종료" $D/logs/${TAG}_node.log; grep -a "^\[replay\]" $D/logs/${TAG}_replay.log; grep -a "^\[sink\]" $D/logs/${TAG}_sink.log
grep -aE "^\[step2b\] (예열|ROS|사람|first_call|solver_ms|solver_steps|realtime|loop_ms|physics|infeasible|arrivals|proc_vram|graph)" $D/logs/${TAG}.log | cut -c1-300
grep -aE "Traceback" -A5 $D/logs/${TAG}.log $D/logs/${TAG}_replay.log $D/logs/${TAG}_sink.log | head -12
$S/vram_summary.sh $D/logs/${TAG}_vram.csv
F=$(ls -t $D/step2/${TAG}_steps_*.csv 2>/dev/null | head -1)
[[ -n "$F" ]] && python3 $S/step2_analyze_steps.py $F | grep -E "^(정상|infeasible  |시도|최대값|참고|항목별)"
/usr/bin/python3 -B $S/step4_analyze_43.py $O $F | tee $D/logs/${TAG}_analysis43.txt

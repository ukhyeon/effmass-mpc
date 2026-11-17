#!/usr/bin/env bash
# step4_3x_run.sh <tag> <A|B|C|D> — 4-3 원인 분리 측정. MPC 설정·장면·예열·Isaac 처리 방식은 4-3 과 같음 (B 만 Isaac 사람 처리 끔)
#   A: 노드 없음. step4_dyn_republish.py 가 4-2 headless 에서 Isaac 이 받은 HumanDynamicsState 기록을 30 Hz 로 다시 발행. 재생 HumanPose 는 그대로
#   B: 노드 있음, Isaac --human-mode recv (구독·수신 기록만, 표시·GPU 텐서 없음)
#   C: 노드 있음, 래퍼에서 64방향 최악 유효질량 끔 (STEP4_NO_FIND_WORST=1)
#   E: C + 노드만 CPU 계산 (STEP4_NODE_CPU=1 → 노드 환경에만 CUDA_VISIBLE_DEVICES=)
#   F: E + 노드 CPU 스레드 1개 (STEP4_NODE_THREADS=1 → 노드 환경에만 OMP/MKL_NUM_THREADS=1, torch.set_num_threads(1))
#   D: C + CPU 코어 분리 (환경 변수 NODE_CPUS, ISAAC_CPUS 로 taskset -c 지정)
#   공통 기록: step4_sys_monitor (프로세스별 CPU%·VRAM, 코어별 사용률, GPU 사용률, 1 s), /hrc/worst_head_meff 수신 수, ~/hrc_ws 전후, ~/.ros 새 파일
set -u
TAG="${1:?tag}"; CASE="${2:?A|B|C|D|E}"; shift 2
D=$HOME/effmass_mpc/env_check; S=$D/scripts; O=$D/step4/$TAG
NPZ=$HOME/hrc_ws/pose_npz/human_state_1781524848.npz
REC=$D/step4/step4_2_headless
rm -f $O.ready $O.done $O.end $O.stop
$S/step4_hrc_snapshot.sh $D/step4/${TAG}_hrc_before > /dev/null
touch $D/step4/${TAG}_t0
HM=full; NOFW=0; ITS=""; NTS=""; NCPU=0; NTHR=""
case $CASE in
  A) ;; B) HM=recv ;; C) NOFW=1 ;; E) NOFW=1; NCPU=1 ;; F) NOFW=1; NCPU=1; NTHR=1 ;;
  D) NOFW=1; ITS="taskset -c ${ISAAC_CPUS:?}"; NTS="taskset -c ${NODE_CPUS:?}" ;;
  *) echo "case?"; exit 2 ;;
esac
echo "[3x] tag $TAG case $CASE human-mode $HM no_find_worst $NOFW isaac_taskset '${ITS}' node_taskset '${NTS}' node_cpu $NCPU, node_threads '${NTHR}', 추가 isaac 인자 '$*', slack 프로세스 $(pgrep -fc /snap/slack)개 (pid 136205 현재 CPU% $(top -b -n 2 -d 1 -p 136205 2>/dev/null | awk 'END{print $9}'))"
/usr/bin/python3 -B $S/step4_sys_monitor.py --proc isaac=step2b_mpc_isaac.py --proc node=step4_dyn_node_wrapper.py \
  --proc republish=step4_dyn_republish.py --proc replay=step4_replay.py ${EXTRA_PROC:-} --out ${O}_sys_monitor.csv --stop-file $O.stop &
SM=$!
$S/vram_monitor.sh $D/logs/${TAG}_vram.csv & VM=$!
$S/run_sys_ros.sh -B $S/step4_js_sink.py --out ${O}_sys_recv_js.csv --stop-file $O.stop > $D/logs/${TAG}_sink.log 2>&1 &
SK=$!
$S/run_sys_ros.sh -B $S/step4_topic_count.py --out ${O}_worst_count.csv --stop-file $O.stop > $D/logs/${TAG}_worst.log 2>&1 &
WC=$!
RUN_ISAAC_NO_DISPLAY=1 ROS_METHOD=min $ITS $S/run_isaac_ros.sh $S/step2b_mpc_isaac.py --scene v2 --no-diag --warmup 10 \
  --tag $TAG --ros --ros-pose-topic none --human --human-mode $HM --ready-file $O.ready --done-file $O.done --ros-out $O "$@" > $D/logs/${TAG}.log 2>&1 &
IP=$!
NP=""
if [[ $CASE == A ]]; then
  $S/run_sys_ros.sh -B $S/step4_dyn_republish.py --rec $REC --out $O > $D/logs/${TAG}_republish.log 2>&1 &
  RP=$!
else
  STEP4_NO_FIND_WORST=$NOFW STEP4_NODE_CPU=$NCPU STEP4_NODE_THREADS=$NTHR STEP4_TASKSET="$NTS" $S/step4_run_dyn_node.sh > $D/logs/${TAG}_node.log 2>&1 &
  NP=$!
fi
$S/run_sys_ros.sh -B $S/step4_replay.py --npz $NPZ ${REPLAY_ARGS:-} --ready-file $O.ready --end-file $O.end --out $O --start-wait 8 > $D/logs/${TAG}_replay.log 2>&1
echo "replay exit=$?"
wait $IP; echo "isaac exit=$?"
if [[ -n "$NP" ]]; then kill -INT $NP; wait $NP; echo "node exit=$? (SIGINT 후 노드 main 의 rclpy.shutdown 중복 호출로 1 이 나올 수 있음)"
else wait $RP; echo "republish exit=$?"; fi
touch $O.stop; wait $SM $SK $WC 2>/dev/null
sleep 1; kill $VM 2>/dev/null; wait $VM 2>/dev/null
for f in $D/step4/node_perf/*.csv; do [ -f "$f" ] && mv -n "$f" "$D/step4/${TAG}_nodeperf_$(basename $f)"; done
$S/step4_hrc_snapshot.sh $D/step4/${TAG}_hrc_after > /dev/null
echo "--- ~/hrc_ws 변경: $(diff $D/step4/${TAG}_hrc_before_files.tsv $D/step4/${TAG}_hrc_after_files.tsv | grep -c '^[<>]')줄"
diff -q $D/step4/${TAG}_hrc_before_summary.txt $D/step4/${TAG}_hrc_after_summary.txt > /dev/null && echo "--- 성능 CSV·__pycache__ 요약 동일"
echo "--- ~/.ros 새 파일: $(find $HOME/.ros -type f -newer $D/step4/${TAG}_t0 | wc -l)"
echo "--- /dev/shm fastrtps: $(ls /dev/shm | grep -c fastrtps)개"
echo "--- 남은 프로세스: $(pgrep -fc 'step2b_mpc_isaac|step4_dyn_node_wrapper|step4_dyn_republish|step4_replay|step4_js_sink|step4_topic_count|step4_sys_monitor')"
grep -a "^\[wrapper\]" $D/logs/${TAG}_node.log 2>/dev/null | grep -v "PerfCsvLogger"; grep -a "^\[republish\]" $D/logs/${TAG}_republish.log 2>/dev/null
grep -a "^\[replay\]" $D/logs/${TAG}_replay.log; grep -a "^\[sink\]" $D/logs/${TAG}_sink.log; grep -a "^\[worst_count\]" $D/logs/${TAG}_worst.log
grep -aE "^\[step2b\] (예열|ROS|사람|first_call|solver_ms|solver_steps|realtime|loop_ms|physics|infeasible|arrivals|proc_vram|graph)" $D/logs/${TAG}.log | cut -c1-300
grep -aE "Traceback" -A5 $D/logs/${TAG}.log $D/logs/${TAG}_*.log | head -12
F=$(ls -t $D/step2/${TAG}_steps_*.csv 2>/dev/null | head -1)
[[ -n "$F" ]] && python3 $S/step2_analyze_steps.py $F | grep -E "^(정상|infeasible  |시도|최대값|참고|항목별)"
/usr/bin/python3 -B $S/step4_analyze_43.py $O $F | tee $D/logs/${TAG}_analysis43.txt

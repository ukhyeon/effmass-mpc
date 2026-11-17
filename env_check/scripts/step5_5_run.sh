#!/usr/bin/env bash
# step5_5_run.sh <tag> <I|J> [mpc_first|sensing_first] — 5-5: Isaac 없음. sensing + human_dynamics_node(CPU 1스레드) + MPC 60 Hz 실시간
#   I: sensing 시각화 끔(정점 계산 유지) / J: 정점 계산까지 끔 (이번 실행에서만)
#   mpc_first (기본, 5-2 와 같은 순서): MPC → 노드 → sensing / sensing_first: sensing 발행 시작 확인 뒤 노드 → MPC
set -u
TAG="${1:?tag}"; CASE="${2:?I|J}"; ORDER="${3:-mpc_first}"
D=$HOME/effmass_mpc/env_check; S=$D/scripts; O=$D/step5/$TAG; mkdir -p $O
rm -f $O/run.ready $O.stop
$S/step5_home_snapshot.sh $O/before
touch $O/t0
NV=0; [[ $CASE == J ]] && NV=1
echo "[5-5] tag $TAG case $CASE order $ORDER $(date +%T) 깊이 카메라 $(lsusb | grep -c 045e:097c) slack $(pgrep -c slack)"
/usr/bin/python3 -B $S/step4_sys_monitor.py --proc mpc=step5_5_mpc_rt.py --proc node=step4_dyn_node_wrapper.py --proc sensing=step5_sensing_wrapper.py \
  --out $O/sys_monitor.csv --stop-file $O.stop &
SM=$!
$S/vram_monitor.sh $O/vram.csv & VM=$!
$S/run_sys_ros.sh -B $S/step4_js_sink.py --out $O/sys_recv_js.csv --stop-file $O.stop > $D/logs/${TAG}_sink.log 2>&1 &
SK=$!
start_sensing() { STEP5_SENSING_NOVIS=1 STEP5_SENSING_NOVERTS=$NV $S/step5_run_sensing.sh $O/sensing > $D/logs/${TAG}_sensing.log 2>&1 & SP=$!; }
start_node() { env STEP4_NO_FIND_WORST=1 STEP4_NODE_CPU=1 STEP4_NODE_THREADS=1 $S/step4_run_dyn_node.sh > $D/logs/${TAG}_node.log 2>&1 & NP=$!; }
start_mpc() { ROS_METHOD=min $S/run_isaac_ros.sh $S/step5_5_mpc_rt.py --tag $TAG --ros-out $O/run --ready-file $O/run.ready > $D/logs/${TAG}.log 2>&1 & MP=$!; }
if [[ $ORDER == sensing_first ]]; then
  start_sensing; T0=$(date +%s)
  until grep -aq "is_ready_shape : True" $D/logs/${TAG}_sensing.log 2>/dev/null; do
    sleep 0.5; kill -0 $SP 2>/dev/null || break; (( $(date +%s) - T0 > 120 )) && break
  done
  echo "[5-5] sensing 체형 추정 완료까지 $(( $(date +%s) - T0 )) s, USB 오류 $(grep -ac LIBUSB_TRANSFER_ERROR $D/logs/${TAG}_sensing.log)줄 → 노드·MPC 시작"
  start_node; start_mpc
else
  start_mpc; start_node; start_sensing
fi
wait $MP; echo "mpc exit=$?"
kill -INT $SP
for i in $(seq 1 20); do kill -0 $SP 2>/dev/null || break; sleep 1; done
kill -0 $SP 2>/dev/null && { echo "sensing 20 s 안에 안 끝남 → SIGTERM"; pkill -TERM -f step5_sensing_wrapper.py; sleep 10; pkill -KILL -f step5_sensing_wrapper.py && echo "→ SIGKILL"; }
wait $SP; echo "sensing exit=$?"
kill -INT $NP; wait $NP; echo "node exit=$?"
touch $O.stop; wait $SM $SK 2>/dev/null; kill $VM 2>/dev/null; wait $VM 2>/dev/null
for f in $D/step4/node_perf/*.csv; do [ -f "$f" ] && mv -n "$f" "$O/nodeperf_$(basename $f)"; done
$S/step5_home_snapshot.sh $O/after
$S/step5_snapshot_diff.sh $O/before $O/after
grep -aqE "LIBUSB_TRANSFER_ERROR|usb_cmd_libusb_cb" $D/logs/${TAG}_sensing.log 2>/dev/null && echo "!!! Kinect USB 오류: $(grep -acE 'LIBUSB_TRANSFER_ERROR' $D/logs/${TAG}_sensing.log)줄, 깊이 카메라 $(lsusb | grep -c 045e:097c)"
echo "--- ~/.ros 새 파일: $(find $HOME/.ros -type f -newer $O/t0 | wc -l), /dev/shm Fast DDS: $(ls /dev/shm | grep -c fastrtps), 남은 프로세스: $(pgrep -fc 'step5_sensing_wrapper|step4_dyn_node_wrapper|step5_5_mpc_rt|step4_sys_monitor|step4_js_sink')"
grep -a "^\[sensing-wrapper\] \(종료\|J:\|B:\)" $D/logs/${TAG}_sensing.log | cut -c1-250; grep -a "^\[wrapper\] \(종료\|장치\)" $D/logs/${TAG}_node.log
grep -a "^\[step5_5\]" $D/logs/${TAG}.log | cut -c1-250
F=$(ls -t $D/step2/${TAG}_steps_*.csv 2>/dev/null | head -1)
[[ -n "$F" ]] && /usr/bin/python3 -B $S/step5_analyze_5.py $O $F | tee $D/logs/${TAG}_analysis.txt

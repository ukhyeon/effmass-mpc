#!/usr/bin/env bash
# step5_2_run.sh <tag> <G|H> [A|B] — 5-2: Kinect + sensing + human_dynamics_node + Isaac·MPC (4-3 설정, 사람 표시·GPU 텐서·메시)
#   G: 노드 CPU 1스레드 (4-3d (F): 64방향 끔, CUDA_VISIBLE_DEVICES=, OMP/MKL 1, torch 1스레드)
#   H: 노드 GPU (4-3 (C): 64방향 끔)
#   (5-3 GUI) RUN_ISAAC_NO_DISPLAY=0 EXTRA_ISAAC="--gui --steps 7200" 로 Isaac 창 표시
#   A: sensing renat.yaml 그대로 (창 표시·jpg 저장, 저장은 step5/) / B: 이번 실행에서만 창 표시·jpg 저장 끔 (STEP5_SENSING_NOVIS=1)
set -u
TAG="${1:?tag}"; CASE="${2:?G|H}"; VIS="${3:-A}"
D=$HOME/effmass_mpc/env_check; S=$D/scripts; O=$D/step5/$TAG; mkdir -p $O
rm -f $O/run.ready $O/run.done $O.stop
$S/step5_home_snapshot.sh $O/before
touch $O/t0
case $CASE in
  G) NODE_ENV=(STEP4_NO_FIND_WORST=1 STEP4_NODE_CPU=1 STEP4_NODE_THREADS=1) ;;
  H) NODE_ENV=(STEP4_NO_FIND_WORST=1) ;;
  *) echo "case?"; exit 2 ;;
esac
SVIS=0; [[ $VIS == B ]] && SVIS=1
echo "[5-2] tag $TAG case $CASE vis $VIS node_env ${NODE_ENV[*]} slack $(pgrep -c slack) $(date +%T)"
/usr/bin/python3 -B $S/step4_sys_monitor.py --proc isaac=step2b_mpc_isaac.py --proc node=step4_dyn_node_wrapper.py --proc sensing=step5_sensing_wrapper.py \
  --out $O/sys_monitor.csv --stop-file $O.stop &
SM=$!
$S/vram_monitor.sh $O/vram.csv & VM=$!
$S/run_sys_ros.sh -B $S/step4_js_sink.py --out $O/sys_recv_js.csv --stop-file $O.stop > $D/logs/${TAG}_sink.log 2>&1 &
SK=$!
RUN_ISAAC_NO_DISPLAY=${RUN_ISAAC_NO_DISPLAY:-1} ROS_METHOD=min $S/run_isaac_ros.sh $S/step2b_mpc_isaac.py --scene v2 --no-diag --warmup 10 \
  --tag $TAG --ros --ros-pose-topic none --human --human-mesh usd --human-dir $D/step5 --mesh-offset node_live \
  --ready-file $O/run.ready --done-file $O/run.done --ros-out $O/run ${EXTRA_ISAAC:-} > $D/logs/${TAG}.log 2>&1 &
IP=$!
env "${NODE_ENV[@]}" $S/step4_run_dyn_node.sh > $D/logs/${TAG}_node.log 2>&1 &
NP=$!
STEP5_SENSING_NOVIS=$SVIS $S/step5_run_sensing.sh $O/sensing > $D/logs/${TAG}_sensing.log 2>&1 &
SP=$!
wait $IP; echo "isaac exit=$?"
kill -INT $SP
# Kinect 캡처가 멈추면 SIGINT 로 끝나지 않으므로(5-2 G 1회차): 20 s 뒤 SIGTERM, 다시 10 s 뒤 SIGKILL
for i in $(seq 1 20); do kill -0 $SP 2>/dev/null || break; sleep 1; done
kill -0 $SP 2>/dev/null && { echo "sensing 20 s 안에 안 끝남 → SIGTERM"; pkill -TERM -f step5_sensing_wrapper.py; sleep 10; pkill -KILL -f step5_sensing_wrapper.py && echo "→ SIGKILL"; }
wait $SP; echo "sensing exit=$? (SIGINT 중 발행·rclpy.shutdown 중복으로 1 이 나올 수 있음)"
kill -INT $NP; wait $NP; echo "node exit=$?"
touch $O.stop; wait $SM $SK 2>/dev/null; kill $VM 2>/dev/null; wait $VM 2>/dev/null
for f in $D/step4/node_perf/*.csv; do [ -f "$f" ] && mv -n "$f" "$O/nodeperf_$(basename $f)"; done
$S/step5_home_snapshot.sh $O/after
$S/step5_snapshot_diff.sh $O/before $O/after
grep -aqE "LIBUSB_TRANSFER_ERROR|usb_cmd_libusb_cb" $D/logs/${TAG}_sensing.log 2>/dev/null && echo "!!! Kinect USB 오류: $(grep -acE 'LIBUSB_TRANSFER_ERROR' $D/logs/${TAG}_sensing.log)줄, lsusb 045e: $(lsusb | grep -c 045e)개"
echo "--- ~/.ros 새 파일: $(find $HOME/.ros -type f -newer $O/t0 | wc -l), /dev/shm Fast DDS: $(ls /dev/shm | grep -c fastrtps), 남은 프로세스: $(pgrep -fc 'step5_sensing_wrapper|step4_dyn_node_wrapper|step2b_mpc_isaac|step4_sys_monitor|step4_js_sink')"
grep -a "^\[sensing-wrapper\] \(종료\|생성\|B:\)" $D/logs/${TAG}_sensing.log | cut -c1-300; grep -a "^\[wrapper\] \(종료\|장치\|(4-3d)\|find_worst\)" $D/logs/${TAG}_node.log
grep -aE "^\[step2b\] (예열|사람|realtime|solver_ms_p95|graph)" $D/logs/${TAG}.log | cut -c1-400
F=$(ls -t $D/step2/${TAG}_steps_*.csv 2>/dev/null | head -1)
/usr/bin/python3 -B $S/step5_analyze_2.py $O $F | tee $D/logs/${TAG}_analysis.txt

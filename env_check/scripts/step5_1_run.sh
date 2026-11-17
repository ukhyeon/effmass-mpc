#!/usr/bin/env bash
# step5_1_run.sh <tag> [기록 초, 기본 60] — 5-1: Kinect + sensing_node 만. 첫 HumanPose 수신부터 기록 초만큼 돌린 뒤 SIGINT
set -u
TAG="${1:?tag}"; DUR="${2:-60}"
D=$HOME/effmass_mpc/env_check; S=$D/scripts; O=$D/step5/$TAG; mkdir -p $O
rm -f $O.stop
$S/step5_home_snapshot.sh $O/before
touch $O/t0
/usr/bin/python3 -B $S/step4_sys_monitor.py --proc sensing=step5_sensing_wrapper.py --out $O/sys_monitor.csv --stop-file $O.stop &
SM=$!
$S/vram_monitor.sh $O/vram.csv & VM=$!
$S/run_sys_ros.sh -B $S/step5_pose_recorder.py --out $O/rec --stop-file $O.stop > $D/logs/${TAG}_recorder.log 2>&1 &
RC=$!
$S/step5_run_sensing.sh $O/sensing > $D/logs/${TAG}_sensing.log 2>&1 &
SP=$!
T0=$(date +%s)
echo "[5-1] sensing 시작 $(date +%T), 첫 HumanPose 기다림 (최대 180 s)"
while ! grep -aq "is_ready_shape : True" $D/logs/${TAG}_sensing.log 2>/dev/null; do
  sleep 0.5; kill -0 $SP 2>/dev/null || { echo "[5-1] sensing 종료됨"; break; }
  (( $(date +%s) - T0 > 180 )) && { echo "[5-1] 180 s 안에 체형 추정 완료 안 됨"; break; }
done
echo "[5-1] 체형 추정 완료 $(date +%T) ($(( $(date +%s) - T0 )) s), ${DUR} s 기록"
sleep $DUR
kill -INT $SP
# Kinect 캡처가 멈추면 SIGINT 로 끝나지 않으므로(5-2 G 1회차): 20 s 뒤 SIGTERM, 다시 10 s 뒤 SIGKILL
for i in $(seq 1 20); do kill -0 $SP 2>/dev/null || break; sleep 1; done
kill -0 $SP 2>/dev/null && { echo "sensing 20 s 안에 안 끝남 → SIGTERM"; pkill -TERM -f step5_sensing_wrapper.py; sleep 10; pkill -KILL -f step5_sensing_wrapper.py && echo "→ SIGKILL"; }
wait $SP; echo "sensing exit=$?"
touch $O.stop; wait $SM $RC 2>/dev/null; kill $VM 2>/dev/null; wait $VM 2>/dev/null
$S/step5_home_snapshot.sh $O/after
$S/step5_snapshot_diff.sh $O/before $O/after
grep -aqE "LIBUSB_TRANSFER_ERROR|usb_cmd_libusb_cb" $D/logs/${TAG}_sensing.log 2>/dev/null && echo "!!! Kinect USB 오류: $(grep -acE 'LIBUSB_TRANSFER_ERROR' $D/logs/${TAG}_sensing.log)줄, lsusb 045e: $(lsusb | grep -c 045e)개"
echo "--- ~/.ros 새 파일: $(find $HOME/.ros -type f -newer $O/t0 | wc -l), /dev/shm Fast DDS: $(ls /dev/shm | grep -c fastrtps), 남은 프로세스: $(pgrep -fc 'step5_sensing_wrapper|step5_pose_recorder|step4_sys_monitor')"
grep -a "^\[sensing-wrapper\]" $D/logs/${TAG}_sensing.log | cut -c1-250; grep -a "^\[recorder\]" $D/logs/${TAG}_recorder.log
grep -aE "Traceback|Error|error" $D/logs/${TAG}_sensing.log | sort | uniq -c | head -10
/usr/bin/python3 -B $S/step5_analyze_1.py $O | tee $D/logs/${TAG}_analysis.txt

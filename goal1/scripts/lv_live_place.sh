#!/usr/bin/env bash
# lv_live_place.sh <tag> [lv_place.py args...] - live: placement mode. Starts sensing (real Kinect, IMU camera pose) and the
#   effmass_human node, then runs lv_place.py in this terminal: stand where you want facing the monitor, press Enter, hold 5 s.
#   On PASS it writes goal1/config/live_frame_user.yml (or --out). Then run lv_live_kinect.sh with LV_FRAME=<that file>.
#   Test mode: LV_PLACE_REPLAY=<camera-frame npz from lv_make_live_replay.py> replays a person instead of the Kinect (camera pose
#   from the npz), lv_place.py should then get --auto.
set -u
TAG="${1:?tag}"; shift
E=$HOME/effmass_mpc/env_check; S=$E/scripts; GG=$HOME/effmass_mpc/goal1; O=$GG/logs/meff_isaac; mkdir -p $O; P=$O/$TAG
busy=$(ps -eo comm=,args= | awk '$1 ~ /^python/ || $1 == "k4aviewer" || $1 == "sensing_start"' | grep -cE "sensing_start|k4aviewer|lv_sensing_wrapper")
if [[ $busy != 0 ]]; then echo "another sensing / k4aviewer is running ($busy); stop it first"; exit 3; fi
touch ${P}_t0
if [[ -n "${LV_PLACE_REPLAY:-}" ]]; then
  $GG/scripts/g1_run_human.sh py $GG/scripts/lv_replay_pose.py --npz $LV_PLACE_REPLAY --camera live --duration ${DURATION:-60} --out ${P}_replay.npz > ${P}_replay.log 2>&1 &
  SP=$!
else
  if ! lsusb -d 045e:097c > /dev/null || ! lsusb -d 045e:097d > /dev/null; then
    echo "Kinect depth/colour camera not on USB (lsusb 045e:097c / 097d missing); reconnect the Kinect and retry"; exit 5; fi
  echo "[place] starting sensing (Kinect + IMU) ..."
  $S/lv_run_sensing.sh ${P}_sensing > ${P}_sensing.log 2>&1 &
  SP=$!
  for i in $(seq 1 120); do grep -q "camera pose\] roll" ${P}_sensing.log 2>/dev/null && break; kill -0 $SP 2>/dev/null || break; sleep 1; done
  grep -a "camera pose\] roll" ${P}_sensing.log | sed 's/.*\[camera pose\]/[camera pose]/' | cut -c1-200
  grep -q "camera pose\] roll" ${P}_sensing.log || { echo "sensing did not report the camera pose; see ${P}_sensing.log"; kill -INT $SP 2>/dev/null; exit 4; }
fi
$GG/scripts/g1_run_human.sh node -p shapes:=v2 -p pelvis_correction:=true > ${P}_node.log 2>&1 &
NP=$!
$GG/scripts/g1_run_human.sh py $GG/scripts/lv_place.py "$@" 2>&1 | tee ${P}_place.log
rc=${PIPESTATUS[0]}
kill -INT $NP 2>/dev/null; wait $NP 2>/dev/null
kill -INT $SP 2>/dev/null; for i in $(seq 1 30); do kill -0 $SP 2>/dev/null || break; sleep 1; done; kill -0 $SP 2>/dev/null && kill $SP
left=$(ps -eo comm=,args= | awk '$1 ~ /^python/' | grep -cE 'effmass_human.human_inertia_node|lv_sensing_wrapper|lv_replay_pose|lv_place')
[[ $left == 0 ]] && find /dev/shm -maxdepth 1 \( -name "fastrtps_*" -o -name "sem.fastrtps_*" \) -newer ${P}_t0 -delete
exit $rc

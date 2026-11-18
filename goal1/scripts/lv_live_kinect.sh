#!/usr/bin/env bash
# lv_live_kinect.sh <tag> [steps] - live run with a person in front of the Kinect (virtual FR3 in Isaac, GUI).
#   1) hrc_ws sensing_node, real Kinect, via lv_run_sensing.sh (writes -> logs/meff_isaac/<tag>_sensing, no vis saving):
#      IMU roll/pitch + camera_pose.yaml yaw/t -> /hrc/camera_pose; body tracking -> /hrc/human_pose (after the body shape is estimated)
#   2) effmass_human node: /hrc/human_pose -> /effmass/human_inertia_state (pelvis correction, shapes v2, CPU 1 thread)
#   3) Isaac GUI: scene_live_front.json, live_frame.yml, live_t1.yml (prediction, filter, speed cap 0.5 m, loss hold 0.5 s),
#      ours k 0.10 (K_GAIN env to change), human capsules shown, status window "human status"
#   4) a terminal "stand monitor": pelvis offset from the standing place, distances, right hand vs the robot path
#   steps: Isaac physics steps (default 36000 = 10 min of simulated time). Stop: close the Isaac window or Ctrl+C here.
#   Before: no other sensing_node / k4aviewer may hold the Kinect.
set -u
TAG="${1:?tag}"; STEPS="${2:-36000}"
E=$HOME/effmass_mpc/env_check; S=$E/scripts; GG=$HOME/effmass_mpc/goal1; O=$GG/logs/meff_isaac; mkdir -p $O; P=$O/$TAG; G=$GG/b2a/g2
# python / k4aviewer processes only (a shell whose command text mentions these names is not counted)
busy=$(ps -eo comm=,args= | awk '$1 ~ /^python/ || $1 == "k4aviewer" || $1 == "sensing_start"' | grep -cE "sensing_start|k4aviewer|lv_sensing_wrapper")
if [[ $busy != 0 ]]; then echo "another sensing / k4aviewer is running ($busy); stop it first"; exit 3; fi
# the Kinect depth (045e:097c) and colour (045e:097d) cameras are on its USB 3 side; without them sensing stops at the IMU read
if ! lsusb -d 045e:097c > /dev/null || ! lsusb -d 045e:097d > /dev/null; then
  echo "Kinect depth/colour camera not on USB (lsusb 045e:097c / 097d missing); reconnect the Kinect (USB-C data cable, power) and retry"; exit 5; fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
rm -f $P.ready $P.done; touch ${P}_t0
echo "[live] $TAG: starting sensing (Kinect + IMU) ..."
$S/lv_run_sensing.sh ${P}_sensing > ${P}_sensing.log 2>&1 &
SP=$!
for i in $(seq 1 120); do grep -q "camera pose\] roll" ${P}_sensing.log 2>/dev/null && break; kill -0 $SP 2>/dev/null || break; sleep 1; done
grep -a "camera pose\] roll" ${P}_sensing.log | sed 's/.*\[camera pose\]/[camera pose]/' | cut -c1-200
grep -q "camera pose\] roll" ${P}_sensing.log || { echo "sensing did not report the camera pose; see ${P}_sensing.log"; kill -INT $SP 2>/dev/null; exit 4; }
$GG/scripts/g1_run_human.sh node -p shapes:=v2 -p pelvis_correction:=true -p perf_csv:=${P}_nodeperf.csv > ${P}_node.log 2>&1 &
NP=$!
DISPLAY=${DISPLAY:-:0} gnome-terminal --title "stand monitor ($TAG)" --geometry 150x20 -- $GG/scripts/g1_run_human.sh py $GG/scripts/lv_stand_monitor.py 2>/dev/null
echo "[live] Isaac GUI starting (about 30-60 s). Stand in view first so sensing can estimate the body shape, then go to the standing place."
EFFMASS_WS=1 RUN_ISAAC_NO_DISPLAY=0 ROS_METHOD=min $S/run_isaac_ros.sh $S/step2b_mpc_isaac.py --gui --scene g2 --robot fr3v2_1 --no-diag --warmup 10 \
  --tag $TAG --ros --ros-pose-topic none --human --human-mode full --human-source inertia --ready-file $P.ready --done-file $P.done --ros-out $P \
  --human-dir $G --human-frame $GG/config/live_frame.yml --scene-json $G/scene_live_front.json --round-trip 1 --profile $GG/config/live_t1.yml \
  --u-def nearest --meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 0 --cost-agg 0 \
  --meff-gain ${K_GAIN:-0.10} --mppi-seed 0 --meff-viz shapes --steps $STEPS > $P.log 2>&1
echo "isaac exit=$?"
kill -INT $NP 2>/dev/null; wait $NP
kill -INT $SP 2>/dev/null; for i in $(seq 1 30); do kill -0 $SP 2>/dev/null || break; sleep 1; done; kill -0 $SP 2>/dev/null && kill $SP
pkill -f "lv_stand_monitor.py" 2>/dev/null
left=$(ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|effmass_human.human_inertia_node|lv_sensing_wrapper|kit/python')
[[ $left == 0 ]] && rm -f /dev/shm/sem.carbonite-sharedmemory
[[ $left == 0 ]] && find /dev/shm -maxdepth 1 \( -name "fastrtps_*" -o -name "sem.fastrtps_*" \) -newer ${P}_t0 -delete
grep -aE "realtime_factor|solver_ms_mean|infeasible_steps|arrivals|\[human-loss\]" $P.log | tail -12 | cut -c1-200
echo "[live] logs: $P.log, ${P}_sensing.log, ${P}_node.log, ${P}_human_inertia_rec.npz"

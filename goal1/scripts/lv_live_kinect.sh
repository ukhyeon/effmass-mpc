#!/usr/bin/env bash
# lv_live_kinect.sh <tag> [steps] - live run with a person in front of the Kinect (virtual FR3 in Isaac, GUI).
#   1) hrc_ws sensing_node, real Kinect, via lv_run_sensing.sh (writes -> logs/meff_isaac/<tag>_sensing; Kinect colour frames with
#      their capture time -> goal1/live/<tag>/color after the run, LV_SAVE_COLOR=0 to turn off; no overlay render, no window):
#      IMU roll/pitch + camera_pose.yaml yaw/t -> /hrc/camera_pose; body tracking -> /hrc/human_pose (after the body shape is estimated)
#   2) effmass_human node: /hrc/human_pose -> /effmass/human_inertia_state (pelvis correction, shapes v2, CPU 1 thread)
#   3) Isaac GUI: scene_live_front.json, live_frame.yml, live_t1.yml (prediction, filter, speed cap 0.5 m, loss hold 0.5 s),
#      ours k 0.10 (K_GAIN env to change), human capsules shown, status window "human status";
#      zone rule: per-part reference by default (part_regen); LV_MUREF_MODE=global = previous single mu_ref; K_GAIN=0 = uniform
#   4) a terminal "stand monitor": pelvis offset from the standing place, distances, right hand vs the robot path
#   steps: Isaac physics steps (default 36000 = 10 min of simulated time). Stop early: Ctrl+C here (once) or close the Isaac
#   window; the loop ends after the current step and the run is saved (--graceful-stop 1, --stop-file <tag>.stopreq).
#   LV_FRAME=<file>: world -> Franka base transform (default goal1/config/live_frame.yml; goal1/config/live_frame_user.yml from
#   lv_live_place.sh puts the virtual robot in front of where you stood facing the monitor). The stand monitor follows it.
#   GUI camera: VIEW_MODE=shoulder (default: above and behind your right shoulder looking at the robot, so reaching forward goes
#   into the screen) or side (behind-left of the robot, you seen from the front); VIEW="eye_x,eye_y,eye_z,target_x,target_y,target_z"
#   (Franka base frame) overrides both.
#   GUI speed (live_prep2): the window is redrawn 10 times per wall second (LV_RENDER_HZ, 0 = every physics step as before), 960x540,
#   anti-aliasing off, human capsules moved only before a redraw. Physics, MPC, cost and the 60 Hz control are unchanged.
#   Before: no other sensing_node / k4aviewer may hold the Kinect.
#   After: lv_make_video.sh <tag> makes goal1/live/<tag>/<tag>_side_by_side.mp4 (colour | Isaac re-render from the run's records).
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
LV_SENSING_SAVEVIS=${LV_SAVE_COLOR:-1} $S/lv_run_sensing.sh ${P}_sensing > ${P}_sensing.log 2>&1 &
SP=$!
for i in $(seq 1 120); do grep -q "camera pose\] roll" ${P}_sensing.log 2>/dev/null && break; kill -0 $SP 2>/dev/null || break; sleep 1; done
grep -a "camera pose\] roll" ${P}_sensing.log | sed 's/.*\[camera pose\]/[camera pose]/' | cut -c1-200
grep -q "camera pose\] roll" ${P}_sensing.log || { echo "sensing did not report the camera pose; see ${P}_sensing.log"; kill -INT $SP 2>/dev/null; exit 4; }
$GG/scripts/g1_run_human.sh node -p shapes:=v2 -p pelvis_correction:=true -p perf_csv:=${P}_nodeperf.csv > ${P}_node.log 2>&1 &
NP=$!
FRAME=${LV_FRAME:-$GG/config/live_frame.yml}
[[ -f $FRAME ]] || { echo "frame file $FRAME not found (run lv_live_place.sh first)"; kill -INT $NP $SP 2>/dev/null; exit 6; }
SCENE=$G/scene_live_front.json
echo "[live] frame file $FRAME, scene $SCENE, mu_ref mode ${LV_MUREF_MODE:-part}"
DISPLAY=${DISPLAY:-:0} gnome-terminal --title "stand monitor ($TAG)" --geometry 150x20 -- $GG/scripts/g1_run_human.sh py $GG/scripts/lv_stand_monitor.py --frame $FRAME 2>/dev/null
echo "[live] Isaac GUI starting (about 30-60 s). Stand in view first so sensing can estimate the body shape, then go to the standing place."
# Ctrl+C here: the trap writes $P.stopreq; Isaac (own session, so the terminal's Ctrl+C does not reach Kit) stops after the
# current step and saves the run. sensing and the node stop on the same Ctrl+C. Closing the Isaac window also stops and saves.
rm -f $P.stopreq
trap 'echo "[live] Ctrl+C: Isaac stops after this step and saves (wait a few seconds)"; touch $P.stopreq' INT TERM
if [[ ${VIEW_MODE:-shoulder} == side ]]; then DEFVIEW="-1.2,-1.7,1.6,0.6,0.0,0.25"; else DEFVIEW="1.85,0.25,1.0,0.3,-0.05,0.3"; fi
VIEW=${VIEW:-$DEFVIEW}   # GUI camera eye,target in the Franka base frame
if [[ ${LV_RENDER_HZ:-10} == 0 ]]; then GUIOPT=""; else GUIOPT="--render-hz ${LV_RENDER_HZ:-10} --gui-res 960x540 --gui-aa 0 --gui-viz-defer 1"; fi
EFFMASS_WS=1 RUN_ISAAC_NO_DISPLAY=0 ROS_METHOD=min setsid $S/run_isaac_ros.sh $S/step2b_mpc_isaac.py --gui --scene g2 --robot fr3v2_1 --no-diag --warmup 10 \
  --tag $TAG --ros --ros-pose-topic none --human --human-mode full --human-source inertia --ready-file $P.ready --done-file $P.done --ros-out $P \
  --human-dir $G --human-frame $FRAME --scene-json $SCENE --round-trip 1 --profile $GG/config/live_t1.yml \
  --u-def nearest --meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 0 --cost-agg 0 \
  --meff-gain ${K_GAIN:-0.10} --meff-muref-mode ${LV_MUREF_MODE:-part} --mppi-seed 0 --meff-viz shapes --steps $STEPS --graceful-stop 1 --stop-file $P.stopreq --viz-cam="$VIEW" $GUIOPT > $P.log 2>&1 &
IP=$!
while kill -0 $IP 2>/dev/null; do wait $IP; done
wait $IP; echo "isaac exit=$?"
trap - INT TERM
kill -INT $NP 2>/dev/null; wait $NP
kill -INT $SP 2>/dev/null; for i in $(seq 1 30); do kill -0 $SP 2>/dev/null || break; sleep 1; done; kill -0 $SP 2>/dev/null && kill $SP
pkill -f "lv_stand_monitor.py" 2>/dev/null
left=$(ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|effmass_human.human_inertia_node|lv_sensing_wrapper|kit/python')
[[ $left == 0 ]] && rm -f /dev/shm/sem.carbonite-sharedmemory
[[ $left == 0 ]] && find /dev/shm -maxdepth 1 \( -name "fastrtps_*" -o -name "sem.fastrtps_*" \) -newer ${P}_t0 -delete
grep -aE "realtime_factor|solver_ms_mean|infeasible_steps|arrivals|\[human-loss\]" $P.log | tail -12 | cut -c1-200
echo "[live] logs: $P.log, ${P}_sensing.log, ${P}_node.log, ${P}_human_inertia_rec.npz"
if [[ -f ${P}_sensing/vis_times.csv ]]; then
  mkdir -p $GG/live/$TAG/color && rm -rf $GG/live/$TAG/color/vis_output && mv ${P}_sensing/vis_output ${P}_sensing/vis_times.csv $GG/live/$TAG/color/
  echo "[live] colour frames: $(ls $GG/live/$TAG/color/vis_output | wc -l) in $GG/live/$TAG/color ($(grep -a 'colour frames saved' ${P}_sensing.log | tail -1 | sed 's/.*\[lv-sensing\] //'))"
  echo "[live] video: $GG/scripts/lv_make_video.sh $TAG"
fi

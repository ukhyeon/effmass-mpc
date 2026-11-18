#!/usr/bin/env bash
# lv_live_run.sh <tag> <sensing 0|1> <gui 0|1> [step2b args...] - live prep timing run with the final live settings.
#   sensing 1: hrc_ws sensing_node with the real Kinect via lv_run_sensing.sh (writes -> <log dir>/<tag>_sensing, no vis saving);
#              it measures roll/pitch with the IMU and publishes /hrc/camera_pose. sensing 0: the replay publishes the camera pose
#              stored in the replay file (--camera live).
#   human: lv_replay_pose.py replays the camera-frame stream for the live standing place (LV_REPLAY, default
#          goal1/data/human_live_r1_v050_cam.npz) on /lv/human_pose; the effmass_human node is remapped to it
#          (pelvis correction on, shapes v2), so a person walking in front of the Kinect does not mix in.
#   Isaac: scene g2 live (scene_live_front.json), --human-frame live_frame.yml, --profile live_t1.yml, ROS path (inertia),
#          headless or GUI (--gui), plus the given step2b args.
set -u
TAG="${1:?tag}"; SENS="${2:?sensing 0|1}"; GUIF="${3:?gui 0|1}"; shift 3
left() { ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|effmass_human.human_inertia_node|lv_replay_pose|lv_sensing_wrapper|step4_sys_monitor|kit/python'; }
E=$HOME/effmass_mpc/env_check; S=$E/scripts; GG=$HOME/effmass_mpc/goal1; O=$GG/logs/meff_isaac; mkdir -p $O; P=$O/$TAG
G=$GG/b2a/g2; RP=${LV_REPLAY:-$GG/data/human_live_r1_v050_cam.npz}
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [[ $(left) != 0 ]]; then echo "[pre-check] stop: processes left $(left)"; exit 3; fi
rm -f $P.ready $P.done $P.stop
$S/step4_hrc_snapshot.sh ${P}_hrc_before > /dev/null; touch ${P}_t0
echo "[lv_live] tag $TAG sensing $SENS gui $GUIF replay $RP isaac args '$*' (start $(date +%T))"
/usr/bin/python3 -B $S/step4_sys_monitor.py --proc isaac=step2b_mpc_isaac.py --proc node=effmass_human.human_inertia_node --proc replay=lv_replay_pose.py \
  --proc sensing=lv_sensing_wrapper.py --out ${P}_sys_monitor.csv --stop-file $P.stop &
SM=$!
SP=""
if [[ $SENS == 1 ]]; then
  $S/lv_run_sensing.sh ${P}_sensing > ${P}_sensing.log 2>&1 &
  SP=$!
  for i in $(seq 1 120); do grep -q "camera pose\] roll" ${P}_sensing.log 2>/dev/null && break; sleep 1; done
  grep -a "camera pose\] roll" ${P}_sensing.log | cut -c1-220
  CAM="none"
else
  CAM="live"
fi
$GG/scripts/g1_run_human.sh node -p shapes:=v2 -p pelvis_correction:=true -p perf_csv:=${P}_nodeperf.csv -r /hrc/human_pose:=/lv/human_pose > ${P}_node.log 2>&1 &
NP=$!
$GG/scripts/g1_run_human.sh py $GG/scripts/lv_replay_pose.py --npz $RP --topic /lv/human_pose --camera $CAM --wait-file $P.ready --duration ${DURATION:-80} \
  --out ${P}_replay.npz > ${P}_replay.log 2>&1 &
RPP=$!
GUIARG=""; NODISP=1
if [[ $GUIF == 1 ]]; then GUIARG="--gui"; NODISP=0; fi
EFFMASS_WS=1 RUN_ISAAC_NO_DISPLAY=$NODISP ROS_METHOD=min timeout -s INT -k 20 ${TIMEOUT:-900} $S/run_isaac_ros.sh $S/step2b_mpc_isaac.py $GUIARG --scene g2 --robot fr3v2_1 \
  --no-diag --warmup 10 --tag $TAG --ros --ros-pose-topic none --human --human-mode full --human-source inertia --ready-file $P.ready --done-file $P.done --ros-out $P \
  --human-dir $G --human-frame $GG/config/live_frame.yml --scene-json $G/scene_live_front.json --round-trip 1 --profile $GG/config/live_t1.yml \
  --u-def nearest --meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 0 --cost-agg 0 "$@" > $P.log 2>&1
echo "isaac exit=$?"
kill -INT $RPP 2>/dev/null; wait $RPP; echo "replay exit=$?"
kill -INT $NP 2>/dev/null; wait $NP; echo "node exit=$?"
if [[ -n $SP ]]; then kill -INT $SP 2>/dev/null; for i in $(seq 1 30); do kill -0 $SP 2>/dev/null || break; sleep 1; done; kill -0 $SP 2>/dev/null && kill $SP; wait $SP; echo "sensing exit=$?"; fi
touch $P.stop; wait $SM 2>/dev/null
$S/step4_hrc_snapshot.sh ${P}_hrc_after > /dev/null
[[ $(left) == 0 ]] && rm -f /dev/shm/sem.carbonite-sharedmemory
[[ $(left) == 0 ]] && find /dev/shm -maxdepth 1 \( -name "fastrtps_*" -o -name "sem.fastrtps_*" \) -newer ${P}_t0 -print -delete | sed "s/^/[cleanup] removed /"
echo "--- ~/hrc_ws changes: $(diff ${P}_hrc_before_files.tsv ${P}_hrc_after_files.tsv | grep -c '^[<>]') lines; processes left: $(left)"
grep -a "^\[replay\]" ${P}_replay.log | tail -2
grep -aE "\[lv-sensing\] end|BLOCKED" ${P}_sensing.log 2>/dev/null | head -3 | cut -c1-200
grep -aE "realtime_factor|solver_ms_mean|infeasible_steps|arrivals|graph_captures" $P.log | tail -5 | cut -c1-200
grep -aE "Traceback" -A8 $P.log ${P}_node.log ${P}_replay.log | head -20

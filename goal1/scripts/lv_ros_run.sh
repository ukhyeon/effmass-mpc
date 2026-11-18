#!/usr/bin/env bash
# lv_ros_run.sh <tag> <human npz> [step2b args...] - live prep: one run through the whole ROS path, no Kinect.
#   lv_replay_pose.py (HumanPose 30 Hz, wall time, starts at <tag>.ready) -> effmass_human node (CPU 1 thread, shapes v2)
#   -> /effmass/human_inertia_state -> step2b --human-source inertia (scene and channel options from the arguments).
# Env: NODE_ARGS (extra node ros-args, default "-p shapes:=v2 -p pelvis_correction:=false"), REPLAY_ARGS (extra replay args,
#   default "--camera legacy"), DURATION (replay seconds, default 75). Shm pre-check skipped (entries of other sessions);
#   only shm entries newer than this run are removed at the end. Timeout: TIMEOUT (default 900 s) for the Isaac process.
set -u
left() { ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|effmass_human.human_inertia_node|lv_replay_pose|step4_sys_monitor|vram_monitor|nvidia-smi dmon|kit/python'; }
TAG="${1:?tag}"; HNPZ="${2:?human npz}"; shift 2
E=$HOME/effmass_mpc/env_check; S=$E/scripts; G=$HOME/effmass_mpc/goal1; O=$G/logs/meff_isaac; mkdir -p $O; P=$O/$TAG
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
if [[ $(left) != 0 ]]; then echo "[pre-check] stop: processes left $(left)"; exit 3; fi
rm -f $P.ready $P.done $P.end $P.stop
$S/step4_hrc_snapshot.sh ${P}_hrc_before > /dev/null; touch ${P}_t0
echo "[lv_ros] tag $TAG human $HNPZ isaac args '$*' node '${NODE_ARGS:--p shapes:=v2 -p pelvis_correction:=false}' replay '${REPLAY_ARGS:---camera legacy}' (start $(date +%T))"
/usr/bin/python3 -B $S/step4_sys_monitor.py --proc isaac=step2b_mpc_isaac.py --proc node=effmass_human.human_inertia_node --proc replay=lv_replay_pose.py --out ${P}_sys_monitor.csv --stop-file $P.stop &
SM=$!
$G/scripts/g1_run_human.sh node ${NODE_ARGS:--p shapes:=v2 -p pelvis_correction:=false} -p perf_csv:=${P}_nodeperf.csv > ${P}_node.log 2>&1 &
NP=$!
$G/scripts/g1_run_human.sh py $G/scripts/lv_replay_pose.py --npz $HNPZ --wait-file $P.ready --duration ${DURATION:-75} --out ${P}_replay.npz ${REPLAY_ARGS:---camera legacy} > ${P}_replay.log 2>&1 &
RP=$!
EFFMASS_WS=1 RUN_ISAAC_NO_DISPLAY="${RUN_ISAAC_NO_DISPLAY:-1}" ROS_METHOD=min timeout -s INT -k 20 ${TIMEOUT:-900} $S/run_isaac_ros.sh $S/step2b_mpc_isaac.py --scene g2 --robot fr3v2_1 --no-diag --warmup 10 \
  --tag $TAG --ros --ros-pose-topic none --human --human-mode full --human-source inertia --ready-file $P.ready --done-file $P.done --ros-out $P "$@" > $P.log 2>&1
echo "isaac exit=$?"
kill -INT $RP 2>/dev/null; wait $RP; echo "replay exit=$?"
kill -INT $NP 2>/dev/null; wait $NP; echo "node exit=$?"
touch $P.stop; wait $SM 2>/dev/null
$S/step4_hrc_snapshot.sh ${P}_hrc_after > /dev/null
[[ $(left) == 0 ]] && rm -f /dev/shm/sem.carbonite-sharedmemory
[[ $(left) == 0 ]] && find /dev/shm -maxdepth 1 \( -name "fastrtps_*" -o -name "sem.fastrtps_*" \) -newer ${P}_t0 -print -delete | sed "s/^/[cleanup] removed /"
echo "--- ~/hrc_ws changes: $(diff ${P}_hrc_before_files.tsv ${P}_hrc_after_files.tsv | grep -c '^[<>]') lines; processes left: $(left)"
grep -a "^\[replay\]" ${P}_replay.log | tail -2
grep -aE "realtime_factor|solver_ms_mean|infeasible_steps|arrivals|graph_captures" $P.log | tail -5 | cut -c1-200
grep -aE "Traceback" -A8 $P.log ${P}_node.log ${P}_replay.log | head -20

#!/usr/bin/env bash
# b2a_viz_run.sh <tag> [b2a_viz_check.py args] - human display check without MPC, same environment as g13_pre_run.sh (ROS paths, EFFMASS_WS=1)
set -u
TAG="${1:?tag}"; shift
S=$HOME/effmass_mpc/env_check/scripts; O=$HOME/effmass_mpc/goal1/logs/meff_isaac; P=$O/$TAG
left() { ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|b2a_viz_check|effmass_human.human_inertia_node|step4_replay|kit/python'; }
if [[ $(left) != 0 || $(ls /dev/shm | grep -vc '^ad_') != 0 ]]; then echo "[pre-check] stop: processes $(left), /dev/shm $(ls /dev/shm | grep -v '^ad_' | tr '\n' ' ')"; exit 3; fi
$S/step4_hrc_snapshot.sh ${P}_hrc_before > /dev/null; touch ${P}_t0
echo "[b2a-viz] $TAG args '$*' start $(date +%T), processes before 0, /dev/shm before 0"
EFFMASS_WS=1 RUN_ISAAC_NO_DISPLAY="${RUN_ISAAC_NO_DISPLAY:-1}" ROS_METHOD=min $S/run_isaac_ros.sh $HOME/effmass_mpc/goal1/scripts/b2a_viz_check.py --out ${P}.json "$@" > $P.log 2>&1
echo "isaac exit=$?"
$S/step4_hrc_snapshot.sh ${P}_hrc_after > /dev/null
[[ $(left) == 0 ]] && rm -f /dev/shm/sem.carbonite-sharedmemory
[[ $(left) == 0 ]] && find /dev/shm -maxdepth 1 \( -name "fastrtps_*" -o -name "sem.fastrtps_*" \) -newer ${P}_t0 -print -delete | sed "s/^/[cleanup] removed /"
echo "--- ~/hrc_ws changes: $(diff ${P}_hrc_before_files.tsv ${P}_hrc_after_files.tsv | grep -c '^[<>]'), /dev/shm left: $(ls /dev/shm | grep -v '^ad_' | tr '\n' ' '), processes left: $(left)"
grep -a "\[viz-check\]" $P.log | cut -c1-400; grep -aE "Traceback|Error\]" -A3 $P.log | grep -v "omni.kit\|Warning" | head -12

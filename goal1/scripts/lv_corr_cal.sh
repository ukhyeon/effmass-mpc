#!/usr/bin/env bash
# lv_corr_cal.sh <tag> <motion> <ver> [key=value ...] - live3m corrected paths: one calibration step for one motion: path version
#   (lv_corr_scenes.py prep), IK at the path ends (lv_corr_ik.py), scenes, then two runs with the human cost off (--meff off, seed 0):
#   start at A (first pass A -> B) and start at B (first pass B -> A); gap and direction per run from lv_corr_analyze.py.
#   Live settings (live_t1.yml), human = the held window of the motion at real time. One Isaac at a time.
# Env: CORR_RERUN=1 reruns finished runs.
set -u
TAG=${1:?tag}; Q=${2:?motion}; V=${3:?ver}; shift 3
GG=$HOME/effmass_mpc/goal1; O=$GG/logs/meff_isaac; SC=$GG/scripts; T=$GG/live/$TAG/corrected/cal/m$Q$V
n=$(ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|kit/python')
[[ $n == 0 ]] || { echo "an Isaac process is running (one Isaac at a time); stop it first"; exit 3; }
mkdir -p $T
if [[ ! -f $T/scene_AB.json || $# -gt 0 ]]; then
  $SC/g1_run_human.sh py $SC/lv_corr_scenes.py $TAG $Q $V prep "$@" 2>$T/prep_err.txt | tail -1 || exit 4
  $HOME/effmass_mpc/env_check/scripts/run_isaac.sh $SC/lv_corr_ik.py $T/ik_in.json $T/ik.json 2>/dev/null | tail -1
  $SC/g1_run_human.sh py $SC/lv_corr_scenes.py $TAG $Q $V finish 2>>$T/prep_err.txt | tail -1
  [[ -f $T/scene_AB.json ]] || { echo "no scene (see $T/prep_err.txt)"; exit 4; }
fi
N=$(python3 -c "import json;print(json.load(open('$T/path.json'))['steps'])")
for D in AB BA; do
  t=corr_${TAG}_m${Q}${V}_${D}_off
  if [[ ${CORR_RERUN:-0} != 1 && -f $O/$t.log ]] && grep -aq "saved .*_result_" $O/$t.log; then echo "$t reused"; continue; fi
  $SC/lv_run.sh $t 600 --scene g2 --shapes v2 --human-dir $GG/b2a/g2 --human-pre $T/human_v2.npz --human-frame $T/frame.yml --human-viz shapes --u-def nearest --cost-agg 0 \
    --profile $GG/config/live_t1.yml --scene-json $T/scene_$D.json --round-trip 1 --steps $N --meff off --b1-log 0 --mppi-seed 0 | head -1
done
$SC/g1_run_human.sh py $SC/lv_corr_analyze.py $TAG $Q $V off 2>$T/analyze_off_err.txt | grep -v "^$"

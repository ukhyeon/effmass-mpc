#!/usr/bin/env bash
# lv_tip_compare.sh <tag> <off|cmp|post> - live cmp tip: the motions of a live recording replayed at real time while the robot shuttles on a
#   path just in front of the fingertip (motions 1, 2) or the elbow (motion 3) (compare_tip/scenes.json from lv_tip_paths.py, lv_tip_ik.py,
#   lv_tip_scenes.py). Live settings (live_t1.yml), human = the motion with 2 s before and after.
#   off : human cost off (--meff off), seed 0, one run per motion -> compare_tip/analysis_off.json (round trip, fingertip / elbow gap)
#   cmp : uniform (k 0) and per-part reference (k 0.10), seeds 0 1 2; seed 0 also saves every 2nd step (VIEW of the live run, zone
#         shells on forearms, hands, elbows) -> compare_tip/analysis.json
#   post: clips (lv_tip_video.py) and REPORT.md with the figure (lv_tip_report.py)
# Env: TIP_RERUN=1 reruns finished runs (default: reuse), TIP_TIMEOUT per run (s, default 600).
set -u
TAG=${1:?tag}; ST=${2:?off|cmp|post}; GG=$HOME/effmass_mpc/goal1; O=$GG/logs/meff_isaac; G=$GG/b2a/g2; SC=$GG/scripts; T=$GG/live/$TAG/compare_tip
[[ -f $T/scenes.json ]] || { echo "no $T/scenes.json"; exit 2; }
n=$(ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|kit/python')
[[ $n == 0 ]] || { echo "an Isaac process is running (one Isaac at a time); stop it first"; exit 3; }
VIEW=$(python3 -c "import json;print(json.load(open('$GG/live/$TAG/compare/runs.json'))['view'])")
NM=$(python3 -c "import json;print(' '.join(str(q+1) for q,m in enumerate(json.load(open('$T/scenes.json'))['motions']) if m['status']=='ok'))")
mv_() { python3 -c "import json;m=json.load(open('$T/scenes.json'))['motions'][$1-1];print(m['$2'])"; }
done_() { [[ ${TIP_RERUN:-0} != 1 && -f $O/$1.log ]] && grep -aq "saved .*_result_" $O/$1.log; }
run_() {  # tag motion extra...
  local t=$1 q=$2; shift 2
  if done_ $t && { [[ $* != *viz-record* ]] || [[ -d $T/frames_$t ]]; }; then echo "$t reused"; return; fi
  rm -rf $T/frames_$t
  G1_VIZ_REC_EXT=jpg $SC/lv_run.sh $t ${TIP_TIMEOUT:-600} --scene g2 --shapes v2 --human-dir $G --human-pre $(mv_ $q human) --human-frame $(mv_ $q frame_file) \
    --human-viz shapes --u-def nearest --cost-agg 0 --profile $GG/config/live_t1.yml --scene-json $(mv_ $q scene) --round-trip 1 --steps $(mv_ $q steps) "$@" | head -1
  grep -aq "saved .*_result_" $O/$t.log || { echo "$t did not finish ($O/${t}_wrap.txt)"; exit 5; }
}
MEFF="--meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 1 --meff-muref-mode part"
if [[ $ST == off ]]; then
  J="{"; for q in $NM; do run_ tip_${TAG}_m${q}_off $q --meff off --b1-log 0 --mppi-seed 0; J="$J\"motion $q\": {\"off\": [\"tip_${TAG}_m${q}_off\"]},"; done; echo "${J%,}}" > $T/runs_off.json
  $SC/g1_run_human.sh py $SC/lv_tip_analyze.py $TAG $T/runs_off.json $T/analysis_off.json 2>$T/analyze_off_err.txt | grep "^motion"
elif [[ $ST == cmp ]]; then
  J="{"
  for q in $NM; do
    J="$J\"motion $q\": {"
    for mode in uni part; do
      if [[ $mode == uni ]]; then K=0; M=uniform; else K=0.10; M=part; fi; L=""
      for s in 0 1 2; do
        t=tip_${TAG}_m${q}_${mode}_s$s; REC=""
        [[ $s == 0 ]] && REC="--viz-eta-shell armelbow --viz-cam=$VIEW --viz-record $T/frames_$t --viz-record-res 640x360 --viz-record-every 2"
        run_ $t $q $MEFF --meff-gain $K --mppi-seed $s $REC; L="$L\"$t\","
      done
      J="$J\"$M\": [${L%,}],"
    done
    J="${J%,}},"
  done
  echo "${J%,}}" > $T/runs.json
  $SC/g1_run_human.sh py $SC/lv_tip_analyze.py $TAG $T/runs.json $T/analysis.json 2>$T/analyze_err.txt | grep "^motion"
else
  python3 $SC/lv_tip_video.py $TAG | tail -1 | cut -c1-300
  python3 $SC/lv_tip_report.py $TAG
fi

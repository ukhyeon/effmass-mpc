#!/usr/bin/env bash
# lv_corr_compare.sh <tag> <motion> <ver> <cmp|mesh> - live3m corrected paths, one motion on its calibrated path version
#   (goal1/live/<tag>/corrected/cal/m<motion><ver>/, from lv_corr_cal.sh). Live settings (live_t1.yml), human = the held window of
#   the motion at real time, the robot starts when the window starts.
#   cmp : uniform (k 0) and per-part reference (k 0.10), seeds 0 1 2, start at A and start at B -> analysis_cmp.json
#   mesh: seed 0, start at A, both methods re-rendered from the recorded joints (--playback-q) with the translucent SMPL-H mesh, zone
#         shells on forearms, hands, elbows; every 2nd step saved (640x360 jpg) to .../frames_<run>; camera from path.json "view"
# Env: CORR_RERUN=1 reruns finished runs; CORR_STARTS (default "AB BA"). One Isaac at a time.
set -u
TAG=${1:?tag}; Q=${2:?motion}; V=${3:?ver}; ST=${4:?cmp|mesh}
GG=$HOME/effmass_mpc/goal1; O=$GG/logs/meff_isaac; SC=$GG/scripts; T=$GG/live/$TAG/corrected/cal/m$Q$V
[[ -f $T/scene_AB.json ]] || { echo "no $T/scene_AB.json"; exit 2; }
busy() { ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|kit/python'; }
[[ $(busy) == 0 ]] || { echo "an Isaac process is running (one Isaac at a time); stop it first"; exit 3; }
N=$(python3 -c "import json;print(json.load(open('$T/path.json'))['steps'])")
MEFF="--meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --meff-muref-mode part"
COMMON="--scene g2 --shapes v2 --human-dir $GG/b2a/g2 --human-pre $T/human_v2.npz --human-frame $T/frame.yml --u-def nearest --cost-agg 0 --profile $GG/config/live_t1.yml --round-trip 1 --steps $N"
if [[ $ST == cmp ]]; then
  for D in ${CORR_STARTS:-AB BA}; do
    for mode in uni part; do
      [[ $mode == uni ]] && K=0 || K=0.10
      for s in 0 1 2; do
        t=corr_${TAG}_m${Q}${V}_${D}_${mode}_s$s
        if [[ ${CORR_RERUN:-0} != 1 && -f $O/$t.log ]] && grep -aq "saved .*_result_" $O/$t.log; then echo "$t reused"; continue; fi
        $SC/lv_run.sh $t 600 $COMMON --human-viz shapes --scene-json $T/scene_$D.json $MEFF --b1-log 1 --meff-gain $K --mppi-seed $s | head -1
        grep -aq "saved .*_result_" $O/$t.log || { echo "$t did not finish ($O/${t}_wrap.txt)"; exit 5; }
      done
    done
  done
  $SC/g1_run_human.sh py $SC/lv_corr_analyze.py $TAG $Q $V cmp 2>$T/analyze_cmp_err.txt | grep -v "^$"
else
  VIEW=$(python3 -c "import json;print(json.load(open('$T/path.json'))['view'])")
  for mode in uni part; do
    [[ $mode == uni ]] && K=0 || K=0.10
    src=corr_${TAG}_m${Q}${V}_AB_${mode}_s0; F=$T/frames_$src
    if [[ ${CORR_RERUN:-0} != 1 && -d $F/cam0 && $(ls $F/cam0 | wc -l) -ge $(( N / 2 - 5 )) ]]; then echo "$src frames reused"; continue; fi
    rm -rf $F
    python3 - "$src" "$T/${src}_q.npz" <<'PY'
import sys, json, re, csv, os, numpy as np
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac"); t, out = sys.argv[1], sys.argv[2]
r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", open(f"{O}/{t}.log", errors="replace").read())[-1]))
np.savez(out, q=np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))]))
PY
    G1_MESH_OPACITY=0.35 G1_VIZ_REC_EXT=jpg $SC/lv_run.sh mesh_$src 600 $COMMON --human-viz mesh --viz-hide-com on --scene-json $T/scene_AB.json $MEFF --b1-log 0 --meff-gain $K --mppi-seed 0 \
      --playback-q $T/${src}_q.npz --viz-eta-shell armelbow --viz-cam=$VIEW --viz-record $F --viz-record-res 640x360 --viz-record-every 2 | head -1
    echo "$src frames $(ls $F/cam0 2>/dev/null | wc -l)"
  done
fi

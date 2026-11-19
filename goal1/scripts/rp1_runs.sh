#!/usr/bin/env bash
# rp1_runs.sh <corr|r1|tip|slow> <acc> [extra] - real_prep1 runs (one Isaac at a time; finished runs are reused unless RP1_RERUN=1).
#   corr <15|10|8>   : corrected-path comparison of live3m (cal/m1b, cal/m3b: uniform k 0 and effective-mass k 0.10, seeds 0 1 2,
#                      start A and B) with the profile goal1/config/real_acc<acc>.yml (live_t1 + acceleration limit; 15 = live_t1.yml),
#                      command log and horizon log (--traj-log 1) on; a finished run without <tag>_traj.npz is rerun.
#                      Tags rp1_a<acc>_m<q>b_<AB|BA>_<uni|part>_s<seed>.
#   r1 <15|10|8>     : moving hand replay, fingertip 0.5 m/s (human_r1_reach_v050), settings of the r1 campaign (profile t1), b1 and
#                      command log on; 15 = the usual robot yml. Tags rp1_a<acc>_r1v050_<uni|part>_s<seed>.
#   tip <10|8>       : first front-pass path of live3m (compare_tip/scenes.json, whole motion with 2 s before and after), motions 2 and 3,
#                      both methods, seeds 0 1 2, profile real_acc<acc>.yml. Added because the steps above 10 rad/s^2 in the exported
#                      records are in these runs (tip_live3m_m2_*, m3_*). Tags rp1_a<acc>_tipm<q>_<uni|part>_s<seed>.
#   slow <acc> <name>: motion 1 on the corrected path, start A, seed 0, both methods, profile goal1/config/real_<name>.yml.
#                      Tags rp1_<name>_m1b_AB_<uni|part>_s0.
set -u
ST=${1:?corr|r1|slow}; ACC=${2:?acc}; NAME=${3:-}
GG=$HOME/effmass_mpc/goal1; O=$GG/logs/meff_isaac; SC=$GG/scripts; G=$GG/b2a/g2; D=$GG/data
busy() { ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|kit/python'; }
done_() { [[ ${RP1_RERUN:-0} != 1 && -f $O/$1.log ]] && grep -aq "saved .*_result_" $O/$1.log && { [[ " $* " != *" --traj-log 1 "* ]] || [[ -f $O/${1}_traj.npz ]]; }; }
MEFF="--meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --meff-muref-mode part"
one() {  # tag timeout args...
  local t=$1 to=$2; shift 2
  if done_ $t "$@"; then echo "$t reused"; return; fi
  [[ $(busy) == 0 ]] || { echo "an Isaac process is running; stop it first"; exit 3; }
  $SC/lv_run.sh $t $to "$@" | head -1
  grep -aq "saved .*_result_" $O/$t.log || echo "$t did not finish ($O/${t}_wrap.txt)"
}
corr_args() {  # q D
  local T=$GG/live/live3m/corrected/cal/m${1}b
  echo "--scene g2 --shapes v2 --human-dir $G --human-pre $T/human_v2.npz --human-frame $T/frame.yml --u-def nearest --cost-agg 0 --round-trip 1 --steps $(python3 -c "import json;print(json.load(open('$T/path.json'))['steps'])") --human-viz shapes --scene-json $T/scene_$2.json $MEFF --b1-log 1 --cmd-log 1 --traj-log 1"
}
if [[ $ST == corr ]]; then
  PF=$GG/config/real_acc$ACC.yml; [[ $ACC == 15 ]] && PF=$GG/config/live_t1.yml
  for q in 1 3; do for Dn in AB BA; do for mode in uni part; do [[ $mode == uni ]] && K=0 || K=0.10; for s in 0 1 2; do
    one rp1_a${ACC}_m${q}b_${Dn}_${mode}_s$s 600 $(corr_args $q $Dn) --profile $PF --meff-gain $K --mppi-seed $s
  done; done; done; done
elif [[ $ST == r1 ]]; then
  RC=""; [[ $ACC != 15 ]] && RC="--robot-cfg $GG/fr3v2_1/fr3v2_1_acc$ACC.yml"
  for mode in uni part; do [[ $mode == uni ]] && K=0.0 || K=0.10; for s in 0 1 2; do
    one rp1_a${ACC}_r1v050_${mode}_s$s 900 --scene g2 --shapes v2 --human-dir $G --human-viz none --u-def nearest $MEFF --b1-log 1 --cost-agg 0 --profile t1 \
      --scene-json $G/scene_b2a4_front_0.38h1.json --round-trip 1 --human-pre $D/human_r1_reach_v050.npz --meff-gain $K --mppi-seed $s --steps 3600 --cmd-log 1 $RC
  done; done
elif [[ $ST == tip ]]; then
  T=$GG/live/live3m/compare_tip
  for q in 2 3; do
    mv_() { python3 -c "import json;print(json.load(open('$T/scenes.json'))['motions'][$q-1]['$1'])"; }
    for mode in uni part; do [[ $mode == uni ]] && K=0 || K=0.10; for s in 0 1 2; do
      one rp1_a${ACC}_tipm${q}_${mode}_s$s 600 --scene g2 --shapes v2 --human-dir $G --human-pre $(mv_ human) --human-frame $(mv_ frame_file) --human-viz shapes --u-def nearest --cost-agg 0 \
        --profile $GG/config/real_acc$ACC.yml --scene-json $(mv_ scene) --round-trip 1 --steps $(mv_ steps) $MEFF --b1-log 1 --cmd-log 1 --meff-gain $K --mppi-seed $s
    done; done
  done
else
  for mode in uni part; do [[ $mode == uni ]] && K=0 || K=0.10
    one rp1_${NAME}_m1b_AB_${mode}_s0 600 $(corr_args 1 AB) --profile $GG/config/real_$NAME.yml --meff-gain $K --mppi-seed 0
  done
fi

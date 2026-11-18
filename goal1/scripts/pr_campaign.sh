#!/usr/bin/env bash
# pr_campaign.sh [groups...] - (part) runs of the per-part reference (--meff-muref-mode part) on the existing scenes, same settings
# and seeds as the global-mode runs they are compared with. Groups: check v2 knob on3 r1 on4 (default all, in this order).
# Each run: lv_run.sh with a timeout of 3x the usual time; a timeout is retried once. Table: goal1/part_regen/runs.tsv.
set -u
source $HOME/effmass_mpc/goal1/scripts/on_lib.sh
O=$HOME/effmass_mpc/goal1/logs/meff_isaac; T=$HOME/effmass_mpc/goal1/part_regen/runs.tsv; LR=$HOME/effmass_mpc/goal1/scripts/lv_run.sh
[[ -f $T ]] || printf "tag\tstart\tend\texit\tattempt\n" > $T
PART="--meff-muref-mode part"
V2A="--scene g2 --shapes v2 --human-pre $D/human_b2a_reach.npz --human-dir $G --human-viz none --u-def nearest --meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 1 --cost-agg 1 --profile t1"
run() {   # run <tag> <timeout> args...  (local variables only: a global "s" here clobbered the seed loop of the v2 group, see pr_campaign2.sh)
  local tag=$1 to=$2 att st ec; shift 2
  for att in 1 2; do
    st=$(date +%T); $LR $tag $to "$@" > /dev/null 2>&1; ec=$?
    printf "%s\t%s\t%s\t%s\t%s\n" $tag $st $(date +%T) $ec $att >> $T
    [[ $ec == 124 || $ec == 137 ]] || return $ec
  done
}
G_="${@:-check v2 knob on3 r1 on4}"
for grp in $G_; do case $grp in
  check)
    $HOME/effmass_mpc/goal1/scripts/lv_base.sh b11 > $HOME/effmass_mpc/goal1/part_regen/check_base_b11.txt 2>&1
    run pr_check_k0_front_s0 360 $COMMON $PART --scene-json $G/scene_b2a4_front_0.38h1.json --single-pass 1 --human-pre $D/human_b2a_reach.npz --meff-gain 0.0 --mppi-seed 0 --steps 304
    run pr_check_timing_front_s0 360 $COMMON $PART --scene-json $G/scene_b2a4_front_0.38h1.json --single-pass 1 --human-pre $D/human_b2a_reach.npz --meff-gain 0.10 --mppi-seed 0 --steps 304 ;;
  v2)
    for s in 0 1 2; do
      run pr_v2_front_part_s$s 360 $V2A $PART --scene-json $G/scene_b2a4_front_0.38h1.json --single-pass 1 --meff-gain 0.10 --mppi-seed $s --steps 304
      run pr_v2_top_part_s$s 450 $V2A $PART --scene-json $G/scene_b2a4_top_0.455.json --single-pass 1 --meff-gain 0.10 --mppi-seed $s --steps 506
    done ;;
  knob)
    for k in 0.05 0.15; do run pr_v2_front_partk${k/0./0}_s0 360 $V2A $PART --scene-json $G/scene_b2a4_front_0.38h1.json --single-pass 1 --meff-gain $k --mppi-seed 0 --steps 304; done ;;
  on3)
    for sc in P2_tipfront P2_top P3_tipfront P3_top25 P4_tipfront P4_top P6_forearmside18 P6_tipfront25 P6_top18; do for s in 0 1 2; do
      run on3_${sc}_part_s$s 360 $COMMON $PART --scene-json $G/scene_b2a4_front_0.38h1.json --single-pass 1 --human-pre $D/human_on3_${sc}_v2.npz --meff-gain 0.10 --mppi-seed $s --steps 420
    done; done ;;
  r1)
    for v in v025 v050 v100; do for s in 0 1 2; do n=3600; to=900; [[ $v == v100 ]] && { n=7200; to=1500; }
      run r1_${v}_part_s$s $to $COMMON $PART --scene-json $G/scene_b2a4_front_0.38h1.json --round-trip 1 --human-pre $D/human_r1_reach_${v}.npz --meff-gain 0.10 --mppi-seed $s --steps $n
    done; done ;;
  on4)
    for m in M1 M2; do for v in v025 v050; do for s in 0 1 2; do
      run on4_${m}_${v}_part_s$s 900 $COMMON $PART --scene-json $G/scene_b2a4_front_0.38h1.json --round-trip 1 --human-pre $D/human_on4_${m}_${v}_v2.npz --meff-gain 0.10 --mppi-seed $s --steps 3600
    done; done; done ;;
esac; done
echo "== campaign done $(date +%T)" >> $T

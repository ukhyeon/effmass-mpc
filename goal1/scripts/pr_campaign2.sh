#!/usr/bin/env bash
# pr_campaign2.sh [groups...] - (part) follow-up runs: v2top (V2 hand-top scene, part, seeds 0-2; pr_campaign.sh lost them to a
# variable clash) and live (replays of the live recordings run3 / run5_ours in both modes, live_t1 profile, their frame files).
set -u
source $HOME/effmass_mpc/goal1/scripts/on_lib.sh
T=$HOME/effmass_mpc/goal1/part_regen/runs.tsv; LR=$HOME/effmass_mpc/goal1/scripts/lv_run.sh; C=$HOME/effmass_mpc/goal1/config
V2A="--scene g2 --shapes v2 --human-pre $D/human_b2a_reach.npz --human-dir $G --human-viz none --u-def nearest --meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 1 --cost-agg 1 --profile t1"
LVA="--scene g2 --shapes v2 --human-dir $G --human-viz none --u-def nearest --meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 0 --cost-agg 0 --profile $C/live_t1.yml --scene-json $G/scene_live_front.json --round-trip 1 --meff-gain 0.10 --mppi-seed 0"
run() {   # run <tag> <timeout> args...  (local variables only)
  local tag=$1 to=$2 att st ec; shift 2
  for att in 1 2; do
    st=$(date +%T); $LR $tag $to "$@" > /dev/null 2>&1; ec=$?
    printf "%s\t%s\t%s\t%s\t%s\n" $tag $st $(date +%T) $ec $att >> $T
    [[ $ec == 124 || $ec == 137 ]] || return $ec
  done
}
for grp in ${@:-v2top live}; do case $grp in
  v2top)
    for seed in 0 1 2; do run pr_v2_top_part_s$seed 450 $V2A --meff-muref-mode part --scene-json $G/scene_b2a4_top_0.455.json --single-pass 1 --meff-gain 0.10 --mppi-seed $seed --steps 506; done ;;
  live)
    for spec in "run3 live_frame.yml 4992" "run5_ours live_frame_user.yml 4668"; do
      set -- $spec
      for mode in global part; do run pr_live_${1}_${mode} 900 $LVA --meff-muref-mode $mode --human-pre $D/human_live_${1}_v2.npz --human-frame $C/$2 --steps $3; done
    done ;;
esac; done
echo "== campaign2 done $(date +%T)" >> $T

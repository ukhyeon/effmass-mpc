#!/usr/bin/env bash
# t1_campaign.sh "scenekey cond seed [suffix]" ... - T1 tuning runs on the B2a-4 scenes (scene_b2a4_<scenekey>.json); same as b2a4_campaign.sh plus T1_ARGS (speed cap, pose rot weight, profile).
# static human (B2a reach pose), shapes v2, u-def nearest, set A, max_all, static human time. Steps from env N (all specs in the call).
# Conditions: off uni10 unim iso massA (unim eta from env ETA_M).
R=$HOME/effmass_mpc/goal1/scripts/g13_pre_run.sh; L=$HOME/effmass_mpc/goal1/logs/meff_isaac/t1_campaign.txt; : >> $L
G=$HOME/effmass_mpc/goal1/b2a/g2
SA="--meff-eta0 0.10 --meff-muref 2.0 --meff-p 0.5 --meff-eta-min 0.05 --meff-eta-max 0.20 --meff-agg max_all"
for spec in "$@"; do
  set -- $spec; p=$1; c=$2; s=$3; tag=t1_${T1_SET:?T1_SET}_${p}_${c}_s${s}${4:-}; n=${N:?N}
  [[ -f $G/scene_b2a4_${p}.json ]] || { echo "no scene $p"; exit 2; }
  case $c in
    off) a="--meff-mode off" ;;
    uni10) a="--meff-mode uniform --meff-eta0 0.10 --meff-agg max_all" ;;
    unim) a="--meff-mode uniform --meff-eta0 ${ETA_M:?ETA_M} --meff-agg max_all" ;;
    iso) a="--meff-mode iso $SA" ;;
    massA) a="--meff-mode mass $SA" ;;
  esac
  echo "=== $tag $(date +%T) steps $n" >> $L
  $R $tag --scene g2 --scene-json $G/scene_b2a4_${p}.json --single-pass 1 --shapes v2 --human-pre ~/effmass_mpc/goal1/data/human_b2a_reach.npz --human-dir $G \
     --human-viz none --u-def nearest --meff on $a --mppi-seed $s --b1-log 1 --cost-agg 1 --steps $n ${T1_ARGS:-} ${B2A_EXTRA:-} >> $L 2>&1
done
echo "=== batch done $(date +%T)" >> $L

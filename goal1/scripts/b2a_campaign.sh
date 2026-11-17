#!/usr/bin/env bash
# b2a_campaign.sh "path cond seed [suffix]" ... - B2a runs: scene g2, single pass A -> B, shapes v2, static human, u-def nearest, set A, max_all.
# Steps per path from env N_FRONT / N_TOP (2 x off arrival time). Conditions: off uni10 unim iso massA (unim eta from env ETA_M).
R=$HOME/effmass_mpc/goal1/scripts/g13_pre_run.sh; L=$HOME/effmass_mpc/goal1/logs/meff_isaac/b2a_campaign.txt; : >> $L
G=$HOME/effmass_mpc/goal1/b2a/g2
SA="--meff-eta0 0.10 --meff-muref 2.0 --meff-p 0.5 --meff-eta-min 0.05 --meff-eta-max 0.20 --meff-agg max_all"
for spec in "$@"; do
  set -- $spec; p=$1; c=$2; s=$3; tag=b2a_${p}_${c}_s${s}${4:-}
  case $p in front) n=${N_FRONT:?N_FRONT} ;; top) n=${N_TOP:?N_TOP} ;; esac
  case $c in
    off) a="--meff-mode off" ;;
    uni10) a="--meff-mode uniform --meff-eta0 0.10 --meff-agg max_all" ;;
    unim) a="--meff-mode uniform --meff-eta0 ${ETA_M:?ETA_M} --meff-agg max_all" ;;
    iso) a="--meff-mode iso $SA" ;;
    massA) a="--meff-mode mass $SA" ;;
  esac
  echo "=== $tag $(date +%T) steps $n" >> $L
  $R $tag --scene g2 --scene-json $G/scene_g2_${p}.json --single-pass 1 --shapes v2 --human-pre ~/effmass_mpc/goal1/data/human_b2a_reach.npz --human-dir $G \
     --human-viz none --u-def nearest --meff on $a --mppi-seed $s --b1-log 1 --cost-agg 1 --steps $n ${B2A_EXTRA:-} >> $L 2>&1
done
echo "=== batch done $(date +%T)" >> $L

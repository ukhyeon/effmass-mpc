#!/usr/bin/env bash
# on_wp1.sh "speed cond seed variant" ... - WP1 variant runs on the R1 scene (front, round trips, moving hand human_r1_reach_<speed>).
# Variants: base, L01 / L02 (shapes moved ahead by file velocity x 0.1 / 0.2 s), P (--meff-motion cv), V075 / V100 (diagnostic profiles, speed cap 0.75 / 1.0 m/s),
# combinations joined with "+" (e.g. L01+V075). Steps 3600 (v025, v050) or 7200 (v100).
source $HOME/effmass_mpc/goal1/scripts/on_lib.sh
for spec in "$@"; do
  set -- $spec; v=$1; c=$2; s=$3; var=$4; tag=on1_${v}_${c}_s${s}_${var//+/_}; n=3600; to=230; [[ $v == v100 ]] && { n=7200; to=420; }
  extra=""; prof="--profile t1"
  for part in ${var//+/ }; do
    case $part in
      base) ;; L01) extra+=" --human-lead-s 0.1" ;; L02) extra+=" --human-lead-s 0.2" ;; P) extra+=" --meff-motion cv" ;;
      V075) prof="--profile d1v075" ;; V100) prof="--profile d1v100" ;; *) echo "bad variant $part"; exit 2 ;;
    esac
  done
  $RUN $tag $to ${COMMON/--profile t1/} $prof --scene-json $G/scene_b2a4_front_0.38h1.json --round-trip 1 --human-pre $D/human_r1_reach_${v}.npz \
       --meff-gain $(kgain $c) --mppi-seed $s --steps $n $extra
done

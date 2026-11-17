#!/usr/bin/env bash
# on_wp4.sh "motion speed cond seed [variant]" ... - WP4 moving collaboration scenes: front robot path, round trips, 60 s (3600 steps),
# human = goal1/data/human_on4_<motion>_<speed>_v2.npz (M1 hand-over, M2 crossing; speed v025 / v050). Optional variant as in on_wp1.sh (L01, L02, P, "+" joins).
source $HOME/effmass_mpc/goal1/scripts/on_lib.sh
for spec in "$@"; do
  set -- $spec; m=$1; v=$2; c=$3; s=$4; var=${5:-base}; tag=on4_${m}_${v}_${c}_s${s}; [[ $var != base ]] && tag+="_${var//+/_}"; extra=""; prof="--profile t1"
  for part in ${var//+/ }; do
    case $part in base) ;; L01) extra+=" --human-lead-s 0.1" ;; L02) extra+=" --human-lead-s 0.2" ;; P) extra+=" --meff-motion cv" ;; F) extra+=" --human-vel-alpha 0.2" ;; C2) prof="--profile t2a_c2" ;; *) echo "bad variant $part"; exit 2 ;; esac
  done
  $RUN $tag 230 ${COMMON/--profile t1/} $prof --scene-json $G/scene_b2a4_front_0.38h1.json --round-trip 1 --human-pre $D/human_on4_${m}_${v}_v2.npz --meff-gain $(kgain $c) --mppi-seed $s --steps 3600 $extra
done

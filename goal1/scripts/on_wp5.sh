#!/usr/bin/env bash
# on_wp5.sh "cond seed variant" ... - WP5 perception imperfection on the R1 0.5 m/s scene (front, round trips, human_r1_reach_v050, 60 s).
# Variants: lag050 / lag100 / lag150 (frame of t - lag), jit1 / jit2 (per frame per segment Gaussian offset, std 1 / 2 cm, noise seed 7),
# drop30 / drop60 (30 / 60 % of frames missing, last frame kept, noise seed 7), lag300, jit4 (extension levels), L01 (lead 0.1 s), P (cv); "+" joins.
source $HOME/effmass_mpc/goal1/scripts/on_lib.sh
# part C2 = profile t2a_c2 instead of t1 (follow-up F6)
for spec in "$@"; do
  set -- $spec; c=$1; s=$2; var=$3; tag=on5_v050_${c}_s${s}_${var//+/_}; extra=""; prof="--profile t1"
  for part in ${var//+/ }; do
    case $part in
      lag050) extra+=" --human-lag-s 0.05" ;; lag100) extra+=" --human-lag-s 0.10" ;; lag150) extra+=" --human-lag-s 0.15" ;;
      jit1) extra+=" --human-jitter-m 0.01 --human-noise-seed 7" ;; jit2) extra+=" --human-jitter-m 0.02 --human-noise-seed 7" ;;
      F) extra+=" --human-vel-alpha 0.2" ;; C2) prof="--profile t2a_c2" ;;
      drop30) extra+=" --human-drop-frac 0.3 --human-noise-seed 7" ;; drop60) extra+=" --human-drop-frac 0.6 --human-noise-seed 7" ;;
      lag300) extra+=" --human-lag-s 0.30" ;; jit4) extra+=" --human-jitter-m 0.04 --human-noise-seed 7" ;; L01) extra+=" --human-lead-s 0.1" ;; P) extra+=" --meff-motion cv" ;;
      *) echo "bad variant $part"; exit 2 ;;
    esac
  done
  $RUN $tag 230 ${COMMON/--profile t1/} $prof --scene-json $G/scene_b2a4_front_0.38h1.json --round-trip 1 --human-pre $D/human_r1_reach_v050.npz --meff-gain $(kgain $c) --mppi-seed $s --steps 3600 $extra
done

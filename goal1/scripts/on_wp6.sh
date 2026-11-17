#!/usr/bin/env bash
# on_wp6.sh "scene cond seed cand" ... - WP6 tuning runs: WP3 static scene (front path, single pass, 420 steps) with profile tuned_t2a_<cand> instead of t1.
source $HOME/effmass_mpc/goal1/scripts/on_lib.sh
for spec in "$@"; do
  set -- $spec; sc=$1; c=$2; s=$3; cand=$4; tag=on6_${sc}_${c}_s${s}_${cand}
  $RUN $tag 120 ${COMMON/--profile t1/} --profile t2a_${cand} --scene-json $G/scene_b2a4_front_0.38h1.json --single-pass 1 --human-pre $D/human_on3_${sc}_v2.npz \
       --meff-gain $(kgain $c) --mppi-seed $s --steps 420
done

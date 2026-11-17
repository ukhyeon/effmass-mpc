#!/usr/bin/env bash
# on_wp3.sh "scene cond seed [suffix]" ... - WP3 static scenes: front robot path (scene_b2a4_front_0.38h1.json), single pass, 420 steps,
# human = goal1/data/human_on3_<scene>_v2.npz (placed posture; scene = <P>_<dir>), conditions off / gk000 (uniform) / gain (ours) / gk005 / gk015.
source $HOME/effmass_mpc/goal1/scripts/on_lib.sh
for spec in "$@"; do
  set -- $spec; sc=$1; c=$2; s=$3; tag=on3_${sc}_${c}_s${s}${4:-}
  if [[ $c == off ]]; then a="--meff-mode off"; else a="--meff-gain $(kgain $c)"; fi
  $RUN $tag 120 $COMMON --scene-json $G/scene_b2a4_front_0.38h1.json --single-pass 1 --human-pre $D/human_on3_${sc}_v2.npz $a --mppi-seed $s --steps ${N:-420}
done

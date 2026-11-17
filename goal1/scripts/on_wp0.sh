#!/usr/bin/env bash
# on_wp0.sh - WP0 baseline runs: V2 front static scene (single pass, 304 steps) uniform k 0 and ours k 0.10 seed 0, and one 60 s round trip (R1 v050 uniform seed 0).
source $HOME/effmass_mpc/goal1/scripts/on_lib.sh
for c in gk000 gain; do
  $RUN on_base_${c}_s0 600 $COMMON --scene-json $G/scene_b2a4_front_0.38h1.json --single-pass 1 --human-pre $D/human_b2a_reach.npz --meff-gain $(kgain $c) --mppi-seed 0 --steps 304
done
$RUN on_r1rep_v050_gk000_s0 900 $COMMON --scene-json $G/scene_b2a4_front_0.38h1.json --round-trip 1 --human-pre $D/human_r1_reach_v050.npz --meff-gain 0.0 --mppi-seed 0 --steps 3600

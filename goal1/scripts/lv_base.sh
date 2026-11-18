#!/usr/bin/env bash
# lv_base.sh <suffix> - live-prep regression check: rerun the two V2 front baseline runs (uniform k 0 and ours k 0.10, seed 0,
# single pass, 304 steps; same arguments as on_wp0.sh) as lv_base_<cond>_s0_<suffix> and compare the executed joint commands
# bit by bit with on_base_<cond>_s0 (on_bitcmp.py). Does not touch overnight/runs.tsv. Exit 0 only if both are bit-equal.
set -u
SUF=${1:?suffix}
source $HOME/effmass_mpc/goal1/scripts/on_lib.sh
R=$HOME/effmass_mpc/goal1/scripts/g13_pre_run.sh; O=$HOME/effmass_mpc/goal1/logs/meff_isaac
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export G13_SHM_CHECK=0   # other sessions left fastrtps shm entries; each run still removes only what it created
ok=0
for c in gk000 gain; do
  tag=lv_base_${c}_s0_${SUF}
  timeout -s INT -k 20 600 $R $tag $COMMON --scene-json $G/scene_b2a4_front_0.38h1.json --single-pass 1 --human-pre $D/human_b2a_reach.npz \
    --meff-gain $(kgain $c) --mppi-seed 0 --steps 304 > $O/${tag}_wrap.txt 2>&1
  echo "$tag exit $?"
  r=$(python3 -B $HOME/effmass_mpc/goal1/scripts/on_bitcmp.py on_base_${c}_s0 $tag)
  echo "$r"
  echo "$r" | grep -q '"bit_equal": true' || ok=1
done
exit $ok

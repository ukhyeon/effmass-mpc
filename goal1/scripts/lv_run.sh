#!/usr/bin/env bash
# lv_run.sh <tag> <timeout_s> [step2b args...] - live-prep single run: g13_pre_run.sh with a timeout (process group INT, then KILL),
# shm pre-check skipped (G13_SHM_CHECK=0, entries left by other sessions); g13_pre_run.sh removes only what the run created.
set -u
TAG=${1:?tag}; TO=${2:?timeout}; shift 2
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 G13_SHM_CHECK=0
O=$HOME/effmass_mpc/goal1/logs/meff_isaac
timeout -s INT -k 20 $TO $HOME/effmass_mpc/goal1/scripts/g13_pre_run.sh $TAG "$@" > $O/${TAG}_wrap.txt 2>&1; ec=$?
echo "$TAG exit $ec"
grep -aE "realtime_factor|solver_ms_mean|infeasible_steps|arrivals|graph_captures" $O/$TAG.log | tail -5 | cut -c1-160
exit $ec

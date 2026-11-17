#!/usr/bin/env bash
# on_run.sh <tag> <timeout_s> [step2b args...] - overnight single run wrapper around g13_pre_run.sh.
# Records wall time (start to end) in goal1/overnight/runs.tsv. Kills the run (whole process group) after timeout_s,
# cleans only what this run left (Isaac processes of this session, carbonite semaphore, fastrtps shm newer than the run start),
# then retries once. A second timeout is recorded as "skipped". Exit 0 ok, 5 timed out twice, other = pre-run exit code.
set -u
TAG=${1:?tag}; TO=${2:?timeout}; shift 2
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
R=$HOME/effmass_mpc/goal1/scripts/g13_pre_run.sh; T=$HOME/effmass_mpc/goal1/overnight/runs.tsv; O=$HOME/effmass_mpc/goal1/logs/meff_isaac
[[ -f $T ]] || printf "tag\tstart\tend\twall_s\texit\tattempt\tnote\n" > $T
pat='step2b_mpc_isaac|effmass_human.human_inertia_node|step4_sys_monitor|vram_monitor|nvidia-smi dmon|kit/python'
cleanup() {
  pkill -f -INT 'step2b_mpc_isaac' 2>/dev/null; sleep 3
  for p in $(ps -eo pid=,comm=,args= | awk '$2!="bash" && $2!="grep" && $2!="sh"' | grep -E "$pat" | awk '{print $1}'); do kill -9 $p 2>/dev/null; done
  sleep 1
  rm -f /dev/shm/sem.carbonite-sharedmemory
  find /dev/shm -maxdepth 1 \( -name "fastrtps_*" -o -name "sem.fastrtps_*" \) -newer $1 -delete 2>/dev/null
}
for att in 1 2; do
  t0=$(date +%s.%N); s=$(date +%T)
  mk=$O/${TAG}_on_t0; touch $mk
  timeout -s INT -k 20 $TO $R $TAG "$@" > $O/${TAG}_wrap.txt 2>&1; ec=$?
  t1=$(date +%s.%N); w=$(python3 -c "print(round($t1-$t0,1))")
  if [[ $ec == 124 || $ec == 137 ]]; then
    printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\n" $TAG $s $(date +%T) $w $ec $att timeout >> $T; cleanup $mk
    [[ $att == 2 ]] && { printf "%s\t-\t-\t-\t-\t-\tskipped after 2 timeouts\n" $TAG >> $T; exit 5; }
    continue
  fi
  printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\n" $TAG $s $(date +%T) $w $ec $att ok >> $T
  exit $ec
done

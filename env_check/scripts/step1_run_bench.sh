#!/usr/bin/env bash
# step1_run_bench.sh <tag> [step1_franka_headless.py 인자...]
#   VRAM 1초 기록을 켠 채 step1_franka_headless.py 를 run_isaac.sh 로 실행하고, 끝나면 요약한다.
#   결과: logs/<tag>.log (stdout+stderr), logs/<tag>_vram.csv, logs/<tag>_summary.txt
set -u
TAG="${1:?tag}"; shift
D="$HOME/effmass_mpc/env_check"
LOG="$D/logs/$TAG.log"; CSV="$D/logs/${TAG}_vram.csv"; SUM="$D/logs/${TAG}_summary.txt"
"$D/scripts/vram_monitor.sh" "$CSV" & MON=$!
sleep 2   # 기동 전 기준 VRAM 샘플 확보
T0=$(date +%s.%N)
"$D/scripts/run_isaac.sh" "$D/scripts/step1_franka_headless.py" "$@" > "$LOG" 2>&1
RC=$?
T1=$(date +%s.%N)
sleep 2; kill $MON 2>/dev/null; wait $MON 2>/dev/null
{
  echo "tag=$TAG  args=$*  exit=$RC"
  printf "process_wall_s=%.1f\n" "$(echo "$T1 - $T0" | bc)"
  echo "baseline_vram_MiB=$(sed -n 2p "$CSV" | cut -d, -f2)"
  "$D/scripts/vram_summary.sh" "$CSV"
  grep -aE "^\[step1\]" "$LOG"
} > "$SUM"
cat "$SUM"

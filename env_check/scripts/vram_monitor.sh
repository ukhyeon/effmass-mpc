#!/usr/bin/env bash
# vram_monitor.sh <out.csv>  — nvidia-smi 를 1초 간격으로 기록 (Ctrl-C/kill 로 종료)
# 열: timestamp, memory.used[MiB], memory.total[MiB], utilization.gpu[%]
out="${1:?출력 csv 경로}"
echo "timestamp,memory.used.MiB,memory.total.MiB,util.gpu.pct" > "$out"
while true; do
  nvidia-smi --query-gpu=timestamp,memory.used,memory.total,utilization.gpu --format=csv,noheader,nounits \
    | sed 's/, */,/g' >> "$out"
  sleep 1
done

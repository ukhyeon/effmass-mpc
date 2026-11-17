#!/usr/bin/env bash
# vram_summary.sh <csv>  — 최대/평균 VRAM, 샘플 수 출력
f="${1:?csv}"
awk -F, 'NR>1 {n++; s+=$2; if($2>m){m=$2; t=$1}} END {printf "samples=%d  max_used=%d MiB (at %s)  mean_used=%.0f MiB\n", n, m, t, s/n}' "$f"

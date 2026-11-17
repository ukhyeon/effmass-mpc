#!/usr/bin/env bash
# 1-3 A단계 (2): precomputed(결정적 재생) 비교 실행. 시간 판정에 쓰지 않음
R=$HOME/effmass_mpc/goal1/scripts/g13_pre_run.sh; L=$HOME/effmass_mpc/goal1/logs/meff_isaac/g13_campaign_p.txt; : >> $L
run() { echo "=== $1 $(date +%T)" >> $L; top -b -n 1 -o %CPU | sed -n 7,10p >> $L; $R "$@" >> $L 2>&1; }
for a in "$@"; do eval "run $a"; done
echo "=== done $(date +%T)" >> $L

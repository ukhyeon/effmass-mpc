#!/usr/bin/env bash
# 1-3 A단계 (1): 동작·분포 실행. 장면 g1, (F) 설정, 실행된 상태의 근접 쌍 기록 (0.3 m). 시간 판정에 쓰지 않음
R=$HOME/effmass_mpc/goal1/scripts/g1_meff_run.sh; L=$HOME/effmass_mpc/goal1/logs/meff_isaac/g13_campaign_a.txt; : > $L
run() { echo "=== $1 $(date +%T)" >> $L; top -b -n 1 -o %CPU | sed -n 7,10p >> $L; $R "$@" >> $L 2>&1; ls /dev/shm | grep -q fastrtps && echo "!!! /dev/shm fastrtps 남음" >> $L; }
run g13_off_near --meff on --meff-mode off --meff-log-near 0.3
run g13_uni10_a --meff on --meff-mode uniform --meff-eta0 0.10 --meff-log-near 0.3
run g13_uni10_b --meff on --meff-mode uniform --meff-eta0 0.10 --meff-log-near 0.3
run g13_uni05 --meff on --meff-mode uniform --meff-eta0 0.05 --meff-log-near 0.3
run g13_uni15 --meff on --meff-mode uniform --meff-eta0 0.15 --meff-log-near 0.3
echo "=== done $(date +%T)" >> $L

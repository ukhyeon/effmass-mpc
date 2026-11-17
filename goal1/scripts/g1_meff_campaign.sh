#!/usr/bin/env bash
# 1-2d Isaac 측정 묶음: 짝 2회 → 끔/켬 번갈아 2회씩 → 덤프 검증 1회 → 표시 headless 시험 1회
R=$HOME/effmass_mpc/goal1/scripts/g1_meff_run.sh; L=$HOME/effmass_mpc/goal1/logs/meff_isaac/campaign.txt; : > $L
run() { echo "=== $1 $(date +%T)" >> $L; $R "$@" >> $L 2>&1; ls /dev/shm | grep -q fastrtps && echo "!!! /dev/shm fastrtps 남음" >> $L; }
run g1d_pair_1 --meff on --meff-paired 1
run g1d_pair_2 --meff on --meff-paired 1
run g1d_off_1 --meff off
run g1d_on_1 --meff on
run g1d_off_2 --meff off
run g1d_on_2 --meff on
run g1d_dump --meff on --meff-dump-steps $(seq -s, 100 100 1100)
run g1d_viz --meff on --meff-viz both --meff-lines all --steps 300
echo "=== done $(date +%T)" >> $L

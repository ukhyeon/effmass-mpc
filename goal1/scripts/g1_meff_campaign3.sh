#!/usr/bin/env bash
# 1-2d 마무리 측정: 1안만 켬 짧게 1 → 최종(1안+2안) 짝 2 → 끔/켬 번갈아 2씩 → topk 선 표시 headless 시험 1
R=$HOME/effmass_mpc/goal1/scripts/g1_meff_run.sh; L=$HOME/effmass_mpc/goal1/logs/meff_isaac/campaign3.txt; : > $L
run() { echo "=== $1 $(date +%T) EFFMASS_THREADS1=${EFFMASS_THREADS1:-1}" >> $L; echo "--- 실행 전 CPU 상위 프로세스" >> $L; top -b -n 1 -o %CPU | sed -n 7,12p >> $L
        THREAD_SNAP=1 $R "$@" --loop-prof 1 >> $L 2>&1; ls /dev/shm | grep -q fastrtps && echo "!!! /dev/shm fastrtps 남음" >> $L; }
EFFMASS_THREADS1=0 run g1d_c1_on --meff on --steps 600
run g1d_f_pair_1 --meff on --meff-paired 1
run g1d_f_pair_2 --meff on --meff-paired 1
run g1d_f_off_1 --meff off
run g1d_f_on_1 --meff on
run g1d_f_off_2 --meff off
run g1d_f_on_2 --meff on
run g1d_f_viz --meff on --meff-viz both --meff-lines topk --meff-lines-k 5 --steps 300
echo "=== done $(date +%T)" >> $L

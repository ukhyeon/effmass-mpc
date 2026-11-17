#!/usr/bin/env bash
# 1-2d 계측 실행: 덤프·디버그 읽기·시각화 끔, --loop-prof 1, 스레드 스냅샷. 짝 1 → 끔 1 → 켬 1
R=$HOME/effmass_mpc/goal1/scripts/g1_meff_run.sh; L=$HOME/effmass_mpc/goal1/logs/meff_isaac/campaign2.txt; : > $L
run() { echo "=== $1 $(date +%T)" >> $L; echo "--- 실행 전 CPU 상위 프로세스" >> $L; top -b -n 1 -o %CPU | sed -n 7,14p >> $L; THREAD_SNAP=1 $R "$@" --loop-prof 1 >> $L 2>&1; ls /dev/shm | grep -q fastrtps && echo "!!! /dev/shm fastrtps 남음" >> $L; }
run g1d_prof_pair --meff on --meff-paired 1
run g1d_prof_off --meff off
run g1d_prof_on --meff on
echo "=== done $(date +%T)" >> $L; top -b -n 1 -o %CPU | sed -n 7,14p >> $L

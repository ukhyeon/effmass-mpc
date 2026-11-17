#!/usr/bin/env bash
# B1 0단계: off 배율 확인 (여유 시간 산정용, 성능 판정 아님). 같은 시간대에 연달아 1회씩
R=$HOME/effmass_mpc/goal1/scripts/g1_meff_run.sh; P=$HOME/effmass_mpc/goal1/scripts/g13_pre_run.sh; L=$HOME/effmass_mpc/goal1/logs/meff_isaac/b1_campaign_rtf.txt; : > $L
run() { local r=$1; shift; echo "=== $1 $(date +%T)" >> $L; top -b -n 1 -o %CPU | sed -n 7,10p >> $L; THREAD_SNAP=1 $r "$@" --loop-prof 1 >> $L 2>&1; }
run $R b1r_ros_chanoff --meff off
run $R b1r_ros_modeoff --meff on --meff-mode off
run $R b1r_ros_modeoff_near --meff on --meff-mode off --meff-log-near 0.3
run $P b1r_pre_modeoff --meff on --meff-mode off
run $P b1r_pre_chanoff --meff off
echo "=== done $(date +%T)" >> $L

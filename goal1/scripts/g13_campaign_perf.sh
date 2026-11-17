#!/usr/bin/env bash
# 1-3 A단계 성능 (지금 경로: 재생 → 사람 노드 → ROS → Isaac). 1-2d 최종과 같은 방식 (THREAD_SNAP, --loop-prof 1)
#   짝: 같은 측정 상태로 1-2d 채널(가중치 0) solver ↔ 1-3 비용 solver 번갈아. 따로: off / uniform 0.10 / mass 후보 A
R=$HOME/effmass_mpc/goal1/scripts/g1_meff_run.sh; L=$HOME/effmass_mpc/goal1/logs/meff_isaac/g13_campaign_perf.txt; : > $L
MA="--meff-mode mass --meff-eta0 0.10 --meff-muref 2.0 --meff-p 0.5 --meff-eta-min 0.05 --meff-eta-max 0.20"
run() { echo "=== $1 $(date +%T)" >> $L; echo "--- 실행 전 CPU 상위 프로세스" >> $L; top -b -n 1 -o %CPU | sed -n 7,12p >> $L
        THREAD_SNAP=1 $R "$@" --loop-prof 1 >> $L 2>&1; }
run g13_pair_maxall --meff on $MA --meff-agg max_all --meff-paired 1 --meff-paired-base channel_off
run g13_pair_nearest --meff on $MA --meff-agg nearest --meff-paired 1 --meff-paired-base channel_off
run g13_sep_off --meff on --meff-mode off
run g13_sep_uni --meff on --meff-mode uniform --meff-eta0 0.10
run g13_sep_mass --meff on $MA --meff-agg max_all
echo "=== done $(date +%T)" >> $L

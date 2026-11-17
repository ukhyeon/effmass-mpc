#!/usr/bin/env bash
# 1-3 B1 실행 (precomputed, 장면 g1, 2,523 스텝 = 재생 1,262 프레임 전부). 인자: "조건 seed [태그끝]" 여러 개. 조건: off uni10 unim massA
# unim 은 ETA_M 환경변수 (mass A seed 0 에서 정한 η_matched) 필요
R=$HOME/effmass_mpc/goal1/scripts/g13_pre_run.sh; L=$HOME/effmass_mpc/goal1/logs/meff_isaac/b1_campaign.txt; : >> $L
MA="--meff-mode mass --meff-eta0 0.10 --meff-muref 2.0 --meff-p 0.5 --meff-eta-min 0.05 --meff-eta-max 0.20 --meff-agg max_all --meff-log-near 0.3"
for spec in "$@"; do
  set -- $spec; c=$1; s=$2; tag=b1_${c}_s${s}${3:-}
  case $c in
    off) a="--meff-mode off" ;;
    uni10) a="--meff-mode uniform --meff-eta0 0.10 --meff-agg max_all" ;;
    unim) a="--meff-mode uniform --meff-eta0 ${ETA_M:?ETA_M} --meff-agg max_all" ;;
    massA) a="$MA" ;;
    iso) a="${MA/--meff-mode mass/--meff-mode iso}" ;;   # (B1 추가) ISO 15066 고정 m_h, 나머지 mass A 와 같음
  esac
  echo "=== $tag $(date +%T)" >> $L; top -b -n 1 -o %CPU | sed -n 7,9p >> $L
  $R $tag --meff on $a --mppi-seed $s --b1-log 1 --steps 2523 --human-viz none --u-def com >> $L 2>&1   # human-viz none + u-def com = same as B1
done
echo "=== batch done $(date +%T)" >> $L

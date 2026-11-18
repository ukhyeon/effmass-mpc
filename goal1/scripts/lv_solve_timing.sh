#!/usr/bin/env bash
# lv_solve_timing.sh <tag> <sensing 0|1> [extra step2b args] - overnight2 bundle 3(a): MPC solve time of the live3m real-time replay (per-part,
#   seed 0, live_t1, headless, b1 log off, cost aggregation off), optionally with the hrc_ws sensing node running on the real Kinect
#   (lv_run_sensing.sh, output to logs/meff_isaac/<tag>_sensing; its GPU load is the point, nobody needs to be in view).
set -u
TAG=${1:?tag}; SENS=${2:?0|1}; shift 2; GG=$HOME/effmass_mpc/goal1; O=$GG/logs/meff_isaac; G=$GG/b2a/g2; C=$GG/live/live3m/compare; S=$HOME/effmass_mpc/env_check/scripts
SP=""
if [[ $SENS == 1 ]]; then
  $S/lv_run_sensing.sh $O/${TAG}_sensing > $O/${TAG}_sensing.log 2>&1 & SP=$!
  for i in $(seq 1 120); do grep -q "camera pose\] roll" $O/${TAG}_sensing.log 2>/dev/null && break; kill -0 $SP 2>/dev/null || break; sleep 1; done
  grep -q "camera pose\] roll" $O/${TAG}_sensing.log || { echo "sensing did not start"; kill -INT $SP 2>/dev/null; exit 4; }
  echo "sensing up (pid $SP)"
fi
FR=$(python3 -c "import json;print(json.load(open('$C/runs.json'))['frame_file'])"); N=$(python3 -c "import json;print(json.load(open('$C/runs.json'))['steps'])")
$GG/scripts/lv_run.sh $TAG 600 --scene g2 --shapes v2 --human-dir $G --human-pre $C/human_rt_v2.npz --human-frame $FR --human-viz none --u-def nearest --meff on --meff-mode gain \
  --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 0 --cost-agg 0 --profile $GG/config/live_t1.yml --scene-json $G/scene_live_front.json \
  --round-trip 1 --meff-muref-mode part --meff-gain 0.10 --mppi-seed 0 --steps $N "$@" | head -3
if [[ -n $SP ]]; then kill -INT $SP 2>/dev/null; for i in $(seq 1 30); do kill -0 $SP 2>/dev/null || break; sleep 1; done; kill -0 $SP 2>/dev/null && kill $SP; echo "sensing stopped"; fi

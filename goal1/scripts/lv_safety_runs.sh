#!/usr/bin/env bash
# lv_safety_runs.sh - overnight2 bundle 4: safety layer off / on on two real-time replays (run3 first live test, live3m) and a simulated
#   tracking loss (live3m, human frame frozen for 3 s during motion 1). Same arguments as the earlier replays (pr_live_run3_part, cmp_live3m_part).
set -u
GG=$HOME/effmass_mpc/goal1; G=$GG/b2a/g2; D=$GG/data; C=$GG/live/live3m/compare; R=$GG/scripts/lv_run.sh
COMMON="--scene g2 --shapes v2 --human-dir $G --human-viz none --u-def nearest --meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --cost-agg 0 --profile $GG/config/live_t1.yml --scene-json $G/scene_live_front.json --round-trip 1 --meff-gain 0.10 --mppi-seed 0 --meff-muref-mode part"
RUN3="$COMMON --b1-log 0 --human-pre $D/human_live_run3_v2.npz --human-frame $GG/config/live_frame.yml --steps 4992"
FR=$(python3 -c "import json;print(json.load(open('$C/runs.json'))['frame_file'])"); N=$(python3 -c "import json;print(json.load(open('$C/runs.json'))['steps'])")
L3M="$COMMON --b1-log 1 --human-pre $C/human_rt_v2.npz --human-frame $FR --steps $N"
ON="--safety-near 1 --safety-loss-stop 1"; ON0="$ON --safety-allow-retreat 0"   # on = retreat allowed (default), on0 = hold whatever the direction
busy() { ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -E 'step2b_mpc_isaac|kit/python' | grep -vc ugrep; }
SPECS=("sf_run3_off|$RUN3" "sf_run3_on|$RUN3 $ON0" "sf_run3_on2|$RUN3 $ON" "sf_live3m_off|$L3M" "sf_live3m_on|$L3M $ON0" "sf_live3m_on2|$L3M $ON"
       "sf_loss_off|$L3M --human-gap-sim 24,27" "sf_loss_on|$L3M --human-gap-sim 24,27 --safety-loss-stop 1" "sf_loss_on2|$L3M --human-gap-sim 24,27 $ON")
for spec in "${SPECS[@]}"; do   # SAFETY_SET="tag tag ..." runs only those
  t=${spec%%|*}; a=${spec#*|}
  if [[ -n ${SAFETY_SET:-} && " $SAFETY_SET " != *" $t "* ]]; then continue; fi
  [[ $(busy) == 0 ]] || { echo "Isaac busy, stop"; exit 3; }
  $R $t 600 $a | head -1
done
echo "== done $(date +%T)"

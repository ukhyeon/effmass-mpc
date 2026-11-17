#!/usr/bin/env bash
# r1_campaign.sh "speed cond seed [suffix]" ... - R1 moving hand runs: front-cal scene (scene_b2a4_front_0.38h1.json), round trips (--round-trip 1),
# precomputed moving hand human_r1_reach_<speed>.npz (shapes v2), profile t1, u-def nearest, gain mode (gk000 = uniform k 0, gain = k 0.10), max_all,
# B1 per-step logging off, rollout cost aggregation off (both computed offline from the joint commands and the human frames). Steps: 3600 (60 s), v100 7200 (120 s).
# Stops the batch when a run has realtime factor < 1.0, infeasible > 0 or a graph capture outside warm-up (criterion B).
R=$HOME/effmass_mpc/goal1/scripts/g13_pre_run.sh; L=$HOME/effmass_mpc/goal1/logs/meff_isaac/r1_campaign.txt; : >> $L; G=$HOME/effmass_mpc/goal1/b2a/g2
for spec in "$@"; do
  set -- $spec; v=$1; c=$2; s=$3; tag=r1_${v}_${c}_s${s}${4:-}; n=${N:-3600}; [[ $v == v100 && -z "${N:-}" ]] && n=7200
  case $c in
    gk000) k=0.0 ;; gain) k=0.10 ;; *) echo bad cond; exit 2 ;;
  esac
  echo "=== $tag $(date +%T) steps $n" >> $L
  $R $tag --scene g2 --scene-json $G/scene_b2a4_front_0.38h1.json --round-trip 1 --shapes v2 --human-pre ~/effmass_mpc/goal1/data/human_r1_reach_${v}.npz --human-dir $G \
     --human-viz none --u-def nearest --meff on --meff-mode gain --meff-dbase 0.065 --meff-gain $k --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all \
     --mppi-seed $s --b1-log 0 --cost-agg 0 --steps $n --profile t1 ${R1_EXTRA:-} >> $L 2>&1
  P=$HOME/effmass_mpc/goal1/logs/meff_isaac/$tag.log; rtf=$(grep -a "realtime_factor" $P | tail -1 | awk '{print $NF}'); inf=$(grep -a "infeasible_steps" $P | tail -1 | awk '{print $NF}'); cap=$(grep -a "graph_captures_by_stage" $P | tail -1 | cut -d: -f2-)
  echo "--- $tag rtf $rtf infeasible $inf captures $cap" >> $L
  if [[ -z "$rtf" ]] || python3 -c "import sys; sys.exit(0 if ($rtf < 1.0 or $inf > 0) else 1)" || [[ "$cap" != *"{'warmup': 1}"* ]]; then echo "=== STOP criterion B at $tag" >> $L; exit 4; fi
done
echo "=== batch done $(date +%T)" >> $L

#!/usr/bin/env bash
# pr_record.sh - (part) 2x2 static video: front / hand-top scene x uniform / proposed (per-part reference), seed 0, zone as contour rings
# (cage), body mesh, COM hidden, TCP trail; same camera and style as v2_record.sh. Steps = arrival + 20. Joint commands are checked
# against the original runs (first N steps) before the frame folders are removed. Outputs in goal1/part_regen/.
set -u
source $HOME/effmass_mpc/goal1/scripts/on_lib.sh
PR=$HOME/effmass_mpc/goal1/part_regen; REC=$PR/rec; mkdir -p $REC; Zd=$HOME/effmass_mpc/goal1/zone; LR=$HOME/effmass_mpc/goal1/scripts/lv_run.sh
CAM="0.58,-1.22,0.62,0.58,0.0,0.47"
VB="--human-viz mesh --viz-hide-com on --viz-trail on --viz-zone-style cage --viz-cam $CAM --viz-record-res 960x540"
BASE="--scene g2 --shapes v2 --human-pre $D/human_b2a_reach.npz --human-dir $G --u-def nearest --meff on --meff-agg max_all --b1-log 1 --cost-agg 1 --profile t1 --single-pass 1 --mppi-seed 0"
UNI="--meff-mode uniform --meff-eta0 0.065"
PART="--meff-mode gain --meff-dbase 0.065 --meff-gain 0.10 --meff-muref 0.53 --meff-eta-max 0.25 --meff-muref-mode part"
rec() {  # name scenekey cond-args zone N
  $LR pr_rec_$1 900 $BASE --scene-json $G/scene_b2a4_$2.json $3 --steps $5 $VB --viz-zone $4 --viz-record $REC/$1 > /dev/null 2>&1; echo "pr_rec_$1 exit $?"
}
rec front_uniform front_0.38h1 "$UNI" $Zd/zone_uniform_small_mesh.npz 160
rec front_part front_0.38h1 "$PART" $PR/zone_part_mesh.npz 185
rec top_uniform top_0.455 "$UNI" $Zd/zone_uniform_small_mesh.npz 238
rec top_part top_0.455 "$PART" $PR/zone_part_mesh.npz 221
python3 - <<'PY'
import json, re, csv, numpy as np
O = "/home/robotics/effmass_mpc/goal1/logs/meff_isaac"; out = {}
def Q(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read(); r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))
    return np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])
for nm, orig in (("front_uniform", "v1_front_0.38h1_u065_s0"), ("front_part", "pr_v2_front_part_s0"), ("top_uniform", "v1_top_0.455_u065_s0"), ("top_part", "pr_v2_top_part_s0")):
    a = Q(f"pr_rec_{nm}"); b = Q(orig); out[nm] = dict(original=orig, n=len(a), bit_equal_first_n=bool(np.array_equal(a, b[:len(a)])))
json.dump(out, open("/home/robotics/effmass_mpc/goal1/part_regen/video_bitcheck.json", "w"), indent=1); print(json.dumps(out))
PY

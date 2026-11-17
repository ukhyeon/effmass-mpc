#!/usr/bin/env bash
# r1_record.sh - R1 demo recordings (display only: body mesh, COM hidden, trail, zone cages along the hand motion, V2 camera), first 1210 steps of the
# seed 0 runs at 0.25 and 1.0 m/s; joint commands checked against the original runs (first N steps).
cd $HOME/effmass_mpc/goal1/scripts; D=$HOME/effmass_mpc/goal1/b2a/demo/r1/rec; ZI=$HOME/effmass_mpc/goal1/zone/r1_seq/index.json; CAM="0.58,-1.22,0.62,0.58,0.0,0.47"
for spec in "v025 gk000 uniform r1_main" "v025 gain ours r1_main" "v100 gk000 uniform r1_fast" "v100 gain ours r1_fast"; do
  set -- $spec
  N=1210 R1_EXTRA="--human-viz mesh --viz-hide-com on --viz-trail on --viz-zone-seq $ZI --viz-zone-seq-cond $3 --viz-cam $CAM --viz-record-res 960x540 --viz-record $D/${4}_$2" ./r1_campaign.sh "$1 $2 0 _rec"
done
python3 - <<'PY'
import json, re, csv, numpy as np
O = "/home/robotics/effmass_mpc/goal1/logs/meff_isaac"; out = {}
def Q(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read(); r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))
    return np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])
for v in ("v025", "v100"):
    for c in ("gk000", "gain"):
        a = Q(f"r1_{v}_{c}_s0_rec"); b = Q(f"r1_{v}_{c}_s0"); out[f"{v}/{c}"] = dict(bit_equal_first_n=bool(np.array_equal(a, b[:len(a)])), n=len(a))
json.dump(out, open(f"{O}/r1_video_bitcheck.json", "w"), indent=1); print(json.dumps(out))
PY

#!/usr/bin/env bash
# demo_record_v2.sh - demo v2 recordings: seed 0 runs of B2a-5 (A front-cal, B top-cal) and g2-block (C), off / uniform_matched / iso / mass A,
# display only (eta shells, trail, body mesh, COM markers hidden, two cameras). Steps cut to (latest arrival in the scene + 20); joint commands are
# checked against the first N steps of the original runs.
cd $HOME/effmass_mpc/goal1/scripts; D=$HOME/effmass_mpc/goal1/b2a/demo/rec_v2; export ETA_M=0.0861
CAMS="0.12,-1.10,1.08,0.46,0.0,0.47;0.53,-0.62,0.48,0.53,0.0,0.47"
for spec in "front_0.38h1 175" "top_0.455 300" "block_0.47hc 235"; do
  set -- $spec
  for c in off unim iso massA; do
    N=$2 B2A_EXTRA="--human-viz mesh --viz-hide-com on --viz-eta-shell on --viz-trail on --viz-cam $CAMS --viz-record $D/${1}_${c}_s0 --viz-record-res 960x540" ./b2a5_campaign.sh "$1 $c 0 _demo2"
  done
done
python3 - <<'PY'
import json, re, csv, numpy as np
O = "/home/robotics/effmass_mpc/goal1/logs/meff_isaac"; out = {}
def Q(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read(); r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))
    return np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])
for k in ("front_0.38h1", "top_0.455", "block_0.47hc"):
    for c in ("off", "unim", "iso", "massA"):
        a = Q(f"b2a5_{k}_{c}_s0_demo2"); b = Q(f"b2a5_{k}_{c}_s0"); n = len(a); out[f"{k}/{c}"] = dict(bit_equal_first_n=bool(np.array_equal(a, b[:n])), n=n, orig=len(b))
json.dump(out, open(f"{O}/demo_v2_bitcheck.json", "w"), indent=1); print(json.dumps(out))
PY

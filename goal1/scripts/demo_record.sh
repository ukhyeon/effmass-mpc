#!/usr/bin/env bash
# demo_record.sh - final demo recordings: the B2a-5 seed 0 runs again with display options only (eta shells, trail, body mesh, two cameras),
# headless frames through --viz-record. Same MPC options as the originals (b2a5_campaign.sh); joint commands checked against the original runs.
cd $HOME/effmass_mpc/goal1/scripts; D=$HOME/effmass_mpc/goal1/b2a/demo/rec
CAMS="0.12,-1.10,1.08,0.46,0.0,0.47;0.53,-0.62,0.48,0.53,0.0,0.47"
for spec in "front_0.38h1 iso 304" "front_0.38h1 massA 304" "top_0.455 iso 506" "top_0.455 massA 506" "block_0.47hc off 338" "block_0.47hc iso 338" "block_0.47hc massA 338"; do
  set -- $spec
  N=$3 B2A_EXTRA="--human-viz mesh --viz-eta-shell on --viz-trail on --viz-cam $CAMS --viz-record $D/${1}_${2}_s0 --viz-record-res 960x540" ./b2a5_campaign.sh "$1 $2 0 _demo"
done
python3 - <<'PY'
import json, re, csv, numpy as np
O = "/home/robotics/effmass_mpc/goal1/logs/meff_isaac"; out = {}
def Q(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read(); r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))
    return np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])
for k, c in (("front_0.38h1", "iso"), ("front_0.38h1", "massA"), ("top_0.455", "iso"), ("top_0.455", "massA"), ("block_0.47hc", "off"), ("block_0.47hc", "iso"), ("block_0.47hc", "massA")):
    a = Q(f"b2a5_{k}_{c}_s0_demo"); b = Q(f"b2a5_{k}_{c}_s0"); out[f"{k}/{c}"] = dict(bit_equal=bool(np.array_equal(a, b)), steps=[len(a), len(b)])
json.dump(out, open(f"{O}/demo_final_bitcheck.json", "w"), indent=1); print(json.dumps(out))
PY

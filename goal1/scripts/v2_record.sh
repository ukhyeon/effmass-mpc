#!/usr/bin/env bash
# v2_record.sh - V2 demo recordings (display only: body mesh, COM hidden, zone surface of the condition, trail, ghost of the other condition),
# one zoomed side camera; steps = arrival + 20; joint commands checked against the original v1_ runs (first N steps).
cd $HOME/effmass_mpc/goal1/scripts; D=$HOME/effmass_mpc/goal1/b2a/demo/v2/rec; Zd=$HOME/effmass_mpc/goal1/zone; CAM="0.58,-1.22,0.62,0.58,0.0,0.47"
VB="--human-viz mesh --viz-hide-com on --viz-trail on --viz-zone-style cage --viz-cam $CAM --viz-record-res 960x540"
rec() {  # name scenekey cond N zone ghostcond
  local g=""; [[ -n "$6" ]] && g="--viz-ghost $6 --viz-ghost-tag v1_${2}_{cond}_s0"
  N=$4 B2A_EXTRA="$VB --viz-zone $Zd/$5 $g --viz-record $D/$1" ./v1_campaign.sh "$2 $3 0 _$1"
}
rec rec_front_u065 front_0.38h1 u065 160 zone_uniform_small_mesh.npz gain
rec rec_front_gain front_0.38h1 gain 150 zone_ours_mesh.npz u065
rec rec_top_u065 top_0.455 u065 238 zone_uniform_small_mesh.npz gain
rec rec_top_gain top_0.455 gain 320 zone_ours_mesh.npz u065
rec rec_knob_gk000 front_0.38h1 gk000 160 zone_uniform_small_mesh.npz ""
rec rec_knob_gk005 front_0.38h1 gk005 168 zone_gk005_mesh.npz ""
rec rec_knob_gain front_0.38h1 gain 150 zone_ours_mesh.npz ""
rec rec_knob_gk015 front_0.38h1 gk015 164 zone_gk015_mesh.npz ""
python3 - <<'PY'
import json, re, csv, numpy as np
O = "/home/robotics/effmass_mpc/goal1/logs/meff_isaac"; out = {}
def Q(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read(); r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))
    return np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])
for nm, k, c in (("rec_front_u065", "front_0.38h1", "u065"), ("rec_front_gain", "front_0.38h1", "gain"), ("rec_top_u065", "top_0.455", "u065"), ("rec_top_gain", "top_0.455", "gain"),
                 ("rec_knob_gk000", "front_0.38h1", "gk000"), ("rec_knob_gk005", "front_0.38h1", "gk005"), ("rec_knob_gain", "front_0.38h1", "gain"), ("rec_knob_gk015", "front_0.38h1", "gk015")):
    a = Q(f"v1_{k}_{c}_s0_{nm}"); b = Q(f"v1_{k}_{c}_s0"); out[nm] = dict(bit_equal_first_n=bool(np.array_equal(a, b[:len(a)])), n=len(a))
json.dump(out, open(f"{O}/v2_video_bitcheck.json", "w"), indent=1); print(json.dumps(out))
PY

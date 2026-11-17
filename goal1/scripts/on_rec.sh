#!/usr/bin/env bash
# on_rec.sh [vid ...] - WP7 display recordings (body mesh, COM hidden, trail, zone cages, V2 camera, 960x540), seed 0, same run arguments as the original
# runs plus display options; joint commands checked bit by bit against the original runs (first N steps) -> logs/meff_isaac/on_wp7_video_bitcheck.json.
source $HOME/effmass_mpc/goal1/scripts/on_lib.sh
REC=$HOME/effmass_mpc/goal1/b2a/demo/overnight/rec; Z=$HOME/effmass_mpc/goal1/zone/overnight; CAM="0.58,-1.22,0.62,0.58,0.0,0.47"; mkdir -p $REC
V="--human-viz mesh --viz-hide-com on --viz-trail on --viz-cam $CAM --viz-record-res 960x540"
for vid in ${@:-on_static_P4_tipfront on_M1_handover on_M2_crossing}; do
  for c in gk000 gain; do
    cn=$([[ $c == gk000 ]] && echo uniform || echo ours)
    case $vid in
      on_static_P4_tipfront) $RUN on3_P4_tipfront_${c}_s0_rec 600 $COMMON --scene-json $G/scene_b2a4_front_0.38h1.json --single-pass 1 --human-pre $D/human_on3_P4_tipfront_v2.npz \
           --meff-gain $(kgain $c) --mppi-seed 0 --steps 420 $V --viz-zone $Z/P4_tipfront/${cn}_static.npz --viz-record $REC/${vid}_$c ;;
      on_M1_handover) $RUN on4_M1_v025_${c}_s0_rec 1200 $COMMON --scene-json $G/scene_b2a4_front_0.38h1.json --round-trip 1 --human-pre $D/human_on4_M1_v025_v2.npz \
           --meff-gain $(kgain $c) --mppi-seed 0 --steps 1210 $V --viz-zone-seq $Z/M1_v025/index.json --viz-zone-seq-cond $cn --viz-record $REC/${vid}_$c ;;
      on_M2_crossing) $RUN on4_M2_v025_${c}_s0_rec 1200 $COMMON --scene-json $G/scene_b2a4_front_0.38h1.json --round-trip 1 --human-pre $D/human_on4_M2_v025_v2.npz \
           --meff-gain $(kgain $c) --mppi-seed 0 --steps 1210 $V --viz-zone-seq $Z/M2_v025/index.json --viz-zone-seq-cond $cn --viz-record $REC/${vid}_$c ;;
    esac
  done
done
python3 - <<'PY'
import json, os, sys
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/goal1/scripts")); import on_bitcmp as B
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac"); out = {}
for a, b in (("on3_P4_tipfront_{c}_s0_rec", "on3_P4_tipfront_{c}_s0"), ("on4_M1_v025_{c}_s0_rec", "on4_M1_v025_{c}_s0"), ("on4_M2_v025_{c}_s0_rec", "on4_M2_v025_{c}_s0")):
    for c in ("gk000", "gain"):
        if os.path.exists(f"{O}/{a.format(c=c)}.log"):
            try:
                out[a.format(c=c)] = B.cmp(a.format(c=c), b.format(c=c))
            except Exception as e:
                out[a.format(c=c)] = dict(error=str(e))
json.dump(out, open(f"{O}/on_wp7_video_bitcheck.json", "w"), indent=1); print(json.dumps(out))
PY

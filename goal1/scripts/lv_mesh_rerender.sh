#!/usr/bin/env bash
# lv_mesh_rerender.sh <tag> - overnight2: re-render the seed 0 runs of compare/ (arm side, whole recording) and compare_tip/ (front pass,
#   per motion) from their recorded joints (--playback-q), with the translucent SMPL-H body mesh (opacity 0.35, display only; vertex
#   files from lv_mesh_verts.py), COM markers hidden, zone shells on forearms, hands, elbows, camera = the live run's VIEW; every 2nd step
#   saved (640x360 jpg) to goal1/live/<tag>/mesh_frames/<run>. Physics / MPC run as in the replays but the robot is set to the recorded
#   joints each step. One Isaac at a time, <= 600 s per run.
set -u
TAG=${1:?tag}; GG=$HOME/effmass_mpc/goal1; O=$GG/logs/meff_isaac; G=$GG/b2a/g2; SC=$GG/scripts; C=$GG/live/$TAG/compare; T=$GG/live/$TAG/compare_tip; M=$GG/live/$TAG/mesh_frames
mkdir -p $M
VIEW=$(python3 -c "import json;print(json.load(open('$C/runs.json'))['view'])")
qnpz() { python3 - "$1" "$2" <<'PY'
import sys, json, re, csv, numpy as np
O = "/home/robotics/effmass_mpc/goal1/logs/meff_isaac"; t, out = sys.argv[1], sys.argv[2]
r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", open(f"{O}/{t}.log", errors="replace").read())[-1]))
np.savez(out, q=np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))]))
PY
}
one() {  # src human frame scene steps gain
  local src=$1 hu=$2 fr=$3 sc=$4 n=$5 k=$6
  if [[ -d $M/$src/cam0 && $(ls $M/$src/cam0 | wc -l) -ge $(( n / 2 - 5 )) ]]; then echo "$src reused"; return; fi
  rm -rf $M/$src; qnpz $src $M/${src}_q.npz
  n_=$(ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|kit/python')
  [[ $n_ == 0 ]] || { echo "Isaac busy"; exit 3; }
  G1_MESH_OPACITY=0.35 G1_VIZ_REC_EXT=jpg $SC/lv_run.sh mesh_$src 600 --scene g2 --shapes v2 --human-dir $G --human-pre $hu --human-frame $fr --human-viz mesh \
    --viz-hide-com on --u-def nearest --meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 0 --cost-agg 0 \
    --profile $GG/config/live_t1.yml --scene-json $sc --round-trip 1 --meff-muref-mode part --meff-gain $k --mppi-seed 0 --steps $n \
    --playback-q $M/${src}_q.npz --viz-eta-shell armelbow --viz-cam=$VIEW --viz-record $M/$src --viz-record-res 640x360 --viz-record-every 2 | head -1
  echo "$src frames $(ls $M/$src/cam0 2>/dev/null | wc -l)"
}
python3 -c "
import json; S=json.load(open('$T/scenes.json'))['motions']
for q,m in enumerate(S): print(q+1, m['human'], m['frame_file'], m['scene'], m['steps'])" > $M/tip_list.txt
while read q hu fr sc n; do
  one tip_${TAG}_m${q}_uni_s0 $hu $fr $sc $n 0; one tip_${TAG}_m${q}_part_s0 $hu $fr $sc $n 0.10
done < $M/tip_list.txt
CF=$(python3 -c "import json;print(json.load(open('$C/runs.json'))['frame_file'])"); CN=$(python3 -c "import json;print(json.load(open('$C/runs.json'))['steps'])")
one cmp_${TAG}_uni_s0 $C/human_rt_v2.npz $CF $G/scene_live_front.json $CN 0
one cmp_${TAG}_part_s0 $C/human_rt_v2.npz $CF $G/scene_live_front.json $CN 0.10

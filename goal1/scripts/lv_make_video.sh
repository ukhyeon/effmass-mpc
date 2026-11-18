#!/usr/bin/env bash
# lv_make_video.sh <tag> - live prep2: side-by-side video of a live run (lv_live_kinect.sh <tag>): Kinect colour | Isaac re-render.
#   1) lv_video_prep.py: from the run's records only (sensing poses, Isaac's pump record, steps csv) -> human file (one frame per
#      sensing message), per-step human frame, per-step measured joints, per-step wall time (goal1/live/<tag>/video_inputs)
#   2) Isaac headless re-render (lv_run.sh, tag <tag>_vid): robot set to the recorded joints of every step (--playback-q), human =
#      the message Isaac had at that step (--human-step-frames), same frame file, scene, mu_ref mode and gain as the run, zone shells
#      on the forearm and hand shapes (--viz-eta-shell arm), camera = the run's VIEW (--viz-cam from its log); every N-th step
#      saved, N = the most steps per 1/30 s of the run's wall time (at least 1). Isaac is not recorded during the live run itself.
#   3) lv_video_compose.py: colour frames (goal1/live/<tag>/color, capture time) and Isaac frames on the wall-time axis, 30 fps,
#      -> goal1/live/<tag>/<tag>_side_by_side.mp4 and video_sync.json (time difference of the two panels).
# Env: VID_TIMEOUT (s, default 1800), VID_RES (default 960x540), VID_KEEP_FRAMES=1 keeps isaac_frames/ (JPEG) after the mp4 is made.
set -u
TAG=${1:?tag}; GG=$HOME/effmass_mpc/goal1; O=$GG/logs/meff_isaac; G=$GG/b2a/g2; L=$GG/live/$TAG; SC=$GG/scripts
[[ -f $O/$TAG.log ]] || { echo "no live run log $O/$TAG.log"; exit 2; }
n=$(ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|kit/python')
[[ $n == 0 ]] || { echo "an Isaac process is running (one Isaac at a time); stop it first"; exit 3; }
mkdir -p $L
echo "[video] 1/3 inputs from the run records"
$SC/g1_run_human.sh py $SC/lv_video_prep.py $TAG 2>$L/video_prep_err.txt | tail -1 > $L/video_prep.json
[[ -s $L/video_prep.json ]] || { echo "lv_video_prep.py failed: $L/video_prep_err.txt"; exit 4; }
eval "$(python3 -c "
import json; d=json.load(open('$L/video_prep.json'))
print(f\"HUMAN='{d['human']}'; FRAME='{d['frame_file']}'; MM='{d['muref_mode']}'; KG='{d['k_gain'] or 0.10}'; N={d['steps']}; VIEW='{d['view']}'; RE={d['record_every']}\")")"
[[ -n $HUMAN ]] || { echo "no sensing poses saved for $TAG (no human file)"; exit 5; }
[[ $MM == part || $MM == global ]] || MM=part
echo "[video] steps $N, frame $(basename $FRAME), mu_ref $MM, gain $KG, view $VIEW, Isaac frame every $RE step(s)"
echo "[video] 2/3 Isaac re-render (headless, about 1-2 min per 1000 steps)"
rm -rf $L/isaac_frames
G1_VIZ_REC_EXT=jpg $SC/lv_run.sh ${TAG}_vid ${VID_TIMEOUT:-1800} --scene g2 --shapes v2 --human-dir $G --human-pre $HUMAN --human-frame $FRAME --human-viz shapes \
  --u-def nearest --meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 0 --cost-agg 0 \
  --profile $GG/config/live_t1.yml --scene-json $G/scene_live_front.json --round-trip 1 --meff-gain $KG --meff-muref-mode $MM --mppi-seed 0 \
  --steps $N --playback-q $L/video_inputs/playback_q.npz --human-step-frames $L/video_inputs/step_frames.npy --viz-eta-shell arm \
  --viz-cam="$VIEW" --viz-record $L/isaac_frames --viz-record-res ${VID_RES:-960x540} --viz-record-every $RE | head -1
nf=$(ls $L/isaac_frames/cam0 2>/dev/null | wc -l); echo "[video] Isaac frames $nf"
[[ $nf -gt 0 ]] || { echo "re-render made no frames: $O/${TAG}_vid.log"; exit 6; }
echo "[video] 3/3 side-by-side mp4"
python3 $SC/lv_video_compose.py $TAG | tail -1
if [[ -s $L/${TAG}_side_by_side.mp4 && ${VID_KEEP_FRAMES:-0} != 1 ]]; then rm -rf $L/isaac_frames; fi
echo "[video] $L/${TAG}_side_by_side.mp4"

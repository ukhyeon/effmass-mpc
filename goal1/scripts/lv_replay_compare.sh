#!/usr/bin/env bash
# lv_replay_compare.sh <tag> - live cmp: one live recording (lv_live_kinect.sh <tag>) replayed headless at real time (human time =
#   simulated time) with the live settings, uniform zone (k 0) and per-part reference (k 0.10), seeds 0 1 2; analysis per motion,
#   two figures, a three-panel video (colour | uniform | per-part, seed 0) with clips, REPORT.md. All in goal1/live/<tag>/compare/.
#   1) pr_live_prep.py: sensing poses of the run -> compare/human_rt_v2.npz (30 Hz, message stamps as time, as the node computes)
#   2) 6 runs lv_run.sh cmp_<tag>_{uni,part}_s{0,1,2} (live_t1.yml, scene_live_front.json, frame file of the run, b1 log on);
#      seed 0 also saves every 2nd step from the run's VIEW with zone shells on forearms, hands and elbows (display only)
#   3) lv_cmp_analyze.py (motions, table), lv_cmp_video.py (mp4, clips), lv_cmp_report.py (figures, REPORT.md)
# Env: CMP_RERUN=1 reruns runs that already finished (default: reuse), CMP_SEG_D / CMP_SEG_T motion rule (0.25 m, 3 s),
#      CMP_TIMEOUT per run (s, default 600), CMP_KEEP_FRAMES=1 keeps the saved Isaac frames.
set -u
TAG=${1:?tag}; GG=$HOME/effmass_mpc/goal1; O=$GG/logs/meff_isaac; G=$GG/b2a/g2; SC=$GG/scripts; C=$GG/live/$TAG/compare
[[ -f $O/$TAG.log && -f $O/${TAG}_human_inertia_rec.npz ]] || { echo "no live run records for $TAG in $O"; exit 2; }
n=$(ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|kit/python')
[[ $n == 0 ]] || { echo "an Isaac process is running (one Isaac at a time); stop it first"; exit 3; }
mkdir -p $C
echo "[cmp] 1/4 human file at real time ($(date +%T))"
$SC/g1_run_human.sh py $SC/pr_live_prep.py $TAG $C/human_rt_v2.npz 2>$C/prep_err.txt | tail -1 > $C/prep.json
[[ -s $C/prep.json ]] || { echo "pr_live_prep.py failed: $C/prep_err.txt"; exit 4; }
eval "$(python3 - $TAG $C <<'PY'
import json, re, sys
tag, C = sys.argv[1], sys.argv[2]; O = C.split("/live/")[0] + "/logs/meff_isaac"
p = json.load(open(f"{C}/prep.json")); log = open(f"{O}/{tag}.log", errors="replace").read()
vc = re.findall(r"'--viz-cam=([-\d.,]+)'", log) or ["1.85,0.25,1.0,0.3,-0.05,0.3"]
n = 2 * int(p["frames_30hz"]) - 1
runs = {"uniform": [f"cmp_{tag}_uni_s{s}" for s in range(3)], "part": [f"cmp_{tag}_part_s{s}" for s in range(3)]}
json.dump(dict(tag=tag, human=p["out"], frame_file=p["frame_file"], scene=C.split("/live/")[0] + "/b2a/g2/scene_live_front.json", view=vc[-1], steps=n,
               duration_s=p["duration_s"], runs=runs), open(f"{C}/runs.json", "w"), indent=1)
print(f"FRAME='{p['frame_file']}'; VIEW='{vc[-1]}'; N={n}")
PY
)"
echo "[cmp] human $(python3 -c "import json;d=json.load(open('$C/prep.json'));print(round(d['duration_s'],1),'s,',d['messages_used'],'messages')"), steps $N, frame $(basename $FRAME), view $VIEW"
BASE="--scene g2 --shapes v2 --human-dir $G --human-pre $C/human_rt_v2.npz --human-frame $FRAME --human-viz shapes --u-def nearest --meff on --meff-mode gain
 --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --b1-log 1 --cost-agg 0 --profile $GG/config/live_t1.yml
 --scene-json $G/scene_live_front.json --round-trip 1 --meff-muref-mode part --steps $N"
echo "[cmp] 2/4 six replays (headless; seed 0 runs also save frames)"
for mode in uni part; do
  if [[ $mode == uni ]]; then K=0; else K=0.10; fi
  for s in 0 1 2; do
    T=cmp_${TAG}_${mode}_s$s
    if [[ ${CMP_RERUN:-0} != 1 && -f $O/$T.log ]] && grep -aq "saved .*_result_" $O/$T.log && [[ -f $O/${T}_b1.npz ]] && { [[ $s != 0 ]] || [[ -d $C/frames_${mode/uni/uniform} ]]; }; then
      echo "$T reused"; continue; fi
    REC=""; if [[ $s == 0 ]]; then M=${mode/uni/uniform}; rm -rf $C/frames_$M; REC="--viz-eta-shell armelbow --viz-cam=$VIEW --viz-record $C/frames_$M --viz-record-res 640x360 --viz-record-every 2"; fi
    G1_VIZ_REC_EXT=jpg $SC/lv_run.sh $T ${CMP_TIMEOUT:-600} $BASE --meff-gain $K --mppi-seed $s $REC | head -2 | tr '\n' ' '; echo
    grep -aq "saved .*_result_" $O/$T.log || { echo "$T did not finish (see $O/${T}_wrap.txt)"; exit 5; }
  done
done
echo "[cmp] 3/4 analysis ($(date +%T))"
$SC/g1_run_human.sh py $SC/lv_cmp_analyze.py $TAG 2>$C/analyze_err.txt | tail -1 || { echo "analysis failed: $C/analyze_err.txt"; exit 6; }
echo "[cmp] 4/4 video, figures, report ($(date +%T))"
python3 $SC/lv_cmp_video.py $TAG | tail -1 | cut -c1-400
python3 $SC/lv_cmp_report.py $TAG
if [[ -s $C/${TAG}_compare.mp4 && ${CMP_KEEP_FRAMES:-0} != 1 ]]; then rm -rf $C/frames_uniform $C/frames_part; fi
echo "[cmp] done: $C/REPORT.md ($(date +%T))"

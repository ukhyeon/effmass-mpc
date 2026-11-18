"""lv_tip_video.py <tag> - live cmp tip: one three-panel clip per motion (colour | uniform | per-part, seed 0) from the frames the
seed 0 runs saved (compare_tip/frames_<run>), via lv_clip3.py. The colour time of step 0 = the slice start frame0 / 30 of the
replay file. Output compare_tip/clip_tip_<motion>.mp4 (+ .json)."""
import json, os, subprocess, sys
GG = os.path.expanduser("~/effmass_mpc/goal1")
tag = sys.argv[1]; T = f"{GG}/live/{tag}/compare_tip"; R = json.load(open(f"{T}/runs.json")); S = {m["name"]: m for m in json.load(open(f"{T}/scenes.json"))["motions"]}
for mot, modes in R.items():
    m = S[mot]; nm = mot.replace(" ", "")
    spec = dict(tag=tag, out=f"{T}/clip_tip_{nm}.mp4", dir_uniform=f"{T}/frames_{modes['uniform'][0]}", dir_part=f"{T}/frames_{modes['part'][0]}", steps=m["steps"],
                tau0_s=m["frame0"] / 30.0, title=f"{tag} {mot}: robot path just in front of the {m['target']} (replay at real time, re-rendered from records)",
                labels=["Kinect colour (recorded)", "uniform zone (k 0), seed 0", "per-part reference (k 0.10), seed 0"])
    json.dump(spec, open(f"{T}/clip_tip_{nm}.spec.json", "w"))
    print(subprocess.run(["python3", f"{GG}/scripts/lv_clip3.py", f"{T}/clip_tip_{nm}.spec.json"], capture_output=True, text=True).stdout.strip()[:300])

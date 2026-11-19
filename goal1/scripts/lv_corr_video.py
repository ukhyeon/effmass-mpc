"""lv_corr_video.py <tag> m1=<ver> m2=<ver> m3=<ver> - live3m corrected paths: one three-panel clip per motion (Kinect colour | uniform |
per-part reference; seed 0, start at A) from the mesh re-render frames of lv_corr_compare.sh mesh (cal/m<q><ver>/frames_<run>), via
lv_clip3.py. The colour time of step 0 = the slice start frame0 / 30 of the replay file. Output corrected/clip_corr_motion<q>.mp4 and
.gif (+ .json, .spec.json)."""
import json, os, subprocess, sys
GG = os.path.expanduser("~/effmass_mpc/goal1"); tag = sys.argv[1]; C = f"{GG}/live/{tag}/corrected"
for a in sys.argv[2:]:
    q, ver = a.split("=")[0][1:], a.split("=")[1]; T = f"{C}/cal/m{q}{ver}"; P = json.load(open(f"{T}/path.json")); src = f"corr_{tag}_m{q}{ver}_AB_{{}}_s0"
    what = "in front of the fingertip" if P["target"] == "fingertip" else "in front of the elbow"
    spec = dict(tag=tag, out=f"{C}/clip_corr_motion{q}.mp4", gif=f"{C}/clip_corr_motion{q}.gif", dir_uniform=f"{T}/frames_{src.format('uni')}", dir_part=f"{T}/frames_{src.format('part')}", steps=P["steps"],
                tau0_s=P["frame0"] / 30.0, title=f"{tag} motion {q}: corrected path {what} (held posture, replay at real time, re-rendered from records)",
                labels=["Kinect colour (recorded)", "uniform zone (k 0), seed 0", "per-part reference (k 0.10), seed 0"])
    json.dump(spec, open(f"{C}/clip_corr_motion{q}.spec.json", "w"))
    print(subprocess.run(["python3", f"{GG}/scripts/lv_clip3.py", f"{C}/clip_corr_motion{q}.spec.json"], capture_output=True, text=True).stdout.strip()[:400])

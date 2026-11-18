"""lv_cmp_video.py <tag> - live cmp: three-panel video (Kinect colour | uniform | per-part reference, seed 0) and per-motion clips.

Time axis = replay time tau (human time = simulated time). Isaac panels: the saved frame of step i = round(tau * 60) - 1 (frames
exist for every 2nd step, nearest used). Colour panel: the recorded colour frame whose sensing publish time (sensing_perf csv
t_ros_sec of that frame index, the same clock as the HumanPose header stamp the replay human is built on) is nearest to
t0 + tau, t0 = stamp of the first message used for the replay file. Report: |colour time - (t0 + tau)| and Isaac step offset.
Clips: per motion, 10 s around the moment of the smallest per-part arm gap (seed 0), inside the recording.
Output: compare/<tag>_compare.mp4 (1920x360), compare/clip_motion<N>.mp4, compare/video_sync.json.
"""
import csv
import glob
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw

GG = os.path.expanduser("~/effmass_mpc/goal1"); O = f"{GG}/logs/meff_isaac"
W, H = 640, 360


def frames(d):
    fs = glob.glob(f"{d}/cam0/f*.jpg") or glob.glob(f"{d}/cam0/f*.png")
    idx = np.array(sorted(int(os.path.basename(f)[1:6]) for f in fs)); ext = os.path.splitext(fs[0])[1] if fs else ".jpg"
    return idx, ext


def near(a, t):
    k = int(np.searchsorted(a, t)); k = min(max(k, 0), len(a) - 1)
    return k - 1 if k > 0 and abs(a[k - 1] - t) <= abs(a[k] - t) else k


def main(tag):
    C = f"{GG}/live/{tag}/compare"; cfg = json.load(open(f"{C}/runs.json")); an = json.load(open(f"{C}/analysis.json"))
    upd = np.load(f"{O}/{tag}_human_inertia_rec.npz")["upd"]; t0 = float(upd[np.argmin(upd[:, 3]), 1]) / 1e9
    if "t0_stamp" in cfg:
        t0 = float(cfg["t0_stamp"])
    perf = {}
    for f in glob.glob(f"{O}/{tag}_sensing/sensing_perf/*.csv"):
        for r in csv.DictReader(open(f)):
            perf[int(r["frame_index"])] = float(r["t_ros_sec"])
    cd = f"{GG}/live/{tag}/color"; ct, cf = [], []
    if os.path.exists(f"{cd}/vis_times.csv"):
        rows = [r for r in csv.DictReader(open(f"{cd}/vis_times.csv")) if os.path.exists(f"{cd}/vis_output/{r['file']}")]
        cap = np.array([float(r["wall_time"]) for r in rows]); ix = [int(r["file"][:-4]) for r in rows]
        off = np.median([perf[i] - c for i, c in zip(ix, cap) if i in perf]) if perf else 0.0
        ct = np.array([perf.get(i, c + off) for i, c in zip(ix, cap)]); cf = [f"{cd}/vis_output/{r['file']}" for r in rows]
        o = np.argsort(ct); ct = ct[o]; cf = [cf[k] for k in o]
    mu, mp = cfg["runs"]["uniform"][0], cfg["runs"]["part"][0]
    di = {m: f"{C}/frames_{m}" for m in ("uniform", "part")}; fi = {m: frames(di[m]) for m in di}
    n = an["steps"]; T = np.arange(0, n / 60.0, 1 / 30.0)
    out = f"{C}/{tag}_compare.mp4"
    ff = subprocess.Popen(["nice", "-n", "19", "ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{3*W}x{H}", "-r", "30", "-i", "-",
                           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", "-preset", "medium", "-threads", "2", out], stdin=subprocess.PIPE)
    mot = [(e["name"], e["t0_s"], e["t1_s"]) for e in an["intervals"] if e["t0_s"] is not None]
    dc, di_ = [], []; blank = Image.new("RGB", (W, H), (25, 25, 25)); ImageDraw.Draw(blank).text((20, H // 2), "no colour frame", fill=(220, 220, 220))
    for tau in T:
        i = max(int(round(tau * 60)) - 1, 0); pan = []
        if len(ct):
            kc = near(ct, t0 + tau); dc.append(abs(ct[kc] - (t0 + tau))); pan.append(Image.open(cf[kc]).convert("RGB").resize((W, H)))
        else:
            pan.append(blank)
        for m in ("uniform", "part"):
            idx, ext = fi[m]; k = idx[near(idx, i)]; di_.append(abs(int(k) - i))
            pan.append(Image.open(f"{di[m]}/cam0/f{k:05d}{ext}").convert("RGB").resize((W, H)))
        fr = Image.new("RGB", (3 * W, H))
        for q, p in enumerate(pan):
            fr.paste(p, (q * W, 0))
        dr = ImageDraw.Draw(fr); lab = [x for x, a, b in mot if a <= tau < b]
        dr.rectangle([0, 0, 3 * W, 18], fill=(0, 0, 0))
        dr.text((8, 3), f"Kinect colour   t {tau:6.2f} s   {lab[0] if lab else 'arms down'}", fill=(255, 255, 255))
        dr.text((W + 8, 3), "uniform zone (k 0, eta 6.5 cm), seed 0", fill=(255, 255, 255)); dr.text((2 * W + 8, 3), "per-part reference (k 0.10), seed 0", fill=(255, 255, 255))
        ff.stdin.write(fr.tobytes())
    ff.stdin.close(); rc = ff.wait()
    clips = []
    for e in an["intervals"]:
        if e["t0_s"] is None or "part" not in e["per_mode"]:
            continue
        tm = e["per_mode"]["part"]["seeds"][0]["t_min_s"]; a = max(0.0, min(tm - 5.0, n / 60.0 - 10.0)); nm = e["name"].replace(" ", "")
        cp = f"{C}/clip_{nm}.mp4"
        r = subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{a:.2f}", "-i", out, "-t", "10", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", cp])
        clips.append(dict(name=e["name"], file=os.path.basename(cp), start_s=a, ok=r.returncode == 0))
    rep = dict(out=out, ffmpeg_exit=rc, duration_s=len(T) / 30.0, colour_frames=len(cf), t0_stamp=t0,
               colour_time_offset_s=dict(median=float(np.median(dc)), p95=float(np.percentile(dc, 95)), max=float(max(dc))) if dc else None,
               isaac_step_offset_max=int(max(di_)), clips=clips)
    json.dump(rep, open(f"{C}/video_sync.json", "w"), indent=1); print(json.dumps(rep))


if __name__ == "__main__":
    main(sys.argv[1])

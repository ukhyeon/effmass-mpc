"""lv_video_compose.py <tag> - live prep2: side-by-side video of a live run: Kinect colour (sensing's frames with wall time) | Isaac re-render.

Inputs (goal1/live/<tag>/): color/vis_output/*.jpg + color/vis_times.csv (file, wall time), video_inputs/step_wall.npy (wall time of every
Isaac step), isaac_frames/cam0/fNNNNN.jpg or .png (re-render, every N-th step). Output timeline: 30 fps from the later start to the earlier end of
the two streams; at each output time T each panel shows its frame nearest to T (colour: capture time, Isaac: wall time of the
step). Sync report: |t_colour - t_isaac| of the two shown frames (target <= 0.1 s), written to video_sync.json.
Without colour frames the left panel says so (Isaac only). Output goal1/live/<tag>/<tag>_side_by_side.mp4 (1920x540, H.264).
"""
import csv
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw

GG = os.path.expanduser("~/effmass_mpc/goal1")
W, H = 960, 540


def near(a, t):
    k = int(np.searchsorted(a, t)); k = min(max(k, 0), len(a) - 1)
    return k - 1 if k > 0 and abs(a[k - 1] - t) <= abs(a[k] - t) else k


def main(tag):
    d = f"{GG}/live/{tag}"; wall = np.load(f"{d}/video_inputs/step_wall.npy"); n = len(wall)
    fdir = f"{d}/isaac_frames/cam0"
    ext = "jpg" if os.path.exists(fdir) and any(f.endswith(".jpg") for f in os.listdir(fdir)) else "png"
    have = np.array([os.path.exists(f"{fdir}/f{i:05d}.{ext}") for i in range(n)])
    idx = np.nonzero(have)[0]
    if not len(idx):
        sys.exit(f"no Isaac frames in {fdir}")
    tw = wall[idx]
    ct, cf = np.array([]), []
    tf = f"{d}/color/vis_times.csv"
    if os.path.exists(tf):
        rows = [r for r in csv.DictReader(open(tf)) if os.path.exists(f"{d}/color/vis_output/{r['file']}")]
        cf = [f"{d}/color/vis_output/{r['file']}" for r in rows]; ct = np.array([float(r["wall_time"]) for r in rows])
        o = np.argsort(ct); ct = ct[o]; cf = [cf[k] for k in o]
    t0 = max(tw[0], ct[0]) if len(ct) else tw[0]; t1 = min(tw[-1], ct[-1]) if len(ct) else tw[-1]
    T = np.arange(t0, t1, 1 / 30.0)
    out = f"{d}/{tag}_side_by_side.mp4"
    ff = subprocess.Popen(["nice", "-n", "19", "ffmpeg", "-v", "error", "-y", "-threads", "1", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{2*W}x{H}", "-r", "30",
                           "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", "-preset", "medium", "-threads", "1", out], stdin=subprocess.PIPE)
    diffs, lag_c, lag_i = [], [], []
    blank = Image.new("RGB", (W, H), (20, 20, 20)); ImageDraw.Draw(blank).text((20, H // 2), "no Kinect colour frames in this run", fill=(220, 220, 220))
    cache = {}
    for t in T:
        i = int(idx[near(tw, t)])
        if i not in cache:
            cache.clear(); cache[i] = Image.open(f"{fdir}/f{i:05d}.{ext}").convert("RGB").resize((W, H))
        right = cache[i]; lag_i.append(abs(t - wall[i]))
        if len(ct):
            kc = near(ct, t)
            left = Image.open(cf[kc]).convert("RGB").resize((W, H)); lag_c.append(abs(t - ct[kc])); diffs.append(abs(ct[kc] - wall[i]))
        else:
            left = blank
        fr = Image.new("RGB", (2 * W, H)); fr.paste(left, (0, 0)); fr.paste(right, (W, 0))
        ImageDraw.Draw(fr).text((W + 12, 10), f"Isaac step {i}  t {t - t0:6.2f} s", fill=(255, 255, 255))
        ff.stdin.write(fr.tobytes())
    ff.stdin.close(); rc = ff.wait()
    rep = dict(tag=tag, out=out, ffmpeg_exit=rc, frames=len(T), duration_s=float(len(T) / 30.0), isaac_frames=int(len(idx)), color_frames=len(cf),
               panel_time_diff_s=dict(max=float(max(diffs)), p95=float(np.percentile(diffs, 95)), median=float(np.median(diffs))) if diffs else None,
               isaac_offset_s_max=float(max(lag_i)), color_offset_s_max=float(max(lag_c)) if lag_c else None, within_0_1s=bool(diffs and max(diffs) <= 0.1) if diffs else None)
    json.dump(rep, open(f"{d}/video_sync.json", "w"), indent=1); print(json.dumps(rep))


if __name__ == "__main__":
    main(sys.argv[1])

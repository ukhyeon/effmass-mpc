"""lv_clip3.py <json_spec> - overnight2: one three-panel clip (Kinect colour | uniform | per-part reference) from saved Isaac frames.
spec keys: tag (live run), out (mp4), dir_uniform, dir_part (folders with cam0/fNNNNN.jpg|png, one frame per saved step), steps (run
length), tau0_s (replay-file time of step 0's human frame: 0 for compare/, frame0 / 30 for a compare_tip slice), t_from_s / t_to_s
(window in run time, optional), title (top line), labels (3 strings), crf (default 23), gif (optional path: same window, 480 px wide,
10 fps), gif_from_s / gif_to_s (optional sub-window for the GIF).
Time axis: run time tau (human at real time); Isaac panels: nearest saved step to round(tau * 60) - 1; colour: the frame whose sensing
publish time (sensing_perf t_ros_sec, the clock of the HumanPose stamps the replay human is built on) is nearest to t0 + tau0 + tau,
t0 = stamp of the first message of the replay file (as lv_cmp_video.py). Writes <out> and <out>.json (offsets, size).
"""
import csv
import glob
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

GG = os.path.expanduser("~/effmass_mpc/goal1"); O = f"{GG}/logs/meff_isaac"
W, H = 640, 360


def frames(d):
    fs = glob.glob(f"{d}/cam0/f*.jpg") or glob.glob(f"{d}/cam0/f*.png")
    return np.array(sorted(int(os.path.basename(f)[1:6]) for f in fs)), (os.path.splitext(fs[0])[1] if fs else ".jpg")


def near(a, t):
    k = int(np.searchsorted(a, t)); k = min(max(k, 0), len(a) - 1)
    return k - 1 if k > 0 and abs(a[k - 1] - t) <= abs(a[k] - t) else k


def font(sz):
    for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"):
        if os.path.exists(f):
            return ImageFont.truetype(f, sz)
    return ImageFont.load_default()


def colour(tag):
    upd = np.load(f"{O}/{tag}_human_inertia_rec.npz")["upd"]; t0 = float(upd[np.argmin(upd[:, 3]), 1]) / 1e9
    perf = {}
    for f in glob.glob(f"{O}/{tag}_sensing/sensing_perf/*.csv"):
        for r in csv.DictReader(open(f)):
            perf[int(r["frame_index"])] = float(r["t_ros_sec"])
    cd = f"{GG}/live/{tag}/color"; rows = [r for r in csv.DictReader(open(f"{cd}/vis_times.csv")) if os.path.exists(f"{cd}/vis_output/{r['file']}")]
    cap = np.array([float(r["wall_time"]) for r in rows]); ix = [int(r["file"][:-4]) for r in rows]
    off = np.median([perf[i] - c for i, c in zip(ix, cap) if i in perf]) if perf else 0.0
    ct = np.array([perf.get(i, c + off) for i, c in zip(ix, cap)]); cf = [f"{cd}/vis_output/{r['file']}" for r in rows]; o = np.argsort(ct)
    return t0, ct[o], [cf[k] for k in o]


def main(spec):
    S = json.load(open(spec)); t0, ct, cf = colour(S["tag"])
    fi = {m: frames(S[f"dir_{m}"]) for m in ("uniform", "part")}; n = S["steps"]
    a = S.get("t_from_s", 0.0); b = min(S.get("t_to_s", n / 60.0), n / 60.0); T = np.arange(a, b, 1 / 30.0)
    lab = S.get("labels", ["Kinect colour (recorded)", "uniform zone (k 0), seed 0", "per-part reference (k 0.10), seed 0"])
    F1, F2 = font(15), font(13); out = S["out"]
    ff = subprocess.Popen(["nice", "-n", "19", "ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{3*W}x{H + 24}", "-r", "30", "-i", "-",
                           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", str(S.get("crf", 23)), "-preset", "medium", "-threads", "2", out], stdin=subprocess.PIPE)
    dc, di = [], []; gif_frames = []
    ga, gb = S.get("gif_from_s", a), S.get("gif_to_s", b)
    for q, tau in enumerate(T):
        i = max(int(round(tau * 60)) - 1, 0); kc = near(ct, t0 + S["tau0_s"] + tau); dc.append(abs(ct[kc] - (t0 + S["tau0_s"] + tau)))
        pan = [Image.open(cf[kc]).convert("RGB").resize((W, H))]
        for m in ("uniform", "part"):
            idx, ext = fi[m]; k = int(idx[near(idx, i)]); di.append(abs(k - i)); pan.append(Image.open(f"{S[f'dir_{m}']}/cam0/f{k:05d}{ext}").convert("RGB").resize((W, H)))
        fr = Image.new("RGB", (3 * W, H + 24), (0, 0, 0))
        for p, im in enumerate(pan):
            fr.paste(im, (p * W, 24))
        d = ImageDraw.Draw(fr); d.text((8, 3), f"{S.get('title', '')}   t {tau:5.1f} s", font=F1, fill=(255, 255, 255))
        for p, l in enumerate(lab):
            d.rectangle([p * W, 24, p * W + 8 + d.textlength(l, font=F2) + 8, 24 + 20], fill=(0, 0, 0)); d.text((p * W + 8, 26), l, font=F2, fill=(255, 255, 255))
        ff.stdin.write(fr.tobytes())
        if S.get("gif") and ga <= tau < gb and q % 3 == 0:
            gif_frames.append(fr.resize((960, int(960 * (H + 24) / (3 * W)))))
    ff.stdin.close(); rc = ff.wait()
    rep = dict(out=out, ffmpeg_exit=rc, duration_s=float(len(T) / 30.0), size_mb=round(os.path.getsize(out) / 1e6, 2) if os.path.exists(out) else None,
               colour_offset_s_max=float(max(dc)), colour_offset_s_median=float(np.median(dc)), isaac_step_offset_max=int(max(di)))
    if gif_frames:
        gif_frames[0].save(S["gif"], save_all=True, append_images=gif_frames[1:], duration=100, loop=0, optimize=True)
        rep.update(gif=S["gif"], gif_mb=round(os.path.getsize(S["gif"]) / 1e6, 2), gif_s=len(gif_frames) / 10.0)
    json.dump(rep, open(out + ".json", "w"), indent=1); print(json.dumps(rep))


if __name__ == "__main__":
    main(sys.argv[1])

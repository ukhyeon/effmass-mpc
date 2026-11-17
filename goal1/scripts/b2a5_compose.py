"""b2a5_compose.py <tagA> <tagB> <out.mp4> - compose Isaac render frames of two runs into one comparison video (system python, PIL + /usr/bin/ffmpeg).
Grid: columns = runs (A left, B right), rows = oblique / side camera. Per column: label, current gap (shape), mu, eta (= shell growth), minimum gap so far.
30 fps for 60 Hz steps (0.5x), 2.5 s pause at the step where the two gaps are smallest together."""
import os, sys, subprocess, shutil, tempfile
import numpy as np
from PIL import Image, ImageDraw, ImageFont
R = os.path.expanduser("~/effmass_mpc/goal1/b2a/render"); ta, tb, out = sys.argv[1], sys.argv[2], sys.argv[3]
ZA, ZB = np.load(f"{R}/{ta}_prep.npz"), np.load(f"{R}/{tb}_prep.npz"); n = min(len(ZA["q"]), len(ZB["q"]))
FB = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"; FR = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
f_t, f_l, f_v, f_s = ImageFont.truetype(FB, 30), ImageFont.truetype(FB, 28), ImageFont.truetype(FR, 25), ImageFont.truetype(FB, 20)
COL = {"iso": (42, 120, 214), "massA": (235, 104, 52), "unim": (237, 161, 0)}
LAB = {"iso": "iso  (ISO fixed hand mass 0.6 kg)", "massA": "mass A  (personal effective mass)", "unim": "uniform_matched"}
W, H, HD = 960, 540, 124
i_pause = int(np.argmin([max(ZA["gap"][i], 0) + max(ZB["gap"][i], 0) for i in range(n)]))
tmp = tempfile.mkdtemp(dir=os.path.expanduser("~/effmass_mpc/goal1/b2a/render")); k = 0


def tile_text(d, x0, z, i, cond):
    g = d; mn = float(np.min(z["gap"][: i + 1]))
    g.rectangle([x0 + 14, HD + 14, x0 + 470, HD + 196], fill=(252, 252, 251), outline=COL[cond], width=4)
    g.text((x0 + 28, HD + 22), LAB[cond].split("  ")[0], font=f_l, fill=COL[cond])
    g.text((x0 + 28, HD + 60), f"gap now  {z['gap'][i]*100:5.1f} cm", font=f_v, fill=(11, 11, 11))
    g.text((x0 + 28, HD + 90), f"min gap  {mn*100:5.1f} cm", font=f_v, fill=(11, 11, 11))
    g.text((x0 + 28, HD + 120), f"mu       {z['mu'][i]:5.2f} kg", font=f_v, fill=(11, 11, 11))
    g.text((x0 + 28, HD + 150), f"eta      {z['eta'][i]*100:5.1f} cm  (shell)", font=f_v, fill=(11, 11, 11))


for i in range(n):
    img = Image.new("RGB", (2 * W, HD + 2 * H), (252, 252, 251)); d = ImageDraw.Draw(img)
    for c, (tag, z) in enumerate(((ta, ZA), (tb, ZB))):
        cond = str(z["cond"])
        for r, cam in enumerate(("oblique", "side")):
            img.paste(Image.open(f"{R}/{tag}/{cam}_{i:04d}.png").convert("RGB"), (c * W, HD + r * H))
        tile_text(d, c * W, z, i, cond); d.rectangle([c * W, HD - 6, (c + 1) * W - 1, HD - 1], fill=COL[cond])
        d.text((c * W + W // 2, HD - 34), LAB[cond], font=f_t, fill=COL[cond], anchor="mm")
        d.text((c * W + 20, HD + H + 12), "side view", font=f_s, fill=(82, 81, 78))
    d.text((2 * W - 20, 12), f"front-cal, seed 0   t = {(i + 1) / 60:4.2f} s   (0.5x)", font=f_s, fill=(82, 81, 78), anchor="ra")
    d.text((20, 12), "translucent shell = hand shape grown by eta (where the cost starts);  black rod = closest robot sphere to the hand", font=f_s, fill=(82, 81, 78))
    d.line([(W, HD), (W, HD + 2 * H)], fill=(252, 252, 251), width=6)
    reps = 1 + (75 if i == i_pause else 0)
    if i == i_pause:
        d.rectangle([W - 210, HD + H - 26, W + 210, HD + H + 26], fill=(11, 11, 11)); d.text((W, HD + H), "closest moment (paused)", font=f_s, fill=(252, 252, 251), anchor="mm")
    for _ in range(reps):
        img.save(f"{tmp}/f{k:05d}.png"); k += 1
subprocess.run(["/usr/bin/ffmpeg", "-loglevel", "error", "-y", "-framerate", "30", "-i", f"{tmp}/f%05d.png", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out], check=True)
shutil.rmtree(tmp); print("wrote", out, k, "frames, pause at step", i_pause)

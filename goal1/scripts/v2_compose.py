"""v2_compose.py - V2 demo videos (system python: PIL + /usr/bin/ffmpeg). 2 x 2 panels from single-camera recordings (goal1/b2a/demo/v2/rec/<name>/cam0).
Cut per panel: motion start (step 0) to arrival at B + 0.3 s; panels aligned at their own closest approach (hand gap from the B1 rows); closest +-0.5 s at
0.5x, the rest real time; current hand gap per panel; title band over the first 0.7 s; 1.5 s end card with the minimum gap per panel; corner note."""
import os, re, json, subprocess, shutil, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont
H0 = os.path.expanduser("~/effmass_mpc/goal1"); REC = f"{H0}/b2a/demo/v2/rec"; OUTD = f"{H0}/b2a/demo/v2"; O = f"{H0}/logs/meff_isaac"; HAND = [18, 19, 20]
W, H, PW, PH = 1920, 1080, 960, 540
FB, FM = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
F = {s: ImageFont.truetype(FB, s) for s in (20, 24, 28, 34, 44)}; FMn = ImageFont.truetype(FM, 26)
NOTE = "zone drawn for point robot, m_r 4.6 kg"
VIDEOS = {
    "v2_main": ("Uniform clearance vs directional effective mass", [
        ("rec_front_u065", "v1_front_0.38h1_u065_s0", "Approach from front", "Uniform clearance (baseline)", (42, 120, 214)),
        ("rec_front_gain", "v1_front_0.38h1_gain_s0", "Approach from front", "Directional effective mass (ours)", (235, 104, 52)),
        ("rec_top_u065", "v1_top_0.455_u065_s0", "Approach from above", "Uniform clearance (baseline)", (42, 120, 214)),
        ("rec_top_gain", "v1_top_0.455_gain_s0", "Approach from above", "Directional effective mass (ours)", (235, 104, 52))]),
    "v2_gain_knob": ("Gain knob, approach from front: k_gain 0 -> 0.05 -> 0.10 -> 0.15", [
        ("rec_knob_gk000", "v1_front_0.38h1_gk000_s0", "k_gain 0.00 (uniform 6.5 cm)", "", (42, 120, 214)),
        ("rec_knob_gk005", "v1_front_0.38h1_gk005_s0", "k_gain 0.05", "", (27, 175, 122)),
        ("rec_knob_gain", "v1_front_0.38h1_gain_s0", "k_gain 0.10 (ours)", "", (235, 104, 52)),
        ("rec_knob_gk015", "v1_front_0.38h1_gk015_s0", "k_gain 0.15", "", (163, 52, 27))])}


def run_info(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read(); r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))
    g = np.load(f"{O}/{tag}_b1.npz")["rows"][:, 12:36][:, HAND].min(1); return g, r["arrival_list"][0]["step"]


def frame(name, i):
    d = f"{REC}/{name}/cam0"; fp = f"{d}/f{i:05d}.png"
    if not os.path.exists(fp):
        have = sorted(int(x[1:6]) for x in os.listdir(d)); fp = f"{d}/f{min(have, key=lambda h: abs(h - i)):05d}.png"
    return fp


def build(vid, title, panels, out):
    os.makedirs(out, exist_ok=True); info = {}
    for nm, tag, *_ in panels:
        g, arr = run_info(tag); nrec = max(int(x[1:6]) for x in os.listdir(f"{REC}/{nm}/cam0"))
        info[nm] = dict(g=g, cl=int(np.argmin(g[: arr + 19])), en=min(arr + 18, nrec), tag=tag, arrival=arr)
    r0 = min(-v["cl"] for v in info.values()); r1 = max(v["en"] - v["cl"] for v in info.values()); rs = [r for r in range(r0, r1 + 1) if abs(r) <= 30 or (r - r0) % 2 == 0]; k = 0
    for fi, r in enumerate(rs):
        img = Image.new("RGB", (W, H), (252, 252, 251)); d = ImageDraw.Draw(img)
        for j, (nm, tag, row, col, rgb) in enumerate(panels):
            v = info[nm]; i = int(np.clip(v["cl"] + r, 0, v["en"])); x0, y0 = (j % 2) * PW, (j // 2) * PH
            img.paste(Image.open(frame(nm, i)).convert("RGB").resize((PW, PH)), (x0, y0))
            d.rectangle([x0, y0, x0 + PW - 1, y0 + 6], fill=rgb)
            lab = f"{row}  |  {col}" if col else row
            d.rectangle([x0 + 12, y0 + 14, x0 + 12 + min(PW - 24, 24 + 15 * len(lab)), y0 + 54], fill=(252, 252, 251), outline=rgb, width=3); d.text((x0 + 24, y0 + 34), lab, font=F[24], fill=rgb, anchor="lm")
            d.rectangle([x0 + 12, y0 + 62, x0 + 292, y0 + 100], fill=(252, 252, 251)); d.text((x0 + 24, y0 + 81), f"hand gap {v['g'][i]*100:5.1f} cm", font=FMn, fill=(11, 11, 11), anchor="lm")
        d.line([(PW, 0), (PW, H)], fill=(252, 252, 251), width=4); d.line([(0, PH), (W, PH)], fill=(252, 252, 251), width=4)
        d.rectangle([W - 140, 10, W - 10, 54], fill=(11, 11, 11)); d.text((W - 75, 32), "0.5x" if abs(r) <= 30 else "1x", font=F[28], fill=(252, 252, 251), anchor="mm")
        d.rectangle([W // 2 - 330, H - 44, W // 2 + 330, H - 8], fill=(11, 11, 11)); d.text((W // 2, H - 26), f"aligned at closest approach   t = {r / 60:+.2f} s", font=F[20], fill=(252, 252, 251), anchor="mm")
        d.rectangle([10, H - 44, 460, H - 8], fill=(252, 252, 251)); d.text((20, H - 26), NOTE, font=F[20], fill=(82, 81, 78), anchor="lm")
        if fi < 21:
            d.rectangle([0, H // 2 - 50, W, H // 2 + 50], fill=(24, 24, 24)); d.text((W // 2, H // 2), title, font=F[44 if len(title) < 60 else 34], fill=(252, 252, 251), anchor="mm")
        img.save(f"{out}/f{k:05d}.png"); k += 1
    card = Image.new("RGB", (W, H), (252, 252, 251)); d = ImageDraw.Draw(card); d.text((W // 2, 110), "Minimum hand gap (shape-based)", font=F[44], fill=(11, 11, 11), anchor="mm")
    d.text((W // 2, 165), title, font=F[24], fill=(82, 81, 78), anchor="mm"); x0, xs = 820, 900 / 0.15; mins = {}
    for j, (nm, tag, row, col, rgb) in enumerate(panels):
        v = info[nm]; gmin = float(v["g"][: v["en"] + 1].min()); mins[nm] = gmin; y = 240 + 170 * j; Lb = int(max(gmin, 0) * xs)
        d.text((x0 - 20, y + 45), (f"{row} / {col.split(' (')[0]}" if col else row), font=F[28], fill=rgb, anchor="rm")
        if Lb > 0:
            d.rectangle([x0, y, x0 + Lb, y + 90], fill=rgb)
        d.text((x0 + Lb + 20, y + 45), f"{gmin*100:.1f} cm", font=F[34], fill=(11, 11, 11), anchor="lm")
    for _ in range(45):
        card.save(f"{out}/f{k:05d}.png"); k += 1
    return dict(frames=k, seconds=k / 30, min_gap_cm={nm: round(v * 100, 2) for nm, v in mins.items()}, closest_step={nm: v["cl"] for nm, v in info.items()},
                cut_end_step={nm: v["en"] for nm, v in info.items()})


if __name__ == "__main__":
    res = {}
    for vid, (title, panels) in VIDEOS.items():
        tmp = f"{OUTD}/frames_{vid}"; r = build(vid, title, panels, tmp); mp4 = f"{OUTD}/{vid}.mp4"
        subprocess.run(["/usr/bin/ffmpeg", "-loglevel", "error", "-y", "-framerate", "30", "-i", f"{tmp}/f%05d.png", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", mp4], check=True)
        res[vid] = dict(file=mp4, **r)
    json.dump(res, open(f"{O}/v2_video_info.json", "w"), indent=1); print(json.dumps(res, indent=1))

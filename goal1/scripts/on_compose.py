"""on_compose.py - overnight WP7 videos (system python3: PIL + /usr/bin/ffmpeg), same layout as r1_compose.py: two panels (uniform | ours) of the single-camera
recordings goal1/b2a/demo/overnight/rec/<vid>_<cond>/cam0, synchronised by simulation time, real time (every second 60 Hz step at 30 fps); per panel the
gap of the measured part (shape, from the offline series of the original run) and the hand state; title band over the first 0.7 s; 1.5 s end card."""
import os, json, subprocess, sys
import numpy as np
from PIL import Image, ImageDraw, ImageFont
H0 = os.path.expanduser("~/effmass_mpc/goal1"); REC = f"{H0}/b2a/demo/overnight/rec"; OUTD = f"{H0}/b2a/demo/overnight"; O = f"{H0}/logs/meff_isaac"
FB, FM = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"; F = {s: ImageFont.truetype(FB, s) for s in (20, 24, 28, 36, 44)}; FMn = ImageFont.truetype(FM, 26)
W, H, PW, PH = 1920, 1080, 960, 540; Y0 = (H - PH) // 2
VIDS = {"on_static_P4_tipfront": ("on3_P4_tipfront_{c}_s0", "human_on3_P4_tipfront_v2", "Static arm to the side, robot crosses in front of the fingertip (seed 0)", 7.0, "forearm/hand gap"),
        "on_M1_handover": ("on4_M1_v025_{c}_s0", "human_on4_M1_v025_v2", "Hand-over, fingertip peak 0.25 m/s, hold 3 s (seed 0)", 20.0, "hand gap"),
        "on_M2_crossing": ("on4_M2_v025_{c}_s0", "human_on4_M2_v025_v2", "Hand sweeps across the robot path, 0.25 m/s (seed 0)", 20.0, "hand gap")}
PAN = [("gk000", "Uniform clearance (baseline)", (42, 120, 214)), ("gain", "Directional effective mass (ours)", (235, 104, 52))]
only = sys.argv[1:] or list(VIDS); res = json.load(open(f"{O}/on_wp7_video_info.json")) if os.path.exists(f"{O}/on_wp7_video_info.json") else {}
for vid in only:
    tp, hf, title, SEC, glab = VIDS[vid]; ph = np.load(f"{H0}/data/{hf}.npz")["phase"]; Z = {c: np.load(f"{O}/{tp.format(c=c)}_on_series.npz") for c, _, _ in PAN}
    n = min(int(SEC * 60), min(len(Z[c]["gap_shape"]) for c in Z)); tmp = f"{OUTD}/frames_{vid}"; os.makedirs(tmp, exist_ok=True); k = 0
    for i in range(0, n, 2):
        img = Image.new("RGB", (W, H), (252, 252, 251)); d = ImageDraw.Draw(img); d.text((24, 40), title, font=F[36], fill=(11, 11, 11), anchor="lm")
        d.text((W - 24, 40), f"t = {(i + 1) / 60:5.2f} s   1x", font=F[28], fill=(82, 81, 78), anchor="rm")
        for j, (c, lab, rgb) in enumerate(PAN):
            dd = f"{REC}/{vid}_{c}/cam0"; fp = f"{dd}/f{i:05d}.png"
            if not os.path.exists(fp):
                have = sorted(int(x[1:6]) for x in os.listdir(dd)); fp = f"{dd}/f{min(have, key=lambda h: abs(h - i)):05d}.png"
            x0 = j * PW; img.paste(Image.open(fp).convert("RGB").resize((PW, PH)), (x0, Y0)); d.rectangle([x0, Y0 - 8, x0 + PW - 1, Y0 - 1], fill=rgb)
            d.text((x0 + PW // 2, Y0 - 40), lab, font=F[28], fill=rgb, anchor="mm"); g = Z[c]["gap_shape"][i]; st = str(ph[min(Z[c]["frame"][i], len(ph) - 1)])
            d.rectangle([x0 + 12, Y0 + 12, x0 + 520, Y0 + 92], fill=(252, 252, 251), outline=rgb, width=3)
            d.text((x0 + 24, Y0 + 32), f"{glab} {g*100:5.1f} cm", font=FMn, fill=(11, 11, 11), anchor="lm"); d.text((x0 + 24, Y0 + 70), f"hand: {st}", font=FMn, fill=(82, 81, 78), anchor="lm")
        d.line([(PW, Y0), (PW, Y0 + PH)], fill=(252, 252, 251), width=4); d.text((24, Y0 + PH + 30), "zone drawn for point robot, m_r 4.6 kg (moving hand: zone for the nearest key pose)", font=F[20], fill=(82, 81, 78), anchor="lm")
        if k < 21:
            d.rectangle([0, H // 2 - 50, W, H // 2 + 50], fill=(24, 24, 24)); d.text((W // 2, H // 2), title, font=F[36], fill=(252, 252, 251), anchor="mm")
        img.save(f"{tmp}/f{k:05d}.png"); k += 1
    card = Image.new("RGB", (W, H), (252, 252, 251)); d = ImageDraw.Draw(card); d.text((W // 2, 200), f"Minimum {glab} in the first {n/60:.0f} s (shape-based)", font=F[44], fill=(11, 11, 11), anchor="mm"); mins = {}
    for j, (c, lab, rgb) in enumerate(PAN):
        g = float(np.min(Z[c]["gap_shape"][:n])); mins[c] = g; d.text((W // 2, 420 + 140 * j), f"{lab}: {g*100:.1f} cm", font=F[44], fill=rgb, anchor="mm")
    for _ in range(45):
        card.save(f"{tmp}/f{k:05d}.png"); k += 1
    mp4 = f"{OUTD}/{vid}.mp4"; subprocess.run(["/usr/bin/ffmpeg", "-loglevel", "error", "-y", "-framerate", "30", "-i", f"{tmp}/f%05d.png", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", mp4], check=True)
    res[vid] = dict(file=mp4, frames=k, seconds=k / 30, min_gap_cm={c: round(g * 100, 2) for c, g in mins.items()})
json.dump(res, open(f"{O}/on_wp7_video_info.json", "w"), indent=1); print(json.dumps(res, indent=1))

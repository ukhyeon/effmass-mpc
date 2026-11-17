"""demo_compose_v2.py - demo v2 (system python: PIL + /usr/bin/ffmpeg). Per scene, 2 x 2 panels (off, uniform_matched, iso, mass A), one camera per file
(cam1 = zoomed side view: main; cam0 = wide oblique: separate file). Cut per condition: from the motion start (step 0; the robot is within 0.30 m of the hand
from the start because of the 0.23 m end rule, so the 0.30 m entry rule does not cut) to the arrival at B + 0.3 s. Panels aligned at each condition's
closest approach (same screen time); outside its cut a panel holds its first / last frame. Closest +-0.5 s at 0.5x, the rest real time. Title band over the
first 0.7 s of motion; 1.5 s end card with the minimum hand gap per condition. Output goal1/b2a/demo/final_v2/."""
import os, re, json, subprocess, shutil
import numpy as np
from PIL import Image, ImageDraw, ImageFont
H0 = os.path.expanduser("~/effmass_mpc/goal1/b2a"); REC = f"{H0}/demo/rec_v2"; FIN = f"{H0}/demo/final_v2"; PREP = f"{H0}/render"; O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac")
CONDS = ["off", "unim", "iso", "massA"]
LAB = {"off": "0 No human cost (speed limit only)", "unim": "1 Uniform clearance (baseline MPPI)", "iso": "2 ISO fixed mass", "massA": "3 Directional effective mass (ours)"}
SHORT = {"off": "0 No human cost", "unim": "1 Uniform clearance", "iso": "2 ISO fixed mass", "massA": "3 Directional eff. mass (ours)"}
COL = {"off": (110, 110, 106), "unim": (200, 135, 0), "iso": (42, 120, 214), "massA": (235, 104, 52)}
SCENES = [("A", "front_0.38h1", "A. Outstretched hand, robot passes in front of the fingertips"), ("B", "top_0.455", "B. Outstretched hand, robot passes over the back of the hand"),
          ("C", "block_0.47hc", "C. Hand blocks the straight path")]
W, H, PW, PH = 1920, 1080, 960, 540
FB, FM = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
F = {s: ImageFont.truetype(FB, s) for s in (22, 26, 30, 36, 44)}; FMn = ImageFont.truetype(FM, 24)


def arrival(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read(); r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1])); return r["arrival_list"][0]["step"]


def frame_path(key, c, cam, i):
    d = f"{REC}/{key}_{c}_s0/cam{cam}"; fp = f"{d}/f{i:05d}.png"
    if not os.path.exists(fp):
        have = sorted(int(x[1:6]) for x in os.listdir(d)); fp = f"{d}/f{min(have, key=lambda h: abs(h - i)):05d}.png"   # steps 0-1: renderer warm-up
    return fp


def build(key, title, cam, out):
    os.makedirs(out, exist_ok=True); Z = {c: np.load(f"{PREP}/b2a5_{key}_{c}_s0_prep.npz") for c in CONDS}
    nrec = {c: len(os.listdir(f"{REC}/{key}_{c}_s0/cam{cam}")) + 2 for c in CONDS}
    cl = {c: int(np.argmin(Z[c]["gap"])) for c in CONDS}; st = {c: 0 for c in CONDS}; en = {c: min(arrival(f"b2a5_{key}_{c}_s0") + 18, nrec[c] - 1) for c in CONDS}
    r0 = min(st[c] - cl[c] for c in CONDS); r1 = max(en[c] - cl[c] for c in CONDS); rs = [r for r in range(r0, r1 + 1) if abs(r) <= 30 or (r - r0) % 2 == 0]; k = 0
    for fi, r in enumerate(rs):
        img = Image.new("RGB", (W, H), (252, 252, 251)); d = ImageDraw.Draw(img)
        for j, c in enumerate(CONDS):
            i = int(np.clip(cl[c] + r, st[c], en[c])); x0, y0 = (j % 2) * PW, (j // 2) * PH
            img.paste(Image.open(frame_path(key, c, cam, i)).convert("RGB").resize((PW, PH)), (x0, y0)); z = Z[c]
            d.rectangle([x0 + 12, y0 + 12, x0 + 600, y0 + 92], fill=(252, 252, 251), outline=COL[c], width=4)
            d.text((x0 + 24, y0 + 20), LAB[c], font=F[22], fill=COL[c])
            et = f"eta {z['eta'][i]*100:4.1f} cm" if c != "off" else "eta    -"
            d.text((x0 + 24, y0 + 54), f"gap {z['gap'][i]*100:5.1f} cm  mu {z['mu'][i]:4.2f} kg  {et}", font=FMn, fill=(11, 11, 11))
        d.line([(PW, 0), (PW, H)], fill=(252, 252, 251), width=4); d.line([(0, PH), (W, PH)], fill=(252, 252, 251), width=4)
        sp = "0.5x" if abs(r) <= 30 else "1x"; d.rectangle([W - 150, 10, W - 10, 56], fill=(11, 11, 11)); d.text((W - 80, 33), sp, font=F[30], fill=(252, 252, 251), anchor="mm")
        d.rectangle([W // 2 - 300, H - 44, W // 2 + 300, H - 8], fill=(11, 11, 11)); d.text((W // 2, H - 26), f"aligned at closest approach   t = {r / 60:+.2f} s", font=F[22], fill=(252, 252, 251), anchor="mm")
        if fi < 21:   # title band over the first 0.7 s of motion
            d.rectangle([0, H // 2 - 50, W, H // 2 + 50], fill=(24, 24, 24)); d.text((W // 2, H // 2), title, font=F[44], fill=(252, 252, 251), anchor="mm")
        img.save(f"{out}/f{k:05d}.png"); k += 1
    mins = {c: float(np.min(Z[c]["gap"][: en[c] + 1])) for c in CONDS}
    card = Image.new("RGB", (W, H), (252, 252, 251)); d = ImageDraw.Draw(card); d.text((W // 2, 120), "Minimum hand gap (shape-based)", font=F[44], fill=(11, 11, 11), anchor="mm")
    d.text((W // 2, 175), title, font=F[26], fill=(82, 81, 78), anchor="mm"); x0, xs, ymax = 640, 1000 / 0.10, 0.10
    d.line([(x0, 240), (x0, 240 + 4 * 150)], fill=(82, 81, 78), width=3)
    for j, c in enumerate(CONDS):
        y = 270 + 150 * j; g = mins[c]; L = int(max(g, 0) * xs)
        d.text((x0 - 20, y + 45), SHORT[c], font=F[30], fill=COL[c], anchor="rm")
        if L > 0:
            d.rectangle([x0, y, x0 + L, y + 90], fill=COL[c])
        d.text((x0 + L + 20, y + 45), f"{g*100:.1f} cm" + ("  (overlap)" if g < 0 else ""), font=F[36], fill=(11, 11, 11), anchor="lm")
    for _ in range(45):
        card.save(f"{out}/f{k:05d}.png"); k += 1
    return dict(frames=k, seconds=k / 30, min_gap_cm={c: round(v * 100, 2) for c, v in mins.items()}, closest_step=cl, cut_end_step=en, aligned_range_steps=[r0, r1])


def encode(src, out):
    subprocess.run(["/usr/bin/ffmpeg", "-loglevel", "error", "-y", "-framerate", "30", "-i", f"{src}/f%05d.png", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out], check=True)


if __name__ == "__main__":
    os.makedirs(FIN, exist_ok=True); tmp = f"{H0}/demo/compose_v2_frames"; info = {}; bund = f"{tmp}/bundle"; os.makedirs(bund, exist_ok=True); b = 0
    for sid, key, title in SCENES:
        for cam, view in ((1, "zoom"), (0, "wide")):
            out = f"{tmp}/{sid}_{view}"; r = build(key, title, cam, out); name = f"{FIN}/demo_{sid}_{key.split('_')[0]}_{view}_4panel.mp4"; encode(out, name)
            info[f"{sid}_{view}"] = dict(file=name, **r)
            if view == "zoom":
                for f in sorted(os.listdir(out)):
                    os.link(f"{out}/{f}", f"{bund}/f{b:05d}.png"); b += 1
    encode(bund, f"{FIN}/demo_all_A_B_C_zoom.mp4"); info["bundle"] = dict(file=f"{FIN}/demo_all_A_B_C_zoom.mp4", frames=b, seconds=b / 30)
    json.dump(info, open(f"{FIN}/demo_v2_info.json", "w"), indent=1); print(json.dumps(info, indent=1))

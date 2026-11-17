"""demo_compose.py - final demo videos from the recorded frames (system python: PIL + /usr/bin/ffmpeg). Per scene: conditions side by side
(time-synchronised by step), camera 1 (oblique) on top, camera 2 (side, horizontal) below; per panel: condition name, current hand gap (shape),
closest-pair mu, eta (B2a-5 run records via b2a5_isaac_prep.py). Real time except the closest window (each condition's closest step +-0.5 s,
union) at 0.5x; 2 s end card with the minimum hand gap per condition. Bundle: A -> B -> C with 1 s title cards. Output goal1/b2a/demo/final/."""
import os, sys, subprocess, shutil
import numpy as np
from PIL import Image, ImageDraw, ImageFont
H0 = os.path.expanduser("~/effmass_mpc/goal1/b2a"); REC = f"{H0}/demo/rec"; FIN = f"{H0}/demo/final"; PREP = f"{H0}/render"
SCENES = [("A", "front_0.38h1", ["iso", "massA"], "Outstretched hand, approach from front", "Scene A: the robot passes in front of the fingertips"),
          ("B", "top_0.455", ["iso", "massA"], "Outstretched hand, pass over the back of the hand", "Scene B: the robot passes over the back of the hand"),
          ("C", "block_0.47hc", ["off", "iso", "massA"], "Hand blocks the straight path", "Scene C: the straight path runs through the hand")]
LAB = {"iso": "iso: ISO fixed mass", "massA": "mass A: directional effective mass", "off": "off: no human cost, speed limit only"}
COL = {"iso": (42, 120, 214), "massA": (235, 104, 52), "off": (110, 110, 106)}
CW, CH, HD = 1920, 1140, 60
FB, FM = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
F = {s: ImageFont.truetype(FB, s) for s in (20, 24, 30, 40)}; FMn = {s: ImageFont.truetype(FM, s) for s in (20, 24)}


def scene_frames(sid, key, conds, title, out):
    os.makedirs(out, exist_ok=True); Z = {c: np.load(f"{PREP}/b2a5_{key}_{c}_s0_prep.npz") for c in conds}
    n = min(len(Z[c]["q"]) for c in conds); nc = len(conds); pw = CW // nc; ph = pw * 9 // 16; y0 = HD + (CH - HD - 2 * ph) // 2
    closest = {c: int(np.argmin(Z[c]["gap"][:n])) for c in conds}; slow = np.zeros(n, bool)
    for c in conds:
        slow[max(0, closest[c] - 30): min(n, closest[c] + 31)] = True
    seq = [i for i in range(n) if slow[i] or i % 2 == 0]; k = 0
    for i in seq:
        img = Image.new("RGB", (CW, CH), (252, 252, 251)); d = ImageDraw.Draw(img)
        d.text((24, HD // 2), f"{sid}. {title}", font=F[30], fill=(11, 11, 11), anchor="lm")
        d.text((CW - 24, HD // 2), f"t = {(i + 1) / 60:4.2f} s" + ("   0.5x (closest window)" if slow[i] else "   1x"), font=F[24], fill=(82, 81, 78), anchor="rm")
        for j, c in enumerate(conds):
            for r in range(2):
                fp = f"{REC}/{key}_{c}_s0/cam{r}/f{i:05d}.png"
                fp = fp if os.path.exists(fp) else f"{REC}/{key}_{c}_s0/cam{r}/f{min(int(x[1:6]) for x in os.listdir(os.path.dirname(fp))):05d}.png"   # steps 0-1: renderer warm-up, first saved frame used
                im = Image.open(fp).convert("RGB").resize((pw, ph)); img.paste(im, (j * pw, y0 + r * ph))
            z = Z[c]; x0 = j * pw
            d.rectangle([x0, y0, x0 + pw - 1, y0 + 6], fill=COL[c])
            bw = min(pw - 20, 600); d.rectangle([x0 + 10, y0 + 14, x0 + 10 + bw, y0 + 96], fill=(252, 252, 251), outline=COL[c], width=3)
            d.text((x0 + 22, y0 + 22), LAB[c], font=F[20] if nc == 3 else F[24], fill=COL[c])
            et = f"eta {z['eta'][i]*100:4.1f} cm" if c != "off" else "eta  -"
            d.text((x0 + 22, y0 + 58), f"gap {z['gap'][i]*100:5.1f} cm  mu {z['mu'][i]:4.2f} kg  {et}", font=FMn[20] if nc == 3 else FMn[24], fill=(11, 11, 11))
            d.rectangle([x0 + 10, y0 + ph + 10, x0 + 10 + min(pw - 20, 360), y0 + ph + 42], fill=(252, 252, 251)); d.text((x0 + 18, y0 + ph + 26), f"{c if c != 'massA' else 'mass A'} - side view", font=F[20], fill=COL[c], anchor="lm")
        img.save(f"{out}/f{k:05d}.png"); k += 1
    last = img.copy(); d = ImageDraw.Draw(last); bx = (CW // 2 - 520, CH // 2 - 60 - 40 * len(conds), CW // 2 + 520, CH // 2 + 60 + 40 * len(conds))
    d.rectangle(bx, fill=(252, 252, 251), outline=(11, 11, 11), width=3); d.text((CW // 2, bx[1] + 50), "Minimum hand gap (shape-based)", font=F[40], fill=(11, 11, 11), anchor="mm")
    mins = {}
    for j, c in enumerate(conds):
        g = float(np.min(Z[c]["gap"][:n])); mins[c] = g
        d.text((CW // 2, bx[1] + 130 + 70 * j), f"{LAB[c].split(':')[0]}:  {g*100:.1f} cm", font=F[40], fill=COL[c], anchor="mm")
    for _ in range(60):
        last.save(f"{out}/f{k:05d}.png"); k += 1
    return k, mins, closest, len(seq)


def card(text, sub, path):
    img = Image.new("RGB", (CW, CH), (24, 24, 24)); d = ImageDraw.Draw(img)
    d.text((CW // 2, CH // 2 - 30), text, font=F[40], fill=(252, 252, 251), anchor="mm"); d.text((CW // 2, CH // 2 + 40), sub, font=F[24], fill=(195, 194, 183), anchor="mm"); img.save(path)


def encode(src, out):
    subprocess.run(["/usr/bin/ffmpeg", "-loglevel", "error", "-y", "-framerate", "30", "-i", f"{src}/f%05d.png", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out], check=True)


if __name__ == "__main__":
    os.makedirs(FIN, exist_ok=True); tmp = f"{H0}/demo/compose_frames"; info = {}
    bund = f"{tmp}/bundle"; os.makedirs(bund, exist_ok=True); b = 0
    for sid, key, conds, title, desc in SCENES:
        out = f"{tmp}/{sid}"; nfr, mins, closest, nseq = scene_frames(sid, key, conds, title, out)
        name = f"{FIN}/demo_{sid}_{key.split('_')[0]}_{'_vs_'.join(conds)}.mp4"; encode(out, name)
        info[sid] = dict(file=name, frames=nfr, seconds=nfr / 30, min_gap_cm={c: round(v * 100, 2) for c, v in mins.items()}, closest_step=closest, scene_frames=nseq)
        card(desc, title, f"{tmp}/card_{sid}.png")
        for _ in range(30):
            shutil.copy(f"{tmp}/card_{sid}.png", f"{bund}/f{b:05d}.png"); b += 1
        for f in sorted(os.listdir(out)):
            os.link(f"{out}/{f}", f"{bund}/f{b:05d}.png"); b += 1
    encode(bund, f"{FIN}/demo_all_A_B_C.mp4"); info["bundle"] = dict(file=f"{FIN}/demo_all_A_B_C.mp4", frames=b, seconds=b / 30)
    import json; json.dump(info, open(f"{FIN}/demo_final_info.json", "w"), indent=1, default=str); print(json.dumps(info, indent=1, default=str))

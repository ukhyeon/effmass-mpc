"""lv_mesh_clips.py <tag> - overnight2: three-panel clips with the translucent body mesh (frames from lv_mesh_rerender.sh) via lv_clip3.py:
  front pass, per motion: goal1/live/<tag>/media/front_<motion>_mesh.mp4 (whole slice, motion with 2 s before and after)
  arm side, per motion:   goal1/live/<tag>/media/side_<motion>_mesh.mp4 (motion window of compare/analysis.json with 2 s before and after)
  GIFs (<= 10 s): difference = front pass motion 1, same = arm side motion 1, 10 s around the closest moment of seed 0 (per-part run
  for the front pass, uniform run for the arm side)."""
import json, os, subprocess, sys
GG = os.path.expanduser("~/effmass_mpc/goal1"); tag = sys.argv[1]; L = f"{GG}/live/{tag}"; M = f"{L}/mesh_frames"; OUT = f"{L}/media"; os.makedirs(OUT, exist_ok=True)
T = f"{L}/compare_tip"; S = {m["name"]: m for m in json.load(open(f"{T}/scenes.json"))["motions"]}; A = json.load(open(f"{T}/analysis.json"))
CA = json.load(open(f"{L}/compare/analysis.json")); CR = json.load(open(f"{L}/compare/runs.json"))
LAB = ["Kinect colour (recorded)", "uniform zone (k 0), seed 0", "per-part reference (k 0.10), seed 0"]
def gif(mp4, out, a, b):   # Isaac panels only (uniform | per-part), 960 px wide, 10 fps, palette
    vf = f"crop=1280:384:640:0,scale=960:-1:flags=lanczos,fps=10,split[x][y];[x]palettegen=max_colors=128[p];[y][p]paletteuse=dither=bayer:bayer_scale=4"
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{a:.2f}", "-t", f"{min(b - a, 10.0):.2f}", "-i", mp4, "-filter_complex", vf, out])
    print("gif", out, round(os.path.getsize(out) / 1e6, 2), "MB", flush=True)


def run(spec, name):
    g = spec.pop("gif", None); ga = spec.pop("gif_from_s", None); gb = spec.pop("gif_to_s", None)
    p = f"{OUT}/{name}.spec.json"; json.dump(spec, open(p, "w")); r = subprocess.run(["python3", f"{GG}/scripts/lv_clip3.py", p], capture_output=True, text=True)
    print(name, (r.stdout.strip() or r.stderr.strip()[-300:])[:300], flush=True)
    if g:
        t0 = spec.get("t_from_s", 0.0); gif(spec["out"], g, ga - t0, gb - t0)
for mot, m in S.items():
    nm = mot.replace(" ", ""); su, sp = f"tip_{tag}_{nm.replace('motion', 'm')}_uni_s0", f"tip_{tag}_{nm.replace('motion', 'm')}_part_s0"
    if not os.path.isdir(f"{M}/{su}/cam0") or not os.path.isdir(f"{M}/{sp}/cam0"):
        print(nm, "front frames missing"); continue
    spec = dict(tag=tag, out=f"{OUT}/front_{nm}_mesh.mp4", dir_uniform=f"{M}/{su}", dir_part=f"{M}/{sp}", steps=m["steps"], tau0_s=m["frame0"] / 30.0, labels=LAB,
                title=f"{tag} {mot}: robot path just in front of the {m['target']} (replay at real time, re-rendered, body mesh translucent)")
    if mot == "motion 1":
        tm = A[mot]["part"]["seeds"][0]["t_min_s"]; spec.update(gif=f"{OUT}/gif_difference_front_{nm}.gif", gif_from_s=max(tm - 5, 0), gif_to_s=min(tm + 5, m["steps"] / 60.0))
    run(spec, f"front_{nm}_mesh")
su, sp = f"cmp_{tag}_uni_s0", f"cmp_{tag}_part_s0"
for e in CA["intervals"]:
    if e["t0_s"] is None:
        continue
    nm = e["name"].replace(" ", "")
    if not os.path.isdir(f"{M}/{su}/cam0") or not os.path.isdir(f"{M}/{sp}/cam0"):
        print(nm, "side frames missing"); continue
    spec = dict(tag=tag, out=f"{OUT}/side_{nm}_mesh.mp4", dir_uniform=f"{M}/{su}", dir_part=f"{M}/{sp}", steps=CR["steps"], tau0_s=0.0, labels=LAB,
                t_from_s=max(e["t0_s"] - 2, 0), t_to_s=e["t1_s"] + 2, title=f"{tag} {e['name']}: recorded robot path, the arm lies across it (replay at real time, re-rendered, body mesh translucent)")
    if e["name"] == "motion 1":
        tm = e["per_mode"]["uniform"]["seeds"][0]["t_min_s"]; spec.update(gif=f"{OUT}/gif_same_side_{nm}.gif", gif_from_s=max(tm - 5, e["t0_s"] - 2), gif_to_s=tm + 5)
    run(spec, f"side_{nm}_mesh")

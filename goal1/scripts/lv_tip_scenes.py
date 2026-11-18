"""lv_tip_scenes.py <tag> [motion=dx,dy ...] - live cmp tip: per motion, the scene json, frame file and human slice for the tip paths.
Inputs: compare_tip/paths.json, compare_tip/ik.json (and ik_shift<dx>_<dy>.json for a motion given as "motion2=0.10,0.20": the virtual
robot base moved by (dx, dy) m in the Franka base frame = world, since the run's frame file has no rotation).
Per motion: half length h = 0.30 m if IK holds on the whole line at dz 0, else the largest symmetric h (1 cm) with all points solved at
the dz (0, +-0.025 .. +-0.10, smallest |dz| first) that gives the largest h; at least 0.15 m or the motion is "no path".
  scene: copy of scene_live_front.json with goals (hand link = TCP + 0.1034, gripper down), q_start = IK at A, tcp_A / tcp_B, path "tip"
  frame: the run's frame file, or a copy with base_position_world moved by (dx, dy) (compare_tip/frame_<motion>.yml)
  human: frames of the motion with 2 s before and after (compare_tip/human_<motion>_v2.npz, input_time from 0, rate 30); data unchanged
Writes compare_tip/scenes.json (all of the above, steps = 2 * frames - 1).
"""
import json
import os
import sys

import numpy as np

GG = os.path.expanduser("~/effmass_mpc/goal1")


RAW = {}


def main(tag, shifts):
    C = f"{GG}/live/{tag}/compare"; T = f"{GG}/live/{tag}/compare_tip"
    cfg = json.load(open(f"{C}/runs.json")); P = json.load(open(f"{T}/paths.json")); base_sc = json.load(open(cfg["scene"]))
    z = dict(np.load(cfg["human"])); F = len(z["p0"]); fr_txt = open(cfg["frame_file"]).read()
    out = dict(tag=tag, motions=[])
    for m in P["motions"]:
        nm = m["name"].replace(" ", ""); sh = shifts.get(nm, (0.0, 0.0))
        ikf = f"{T}/ik.json" if sh == (0.0, 0.0) else f"{T}/ik_shift" + RAW[nm].replace(",", "_") + ".json"
        if not os.path.exists(ikf):
            sys.exit(f"no IK file {ikf}")
        IK = json.load(open(ikf))
        s = np.array(IK["s"]); best = None
        for dz in IK["dz"]:
            d = IK["motions"][m["name"]][f"{dz:+.3f}"]; ok = np.array(d["success"], bool); i0 = int(np.argmin(np.abs(s)))
            h = 0
            while i0 - h - 1 >= 0 and i0 + h + 1 < len(s) and ok[i0 - h - 1:i0 + h + 2].all():
                h += 1
            if ok[i0] and (best is None or h > best[1]):
                best = (dz, h, d)
            if h == i0:
                break
        rec = dict(name=m["name"], t0_s=m["t0_s"], t1_s=m["t1_s"], target=m["target"], side=m["side"], base_shift=list(sh), offset_m=m.get("offset_m", P["offset_m"]))
        if best is None or s[i0 + best[1]] < 0.15 - 1e-9:
            rec["status"] = "no path"; out["motions"].append(rec); continue
        dz, h, d = best; ia, ib = i0 - h, i0 + h; hl = float(s[ib])
        c = np.array(m["centre"]) - np.array([sh[0], sh[1], 0.0]) + [0, 0, dz]; u = np.array(m["u"])
        A, B = c - hl * u, c + hl * u; q4 = [0.0, 1.0, 0.0, 0.0]
        sc = dict(base_sc); sc.pop("live", None)
        sc.update(path="tip", key=f"{tag}_{nm}", scene="live_tip", goals_wxyz=[(A + [0, 0, 0.1034]).tolist() + q4, (B + [0, 0, 0.1034]).tolist() + q4],
                  q_start=d["q"][ia], tcp_A=A.tolist(), tcp_B=B.tolist(), dy=[-hl, hl], plates=[],
                  note=f"live cmp tip ({tag} {m['name']}): TCP line through the point {m.get('offset_m', P['offset_m'])} m beyond the {m['target']} along the arm axis, "
                       f"horizontal and perpendicular to it, half length {hl:.2f} m, dz {dz:+.3f} m, base shift {sh}")
        scp = f"{T}/scene_{nm}.json"; json.dump(sc, open(scp, "w"), indent=1)
        frp = cfg["frame_file"]
        if sh != (0.0, 0.0):
            import re
            mm = re.search(r"base_position_world:\s*\[([^\]]+)\]", fr_txt); b = [float(v) for v in mm.group(1).split(",")]
            nb = [b[0] + sh[0], b[1] + sh[1], b[2]]
            txt = fr_txt.replace(mm.group(0), f"base_position_world: [{nb[0]:.3f}, {nb[1]:.3f}, {nb[2]:.3f}]")
            txt = f"# live cmp tip: copy of {os.path.basename(cfg['frame_file'])} with the virtual robot base moved by ({sh[0]:+.2f}, {sh[1]:+.2f}) m for {m['name']}\n" + txt
            frp = f"{T}/frame_{nm}.yml"; open(frp, "w").write(txt)
        f0 = max(int(round(m["t0_s"] * 30)) - 60, 0); f1 = min(int(round(m["t1_s"] * 30)) + 60, F - 1); n = f1 - f0 + 1
        hz = {k: (v[f0:f1 + 1] if isinstance(v, np.ndarray) and v.ndim >= 1 and len(v) == F else v) for k, v in z.items()}
        hz["input_time"] = np.arange(n) / 30.0; hz["rate"] = np.float64(30.0)
        hz["source"] = np.array(f"{tag} {m['name']}: frames {f0}-{f1} of compare/human_rt_v2.npz (motion with 2 s before and after), data unchanged")
        hp = f"{T}/human_{nm}_v2.npz"; np.savez(hp, **hz)
        rec.update(status="ok", scene=scp, frame_file=frp, human=hp, frame0=f0, frames=n, steps=2 * n - 1, core_s=[(int(round(m["t0_s"] * 30)) - f0) / 30.0, (int(round(m["t1_s"] * 30)) - f0) / 30.0],
                   half_length_m=hl, dz_m=dz, tcp_A=A.tolist(), tcp_B=B.tolist(), centre=c.tolist(), tip=(np.array(m["tip"]) - [sh[0], sh[1], 0.0]).tolist(),
                   axis=m["axis"], target_shape=m["target_shape"], hold_frames=[int(f) - f0 for f in m.get("hold_frames", []) if f0 <= f <= f1],
                   hold_share=m.get("hold_share"))
        out["motions"].append(rec)
        print(m["name"], "half %.2f m, dz %+.3f, shift %s, frames %d-%d, steps %d" % (hl, dz, sh, f0, f1, 2 * n - 1))
    json.dump(out, open(f"{T}/scenes.json", "w"), indent=1)


if __name__ == "__main__":
    sh = {}
    for a in sys.argv[2:]:
        k, v = a.split("="); sh[k] = tuple(float(x) for x in v.split(",")); RAW[k] = v
    main(sys.argv[1], sh)

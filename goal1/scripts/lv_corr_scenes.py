"""lv_corr_scenes.py <tag> <motion 1..> <ver> <prep|finish> [key=value ...] - live3m corrected paths: one path version of one motion.
Files in goal1/live/<tag>/corrected/cal/m<motion><ver>/. The recorded human data are not changed (a slice is copied).
prep (keys, m unless noted; defaults in DEF):
  phi   path tilt (deg): 0 = horizontal line u = z x d_h (perpendicular to the target direction d); 90 = the line w = d x u (perpendicular
        to d in the vertical plane of d, pointing up); values between turn the line from u to w
  L     horizontal distance from the reference point to the path centre along d_h (horizontal part of d)
  zrel  path centre height minus reference point height
  lat   path centre moved along u (0 = the reference point at the path centre)
  r0    horizontal distance from the path centre to the virtual robot base
  psi   base direction turned about the vertical from d_h (deg; 0 = the base on the line of d beyond the path)
  blat  base moved along u (the path centre then lies off the robot's plane of symmetry)
  h     half length (0.30)
  tail  seconds of the recording kept after the held window (1.5)
  camside, camturn  video camera only: side of the robot-human line it stands on (+1 / -1) and a turn about the vertical (deg)
  -> params.json, path.json (reference posture = median of the held window, target direction, path A / B in the original frame and
     in the frame of the moved base, base shift), ik_in.json (A, B, centre)
finish (after lv_corr_ik.py wrote ik.json): scene_AB.json / scene_BA.json (start at A resp. B, q_start = IK), frame.yml (base moved),
  human_v2.npz (+ human_verts.npz when compare/human_rt_verts.npz exists): frames from the first held frame to tail s after the window.
"""
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lv_corr_lib as CL

GG = CL.GG; DEF = dict(phi=0.0, L=0.20, zrel=-0.15, lat=0.0, r0=0.55, psi=0.0, blat=0.0, h=0.30, tail=1.5, camside=1.0, camturn=0.0)


def rz(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg)); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def prep(tag, q, ver, kv):
    T = f"{GG}/live/{tag}/corrected/cal/m{q}{ver}"; os.makedirs(T, exist_ok=True); p = dict(DEF); p.update(kv); H = CL.Human(tag); m = H.motion(q - 1)
    win, p0, p1 = H.held(m); ref, kref, d = H.tref(m, p0, p1); dh = np.array([d[0], d[1], 0.0]); dh /= np.linalg.norm(dh)
    u = np.cross([0, 0, 1.0], dh); w = np.cross(d, u); w /= np.linalg.norm(w); ph = np.radians(p["phi"]); e = np.cos(ph) * u + np.sin(ph) * w; e /= np.linalg.norm(e)
    c = ref + p["L"] * dh + np.array([0, 0, p["zrel"]]) + p["lat"] * u; base = c + p["r0"] * (rz(p["psi"]) @ dh) + p["blat"] * u; sh = np.array([base[0], base[1], 0.0])
    A, B = c - p["h"] * e, c + p["h"] * e
    out = dict(tag=tag, motion=m["name"], q=q, ver=ver, side=m["side"], target=m["target"], held_frames=list(win), held_s=(win[1] - win[0] + 1) / 30.0, ref_point=ref.tolist(),
               ref_shape=int(kref), d=d.tolist(), d_elev_deg=float(np.degrees(np.arcsin(d[2]))), u=u.tolist(), e=e.tolist(), centre=c.tolist(), A=A.tolist(), B=B.tolist(),
               base_shift=sh[:2].tolist(), centre_base=(c - sh).tolist(), A_base=(A - sh).tolist(), B_base=(B - sh).tolist(), half_length_m=p["h"], params=p)
    json.dump(p, open(f"{T}/params.json", "w"), indent=1); json.dump(out, open(f"{T}/path.json", "w"), indent=1)
    json.dump(dict(points=[out["A_base"], out["B_base"], out["centre_base"]]), open(f"{T}/ik_in.json", "w"))
    print(m["name"], "held", win, "%.1f s" % out["held_s"], "d elev %.1f" % out["d_elev_deg"], "A", np.round(out["A_base"], 3), "B", np.round(out["B_base"], 3), "shift", np.round(sh[:2], 3))


def finish(tag, q, ver):
    T = f"{GG}/live/{tag}/corrected/cal/m{q}{ver}"; P = json.load(open(f"{T}/path.json")); IK = json.load(open(f"{T}/ik.json")); H = CL.Human(tag); cfg = H.cfg
    if not (IK["success"][0] and IK["success"][1]):
        sys.exit(f"IK failed at the path ends: {IK['success']}")
    base_sc = json.load(open(cfg["scene"])); q4 = [0.0, 1.0, 0.0, 0.0]; A, B = np.array(P["A_base"]), np.array(P["B_base"]); h = P["half_length_m"]
    for nm, (a, b, qs) in dict(AB=(A, B, IK["q"][0]), BA=(B, A, IK["q"][1])).items():
        sc = dict(base_sc); sc.pop("live", None)
        sc.update(path="corr", key=f"{tag}_m{q}{ver}_{nm}", scene="live_corr", goals_wxyz=[(a + [0, 0, 0.1034]).tolist() + q4, (b + [0, 0, 0.1034]).tolist() + q4], q_start=qs,
                  tcp_A=a.tolist(), tcp_B=b.tolist(), dy=[-h, h], plates=[],
                  note=f"live3m corrected ({P['motion']}, version {ver}, start {nm[0]}): TCP line in front of the {P['target']}, params {json.dumps(P['params'])}")
        json.dump(sc, open(f"{T}/scene_{nm}.json", "w"), indent=1)
    fr_txt = open(cfg["frame_file"]).read(); mm = re.search(r"base_position_world:\s*\[([^\]]+)\]", fr_txt); b = [float(v) for v in mm.group(1).split(",")]; sh = P["base_shift"]
    txt = fr_txt.replace(mm.group(0), f"base_position_world: [{b[0] + sh[0]:.4f}, {b[1] + sh[1]:.4f}, {b[2]:.4f}]")
    open(f"{T}/frame.yml", "w").write(f"# live3m corrected: copy of {os.path.basename(cfg['frame_file'])} with the virtual robot base moved by ({sh[0]:+.3f}, {sh[1]:+.3f}) m for {P['motion']} (version {ver})\n" + txt)
    z = dict(H.z); F = H.F; f0, f1 = P["held_frames"][0], min(P["held_frames"][1] + int(round(P["params"]["tail"] * 30)), F - 1); n = f1 - f0 + 1
    hz = {k: (v[f0:f1 + 1] if isinstance(v, np.ndarray) and v.ndim >= 1 and len(v) == F else v) for k, v in z.items()}
    hz["input_time"] = np.arange(n) / 30.0; hz["rate"] = np.float64(30.0)
    hz["source"] = np.array(f"{tag} {P['motion']}: frames {f0}-{f1} of compare/human_rt_v2.npz (held window and {P['params']['tail']} s after), data unchanged")
    np.savez(f"{T}/human_v2.npz", **hz)
    vf = cfg["human"].replace("_v2.npz", "_verts.npz")
    if os.path.exists(vf):
        v = np.load(vf); np.savez(f"{T}/human_verts.npz", **{k: (v[k][f0:f1 + 1] if v[k].ndim >= 1 and len(v[k]) == F else v[k]) for k in v.files})
    d = np.array(P["d"]); nh = -np.array([d[0], d[1], 0.0]); nh /= np.linalg.norm(nh); nh = rz(P["params"].get("camturn", 0.0)) @ nh; sd = rz(-90.0) @ nh * P["params"].get("camside", 1.0)
    tg = np.array(P["centre_base"]) + 0.25 * nh; tg[2] = P["ref_point"][2]; eye = tg + 0.85 * nh + 2.35 * sd + np.array([0, 0, 0.95])
    P["view"] = ",".join(f"{x:.3f}" for x in list(eye) + list(tg))   # video camera (eye, target) in the base frame, as the live run's view: from the side, slightly above
    P.update(frame0=f0, frames=n, steps=2 * n - 1, held_slice=[0, P["held_frames"][1] - f0], human=f"{T}/human_v2.npz", frame_file=f"{T}/frame.yml",
             scene_AB=f"{T}/scene_AB.json", scene_BA=f"{T}/scene_BA.json", q_centre=IK["q"][2], ik_centre_ok=bool(IK["success"][2]))
    json.dump(P, open(f"{T}/path.json", "w"), indent=1); print(P["motion"], "frames", f0, f1, "steps", P["steps"], "held slice", P["held_slice"])


if __name__ == "__main__":
    a = sys.argv; kv = {x.split("=")[0]: float(x.split("=")[1]) for x in a[5:]}
    prep(a[1], int(a[2]), a[3], kv) if a[4] == "prep" else finish(a[1], int(a[2]), a[3])

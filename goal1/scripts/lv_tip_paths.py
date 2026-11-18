"""lv_tip_paths.py <tag> [offset_m | motionN=offset_m ...] - live cmp tip: one robot path per motion of compare/analysis.json, passing just in front of the fingertip
(motions 1, 2) or the elbow (motion 3 and later), from the motion's representative posture (per-shape median of p0 / p1 over the motion's frames,
Franka base frame via the run's frame file). The human data are not changed.
  arm side: the side (left 13/15/17, right 14/16/18) whose hand is nearer to the old path (tcp_A - tcp_B) over the motion
  fingertip: the hand-shape surface point furthest along the forearm axis d (elbow -> wrist, forearm capsule p0 -> p1)
  elbow: the surface point furthest along the upper-arm axis a (shoulder -> elbow) of the upper-arm capsule end and the forearm capsule end p0
  centre c = tip + offset * axis (default 0.03 m; per motion "motion1=0.12" overrides); line direction u = unit(z x axis) (horizontal, perpendicular to the axis);
  TCP line c - h u .. c + h u, h = 0.30 m. Representative posture: see HOLD below (env TIP_HOLD, default 0.04 m; 0 = whole motion). Writes compare_tip/paths.json (and the representative shapes).
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g1_human_link as L

GG = os.path.expanduser("~/effmass_mpc/goal1")
HOLD = float(os.environ.get("TIP_HOLD", "0.04"))   # (hold) 0 = median over the whole motion (first version)


def main(tag, off=0.03, h=0.30, per=None):
    C = f"{GG}/live/{tag}/compare"; T = f"{GG}/live/{tag}/compare_tip"; os.makedirs(T, exist_ok=True)
    cfg = json.load(open(f"{C}/runs.json")); an = json.load(open(f"{C}/analysis.json")); sc = json.load(open(cfg["scene"]))
    z = np.load(cfg["human"]); F = len(z["p0"]); R, t = L.load_frame(cfg["frame_file"])
    P0 = z["p0"].reshape(F, -1, 3).astype(float) @ R.T + t; P1 = z["p1"].reshape(F, -1, 3).astype(float) @ R.T + t
    rad = z["shape_radius"].astype(float); seg = z["shape_segment"].astype(int)
    A0, B0 = np.array(sc["tcp_A"]), np.array(sc["tcp_B"])
    ks = lambda s: [k for k in range(len(rad)) if seg[k] == s and rad[k] > 0]
    out = dict(tag=tag, offset_m=off, half_length_m=h, frame_file=cfg["frame_file"], motions=[])
    mots = [e for e in an["intervals"] if e["t0_s"] is not None]
    for q, e in enumerate(mots):
        fs_all = np.arange(int(round(e["t0_s"] * 30)), min(int(round(e["t1_s"] * 30)), F)); fs = fs_all
        p0 = np.median(P0[fs], 0); p1 = np.median(P1[fs], 0)
        def dline(p):
            ab = B0 - A0; tt = np.clip((p - A0) @ ab / (ab @ ab), 0, 1); return float(np.linalg.norm(p - (A0 + tt * ab)))
        side = min(("L", (13, 15, 17)), ("R", (14, 16, 18)), key=lambda sd: min(dline(0.5 * (p0[k] + p1[k])) for k in ks(sd[1][2])))
        su, sf, sh = side[1]; fa = ks(sf)[0]; ua = ks(su)[0]
        target = "fingertip" if q < 2 else "elbow"
        if target == "fingertip":
            ax = p1[fa] - p0[fa]; ax /= np.linalg.norm(ax)
            cand = []
            for k in ks(sh):
                a = p1[k] - p0[k]; n = np.linalg.norm(a); a = a / n if n > 1e-9 else ax
                cand += [(p1[k] + rad[k] * a, k), (p0[k] - rad[k] * a, k)]
        else:
            ax = p1[ua] - p0[ua]; ax /= np.linalg.norm(ax)
            cand = [(p1[ua] + rad[ua] * ax, ua), (p0[fa] + rad[fa] * ax, fa)]
        tip, kt = max(cand, key=lambda c: float(c[0] @ ax))
        if HOLD:   # (hold) representative posture = median over the frames that hold the pose: start from the middle third of the motion,
            # keep frames whose target point is within HOLD m of that median perpendicular to the path direction u (sliding along the
            # path does not change the distance to it), then the median of the kept frames; the same target / axis rules again
            def tpt(f):
                if target == "fingertip":
                    return P1[f][kt] + rad[kt] * (P1[f][kt] - P0[f][kt]) / max(np.linalg.norm(P1[f][kt] - P0[f][kt]), 1e-9)
                return P1[f][kt] + rad[kt] * (P1[f][kt] - P0[f][kt]) / max(np.linalg.norm(P1[f][kt] - P0[f][kt]), 1e-9) if kt == ua else P0[f][kt]
            mid = fs_all[len(fs_all) // 3:2 * len(fs_all) // 3]; u0 = np.cross([0, 0, 1.0], ax); u0 /= np.linalg.norm(u0)
            m0 = np.median([tpt(f) for f in mid], 0)
            keep = np.array([f for f in fs_all if np.linalg.norm((tpt(f) - m0) - ((tpt(f) - m0) @ u0) * u0) <= HOLD])
            fs = keep; p0 = np.median(P0[fs], 0); p1 = np.median(P1[fs], 0)
            if target == "fingertip":
                ax = p1[fa] - p0[fa]; ax /= np.linalg.norm(ax); cand = []
                for k in ks(sh):
                    a = p1[k] - p0[k]; n = np.linalg.norm(a); a = a / n if n > 1e-9 else ax
                    cand += [(p1[k] + rad[k] * a, k), (p0[k] - rad[k] * a, k)]
            else:
                ax = p1[ua] - p0[ua]; ax /= np.linalg.norm(ax); cand = [(p1[ua] + rad[ua] * ax, ua), (p0[fa] + rad[fa] * ax, fa)]
            tip, kt = max(cand, key=lambda c: float(c[0] @ ax))
        om = (per or {}).get(e["name"].replace(" ", ""), off)
        c = tip + om * ax; u = np.cross([0, 0, 1.0], ax); u /= np.linalg.norm(u)
        bend = float(np.degrees(np.arccos(np.clip(((p1[ua] - p0[ua]) / np.linalg.norm(p1[ua] - p0[ua])) @ ((p1[fa] - p0[fa]) / np.linalg.norm(p1[fa] - p0[fa])), -1, 1))))
        out["motions"].append(dict(name=e["name"], t0_s=e["t0_s"], t1_s=e["t1_s"], side=side[0], target=target, offset_m=om, target_shape=int(kt), axis=ax.tolist(),
                                   tip=tip.tolist(), centre=c.tolist(), u=u.tolist(), A=(c - h * u).tolist(), B=(c + h * u).tolist(),
                                   axis_elev_deg=float(np.degrees(np.arcsin(np.clip(ax[2], -1, 1)))), elbow_bend_deg=bend, frames=[int(fs_all[0]), int(fs_all[-1])],
                                   hold_frames=[int(f) for f in fs], hold_share=float(len(fs) / len(fs_all)), hold_m=HOLD))
        np.savez(f"{T}/rep_{e['name'].replace(' ', '')}.npz", p0=p0, p1=p1, rad=rad, seg=seg)
    json.dump(out, open(f"{T}/paths.json", "w"), indent=1)
    for m in out["motions"]:
        print(m["name"], m["side"], m["target"], "tip", np.round(m["tip"], 3), "axis", np.round(m["axis"], 2), "elev %.0f" % m["axis_elev_deg"], "bend %.0f" % m["elbow_bend_deg"],
              "A", np.round(m["A"], 3), "B", np.round(m["B"], 3))


if __name__ == "__main__":
    a = sys.argv[2:]; per = {x.split("=")[0]: float(x.split("=")[1]) for x in a if "=" in x}; g = [float(x) for x in a if "=" not in x]
    main(sys.argv[1], g[0] if g else 0.03, per=per)

"""pr_live_prep.py <tag> - (part) turn a live Kinect run into a precomputed replay file (hrc_ws world) for the deterministic path.

Human per sensing message: sensing's saved camera-frame poses (<tag>_sensing/pose_npz) through HumanInertiaCore (camera pose and betas
from the sensing log, pelvis correction, shapes v2) - the node's computation. Time of message k: the input stamp Isaac recorded for
input_count k + 1 (<tag>_human_inertia_rec.npz upd); messages Isaac never pumped get times interpolated between known ones.
Resampled at 30 Hz (latest message at or before each tick); velocity from the core with the stamp difference (as the node).
Writes goal1/data/human_live_<tag>_v2.npz (or the path given as 2nd argument) (keys of the precomputed files) and prints the frame file the run used.
"""
import glob
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.expanduser("~/effmass_mpc/ws/src/effmass_human"))

H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; GG = f"{H}/effmass_mpc/goal1"


def main(tag):
    from effmass_human.human_inertia_core import HumanInertiaCore
    from effmass_human.human_shapes import ShapeSet
    slog = open(f"{O}/{tag}_sensing.log", errors="replace").read()
    m = re.findall(r"\[camera pose\] roll ([-\d.]+) deg, pitch ([-\d.]+) deg.*?yaw ([-\d.]+) deg, t \[([^\]]+)\]", slog)[-1]
    cam = dict(roll_deg=float(m[0]), pitch_deg=float(m[1]), yaw_deg=float(m[2]), t=[float(v) for v in m[3].split(",")])
    bm = re.findall(r"cliff beta mean : \[([^\]]+)\]", slog, re.S)
    betas = np.array([float(v) for v in bm[-1].split()], np.float32)[:10]
    pz = np.load(sorted(glob.glob(f"{O}/{tag}_sensing/pose_npz/*.npz"))[-1]); PO, TR = pz["poses"], pz["trans"]; N = len(PO)
    upd = np.load(f"{O}/{tag}_human_inertia_rec.npz")["upd"]
    known = {int(c) - 1: s / 1e9 for c, s in zip(upd[:, 3], upd[:, 1])}
    ks = np.array(sorted(k for k in known if 0 <= k < N)); ts = np.array([known[k] for k in ks])
    tall = np.interp(np.arange(N), ks, ts, left=np.nan, right=np.nan)
    ok = np.isfinite(tall); idx = np.nonzero(ok)[0]
    core = HumanInertiaCore("cpu", camera_pose=cam, pelvis_correction=True); core.set_betas(betas)
    V2 = json.load(open(f"{GG}/data/shapes_v2.json")); E = V2["shapes"] + [V2["inactive"]] * (24 - len(V2["shapes"]))
    core.shapes = ShapeSet([1] * 24, [e["segment"] for e in E], [e["p0_local"] for e in E], [e["p1_local"] for e in E], [e["radius"] for e in E], core.shapes.distal_local)
    msgs = []; prev = None; pt = None
    for k in idx:
        dt = (tall[k] - pt) if pt is not None else None
        o = core.compute(PO[k], TR[k], prev=prev, dt=dt, betas=betas); prev = o; pt = tall[k]
        msgs.append((tall[k], o["com"][0].numpy(), o["vel"][0].numpy(), o["inv_inertia"][0].numpy(), o["shapes"]["p0"], o["shapes"]["p1"]))
    t0 = msgs[0][0]; T = np.array([x[0] for x in msgs]) - t0
    F = int(np.floor(T[-1] * 30)) + 1; tick = np.arange(F) / 30.0; j = np.searchsorted(T, tick + 1e-9, side="right") - 1
    pick = lambda i: np.stack([msgs[jj][i] for jj in j]).astype(np.float32)
    sh = o["shapes"]
    out = dict(com=pick(1), vel=pick(2), inv=pick(3).reshape(F, 19, 3, 3), p0=pick(4), p1=pick(5), input_time=tick.astype(np.float64), rate=np.float64(30.0),
               shape_radius=np.asarray(sh["radius"], np.float32), shape_segment=np.asarray(sh["segment"], np.int64), shape_type=np.asarray(sh["type"], np.uint8),
               frame=j.astype(np.int64), source=np.array(f"live run {tag}: sensing poses -> core (pelvis correction, shapes v2), 30 Hz resampled"),
               betas=betas, camera=np.array([cam["roll_deg"], cam["pitch_deg"], cam["yaw_deg"], *cam["t"]]))
    path = sys.argv[2] if len(sys.argv) > 2 else f"{GG}/data/human_live_{tag}_v2.npz"; np.savez(path, **out)   # (live cmp) optional output path
    rlog = open(f"{O}/{tag}.log", errors="replace").read(); fm = re.findall(r"\(live\) ROS human: frame file (\S+),", rlog)
    print(json.dumps(dict(tag=tag, out=path, frames_30hz=F, duration_s=float(T[-1]), messages_used=len(msgs), sensing_frames=N,
                          frame_file=fm[-1] if fm else f"{GG}/config/live_frame.yml", camera=cam)))


if __name__ == "__main__":
    main(sys.argv[1])

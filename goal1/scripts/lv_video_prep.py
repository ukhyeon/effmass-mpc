"""lv_video_prep.py <tag> - live prep2: inputs for re-rendering a live run (lv_live_kinect.sh) in Isaac and for the side-by-side video.

From the run's own records only:
  - human: sensing's saved camera-frame poses (<tag>_sensing/pose_npz, one row per published HumanPose) -> HumanInertiaCore with the
    run's camera pose, betas, pelvis correction and shapes v2 (the node's computation) -> goal1/data/human_vid_<tag>_v2.npz,
    one frame per sensing message (hrc_ws world)
  - per Isaac step: the message Isaac had pumped last (<tag>_human_inertia_rec.npz upd, input_count -> pose row) -> step_frames.npy,
    the measured joints of the step (steps csv q0..q6) -> playback_q.npz, the wall time of the step (cumulative loop_ms anchored
    at the pump times of new messages, as lv_analyze.py --ros) -> step_wall.npy
Writes goal1/live/<tag>/video_inputs/ and prints a json line (frame file of the run, steps, mu_ref mode, k_gain).
"""
import csv
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
    out = f"{GG}/live/{tag}/video_inputs"; os.makedirs(out, exist_ok=True)
    slog = open(f"{O}/{tag}_sensing.log", errors="replace").read()
    m = re.findall(r"\[camera pose\] roll ([-\d.]+) deg, pitch ([-\d.]+) deg.*?yaw ([-\d.]+) deg, t \[([^\]]+)\]", slog)[-1]
    cam = dict(roll_deg=float(m[0]), pitch_deg=float(m[1]), yaw_deg=float(m[2]), t=[float(v) for v in m[3].split(",")])
    bm = re.findall(r"cliff beta mean : \[([^\]]+)\]", slog, re.S)
    betas = np.array([float(v) for v in bm[-1].split()], np.float32)[:10] if bm else np.zeros(10, np.float32)
    pzs = sorted(glob.glob(f"{O}/{tag}_sensing/pose_npz/*.npz"))
    rlog = open(f"{O}/{tag}.log", errors="replace").read()
    r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", rlog)[-1])); rows = list(csv.DictReader(open(r["steps_csv"])))
    n = len(rows); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows])
    upd = np.load(f"{O}/{tag}_human_inertia_rec.npz")["upd"]
    hnew = np.array([int(float(x["h_new"])) for x in rows]); loop = np.array([float(x["loop_ms"]) for x in rows]) / 1000
    fm = re.findall(r"\(live\) ROS human: frame file (\S+),", rlog); frame_file = fm[-1] if fm else f"{GG}/config/live_frame.yml"
    mm = re.findall(r"'mode': '(\w+)'", rlog); kg = re.findall(r"--meff-gain', '([\d.]+)'", rlog)
    vc = re.findall(r"'--viz-cam=([-\d.,]+)'", rlog) or re.findall(r"'--viz-cam', '([-\d.,]+)'", rlog)
    t_end = np.cumsum(loop); inew = np.nonzero(hnew)[0]; k = min(len(inew), len(upd))
    if k >= 2:
        wall = np.interp(np.arange(n), inew[:k], upd[:k, 4] / 1e9)
        w0 = t_end + np.median(upd[:k, 4] / 1e9 - t_end[inew[:k]])
        wall[:inew[0]] = w0[:inew[0]] + (wall[inew[0]] - w0[inew[0]]); wall[inew[k - 1] + 1:] = w0[inew[k - 1] + 1:] + (wall[inew[k - 1]] - w0[inew[k - 1]])
    else:   # no human messages (nobody in front of the camera): loop clock only, anchored at the run's ready time
        wall = t_end + os.path.getmtime(f"{O}/{tag}.ready")
    sf = np.zeros(n, np.int64); j = -1
    for i in range(n):
        if hnew[i] and j + 1 < len(upd):
            j += 1
        sf[i] = int(upd[j, 3]) - 1 if j >= 0 else 0
    np.savez(f"{out}/playback_q.npz", q=Q); np.save(f"{out}/step_wall.npy", wall); np.save(f"{out}/step_frames.npy", sf)
    human = ""
    if pzs:
        pz = np.load(pzs[-1]); PO, TR = pz["poses"], pz["trans"]; N = len(PO)
        core = HumanInertiaCore("cpu", camera_pose=cam, pelvis_correction=True); core.set_betas(betas)
        V2 = json.load(open(f"{GG}/data/shapes_v2.json")); E = V2["shapes"] + [V2["inactive"]] * (24 - len(V2["shapes"]))
        core.shapes = ShapeSet([1] * 24, [e["segment"] for e in E], [e["p0_local"] for e in E], [e["p1_local"] for e in E], [e["radius"] for e in E],
                               core.shapes.distal_local)
        R = {kk: [] for kk in ("com", "vel", "inv", "p0", "p1")}
        for i in range(N):
            o = core.compute(PO[i], TR[i], betas=betas)
            R["com"].append(o["com"][0].numpy()); R["vel"].append(np.zeros((19, 3), np.float32)); R["inv"].append(o["inv_inertia"][0].numpy())
            R["p0"].append(o["shapes"]["p0"]); R["p1"].append(o["shapes"]["p1"])
        sh = o["shapes"]
        human = f"{GG}/data/human_vid_{tag}_v2.npz"
        np.savez(human, com=np.array(R["com"], np.float32), vel=np.array(R["vel"], np.float32), inv=np.array(R["inv"], np.float32),
                 p0=np.array(R["p0"], np.float32), p1=np.array(R["p1"], np.float32), input_time=np.arange(N) / 30.0, rate=np.float64(30.0),
                 shape_radius=np.asarray(sh["radius"], np.float32), shape_segment=np.asarray(sh["segment"], np.int64), shape_type=np.asarray(sh["type"], np.uint8),
                 source=np.array(f"live run {tag}: one frame per sensing message (re-render only)"))
        sf = np.clip(sf, 0, N - 1); np.save(f"{out}/step_frames.npy", sf)
    info = dict(tag=tag, steps=n, human=human, frame_file=frame_file, muref_mode="part" if "gain_part" in mm else ("global" if mm else "?"),
                cost_mode=mm[-1] if mm else None, view=vc[-1] if vc else "1.85,0.25,1.0,0.3,-0.05,0.3", k_gain=float(kg[-1]) if kg else None, wall_start=float(wall[0]), wall_end=float(wall[-1]),
                human_messages=int(len(upd)), step_dt_median=float(np.median(np.diff(wall))) if n > 1 else None,
                record_every=int(max(1, np.floor((1 / 30.0) / max(float(np.median(np.diff(wall))), 1e-3)))) if n > 1 else 1, out=out)
    json.dump(info, open(f"{out}/info.json", "w"), indent=1); print(json.dumps(info))


if __name__ == "__main__":
    main(sys.argv[1])

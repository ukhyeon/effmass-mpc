"""lv_live_analyze.py <tag> - analysis of a live Kinect run (lv_live_kinect.sh).

The live path does not store human positions, so they are recomputed exactly as the node did: sensing's saved camera-frame
poses (<tag>_sensing/pose_npz, one row per published HumanPose) -> HumanInertiaCore (camera pose and betas from the sensing log,
pelvis correction, shapes v2) -> live_frame.yml -> Franka base frame. Isaac step i uses the message it had pumped last
(<tag>_human_inertia_rec.npz upd, input_count -> pose row input_count - 1), i.e. the human the MPC saw (not ground truth).
Outputs: min gaps (hand shapes of both hands, all shapes; shape-based, MPC sphere radius + 0.02), near-human robot speed p99,
human placement (pelvis vs the standing place), right-hand reach vs the robot path, approach events (right hand within 0.15 m of
the TCP path segment) with the robot TCP speed / gap during them, human data rate and age. Writes <tag>_live_analysis.json and
<tag>_live_series.npz.
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
import b2a_analyze as BA
import shape_metrics as SM
import g1_human_link as L
import lv_analyze as A

O = BA.O; GG = os.path.expanduser("~/effmass_mpc/goal1")


def main(tag):
    from effmass_human.human_inertia_core import HumanInertiaCore
    from effmass_human.human_shapes import ShapeSet
    slog = open(f"{O}/{tag}_sensing.log", errors="replace").read()
    m = re.findall(r"\[camera pose\] roll ([-\d.]+) deg, pitch ([-\d.]+) deg.*?yaw ([-\d.]+) deg, t \[([^\]]+)\]", slog)[-1]
    cam = dict(roll_deg=float(m[0]), pitch_deg=float(m[1]), yaw_deg=float(m[2]), t=[float(v) for v in m[3].split(",")])
    bm = re.findall(r"cliff beta mean : \[([^\]]+)\]", slog, re.S)
    betas = np.array([float(v) for v in bm[-1].split()], np.float32)[:10] if bm else np.zeros(10, np.float32)
    pz = np.load(sorted(glob.glob(f"{O}/{tag}_sensing/pose_npz/*.npz"))[-1]); PO, TR = pz["poses"], pz["trans"]
    core = HumanInertiaCore("cpu", camera_pose=cam, pelvis_correction=True); core.set_betas(betas)
    V2 = json.load(open(f"{GG}/data/shapes_v2.json")); E = V2["shapes"] + [V2["inactive"]] * (24 - len(V2["shapes"]))
    core.shapes = ShapeSet([1] * 24, [e["segment"] for e in E], [e["p0_local"] for e in E], [e["p1_local"] for e in E], [e["radius"] for e in E],
                           core.shapes.distal_local)
    rlog = open(f"{O}/{tag}.log", errors="replace").read()
    fm = re.findall(r"\(live\) ROS human: frame file (\S+),", rlog)
    frame_file = fm[-1] if fm else f"{GG}/config/live_frame.yml"             # the frame the run used
    Rl, tl = L.load_frame(frame_file)
    F = len(PO); COM = np.zeros((F, 19, 3)); P0 = np.zeros((F, 24, 3)); P1 = np.zeros((F, 24, 3))
    for k in range(F):
        o = core.compute(PO[k], TR[k], betas=betas)
        COM[k] = o["com"][0].numpy(); P0[k] = o["shapes"]["p0"]; P1[k] = o["shapes"]["p1"]
    rad = np.asarray(o["shapes"]["radius"], float); seg = np.asarray(o["shapes"]["segment"])
    comb, p0b, p1b = COM @ Rl.T + tl, P0 @ Rl.T + tl, P1 @ Rl.T + tl                 # Franka base frame
    r, rows = A.load_run(tag); n = len(rows)
    Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows])
    hnew = np.array([int(float(x["h_new"])) for x in rows]); loop = np.array([float(x["loop_ms"]) for x in rows]) / 1000
    rec = np.load(f"{O}/{tag}_human_inertia_rec.npz"); upd = rec["upd"]
    fr = np.full(n, -1); k = -1
    for i in range(n):
        if hnew[i] and k + 1 < len(upd):
            k += 1
        if k >= 0:
            fr[i] = int(upd[k, 3]) - 1                                                    # input_count -> pose row
    # wall time per step from pump anchors (as lv_analyze --ros), age = wall - input stamp
    inew = np.nonzero(hnew)[0]; mm = min(len(inew), len(upd))
    wall = np.interp(np.arange(n), inew[:mm], upd[:mm, 4] / 1e9)
    st = np.full(n, np.nan); k = -1
    for i in range(n):
        if hnew[i] and k + 1 < len(upd):
            k += 1
        if k >= 0:
            st[i] = upd[k, 1] / 1e9
    age = (wall - st) * 1000
    ok = (fr >= 0) & (fr < F)
    hand = [kk for kk in range(24) if seg[kk] in (17, 18) and rad[kk] > 0]; rhand = [kk for kk in range(24) if seg[kk] == 18 and rad[kk] > 0]
    allk = [kk for kk in range(24) if rad[kk] > 0]
    C = np.stack([BA.RI.compute(q)[0] for q in Q])
    HB, HT = BA.RI.body_of["fr3v2_1_hand"]
    TCP = np.stack([(BA.RI._fk(q)[HB + 1] @ HT) @ np.array([0, 0, 0.1034, 1.0]) for q in Q])[:, :3]
    gh = np.full(n, np.nan); ga = np.full(n, np.nan); gall = np.full((n, len(C[0])), np.nan)
    for i in np.nonzero(ok)[0]:
        f = fr[i]
        G = np.stack([SM.segdist(C[i], p0b[f][kk], p1b[f][kk]) - rad[kk] for kk in allk], 1) - BA.RS[:, None]
        gall[i] = G.min(1); ga[i] = G.min()
        gh[i] = (np.stack([SM.segdist(C[i], p0b[f][kk], p1b[f][kk]) - rad[kk] for kk in hand], 1) - BA.RS[:, None]).min()
    vsph = np.linalg.norm(np.diff(C, axis=0), axis=-1) * 60; near = (gall[1:] <= 0.4) & np.isfinite(gall[1:]); vnear = vsph[near]
    sc = json.load(open(f"{GG}/b2a/g2/scene_live_front.json")); A_, B_ = np.array(sc["tcp_A"]), np.array(sc["tcp_B"])
    # right hand (segment 18 COM and its shapes) vs the TCP path segment, per sensing frame
    rh = comb[:, 18]; dpath = SM.segdist(rh, A_, B_)
    rh_surf = np.stack([SM.segdist(np.r_[p0b[:, kk], p1b[:, kk]], A_, B_)[:F].clip(0) for kk in rhand], 1).min(1)
    pel_w = COM[:, 0, :2]; camxy = np.asarray(sc["live"]["camera_world_xy"])
    target = (Rl.T @ (np.r_[sc["live"]["stand_pelvis_base_xy"], 0.0] - tl))[:2]   # standing place in world for this frame
    # approach events on the steps: right hand COM within 0.15 m of the path segment
    dstep = np.where(ok, dpath[np.clip(fr, 0, F - 1)], np.nan); app = dstep < 0.15
    ee = TCP
    ev = []; i = 0
    while i < n:
        if app[i]:
            j = i
            while j < n and app[j]:
                j += 1
            seg_ = slice(i, j)
            ev.append(dict(t0_s=float(wall[i] - wall[0]), dur_s=float(wall[j - 1] - wall[i]), steps=int(j - i), min_hand_dist_path_m=float(np.nanmin(dstep[seg_])),
                           min_gap_hand_m=float(np.nanmin(gh[seg_])), min_gap_all_m=float(np.nanmin(ga[seg_])),
                           ee_speed_mean_mps=float(np.nanmean(np.linalg.norm(np.diff(ee[seg_], axis=0), axis=1) * 60)) if ee is not None and j - i > 1 else None))
            i = j
        else:
            i += 1
    nsens = len(PO); dur_s = float(wall[-1] - wall[0])
    res = dict(tag=tag, frame_file=frame_file, stand_place_world=target.tolist(), camera_pose=cam, betas=betas.tolist(), sensing_frames=nsens, steps=n, steps_with_human=int(ok.sum()), realtime_factor=r["realtime_factor"],
               solver_ms_mean=r["solver_ms_mean"], infeasible=int(r["infeasible_steps"]), arrivals=int(r["arrivals"]), early_stop=r.get("early_stop"),
               loop_wall_s=dur_s, human_msgs_to_isaac=int(len(upd)), human_rate_hz=float(len(upd) / dur_s) if dur_s > 0 else None,
               age_ms_median=float(np.nanmedian(age)), age_ms_p95=float(np.nanpercentile(age, 95)),
               min_gap_hand=float(np.nanmin(gh)), min_gap_all=float(np.nanmin(ga)), pen_steps_all=int(np.nansum(ga < 0)),
               v_near_p99=float(np.percentile(vnear, 99)) if len(vnear) else None, sphere_steps_near_04=int(near.sum()),
               pelvis_world_median=np.median(pel_w, 0).tolist(), pelvis_offset_from_place_m=float(np.hypot(*(np.median(pel_w, 0) - target))),
               pelvis_base_dist_m=float(np.hypot(*(Rl @ np.r_[np.median(pel_w, 0), 0.0] + tl)[:2])), pelvis_camera_dist_m=float(np.hypot(*(np.median(pel_w, 0) - camxy))),
               rhand_min_dist_to_path_m=float(dpath.min()), rhand_surface_min_dist_to_path_m=float(rh_surf.min()),
               rhand_base_xyz_at_closest=rh[int(np.argmin(dpath))].tolist(), rhand_frames_within_015=int((dpath < 0.15).sum()),
               approach_events=ev)
    json.dump(res, open(f"{O}/{tag}_live_analysis.json", "w"), indent=1)
    np.savez(f"{O}/{tag}_live_series.npz", frame=fr, gap_hand=gh, gap_all=ga, age_ms=age, wall=wall, rhand_dist_path=dpath, pelvis_world=pel_w, rhand_base=rh)
    print(json.dumps({k: v for k, v in res.items() if k not in ("approach_events", "betas")}, indent=0))
    for e in ev:
        print("approach", json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in e.items()}))


if __name__ == "__main__":
    main(sys.argv[1])

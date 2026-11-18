"""lv_make_live_replay.py - live prep: camera-frame HumanPose stream that puts a precomputed motion at the live standing place
under a given camera pose (the one sensing_node publishes: IMU roll/pitch + camera_pose.yaml yaw/t), so the live pipeline
(effmass_human node with pelvis correction -> live_frame.yml) can be run end to end without a person.

  python3 lv_make_live_replay.py --src human_r1_reach_v050_v2.npz --live human_live_r1_v050_v2.npz --roll 101.05 --pitch 0.58
         --yaw 0 --t -0.83 -2.10 1.66 --out human_live_r1_v050_cam.npz

Source poses: pose66 (camera frame of the legacy camera, roll 110.8, t (-0.80, -2.3, 1.55)), trans T0 (r1_motion.py), betas 0.
Target: the --live file (same motion moved to the standing place, hrc_ws world). With M = live-world rotation of the old world
(Rl^T R1) and c its offset, A = cam_R^T:
  root:  R_c' = A_new^T M A_old R_c
  trans: T' = A_new^T (M (J0 + A_old T0 + t_old) + c - t_new) - J0      (node with pelvis correction: pelvis = A_new (T' + J0) + t_new)
Check: HumanInertiaCore (camera pose new, pelvis_correction True) on the output reproduces the --live segment COMs.
Output keys: pose66, trans (per frame), betas, camera (the pose used), check_max_err_m.
"""
import argparse
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.expanduser("~/effmass_mpc/ws/src/effmass_human"))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g1_human_link as L
from effmass_human.human_inertia_core import HumanInertiaCore, LEGACY_CAMERA_POSE
from effmass_human.utils import camera_to_world_rotmat

ap = argparse.ArgumentParser()
ap.add_argument("--src", required=True); ap.add_argument("--live", required=True); ap.add_argument("--out", required=True)
ap.add_argument("--trans-from", default=os.path.expanduser("~/hrc_ws/pose_npz/human_state_1781524848.npz"))
ap.add_argument("--roll", type=float, required=True); ap.add_argument("--pitch", type=float, required=True); ap.add_argument("--yaw", type=float, default=0.0)
ap.add_argument("--t", nargs=3, type=float, required=True)
ap.add_argument("--scene", default=os.path.expanduser("~/effmass_mpc/goal1/b2a/g2/scene_live_front.json"))
ap.add_argument("--world-xy", nargs=2, type=float, default=None, help="(placement test) move the person so the pelvis is at this world xy ...")
ap.add_argument("--heading-deg", type=float, default=None, help="... and faces this world heading (deg from +x, CCW); the --live file faces 0 deg")
ap.add_argument("--out-world", default="", help="(placement test) also write the moved motion as a precomputed file in hrc_ws world")
a = ap.parse_args()

A_of = lambda r, p, y: camera_to_world_rotmat(r, p, y).numpy().astype(np.float64).T
Lg = LEGACY_CAMERA_POSE
A_old = A_of(Lg["roll_deg"], Lg["pitch_deg"], Lg["yaw_deg"]); t_old = np.asarray(Lg["t"], np.float64)
A_new = A_of(a.roll, a.pitch, a.yaw); t_new = np.asarray(a.t, np.float64)
R1, t1 = L.load_frame(os.path.expanduser("~/effmass_mpc/goal1/b2a/g2/temp_transform.json"))
Rl, tl = L.load_frame(os.path.expanduser("~/effmass_mpc/goal1/config/live_frame.yml"))
src = np.load(a.src); live = np.load(a.live)
P = src["pose66"].astype(np.float64).reshape(len(src["pose66"]), 22, 3); F = len(P)
T0 = np.load(a.trans_from)["trans"][0].astype(np.float64)
core = HumanInertiaCore("cpu", camera_pose="legacy"); core.set_betas(np.zeros(10, np.float32))
J0 = core.rest_joints_19[0, 0].numpy().astype(np.float64)
# the shift d used to build --live: pelvis COM moved to the standing place (same rule as the scene check)
com_old_isaac = src["com"].reshape(F, 19, 3)[0, 0].astype(np.float64) @ R1.T + t1
import json
d = np.zeros(3); d[:2] = np.asarray(json.load(open(a.scene))["live"]["stand_pelvis_base_xy"]) - com_old_isaac[:2]
M = Rl.T @ R1; c = Rl.T @ (t1 + d - tl)                    # live world p = M p_old + c
live_com = live["com"].reshape(F, 19, 3).astype(np.float64)
Rw = np.eye(3); move = lambda p: p
if a.world_xy is not None:                                  # extra rigid move in world: rotate about the pelvis, then put the pelvis at world_xy
    h = np.radians(a.heading_deg or 0.0); ch, sh = np.cos(h), np.sin(h)
    Rw = np.array([[ch, -sh, 0.0], [sh, ch, 0.0], [0.0, 0.0, 1.0]])
    pel = live_com[0, 0].copy(); tgt = np.array([a.world_xy[0], a.world_xy[1], pel[2]])
    c = Rw @ (c - pel) + tgt; M = Rw @ M
    move = lambda p: (p - pel) @ Rw.T + tgt
    live_com = move(live_com)
Rc = Rotation.from_rotvec(P[:, 0]).as_matrix()
Rc_new = np.einsum("ij,fjk->fik", A_new.T @ M @ A_old, Rc)
pose_new = P.copy(); pose_new[:, 0] = Rotation.from_matrix(Rc_new).as_rotvec()
T_new = A_new.T @ (M @ (J0 + A_old @ T0 + t_old) + c - t_new) - J0
trans = np.tile(T_new, (F, 1))
# check through the node path
chk = HumanInertiaCore("cpu", camera_pose=dict(roll_deg=a.roll, pitch_deg=a.pitch, yaw_deg=a.yaw, t=list(t_new)), pelvis_correction=True)
chk.set_betas(np.zeros(10, np.float32)); err = 0.0
for f in range(0, F, 25):
    o = chk.compute(pose_new[f].reshape(-1).astype(np.float32), trans[f].astype(np.float32), betas=np.zeros(10, np.float32))
    err = max(err, float(np.abs(o["com"][0].numpy().astype(np.float64) - live_com[f]).max()))
np.savez(a.out, pose66=pose_new.reshape(F, 66).astype(np.float32), trans=trans.astype(np.float32), betas=np.zeros(10, np.float32),
         camera=np.array([a.roll, a.pitch, a.yaw, *t_new]), check_max_err_m=err, src=a.src, live=a.live)
if a.out_world:
    out = {k: live[k] for k in live.files}
    for k in ("com", "p0", "p1"):
        out[k] = move(live[k].reshape(F, -1, 3).astype(np.float64)).reshape(live[k].shape).astype(live[k].dtype)
    out["vel"] = (live["vel"].reshape(F, 19, 3).astype(np.float64) @ Rw.T).reshape(live["vel"].shape).astype(live["vel"].dtype)
    out["inv"] = np.einsum("ij,fsjk,lk->fsil", Rw, live["inv"].reshape(F, 19, 3, 3).astype(np.float64), Rw).reshape(live["inv"].shape).astype(live["inv"].dtype)
    Mr = Rw @ (Rl.T @ R1)                                   # 'rot' of the source files is in the legacy node world
    out["rot"] = np.einsum("ij,fsjk->fsik", Mr, src["rot"].reshape(F, 19, 3, 3).astype(np.float64)).reshape(src["rot"].shape).astype(src["rot"].dtype)
    np.savez(a.out_world, **out)
    print(f"[live replay] world file {a.out_world}")
print(f"[live replay] {F} frames, trans {np.round(T_new, 4).tolist()}, check vs --live COM: max {err * 1000:.3f} mm -> {a.out}")

"""lv_place.py - live: place the virtual Franka relative to where the person stands (facing the monitor).

Reads /effmass/human_inertia_state (hrc_ws world, effmass_human node) and /hrc/camera_pose. After Enter (or at once with --auto)
and a 3 s countdown it records --seconds (5 s): pelvis position (segment COM 0) and the body heading (horizontal part of the pelvis
local +z axis, SMPL forward), mean and spread. Spread of the heading > --max-heading-std (5 deg) -> measure again.

World -> Franka base (same relative layout as scene_live_front.json): the pelvis goes to base (1.05, -0.15) and the person faces
the base -x direction (towards the robot). With heading psi: base yaw theta = psi - 180 deg, base xy = pelvis_xy - Rz(theta) (1.05, -0.15),
base z = 0.9 (world). p_base = Rz(theta)^T (p_world - base); written as live_frame_user.yml (rotation_z_deg, base_position_world,
read by g1_human_link.load_frame) with the measurement.

Checks (fail -> not saved, unless --force):
  - horizontal distance pelvis - camera in [1.9, 3.6] m
  - standing place (pelvis) and the fully reached right fingertip (scene contact point, base (0.34, 0.04, 0.44)) inside the camera's
    horizontal field of view with a 5 deg margin (depth NFOV 75 deg wide) and within 3.86 m
  - camera side: angle between the person's heading and the camera->person direction: <= 40 deg = camera behind the person
    (warning: the reaching arm can be hidden by the body), >= 140 deg = front, else side
Also checks the result: the measured pelvis maps to base (1.05, -0.15); effective mass unchanged under the rotation (1e-9).
"""
import argparse
import datetime
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from effmass_msgs.msg import HumanInertiaState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g1_human_link as L

GG = os.path.expanduser("~/effmass_mpc/goal1")
STAND_BASE = np.array([1.05, -0.15]); BASE_Z = 0.9
REACH_BASE = np.array([0.34, 0.04, 0.44])          # right fingertip at full reach on the path (scene_live_front.json live.check_with)
HFOV_DEG = 75.0; MARGIN_DEG = 5.0; RANGE_M = 3.86; DIST_RANGE = (1.9, 3.6)


def Rz(deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def cam_R(roll, pitch, yaw):
    a, b, c = np.radians([roll, pitch, yaw])
    Rx = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    Ry = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
    Rz_ = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
    return Rx @ Ry @ Rz_                           # = utils.camera_to_world_rotmat (hrc_ws convention)


def view(p_world, cam):
    """horizontal angle from the optical axis (deg) and range (m) of a world point in the colour camera frame"""
    C = cam_R(cam["roll_deg"], cam["pitch_deg"], cam["yaw_deg"]); pc = C @ (np.asarray(p_world, float) - np.asarray(cam["t"], float))
    return float(np.degrees(np.arctan2(pc[0], pc[2]))), float(np.linalg.norm(pc))


def frame_from(pelvis_xy, heading_deg):
    theta = heading_deg - 180.0
    b = np.r_[np.asarray(pelvis_xy, float) - (Rz(theta) @ np.r_[STAND_BASE, 0.0])[:2], BASE_Z]
    return theta, b


def evaluate(pel, head, head_std, cam):
    theta, b = frame_from(pel[:2], head)
    R = Rz(theta).T; t = -R @ b
    pb = R @ pel + t
    reach_w = R.T @ (REACH_BASE - t)
    d_cam = float(np.hypot(*(pel[:2] - np.asarray(cam["t"][:2]))))
    a_st, r_st = view(np.r_[pel[:2], 1.0], cam); a_rc, r_rc = view(reach_w, cam)
    lim = HFOV_DEG / 2 - MARGIN_DEG
    to_person = pel[:2] - np.asarray(cam["t"][:2]); hv = np.array([np.cos(np.radians(head)), np.sin(np.radians(head))])
    ang = float(np.degrees(np.arccos(np.clip(hv @ to_person / np.linalg.norm(to_person), -1, 1))))
    side = "behind" if ang <= 40 else ("front" if ang >= 140 else "side")
    fails, warns = [], []
    if not (DIST_RANGE[0] <= d_cam <= DIST_RANGE[1]):
        fails.append(f"camera distance {d_cam:.2f} m outside {DIST_RANGE[0]}-{DIST_RANGE[1]} m")
    if abs(a_st) > lim or r_st > RANGE_M:
        fails.append(f"standing place at {a_st:+.1f} deg / {r_st:.2f} m (limit +-{lim:.1f} deg, {RANGE_M} m)")
    if abs(a_rc) > lim or r_rc > RANGE_M:
        fails.append(f"reached hand at {a_rc:+.1f} deg / {r_rc:.2f} m (limit +-{lim:.1f} deg, {RANGE_M} m)")
    if side == "behind":
        warns.append(f"camera sees you from behind (heading vs camera->you {ang:.0f} deg <= 40): the reaching arm can be hidden by your body")
    # checks of the transform itself
    rng = np.random.default_rng(0); Lam = rng.normal(size=(3, 3)); Lam = Lam @ Lam.T + 0.1 * np.eye(3); worst = 0.0
    for _ in range(200):
        u = rng.normal(size=3); u /= np.linalg.norm(u); mw = 1 / (u @ Lam @ u); ub = R @ u; mb = 1 / (ub @ (R @ Lam @ R.T) @ ub)
        worst = max(worst, abs(mb - mw) / abs(mw))
    return dict(theta=theta, base=b, pelvis_base=pb, reach_world=reach_w, cam_dist=d_cam, stand_angle=a_st, stand_range=r_st, reach_angle=a_rc,
                reach_range=r_rc, camera_side=side, side_angle=ang, fails=fails, warns=warns, meff_rel_err=worst, head_std=head_std)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=f"{GG}/config/live_frame_user.yml")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--max-heading-std", type=float, default=5.0)
    ap.add_argument("--force", action="store_true", help="save even if a check fails")
    ap.add_argument("--auto", action="store_true", help="no Enter: measure as soon as a person is seen (tests)")
    ap.add_argument("--tries", type=int, default=20)
    a = ap.parse_args()
    be = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    tl = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    rclpy.init(); n = rclpy.create_node("lv_place"); buf = []; cam = {}; rec = [False]
    n.create_subscription(String, "/hrc/camera_pose", lambda m: cam.update(json.loads(m.data)), tl)

    def on(m):
        P = np.array(m.com_position, float).reshape(-1, 3); Rr = np.array(m.rotation, float).reshape(-1, 3, 3)
        f = Rr[0] @ np.array([0.0, 0.0, 1.0])
        buf.append((time.time(), P[0], np.degrees(np.arctan2(f[1], f[0])))) if rec[0] else None
        last[0] = (P[0], np.degrees(np.arctan2(f[1], f[0])))
    last = [None]
    n.create_subscription(HumanInertiaState, "/effmass/human_inertia_state", on, be)
    spin = lambda s: [rclpy.spin_once(n, timeout_sec=0.02) for _ in range(int(s / 0.02))]
    print("[place] waiting for the camera pose and a person ...", flush=True)
    t0 = time.time()
    while (not cam or last[0] is None) and time.time() - t0 < 300:
        spin(0.5)
        if time.time() - t0 > 3 and int(time.time() - t0) % 5 == 0:
            print(f"[place]   camera pose {'ok' if cam else 'waiting'}, person {'seen' if last[0] is not None else 'not yet (sensing estimates the body shape first)'}", flush=True)
    if not cam or last[0] is None:
        sys.exit("[place] no camera pose or no person within 300 s")
    print(f"[place] camera pose: roll {cam['roll_deg']:.2f}, pitch {cam['pitch_deg']:.2f}, yaw {cam['yaw_deg']:.2f}, t {cam['t']}", flush=True)
    for attempt in range(1, a.tries + 1):
        if not a.auto:
            input(f"[place] attempt {attempt}: stand where you want, facing the monitor. Enter = measure ({a.seconds:g} s after a 3 s countdown): ")
        for k in (3, 2, 1):
            print(f"[place]   {k} ...", flush=True); spin(1.0)
        buf.clear(); rec[0] = True; print("[place]   measuring, hold still", flush=True); spin(a.seconds); rec[0] = False
        if len(buf) < 10:
            print(f"[place] only {len(buf)} samples - is the person tracked? measure again", flush=True); continue
        pel = np.mean([b[1] for b in buf], 0); ps = np.std([b[1] for b in buf], 0)
        hv = np.array([[np.cos(np.radians(b[2])), np.sin(np.radians(b[2]))] for b in buf]); mh = hv.mean(0)
        head = float(np.degrees(np.arctan2(mh[1], mh[0])))
        dev = np.degrees(np.arctan2(hv[:, 1] * np.cos(np.radians(head)) - hv[:, 0] * np.sin(np.radians(head)), hv @ np.array([np.cos(np.radians(head)), np.sin(np.radians(head))])))
        hstd = float(np.std(dev)); hmax = float(np.abs(dev).max())
        print(f"[place] pelvis world ({pel[0]:+.3f}, {pel[1]:+.3f}, {pel[2]:.3f}) std ({ps[0]*100:.1f}, {ps[1]*100:.1f}) cm; "
              f"heading {head:+.1f} deg (from world +x, CCW) std {hstd:.1f} deg, max dev {hmax:.1f} deg; {len(buf)} samples", flush=True)
        if hmax > a.max_heading_std:
            print(f"[place] FAIL: heading moved {hmax:.1f} deg > {a.max_heading_std:g} deg during the measurement - hold still and measure again", flush=True)
            continue
        e = evaluate(pel, head, hstd, cam)
        print(f"[place] camera: distance {e['cam_dist']:.2f} m, standing place {e['stand_angle']:+.1f} deg / {e['stand_range']:.2f} m, "
              f"reached hand {e['reach_angle']:+.1f} deg / {e['reach_range']:.2f} m (limit +-{HFOV_DEG/2-MARGIN_DEG:.1f} deg, {RANGE_M} m); "
              f"camera sees you from the {e['camera_side']} ({e['side_angle']:.0f} deg)", flush=True)
        print(f"[place] virtual robot: base world ({e['base'][0]:+.3f}, {e['base'][1]:+.3f}, {e['base'][2]:.2f}), yaw {e['theta']:+.1f} deg; "
              f"your pelvis -> base ({e['pelvis_base'][0]:.3f}, {e['pelvis_base'][1]:.3f}); effective mass rel. diff {e['meff_rel_err']:.1e}", flush=True)
        for w in e["warns"]:
            print(f"[place] WARNING: {w}", flush=True)
        if e["fails"] and not a.force:
            for f in e["fails"]:
                print(f"[place] FAIL: {f}", flush=True)
            print("[place] not saved - move and measure again (or run with --force)", flush=True)
            continue
        out = dict(frame="live_user", rotation_z_deg=float(e["theta"]), base_position_world=[float(v) for v in e["base"]],
                   measured=datetime.datetime.now().isoformat(timespec="seconds"), pelvis_world=[float(v) for v in pel], pelvis_std_m=[float(v) for v in ps],
                   heading_deg=head, heading_std_deg=hstd, heading_max_dev_deg=hmax, samples=len(buf), camera_pose=cam,
                   checks=dict(camera_distance_m=e["cam_dist"], stand_angle_deg=e["stand_angle"], stand_range_m=e["stand_range"], reach_angle_deg=e["reach_angle"],
                               reach_range_m=e["reach_range"], camera_side=e["camera_side"], side_angle_deg=e["side_angle"], fails=e["fails"], warnings=e["warns"],
                               forced=bool(e["fails"]), pelvis_in_base=[float(v) for v in e["pelvis_base"]], meff_rel_err=e["meff_rel_err"]),
                   stand_pelvis_base_xy=STAND_BASE.tolist(), reach_point_world=[float(v) for v in e["reach_world"]])
        import yaml
        out = json.loads(json.dumps(out, default=float))      # plain floats for yaml (numpy scalars)
        body = yaml.safe_dump(out, sort_keys=False)            # dump first, write the file only when it succeeded
        hdr = ("# live_frame_user.yml - written by lv_place.py: world -> Franka base for the place where the person stood facing the monitor.\n"
               "# p_base = Rz(rotation_z_deg)^T (p_world - base_position_world); the measured pelvis goes to base (1.05, -0.15), facing base -x.\n"
               "# Same relative layout as scene_live_front.json. live_frame.yml is not changed.\n")
        open(a.out, "w").write(hdr + body)
        R, t = L.load_frame(a.out); chk = R @ pel + t
        print(f"[place] PASS{' (forced)' if e['fails'] else ''}: saved {a.out}; reload check pelvis -> base ({chk[0]:.4f}, {chk[1]:.4f})", flush=True)
        n.destroy_node(); rclpy.shutdown(); return 0
    print("[place] no placement saved", flush=True); return 1


if __name__ == "__main__":
    sys.exit(main())

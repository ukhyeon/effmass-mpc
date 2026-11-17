"""b2a_scene_design.py - scene g2 placement and the three TCP lines (system python3; shapes v2 static pose).
Placement: g1 transform rotated about z so the human faces -x (toward the robot base at the origin); z offset as g1 (feet on the floor);
x/y offset so that the right-hand fingertip (support of the v2 right-hand shapes along the arm axis) is at (X_TIP, Y_HAND).
Lines are for the gripper TCP (hand link + 0.1034 m along its z). Arm axis a = elbow -> wrist (toward the robot).
  front: travel along y, passes in front of the fingertip, nearest direction = a
  top  : travel along y, passes over the finger-row capsule axis, nearest direction = +z
  side : travel along z (top -> bottom), passes beside the outward (pinky-side) end of the hand, nearest direction = horizontal normal
         (a line parallel to the common travel direction y cannot have its nearest direction along y, so side travels vertically)
Each line: gap to the v2 right-hand shapes = D0 at the closest point; checks gap to forearm >= D0 + 0.05, torso/shoulder/head >= 0.35.
Output: goal1/b2a/g2/temp_transform.json, goal1/b2a/g2/lines.json"""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import shape_metrics as SM
H = os.path.expanduser("~"); D = f"{H}/effmass_mpc/goal1/data"; OUT = f"{H}/effmass_mpc/goal1/b2a/g2"
X_TIP = float(os.environ.get("B2A_X_TIP", 0.50)); Y_HAND = float(os.environ.get("B2A_Y_HAND", 0.0)); D0 = 0.03; HALF = float(os.environ.get("B2A_HALF", 0.25))
Z = np.load(f"{D}/human_b2a_reach_v2.npz"); rep = json.load(open(f"{H}/effmass_mpc/goal1/b2a/pose_report.json"))
g1 = json.load(open(f"{H}/effmass_mpc/env_check/step4/temp_transform.json"))
fwd = np.array(rep["forward_node_world"]); th = np.arctan2(0.0, -1.0) - np.arctan2(fwd[1], fwd[0])
R = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1.0]]); t = np.array([0.0, 0.0, g1["t"][2]])
P0, P1, RAD, SEG = Z["p0"][0].astype(float), Z["p1"][0].astype(float), Z["shape_radius"].astype(float), Z["shape_segment"]
tw = lambda p: p @ R.T + t
p0, p1 = tw(P0), tw(P1); el, wr = tw(np.array(rep["elbow"])), tw(np.array(rep["wrist"])); a = wr - el; a /= np.linalg.norm(a)
HAND = [k for k in range(24) if SEG[k] == 18 and RAD[k] > 0]


def cap_pts(k, n=4000):
    return SM.cap_samples(p0[k], p1[k], RAD[k], n, np.random.default_rng(k))


hp = np.vstack([cap_pts(k) for k in HAND]); tip0 = hp[np.argmax(hp @ a)]
shift = np.array([X_TIP, Y_HAND, 0.0]) - np.array([tip0[0], tip0[1], 0.0]); t = t + shift
p0, p1 = tw(P0), tw(P1); el, wr = tw(np.array(rep["elbow"])), tw(np.array(rep["wrist"])); com = tw(np.array(rep["hand_com"])); sh = tw(np.array(rep["shoulder"]))
hp = np.vstack([cap_pts(k) for k in HAND]); tip = hp[np.argmax(hp @ a)]


def gap_line(c, d, ks):
    """min gap between the TCP segment c +- HALF*d and the capsules ks (dense sampling of the line)"""
    s = np.linspace(-HALF, HALF, 2001); L = c + np.outer(s, d)
    g = np.array([min(SM.segdist(L, p0[k], p1[k]) - RAD[k]) if False else (SM.segdist(L, p0[k], p1[k]) - RAD[k]).min() for k in ks])
    return float(g.min()), ks[int(np.argmin(g))]


def nearest_dir(c, d, ks):
    s = np.linspace(-HALF, HALF, 4001); L = c + np.outer(s, d); best = (1e9, None, None)
    for k in ks:
        ab = p1[k] - p0[k]; L2 = max(ab @ ab, 1e-12); tt = np.clip(((L - p0[k]) @ ab) / L2, 0, 1); Q = p0[k] + tt[:, None] * ab; dd = np.linalg.norm(L - Q, axis=1) - RAD[k]; j = int(np.argmin(dd))
        if dd[j] < best[0]:
            best = (dd[j], L[j], (L[j] - Q[j]) / np.linalg.norm(L[j] - Q[j]))
    return best


ey = np.array([0, 1.0, 0]); ez = np.array([0, 0, 1.0])
# hand shapes in this frame
fr_k = max(HAND, key=lambda k: RAD[k]); pr_k = [k for k in HAND if k != fr_k and RAD[k] > 0.01][0]; th_k = [k for k in HAND if RAD[k] <= 0.01][0]
fr_mid = 0.5 * (p0[fr_k] + p1[fr_k])
lines = {}
# front: line along y through the point D0 beyond the fingertip along a
c = tip + a * D0; lines["front"] = dict(center=c, dir=ey)
# top: line along y over the finger-row axis
c = np.array([fr_mid[0], fr_mid[1], 0.0]); zt = max((p0[k][2] if True else 0) for k in [fr_k]); c[2] = max(p0[fr_k][2], p1[fr_k][2]) + RAD[fr_k] + D0; lines["top"] = dict(center=c, dir=ey)
# side: vertical line beside the outward end of the hand (outward = +y side here: right hand, human faces -x, the person's right = +y)
right = np.cross(a, ez); right = right / np.linalg.norm(right)           # person right = forward (a) x up
ends = hp[np.argmax(hp @ right)]; c = ends + right * D0; lines["side"] = dict(center=c, dir=ez)
res = {}
for nm, L in lines.items():
    c, d = L["center"], L["dir"]
    # adjust the offset so that the gap to the hand shapes is exactly D0 (move along the intended nearest direction)
    nd = {"front": a, "top": ez, "side": right}[nm]
    for _ in range(30):
        g, _ = gap_line(c, d, HAND); c = c + nd * (D0 - g)
    g, kk = gap_line(c, d, HAND); gd, gp, gdir = nearest_dir(c, d, HAND)
    ang = lambda u, v: float(np.degrees(np.arccos(np.clip(abs(u @ v), -1, 1))))
    fa = [k for k in range(24) if SEG[k] == 16 and RAD[k] > 0]; tor = [k for k in range(24) if SEG[k] in (0, 3, 8, 9, 12) and RAD[k] > 0]
    res[nm] = dict(center=c.tolist(), dir=d.tolist(), A=(c - HALF * d).tolist(), B=(c + HALF * d).tolist(), gap_hand=g, nearest_hand_shape=int(kk), nearest_point=gp.tolist(),
                   nearest_dir=gdir.tolist(), nearest_dir_vs_arm_axis_deg=ang(gdir, a), nearest_dir_vs_vertical_deg=ang(gdir, ez), nearest_dir_vs_right_deg=ang(gdir, right),
                   gap_forearm=gap_line(c, d, fa)[0], gap_torso_neck_head=gap_line(c, d, tor)[0])
tf = dict(R=R.tolist(), t=t.tolist(), rule="p_isaac = R @ p_nodeworld + t ; v_isaac = R @ v_nodeworld | scene g2: human faces -x, right fingertip at (X_TIP, Y_HAND), z offset as g1",
          x_tip=X_TIP, y_hand=Y_HAND, yaw_deg=float(np.degrees(th)))
json.dump(tf, open(f"{OUT}/temp_transform.json", "w"), indent=1)
info = dict(fingertip=tip.tolist(), hand_com=com.tolist(), wrist=wr.tolist(), shoulder=sh.tolist(), arm_axis=a.tolist(), person_right=right.tolist(), D0=D0, half_length=HALF,
            fingertip_base_horizontal=float(np.hypot(tip[0], tip[1])), hand_height_above_base=float(com[2]), hand_shapes=HAND, finger_row=fr_k, palm_row=pr_k, thumb=th_k, lines=res)
json.dump(info, open(f"{OUT}/lines.json", "w"), indent=1)
print(json.dumps({k: v for k, v in info.items() if k != "lines"}, indent=0))
for nm, r in res.items():
    print(nm, {k: (np.round(v, 4).tolist() if isinstance(v, list) else round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})

"""b2a4_scene.py <front|top> <key> - B2a-4 line ends and scene json for one calibration candidate (system python via g1_run_human.sh py).
Ends: smallest symmetric half length h (1 cm grid) with all IK points within +-h successful and the robot (all MPC spheres, yml + 0.02) >= 0.23 m from
every v2 hand shape at both end poses; if none, each end separately (smallest |dy| per side with the same rule). Writes
goal1/b2a/g2/scene_b2a4_<path>_<key>.json (goals = hand-link targets, gripper down; q_start = IK at A; single pass A -> B, +y)."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import b2a2_halflen as HL
path, key = sys.argv[1], sys.argv[2]
IK = json.load(open(f"{BA.G}/b2a4_ik.json")); d = IK[path][key]; dy = np.array(IK["dy"]); ok = np.array(d["success"], bool); Q = np.array(d["q"]); c = np.array(d["center"])
gap = np.array([HL.gap_hand(q) if o else np.nan for q, o in zip(Q, ok)]); marg = np.min(np.minimum(Q - HL.LO, HL.HI - Q), 1)
i0 = int(np.argmin(np.abs(dy)))
good = lambda i: ok[i] and gap[i] >= 0.23
def contiguous(ia, ib):
    return bool(ok[min(ia, ib):max(ia, ib) + 1].all())
ia = ib = None; sym = False
for h in range(1, i0 + 1):
    a, b = i0 - h, i0 + h
    if good(a) and good(b) and contiguous(a, b):
        ia, ib, sym = a, b, True; break
if ia is None:
    ia = next((i0 - h for h in range(1, i0 + 1) if good(i0 - h) and contiguous(i0 - h, i0)), None)
    ib = next((i0 + h for h in range(1, len(dy) - i0) if good(i0 + h) and contiguous(i0, i0 + h)), None)
assert ia is not None and ib is not None, "no ends satisfy the rule"
tcp = lambda i: [c[0], c[1] + dy[i], c[2]]; hand = lambda i: [c[0], c[1] + dy[i], c[2] + 0.1034]; q4 = [0.0, 1.0, 0.0, 0.0]
js = dict(path=path, key=key, scene="b2a4", goals_wxyz=[hand(ia) + q4, hand(ib) + q4], q_start=Q[ia].tolist(), tcp_A=tcp(ia), tcp_B=tcp(ib), dy=[float(dy[ia]), float(dy[ib])], symmetric=sym,
          end_gap=[float(gap[ia]), float(gap[ib])], end_margin=[float(marg[ia]), float(marg[ib])], line_min_margin=float(marg[ia:ib + 1].min()), plates=[],
          note="B2a-4 calibration line; ends >= 0.23 m robot - hand shapes")
json.dump(js, open(f"{BA.G}/scene_b2a4_{path}_{key}.json", "w"), indent=1)
print(json.dumps({k: (np.round(v, 4).tolist() if isinstance(v, (list, float)) else v) for k, v in js.items() if k not in ("goals_wxyz", "q_start")}))

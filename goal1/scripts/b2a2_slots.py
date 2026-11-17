"""b2a2_slots.py - B2a-2 line ends and slot plates (system python via g1_run_human.sh py), checks before any run.
Ends: front A/B at dy -0.42/+0.42; top A/B at dy -0.42/+0.39 (asymmetric: the symmetric top line cannot keep >= 0.23 m and stay reachable).
Plates (cuRobo cuboids, 1 cm thick, axis aligned):
  slot-front: horizontal plates above and below the hand, x from the fingertip back toward the body, y +-0.30 around the hand.
  slot-top  : vertical plate in front of the fingertip (normal x) and vertical plate on the outward (+y) side (normal y), top edge 3 cm below the hand top.
Checks: (1) plate - v2 hand shape gap >= 0.22 m; (2) plate - every human shape gap > 0; (3) robot (MPC spheres) - plate gap >= 0.025 m on every IK pose of
the intended line; (4) info: robot - plate gap on the IK poses of the other line (the route the plates should block), at |dy| <= 0.12."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
H = os.path.expanduser("~"); IKX = json.load(open(f"{BA.G}/ik_ext.json")); L = json.load(open(f"{BA.G}/lines.json"))
ENDS = {"front": (-0.42, 0.42), "top": (-0.42, 0.39)}
ALLH = [k for k in range(24) if BA.RAD[k] > 0]
htop = max(max(BA.P0[k][2], BA.P1[k][2]) + BA.RAD[k] for k in BA.HAND); hbot = min(min(BA.P0[k][2], BA.P1[k][2]) - BA.RAD[k] for k in BA.HAND)
tipx = L["fingertip"][0]; hy = L["hand_com"][1]
ymax_hand = max(max(BA.P0[k][1], BA.P1[k][1]) + BA.RAD[k] for k in BA.HAND); xback = max(max(BA.P0[k][0], BA.P1[k][0]) + BA.RAD[k] for k in BA.HAND)
G = 0.22; T = 0.01
box = lambda name, lo, hi: dict(name=name, lo=np.array(lo, float), hi=np.array(hi, float))
XS = 0.06   # plates start 6 cm behind the fingertip: closer, the arm above the TCP on the front line touches the upper plate (scan in the report)
PL = {"slot_front": [box("plate_up", [tipx + XS, hy - 0.30, htop + G], [tipx + XS + 0.35, hy + 0.30, htop + G + T]),
                     box("plate_down", [tipx + XS, hy - 0.30, hbot - G - T], [tipx + XS + 0.35, hy + 0.30, hbot - G])],
      "slot_top": [box("plate_front", [tipx - G - 0.001 - T, hy - 0.30, hbot - G], [tipx - G - 0.001, hy + 0.30, htop - 0.03]),
                   box("plate_side", [tipx - G, ymax_hand + G, hbot - G], [xback, ymax_hand + G + T, htop - 0.03])]}


def pt_box(P, b):
    d = np.maximum(np.maximum(b["lo"] - P, 0), P - b["hi"]); out = np.linalg.norm(d, axis=-1)
    inside = np.all((P >= b["lo"]) & (P <= b["hi"]), axis=-1)
    return np.where(inside, -np.min(np.minimum(P - b["lo"], b["hi"] - P), axis=-1), out)


def cap_box(k, b, n=400):
    t = np.linspace(0, 1, n)[:, None]; A = BA.P0[k] + t * (BA.P1[k] - BA.P0[k]); return float(pt_box(A, b).min() - BA.RAD[k])


def robot_box(q, b):
    C = BA.RI.compute(np.array(q))[0]; return float((pt_box(C, b) - BA.RS).min())


res = {"ends": {}, "plates": {}}
for nm, (ya, yb) in ENDS.items():
    d = IKX[nm]; dy = np.array(d["dy"]); ia, ib = int(np.argmin(np.abs(dy - ya))), int(np.argmin(np.abs(dy - yb)))
    res["ends"][nm] = dict(dy=[ya, yb], A_tcp=d["tcp"][ia], B_tcp=d["tcp"][ib], hand_A=d["hand_target"][ia], hand_B=d["hand_target"][ib], q_A=d["q"][ia], ia=ia, ib=ib,
                           all_ik_ok=bool(np.all(np.array(d["success"])[ia:ib + 1])))
for sc, plates in PL.items():
    line = "front" if sc == "slot_front" else "top"; other = "top" if line == "front" else "front"; e = res["ends"][line]
    r = dict(plates=[dict(name=b["name"], center=((b["lo"] + b["hi"]) / 2).tolist(), dims=(b["hi"] - b["lo"]).tolist()) for b in plates], checks={})
    for b in plates:
        c1 = min(cap_box(k, b) for k in BA.HAND); c2 = min(cap_box(k, b) for k in ALLH)
        qs = IKX[line]["q"][e["ia"]:e["ib"] + 1]; c3 = min(robot_box(q, b) for q in qs)
        oth = IKX[other]; m = [i for i, y in enumerate(oth["dy"]) if abs(y) <= 0.12 and oth["success"][i]]; c4 = min(robot_box(oth["q"][i], b) for i in m)
        r["checks"][b["name"]] = dict(hand_gap=c1, human_gap=c2, robot_gap_intended_line=c3, robot_gap_other_line_window=c4,
                                      rule1=c1 >= 0.22 - 1e-9, rule2=c2 > 0, rule3=c3 >= 0.025)
    res["plates"][sc] = r
json.dump(res, open(f"{BA.G}/slots.json", "w"), indent=1, default=float)
print("hand top", round(htop, 4), "bottom", round(hbot, 4), "tip x", round(tipx, 4), "outer y", round(ymax_hand, 4), "back x", round(xback, 4))
for nm, e in res["ends"].items():
    print(nm, "A", np.round(e["A_tcp"], 3).tolist(), "B", np.round(e["B_tcp"], 3).tolist(), "all IK ok between", e["all_ik_ok"])
for sc, r in res["plates"].items():
    print("==", sc)
    for p_ in r["plates"]:
        print("  ", p_["name"], "center", np.round(p_["center"], 3).tolist(), "dims", np.round(p_["dims"], 3).tolist())
    for n_, c in r["checks"].items():
        print("  ", n_, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in c.items()})

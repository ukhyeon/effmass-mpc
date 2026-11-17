"""b2a2_halflen.py - choose the half length: smallest h (same for front and top) such that every IK point within +-h succeeds and the robot
(all MPC spheres, yml + 0.02) is >= 0.23 m (eta_max 0.20 + 0.03) from every v2 hand shape at both end points. Reports gaps and joint margins."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import xml.etree.ElementTree as ET
H = os.path.expanduser("~"); IKX = json.load(open(f"{BA.G}/ik_ext.json"))
root = ET.parse(f"{H}/effmass_mpc/goal1/fr3v2_1/fr3v2_1_curobo_narrow.urdf").getroot()
lim = {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper"))) for j in root.findall("joint") if j.get("type") == "revolute"}
LO = np.array([lim[f"fr3v2_1_joint{i}"][0] for i in range(1, 8)]); HI = np.array([lim[f"fr3v2_1_joint{i}"][1] for i in range(1, 8)])
ALLH = [k for k in range(24) if BA.RAD[k] > 0]


def gap_hand(q, ks=BA.HAND):
    C = BA.RI.compute(np.array(q))[0]
    return float(min((BA.SM.segdist(C, BA.P0[k], BA.P1[k]) - BA.RAD[k] - BA.RS).min() for k in ks))


G = {}
for nm, d in IKX.items():
    dy = np.array(d["dy"]); ok = np.array(d["success"], bool); q = np.array(d["q"])
    G[nm] = dict(dy=dy, ok=ok, q=q, gap=np.array([gap_hand(x) if o else np.nan for x, o in zip(q, ok)]), margin=np.min(np.minimum(q - LO, HI - q), 1))
best = None
for h in np.round(np.arange(0.20, 0.601, 0.01), 2):
    good = True; info = {}
    for nm, g in G.items():
        m = np.abs(g["dy"]) <= h + 1e-9
        ia, ib = int(np.argmin(np.abs(g["dy"] + h))), int(np.argmin(np.abs(g["dy"] - h)))
        if not g["ok"][m].all() or g["gap"][ia] < 0.23 or g["gap"][ib] < 0.23:
            good = False; break
        info[nm] = dict(gap_A=float(g["gap"][ia]), gap_B=float(g["gap"][ib]), margin_A=float(g["margin"][ia]), margin_B=float(g["margin"][ib]), min_margin_on_line=float(g["margin"][m].min()),
                        A_tcp=IKX[nm]["tcp"][ia], B_tcp=IKX[nm]["tcp"][ib], q_A=IKX[nm]["q"][ia], hand_A=IKX[nm]["hand_target"][ia], hand_B=IKX[nm]["hand_target"][ib], ia=ia, ib=ib)
    if good:
        best = (float(h), info); break
for nm, g in G.items():
    print(nm, "gap to hand shapes by |dy|:", {f"{y:+.2f}": round(float(v), 3) for y, v in zip(g["dy"], g["gap"]) if abs(round(y * 100)) % 5 == 0 and abs(y) >= 0.2}, "IK fail dy:", g["dy"][~g["ok"]].tolist())
print("chosen half length:", best[0] if best else None)
if best:
    print(json.dumps(best[1], indent=1)[:2500])
    json.dump(dict(half_length=best[0], lines=best[1]), open(f"{BA.G}/halflen.json", "w"), indent=1)

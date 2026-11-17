"""v1_v2cand.py - V2 scene candidates on the zone map (offline geometry only, no MPC; system python via g1_run_human.sh py).
TCP lines along +y (gripper down): front lines at hand height (IK grid b2a4_ik.json, x 0.20-0.51) and lines over the back of the hand / fingers
(x 0.50-0.62, heights above the hand top). For each line, point robot at the TCP (radius 0, as the zone map): which part of the ours zone it crosses
(hand / forearm / upper arm), deepest penetration min_s(d_s - eta_s), whether it also crosses the uniform-small zone, and on the over-hand lines which
zone is met first along +y. Ends: robot (all MPC spheres) gap to the hand shapes and to the forearm, with the rule eta_max + 0.03 = 0.28 m, on the IK grids;
joint margins. Output logs/meff_isaac/v1_v2cand.json and goal1/zone/v2_candidates.png."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import v1_zone_field as ZF
import b2a_analyze as BA
import b2a2_halflen as HL
import shape_metrics as SM
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
PART = {18: "hand", 16: "forearm", 14: "upper arm"}; ys = np.arange(-0.45, 0.4501, 0.005); HTOP = max(max(ZF.P0[k][2], ZF.P1[k][2]) + ZF.RAD[k] for k in ZF.SH if ZF.SEG[k] == 18)
FORE = [k for k in ZF.SH if ZF.SEG[k] == 16]


def line_info(x, z):
    P = np.stack([np.full_like(ys, x), ys, np.full_like(ys, z)], 1); fo, mu, kb = ZF.field(P, "ours"); fs, _, _ = ZF.field(P, "uniform_small")
    ins = fo <= 0; parts = sorted({PART[int(ZF.SEG[k])] for k in kb[ins]}) if ins.any() else []
    first = PART[int(ZF.SEG[kb[ins][0]])] if ins.any() else None
    return dict(x=x, z=z, ours_min=float(fo.min()), small_min=float(fs.min()), in_ours=bool(ins.any()), in_small=bool((fs <= 0).any()), parts=parts, first_part_along_y=first,
                ours_len_m=float(ins.sum() * 0.005), mu_at_deepest=float(mu[int(np.argmin(fo))]), y_entry=float(ys[ins][0]) if ins.any() else None)


res = {"front": [], "over": [], "ends": {}}
for x in np.round(np.arange(0.24, 0.52, 0.02), 3):
    res["front"].append(line_info(float(x), 0.4416))
for x in (0.50, 0.53, 0.546, 0.57, 0.60, 0.62):
    for dz in (0.03, 0.05, 0.07, 0.09, 0.11, 0.13, 0.15):
        res["over"].append(line_info(float(x), float(HTOP + dz)))
for r in res["front"]:
    print(f"front x {r['x']:.2f}: ours {r['in_ours']} ({r['ours_min']*100:6.1f} cm, parts {r['parts']}, mu {r['mu_at_deepest']:.2f}) small {r['in_small']} ({r['small_min']*100:6.1f} cm)")
for r in res["over"]:
    print(f"over x {r['x']:.3f} z {r['z']:.3f} (+{(r['z']-HTOP)*100:.0f} cm): ours {r['in_ours']} ({r['ours_min']*100:6.1f}) first {r['first_part_along_y']} parts {r['parts']} small {r['in_small']} ({r['small_min']*100:6.1f})")
IK = json.load(open(f"{BA.G}/b2a4_ik.json")); dy = np.array(IK["dy"])


def end_gaps(q):
    C = BA.RI.compute(np.array(q))[0]; gh = HL.gap_hand(q); gf = float(min((SM.segdist(C, BA.P0[k], BA.P1[k]) - BA.RAD[k] - BA.RS).min() for k in FORE)); return gh, gf


for path, keys in (("front", ["0.30", "0.32", "0.34", "0.36", "0.38", "0.40", "o050z5145", "o050z5345", "o050z5545", "o053z5145", "o053z5345", "o053z5545"]), ("top", ["0.475", "0.49", "0.50", "0.52", "0.54", "0.56", "0.58", "0.60"])):
    for kkey in keys:
        if kkey not in IK[path]:
            continue
        d = IK[path][kkey]; ok = np.array(d["success"], bool); Q = np.array(d["q"]); i0 = int(np.argmin(np.abs(dy)))
        def best(side):
            rng = range(i0 - 1, -1, -1) if side < 0 else range(i0 + 1, len(dy))
            last = None
            for i in rng:
                if not ok[i]:
                    break
                gh, gf = end_gaps(Q[i]); m = float(np.min(np.minimum(Q[i] - HL.LO, HL.HI - Q[i])))
                if gh >= 0.28 and gf >= 0.28 and last is None:
                    last = dict(dy=float(dy[i]), gap_hand=gh, gap_forearm=gf, margin=m)
            reach = dy[i0 - 1::-1][np.argmin(ok[i0 - 1::-1])] if side < 0 else None
            return last
        a, b = best(-1), best(1); cont = (int(np.argmax(~ok[:i0][::-1])) if (~ok[:i0]).any() else i0, int(np.argmax(~ok[i0:])) if (~ok[i0:]).any() else len(dy) - i0)
        res["ends"][f"{path}_{kkey}"] = dict(A=a, B=b, reach_dy=[float(-dy[i0 - cont[0]] if cont[0] else -0.6), float(dy[i0 + cont[1] - 1])], center=d["center"])
        print(f"ends {path} {kkey}: A {a} B {b} reach {res['ends'][f'{path}_{kkey}']['reach_dy']}")
json.dump(res, open(f"{BA.O}/v1_v2cand.json", "w"), indent=1, default=float)

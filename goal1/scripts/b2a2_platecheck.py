"""b2a2_platecheck.py - plate effect check (slot_front vs free_front, off and uni10, seeds 0 and 1) from saved records (g1_run_human.sh py).
Per run: arrival step, side (b2a_side.py result), TCP x / z at the hand-shape minimum in the pass window, max TCP z over the hand top, executed robot-plate
minimum gap (all MPC spheres, yml + 0.02; the plates of slot_front are evaluated for both scenes), and rollout plate counts (slot_front only, costagg)."""
import json, os, re, csv, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import b2a2_slots as SL
O = BA.O; side = {r["tag"]: r for r in json.load(open(f"{O}/b2a2_side_check.json"))}
HY = json.load(open(f"{BA.G}/lines.json"))["hand_com"][1]; PL = SL.PL["slot_front"]; out = []
for sc in ("free_front", "slot_front"):
    for c in ("off", "uni10"):
        for s in (0, 1):
            tag = f"b2a2_{sc}_{c}_s{s}"; r = BA.result(tag); rows = list(csv.DictReader(open(r["steps_csv"]))); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows])
            B = np.load(f"{O}/{tag}_b1.npz")["rows"]; tcp = B[:, 8:11] - np.array([0, 0, 0.1034]); w = np.abs(tcp[:, 1] - HY) <= 0.12; i = side[tag]["step"]
            Cs = [BA.RI.compute(q)[0] for q in Q]; pg = {b["name"]: min(float((SL.pt_box(C, b) - BA.RS).min()) for C in Cs) for b in PL}
            pl = {b["name"]: min(range(len(Cs)), key=lambda k: float((SL.pt_box(Cs[k], b) - BA.RS).min())) for b in PL}
            d = dict(tag=tag, arrival=(r["arrival_list"][0]["step"] if r["arrival_list"] else None), side=side[tag]["side"], u=side[tag]["u_vs_front_deg"], mu=side[tag]["mu"],
                     tcp_at_min=tcp[i].tolist(), tcp_max_above_hand_top=side[tag]["tcp_max_above_hand_top"], tcp_x_at_max_z=float(tcp[w][np.argmax(tcp[w, 2]), 0]),
                     exec_plate_gap=pg, exec_plate_gap_step=pl)
            ca = f"{O}/{tag}_costagg.npz"; z = np.load(ca)
            if "plate_n" in z.files:
                d["rollout_plate_steps"] = (z["plate_n"] > 0).sum(0).tolist(); d["rollout_plate_count"] = z["plate_n"].sum(0).tolist(); d["rollout_plate_min"] = z["plate_min"].min(0).tolist()
            d["rollout_human_steps"] = int((z["n"].sum(1) > 0).sum())
            out.append(d); print(json.dumps({k: (np.round(v, 3).tolist() if isinstance(v, (list, float)) else v) for k, v in d.items()}))
json.dump(out, open(f"{O}/b2a2_platecheck.json", "w"), indent=1)

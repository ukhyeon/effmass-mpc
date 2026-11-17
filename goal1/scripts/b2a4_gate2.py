"""b2a4_gate2.py - B2a-4 final gate (system python via g1_run_human.sh py): front-cal line x 0.31, height +1 cm (scene front_0.31h1); closest poses of
off seed 0 and seed 1; gate = mean of the two (mass A eta - iso eta) >= 0.04 m. top-cal: z 0.514 off seed 0 (B2a-2 probe, same scene).
eta_matched = mean(front-cal mass A eta averaged over the two seeds, top-cal mass A eta)."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import b2a4_gate as GT
res = {}
for nm, tag in (("front_s0", "b2a4_front_0.31h1_off_s0_cal"), ("front_s1", "b2a4_front_0.31h1_off_s1_cal"), ("top_s0", "b2a2_free_top_off_s0_probe")):
    m = json.load(open(f"{BA.O}/b2a4_cal/{tag}.json")); r = GT.at_pose(m["q_at_min"]); r.update(source_run=tag, step=m["step"], d_eta=r["eta_massA"] - r["eta_iso"]); res[nm] = r
    print(nm, json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items() if k != "u"}))
d = [res["front_s0"]["d_eta"], res["front_s1"]["d_eta"]]
res["gate"] = dict(front_d_eta_s0=d[0], front_d_eta_s1=d[1], front_d_eta_mean=float(np.mean(d)), passed=bool(np.mean(d) >= 0.04))
fa = float(np.mean([res["front_s0"]["eta_massA"], res["front_s1"]["eta_massA"]])); res["front_eta_massA_mean"] = fa
res["eta_matched"] = 0.5 * (fa + res["top_s0"]["eta_massA"])
json.dump(res, open(f"{BA.G}/b2a4_gate2.json", "w"), indent=1); print("gate", res["gate"], "front mass A eta mean", round(fa, 4), "eta_matched", round(res["eta_matched"], 4))

"""b2a5_gate.py - B2a-5 gate (system python via g1_run_human.sh py): closest poses of the off seed 0 and 1 calibration runs (profile t1) on
front-cal (x 0.38, height +1 cm) and top-cal (z 0.455). Gate: front mean (mass A eta - iso eta) >= 0.04 m. uniform_matched = (front mass A eta mean +
top mass A eta mean) / 2. Also: is the off gap smaller than each condition's eta (does every condition's cost switch on)."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import b2a4_gate as GT
res = {}
for sc, key in (("front", "front_0.38h1"), ("top", "top_0.455")):
    for s in (0, 1):
        tag = f"b2a5_{key}_off_s{s}_cal"; m = json.load(open(f"{BA.O}/b2a4_cal/{tag}.json")); r = GT.at_pose(m["q_at_min"]); r.update(source_run=tag, d_eta=r["eta_massA"] - r["eta_iso"])
        res[f"{sc}_s{s}"] = r
        print(f"{sc} s{s}", json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items() if k not in ("u", "source_run")}))
fA = np.mean([res["front_s0"]["eta_massA"], res["front_s1"]["eta_massA"]]); tA = np.mean([res["top_s0"]["eta_massA"], res["top_s1"]["eta_massA"]])
d = [res["front_s0"]["d_eta"], res["front_s1"]["d_eta"]]
res["gate"] = dict(front_d_eta_s0=d[0], front_d_eta_s1=d[1], front_d_eta_mean=float(np.mean(d)), passed=bool(np.mean(d) >= 0.04))
res["eta_matched"] = float((fA + tA) / 2); res["front_eta_massA_mean"] = float(fA); res["top_eta_massA_mean"] = float(tA)
res["cost_on_check"] = {f"{sc}_s{s}": dict(off_gap=res[f"{sc}_s{s}"]["gap"], below_eta_massA=res[f"{sc}_s{s}"]["gap"] < res[f"{sc}_s{s}"]["eta_massA"],
                                             below_eta_iso=res[f"{sc}_s{s}"]["gap"] < res[f"{sc}_s{s}"]["eta_iso"], below_eta_matched=res[f"{sc}_s{s}"]["gap"] < res["eta_matched"])
                        for sc in ("front", "top") for s in (0, 1)}
json.dump(res, open(f"{BA.O}/b2a5_gate.json", "w"), indent=1)
print("gate", res["gate"]); print("eta_matched", round(res["eta_matched"], 4), "front A", round(fA, 4), "top A", round(tA, 4)); print(json.dumps(res["cost_on_check"]))

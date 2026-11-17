"""v1_gain_table.py - gain-mode eta at the off closest poses of B2a-5 (front-cal, top-cal, seeds 0 and 1; system python via g1_run_human.sh py).
eta = clamp(d_base + k_gain * max(sqrt(mu / mu_ref) - 1, 0), d_base, eta_max), d_base 0.065, k_gain 0.10, mu_ref 0.53, eta_max 0.25; mu = personal,
nearest definition, robot m_r at the executed pose. If the front eta is outside 15-20 cm, the k_gain giving 17.5 cm is reported (not applied)."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import b2a4_gate as GT
DB, KG, MR, EM = 0.065, 0.10, 0.53, 0.25
eta = lambda mu, k=KG: float(np.clip(DB + k * max(np.sqrt(mu / MR) - 1, 0), DB, EM))
res = {}
for sc, key in (("front", "front_0.38h1"), ("top", "top_0.455")):
    for s in (0, 1):
        m = json.load(open(f"{BA.O}/b2a4_cal/b2a5_{key}_off_s{s}_cal.json")); r = GT.at_pose(m["q_at_min"]); mu = r["mu"]; x = np.sqrt(mu / MR)
        row = dict(sphere=r["sphere_link"], gap=r["gap"], u_vs_front_deg=r["u_vs_front_deg"], m_h=r["m_h"], m_r=r["m_r"], mu=mu, sqrt_mu_over_ref=float(x), eta_gain=eta(mu),
                   eta_setA=r["eta_massA"], eta_iso=r["eta_iso"])
        if sc == "front" and not (0.15 <= row["eta_gain"] <= 0.20):
            row["k_gain_for_17_5cm"] = float((0.175 - DB) / (x - 1)) if x > 1 else None
        res[f"{sc}_s{s}"] = row
        print(f"{sc} s{s}: sphere {row['sphere']:20s} u {row['u_vs_front_deg']:6.1f} m_h {row['m_h']:.2f} m_r {row['m_r']:.2f} mu {mu:.3f} sqrt(mu/mref) {x:.3f} eta_gain {row['eta_gain']*100:.1f} cm"
              + (f" k_gain for 17.5 cm {row['k_gain_for_17_5cm']:.3f}" if "k_gain_for_17_5cm" in row else ""))
json.dump(dict(params=dict(d_base=DB, k_gain=KG, mu_ref=MR, eta_max=EM), poses=res), open(f"{BA.O}/v1_gain_table.json", "w"), indent=1)

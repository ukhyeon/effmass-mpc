"""b2a_gate_alt.py - information only: gate deltas for alternative definitions at the same closest poses as b2a_gate.py (orientation 'down').
u definitions: com (current kernel) and closest (sphere centre -> closest point on the hand shape). Parameter sets A (p 0.5, eta 0.05-0.20) and
B (p 0.7, eta 0.04-0.20). m_r from the same pose and sphere."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import shape_metrics as SM
from effmass_robot.robot_inertia_np import RobotInertiaNP
H = os.path.expanduser("~"); G = f"{H}/effmass_mpc/goal1/b2a/g2"
gate = json.load(open(f"{G}/gate.json")); tf = json.load(open(f"{G}/temp_transform.json")); R, t = np.array(tf["R"]), np.array(tf["t"])
Z = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_v2.npz"); rad = Z["shape_radius"].astype(float)
p0 = Z["p0"][0].astype(float) @ R.T + t; p1 = Z["p1"][0].astype(float) @ R.T + t; COM = Z["com"][0][18].astype(float) @ R.T + t; Lh = R @ Z["inv"][0][18].astype(float) @ R.T
RI = RobotInertiaNP(rotor=True)
SETS = {"A": (0.10, 2.0, 0.5, 0.05, 0.20), "B": (0.10, 2.0, 0.7, 0.04, 0.20)}
out = {}
for dname in ("com", "closest"):
    for sn, (e0, mref, p, emin, emax) in SETS.items():
        row = {}
        for nm in ("front", "top", "side"):
            g = gate["down"][nm]; C, Lr, _ = RI.compute(np.array(g["q"])); c = C[g["sphere"]]; k = g["hand_shape"]
            if dname == "com":
                u = COM - c
            else:
                ab = p1[k] - p0[k]; tt = np.clip(((c - p0[k]) @ ab) / max(ab @ ab, 1e-12), 0, 1); u = p0[k] + tt * ab - c
            u /= np.linalg.norm(u); ah, ar = u @ Lh @ u, u @ Lr[g["sphere"]] @ u; mu = 1 / (ah + ar)
            row[nm] = dict(m_h=1 / ah, m_r=1 / ar, mu=mu, eta=float(np.clip(e0 * (mu / mref) ** p, emin, emax)))
        row["front_minus_top_cm"] = (row["front"]["eta"] - row["top"]["eta"]) * 100; row["front_minus_side_cm"] = (row["front"]["eta"] - row["side"]["eta"]) * 100
        out[f"{dname}/{sn}"] = row
        print(f"{dname:7s} set {sn}: " + "  ".join(f"{nm} m_h {row[nm]['m_h']:.2f} mu {row[nm]['mu']:.2f} eta {row[nm]['eta']:.3f}" for nm in ("front", "top", "side"))
              + f"  | front-top {row['front_minus_top_cm']:.1f} cm, front-side {row['front_minus_side_cm']:.1f} cm")
json.dump(out, open(f"{G}/gate_alt.json", "w"), indent=1)

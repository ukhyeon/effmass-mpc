"""b2a4_gate.py - B2a-4 offline gate at the closest moment of the calibrated off runs (system python via g1_run_human.sh py).
Pose = executed q at the minimum shape-based hand gap (b2a4_cal json); nearest robot sphere (MPC radius) to the v2 hand shapes; u = sphere centre ->
nearest point on that shape axis (COM fallback within 1 mm); m_h = 1/(u' Lh u), m_r = 1/(u' Lr u), mu, eta (set A); iso: m_h = 0.6 kg.
Gate: front-cal (eta mass A - eta iso) >= 0.04 m. eta_matched = mean of front-cal and top-cal mass A eta."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import shape_metrics as SM
A = dict(eta0=0.10, muref=2.0, p=0.5, emin=0.05, emax=0.20)
eta = lambda mu: float(np.clip(A["eta0"] * (mu / A["muref"]) ** A["p"], A["emin"], A["emax"]))


def at_pose(q):
    C, Lr, _ = BA.RI.compute(np.asarray(q, float))
    G_ = np.stack([SM.segdist(C, BA.P0[k], BA.P1[k]) - BA.RAD[k] for k in BA.HAND], 1) - BA.RS[:, None]; j, kk = np.unravel_index(np.argmin(G_), G_.shape); k = BA.HAND[kk]
    ab = BA.P1[k] - BA.P0[k]; t = np.clip(((C[j] - BA.P0[k]) @ ab) / max(ab @ ab, 1e-12), 0, 1); Qp = BA.P0[k] + t * ab; fb = bool(np.linalg.norm(Qp - C[j]) < 1e-3)
    u = (BA.COMS[18] if fb else Qp) - C[j]; u /= np.linalg.norm(u); ah = float(u @ BA.LH @ u); ar = float(u @ Lr[j] @ u)
    mu = 1 / (ah + ar); mui = 1 / (1 / 0.6 + ar)
    return dict(gap=float(G_[j, kk]), sphere=int(j), sphere_link=BA.RI.sphere_link[j], hand_shape=int(k), fallback=fb, u=u.tolist(), u_vs_front_deg=float(np.degrees(np.arccos(np.clip(u @ (-BA.AX), -1, 1)))),
                m_h=1 / ah, m_r=1 / ar, mu=mu, sqrt_mu_over_ref=float(np.sqrt(mu / A["muref"])), eta_massA=eta(mu), mu_iso=mui, eta_iso=eta(mui))


if __name__ == "__main__":
    res = {}
    for nm, tag in (("front_cal", "b2a4_front_0.31_off_s0_cal"), ("top_cal", "b2a2_free_top_off_s0_probe")):
        m = json.load(open(f"{BA.O}/b2a4_cal/{tag}.json")); r = at_pose(m["q_at_min"]); r["source_run"] = tag; r["step"] = m["step"]; res[nm] = r
        print(nm, json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items() if k != "u"}))
    res["gate"] = dict(front_massA_minus_iso=res["front_cal"]["eta_massA"] - res["front_cal"]["eta_iso"]); res["gate"]["passed"] = res["gate"]["front_massA_minus_iso"] >= 0.04
    res["eta_matched"] = 0.5 * (res["front_cal"]["eta_massA"] + res["top_cal"]["eta_massA"])
    json.dump(res, open(f"{BA.G}/b2a4_gate.json", "w"), indent=1); print("gate", res["gate"], "eta_matched", round(res["eta_matched"], 4))

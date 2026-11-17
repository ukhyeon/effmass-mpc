"""b2a4_cal.py <tag> [line_x] - B2a-4 calibration measures from one saved run (system python via g1_run_human.sh py).
Shape-based hand gap per step from the B1 rows (kernel distance, MPC sphere radius); minimum over the run; at that step: robot sphere, side (as b2a_side.py:
largest of up / down / front / side components of the direction from the closest hand-shape axis point to the sphere centre... using the shape surface),
u (nearest definition) vs frontal angle, m_h, m_r, mu (B1 record), TCP max height above the hand top inside the pass window, max TCP deviation from the line x,
arrival step."""
import json, os, re, csv, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
HY = json.load(open(f"{BA.G}/lines.json"))["hand_com"][1]; HTOP = max(max(BA.P0[k][2], BA.P1[k][2]) + BA.RAD[k] for k in BA.HAND)


def measure(tag, line_x=None):
    r = BA.result(tag); rows = list(csv.DictReader(open(r["steps_csv"]))); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows])
    B = np.load(f"{BA.O}/{tag}_b1.npz")["rows"]; tcp = B[:, 8:11] - np.array([0, 0, 0.1034]); w = np.abs(tcp[:, 1] - HY) <= 0.12
    D = B[:, 12:36][:, BA.HAND]; i, jj = np.unravel_index(int(np.argmin(D)), D.shape); k = BA.HAND[jj]; s = int(B[i, 36 + k]); c = BA.RI.compute(Q[i])[0][s]
    ab = BA.P1[k] - BA.P0[k]; t = np.clip(((c - BA.P0[k]) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = BA.P0[k] + t * ab; n = (c - q) / np.linalg.norm(c - q)
    comp = {"up": float(n[2]), "down": float(-n[2]), "front": float(n @ BA.AX), "side": float(abs(n[1]))}; side = max(comp, key=comp.get)
    u, _, _ = BA.shape_u(c)
    arr = r.get("arrival_list", [])
    return dict(tag=tag, min_gap=float(D[i, jj]), step=int(i), sphere=BA.RI.sphere_link[s], hand_shape=int(k), side=side, comp={a: round(b, 3) for a, b in comp.items()},
                u_vs_front_deg=float(np.degrees(np.arccos(np.clip(u @ (-BA.AX), -1, 1)))), m_h=float(B[i, 84 + k]), m_r=float(B[i, 108 + k]), mu=float(B[i, 60 + k]),
                tcp_at_min=tcp[i].round(4).tolist(), tcp_max_above_hand_top=float(tcp[w, 2].max() - HTOP) if w.any() else None,
                tcp_x_in_window=[float(tcp[w, 0].min()), float(tcp[w, 0].max())] if w.any() else None,
                max_dev_x=float(np.abs(tcp[:, 0] - line_x).max()) if line_x is not None else None,
                arrival_step=(arr[0]["step"] if arr else None), infeasible=int(r["infeasible_steps"]), steps=len(Q), q_at_min=Q[i].tolist())


if __name__ == "__main__":
    m = measure(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else None)
    print(json.dumps({k: v for k, v in m.items() if k != "q_at_min"}))
    os.makedirs(f"{BA.O}/b2a4_cal", exist_ok=True); json.dump(m, open(f"{BA.O}/b2a4_cal/{sys.argv[1]}.json", "w"), indent=1)

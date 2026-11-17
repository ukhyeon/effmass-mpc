"""b2a3_design.py - B2a-3 design and offline gate (system python via g1_run_human.sh py). The robot holds one pose (b2a3_ik.json, x 0.47);
the static v2 human (B2a pose, scene g2 transform) is translated rigidly:
  front: along the arm axis a (toward the robot), lateral offset so that the fingertip line passes the TCP (y = 0);
  side : first along a so that the right-hand COM is level with the TCP along a, then along person_right (+y = pinky / hand-edge side, thumb is at -y).
Stop: shape-based gap (v2 hand shapes vs all MPC spheres, yml + 0.02) = 0 with the robot at the hold pose. Start: the closest point before the stop,
on the 0.2/30 m frame grid, with gap >= 0.30 m. Speed 0.2 m/s. Gate point: gap = 0.03 m. Gate: front (eta mass A - eta iso) >= 0.04 m.
Writes goal1/b2a/g2/b2a3_design.json and scene_b2a3.json; with 'write' also the precomputed files data/human_b2a3_<scene>_v2.npz and _verts.npz."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import shape_metrics as SM
H = os.path.expanduser("~"); G = BA.G; D_ = f"{H}/effmass_mpc/goal1/data"
L = json.load(open(f"{G}/lines.json")); IKT = {round(d["x"], 2): d for d in json.load(open(f"{G}/b2a3_ik.json"))}; X_T = 0.47; P = IKT[X_T]
q = np.array(P["q"]); C, Lr, _ = BA.RI.compute(q); RS = BA.RS; T = np.array(P["tcp"])
AX = BA.AX / np.linalg.norm(BA.AX); RIGHT = np.array(L["person_right"], float); RIGHT /= np.linalg.norm(RIGHT)
V = 0.2; RATE = 30.0; DF = V / RATE; HOLD = 2.0; SPARE = 1.0
ALL = [k for k in range(24) if BA.RAD[k] > 0]; TORSO = [k for k in ALL if BA.SEG[k] in (0, 3, 8)]; HEAD = [k for k in ALL if BA.SEG[k] == 12]
A = dict(eta0=0.10, muref=2.0, p=0.5, emin=0.05, emax=0.20); M_ISO = 0.6
eta = lambda mu: float(np.clip(A["eta0"] * (mu / A["muref"]) ** A["p"], A["emin"], A["emax"]))


def gap(Dv, ks):
    G_ = np.stack([SM.segdist(C, BA.P0[k] + Dv, BA.P1[k] + Dv) - BA.RAD[k] for k in ks], 1) - RS[:, None]
    j, kk = np.unravel_index(np.argmin(G_), G_.shape); return float(G_[j, kk]), int(j), ks[kk]


def root(f, lo, hi, target):
    """f decreasing in s on [lo, hi], f(lo) > target >= f(hi): bisection"""
    for _ in range(60):
        m = 0.5 * (lo + hi)
        lo, hi = (m, hi) if f(m) > target else (lo, m)
    return 0.5 * (lo + hi)


def scene(name, D0, d):
    f = lambda s: gap(D0 + s * d, BA.HAND)[0]
    ss = np.arange(-1.0, 0.6, 0.002); gs = np.array([f(s) for s in ss]); i = int(np.argmax(gs <= 0)); assert gs[i] <= 0 and np.all(gs[:i] > 0)
    s_stop = root(f, ss[i - 1], ss[i], 0.0); s_gate = root(f, ss[0], s_stop, 0.03)
    N = 1
    while f(s_stop - N * DF) < 0.30:
        N += 1
    mono = bool(np.all(np.diff([f(s_stop - k * DF) for k in range(N + 1)]) > 0))
    Dst, Dsp, Dg = D0 + (s_stop - N * DF) * d, D0 + s_stop * d, D0 + s_gate * d
    g, j, k = gap(Dg, BA.HAND)
    ab = BA.P1[k] - BA.P0[k]; Pk0 = BA.P0[k] + Dg; tt = np.clip(((C[j] - Pk0) @ ab) / max(ab @ ab, 1e-12), 0, 1); Qp = Pk0 + tt * ab
    fb = bool(np.linalg.norm(Qp - C[j]) < 1e-3); u = ((BA.COMS[18] + Dg) if fb else Qp) - C[j]; u /= np.linalg.norm(u)
    ah = float(u @ BA.LH @ u); ar = float(u @ Lr[j] @ u); mh, mr = 1 / ah, 1 / ar; mu = 1 / (ah + ar); mui = 1 / (1 / M_ISO + ar)
    chk = {}
    for tag_, Dv in (("start", Dst), ("stop", Dsp)):
        chk[tag_] = dict(hand=gap(Dv, BA.HAND)[0], torso=gap(Dv, TORSO)[0], head=gap(Dv, HEAD)[0], all_but_hand=gap(Dv, [k_ for k_ in ALL if k_ not in BA.HAND]))
        chk[tag_]["all_but_hand"] = dict(gap=chk[tag_]["all_but_hand"][0], sphere=BA.RI.sphere_link[chk[tag_]["all_but_hand"][1]], shape=chk[tag_]["all_but_hand"][2], seg=int(BA.SEG[chk[tag_]["all_but_hand"][2]]))
    t_app = N / RATE; steps = int(round((t_app + HOLD + SPARE) * 60))
    return dict(scene=name, dir=d.tolist(), D0=D0.tolist(), D_start=Dst.tolist(), D_stop=Dsp.tolist(), travel_m=N * DF, frames_approach=N, approach_s=t_app, hold_s=HOLD, spare_s=SPARE,
                steps=steps, gap_monotonic_on_approach=mono, checks=chk, rule_start_hand=chk["start"]["hand"] >= 0.30,
                rule_torso_head=all(chk[x][y] >= 0.35 for x in ("start", "stop") for y in ("torso", "head")),
                gate=dict(D=Dg.tolist(), shape_gap=g, sphere=j, sphere_link=BA.RI.sphere_link[j], hand_shape=int(k), fallback=fb, u=u.tolist(),
                          u_vs_arm_axis_deg=float(np.degrees(np.arccos(np.clip(u @ (-AX), -1, 1)))), m_h=mh, m_r=mr, mu=mu, eta_massA=eta(mu), mu_iso=mui, eta_iso=eta(mui)))


tip = np.array(L["fingertip"]); com = BA.COMS[18]
res = dict(hold=dict(tcp=T.tolist(), hand_target=P["hand_target"], q=P["q"], joint_margins=P["margins"], min_margin=P["min_margin"], worst_joint=P["worst_joint"],
                     hand_com_height=float(L["hand_height_above_base"])), speed=V, rate=RATE, scenes={})
res["scenes"]["front"] = scene("front", np.array([0.0, T[1] - tip[1], 0.0]), AX)
res["scenes"]["side"] = scene("side", ((T - com) @ AX) * AX, RIGHT)
F, S_ = res["scenes"]["front"]["gate"], res["scenes"]["side"]["gate"]
res["gate"] = dict(front_massA_minus_iso=F["eta_massA"] - F["eta_iso"], passed=F["eta_massA"] - F["eta_iso"] >= 0.04, side_massA_minus_iso=S_["eta_massA"] - S_["eta_iso"],
                   eta_matched=0.5 * (F["eta_massA"] + S_["eta_massA"]))
json.dump(res, open(f"{G}/b2a3_design.json", "w"), indent=1)
json.dump(dict(path="hold", scene="b2a3", goals_wxyz=[P["hand_target"] + [0.0, 1.0, 0.0, 0.0]] * 2, q_start=P["q"], tcp=P["tcp"], plates=[],
               note="B2a-3: hold one pose (start = goal, single pass to the same goal); human moves (precomputed)"), open(f"{G}/scene_b2a3.json", "w"), indent=1)
print("hold TCP", np.round(T, 4).tolist(), "min joint margin", round(P["min_margin"], 3), "j", P["worst_joint"])
for nm, s in res["scenes"].items():
    g = s["gate"]
    print(f"== {nm}: travel {s['travel_m']:.4f} m, frames {s['frames_approach']}, approach {s['approach_s']:.3f} s, steps {s['steps']}, gap monotonic {s['gap_monotonic_on_approach']}")
    for t_, c in s["checks"].items():
        print(f"   {t_}: hand {c['hand']:.4f} torso {c['torso']:.4f} head {c['head']:.4f} other {c['all_but_hand']}")
    print(f"   gate: sphere {g['sphere_link']}({g['sphere']}) shape {g['hand_shape']} gap {g['shape_gap']:.4f} u-vs-axis {g['u_vs_arm_axis_deg']:.1f} m_h {g['m_h']:.3f} m_r {g['m_r']:.3f} mu {g['mu']:.3f} "
          f"eta mass A {g['eta_massA']:.4f} iso mu {g['mu_iso']:.3f} eta {g['eta_iso']:.4f} fallback {g['fallback']}")
print("gate", res["gate"])

if len(sys.argv) > 1 and sys.argv[1] == "write":
    tf = json.load(open(f"{G}/temp_transform.json")); Rm = np.array(tf["R"])
    Z = np.load(f"{D_}/human_b2a_reach_v2.npz"); MV = np.load(f"{D_}/human_b2a_reach_verts.npz")
    for nm, s in res["scenes"].items():
        d = np.array(s["dir"]); N = s["frames_approach"]; Fn = N + int(round((HOLD + SPARE) * RATE)) + 2
        lam = np.minimum(np.arange(Fn), N) * DF; Dw = np.array(s["D_start"])[None] + lam[:, None] * d[None]        # Isaac frame translation per frame
        Dn = Dw @ Rm                                                                                                  # node frame: R^T D
        vn = np.where((np.arange(Fn) < N)[:, None], (V * d) @ Rm, 0.0)
        out = {k: Z[k] for k in Z.files if Z[k].ndim == 0 or Z[k].shape[0] != 1 or k in ("mass",)}
        for k in ("com", "jprox", "jdist"):
            out[k] = (Z[k][0][None].astype(np.float64) + Dn[:, None, :]).astype(np.float32)
        for k in ("p0", "p1"):
            out[k] = (Z[k][0][None].astype(np.float64) + Dn[:, None, :]).astype(np.float32)
        out["vel"] = np.repeat(vn[:, None, :], 19, 1).astype(np.float32)
        out["rot"] = np.repeat(Z["rot"], Fn, 0); out["inv"] = np.repeat(Z["inv"], Fn, 0); out["pose66"] = np.repeat(Z["pose66"], Fn, 0)
        out["input_time"] = np.arange(Fn) / RATE; out["frame"] = np.arange(Fn); out["vdt"] = np.full(Fn, 1 / RATE, np.float32); out["vvalid"] = np.ones(Fn, bool)
        out["compute_ms"] = np.zeros(Fn); out["rate"] = np.array(RATE)
        out["b2a3"] = np.array(json.dumps(dict(scene=nm, dir=s["dir"], D_start=s["D_start"], D_stop=s["D_stop"], frames_approach=N, speed=V, translation_isaac=Dw.tolist())))
        for k in Z.files:
            if k not in out:
                out[k] = Z[k]
        np.savez(f"{D_}/human_b2a3_{nm}_v2.npz", **out)
        np.savez(f"{D_}/human_b2a3_{nm}_verts.npz", verts=(MV["verts"][0][None].astype(np.float64) + Dn[:, None, :]).astype(np.float32), faces=MV["faces"], source=np.array(f"b2a3 {nm}: B2a verts translated"))
        print("wrote", nm, Fn, "frames")

"""on_wp4_motion.py - WP4 moving collaboration motions (system python via g1_run_human.sh py; no MPC). Same method as r1_motion.py: key poses every
1 cm of the fingertip path, right shoulder (3) + elbow hinge (1) + wrist (3) optimised with the node body model (fingertip on target dominant, palm
orientation and forearm direction of the reference pose kept softly, continuity), per-frame poses interpolated between key poses, frames recomputed with
the node code (shapes v2, velocity from the previous frame), then the whole motion placed by one rigid yaw + translation (Isaac frame).
M1 hand-over: hold pose = WP2 P6_handover placed by the WP3 'tipfront' search (target gap 0.025 m to the off path, torso >= 0.25 m since 0.35 m is infeasible); retracted = fingertip moved back along
the reach direction (shoulder -> fingertip) by D = smallest of 0.30, 0.32, ... m with every hand shape >= 0.35 m from the off-path robot cloud.
Cycle rest 1.5 s -> reach (min-jerk) -> hold 3.0 s -> retract -> ... Labels rest / reaching / hold / retracting (reference event: hold onset).
M2 crossing: P7_down (forearm toward the work table) fingertip swept sideways (human right-left) by 0.40 m with the sweep midpoint on the robot TCP line
(x 0.38 + 0.04, y 0, z 0.47); yaw chosen as the largest crossing angle to the path (90, 80, ... 30 deg) with torso shapes >= 0.35 m from the off-path
cloud in every key pose. Cycle rest 1.5 s -> sweep -> rest 1.5 s -> sweep back. Labels rest / cross_in / cross_out (cross_out onset = fingertip on the
TCP line, the reference event). Peak fingertip speed 0.25 / 0.5 m/s (one rescale from the measured peak), 60 s, 30 fps.
Outputs goal1/data/human_on4_<M>_<v>_v2.npz (+ _v2_armverts.npz, _verts.npz), logs/meff_isaac/on_wp4_motion_check.json."""
import json, os, sys, time
import numpy as np, torch
from scipy.optimize import minimize
torch.set_num_threads(1)
sys.path.insert(0, os.path.dirname(__file__))
import on_wp2_postures as WP
import on_wp3_scenes as W3
import shape_metrics as SM
H = os.path.expanduser("~"); D_ = WP.D_; O = WP.O; RATE = 30.0; Ri, ti = WP.Ri, WP.ti; nz = WP.nz; ang = WP.ang
mj = lambda tau: 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
X = {n: np.array(p["x"]) for n, p in json.load(open(f"{O}/on_wp2_postures.json")).items()}


def keyposes(xref, targets):
    g0 = WP.fk(xref); keys = [xref.copy()]
    for tg in targets[1:]:
        xp = keys[-1]
        def obj(x):
            g = WP.fk(x); return 1e4 * np.sum((g["tip"] - tg) ** 2) + 0.5 * ang(g["palm"], g0["palm"]) ** 2 + 0.3 * ang(g["wr"] - g["el"], g0["wr"] - g0["el"]) ** 2 + 0.05 * np.sum((x - xp) ** 2) + 10 * min(x[3], 0) ** 2
        keys.append(minimize(obj, xp, method="Nelder-Mead", options=dict(xatol=1e-6, fatol=1e-10, maxiter=6000, maxfev=6000)).x)
    return np.array(keys)


def place_fn(yaw, piv, dst):
    R = W3.Rz(yaw); return R, (lambda P: (P - piv) @ R.T + dst)


def torso_and_hand_gap(o, fI):
    sh = o["shapes"]; P0 = np.asarray(sh["p0"]) @ Ri.T + ti; P1 = np.asarray(sh["p1"]) @ Ri.T + ti; P0, P1 = fI(P0), fI(P1); rad = np.asarray(sh["radius"]); seg = np.asarray(sh["segment"])
    g = lambda ks: min((SM.segdist(W3.CL, P0[k], P1[k]) - rad[k] - W3.CR).min() for k in ks)
    return g([k for k in range(24) if rad[k] > 0 and seg[k] in W3.TORSO]), g([k for k in range(24) if rad[k] > 0 and seg[k] == 18])


def frames(KX, KS, s_of_t, labels, R, fI, tag):
    x_of = lambda s: np.array([np.interp(s, KS, KX[:, j]) for j in range(7)]); NF = len(s_of_t)
    RN = Ri.T @ R @ Ri; toN = lambda P: (P - ti) @ Ri
    A = {k: [] for k in ("com", "vel", "rot", "inv", "jprox", "jdist", "p0", "p1", "vdt", "vvalid", "compute_ms")}; prev = None; poses = []; VF = []; VA = []; tips = []
    LAB = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy"); ARMV = np.nonzero(np.isin(LAB, [14, 16, 18]))[0]
    FACES = np.asarray(WP.core.smplh.faces, np.int64); ARMF = FACES[np.isin(FACES, ARMV).all(1)]; remap = -np.ones(6890, int); remap[ARMV] = np.arange(len(ARMV))
    mv = lambda P: toN(fI(np.asarray(P, np.float64) @ Ri.T + ti))
    for k in range(NF):
        p = WP.pose_of(x_of(s_of_t[k])); poses.append(p.reshape(-1)); tk = time.perf_counter()
        o = WP.core.compute(p.reshape(-1), WP.T0, prev=prev, dt=(1 / RATE if prev is not None else None)); sh = o["shapes"]
        A["com"].append(mv(WP.f64(o["com"][0]))); A["vel"].append(WP.f64(o["vel"][0]) @ RN.T); A["rot"].append(np.einsum("ij,sjk->sik", RN, WP.f64(o["world_rot"][0])))
        A["inv"].append(np.einsum("ij,sjk,lk->sil", RN, WP.f64(o["inv_inertia"][0]), RN)); A["jprox"].append(mv(WP.f64(o["joints"][0]))); A["jdist"].append(mv(np.asarray(o["distal"], np.float64)))
        A["p0"].append(mv(sh["p0"])); A["p1"].append(mv(sh["p1"])); A["vdt"].append(1 / RATE if (prev is not None and o["vel_valid"]) else 0.0); A["vvalid"].append(bool(o["vel_valid"]))
        A["compute_ms"].append((time.perf_counter() - tk) * 1e3); tips.append(fI(np.asarray(o["distal"], float)[18] @ Ri.T + ti))
        V = mv(WP.verts(p.reshape(-1)) @ Ri.T + ti) if False else toN(fI(WP.verts(p.reshape(-1)) @ Ri.T + ti)); VF.append(V.astype(np.float16)); VA.append(V[ARMV].astype(np.float32)); prev = o
    ref = np.load(f"{D_}/human_b2a_reach_v2.npz"); out = f"{D_}/human_on4_{tag}_v2.npz"
    np.savez(out, input_time=np.arange(NF) / RATE, frame=np.arange(NF), mass=ref["mass"], shape_type=ref["shape_type"], shape_segment=ref["shape_segment"], shape_radius=ref["shape_radius"],
             segment_names=ref["segment_names"], rate=RATE, betas=np.zeros(10, np.float32), source_npz="on_wp4_motion.py", vdt_from="", shapes_version="v2", shape_active=ref["shape_active"],
             pose66=np.array(poses, np.float32), phase=np.array(labels, dtype="U10"), s_path=np.asarray(s_of_t, np.float32), tip=np.array([toN(t_) for t_ in tips], np.float32),
             **{k: (np.asarray(vv, np.float32) if k not in ("vvalid", "compute_ms") else np.asarray(vv)) for k, vv in A.items()})
    np.savez(f"{D_}/human_on4_{tag}_verts.npz", verts=np.array(VF), faces=FACES, source=np.array("on_wp4_motion.py node body model, float16"))
    np.savez(f"{D_}/human_on4_{tag}_v2_armverts.npz", verts=np.array(VA), vid=ARMV, faces_local=remap[ARMF])
    tips = np.array(tips); vt = np.linalg.norm(np.diff(tips, axis=0), axis=1) * RATE
    return out, tips, vt


def schedule_m1(D, Tr, n, rest=1.5, hold=3.0):
    P = rest + Tr + hold + Tr; t = np.arange(n) / RATE; ph = np.mod(t, P); s = np.empty(n); st = np.empty(n, object)
    for k, tt in enumerate(ph):
        if tt < rest:
            s[k], st[k] = D, "rest"
        elif tt < rest + Tr:
            s[k], st[k] = D * (1 - mj((tt - rest) / Tr)), "reaching"
        elif tt < rest + Tr + hold:
            s[k], st[k] = 0.0, "hold"
        else:
            s[k], st[k] = D * mj((tt - rest - Tr - hold) / Tr), "retracting"
    return s, st, P


def schedule_m2(A, Tr, n, rest=1.5):
    P = 2 * (rest + Tr); t = np.arange(n) / RATE; ph = np.mod(t, P); s = np.empty(n); st = np.empty(n, object)
    for k, tt in enumerate(ph):
        if tt < rest:
            s[k], st[k] = 0.0, "rest"
        elif tt < rest + Tr:
            s[k] = A * mj((tt - rest) / Tr); st[k] = "cross_in" if s[k] < A / 2 else "cross_out"
        elif tt < 2 * rest + Tr:
            s[k], st[k] = A, "rest"
        else:
            s[k] = A * (1 - mj((tt - 2 * rest - Tr) / Tr)); st[k] = "cross_in" if s[k] > A / 2 else "cross_out"
    return s, st, P


if __name__ == "__main__":
    which = sys.argv[1:] or ["M1", "M2"]; NF = int(60 * RATE) + 2; ck = json.load(open(f"{O}/on_wp4_motion_check.json")) if os.path.exists(f"{O}/on_wp4_motion_check.json") else {}
    if "M1" in which:
        info, M = W3.search("P6_handover", "tipfront", 0.025, torso_min=0.25)   # WP3 torso rule 0.35 is infeasible for P6 (D4-1)
        assert info is not None, "no tipfront placement for P6_handover"
        R, fI = place_fn(np.radians(info["yaw_deg"]), np.array(info["anchor"]), np.array(info["dst"]))
        xh = X["P6_handover"]; g0 = WP.fk(xh); tip0 = g0["tip"]; u = nz(tip0 - g0["sh"])
        targets = [tip0 - 0.01 * i * u for i in range(0, 46)]; KX = keyposes(xh, targets); KS = 0.01 * np.arange(len(KX)); D = None
        for i in range(30, 46, 2):
            tg, hg = torso_and_hand_gap(WP.core.compute(WP.pose_of(KX[i]).reshape(-1), WP.T0, prev=None, dt=None), fI)
            print(f"M1 s {0.01*i:.2f} hand gap {hg:.3f} torso {tg:.3f}", flush=True)
            if hg >= 0.35:
                D = 0.01 * i; break
        assert D is not None, "M1: no retract distance <= 0.45 m"
        KX, KS = KX[: int(round(D / 0.01)) + 1], KS[: int(round(D / 0.01)) + 1]
        for v in (0.25, 0.5):
            Tr = 1.875 * D / v
            for it in range(2):
                s, st, Pc = schedule_m1(D, Tr, int((1.5 + Tr + 0.2) * RATE)); tp = np.array([WP.fk(np.array([np.interp(sv, KS, KX[:, j]) for j in range(7)]))["tip"] for sv in s])
                if it == 0:
                    Tr *= (np.linalg.norm(np.diff(tp, axis=0), axis=1) * RATE).max() / v
            s, st, Pc = schedule_m1(D, Tr, NF); tag = f"M1_v{int(round(v * 100)):03d}"; out, tips, vt = frames(KX, KS, s, st, R, fI, tag)
            dev = np.linalg.norm(np.cross(tips - fI(tip0[None])[0], R @ u), axis=1)
            ck[tag] = dict(file=out, D_retract_m=D, reach_time_s=Tr, period_s=Pc, rest_s=1.5, hold_s=3.0, tip_peak_speed=float(vt.max()), tip_line_dev_max_m=float(dev.max()), placement=info, frames=NF)
            print(tag, {k: (round(x_, 4) if isinstance(x_, float) else x_) for k, x_ in ck[tag].items() if k != "placement"}, flush=True)
    if "M2" in which:
        xr = X["P7_down"]; g0 = WP.fk(xr); lat = np.array([0, 1.0, 0]); A = 0.40
        targets = [g0["tip"] - (A / 2) * lat + 0.01 * i * lat for i in range(0, 41)]
        KX = keyposes(xr, [g0["tip"]] + [g0["tip"] - 0.01 * i * lat for i in range(1, 21)]); KX = KX[::-1]
        KX2 = keyposes(xr, [g0["tip"]] + [g0["tip"] + 0.01 * i * lat for i in range(1, 21)]); KX = np.concatenate([KX, KX2[1:]]); KS = 0.01 * np.arange(len(KX))
        tipC = g0["tip"]; chosen = None; TORSO_M2 = 0.35
        for TORSO_M2 in (0.35, 0.25):
            for th in (90, 80, 70, 60, 50, 40, 30):
                # sweep direction lat (human right) mapped to a direction at angle th from the robot path (+y): yaw so that R @ lat = (sin th, cos th, 0) ... toward the robot base side first
                yaw = np.arctan2(np.cos(np.radians(th)), -np.sin(np.radians(th))) - np.arctan2(lat[1], lat[0])
                for sgn in (1, -1):
                    yw = yaw if sgn == 1 else yaw + np.pi
                    R, fI = place_fn(yw, tipC, np.array([0.42, 0.0, 0.47]))
                    tg = min(torso_and_hand_gap(WP.core.compute(WP.pose_of(KX[i]).reshape(-1), WP.T0, prev=None, dt=None), fI)[0] for i in (0, 20, 40))
                    print(f"M2 angle {th} side {sgn} torso gap {tg:.3f}", flush=True)
                    if tg >= TORSO_M2:
                        chosen = (th, yw, R, fI, tg); break
                if chosen:
                    break
            if chosen:
                break
        assert chosen is not None, "M2: no crossing angle with torso >= 0.25 m"
        th, yw, R, fI, tg = chosen
        for v in (0.25, 0.5):
            Tr = 1.875 * A / v
            for it in range(2):
                s, st, Pc = schedule_m2(A, Tr, int((1.5 + Tr + 0.2) * RATE)); tp = np.array([WP.fk(np.array([np.interp(sv, KS, KX[:, j]) for j in range(7)]))["tip"] for sv in s])
                if it == 0:
                    Tr *= (np.linalg.norm(np.diff(tp, axis=0), axis=1) * RATE).max() / v
            s, st, Pc = schedule_m2(A, Tr, NF); tag = f"M2_v{int(round(v * 100)):03d}"; out, tips, vt = frames(KX, KS, s, st, R, fI, tag)
            ck[tag] = dict(file=out, torso_min_used=TORSO_M2, sweep_m=A, crossing_angle_deg=th, yaw_deg=float(np.degrees(yw)), torso_gap_min_keyposes_m=tg, sweep_time_s=Tr, period_s=Pc, rest_s=1.5,
                           tip_peak_speed=float(vt.max()), tip_z_range=[float(tips[:, 2].min()), float(tips[:, 2].max())], frames=NF)
            print(tag, {k: (round(x_, 4) if isinstance(x_, float) else x_) for k, x_ in ck[tag].items()}, flush=True)
    json.dump(ck, open(f"{O}/on_wp4_motion_check.json", "w"), indent=1, default=lambda o: o.item() if hasattr(o, "item") else str(o))

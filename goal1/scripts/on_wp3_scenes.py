"""on_wp3_scenes.py - WP3 static scenes (system python via g1_run_human.sh py; no MPC). The robot path stays the front scene (scene_b2a4_front_0.38h1.json,
TCP A -> B along y at x 0.38, z 0.45); each posture (WP2 file, node code) is placed by a rigid yaw rotation about the vertical axis through an anchor
point plus a translation (applied to the precomputed arrays: positions, shapes, inertia rotated, velocity 0; exact for a rigid move).
Placement search against the robot sphere cloud of the off run on the front scene (b2a5_front_0.38h1_off_s0, every 2nd step, MPC sphere radius + 0.02):
  tipfront    : target hand shapes, robot path crosses in front of the fingertip: closest-point normal (human -> robot) . hand pointing >= 0.7,
                closest hand point within 4 cm of the fingertip; shift along +x (human side) to reach the target gap.
  top         : target hand shapes, robot passes over the hand: normal . z >= 0.7; shift along -z.
  forearmside : target forearm shapes, robot passes beside the forearm: forearm gap <= hand gap + 0.005 and <= upper-arm gap, |normal . z| <= 0.6; shift along +x.
Common: target gap 0.025 m (off path), torso shapes (pelvis, abdomen, chest) >= 0.35 m from the cloud, every other shape >= target gap.
Grid: yaw 0..350 deg step 10, anchor y in {-0.2, -0.1, 0, 0.1, 0.2}, anchor height / x per direction. Score = alignment - |y| (prefer the path middle).
Usage: on_wp3_scenes.py <posture> [...] [--gap=0.025] [--only=<dir>] [--adjust=<scene>:<dx>] -> goal1/data/human_on3_<P>_<dir>_v2.npz (+ _v2_armverts; display mesh human_on3_<P>_<dir>_verts.npz),
logs/meff_isaac/on_wp3_scenes.json (appended per scene)."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import shape_metrics as SM
H = os.path.expanduser("~"); D_ = f"{H}/effmass_mpc/goal1/data"; O = BA.O; Rm, tv = BA.Rm, BA.tv
LAB = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy"); ARMV = np.nonzero(np.isin(LAB, [14, 16, 18]))[0]
PO = json.load(open(f"{O}/on_wp2_postures.json"))
r = BA.result("b2a5_front_0.38h1_off_s0"); import csv
Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])[::2]
CL = np.concatenate([BA.RI.compute(q)[0] for q in Q]); CR = np.tile(BA.RS, len(Q))
TORSO = (0, 3, 8); nz = lambda v: v / np.linalg.norm(v)


def Rz(a):
    c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def load(name):
    z = np.load(PO[name]["file"]); d = {k: z[k] for k in z.files}
    I = dict(p0=z["p0"][0].astype(float) @ Rm.T + tv, p1=z["p1"][0].astype(float) @ Rm.T + tv, com=z["com"][0].astype(float) @ Rm.T + tv,
             jd=z["jdist"][0].astype(float) @ Rm.T + tv, jp=z["jprox"][0].astype(float) @ Rm.T + tv, rad=z["shape_radius"].astype(float), seg=z["shape_segment"])
    M = np.load(f"{D_}/human_on_{name}_mesh.npz"); I["V"] = M["verts"].astype(float); I["F"] = M["faces"]; return d, I


def moved(I, yaw, piv, dst):
    R = Rz(yaw); f = lambda P: (P - piv) @ R.T + dst
    return dict(p0=f(I["p0"]), p1=f(I["p1"]), com=f(I["com"]), jd=f(I["jd"]), jp=f(I["jp"]), rad=I["rad"], seg=I["seg"], R=R, piv=piv, dst=dst)


def shape_gaps(M):
    out = {}
    for k in range(len(M["rad"])):
        if M["rad"][k] <= 0:
            continue
        d = SM.segdist(CL, M["p0"][k], M["p1"][k]) - M["rad"][k] - CR; j = int(np.argmin(d)); out[k] = (float(d[j]), j)
    return out


def closest_normal(M, k, j):
    a, b = M["p0"][k], M["p1"][k]; ab = b - a; c = CL[j]; t = np.clip(((c - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = a + t * ab; return nz(c - q), q + nz(c - q) * M["rad"][k]


def evaluate(I, name, dirn, yaw, anchor_fn, place, gap):
    """place(anchor) -> (dst base, shift axis). Bisection on the shift so that the target gap = gap."""
    a0 = anchor_fn(I); base, ax = place
    def at(s):
        return moved(I, yaw, a0, base + s * ax)
    tgt = [k for k in range(len(I["rad"])) if I["rad"][k] > 0 and I["seg"][k] in ((18,) if dirn != "forearmside" else (16,))]
    g = lambda M: min(shape_gaps_k(M, k)[0] for k in tgt)
    lo, hi = -0.3, 0.6
    if g(at(hi)) < gap:
        return None
    for _ in range(26):
        mid = (lo + hi) / 2
        if g(at(mid)) < gap:
            lo = mid
        else:
            hi = mid
    M = at(hi); G = shape_gaps(M); kt = min(tgt, key=lambda k: G[k][0]); n, cp = closest_normal(M, kt, G[kt][1])
    hand = [k for k in G if M["seg"][k] == 18]; fore = [k for k in G if M["seg"][k] == 16]; up = [k for k in G if M["seg"][k] == 14]
    gh, gf, gu = min(G[k][0] for k in hand), min(G[k][0] for k in fore), min(G[k][0] for k in up)
    gt = min(G[k][0] for k in G if M["seg"][k] in TORSO); go = min(G[k][0] for k in G if M["seg"][k] not in (14, 16, 18))
    tip = M["jd"][18]; wr = M["jp"][18]; pt = nz(tip - wr)
    if dirn == "tipfront":
        al = float(n @ pt); ok = al >= 0.7 and np.linalg.norm(cp - tip) < 0.04
    elif dirn == "top":
        al = float(n[2]); ok = al >= 0.7
    else:
        al = 1 - abs(float(n[2])); ok = gf <= gh + 0.005 and gf <= gu and abs(n[2]) <= 0.6
    ok = ok and gt >= 0.35 and go >= gap - 1e-3 and min(gh, gf, gu) >= gap - 1e-3
    return dict(ok=bool(ok), align=al, yaw_deg=float(np.degrees(yaw)), shift=float(hi), anchor=a0.tolist(), dst=(base + hi * ax).tolist(), gap_hand=gh, gap_forearm=gf, gap_upper=gu,
                gap_torso=gt, gap_other=go, closest_seg=int(M["seg"][kt]), normal=n.round(3).tolist()), M


_cache = {}
def shape_gaps_k(M, k):
    d = SM.segdist(CL, M["p0"][k], M["p1"][k]) - M["rad"][k] - CR; j = int(np.argmin(d)); return float(d[j]), j


def search(name, dirn, gap):
    d, I = load(name); tip = I["jd"][18]; wr = I["jp"][18]; hc = I["com"][18]; el = I["jp"][16]; pt = nz(tip - wr); fa = nz(wr - el)
    best = None
    if dirn == "tipfront":
        h = np.array([pt[0], pt[1], 0.0])
        if np.linalg.norm(h) < 0.3:
            return None, None
        y0s = [0.0, -0.1, 0.1, -0.2, 0.2]; base_yaw = np.arctan2(-0.0, -1.0) - np.arctan2(h[1], h[0]); yaws = [base_yaw + np.radians(dd) for dd in (0, -15, 15, -30, 30)]
        cands = [(yw, (lambda I_: I_["jd"][18]), (np.array([0.38, y0, zq]), np.array([1.0, 0, 0]))) for yw in yaws for y0 in y0s for zq in (0.50, 0.46, 0.55)]
    elif dirn == "top":
        h = np.array([pt[0], pt[1], 0.0]); base_yaw = (np.arctan2(0.0, -1.0) - np.arctan2(h[1], h[0])) if np.linalg.norm(h) > 0.3 else 0.0
        yaws = [base_yaw + np.radians(dd) for dd in (0, -20, 20, -40, 40)]
        anc = [lambda I_: I_["com"][18], lambda I_: (I_["com"][18] + I_["jd"][18]) / 2, lambda I_: I_["jd"][18]]
        cands = [(yw, af, (np.array([x0, y0, 0.45]), np.array([0, 0, -1.0]))) for yw in yaws for af in anc for x0 in (0.38, 0.42) for y0 in (0.0, -0.1, 0.1, -0.2, 0.2)]
    else:
        cands = [(np.radians(yd), (lambda I_: (I_["jp"][16] + I_["jp"][18]) / 2), (np.array([0.38, y0, zq]), np.array([1.0, 0, 0])))
                 for yd in range(0, 360, 10) for y0 in (0.0, -0.1, 0.1, -0.2, 0.2) for zq in (0.50, 0.45, 0.55)]
    for yw, af, pl in cands:
        res = evaluate(I, name, dirn, yw, af, pl, gap)
        if res is None:
            continue
        info, M = res
        if not info["ok"]:
            continue
        sc = info["align"] - abs(info["dst"][1]) + 0.2 * min(info["gap_torso"] - 0.35, 0.2)
        if best is None or sc > best[0]:
            best = (sc, info, M)
    return (best[1], best[2]) if best else (None, None)


def write(name, dirn, info, M, gap, tag):
    d, I = load(name); R = M["R"]; piv = np.array(info["anchor"]); dst = np.array(info["dst"])
    fI = lambda P: (P - piv) @ R.T + dst; toN = lambda P: (P - tv) @ Rm                # Isaac -> node: p_n = R^T (p_I - t)
    RN = Rm.T @ R @ Rm                                                                    # rotation in node coordinates
    out = dict(d)
    for k in ("com", "jprox", "jdist", "p0", "p1"):
        A = d[k].astype(float); out[k] = toN(fI(A @ Rm.T + tv)).astype(d[k].dtype)
    out["vel"] = np.zeros_like(d["vel"]); out["inv"] = np.einsum("ij,fsjk,lk->fsil", RN, d["inv"].astype(float), RN).astype(d["inv"].dtype)
    if "rot" in d:
        out["rot"] = np.einsum("ij,fsjk->fsik", RN, d["rot"].astype(float)).astype(d["rot"].dtype)
    out["tip"] = out["jdist"][:, 18].astype(np.float32); out["phase"] = np.array(["hold"] * len(out["com"]), dtype="U10")
    f = f"{D_}/human_on3_{tag}_v2.npz"; np.savez(f, **out)
    V = fI(I["V"]); VN = toN(V)
    remap = -np.ones(len(V), int); remap[ARMV] = np.arange(len(ARMV)); FA = I["F"][np.isin(I["F"], ARMV).all(1)]
    np.savez(f"{D_}/human_on3_{tag}_v2_armverts.npz", verts=VN[ARMV][None].astype(np.float32), vid=ARMV, faces_local=remap[FA])
    np.savez(f"{D_}/human_on3_{tag}_verts.npz", verts=VN[None].astype(np.float16), faces=I["F"])
    return f


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]; opt = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--"))
    gap = float(opt.get("gap", 0.025)); dirs = [opt["only"]] if "only" in opt else ["tipfront", "top", "forearmside"]
    J = f"{O}/on_wp3_scenes.json"; allr = json.load(open(J)) if os.path.exists(J) else {}
    for name in args:
        for dn in dirs:
            tag = f"{name.split('_')[0]}_{dn}"; info, M = search(name, dn, gap)
            if info is None:
                allr[tag] = dict(posture=name, direction=dn, feasible=False, gap_target=gap); print(tag, "no feasible placement", flush=True)
            else:
                f = write(name, dn, info, M, gap, tag); allr[tag] = dict(posture=name, direction=dn, feasible=True, file=f, gap_target=gap, **info)
                print(tag, json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in info.items() if k not in ("anchor",)}), flush=True)
            json.dump(allr, open(J, "w"), indent=1)

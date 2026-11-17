"""v2_zone_lib.py - zone field for any static precomputed human (same definitions as v1_zone.py): load(npz) gives the right-arm v2 shapes in the
scene g2 frame; field(D, P, cond, k) = min_s (d_s - eta_s) with the arg-min shape mu / index; thick(D, origin, dir, cond, k) = normal-direction
thickness: from the surface point of the given shapes that is farthest along dir (support point; the outward normal there is dir for a capsule),
march along dir until the point leaves the zone."""
import json, os
import numpy as np
H = os.path.expanduser("~"); G = f"{H}/effmass_mpc/goal1/b2a/g2"
tf = json.load(open(f"{G}/temp_transform.json")); R, t = np.array(tf["R"]), np.array(tf["t"])
DB, MR, EM, MROB = 0.065, 0.53, 0.25, 4.6


def load(npz):
    Z = np.load(npz); SEG = Z["shape_segment"]; RAD = Z["shape_radius"].astype(float)
    D = dict(SEG=SEG, RAD=RAD, P0=Z["p0"][0].astype(float) @ R.T + t, P1=Z["p1"][0].astype(float) @ R.T + t, COM=Z["com"][0].astype(float) @ R.T + t,
             LAM={g: R @ Z["inv"][0][g].astype(float) @ R.T for g in (14, 16, 18)}, JP=Z["jprox"][0].astype(float) @ R.T + t, JD=Z["jdist"][0].astype(float) @ R.T + t)
    D["SH"] = [k for k in range(len(RAD)) if RAD[k] > 0 and SEG[k] in (14, 16, 18)]; D["HAND"] = [k for k in D["SH"] if SEG[k] == 18]
    return D


def eta_of(mu, cond, k):
    if cond == "uniform":
        return np.full_like(mu, DB)
    return np.clip(DB + k * np.maximum(np.sqrt(mu / MR) - 1, 0), DB, EM)


def field(D, P, cond="ours", k=0.10, shapes=None):
    fb = np.full(len(P), np.inf); mub = np.zeros(len(P)); kb = np.full(len(P), -1)
    for s in (shapes or D["SH"]):
        a, b = D["P0"][s], D["P1"][s]; ab = b - a; tt = np.clip(((P - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = a + tt[:, None] * ab; v = q - P; n = np.linalg.norm(v, axis=1)
        d = n - D["RAD"][s]; u = v / np.maximum(n, 1e-9)[:, None]; mh = 1.0 / np.maximum(np.einsum("ni,ij,nj->n", u, D["LAM"][D["SEG"][s]], u), 1e-12); mu = 1.0 / (1.0 / mh + 1.0 / MROB)
        f = d - eta_of(mu, cond, k); m = f < fb; fb[m] = f[m]; mub[m] = mu[m]; kb[m] = s
    return fb, mub, kb


def support(D, dirv, shapes):
    best = None
    for s in shapes:
        for e in (D["P0"][s], D["P1"][s]):
            p = e + D["RAD"][s] * dirv; v = float(p @ dirv)
            if best is None or v > best[0]:
                best = (v, p, s)
    return best[1], best[2]


def thick(D, dirv, cond="ours", k=0.10, shapes=None):
    dirv = np.asarray(dirv, float) / np.linalg.norm(dirv); p0, s = support(D, dirv, shapes or D["HAND"]); rr = np.arange(0, 0.6, 0.0005)
    f, mu, kb = field(D, p0[None] + rr[:, None] * dirv[None], cond, k); i = int(np.argmax(f > 0)); return float(rr[i]), p0, int(s)


def load_frame(npz, f):
    """same as load() for frame f of a multi-frame precomputed file"""
    Z = np.load(npz); SEG = Z["shape_segment"]; RAD = Z["shape_radius"].astype(float)
    D = dict(SEG=SEG, RAD=RAD, P0=Z["p0"][f].astype(float) @ R.T + t, P1=Z["p1"][f].astype(float) @ R.T + t, COM=Z["com"][f].astype(float) @ R.T + t,
             LAM={g: R @ Z["inv"][f][g].astype(float) @ R.T for g in (14, 16, 18)})
    D["SH"] = [k for k in range(len(RAD)) if RAD[k] > 0 and SEG[k] in (14, 16, 18)]; D["HAND"] = [k for k in D["SH"] if SEG[k] == 18]
    return D

"""b2a_torso_alt.py - torso capsule layout proposals (report only, nothing is changed). Same metrics as b2a_shape_check.py.
Body axes from the static pose: up = +z, right = forward x up (horizontal).
  current : 2 vertical capsules per segment (pelvis, abdomen, chest), split left/right (node layout), 6 capsules
  alt1    : 1 horizontal capsule per segment along the right axis through the vertex centroid, 3 capsules
  alt2    : 2 horizontal capsules per segment along the right axis, vertices split at the median height, 6 capsules
  alt3    : alt2 for chest and abdomen, 1 horizontal capsule for pelvis, 5 capsules
Capsule fit = node _axis_capsule rule: axis through the centroid, end points from the projection range moved in by r, r = p95 distance."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import b2a_shape_check as C
rep = C.rep; up = np.array([0, 0, 1.0]); fwd = np.array(rep["forward_node_world"]); right = np.cross(fwd, up); right /= np.linalg.norm(right)


def axis_capsule(P, u):
    c = P.mean(0); proj = (P - c) @ u; lo, hi = float(proj.min()), float(proj.max()); r = float(np.percentile(np.linalg.norm((P - c) - np.outer(proj, u), axis=1), 95))
    for _ in range(6):
        a_, b_ = lo + r, hi - r
        if a_ > b_:
            a_ = b_ = 0.5 * (lo + hi)
        a, b = c + u * a_, c + u * b_; r = float(np.percentile(C.segdist(P, a, b), 95))
    return a, b, r


def layout(kind):
    caps = {}
    for s in (0, 3, 8):
        P = C.V[C.LAB == s]
        if kind == "current":
            caps[s] = [(C.P0[k], C.P1[k], C.RAD[k]) for k in np.nonzero(C.SSEG == s)[0]]
        elif kind == "alt1" or (kind == "alt3" and s == 0):
            caps[s] = [axis_capsule(P, right)]
        else:
            zm = np.median(P[:, 2]); caps[s] = [axis_capsule(P[P[:, 2] >= zm], right), axis_capsule(P[P[:, 2] < zm], right)]
    return caps


res = {}
for kind in ("current", "alt1", "alt2", "alt3"):
    caps = layout(kind); r = {"n_capsules": sum(len(v) for v in caps.values())}
    for s, cs in caps.items():
        r[C.NAMES[s]] = dict(radius=[round(c[2], 4) for c in cs], length=[round(float(np.linalg.norm(c[1] - c[0])), 4) for c in cs],
                             under=C.under(C.V[C.LAB == s], cs), over=C.over(cs, seed=100 + s))
    res[kind] = r; print(kind, json.dumps(r), flush=True)
json.dump(res, open(f"{C.OUT}/torso_alt.json", "w"), indent=1)

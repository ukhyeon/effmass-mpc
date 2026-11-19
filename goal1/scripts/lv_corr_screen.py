"""lv_corr_screen.py <tag> <run_tag> <r0> <z_tcp> [motion ...] - live3m corrected paths, screening (no Isaac): places the reference
posture of a motion (median of the held window) against the robot trajectory of a human-cost-off run on the mirror-symmetric line
(robot frame: A = (r0, +0.30, z), B = (r0, -0.30, z)). The human is turned about the vertical so that the target direction d points at
the robot (-x) - on the real run the robot base is moved instead (same up to the turn of joint 1) - and placed with
  L    = horizontal distance from the TCP line to the reference point along d (m)
  zrel = TCP line height minus reference point height (m)
For each zrel: L solved so that the smaller of the two per-direction gaps is 3 cm; per direction (A->B, B->A): min gap over all
shapes, the shape and robot link of the closest pair, the angle between the direction to the robot and d."""
import sys
import numpy as np
import lv_corr_lib as CL


def passes(Q, arr, A, B):
    u = (B - A) / np.linalg.norm(B - A); Ln = np.linalg.norm(B - A); C = np.stack([CL.spheres(q) for q in Q]); s = np.array([(CL.tcp_of(q) - A) @ u for q in Q])
    cuts = [0] + arr + [len(Q)]; out = []
    for i, (a, b) in enumerate(zip(cuts[:-1], cuts[1:])):
        if i == 0 or b - a < 30 or (i == len(cuts) - 2 and b == len(Q)):
            continue   # first pass (start transient) and the unfinished last one left out
        idx = [k for k in range(a, b) if CL.END <= s[k] <= Ln - CL.END]
        if idx:
            out.append(("A->B" if i % 2 == 0 else "B->A", C[idx], (b - a) / 60.0))
    return out


def place(H, m, p0, p1, r0, zc, Lh, zrel):
    ref, k, d = H.tref(m, p0, p1); dh = np.array([d[0], d[1], 0.0]); dh /= np.linalg.norm(dh); th = np.arctan2(dh[1], dh[0]) - np.pi
    c, s = np.cos(-th), np.sin(-th); Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]); off = np.array([r0 + Lh, 0.0, zc - zrel])
    return (p0 - ref) @ Rz.T + off, (p1 - ref) @ Rz.T + off, Rz @ d


def evaluate(H, m, PS, q0, q1, d):
    res = []
    for nm, C, T in PS:
        best = None
        for Ci in C:
            g = CL.nearest(Ci, q0, q1, H.rad, H.act)
            if best is None or g[0] < best[0]:
                best = g
        res.append((nm, best[0], CL.ang(best[3], d), int(H.seg[best[2]]), CL.LINK[best[1]].replace("fr3v2_1_", ""), T, best[3]))
    return res


def main(tag, run, r0, zc, mots):
    H = CL.Human(tag); Q, arr, r = CL.load_run(run); A, B = np.array([r0, 0.3, zc]), np.array([r0, -0.3, zc]); PS = passes(Q, arr, A, B)
    print(run, "passes used", [(p[0], round(p[2], 2)) for p in PS])
    for q in mots:
        m = H.motion(q); win, p0, p1 = H.held(m); print(m["name"], m["target"], "held frames", win, "%.1f s" % ((win[1] - win[0] + 1) / 30.0))
        for zrel in np.arange(-0.30, 0.0501, 0.025):
            lo, hi = 0.0, 0.5
            for _ in range(22):
                Lh = 0.5 * (lo + hi); q0, q1, d = place(H, m, p0, p1, r0, zc, Lh, zrel); g = min(e[1] for e in evaluate(H, m, PS, q0, q1, d))
                lo, hi = (Lh, hi) if g < 0.03 else (lo, Lh)
            q0, q1, d = place(H, m, p0, p1, r0, zc, Lh, zrel); ev = evaluate(H, m, PS, q0, q1, d)
            by = {}
            for e in ev:
                by.setdefault(e[0], []).append(e)
            txt = "; ".join(f"{k} " + " ".join(f"{100 * e[1]:.1f}cm/{e[2]:.0f}deg/seg{e[3]}/{e[4]}" for e in v) for k, v in by.items())
            print(f"  zrel {zrel:+.3f} L {Lh:.3f}  {txt}")


if __name__ == "__main__":
    a = sys.argv; main(a[1], a[2], float(a[3]), float(a[4]), [int(x) - 1 for x in a[5:]] or [0, 1, 2])

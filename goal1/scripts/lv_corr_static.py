"""lv_corr_static.py <tag> <motion> <ref_path_json> <tcp_z_min> <tcp_z_max> - live3m corrected paths: best case of a horizontal pass in front
of the target for the held reference posture, with the arm raised about the shoulder by 0, 3, .. 15 deg (rigid turn of the upper arm,
forearm and hand shapes about the upper-arm capsule start, about the horizontal axis perpendicular to shoulder -> reference point;
elbow angle unchanged). No Isaac and no MPC: the robot is the gripper-down end of the arm only (spheres of link 6, link 7, hand and
fingers at their offsets from the TCP, taken from the IK pose q_centre of <ref_path_json>; the hand orientation is fixed in the world,
so these offsets do not depend on the base), moved on the straight line u through the path centre, |s| <= 0.20 m (the 0.10 m at both
ends of the 0.60 m line left out). This is the most favourable case: no path bulge, upper links ignored.
For each raise angle and each path height in the allowed range (TCP z, 1 cm steps): L solved so that the gap to the target shapes is
3 cm; then the gap to every other shape (min, which segment), and the angle of the closest target pair to d. A height is feasible when
the other shapes are farther than the target (>= 3 cm) and the angle is <= 30 deg. Prints per raise angle: fingertip / elbow reference
point rise, the feasible heights, and the best non-feasible height (largest gap to the other shapes with the angle <= 30 deg)."""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lv_corr_lib as CL
import shape_metrics as SM


def raise_arm(H, m, p0, p1, deg):
    S = p0[m["ua"]].copy(); ref = H.tref(m, p0, p1)[0]; hz = np.array([ref[0] - S[0], ref[1] - S[1], 0.0]); hz /= np.linalg.norm(hz); ax = np.cross(hz, [0, 0, 1.0])   # turn that lifts the reference point
    th = np.radians(deg); K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]]); R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
    q0, q1 = p0.copy(), p1.copy()
    for k in H.act:
        if H.seg[k] in CL.ARM[m["side"]]:
            q0[k] = S + R @ (p0[k] - S); q1[k] = S + R @ (p1[k] - S)
    return q0, q1


def main(tag, q, ref_json, zmin, zmax):
    H = CL.Human(tag); m = H.motion(q - 1); win, p0, p1 = H.held(m); qc = np.array(json.load(open(ref_json))["q_centre"]); C0 = CL.spheres(qc); tcp = CL.tcp_of(qc)
    sel = [j for j in range(len(C0)) if any(x in CL.LINK[j] for x in ("link6", "link7", "hand", "finger"))]; OFF = C0[sel] - tcp; RSs = CL.RS[sel]
    tk = sorted({c[2] for c in H.tpoints(m, p0, p1)}); other = [k for k in H.act if k not in tk and not (m["target"] == "elbow" and H.seg[k] in CL.ARM[m["side"]])]
    ss = np.arange(-0.20, 0.2001, 0.01); out = []
    for deg in (0, 3, 6, 9, 12, 15):
        a0, a1 = raise_arm(H, m, p0, p1, deg); ref, kref, d = H.tref(m, a0, a1); ref0 = H.tref(m, p0, p1)[0]; dh = np.array([d[0], d[1], 0.0]); dh /= np.linalg.norm(dh); u = np.cross([0, 0, 1.0], dh); rows = []
        for z in np.arange(zmin, zmax + 1e-9, 0.01):
            def gaps(Lh):
                c = np.array([ref[0], ref[1], 0.0]) + Lh * dh + np.array([0, 0, z]); bt = None; bo = None
                for s in ss:
                    Cc = c + s * u + OFF
                    for k in tk:
                        if m["target"] == "fingertip":
                            dd = SM.segdist(Cc, a0[k], a1[k]) - H.rad[k] - RSs; j = int(np.argmin(dd)); ab = a1[k] - a0[k]; tq = np.clip((Cc[j] - a0[k]) @ ab / max(ab @ ab, 1e-12), 0, 1); v = Cc[j] - (a0[k] + tq * ab)
                            if bt is None or dd[j] < bt[0]:
                                bt = (float(dd[j]), v / np.linalg.norm(v))
                    if m["target"] == "elbow":
                        for p, rr, _ in H.tpoints(m, a0, a1):
                            dd = np.linalg.norm(Cc - p, axis=1) - rr - RSs; j = int(np.argmin(dd)); v = Cc[j] - p
                            if bt is None or dd[j] < bt[0]:
                                bt = (float(dd[j]), v / np.linalg.norm(v))
                    for k in other:
                        dd = float((SM.segdist(Cc, a0[k], a1[k]) - H.rad[k] - RSs).min())
                        if bo is None or dd < bo[0]:
                            bo = (dd, int(H.seg[k]))
                return bt, bo
            lo, hi = 0.0, 0.6
            for _ in range(24):
                Lh = 0.5 * (lo + hi); bt, bo = gaps(Lh); lo, hi = (Lh, hi) if bt[0] < 0.03 else (lo, Lh)
            bt, bo = gaps(Lh); rows.append(dict(z=float(z), L=float(Lh), angle=CL.ang(bt[1], d), other_gap=bo[0], other_seg=bo[1]))
        feas = [r for r in rows if r["other_gap"] >= 0.03 and r["angle"] <= 30]; okang = [r for r in rows if r["angle"] <= 30]
        best = max(okang, key=lambda r: r["other_gap"]) if okang else None
        out.append(dict(raise_deg=deg, ref_rise_m=float(np.linalg.norm(ref - ref0)), d_elev_deg=float(np.degrees(np.arcsin(d[2]))), feasible_z=[round(r["z"], 2) for r in feas], best=best, rows=rows))
        print(f"raise {deg:2d} deg: reference point moved {100 * np.linalg.norm(ref - ref0):.1f} cm, d elev {np.degrees(np.arcsin(d[2])):.1f} deg, feasible TCP heights {[round(r['z'], 2) for r in feas]}"
              + ("" if best is None else f"; best with angle <= 30: z {best['z']:.2f}, angle {best['angle']:.0f} deg, other shapes {100 * best['other_gap']:.1f} cm (segment {best['other_seg']})"), flush=True)
    return out


if __name__ == "__main__":
    a = sys.argv; r = main(a[1], int(a[2]), a[3], float(a[4]), float(a[5]))
    json.dump(r, open(os.path.expanduser(f"~/effmass_mpc/goal1/live/{a[1]}/corrected/static_motion{a[2]}.json"), "w"), indent=1)

"""lv_corr_lib.py - live3m corrected paths (goal1/live/<tag>/corrected): shared helpers.
Human data: compare/human_rt_v2.npz of the live run (not changed), Franka base frame of the run's frame file (no base shift).
Motion target and target direction d (unit):
  motions 1, 2 (fingertip): d = forearm axis (forearm capsule p0 -> p1, elbow -> wrist); target shapes = the hand shapes of that arm;
     reference point = the hand surface point furthest along d.
  motion 3 (elbow): d = upper-arm axis (shoulder -> elbow); target = the sphere at the upper-arm capsule end p1 and the sphere at
     the forearm capsule end p0; reference point = their surface point furthest along d.
Held window: the longest run of consecutive frames of the motion in which every end point of the target shapes stays within TOL
(0.03 m) of the reference posture (median over the window, iterated).
Gap rule (as the regenerated tables): every robot sphere (MPC radius + 0.02 m) against every active human shape; steps whose TCP is
within 0.10 m of a path end are left out. Direction at the closest moment: unit vector from the nearest human surface point to the
nearest robot sphere centre (= minus the robot -> human direction), angle to d of that frame.
"""
import csv
import json
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import b2a_analyze as BA
import shape_metrics as SM
import g1_human_link as L

GG = os.path.expanduser("~/effmass_mpc/goal1"); O = BA.O; TOL = 0.03; END = 0.10
HB, HT = BA.RI.body_of["fr3v2_1_hand"]; LINK = np.array(BA.RI.sphere_link); RS = np.asarray(BA.RS, float)
ARM = {"L": (13, 15, 17), "R": (14, 16, 18)}


def tcp_of(q):
    return ((BA.RI._fk(q)[HB + 1] @ HT) @ np.array([0, 0, 0.1034, 1.0]))[:3]


def spheres(q):
    return BA.RI.compute(q)[0]


class Human:
    """All frames of the live run in the Franka base frame of the run (no base shift)."""

    def __init__(self, tag):
        C = f"{GG}/live/{tag}/compare"; self.cfg = json.load(open(f"{C}/runs.json")); self.an = json.load(open(f"{C}/analysis.json"))
        z = np.load(self.cfg["human"]); self.z = z; F = len(z["p0"]); self.F = F; R, t = L.load_frame(self.cfg["frame_file"]); self.R = R; self.t = t
        self.P0 = z["p0"].reshape(F, -1, 3).astype(float) @ R.T + t; self.P1 = z["p1"].reshape(F, -1, 3).astype(float) @ R.T + t
        self.rad = z["shape_radius"].astype(float); self.seg = z["shape_segment"].astype(int); self.act = [k for k in range(len(self.rad)) if self.rad[k] > 0]
        self.mots = [e for e in self.an["intervals"] if e["t0_s"] is not None]

    def ks(self, s):
        return [k for k in self.act if self.seg[k] == s]

    def motion(self, q, side="L"):
        e = self.mots[q]; su, sf, sh = ARM[side]; f0 = int(round(e["t0_s"] * 30)); f1 = min(int(round(e["t1_s"] * 30)), self.F - 1)
        return dict(name=e["name"], q=q, side=side, target="fingertip" if q < 2 else "elbow", f0=f0, f1=f1, ua=self.ks(su)[0], fa=self.ks(sf)[0], hand=self.ks(sh))

    def tdir(self, m, p0, p1):
        k = m["fa"] if m["target"] == "fingertip" else m["ua"]; d = p1[k] - p0[k]; return d / np.linalg.norm(d)

    def tpoints(self, m, p0, p1):
        """end points and radii that define the target"""
        if m["target"] == "fingertip":
            return [(p, self.rad[k], k) for k in m["hand"] for p in (p0[k], p1[k])]
        return [(p1[m["ua"]], self.rad[m["ua"]], m["ua"]), (p0[m["fa"]], self.rad[m["fa"]], m["fa"])]

    def tref(self, m, p0, p1):
        """reference point: target surface point furthest along d, the shape it belongs to, and d"""
        d = self.tdir(m, p0, p1); p, r, k = max(self.tpoints(m, p0, p1), key=lambda c: float(c[0] @ d) + c[1]); return p + r * d, k, d

    def held(self, m):
        fs = np.arange(m["f0"], m["f1"] + 1); X = np.array([np.concatenate([c[0] for c in self.tpoints(m, self.P0[f], self.P1[f])]) for f in fs]).reshape(len(fs), -1, 3)
        mid = slice(len(fs) // 3, 2 * len(fs) // 3); ref = np.median(X[mid], 0); best = None
        for _ in range(6):
            ok = np.linalg.norm(X - ref, axis=2).max(1) <= TOL; runs = []; a = None
            for i, v in enumerate(list(ok) + [False]):
                if v and a is None:
                    a = i
                if not v and a is not None:
                    runs.append((a, i - 1)); a = None
            if not runs:
                break
            a, b = max(runs, key=lambda r: r[1] - r[0]); best = (int(fs[a]), int(fs[b])); ref = np.median(X[a:b + 1], 0)
        hf = np.arange(best[0], best[1] + 1); p0 = np.median(self.P0[hf], 0); p1 = np.median(self.P1[hf], 0)
        return best, p0, p1


def load_run(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read(); r = json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))
    rows = list(csv.DictReader(open(r["steps_csv"]))); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows])
    return Q, [int(a["step"]) for a in (r.get("arrival_list") or [])], r


def nearest(C, p0, p1, rad, act):
    """min gap over robot spheres C and shapes: gap, sphere, shape, unit vector human point -> sphere centre"""
    best = None
    for k in act:
        d = SM.segdist(C, p0[k], p1[k]) - rad[k] - RS; j = int(np.argmin(d))
        if best is None or d[j] < best[0]:
            a, b = p0[k], p1[k]; ab = b - a; tq = np.clip((C[j] - a) @ ab / max(ab @ ab, 1e-12), 0, 1); v = C[j] - (a + tq * ab)
            best = (float(d[j]), j, k, v / max(np.linalg.norm(v), 1e-9))
    return best


def ang(v, d):
    return float(np.degrees(np.arccos(np.clip(v @ d, -1, 1))))

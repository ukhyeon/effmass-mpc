"""b2a4_video.py <front|top> [condA condB] [seed] - side-by-side comparison video of two conditions from saved runs (system python via
g1_run_human.sh py; encoder: system /usr/bin/ffmpeg through matplotlib). Rows: side view (x, z), top view (x, y), time series. Robot = MPC collision
spheres (yml radius, from the executed q), hand / forearm = node SMPL-H mesh + v2 hand capsules, dashed outline = hand capsules grown by the eta of
the closest pair at that step (the active distance of the cost), line = closest robot sphere -> closest hand-shape point. Shows the first 1.2 s (72 steps) at 8 fps
(1/8 speed) with a 2.5 s pause at the step where the two gaps are smallest together."""
import json, os, sys, csv
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.collections import PolyCollection, PatchCollection
from matplotlib.patches import Circle, Polygon
sys.path.insert(0, os.path.dirname(__file__))
import b2a_analyze as BA
import b2a4_figs as FG          # palette, labels, mesh / capsule helpers (figures are drawn only when run as a script)
sc = sys.argv[1]; ca, cb = (sys.argv[2], sys.argv[3]) if len(sys.argv) > 3 else ("iso", "massA"); seed = int(sys.argv[4]) if len(sys.argv) > 4 else 0
key = FG.SC[sc]; A = dict(eta0=0.10, muref=2.0, p=0.5, emin=0.05, emax=0.20)
eta = lambda mu: float(np.clip(A["eta0"] * (mu / A["muref"]) ** A["p"], A["emin"], A["emax"]))


def load(cond):
    tag = f"b2a4_{key}_{cond}_s{seed}"; r = BA.result(tag); Q = np.array([[float(x[f"q{i}"]) for i in range(7)] for x in csv.DictReader(open(r["steps_csv"]))])
    B = np.load(f"{BA.O}/{tag}_b1.npz")["rows"]; eta0 = float(np.load(f"{BA.O}/{tag}_b1.npz")["cost_par"][1]); out = []
    for i, q in enumerate(Q):
        C, Lr, _ = BA.RI.compute(q); D = B[i, 12:36][BA.HAND]; jj = int(np.argmin(D)); k = BA.HAND[jj]; s = int(B[i, 36 + k]); mu = float(B[i, 60 + k]); mr = float(B[i, 108 + k])
        u, _, qp = BA.shape_u(C[s]); mh = 1 / float(u @ BA.LH @ u)
        e = eta(mu) if cond == "massA" else eta(1 / (1 / 0.6 + (1 / mr if np.isfinite(mr) and mr > 0 else 0))) if cond == "iso" else (eta0 if cond in ("uni10", "unim") else None)
        out.append(dict(C=C, s=s, k=k, gap=float(D[jj]), mu=mu, mh=mh, eta=e, u=float(np.degrees(np.arccos(np.clip(u @ (-BA.AX), -1, 1)))), qp=qp, tcp=B[i, 8:11] - np.array([0, 0, 0.1034])))
    return out


DA, DB = load(ca), load(cb); n = min(len(DA), len(DB)); t = np.arange(1, n + 1) / 60.0
NS = min(n, 72); FPS = 8; i_pause = int(np.argmin([max(DA[i]["gap"], 0) + max(DB[i]["gap"], 0) for i in range(NS)]))
VIEWS = [("side view (x, z)", 0, 2, 1, (0.26, 0.72), (0.34, 0.64)), ("top view (x, y)", 0, 1, 2, (0.26, 0.72), (-0.32, 0.18))]
fig = plt.figure(figsize=(16, 11), dpi=100); gs = fig.add_gridspec(3, 2, height_ratios=[3.0, 3.4, 1.5], hspace=0.28, wspace=0.10, left=0.05, right=0.98, top=0.93, bottom=0.06)
axv = [[fig.add_subplot(gs[r, c]) for c in range(2)] for r in range(2)]; axg = fig.add_subplot(gs[2, 0]); axm = fig.add_subplot(gs[2, 1])
static = []
for r, (vn, ix, iy, iz, xl, yl) in enumerate(VIEWS):
    for c, cond in enumerate((ca, cb)):
        a = axv[r][c]; T = FG.FT.V[FG.FT.FF][:, :, [ix, iy]]; dep = FG.FT.V[FG.FT.FF][:, :, iz].mean(1); o = np.argsort(dep if iz == 2 else -dep)
        a.add_collection(PolyCollection(T[o], facecolors=(0.87, 0.70, 0.58, 1.0), edgecolors=(0.6, 0.45, 0.35, 0.3), lw=0.2))
        for k in BA.HAND:
            a.add_patch(Polygon(FG.FT.stadium(BA.P0[k][[ix, iy]], BA.P1[k][[ix, iy]], BA.RAD[k]), closed=True, fill=False, edgecolor=(0.3, 0.3, 0.3, 0.8), lw=0.7))
        js = json.load(open(f"{BA.G}/scene_b2a4_{key}.json")); A_, B_ = js["tcp_A"], js["tcp_B"]; a.plot([A_[ix], B_[ix]], [A_[iy], B_[iy]], ":", color="#52514e", lw=1)
        a.set_xlim(*xl); a.set_ylim(*yl); a.set_aspect("equal"); a.grid(alpha=0.2, lw=0.5); a.set_xlabel("xyz"[ix] + " [m]"); a.set_ylabel("xyz"[iy] + " [m]")
        a.set_title(f"{FG.LAB[cond]} - {vn}", color="#0b0b0b", fontsize=11)
for a, kk, lab, sc_ in ((axg, "gap", "closest hand gap, shape [cm]", 100), (axm, "mu", "closest-pair mu [kg]", 1)):
    for cond, D in ((ca, DA), (cb, DB)):
        a.plot(t, [d[kk] * sc_ for d in D[:n]], color=FG.COL[cond], lw=2, label=FG.LAB[cond])
    a.set_xlabel("time [s]"); a.set_ylabel(lab); a.grid(alpha=0.25, lw=0.5); a.legend(fontsize=9, loc="upper right")
axg.set_ylim(0, 30); axg.set_xlim(0, NS / 60); axm.set_xlim(0, NS / 60); cur = [axg.axvline(0, color="#0b0b0b", lw=1), axm.axvline(0, color="#0b0b0b", lw=1)]
dyn = []
fig.suptitle(f"B2a-4 {sc}-cal, seed {seed}: {FG.LAB[ca]} vs {FG.LAB[cb]}  (first {NS/60:.1f} s at 1/8 speed, pause at the closest moment; dashed = hand shape grown by eta of the closest pair)", fontsize=12)


def frame(i):
    for h in dyn:
        h.remove()
    dyn.clear()
    for r, (vn, ix, iy, iz, xl, yl) in enumerate(VIEWS):
        for c, (cond, D) in enumerate(((ca, DA), (cb, DB))):
            a = axv[r][c]; d = D[i]; col = FG.COL[cond]; C = d["C"]; o = np.argsort(C[:, iz] if iz == 1 else -C[:, iz])
            pc = PatchCollection([Circle((C[j, ix], C[j, iy]), BA.RI.sphere_radius[j]) for j in o], facecolor=(0.55, 0.55, 0.6, 0.35), edgecolor=(0.3, 0.3, 0.35, 0.5), lw=0.4)
            dyn.append(a.add_collection(pc))
            dyn.append(a.add_patch(Circle((C[d["s"], ix], C[d["s"], iy]), BA.RI.sphere_radius[d["s"]], facecolor=col, edgecolor="#0b0b0b", lw=0.8, alpha=0.9)))
            if d["eta"] is not None:
                k = d["k"]; dyn.append(a.add_patch(Polygon(FG.FT.stadium(BA.P0[k][[ix, iy]], BA.P1[k][[ix, iy]], BA.RAD[k] + d["eta"]), closed=True, fill=False, edgecolor=col, lw=1.5, ls="--")))
            dyn.append(a.plot([C[d["s"], ix], d["qp"][ix]], [C[d["s"], iy], d["qp"][iy]], "-", color="#0b0b0b", lw=1)[0])
            tr = np.array([x["tcp"] for x in D[:i + 1]]); dyn.append(a.plot(tr[:, ix], tr[:, iy], "-", color=col, lw=2)[0])
            if r == 0:
                txt = (f"t {t[i]:.3f} s\ngap {d['gap']*100:5.2f} cm\nu {d['u']:5.1f} deg from frontal\nm_h {d['mh']:.2f} kg   mu {d['mu']:.2f} kg\n"
                       f"eta {d['eta']*100:.1f} cm" if d["eta"] is not None else f"t {t[i]:.3f} s\ngap {d['gap']*100:5.2f} cm")
                dyn.append(a.text(0.02, 0.97, txt, transform=a.transAxes, va="top", ha="left", fontsize=10, family="monospace", color="#0b0b0b",
                                  bbox=dict(facecolor="#fcfcfb", edgecolor="#c3c2b7", alpha=0.9)))
    for v in cur:
        v.set_xdata([t[i], t[i]])


out = f"{BA.OUT}/b2a4_{sc}_{ca}_vs_{cb}_s{seed}.mp4"
w = FFMpegWriter(fps=FPS, codec="libx264", bitrate=3000, extra_args=["-pix_fmt", "yuv420p"])
with w.saving(fig, out, dpi=100):
    for i in range(NS):
        frame(i); w.grab_frame()
        if i == i_pause:
            for _ in range(int(2.5 * FPS)):
                w.grab_frame()
    for _ in range(FPS):
        w.grab_frame()
print("wrote", out, n, "frames")

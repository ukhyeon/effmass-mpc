"""b2a5_fig_di.py - B2a-5 danger index (D_max 0.30 m, f_I = mu / 4.6) maximum per run: panels mu_personal and mu_ISO; x = scene / condition; dots = seeds."""
import json, os
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac"); OUT = os.path.expanduser("~/effmass_mpc/goal1/b2a")
R = [r for r in json.load(open(f"{O}/b2a5_metrics2.json"))["runs"] if r["set"] == "b2a5"]
COL = {"off": "#8a8984", "iso": "#2a78d6", "massA": "#eb6834", "unim": "#eda100"}; MK = {"off": "x", "iso": "o", "massA": "s", "unim": "D"}
LAB = {"off": "off", "iso": "iso", "massA": "mass A", "unim": "uniform\nmatched"}; CONDS = ["off", "iso", "massA", "unim"]
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
fig, ax = plt.subplots(1, 2, figsize=(14, 5), sharey=True)
for a, mn, title in ((ax[0], "personal", "DI with mu_personal (same model for every condition)"), (ax[1], "iso", "DI with mu_ISO (hand m_h fixed 0.6 kg)")):
    xt, xl = [], []
    for si, sc in enumerate(("front", "top")):
        for ci, c in enumerate(CONDS):
            x0 = si * 5 + ci; rs = sorted([r for r in R if r["scene"] == sc and r["cond"] == c], key=lambda r: r["seed"])
            y = np.array([r[f"DI_{mn}_lin_D30_max"] for r in rs]); xj = x0 + np.linspace(-0.25, 0.25, len(rs))
            a.plot(xj, y, MK[c], color=COL[c], ms=7, mec=(COL[c] if c == "off" else "#fcfcfb"), mew=(2 if c == "off" else 1), ls="none")
            for xx, yy, r in zip(xj, y, rs):
                a.annotate(str(r["seed"]), (xx, yy), textcoords="offset points", xytext=(0, 5), ha="center", fontsize=6, color="#52514e")
            a.plot([x0 - 0.3, x0 + 0.3], [y.mean()] * 2, color="#0b0b0b", lw=1); xt.append(x0); xl.append(f"{sc}-cal\n{LAB[c]}")
    a.set_xticks(xt); a.set_xticklabels(xl, fontsize=8); a.set_title(title); a.grid(axis="y", alpha=0.25, lw=0.5); a.axvline(4, color="#c3c2b7", lw=0.8)
ax[0].set_ylabel("danger index max  (f_d f_v f_I, D_max 0.30 m, f_I = mu / 4.6)")
fig.suptitle("B2a-5 (profile t1): danger index maximum per run (dots = seeds, bar = mean)"); fig.tight_layout(); fig.savefig(f"{OUT}/b2a5_fig4_di.png", dpi=120); print("ok")

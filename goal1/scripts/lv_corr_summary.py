"""lv_corr_summary.py <tag> m1=<ver> m3=<ver> ... - live3m corrected paths: summary of the final path versions (motions not listed = criteria
not met, no comparison). Reads cal/m<q><ver>/{path.json, analysis_off.json, analysis_cmp.json}, compare/analysis.json (arm side) and
compare_tip/{analysis.json, scenes.json} (front pass). Writes corrected/summary.json, summary.md (tables, ASCII) and fig_corrected.png.
Per motion:
  path: TCP line height and its change from the compare_tip line, distance L, base shift, tilt
  off (human cost off, start at A and start at B, seed 0): per direction the gaps of the passes inside the held window (first pass of
      each run = from rest), angle between the direction to the robot and the target direction at each closest moment
  cmp (uniform / per-part, seeds 0-2 x start A, B): closest-approach gap per run (all shapes, held window, path ends left out),
      median [min, max] of the 6 runs, difference of the medians; angle at the closest moment; zone thickness of the closest pair
      (eta from the run's record; static zone along the approach direction); first pass time; overlap steps; gap to the target only
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

GG = os.path.expanduser("~/effmass_mpc/goal1")


def st(v):
    v = [x for x in v if x is not None]
    return None if not v else dict(median=float(np.median(v)), lo=float(min(v)), hi=float(max(v)), n=len(v))


def f(v, s=100.0, d=1):
    return "-" if not v else f"{v['median'] * s:.{d}f} [{v['lo'] * s:.{d}f}, {v['hi'] * s:.{d}f}]"


def main(tag, vers):
    C = f"{GG}/live/{tag}/corrected"; side = {e["name"]: e["per_mode"] for e in json.load(open(f"{GG}/live/{tag}/compare/analysis.json"))["intervals"] if e["t0_s"] is not None}
    tip = json.load(open(f"{GG}/live/{tag}/compare_tip/analysis.json")); tsc = {m["name"]: m for m in json.load(open(f"{GG}/live/{tag}/compare_tip/scenes.json"))["motions"]}; S = {}
    for q, ver in sorted(vers.items()):
        T = f"{C}/cal/m{q}{ver}"; P = json.load(open(f"{T}/path.json")); off = json.load(open(f"{T}/analysis_off.json")); nm = P["motion"]
        r = dict(ver=ver, target=P["target"], held_s=P["held_s"], params=P["params"], tcp_z=P["centre_base"][2], tcp_z_tip=tsc[nm]["tcp_A"][2], dz_from_tip=P["centre_base"][2] - tsc[nm]["tcp_A"][2],
                 base_shift=P["base_shift"], base_shift_tip=tsc[nm]["base_shift"], d_elev_deg=P["d_elev_deg"])
        ps = [dict(p, run=D, first=(i == 0)) for D in ("AB", "BA") for i, p in enumerate(off[D]["off"][0]["passes"])]
        r["off"] = {d_: dict(gaps=[p["gap"] for p in ps if p["dir"] == d_], first=[p["gap"] for p in ps if p["dir"] == d_ and p["first"]], angles=[p["angle_deg"] for p in ps if p["dir"] == d_],
                             zone_part=[p["zone_part"] for p in ps if p["dir"] == d_], zone_uniform=[p["zone_uniform"] for p in ps if p["dir"] == d_], seg=[p["seg"] for p in ps if p["dir"] == d_]) for d_ in ("A->B", "B->A")}
        r["off_within"] = [int(sum(0.02 <= p["gap"] <= 0.04 for p in ps)), len(ps)]; r["off_angle_max"] = max(p["angle_deg"] for p in ps)
        if os.path.exists(f"{T}/analysis_cmp.json"):
            cm = json.load(open(f"{T}/analysis_cmp.json")); r["cmp"] = {}
            for md in ("uniform", "part"):
                runs = [dict(o, start=D) for D in ("AB", "BA") for o in cm.get(D, {}).get(md, []) if "min" in o]
                r["cmp"][md] = dict(gap=st([o["min"]["gap"] for o in runs]), gap_A=st([o["min"]["gap"] for o in runs if o["start"] == "AB"]), gap_B=st([o["min"]["gap"] for o in runs if o["start"] == "BA"]),
                                    gaps=[o["min"]["gap"] for o in runs], angle=st([o["min"]["angle_deg"] for o in runs]), eta=st([o["min"].get("eta_b1") for o in runs]),
                                    zone_static=st([o["min"]["zone_part" if md == "part" else "zone_uniform"] for o in runs]), first_pass_s=st([o["first_pass_s"] for o in runs]),
                                    no_arrival_runs=int(sum(o["first_pass_s"] is None for o in runs)), overlap_steps=int(sum(o["overlap_steps"] for o in runs)), target_gap=st([o["target_gap_min"] for o in runs]),
                                    seg=[o["min"]["seg"] for o in runs], first_gap=st([o["passes"][0]["gap"] for o in runs]), n=len(runs))
            r["diff_cm"] = 100 * (r["cmp"]["part"]["gap"]["median"] - r["cmp"]["uniform"]["gap"]["median"])
            r["diff_target_cm"] = 100 * (r["cmp"]["part"]["target_gap"]["median"] - r["cmp"]["uniform"]["target_gap"]["median"])
        S[nm] = r
    prev = {m: dict(side={md: side[m][md]["min_gap_arm"] for md in ("uniform", "part")}, tip={md: tip[m][md]["target_gap"] for md in ("uniform", "part")}) for m in side}
    json.dump(dict(tag=tag, motions=S, previous=prev), open(f"{C}/summary.json", "w"), indent=1)
    L = ["| motion | off gap A->B (cm) | off gap B->A (cm) | off angle (deg) | passes in 2-4 cm |", "|---|---|---|---|---|"]
    for m, r in S.items():
        L.append(f"| {m} | " + " / ".join(f"{100 * x:.1f}" for x in r["off"]["A->B"]["gaps"]) + " | " + " / ".join(f"{100 * x:.1f}" for x in r["off"]["B->A"]["gaps"]) + " | "
                 + f"{min(r['off']['A->B']['angles'] + r['off']['B->A']['angles']):.0f}-{r['off_angle_max']:.0f} | {r['off_within'][0]} of {r['off_within'][1]} |")
    L += ["", "| motion | mode | gap all shapes (cm) | start A / start B (cm) | angle (deg) | eta at min (cm) | first pass (s) | overlap steps |", "|---|---|---|---|---|---|---|---|"]
    for m, r in S.items():
        for md in ("uniform", "part"):
            if "cmp" in r:
                c = r["cmp"][md]; L.append(f"| {m} | {md} | {f(c['gap'])} | {f(c['gap_A'])} / {f(c['gap_B'])} | {f(c['angle'], 1, 0)} | {f(c['eta'])} | {f(c['first_pass_s'], 1, 2)} ({c['no_arrival_runs']} runs without arrival) | {c['overlap_steps']} |")
    L += ["", "Per-part minus uniform (median of 6 runs, cm): " + "; ".join(f"{m}: {r['diff_cm']:+.1f} (target only {r['diff_target_cm']:+.1f})" for m, r in S.items() if "cmp" in r), "",
          "Previous comparisons (median of seeds 0-2, cm; arm side = closest arm part over the motion, front pass = target gap on the held frames): "
          + "; ".join(f"{m}: arm side {100 * p['side']['uniform']['median']:.1f} -> {100 * p['side']['part']['median']:.1f}, front pass {100 * p['tip']['uniform']['median']:.1f} -> {100 * p['tip']['part']['median']:.1f}" for m, p in prev.items()), ""]
    open(f"{C}/summary.md", "w").write("\n".join(L)); print("\n".join(L))
    mots = list(prev); fig, ax = plt.subplots(figsize=(9.5, 4.0)); w = 0.13; COL = {"uniform": "#7f7f7f", "part": "#2f6fb0"}; HA = {"side": None, "tip": "//", "corr": "xx"}
    LAB = {"side": "arm side (recorded path)", "tip": "front pass (first version)", "corr": "corrected path"}
    for qi, m in enumerate(mots):
        j = 0
        for ap in ("side", "tip", "corr"):
            for md in ("uniform", "part"):
                v = prev[m][ap][md] if ap != "corr" else (S.get(m, {}).get("cmp", {}).get(md, {}).get("gap"))
                x = qi + (j - 2.5) * w; j += 1
                if not v:
                    continue
                ax.bar(x, v["median"] * 100, w * 0.92, color=COL[md], hatch=HA[ap], edgecolor="white", lw=0.5, label=f"{LAB[ap]} - {'uniform' if md == 'uniform' else 'per-part'}" if qi == 0 else None)
                ax.plot([x, x], [v["lo"] * 100, v["hi"] * 100], color="k", lw=0.8); ax.text(x, max(v["hi"] * 100, 0) + 0.3, f"{v['median'] * 100:.1f}", ha="center", fontsize=7)
        if m not in S or "cmp" not in S[m]:
            ax.text(qi + 0.26, 1.0, "corrected:\ncriteria not met", ha="center", fontsize=7, color="#c0392b")
    ax.set_xticks(range(len(mots))); ax.set_xticklabels([f"{m}\n({tsc[m]['target']})" for m in mots]); ax.set_ylabel("closest-approach gap (cm)"); ax.legend(fontsize=7, ncol=3, loc="upper left")
    ax.set_ylim(0, ax.get_ylim()[1] + 3.0); ax.set_title(f"{tag}: closest-approach gap, uniform vs per-part (median, whisker = range of runs)", fontsize=9)
    fig.tight_layout(); fig.savefig(f"{C}/fig_corrected.png", dpi=150); plt.close(fig)


if __name__ == "__main__":
    main(sys.argv[1], {int(a.split("=")[0][1:]): a.split("=")[1] for a in sys.argv[2:]})

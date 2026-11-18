"""lv_cmp_report.py <tag> - live cmp: two figures and a one-page REPORT.md in goal1/live/<tag>/compare/ (system python3, matplotlib).
  fig_gap_time.png  arm min gap (hand, forearm, elbow, upper arm) over replay time, uniform vs per-part (median of seeds, band =
                    seed range), motions shaded
  fig_gap_bars.png  min arm gap per interval (motions, arms down), median of seeds, whisker = seed range
An existing "## Interpretation" section of REPORT.md is kept when the report is rewritten.
"""
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

GG = os.path.expanduser("~/effmass_mpc/goal1")
COL = {"uniform": "#7f7f7f", "part": "#2f6fb0"}; LAB = {"uniform": "uniform (k 0, eta 6.5 cm)", "part": "per-part reference (k 0.10)"}
plt.rcParams.update({"font.size": 9})


def f1(v, s=100.0, d=1):
    return "-" if v is None else f"{v['median'] * s:.{d}f} [{v['lo'] * s:.{d}f}, {v['hi'] * s:.{d}f}]"


def main(tag):
    C = f"{GG}/live/{tag}/compare"; an = json.load(open(f"{C}/analysis.json")); z = np.load(f"{C}/gaps_ts.npz"); ms = an["modes"]
    n = an["steps"]; t = (np.arange(n) + 1) / 60.0
    fig, ax = plt.subplots(figsize=(10, 3.6))
    for a, b in z["motions"]:
        ax.axvspan(a, b, color="#f2e6c9", lw=0)
    for e in an["intervals"]:
        if e["t0_s"] is not None:
            ax.text((e["t0_s"] + e["t1_s"]) / 2, 52, e["name"], ha="center", fontsize=8, color="#555555")
    for m in ms:
        G = np.array([z[k][:n] for k in z.files if k.startswith(m + "_s")]) * 100; G = np.clip(G, -20, 60)
        ax.fill_between(t, G.min(0), G.max(0), color=COL[m], alpha=0.2, lw=0); ax.plot(t, np.median(G, 0), color=COL[m], lw=1.2, label=LAB[m])
    ax.axhline(0, color="k", lw=0.8); ax.set_ylim(-20, 60); ax.set_xlim(0, t[-1])
    ax.set_xlabel("replay time (s), human at real time"); ax.set_ylabel("min gap robot - arm (cm)")
    ax.set_title(f"{tag}: robot to hand / forearm / elbow / upper arm (median of seeds 0-2, band = range; shaded = motion)")
    ax.legend(loc="upper right", fontsize=8); fig.tight_layout(); fig.savefig(f"{C}/fig_gap_time.png", dpi=150); plt.close(fig)
    iv = an["intervals"]; fig, ax = plt.subplots(figsize=(1.6 * len(iv) + 2.5, 3.4))
    for j, m in enumerate(ms):
        for q, e in enumerate(iv):
            v = e["per_mode"].get(m, {}).get("min_gap_arm")
            if not v:
                continue
            x = q + (j - 0.5) * 0.36
            ax.bar(x, v["median"] * 100, 0.34, color=COL[m], label=LAB[m] if q == 0 else None)
            ax.plot([x, x], [v["lo"] * 100, v["hi"] * 100], color="k", lw=0.8)
            ax.text(x, max(v["hi"] * 100, 0) + 0.8, f"{v['median'] * 100:.1f}", ha="center", fontsize=7)
    ax.axhline(0, color="k", lw=0.8); ax.set_xticks(range(len(iv))); ax.set_xticklabels([e["name"] for e in iv])
    ax.set_ylabel("min arm gap in the interval (cm)"); ax.legend(fontsize=8); ax.set_title(f"{tag}: min gap per interval (median of seeds, whisker = range)")
    fig.tight_layout(); fig.savefig(f"{C}/fig_gap_bars.png", dpi=150); plt.close(fig)
    vs = json.load(open(f"{C}/video_sync.json")) if os.path.exists(f"{C}/video_sync.json") else None
    L = [f"# {tag}: uniform vs per-part reference, replay at real time", "",
         f"Recording {tag} (human data of the live run) replayed headless with human time = simulated time, live settings (live_t1.yml, "
         f"scene_live_front.json, frame of the run), seeds 0-2 per mode. Length {an['duration_s']:.1f} s ({an['steps']} steps).",
         f"Motions: an arm part (hand, forearm, elbow) within {an['seg_d_m'] * 100:.0f} cm of the robot reference path (TCP segment A-B) for "
         f">= {an['seg_t_s']:.0f} s (gaps < {an['bridge_s']} s bridged); found {sum(1 for e in iv if e['t0_s'] is not None)}.", "",
         "Values: median [min, max] of seeds 0-2. Gap = robot sphere surface (incl. 0.02 m buffer) to shape surface, cm. eta = zone thickness of that mode "
         "at the moment of the min arm gap (cm). Overlap = steps with an arm gap < 0. Pass = steps between arrivals (s).", "",
         "| interval | mode | hand | forearm | elbow | upper arm | closest | eta | overlap | arrivals | pass s |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for e in iv:
        nm = e["name"] + (f" ({e['t0_s']:.1f}-{e['t1_s']:.1f} s)" if e["t0_s"] is not None else "")
        for m in ms:
            p = e["per_mode"].get(m)
            if not p:
                continue
            L.append(f"| {nm} | {m} | " + " | ".join(f1(p["min_gap"][k]) for k in ("hand", "forearm", "elbow", "upper_arm"))
                     + f" | {'/'.join(p['closest'])} | {f1(p['eta_at_min'])} | {f1(p['overlap_steps'], 1, 0)} | {f1(p['arrivals'], 1, 0)} | {f1(p['pass_time_s'], 1, 2)} |")
    L += ["", "Per-part minus uniform, min arm gap (median of seeds, cm): " + ", ".join(
        f"{e['name']} {100 * (e['per_mode']['part']['min_gap_arm']['median'] - e['per_mode']['uniform']['min_gap_arm']['median']):+.1f}"
        for e in iv if "part" in e["per_mode"] and "uniform" in e["per_mode"]) + ".",
        "First step where the two modes' joint commands differ (per seed, s): " + ", ".join("-" if x is None else f"{x:.2f}" for x in an.get("first_divergence_s", []))
        + "; before it the two runs are bit-equal (same seed, same human, all zones at eta0). Nearest human shape then (gap cm / segment): "
        + ", ".join(f"{d['gap_cm']:.1f}/{d['segment']}" for d in an.get("divergence_nearest", [])) + ".",
        f"Realtime factor of the replays (headless): " + "; ".join(f"{m} " + ", ".join(f"{x:.2f}" for x in v) for m, v in an["rtf"].items()) + ".", "",
        "![gap over time](fig_gap_time.png)", "", "![gap per interval](fig_gap_bars.png)", ""]
    if vs:
        L += [f"Video: {os.path.basename(vs['out'])} (Kinect colour | uniform | per-part, seed 0, {vs['duration_s']:.1f} s); zone shells on forearms, hands and elbows. "
              + (f"Colour frame to replay time offset median {vs['colour_time_offset_s']['median'] * 1000:.0f} ms, max {vs['colour_time_offset_s']['max'] * 1000:.0f} ms. " if vs.get("colour_time_offset_s") else "No colour frames. ")
              + "Clips: " + ", ".join(f"{c['file']} ({c['start_s']:.1f} s +10 s)" for c in vs["clips"]) + ".", ""]
    old = open(f"{C}/REPORT.md").read() if os.path.exists(f"{C}/REPORT.md") else ""
    if "## Interpretation" in old:
        L += [old[old.index("## Interpretation"):].rstrip(), ""]
    open(f"{C}/REPORT.md", "w").write("\n".join(L)); print(f"{C}/REPORT.md, fig_gap_time.png, fig_gap_bars.png")


if __name__ == "__main__":
    main(sys.argv[1])

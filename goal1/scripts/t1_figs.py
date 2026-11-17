"""t1_figs.py - T1 summary json and figures (system python via g1_run_human.sh py).
logs/meff_isaac/t1_summary.json: per setting the target checks; ROS-path realtime factor and paired solver increase.
b2a/t1_fig1_settings.png: near-human sphere speed p99 and gripper rotation error at the closest moment per setting (dots = runs).
b2a/t1_fig2_traj.png: final setting (s4 = profile t1) seed 0 TCP paths of off and uniform_matched with B2a-4 off seed 0, side (x, z) and top (x, y)."""
import json, os, re, sys
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon
sys.path.insert(0, os.path.dirname(__file__))
import b2a4_figs as FG
O, G, OUT = FG.O, FG.G, FG.OUT
SETS = {"s1": "w 1e4, rot 60", "s2": "w 1e5, rot 60", "s3": "w 3e5, rot 60", "s4": "w 3e5, rot 200"}
M = {s: json.load(open(f"{O}/t1_{s}_metrics.json")) for s in SETS}
res = loadres = lambda tag: json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", open(f"{O}/{tag}.log", errors="replace").read())[-1]))
S = {"settings": {}}
for s, lab in SETS.items():
    rs = M[s]
    S["settings"][s] = dict(label=lab, v_near_p99_max=max(r["v_near_p99"] for r in rs), yaw_at_min_max=max(r["yaw_at_min_deg"] for r in rs), all_arrived=all(r["arrived"] for r in rs),
                            infeasible=sum(r["infeasible"] for r in rs), pass_v=all(r["v_near_p99"] <= 0.55 for r in rs), pass_yaw=all(r["yaw_at_min_deg"] <= 10 for r in rs),
                            runs={r["tag"]: {k: r[k] for k in ("v_near_p99", "v_near_max", "yaw_at_min_deg", "yaw_window_max_deg", "arrival_step", "infeasible", "max_dev_line_xz", "pass_time_s",
                                                                 "ee_path_m", "hand_gap_shape", "hand_gap_mesh", "side")} for r in rs})
rm = loadres("t1_ros_mass"); rp = loadres("t1_ros_pair")
S["ros_path"] = dict(run="t1_ros_mass (g1 replay via human node, mass A set A, profile t1)", realtime_factor=rm["realtime_factor"], loop_ms_mean=rm["loop_ms_mean"], solver_ms_mean=rm["solver_ms_mean"],
                     infeasible=rm["infeasible_steps"], captures=rm["graph_captures_by_stage"], paired_speedcap=rp["meff"]["paired"])
S["final"] = "s4"; json.dump(S, open(f"{O}/t1_summary.json", "w"), indent=1)
print(json.dumps({s: {k: v for k, v in d.items() if k != "runs"} for s, d in S["settings"].items()}, indent=0)); print(S["ros_path"])

plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
CC = {"front": "#2a78d6", "top": "#eb6834"}; MK = {"off": "o", "unim": "D"}
fig, ax = plt.subplots(1, 2, figsize=(13, 4.8))
for a, key, lim, lab in ((ax[0], "v_near_p99", 0.55, "near-human sphere speed p99 [m/s]"), (ax[1], "yaw_at_min_deg", 10, "gripper rotation error at the closest moment [deg]")):
    for i, s in enumerate(SETS):
        for j, r in enumerate(M[s]):
            x = i + (-0.27 + 0.18 * (r["scene"] == "top") + 0.09 * (r["cond"] == "unim") + 0.04 * r["seed"])
            a.plot(x, r[key], MK[r["cond"]], color=CC[r["scene"]], ms=7, mec="#fcfcfb", mew=1)
    a.axhline(lim, color="#52514e", ls=":", lw=1); a.text(len(SETS) - 0.5, lim, f" target {lim}", va="bottom", ha="right", fontsize=8, color="#52514e")
    a.set_xticks(range(len(SETS))); a.set_xticklabels([f"{s}\n{SETS[s]}" for s in SETS]); a.set_ylabel(lab); a.grid(axis="y", alpha=0.25, lw=0.5)
h = [plt.Line2D([], [], marker=MK[c], color=CC[sc], ls="none", ms=7, label=f"{sc}-cal {'off' if c == 'off' else 'uniform_matched'}") for sc in CC for c in MK]
ax[0].legend(handles=h, fontsize=8, loc="upper right"); fig.suptitle("T1 tuning: settings tried (off and uniform_matched, seeds 0 and 1)"); fig.tight_layout(); fig.savefig(f"{OUT}/t1_fig1_settings.png", dpi=120); plt.close(fig)

VIEWS = {"side view (x, z)": (0, 2, 1), "top view (x, y)": (0, 1, 2)}; LINES = {"B2a-4 off (no cap)": ("b2a4_{k}_off_s0", "#8a8984", "--"), "t1 off": ("t1_s4_{k}_off_s0", "#2a78d6", "-"),
                                                                             "t1 uniform_matched": ("t1_s4_{k}_unim_s0", "#eda100", "-")}
fig, ax = plt.subplots(2, 2, figsize=(12, 9))
for pi, sc in enumerate(FG.SC):
    js = json.load(open(f"{G}/scene_b2a4_{FG.SC[sc]}.json")); A_, B_ = np.array(js["tcp_A"]), np.array(js["tcp_B"])
    for vi, (vn, (ix, iy, iz)) in enumerate(VIEWS.items()):
        a = ax[pi, vi]; T = FG.FT.V[FG.FT.FF][:, :, [ix, iy]]; dep = FG.FT.V[FG.FT.FF][:, :, iz].mean(1); o = np.argsort(dep if iz == 2 else -dep)
        a.add_collection(PolyCollection(T[o], facecolors=(0.87, 0.70, 0.58, 1.0), edgecolors=(0.6, 0.45, 0.35, 0.3), lw=0.2))
        for k in range(24):
            if FG.FT.SEG[k] in (16, 18) and FG.FT.RAD[k] > 0:
                a.add_patch(Polygon(FG.FT.stadium(FG.FT.P0[k][[ix, iy]], FG.FT.P1[k][[ix, iy]], FG.FT.RAD[k]), closed=True, facecolor=(0.3, 0.3, 0.3, 0.08), edgecolor=(0.3, 0.3, 0.3, 0.8), lw=0.7))
        a.plot([A_[ix], B_[ix]], [A_[iy], B_[iy]], ":", color="#52514e", lw=1, label="TCP line")
        for lab, (pat, col, ls) in LINES.items():
            tcp = np.load(f"{O}/{pat.format(k=FG.SC[sc])}_b1.npz")["rows"][:, 8:11] - np.array([0, 0, 0.1034]); a.plot(tcp[:, ix], tcp[:, iy], ls, color=col, lw=2, label=lab)
        a.set_aspect("equal"); a.grid(alpha=0.25, lw=0.5); a.set_title(f"{sc}-cal: {vn}"); a.set_xlabel("xyz"[ix] + " [m]"); a.set_ylabel("xyz"[iy] + " [m]")
        a.set_xlim(0.25, 0.80); a.set_ylim((0.34, 0.66) if vi == 0 else (-0.45, 0.45))
ax[0, 1].legend(fontsize=8, loc="lower right"); fig.suptitle("T1 final setting (profile t1) seed 0 TCP paths vs B2a-4 off"); fig.tight_layout(); fig.savefig(f"{OUT}/t1_fig2_traj.png", dpi=120); plt.close(fig)
print("ok")

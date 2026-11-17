"""on_wp3_summary.py - WP3 static scene matrix summary from on_analyze outputs (no runs). Per scene x condition (seeds pooled): target gap (shape, mesh;
target = forearm + hand shapes), side, u vs front, mu, eta, gap / eta, pass time, near-human speed p99, arrival, infeasible, cost shape at the executed
state (not the rollout aggregate). Classification per scene (uniform vs ours, seed means): seed spread s = max over conditions of the seed range of the gap.
  'ours farther'          : ours - uniform > max(1 cm, s)
  'no difference (light)' : |ours - uniform| <= max(1 cm, s) and ours eta at the minimum < 7.5 cm (mu near or below mu_ref)
  'no difference'         : |ours - uniform| <= max(1 cm, s) otherwise
  'path changed'          : passing side differs between the conditions in the majority of seeds (checked first)
  'saturation / arrival / time' : any run not arrived or infeasible > 0, or ours pass time > 1.3 x uniform, or ours eta at eta_max (checked first)
Outputs logs/meff_isaac/on_wp3_summary.json, goal1/overnight/wp3_scene_gap.png, goal1/overnight/wp3_effect_table.png."""
import json, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; OV = f"{H}/effmass_mpc/goal1/overnight"
P = json.load(open(f"{O}/on_wp3_passes.json")); R = json.load(open(f"{O}/on_wp3_runs.json"))
scenes = sorted({p["scene_name"] for p in P}, key=lambda s: (s.split("_")[0], s)); out = {}
for sc in scenes:
    row = {}
    for c in ("off", "gk000", "gain"):
        ps = [p for p in P if p["scene_name"] == sc and p["cond"] == c and p["pass_idx"] == 0]; rs = [r for r in R if r["scene_name"] == sc and r["cond"] == c]
        if not ps:
            continue
        g = [p["gap_shape"] for p in ps]
        row[c] = dict(seeds=[p["seed"] for p in ps], gap_shape=g, gap_mean=float(np.mean(g)), gap_mesh=[p["gap_mesh"] for p in ps], side=[p["side"] for p in ps],
                      u_deg=[p["u_vs_front_deg"] for p in ps], mu=[p["mu"] for p in ps], eta=[p["eta"] for p in ps], min_seg=[p["min_seg"] for p in ps],
                      ratio=[p["gap_shape"] / p["eta"] if p["eta"] else None for p in ps], pass_time=[r["arrival_s"] for r in rs], v99=[r["v_near_p99"] for r in rs],
                      arrived=[r["arrived"] for r in rs], infeasible=[r["infeasible"] for r in rs], cost_seg=[p["cost_shape_seg"] for p in ps], rtf=[r["realtime_factor"] for r in rs], torso_cost_steps=[r.get("torso_cost_steps") for r in rs], torso_share=[r.get("torso_share_of_step_max") for r in rs],
                      cost_steps=[r.get("cost_steps") for r in rs],
                      pen_shape=[r["pen_shape_hand"] for r in rs])
    if "gk000" in row and "gain" in row:
        u, o = row["gk000"], row["gain"]; s = max(np.ptp(u["gap_shape"]), np.ptp(o["gap_shape"])); thr = max(0.01, s); d = o["gap_mean"] - u["gap_mean"]
        pt_u = np.mean([x for x in u["pass_time"] if x]) if any(u["pass_time"]) else None; pt_o = np.mean([x for x in o["pass_time"] if x]) if any(o["pass_time"]) else None
        side_diff = sum(a != b for a, b in zip(u["side"], o["side"])) > len(u["side"]) / 2
        bad = (not all(u["arrived"] + o["arrived"])) or sum(u["infeasible"] + o["infeasible"]) > 0 or (pt_u and pt_o and pt_o > 1.3 * pt_u) or max(o["eta"]) >= 0.25 - 1e-9
        if bad:
            cl = "saturation / arrival / time"
        elif side_diff:
            cl = "path changed"
        elif d > thr:
            cl = "ours farther"
        elif abs(d) <= thr and max(o["eta"]) < 0.075:
            cl = "no difference (light)"
        elif abs(d) <= thr:
            cl = "no difference"
        else:
            cl = "ours closer"
        row["class"] = cl; row["diff_m"] = float(d); row["threshold_m"] = float(thr); row["pass_time_ratio"] = float(pt_o / pt_u) if (pt_u and pt_o) else None
    row["torso_min_m"] = 0.25 if sc.endswith("25") else (0.18 if sc.endswith("18") else 0.35)
    out[sc] = row
json.dump(out, open(f"{O}/on_wp3_summary.json", "w"), indent=1)
for sc, r in out.items():
    f = lambda c: f"{100*r[c]['gap_mean']:5.1f}" if c in r else "  -  "
    print(f"{sc:22s} off {f('off')} uni {f('gk000')} ours {f('gain')} diff {100*r.get('diff_m', np.nan):5.1f} thr {100*r.get('threshold_m', np.nan):4.1f} "
          f"eta {np.round(100*np.array(r['gain']['eta']),1) if 'gain' in r else '-'} side u {r['gk000']['side'] if 'gk000' in r else '-'} o {r['gain']['side'] if 'gain' in r else '-'} "
          f"PT {r.get('pass_time_ratio') and round(r['pass_time_ratio'],2)} torso u {r['gk000']['torso_cost_steps'] if 'gk000' in r else '-'} o {r['gain']['torso_cost_steps'] if 'gain' in r else '-'} "
          f"o share {r['gain']['torso_share'] if 'gain' in r else '-'} -> {r.get('class')}")
# figure: scene x condition gap
sc2 = [s for s in scenes if "gk000" in out[s] and "gain" in out[s]]; fig, ax = plt.subplots(figsize=(max(10, 1.1 * len(sc2)), 6)); x = np.arange(len(sc2))
for j, (c, col, lab) in enumerate((("off", "#9a9a9a", "off"), ("gk000", "#2a78d6", "uniform"), ("gain", "#d1495b", "ours"))):
    for i, s in enumerate(sc2):
        if c in out[s]:
            g = 100 * np.array(out[s][c]["gap_shape"]); ax.scatter(np.full(len(g), i + (j - 1) * 0.25), g, color=col, s=22, label=lab if i == 0 else None, zorder=3)
            ax.plot([i + (j - 1) * 0.25 - 0.1, i + (j - 1) * 0.25 + 0.1], [g.mean()] * 2, color=col, lw=2)
ax.axhline(6.5, color="#2a78d6", ls=":", lw=1); ax.axhline(0, color="#333", lw=0.8)
ax.set_xticks(x); ax.set_xticklabels([f"{s}\n{out[s].get('class', '')}" for s in sc2], rotation=35, ha="right", fontsize=8); ax.set_ylabel("minimum forearm/hand gap (cm, shape)")
ax.set_title("WP3 static scenes: gap by scene and condition (dots = seeds, bar = mean; dotted = uniform eta 6.5 cm)"); ax.legend(frameon=False); ax.grid(axis="y", alpha=0.3)
fig.tight_layout(); fig.savefig(f"{OV}/wp3_scene_gap.png", dpi=130); plt.close(fig)

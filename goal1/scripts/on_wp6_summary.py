"""on_wp6_summary.py jobs <out.json> | summary - WP6 tuning: jobs for every on6_* log (WP3 scenes, target forearm + hand); summary compares each
candidate with the t1 runs of WP3 on the same scenes, conditions and seeds: near-human speed p99, arrival, infeasible, penetration, RTF, gap, gap / eta,
pass time, passing side. Output logs/meff_isaac/on_wp6_summary.json."""
import glob, json, os, re, sys
import numpy as np
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac"); G = os.path.expanduser("~/effmass_mpc/goal1/b2a/g2/scene_b2a4_front_0.38h1.json"); K = {"gk000": 0.0, "gain": 0.10}
if sys.argv[1] == "jobs":
    jobs = []
    for f in sorted(glob.glob(f"{O}/on6_*.log")):
        t = os.path.basename(f)[:-4]; m = re.match(r"on6_(P\d_[a-z0-9]+)_(gk000|gain)_s(\d)_(c\d)$", t)
        if m:
            sc, c, s, cand = m.groups(); jobs.append(dict(tag=t, human=f"human_on3_{sc}_v2", k=K[c], scene=G, target_segs=[16, 18], single_pass=True, group="wp6", scene_name=sc, cond=c, seed=int(s), variant=cand))
    json.dump(jobs, open(sys.argv[2], "w")); print(len(jobs), "jobs"); sys.exit(0)
P6 = json.load(open(f"{O}/on_wp6_passes.json")); R6 = json.load(open(f"{O}/on_wp6_runs.json")); P3 = json.load(open(f"{O}/on_wp3_passes.json")); R3 = json.load(open(f"{O}/on_wp3_runs.json"))
def row(P, R, sc, c, s, var):
    p = [x for x in P if x["scene_name"] == sc and x["cond"] == c and x["seed"] == s and x["variant"] == var and x["pass_idx"] == 0]; r = [x for x in R if x["scene_name"] == sc and x["cond"] == c and x["seed"] == s and x["variant"] == var]
    if not p or not r:
        return None
    p, r = p[0], r[0]; return dict(v99=r["v_near_p99"], arrived=r["arrived"], infeasible=r["infeasible"], pen=r["pen_shape_hand"], rtf=r["realtime_factor"], gap=p["gap_shape"], ratio=p["gap_shape"] / p["eta"], pass_time=r["arrival_s"], side=p["side"])
out = {}
for cand in sorted({x["variant"] for x in R6}):
    keys = sorted({(x["scene_name"], x["cond"], x["seed"]) for x in R6 if x["variant"] == cand}); rows = []
    for sc, c, s in keys:
        a = row(P6, R6, sc, c, s, cand); b = row(P3, R3, sc, c, s, "base")
        if a and b:
            rows.append(dict(scene=sc, cond=c, seed=s, cand=a, t1=b))
    agg = lambda w, f: {k: f([r[w][k] for r in rows if r["cond"] == cc]) for k in ("gap", "ratio") for cc in ("gk000", "gain")} if False else None
    summ = {}
    for cc in ("gk000", "gain"):
        rr = [r for r in rows if r["cond"] == cc]
        if rr:
            summ[cc] = {f"{w}_{k}": float(np.mean([r[w][k] for r in rr])) for w in ("cand", "t1") for k in ("gap", "ratio", "pass_time")}
            summ[cc].update({f"{w}_v99_max": float(max(r[w]["v99"] for r in rr)) for w in ("cand", "t1")})
            summ[cc].update(dict(must_ok=all(r["cand"]["arrived"] and r["cand"]["infeasible"] == 0 and r["cand"]["pen"] == 0 and r["cand"]["v99"] <= 0.52 and r["cand"]["rtf"] >= 1.0 for r in rr),
                                 side_same=all(r["cand"]["side"] == r["t1"]["side"] for r in rr)))
    out[cand] = dict(rows=rows, summary=summ)
    for cc, x in summ.items():
        print(cand, cc, {k: round(v, 3) if isinstance(v, float) else v for k, v in x.items()})
    for r in rows:
        print("  ", r["scene"], r["cond"], "s%d" % r["seed"], "v99 %.3f -> %.3f" % (r["t1"]["v99"], r["cand"]["v99"]), "gap %.1f -> %.1f" % (100 * r["t1"]["gap"], 100 * r["cand"]["gap"]),
              "PT %.2f -> %.2f" % (r["t1"]["pass_time"] or -1, r["cand"]["pass_time"] or -1), r["cand"]["side"])
json.dump(out, open(f"{O}/on_wp6_summary.json", "w"), indent=1)

"""on_wp3_jobs.py <out.json> - job list for on_analyze.py over every on3_* run log present (WP3 static scenes; target = forearm + hand)."""
import glob, json, os, re, sys
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac"); G = os.path.expanduser("~/effmass_mpc/goal1/b2a/g2/scene_b2a4_front_0.38h1.json")
K = {"off": None, "gk000": 0.0, "gain": 0.10, "gk005": 0.05, "gk015": 0.15}; jobs = []
for f in sorted(glob.glob(f"{O}/on3_*.log")):
    t = os.path.basename(f)[:-4]; m = re.match(r"on3_(P\d_[a-z0-9]+)_(off|gk000|gain|gk005|gk015)_s(\d)(.*)$", t)
    if not m or "_rec" in t:
        continue
    sc, c, s, suf = m.groups()
    jobs.append(dict(tag=t, human=f"human_on3_{sc}_v2", k=K[c], scene=G, target_segs=[16, 18], single_pass=True, cost_split=True, group="wp3", scene_name=sc, scene_key=sc, cond=c, seed=int(s), variant=suf or "base"))
for j in jobs:
    j["scene"] = G; j["scene_name"] = j.pop("scene_key")
json.dump(jobs, open(sys.argv[1], "w")); print(len(jobs), "jobs")

"""on_wp5_jobs.py <out.json> - on_analyze job list for every on5_* run log (WP5, R1 0.5 m/s scene; true human = human_r1_reach_v050)."""
import glob, json, os, re, sys
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac"); G = os.path.expanduser("~/effmass_mpc/goal1/b2a/g2/scene_b2a4_front_0.38h1.json"); jobs = []
for f in sorted(glob.glob(f"{O}/on5_*.log")):
    t = os.path.basename(f)[:-4]; m = re.match(r"on5_v050_(gk000|gain)_s(\d)_(.+)$", t)
    if not m:
        continue
    c, s, var = m.groups(); jobs.append(dict(tag=t, human="human_r1_reach_v050", k={"gk000": 0.0, "gain": 0.10}[c], scene=G, group="wp5", speed="v050", cond=c, seed=int(s),
                                             variant=var.replace("_L01", "+L01").replace("_P", "+P")))
json.dump(jobs, open(sys.argv[1], "w")); print(len(jobs), "jobs")

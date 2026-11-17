"""shapes_eval.py - v1 vs v2 shape metrics on the fit frames (static pose + replay 0/300/600/900/1261) -> goal1/b2a/shapes_eval.json"""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
import shape_metrics as SM
H = os.path.expanduser("~"); D = f"{H}/effmass_mpc/goal1/data"
LAB = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy")
S = np.load(f"{D}/human_b2a_reach.npz"); SV = np.load(f"{D}/human_b2a_reach_verts.npz"); F = SV["faces"]
R = np.load(f"{D}/human_precomputed_1781524848.npz"); RV = np.load(f"{D}/human_precomputed_1781524848_verts.npz", mmap_mode="r")["verts"]
V2 = json.load(open(f"{D}/shapes_v2.json")); NAMES = list(R["segment_names"])
FR = [("static", S, 0, SV["verts"][0])] + [(f"replay{f}", R, f, np.asarray(RV[f])) for f in (0, 300, 600, 900, 1261)]
OWN_V2 = {10: [8], 11: [8]}     # v2: shoulders covered by the chest capsule
res = {"v1": {}, "v2": {}}
for name, Z, f, V in FR:
    mesh = SM.Mesh(V.astype(float), F, LAB); rot, J = Z["rot"][f].astype(float), Z["jprox"][f].astype(float)
    sets = {"v1": [(int(s), Z["p0"][f][k].astype(float), Z["p1"][f][k].astype(float), float(Z["shape_radius"][k])) for k, s in enumerate(Z["shape_segment"])],
            "v2": [(e["segment"], J[e["segment"]] + rot[e["segment"]] @ np.array(e["p0_local"]), J[e["segment"]] + rot[e["segment"]] @ np.array(e["p1_local"]), e["radius"]) for e in V2["shapes"]]}
    for ver, shp in sets.items():
        allc = [(a, b, r) for _, a, b, r in shp]; out = {}
        for s in range(19):
            own = [c for t, *c in shp if t == s]
            out[NAMES[s]] = dict(n_shapes=len(own), under=SM.under(V[LAB == s].astype(float), allc),
                                 over=SM.over(mesh, [tuple(c) for c in own], [s] + ([10, 11] if (ver == "v2" and s == 8) else []), seed=s))
        res[ver][name] = out
        print(name, ver, " ".join(f"{NAMES[s][:6]}:{out[NAMES[s]]['under']['p95_m']*100:.1f}/{(out[NAMES[s]]['over'] or {}).get('median_m', float('nan'))*100:.1f}" for s in range(19)), flush=True)
json.dump(res, open(f"{H}/effmass_mpc/goal1/b2a/shapes_eval.json", "w"), indent=1)

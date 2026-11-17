"""g1_variant_spheres.py — 지금 옮긴 FR3 구(fr3_port.yml)가 fr3 / fr3v2 / fr3v2_1 충돌 형상을 덮는 정도 (g1_sphere_eval 과 같은 방법) + 메시 크기"""
import os, json, numpy as np, trimesh, yaml
import xml.etree.ElementTree as ET
V = os.path.expanduser("~/effmass_mpc/goal1/fr3/variants")
S = yaml.safe_load(open(os.path.expanduser("~/effmass_mpc/goal1/fr3/spheres/fr3_port.yml")))["collision_spheres"]
def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]])
def geom(m):
    out = {}
    for l in ET.parse(f"{V}/{m}.urdf").getroot().findall("link"):
        ms = []
        for c in l.findall("collision"):
            g = c.find("geometry")[0]; o = c.find("origin"); T = np.eye(4)
            if o is not None:
                T[:3, :3] = rpy2R(*[float(x) for x in o.get("rpy", "0 0 0").split()]); T[:3, 3] = [float(x) for x in o.get("xyz", "0 0 0").split()]
            mm = trimesh.load(os.path.join(V, g.get("filename")), force="mesh") if g.tag == "mesh" else trimesh.creation.box(extents=[float(x) for x in g.get("size").split()]) if g.tag == "box" else None
            if mm is None: continue
            mm.apply_transform(T); ms.append(mm)
        if ms: out[l.get("name").replace(m + "_", "fr3_")] = trimesh.util.concatenate(ms)
    return out
def cover(mesh, C, R):
    pts, _ = trimesh.sample.sample_surface(mesh, 4000, seed=0)
    d = (np.linalg.norm(pts[:, None] - C[None], axis=2) - R[None]).min(1)
    return round(100 * float((d > 0).mean()), 1), round(float(max(0, np.percentile(d, 95)) * 1000), 1), round(float(max(0, d.max()) * 1000), 1)
G = {m: geom(m) for m in ["fr3", "fr3v2", "fr3v2_1"]}
res = {}
for ln, sp in S.items():
    C = np.array([s["center"] for s in sp]); R = np.array([s["radius"] for s in sp])
    row = {m: dict(cover=cover(G[m][ln], C, R), extent_mm=(G[m][ln].extents * 1000).round(1).tolist()) for m in G if ln in G[m]}
    res[ln] = row
    print(f"{ln:16s} " + " | ".join(f"{m}: 밖 {r['cover'][0]}% p95 {r['cover'][1]} 최대 {r['cover'][2]} mm, 크기 {r['extent_mm']}" for m, r in row.items()))
json.dump(res, open(os.path.expanduser("~/effmass_mpc/goal1/logs/g1_variant_spheres.json"), "w"), indent=1)

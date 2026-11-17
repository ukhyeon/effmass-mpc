"""g1_sphere_eval.py — FR3 충돌 구 결정용 비교 (Isaac Python: trimesh, curobo; SimulationApp 없음)

  각 링크 충돌 형상(URDF <collision>: 메시 또는 박스, origin 포함) 표면에서 점 4,000개를 뽑아
  '구 바깥에 있는 표면점 비율'과 '구 표면 밖 거리 p95·최대'를 잰다.
  (1) Panda 구 → Panda 형상 (기준), (2) Panda 구를 FR3 링크로 옮김 → FR3 형상, (3) cuRobo fit_spheres_to_mesh 로 FR3 형상에 새로 맞춤 (Panda 와 같은 개수)
  오른쪽 손가락: Panda 는 관절 회전 없음·형상을 Rz(π) 회전, FR3 는 관절이 Rz(π) → 같은 세계 형상. 옮길 때 구 중심에 Rz(π) 적용.
  출력: goal1/fr3/spheres/fr3_port.yml, fr3_fit.yml, goal1/logs/g1_sphere_eval.json
"""
import json, os, sys
import numpy as np, trimesh, yaml
import xml.etree.ElementTree as ET
from curobo.geom.sphere_fit import fit_spheres_to_mesh, SphereFitType
from curobo.util_file import get_assets_path, get_robot_configs_path, join_path
G = os.path.expanduser("~/effmass_mpc/goal1")
np.random.seed(0)
PANDA_URDF = join_path(get_assets_path(), "robot/franka_description/franka_panda.urdf"); PANDA_ROOT = os.path.dirname(PANDA_URDF)
FR3_URDF = f"{G}/fr3/fr3_curobo.urdf"; FR3_ROOT = f"{G}/fr3"
PS = yaml.safe_load(open(join_path(get_robot_configs_path(), "spheres/franka_mesh.yml")))["collision_spheres"]
def rpy2R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]])
def link_geom(urdf, root):
    out = {}
    for l in ET.parse(urdf).getroot().findall("link"):
        ms = []
        for c in l.findall("collision"):
            g = c.find("geometry")[0]; o = c.find("origin")
            T = np.eye(4)
            if o is not None:
                T[:3, :3] = rpy2R(*[float(x) for x in o.get("rpy", "0 0 0").split()]); T[:3, 3] = [float(x) for x in o.get("xyz", "0 0 0").split()]
            if g.tag == "mesh":
                m = trimesh.load(os.path.join(root, g.get("filename")), force="mesh")
            elif g.tag == "box":
                m = trimesh.creation.box(extents=[float(x) for x in g.get("size").split()])
            else:
                continue
            m.apply_transform(T); ms.append(m)
        if ms:
            out[l.get("name")] = trimesh.util.concatenate(ms)
    return out
PG, FG = link_geom(PANDA_URDF, PANDA_ROOT), link_geom(FR3_URDF, FR3_ROOT)
def cover(mesh, C, R, n=4000):
    pts, _ = trimesh.sample.sample_surface(mesh, n, seed=0)
    d = (np.linalg.norm(pts[:, None] - C[None], axis=2) - R[None]).min(1)
    return dict(out_frac=float((d > 0).mean()), out_p95_mm=float(max(0, np.percentile(d, 95)) * 1000), out_max_mm=float(max(0, d.max()) * 1000))
Rz = rpy2R(0, 0, np.pi)
names = [k for k in PS if k in PG]
res, port, fit = {}, {}, {}
for pn in names:
    fn = pn.replace("panda_", "fr3_")
    C = np.array([s["center"] for s in PS[pn]], float); R = np.array([s["radius"] for s in PS[pn]], float)
    Cf = C @ Rz.T if pn == "panda_rightfinger" else C
    r = dict(n=len(R), panda_on_panda=cover(PG[pn], C, R), port_on_fr3=cover(FG[fn], Cf, R))
    port[fn] = [{"center": [round(float(x), 5) for x in c], "radius": float(rr)} for c, rr in zip(Cf, R)]
    try:
        fc, fr = fit_spheres_to_mesh(FG[fn], len(R), surface_sphere_radius=0.01, fit_type=SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE)
        fc, fr = np.asarray(fc, float), np.asarray(fr, float)
        r["fit_on_fr3"] = cover(FG[fn], fc, fr); r["fit_n"] = len(fr)
        fit[fn] = [{"center": [round(float(x), 5) for x in c], "radius": round(float(rr), 5)} for c, rr in zip(fc, fr)]
    except Exception as e:
        r["fit_error"] = repr(e)[:200]
    r["fr3_mesh_extent_mm"] = (FG[fn].extents * 1000).round(1).tolist(); r["panda_mesh_extent_mm"] = (PG[pn].extents * 1000).round(1).tolist()
    res[fn] = r
    f = lambda d: f"{100*d['out_frac']:5.1f}% p95 {d['out_p95_mm']:5.1f} max {d['out_max_mm']:5.1f}"
    print(f"{fn:16s} n={len(R):2d} | Panda구→Panda {f(r['panda_on_panda'])} | 옮김→FR3 {f(r['port_on_fr3'])} | 맞춤→FR3 " +
          (f(r['fit_on_fr3']) + f" (n={r['fit_n']})" if "fit_on_fr3" in r else r.get("fit_error", "")) +
          f" | 크기 Panda {r['panda_mesh_extent_mm']} FR3 {r['fr3_mesh_extent_mm']}")
os.makedirs(f"{G}/fr3/spheres", exist_ok=True)
yaml.safe_dump({"collision_spheres": port}, open(f"{G}/fr3/spheres/fr3_port.yml", "w"), sort_keys=False)
yaml.safe_dump({"collision_spheres": fit}, open(f"{G}/fr3/spheres/fr3_fit.yml", "w"), sort_keys=False)
json.dump(res, open(f"{G}/logs/g1_sphere_eval.json", "w"), indent=1)

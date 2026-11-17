"""t1_speed_check.py <tag> - CPU (float64 numpy) reference of the T1 speed-cap rollout cost vs the GPU value in the dumps (<tag>_dump_*.npz):
v = |c_h - c_(h-1)| * inv_dt_h, penalty = w * max over spheres with dist <= d_cap of max(0, v - v_cap)^2, h >= 1; also inv_dt vs traj_dt."""
import glob, json, os, sys
import numpy as np
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac"); res = []
for f in sorted(glob.glob(f"{O}/{sys.argv[1]}_dump_*.npz")):
    z = np.load(f); x = z["sc_sph"][..., :3].astype(np.float64); dist = z["sc_dist"].astype(np.float64); g = z["sc_vcost"].astype(np.float64); v_cap, d_cap, w = z["sc_par"].astype(np.float64)
    td = z["traj_dt"].astype(np.float64).reshape(-1); inv_dt_ref = 1 / td[1:]
    v = np.linalg.norm(x[:, 1:] - x[:, :-1], axis=-1) * inv_dt_ref[None, :, None]; ex = np.maximum(v - v_cap, 0)
    ref = np.zeros_like(g); ref[:, 1:] = w * np.where(dist[:, 1:] <= d_cap, ex ** 2, 0).max(-1)
    d = np.abs(ref - g); rel = d / np.maximum(np.abs(ref), 1e-9)
    r = dict(file=os.path.basename(f), n=int(ref.size), n_pos_ref=int((ref > 0).sum()), n_pos_gpu=int((g > 0).sum()), max_abs=float(d.max()), max_rel_on_pos=float(rel[ref > 1e-6].max()) if (ref > 1e-6).any() else None,
             inv_dt_maxrel=float(np.abs(z["sc_inv_dt"] / inv_dt_ref - 1).max()), max_cost=float(ref.max()), v_max=float(v.max()), frac_near=float((dist[:, 1:] <= d_cap).mean()))
    res.append(r); print(json.dumps(r))
json.dump(res, open(f"{O}/t1_speed_check.json", "w"), indent=1)

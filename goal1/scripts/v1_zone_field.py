"""v1_zone_field.py - importable zone field (same definitions as v1_zone.py, which builds the maps)."""
import json, os
import numpy as np
H = os.path.expanduser("~"); G = f"{H}/effmass_mpc/goal1/b2a/g2"; OUT = f"{H}/effmass_mpc/goal1/zone"; O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"
tf = json.load(open(f"{G}/temp_transform.json")); R, t = np.array(tf["R"]), np.array(tf["t"]); L = json.load(open(f"{G}/lines.json")); AX = np.array(L["arm_axis"]); AX /= np.linalg.norm(AX)
Z = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_v2.npz"); SEG = Z["shape_segment"]; RAD = Z["shape_radius"].astype(float)
P0 = Z["p0"][0].astype(float) @ R.T + t; P1 = Z["p1"][0].astype(float) @ R.T + t; COM = Z["com"][0].astype(float) @ R.T + t
LAM = {g: R @ Z["inv"][0][g].astype(float) @ R.T for g in (14, 16, 18)}
SH = [k for k in range(24) if RAD[k] > 0 and SEG[k] in (14, 16, 18)]; HAND = [k for k in SH if SEG[k] == 18]
MV = np.load(f"{H}/effmass_mpc/goal1/data/human_b2a_reach_verts.npz"); V = MV["verts"][0].astype(float) @ R.T + t; F = MV["faces"]; LAB = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy")
FA = F[np.isin(LAB[F], [14, 16, 18]).all(1)]
GT = json.load(open(f"{O}/v1_gain_table.json")); ETA_L = float(np.mean([GT["poses"]["front_s0"]["eta_gain"], GT["poses"]["front_s1"]["eta_gain"]]))
DB, KG, MR, EM, MROB = 0.065, 0.10, 0.53, 0.25, 4.6
CONDS = {"uniform_small": "uniform-small (eta 6.5 cm)", "uniform_large": f"uniform-large (eta {ETA_L*100:.1f} cm)", "ours": "ours (gain, directional mu)"}


def field(Pts, cond):
    """Pts (N,3) -> f = min_s (d_s - eta_s), mu of the arg-min shape, index of the arg-min shape"""
    fbest = np.full(len(Pts), np.inf); mub = np.zeros(len(Pts)); kb = np.full(len(Pts), -1)
    for k in SH:
        a, b = P0[k], P1[k]; ab = b - a; tt = np.clip(((Pts - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = a + tt[:, None] * ab; v = q - Pts; n = np.linalg.norm(v, axis=1)
        d = n - RAD[k]; u = v / np.maximum(n, 1e-9)[:, None]; mh = 1.0 / np.maximum(np.einsum("ni,ij,nj->n", u, LAM[SEG[k]], u), 1e-12); mu = 1.0 / (1.0 / mh + 1.0 / MROB)
        e = np.full(len(Pts), DB) if cond == "uniform_small" else np.full(len(Pts), ETA_L) if cond == "uniform_large" else np.clip(DB + KG * np.maximum(np.sqrt(mu / MR) - 1, 0), DB, EM)
        f = d - e; m = f < fbest; fbest[m] = f[m]; mub[m] = mu[m]; kb[m] = k
    return fbest, mub, kb



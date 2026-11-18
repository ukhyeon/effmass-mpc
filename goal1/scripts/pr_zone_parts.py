"""pr_zone_parts.py - (part) direction-wise zone thickness per body part, 7 postures, previous (global mu_ref 0.53) vs proposed (part).

Thickness of a part's zone towards a robot approaching along unit direction u (400 Fibonacci directions on the sphere):
eta(u) = clamp(d_base + k max(sqrt(mu(u)/mu_ref) - 1, 0), d_base, eta_max), mu(u) = 1 / (u Lambda_h(part) u + 1/m_r), robot point
mass m_r 4.6 kg (as v2_zone_lib / on_wp2_zone). global: mu_ref 0.53 kg; part: mu_ref = mu_iso = 1 / (1/m_iso(part) + 1/m_r),
m_iso from goal1/data/iso_15066.json (feet = calf). Each part is measured with its own segment(s): hand = right hand (18),
forearm = right lower arm (16), upper arm = right upper arm (14), elbow = union of forearm and upper arm (max over the two),
torso = union of pelvis, abdomen, chest (0, 3, 8), head (12). Lambda_h from frame 0 of each posture file (on_wp2_postures.json).
Per posture and part: min, max, share of directions with eta > d_base. Writes goal1/part_regen/zone_parts.json.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g1_meff_gpu as G

H = os.path.expanduser("~"); O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; OUT = f"{H}/effmass_mpc/goal1/part_regen"
DB, K, EM, MR_G, MROB = 0.065, 0.10, 0.25, 0.53, 4.6
PARTS = {"hand": [18], "forearm": [16], "upper_arm": [14], "elbow": [16, 14], "torso": [0, 3, 8], "head": [12]}
MISO = G.iso_mass_19()


def fib(n):
    i = np.arange(n) + 0.5; phi = np.arccos(1 - 2 * i / n); th = np.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)


def eta(inv, segs, U, mode):
    out = np.full(len(U), DB)
    for g in segs:
        mh_inv = np.einsum("ni,ij,nj->n", U, inv[g], U); mu = 1.0 / (mh_inv + 1.0 / MROB)
        ref = MR_G if mode == "global" else 1.0 / (1.0 / MISO[g] + 1.0 / MROB)
        out = np.maximum(out, np.clip(DB + K * np.maximum(np.sqrt(mu / ref) - 1, 0), DB, EM))
    return out


def main():
    PO = json.load(open(f"{O}/on_wp2_postures.json")); U = fib(400); res = {}
    for name, p in PO.items():
        inv = np.load(p["file"])["inv"][0].reshape(19, 3, 3).astype(np.float64)
        res[name] = {}
        for part, segs in PARTS.items():
            res[name][part] = {}
            for mode in ("global", "part"):
                e = eta(inv, segs, U, mode)
                res[name][part][mode] = dict(min_cm=float(e.min() * 100), max_cm=float(e.max() * 100), above_base_share=float((e > DB + 1e-9).mean()))
    summ = {}
    for part in PARTS:
        summ[part] = {}
        for mode in ("global", "part"):
            mins = [res[n][part][mode]["min_cm"] for n in res]; maxs = [res[n][part][mode]["max_cm"] for n in res]; sh = [res[n][part][mode]["above_base_share"] for n in res]
            summ[part][mode] = dict(min_cm_range=[min(mins), max(mins)], max_cm_range=[min(maxs), max(maxs)], above_base_share_range=[min(sh), max(sh)])
    json.dump(dict(definition=__doc__, m_iso_kg={n: float(v) for n, v in zip(G.ISO_SEGMENTS, MISO)}, per_posture=res, summary=summ),
              open(f"{OUT}/zone_parts.json", "w"), indent=1)
    print(f"{'part':10s} {'mode':6s} {'min (cm)':>14s} {'max (cm)':>14s} {'share > base':>16s}")
    for part in PARTS:
        for mode in ("global", "part"):
            s = summ[part][mode]
            print(f"{part:10s} {mode:6s} {s['min_cm_range'][0]:6.1f}-{s['min_cm_range'][1]:5.1f} {s['max_cm_range'][0]:6.1f}-{s['max_cm_range'][1]:5.1f} "
                  f"{100*s['above_base_share_range'][0]:7.0f}-{100*s['above_base_share_range'][1]:3.0f} %")


if __name__ == "__main__":
    main()

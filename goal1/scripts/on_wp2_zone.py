"""on_wp2_zone.py - WP2 direction-wise zone thickness per posture (system python via g1_run_human.sh py; no MPC). Same zone definition as v2_zone_lib
(right-arm v2 shapes, robot point mass 4.6 kg isotropic, uniform eta 0.065, ours gain k 0.10, mu_ref 0.53, eta_max 0.25). Thickness along a direction from
an anchor inside the arm (moved onto the nearest shape axis if it is outside every shape): march from the anchor until the point leaves every arm shape (surface, r_s), continue until it leaves the zone (r_z);
thickness = r_z - r_s; mu and eta of the nearest shape at the surface point. Directions: tip front (hand pointing), back of hand, palm, hand inner
(toward the body midline) / outer side, forearm top / outer side, elbow (point of the elbow; for a straight arm the downward perpendicular), upper arm
outer side. mu range over 400 directions from the hand, forearm and upper-arm anchors; parts where eta reaches eta_max. Picks 4 postures for WP3 whose
ours thickness patterns differ most (greedy max-min distance on the direction vector, the reference reach pose excluded because its front and top
scenes are already in WP3). Outputs logs/meff_isaac/on_wp2_zone.json, goal1/overnight/wp2_zone_thickness.png, goal1/overnight/postures_preview.png."""
import json, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
sys.path.insert(0, os.path.dirname(__file__))
import v2_zone_lib as ZL
H = os.path.expanduser("~"); D_ = f"{H}/effmass_mpc/goal1/data"; O = f"{H}/effmass_mpc/goal1/logs/meff_isaac"; OV = f"{H}/effmass_mpc/goal1/overnight"
PO = json.load(open(f"{O}/on_wp2_postures.json")); LAB = np.load(f"{H}/effmass_mpc/goal1/b2a/vertex_segment.npy")
nz = lambda v: v / np.linalg.norm(v); U_ = np.array([0, 0, 1.0])
DIRS = ["tip_front", "hand_back", "palm", "hand_inner", "hand_outer", "forearm_top", "forearm_side", "elbow", "upper_arm_side"]


def armd(D, P):
    d = np.full(len(P), np.inf)
    for s in D["SH"]:
        a, b = D["P0"][s], D["P1"][s]; ab = b - a; tt = np.clip(((P - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); d = np.minimum(d, np.linalg.norm(a + tt[:, None] * ab - P, axis=1) - D["RAD"][s])
    return d


def inside(D, anchor):
    """anchor moved onto the axis of the nearest arm shape when it is not inside any arm shape (the hand COM lies outside the thin hand capsules)"""
    if armd(D, anchor[None])[0] < 0:
        return anchor
    best = None
    for s in D["SH"]:
        a, b = D["P0"][s], D["P1"][s]; ab = b - a; tt = np.clip(((anchor - a) @ ab) / max(ab @ ab, 1e-12), 0, 1); q = a + tt * ab; d = np.linalg.norm(q - anchor) - D["RAD"][s]
        if best is None or d < best[0]:
            best = (d, q)
    return best[1]


def thick_from(D, anchor, dirv, cond):
    dirv = nz(dirv); anchor = inside(D, np.asarray(anchor, float)); rr = np.arange(0, 0.8, 0.0005); P = anchor[None] + rr[:, None] * dirv[None]
    ds = armd(D, P); i_s = int(np.argmax(ds >= 0)); f, mu, kb = ZL.field(D, P, cond); out = f > 0; out[:i_s] = False; i_z = int(np.argmax(out))
    mu_s = float(mu[i_s]); eta = float(ZL.eta_of(np.array([mu_s]), cond, 0.10)[0]); return dict(t=float(rr[i_z] - rr[i_s]), mu=mu_s, eta=eta, shape=int(kb[i_s]), sat=bool(eta >= ZL.EM - 1e-9))


def frames(D, p):
    sh, el, wr = np.array(p["shoulder"]), np.array(p["elbow"]), np.array(p["wrist"]); tip = np.array(p["tip"]); pal = nz(np.array(p["palm_normal"]))
    pt = nz(tip - wr); hc = D["COM"][18]; mid = D["COM"][8]; side = nz(np.cross(pt, pal)); inner = side if (mid - hc) @ side > 0 else -side
    fa = nz(wr - el); ua = nz(el - sh); top = U_ - (U_ @ fa) * fa; top = nz(top) if np.linalg.norm(top) > 0.2 else nz(-pal - (-pal @ fa) * fa)
    fs = nz(np.cross(fa, top)); fs = fs if (fs @ (wr - mid)) > 0 else -fs
    eo = ua - fa; eo = nz(eo) if np.linalg.norm(eo) > 0.35 else nz(-top)
    us = U_ - (U_ @ ua) * ua; us = nz(np.cross(ua, nz(us) if np.linalg.norm(us) > 0.2 else np.array([1.0, 0, 0]))); us = us if (us @ (el - mid)) > 0 else -us
    fm, um = (el + wr) / 2, (sh + el) / 2; hm = (hc + tip) / 2
    return {"tip_front": (hm, pt), "hand_back": (hc, -pal), "palm": (hc, pal), "hand_inner": (hc, inner), "hand_outer": (hc, -inner), "forearm_top": (fm, top),
            "forearm_side": (fm, fs), "elbow": (el, eo), "upper_arm_side": (um, us)}


def fib(n):
    i = np.arange(n) + 0.5; ph = np.arccos(1 - 2 * i / n); th = np.pi * (1 + 5 ** 0.5) * i; return np.stack([np.cos(th) * np.sin(ph), np.sin(th) * np.sin(ph), np.cos(ph)], 1)


if __name__ == "__main__":
    res = {}
    for name, p in PO.items():
        D = ZL.load(p["file"]); fr = frames(D, p); r = {"directions": {}}
        for dn in DIRS:
            a, v = fr[dn]; r["directions"][dn] = {c: thick_from(D, a, v, c) for c in ("uniform", "ours")}
            r["directions"][dn]["dir"] = nz(v).round(3).tolist()
        mus = {}
        for part, anc in (("hand", D["COM"][18]), ("forearm", fr["forearm_top"][0]), ("upper_arm", fr["upper_arm_side"][0])):
            vals = [thick_from(D, anc, v, "ours") for v in fib(400)]; seg = {"hand": 18, "forearm": 16, "upper_arm": 14}[part]
            own = [x for x in vals if D["SEG"][x["shape"]] == seg] or vals
            mus[part] = dict(mu_min=float(min(x["mu"] for x in own)), mu_max=float(max(x["mu"] for x in own)), eta_max_frac=float(np.mean([x["sat"] for x in own])),
                             thick_min=float(min(x["t"] for x in own)), thick_max=float(max(x["t"] for x in own)))
        r["mu_by_part"] = mus; r["eta_at_max_parts"] = [k for k, v in mus.items() if v["eta_max_frac"] > 0]; res[name] = r
        print(name, " ".join(f"{dn}:{100*r['directions'][dn]['ours']['t']:.1f}" for dn in DIRS), {k: (round(v["mu_min"], 2), round(v["mu_max"], 2), round(v["eta_max_frac"], 2)) for k, v in mus.items()}, flush=True)
    # pick 4 postures (reference excluded) by greedy max-min distance of the ours thickness vector
    cand = [n for n in res if n != "P1_reach" and PO[n]["plausible"]]; vec = {n: np.array([res[n]["directions"][d]["ours"]["t"] for d in DIRS]) for n in res}
    dist = lambda a, b: float(np.linalg.norm(vec[a] - vec[b]))
    first = max(cand, key=lambda n: dist(n, "P1_reach")); pick = [first]
    while len(pick) < min(4, len(cand)):
        pick.append(max([n for n in cand if n not in pick], key=lambda n: min(dist(n, m) for m in pick + ["P1_reach"])))
    res["_pick"] = dict(postures=pick, rule="greedy max-min Euclidean distance of the 9-direction ours thickness vector, distance to the reference reach pose included",
                        pairwise_cm={f"{a}-{b}": round(100 * dist(a, b), 1) for i, a in enumerate(pick + ["P1_reach"]) for b in (pick + ["P1_reach"])[i + 1:]})
    json.dump(res, open(f"{O}/on_wp2_zone.json", "w"), indent=1); print("pick", pick)
    # thickness figure
    names = list(PO); fig, ax = plt.subplots(1, 1, figsize=(15, 6)); w = 0.8 / len(names); cm = plt.get_cmap("tab10")
    for j, n in enumerate(names):
        xs = np.arange(len(DIRS)) + (j - len(names) / 2 + 0.5) * w
        ax.bar(xs, [100 * res[n]["directions"][d]["ours"]["t"] for d in DIRS], w * 0.9, color=cm(j), label=n + (" *" if n in pick else ""))
        sat = [res[n]["directions"][d]["ours"]["sat"] for d in DIRS]
        for x_, d, s_ in zip(xs, DIRS, sat):
            if s_:
                ax.text(x_, 100 * res[n]["directions"][d]["ours"]["t"] + 0.3, "max", ha="center", fontsize=6, rotation=90)
    ax.axhline(6.5, color="#555", ls="--", lw=1); ax.text(len(DIRS) - 0.5, 6.8, "uniform 6.5 cm", ha="right", fontsize=8, color="#555")
    ax.set_xticks(range(len(DIRS))); ax.set_xticklabels(DIRS); ax.set_ylabel("ours zone thickness (cm)"); ax.legend(fontsize=8, ncol=4, frameon=False)
    ax.set_title("WP2 zone thickness by direction and posture (ours k 0.10; * = picked for WP3; 'max' = eta at eta_max 25 cm)"); ax.grid(axis="y", alpha=0.3)
    fig.tight_layout(); fig.savefig(f"{OV}/wp2_zone_thickness.png", dpi=130); plt.close(fig)
    # preview: every posture from the same viewpoint
    n = len(names); fig = plt.figure(figsize=(5.2 * ((n + 1) // 2), 11))
    for j, nm in enumerate(names):
        M = np.load(f"{D_}/human_on_{nm}_mesh.npz"); V, F = M["verts"].astype(float), M["faces"]; ax = fig.add_subplot(2, (n + 1) // 2, j + 1, projection="3d")
        arm = np.isin(LAB[F], [14, 16, 18]).all(1); keep = np.ones(len(F), bool)
        ax.add_collection3d(Poly3DCollection(V[F[keep & ~arm]], facecolor="#bdb8b0", edgecolor="none", alpha=0.18))
        ax.add_collection3d(Poly3DCollection(V[F[arm]], facecolor="#d2691e", edgecolor="none", alpha=0.9))
        ax.plot([0.38, 0.38], [-0.37, 0.37], [0.45, 0.45], color="#2a78d6", lw=2)
        ax.set_xlim(0.3, 1.45); ax.set_ylim(-0.55, 0.8); ax.set_zlim(-0.35, 1.0); ax.set_box_aspect((1.15, 1.35, 1.35)); ax.view_init(elev=14, azim=-125)
        p = PO[nm]; ax.set_title(f"{nm}{' *' if nm in pick else ''}\nelbow {p['elbow_flexion_deg']:.0f} deg, {'ok' if p['plausible'] else 'CHECK'}", fontsize=10)
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
    fig.suptitle("WP2 postures, same viewpoint (blue line = robot TCP path of the front scene; orange = right arm; * = picked for WP3)", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95]); fig.savefig(f"{OV}/postures_preview.png", dpi=110); plt.close(fig)

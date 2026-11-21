"""step2b_mpc_isaac_field.py - launcher used for the public Isaac examples of analysis/field (2026-11-20).
Runs env_check/scripts/step2b_mpc_isaac.py UNCHANGED, with the risk field of goal1/scripts/g1_meff_gpu.py switched on
(MeffChannel.set_field(1, W)) right after every MeffChannel.set_cost call. Uniform and effective-mass avoidance get the same field;
the field width follows each method's clearance eta (hill W (1 - (d - eta) / eta)^2 for eta <= d < 2 eta, W inside eta).
Own option (removed before step2b parses argv):
  --gh-hf-wt W   risk-field height W (cost units of --meff-w); 0 or absent = field off = plain step2b
g1_meff_gpu is patched at import time through a meta-path hook (no torch import before step2b starts the SimulationApp).
The file name contains "step2b_mpc_isaac" on purpose, so process checks that look for step2b see this run.
Example (V2 front, effective-mass avoidance + risk field, seed 0; uniform avoidance: --meff-gain 0.0):
  env_check/scripts/run_isaac_ros.sh analysis/field/tools/step2b_mpc_isaac_field.py --gh-hf-wt 100 --scene g1 --robot fr3v2_1 --no-diag \
    --warmup 10 --tag v2_front_pf_s0 --ros --ros-pose-topic none --human --human-mode full --human-source precomputed \
    --scene g2 --shapes v2 --human-pre goal1/data/human_b2a_reach.npz --human-dir goal1/b2a/g2 --human-viz none --u-def nearest \
    --meff on --meff-mode gain --meff-dbase 0.065 --meff-muref 0.53 --meff-eta-max 0.25 --meff-agg max_all --meff-muref-mode part \
    --b1-log 1 --cost-agg 1 --profile t1 --scene-json goal1/b2a/g2/scene_b2a4_front_0.38h1.json --single-pass 1 --meff-gain 0.10 \
    --mppi-seed 0 --steps 304
  (the human input file is not published; see the repository README, "미포함 항목")"""
import importlib.util
import os
import runpy
import sys

S2B = os.path.expanduser("~/effmass_mpc/env_check/scripts/step2b_mpc_isaac.py")
W = 0.0; rest = []; a = sys.argv[1:]; i = 0
while i < len(a):
    if a[i] == "--gh-hf-wt":
        W = float(a[i + 1]); i += 2; continue
    rest.append(a[i]); i += 1


def _patch(mod):
    orig = mod.MeffChannel.set_cost

    def set_cost(self, *x, **k):
        r = orig(self, *x, **k)
        self.set_field(1, W)
        return r
    mod.MeffChannel.set_cost = set_cost
    print(f"[step2b] (gh_field) risk field on: W {W} (hill eta .. 2 eta, W inside eta)", flush=True)


class _Hook:
    def find_spec(self, name, path=None, target=None):
        if name != "g1_meff_gpu":
            return None
        sys.meta_path.remove(self)
        spec = importlib.util.find_spec(name)
        if spec is None:
            return None
        ex = spec.loader.exec_module

        def exec_module(m):
            ex(m)
            _patch(m)
        spec.loader.exec_module = exec_module
        return spec


if W > 0:
    sys.meta_path.insert(0, _Hook())
sys.argv = [S2B] + rest
runpy.run_path(S2B, run_name="__main__")

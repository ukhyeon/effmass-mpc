"""on_bitcmp.py <tagA> <tagB> [n] - compare executed joint commands (steps csv) of two runs, bit by bit over the first n steps (default: common length)."""
import csv, json, os, re, sys
import numpy as np
O = os.path.expanduser("~/effmass_mpc/goal1/logs/meff_isaac")


def result(tag):
    log = open(f"{O}/{tag}.log", errors="replace").read()
    return json.load(open(re.findall(r"saved (\S+_result_\S+\.json)", log)[-1]))


def qcmd(tag):
    rows = list(csv.DictReader(open(result(tag)["steps_csv"])))
    return np.array([[float(x[f"q{i}"]) for i in range(7)] for x in rows])


def cmp(a, b, n=None):
    qa, qb = qcmd(a), qcmd(b); m = min(len(qa), len(qb)) if n is None else n
    eq = bool(len(qa) >= m and len(qb) >= m and np.array_equal(qa[:m], qb[:m]))
    d = dict(a=a, b=b, n=int(m), len_a=len(qa), len_b=len(qb), bit_equal=eq, max_abs_diff=float(np.abs(qa[:m] - qb[:m]).max()) if m else None)
    if not eq and m:
        diff = np.where(np.any(qa[:m] != qb[:m], axis=1))[0]; d["first_diff_step"] = int(diff[0])
    return d


if __name__ == "__main__":
    print(json.dumps(cmp(sys.argv[1], sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else None)))

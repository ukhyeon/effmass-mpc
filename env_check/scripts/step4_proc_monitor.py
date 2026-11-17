"""step4_proc_monitor.py --pid N --out csv [--stop-file F] — 프로세스 CPU%(1 s 간 utime+stime 증가), RSS, 그 pid 의 GPU 메모리(nvidia-smi), GPU 전체 사용률"""
import argparse, os, subprocess, time
ap = argparse.ArgumentParser(); ap.add_argument("--pid", type=int, required=True); ap.add_argument("--out", required=True)
ap.add_argument("--stop-file", default=None); a = ap.parse_args()
tck = os.sysconf("SC_CLK_TCK")
def cpu():
    f = open(f"/proc/{a.pid}/stat").read().rsplit(")", 1)[1].split(); return (int(f[11]) + int(f[12])) / tck
def rss():
    for l in open(f"/proc/{a.pid}/status"):
        if l.startswith("VmRSS"): return int(l.split()[1]) / 1024
def gpu():
    mem = None
    try:
        out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=5).stdout
        for l in out.strip().splitlines():
            p, m = [x.strip() for x in l.split(",")]
            if int(p) == a.pid: mem = int(m)
        util = subprocess.run(["nvidia-smi", "--query-gpu=utilization.gpu", "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=5).stdout.strip()
        return mem, int(util)
    except Exception:
        return mem, None
with open(a.out, "w") as f:
    f.write("t,cpu_pct,rss_mib,gpu_mem_mib,gpu_util_pct\n")
    c0, t0 = cpu(), time.time()
    while os.path.exists(f"/proc/{a.pid}") and not (a.stop_file and os.path.exists(a.stop_file)):
        time.sleep(1.0)
        try:
            c1, t1 = cpu(), time.time(); m, u = gpu()
            f.write(f"{t1:.3f},{100 * (c1 - c0) / (t1 - t0):.1f},{rss():.1f},{'' if m is None else m},{'' if u is None else u}\n"); f.flush()
            c0, t0 = c1, t1
        except FileNotFoundError:
            break

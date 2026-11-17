"""step4_sys_monitor.py --proc label=pattern [...] --out csv --stop-file F — 1 s 간격 시스템 기록 (시스템 python3 -B, 새 패키지 없음)

  프로세스별: pgrep -f <pattern> 중 RSS 가 가장 큰 pid (bash 래퍼 대신 실제 python 프로세스). CPU% = 1 s 간 utime+stime 증가 (100% = 1 스레드),
              RSS, 그 pid 의 GPU 메모리 (nvidia-smi --query-compute-apps)
  --proc label=pid:N 이면 그 pid 를 그대로 기록 (측정과 무관한 다른 프로그램 부하 기록용)
  코어별: /proc/stat cpu0..cpuN 의 1 s 간 (전체 − idle − iowait) / 전체 × 100
  GPU: utilization.gpu, memory.used (nvidia-smi)
"""
import argparse, os, subprocess, time
ap = argparse.ArgumentParser(); ap.add_argument("--proc", action="append", default=[]); ap.add_argument("--out", required=True)
ap.add_argument("--stop-file", required=True); a = ap.parse_args()
tck = os.sysconf("SC_CLK_TCK"); NC = os.cpu_count()
labels = [p.split("=", 1)[0] for p in a.proc]; pats = {p.split("=", 1)[0]: p.split("=", 1)[1] for p in a.proc}
pid = {l: None for l in labels}; last = {l: None for l in labels}
def rss(p):
    try:
        for l in open(f"/proc/{p}/status"):
            if l.startswith("VmRSS"): return int(l.split()[1]) / 1024
    except Exception: return None
def find(l):
    if pats[l].startswith("pid:"):
        q = int(pats[l][4:]); return q if os.path.exists(f"/proc/{q}") else None
    out = subprocess.run(["pgrep", "-f", pats[l]], capture_output=True, text=True).stdout.split()
    c = [(rss(int(x)) or 0, int(x)) for x in out if int(x) != os.getpid()]
    return max(c)[1] if c and max(c)[0] > 50 else None
def nthr(p):
    for l in open(f"/proc/{p}/status"):
        if l.startswith("Threads"): return int(l.split()[1])
def pcpu(p):
    f = open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split(); return (int(f[11]) + int(f[12])) / tck, int(f[36])
def cores():
    r = {}
    for l in open("/proc/stat"):
        if l.startswith("cpu") and l[3].isdigit():
            x = l.split(); v = list(map(int, x[1:9])); r[int(x[0][3:])] = (sum(v), v[3] + v[4])
    return r
def gpu():
    try:
        u = subprocess.run(["nvidia-smi", "--query-gpu=utilization.gpu,memory.used", "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=5).stdout.strip().split(",")
        o = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,used_memory", "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=5).stdout
        pm = {int(x.split(",")[0]): int(x.split(",")[1]) for x in o.strip().splitlines() if x.strip()}
        return int(u[0]), int(u[1]), pm
    except Exception:
        return None, None, {}
hdr = ["t", "gpu_util_pct", "gpu_mem_mib"] + sum([[f"{l}_pid", f"{l}_cpu_pct", f"{l}_last_cpu", f"{l}_rss_mib", f"{l}_vram_mib", f"{l}_threads"] for l in labels], []) + [f"core{i}" for i in range(NC)]
with open(a.out, "w") as f:
    f.write(",".join(hdr) + "\n")
    c0 = cores(); t0 = time.time()
    while not os.path.exists(a.stop_file):
        time.sleep(1.0)
        t1 = time.time(); c1 = cores(); gu, gm, pm = gpu()
        row = [f"{t1:.3f}", "" if gu is None else gu, "" if gm is None else gm]
        for l in labels:
            if pid[l] is None or not os.path.exists(f"/proc/{pid[l]}"):
                pid[l] = find(l); last[l] = None
            p = pid[l]; cp = lc = r = th = ""
            if p:
                try:
                    c, lc = pcpu(p); r = f"{rss(p):.1f}"; th = nthr(p)
                    if last[l] is not None: cp = f"{100 * (c - last[l][0]) / (t1 - last[l][1]):.1f}"
                    last[l] = (c, t1)
                except Exception: pid[l] = None
            row += [p or "", cp, lc, r, pm.get(p, "") if p else "", th]
        for i in range(NC):
            dt = c1[i][0] - c0[i][0]; di = c1[i][1] - c0[i][1]
            row.append(f"{100 * (dt - di) / dt:.1f}" if dt > 0 else "")
        f.write(",".join(map(str, row)) + "\n"); f.flush()
        c0, t0 = c1, t1

"""step3_analyze_comm.py <접두어> — 3-1/3-2 통신 결과 요약 (시스템 python3)
  sys→isaac: <p>_sys_sent.csv 대 <p>_isaac_recv_{pose,dyn}.csv  (kind 별 누락률, 내용 일치, 지연)
  isaac→sys: <p>_isaac_sent_js.csv 대 <p>_sys_recv_js.csv (시스템 노드 수신 구간 안에서 발행된 것만 대상)
"""
import csv, json, sys, os
p = sys.argv[1]
def rd(f): return list(csv.DictReader(open(f))) if os.path.exists(f) else []
def stats(xs):
    if not xs: return "n=0"
    s = sorted(xs); n = len(s); p95 = s[max(0, int(round(0.95 * n)) - 1)]
    return f"n={n} 평균 {sum(s)/n:.3f} ms / p95 {p95:.3f} ms / 최대 {s[-1]:.3f} ms"
out = {}
sent = rd(p + "_sys_sent.csv")
for topic in ("pose", "dyn"):
    recv = rd(f"{p}_isaac_recv_{topic}.csv")
    for kind in sorted({r["kind"] for r in sent if r["kind"].startswith(topic)}):
        s = {int(r["seq"]) for r in sent if r["kind"] == kind}
        rr = [r for r in recv if r["kind"] == kind]
        got = {int(r["seq"]) for r in rr}
        dup = len(rr) - len(got)
        lat = [(int(r["t_recv_ns"]) - int(r["stamp_ns"])) / 1e6 for r in rr]
        okn = sum(int(r["ok"]) for r in rr)
        loss = 1 - len(got & s) / len(s) if s else float("nan")
        line = f"{kind:7s} 발행 {len(s)} 수신 {len(got & s)} 누락률 {loss*100:.3f}% 중복 {dup} 내용일치 {okn}/{len(rr)} 지연 {stats(lat)}"
        print("[comm] sys→isaac " + line); out[kind] = dict(sent=len(s), recv=len(got & s), loss=loss, dup=dup, ok=okn, n=len(rr), lat=lat)
js_sent = rd(p + "_isaac_sent_js.csv"); js_recv = rd(p + "_sys_recv_js.csv")
meta = json.load(open(p + "_sys_meta.json")) if os.path.exists(p + "_sys_meta.json") else {}
if js_sent and meta:
    lo, hi = meta["listen_start_ns"] + 500_000_000, meta["listen_end_ns"] - 500_000_000
    target = {int(r["seq"]) for r in js_sent if lo <= int(r["t_send_ns"]) <= hi}
    got = {int(r["seq"]) for r in js_recv}
    rr = [r for r in js_recv if int(r["seq"]) in target]
    lat = [(int(r["t_recv_ns"]) - int(r["stamp_ns"])) / 1e6 for r in rr]
    oks = [r["ok"] for r in rr if r["ok"] != ""]
    loss = 1 - len(target & got) / len(target) if target else float("nan")
    print(f"[comm] isaac→sys js      대상(수신 구간 내 발행) {len(target)} 수신 {len(target & got)} 누락률 {loss*100:.3f}% "
          f"중복 {len(js_recv) - len(got)} 내용일치 {sum(int(x) for x in oks)}/{len(oks) if oks else 'log'} 지연 {stats(lat)}")
    out["js"] = dict(target=len(target), recv=len(target & got), loss=loss)
json.dump({k: {kk: vv for kk, vv in v.items() if kk != "lat"} for k, v in out.items()}, open(p + "_summary.json", "w"), indent=1)

"""step3_analyze_mpc.py <접두어> — 3-2 (MPC 동시 실행) ROS 통신 요약 (시스템 python3)
  HumanPose: 시스템 발행 중 Isaac 측정 루프 구간 [loop_start, loop_end - 0.1 s] 안에서 발행된 것만 대상
  JointState: Isaac 발행 중 시스템 노드 수신 구간 안의 것만 대상, 순번별 관절값을 정확히 비교
"""
import csv, json, sys
p = sys.argv[1]
rd = lambda f: list(csv.DictReader(open(f)))
def stats(xs):
    if not xs: return dict(n=0)
    s = sorted(xs); n = len(s); return dict(n=n, mean=sum(s)/n, p95=s[max(0, int(round(0.95*n))-1)], max=s[-1])
im = json.load(open(p + "_isaac_meta.json")); sm = json.load(open(p + "_sys_meta.json"))
lo, hi = im["loop_start_ns"], im["loop_end_ns"] - 100_000_000
sent = [r for r in rd(p + "_sys_sent.csv") if r["kind"] == "pose30" and lo <= int(r["t_send_ns"]) <= hi]
recv = [r for r in rd(p + "_isaac_recv_pose.csv") if r["kind"] == "pose30"]
s = {int(r["seq"]) for r in sent}; got = {int(r["seq"]): r for r in recv}
inw = [got[k] for k in s if k in got]
lat = [(int(r["t_recv_ns"]) - int(r["stamp_ns"])) / 1e6 for r in inw]
pose = dict(target=len(s), recv=len(inw), loss_pct=100 * (1 - len(inw) / len(s)) if s else None,
            ok=sum(int(r["ok"]) for r in inw), dup=len(recv) - len(got), latency_ms=stats(lat),
            sent_rate_hz=(len(s) / ((hi - lo) / 1e9)))
js_s = {int(r["seq"]): r for r in rd(p + "_isaac_sent_js.csv")
        if sm["listen_start_ns"] + 500_000_000 <= int(r["t_send_ns"]) <= sm["listen_end_ns"] - 500_000_000}
js_r = {int(r["seq"]): r for r in rd(p + "_sys_recv_js.csv")}
both = [k for k in js_s if k in js_r]
same = sum(1 for k in both if all(float(js_s[k][f"p{i}"]) == float(js_r[k][f"p{i}"]) for i in range(9)))
jl = [(int(js_r[k]["t_recv_ns"]) - int(js_r[k]["stamp_ns"])) / 1e6 for k in both]
js_all = rd(p + "_isaac_sent_js.csv")
dur = (int(js_all[-1]["t_send_ns"]) - int(js_all[0]["t_send_ns"])) / 1e9 if len(js_all) > 1 else 0
js = dict(target=len(js_s), recv=len(both), loss_pct=100 * (1 - len(both) / len(js_s)) if js_s else None, values_equal=same,
          latency_ms=stats(jl), wall_rate_hz=(len(js_all) - 1) / dur if dur else None)
# 예열 구간(측정 통계와 별도): 예열 시작~끝 사이에 발행된 HumanPose
warm = None
if im.get("warmup_start_ns"):
    wlo, whi = im["warmup_start_ns"], im["warmup_end_ns"]
    ws = {int(r["seq"]) for r in rd(p + "_sys_sent.csv") if r["kind"] == "pose30" and wlo <= int(r["t_send_ns"]) <= whi}
    wr = [got[k] for k in ws if k in got]
    wl = [(int(r["t_recv_ns"]) - int(r["stamp_ns"])) / 1e6 for r in wr]
    warm = dict(window_s=(whi - wlo) / 1e9, target=len(ws), recv=len(wr), loss_pct=100 * (1 - len(wr) / len(ws)) if ws else None,
                latency_ms=stats(wl))
out = dict(pose=pose, js=js, warmup_window_pose=warm)
json.dump(out, open(p + "_summary.json", "w"), indent=1)
f = lambda d: f"n={d['n']} 평균 {d['mean']:.3f} / p95 {d['p95']:.3f} / 최대 {d['max']:.3f} ms" if d.get("n") else "n=0"
print(f"[ros] HumanPose 30Hz (측정 루프 구간): 대상 {pose['target']} 수신 {pose['recv']} 누락률 {pose['loss_pct']:.3f}% "
      f"가벼운 검사 일치 {pose['ok']}/{pose['recv']} 중복 {pose['dup']} 지연 {f(pose['latency_ms'])} (발행률 {pose['sent_rate_hz']:.2f} Hz)")
if warm:
    print(f"[ros] (측정 통계와 별도) 예열 구간 {warm['window_s']:.2f}s 동안 발행된 HumanPose: 대상 {warm['target']} 수신 {warm['recv']} "
          f"누락률 {warm['loss_pct'] if warm['loss_pct'] is None else round(warm['loss_pct'],3)}% 지연 {f(warm['latency_ms'])}")
print(f"[ros] JointState (물리 스텝마다, 실제 {js['wall_rate_hz']:.2f} Hz): 대상 {js['target']} 수신 {js['recv']} 누락률 {js['loss_pct']:.3f}% "
      f"관절값 일치 {js['values_equal']}/{js['recv']} 지연 {f(js['latency_ms'])}")

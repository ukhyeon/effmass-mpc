"""step5_sensing_redirect.py — sensing_node 쓰기를 실행 중에만 step5/<tag>/ 아래로 돌린다 (파일 수정 없음)

install(out_dir) 이 하는 일 (hrc_body_tracking.sensing_node import 전에 호출):
  1) 감사 훅: ~/hrc_ws, ~/realtime_smpl, ~/indy_ws, ~/.local, /opt/ros 아래 파이썬 쓰기 open·mkdir·rename·remove·rmdir 차단·기록
     (cv2.imwrite 같은 C++ 쓰기는 훅에 안 잡힘 → 아래 3) 의 경로 바꾸기 + 실행 전후 파일 목록 비교로 확인)
  2) PerfCsvLogger.__init__ 감싸기: path → <out>/sensing_perf/<같은 파일명>
  3) sensing_node 모듈 안의 이름 os·glob·cv2 만 대리 객체로 바꿈 (다른 모듈·전역 os 는 그대로)
       경로 접두어 ~/hrc_ws/vis_output → <out>/vis_output, ~/hrc_ws/body_crop → <out>/body_crop, ~/hrc_ws/pose_npz → <out>/pose_npz
       대상 함수: os.makedirs, os.remove, glob.glob, cv2.imwrite (나머지 속성은 원래 것으로 넘김)
  4) HRCSensingNode.__init__ 감싸기: 원래 __init__ 뒤 self.vis_dp, self.pose_npz_dir 를 <out>/... 로
"""
import os, sys
HOME = os.path.expanduser("~")
HRC = os.path.realpath(f"{HOME}/hrc_ws") + os.sep
PROTECT = [HRC] + [os.path.realpath(p) + os.sep for p in (f"{HOME}/realtime_smpl", f"{HOME}/indy_ws", f"{HOME}/.local", "/opt/ros")]
BLOCKED, REWRITES = [], {}


def _prot(p):
    try:
        p = os.path.realpath(os.fsdecode(p))
    except Exception:
        return None
    return p if any(p.startswith(x) for x in PROTECT) else None


def _audit(event, args):
    try:
        if event == "open":
            path, mode, flags = args[0], args[1], (args[2] if len(args) > 2 else 0)
            wr = os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_APPEND | os.O_TRUNC
            writing = (isinstance(mode, str) and any(c in mode for c in "wax+")) or (mode is None and isinstance(flags, int) and (flags & wr))
            if writing and isinstance(path, (str, bytes, os.PathLike)) and _prot(path):
                BLOCKED.append((event, _prot(path))); print(f"[sensing-wrapper] BLOCKED {event} {_prot(path)} mode={mode}", flush=True)
                raise PermissionError(f"step5 wrapper: write blocked: {path}")
        elif event in ("os.mkdir", "os.rename", "os.remove", "os.rmdir", "shutil.rmtree", "os.unlink"):
            if isinstance(args[0], (str, bytes, os.PathLike)) and _prot(args[0]):
                BLOCKED.append((event, _prot(args[0]))); print(f"[sensing-wrapper] BLOCKED {event} {_prot(args[0])}", flush=True)
                raise PermissionError(f"step5 wrapper: {event} blocked: {args[0]}")
    except PermissionError:
        raise
    except Exception:
        pass


def install(out):
    out = os.path.abspath(out)
    MAP = {f"{HOME}/hrc_ws/vis_output": f"{out}/vis_output", f"{HOME}/hrc_ws/body_crop": f"{out}/body_crop",
           f"{HOME}/hrc_ws/pose_npz": f"{out}/pose_npz"}
    for d in list(MAP.values()) + [f"{out}/sensing_perf"]:
        os.makedirs(d, exist_ok=True)

    def rw(p, what):
        s = os.fsdecode(p)
        for a, b in MAP.items():
            if s == a or s.startswith(a + "/"):
                n = b + s[len(a):]
                REWRITES[what] = REWRITES.get(what, 0) + 1
                if REWRITES[what] <= 3:
                    print(f"[sensing-wrapper] {what}: {s} -> {n}", flush=True)
                return n
        return p

    import glob as _glob, cv2 as _cv2

    class _OS:
        def __getattr__(self, k): return getattr(os, k)
        def makedirs(self, p, *a, **k): return os.makedirs(rw(p, "os.makedirs"), *a, **k)
        def remove(self, p, *a, **k): return os.remove(rw(p, "os.remove"), *a, **k)

    class _GLOB:
        def __getattr__(self, k): return getattr(_glob, k)
        def glob(self, p, *a, **k): return _glob.glob(rw(p, "glob.glob"), *a, **k)

    class _CV2:
        def __getattr__(self, k): return getattr(_cv2, k)
        def imwrite(self, p, *a, **k): return _cv2.imwrite(rw(p, "cv2.imwrite"), *a, **k)

    import hrc_body_tracking.perf_logger as _pl
    _orig = _pl.PerfCsvLogger.__init__

    def _perf_init(self, path, fieldnames, flush_every=30):
        new = os.path.join(out, "sensing_perf", os.path.basename(path))
        print(f"[sensing-wrapper] PerfCsvLogger 경로 변경: {path} -> {new}", flush=True)
        return _orig(self, new, fieldnames, flush_every)

    _pl.PerfCsvLogger.__init__ = _perf_init
    sys.addaudithook(_audit)
    import hrc_body_tracking.sensing_node as S
    S.os, S.glob, S.cv2 = _OS(), _GLOB(), _CV2()
    _orig_node = S.HRCSensingNode.__init__

    def _node_init(self, *a, **k):
        _orig_node(self, *a, **k)
        print(f"[sensing-wrapper] 생성 직후 vis_dp {self.vis_dp} -> {out}/vis_output, pose_npz_dir {self.pose_npz_dir} -> {out}/pose_npz; "
              f"visualization {self.visualization_enabled}, imshow {self.vis_config.get('imshow')}, imsave {self.vis_config.get('imsave')}, "
              f"vis_smpl {self.vis_config.get('vis_smpl')}, device {self.device}", flush=True)
        self.vis_dp = f"{out}/vis_output"
        if os.environ.get("STEP5_SENSING_NOVERTS") == "1":
            # (5-5 J) 이 실행에서만 정점 계산까지 끔: timer_callback 의 "visualization_enabled and cliff_beta" 분기를 건너뛰게 함
            #   → smpl_output 없음 → HumanPose.smpl_vertices·smpl_joint 비움, smpl_betas 는 CLIFF betas 그대로 (설정 파일 수정 없음)
            self.visualization_enabled = False
            print("[sensing-wrapper] J: 이 실행에서만 visualization_enabled False → 정점 계산·렌더 없음 (렌더 스레드는 생성됐으나 패킷 없음)", flush=True)
        if os.environ.get("STEP5_SENSING_NOVIS") == "1":
            # 사용자 결정(5-2 재개 시): 실시간 시각화 끔. 정점 계산(smpl_vertices)은 유지하고 화면 렌더·창·jpg 저장만 뺌
            self.vis_config["imshow"] = False; self.vis_config["imsave"] = False
            self._novis_dropped = 0
            def _drop(packet, _self=self):
                _self._novis_dropped += 1
            self._submit_render_packet = _drop
            print(f"[sensing-wrapper] B: 이 실행에서만 imshow·imsave False, 렌더 패킷 버림(렌더 스레드 대기만) "
                  f"(enable {self.visualization_enabled}, vis_smpl {self.vis_config.get('vis_smpl')} 유지 → smpl_vertices 계산·발행 유지)", flush=True)
        self.pose_npz_dir = f"{out}/pose_npz"

    S.HRCSensingNode.__init__ = _node_init
    print(f"[sensing-wrapper] module {S.__file__}, out {out}, PYTHONDONTWRITEBYTECODE={os.environ.get('PYTHONDONTWRITEBYTECODE')} "
          f"ROS_LOG_DIR={os.environ.get('ROS_LOG_DIR')} DISPLAY={os.environ.get('DISPLAY')} cwd {os.getcwd()}", flush=True)
    return S

"""step4_dyn_node_wrapper.py — 설치된 human_dynamics_node 를 그대로 실행하되, 성능 로거 저장 경로만 우리 폴더로 바꾼다.

  - 노드 소스·계산은 바꾸지 않는다 (설치본 hrc_body_tracking.human_dynamics_node.main() 을 그대로 호출).
  - hrc_body_tracking.perf_logger.PerfCsvLogger.__init__ 만 감싸서 path → ~/effmass_mpc/env_check/step4/node_perf/<같은 파일명>
  - 안전장치: sys.addaudithook 으로 ~/hrc_ws 아래 쓰기 모드 open / mkdir / rename / remove 시도를 막고 기록 (PermissionError)
  - (4-3 C) 환경 변수 STEP4_NO_FIND_WORST=1 이면 DynamicsNode.__init__ 을 감싸, 원래 __init__ (renat.yaml 읽기 포함)이 끝난 직후
    이 프로세스 안에서만 self.find_worst_vis_enabled = False 로 바꾼다. timer_callback 이 매 계산 이 값을 확인하므로
    publish_worst_target_meff_direction (64방향 최악 유효질량, /hrc/worst_head_meff 발행) 이 호출되지 않는다. 파일은 바꾸지 않는다.
  - (4-3c E) 노드 실행 환경에만 CUDA_VISIBLE_DEVICES= 를 주면 노드가 torch.device('cpu') 를 고른다 (설치본 41행). 장치를 로그에 남긴다.
  실행: step4_run_dyn_node.sh (PYTHONDONTWRITEBYTECODE=1, ROS_LOG_DIR 지정)
"""
import os
import sys

HRC = os.path.realpath(os.path.expanduser("~/hrc_ws")) + os.sep
OUT = os.path.expanduser("~/effmass_mpc/env_check/step4/node_perf")
os.makedirs(OUT, exist_ok=True)
BLOCKED = []


def _audit(event, args):
    try:
        if event == "open":
            path, mode, flags = args[0], args[1], (args[2] if len(args) > 2 else 0)
            wr_flags = os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_APPEND | os.O_TRUNC
            writing = (isinstance(mode, str) and any(c in mode for c in "wax+")) or (
                mode is None and isinstance(flags, int) and (flags & wr_flags))
            if isinstance(path, (str, bytes, os.PathLike)) and writing:
                p = os.path.realpath(os.fsdecode(path))
                if p.startswith(HRC):
                    BLOCKED.append((event, p, mode))
                    print(f"[wrapper] BLOCKED {event} {p} mode={mode}", flush=True)
                    raise PermissionError(f"step4 wrapper: write under ~/hrc_ws blocked: {p}")
        elif event in ("os.mkdir", "os.rename", "os.remove", "os.rmdir", "shutil.rmtree"):
            p = os.path.realpath(os.fsdecode(args[0]))
            if p.startswith(HRC):
                BLOCKED.append((event, p, ""))
                print(f"[wrapper] BLOCKED {event} {p}", flush=True)
                raise PermissionError(f"step4 wrapper: {event} under ~/hrc_ws blocked: {p}")
    except PermissionError:
        raise
    except Exception:
        pass


import hrc_body_tracking.perf_logger as _pl  # noqa: E402

_orig_init = _pl.PerfCsvLogger.__init__


def _redirect_init(self, path, fieldnames, flush_every=30):
    new = os.path.join(OUT, os.path.basename(path))
    print(f"[wrapper] PerfCsvLogger 경로 변경: {path} -> {new}", flush=True)
    return _orig_init(self, new, fieldnames, flush_every)


_pl.PerfCsvLogger.__init__ = _redirect_init
sys.addaudithook(_audit)

import hrc_body_tracking  # noqa: E402
import hrc_body_tracking.human_dynamics_node as _node  # noqa: E402

if os.environ.get("STEP4_TORCH_THREADS"):
    import torch as _torch
    _torch.set_num_threads(int(os.environ["STEP4_TORCH_THREADS"]))
    print(f"[wrapper] (4-3d) torch.set_num_threads({os.environ['STEP4_TORCH_THREADS']}) 이 실행에서만, OMP_NUM_THREADS={os.environ.get('OMP_NUM_THREADS')} "
          f"MKL_NUM_THREADS={os.environ.get('MKL_NUM_THREADS')}", flush=True)

_orig_node_init = _node.DynamicsNode.__init__


def _node_init(self, *a, **k):
    _orig_node_init(self, *a, **k)
    import torch
    print(f"[wrapper] 장치: CUDA_VISIBLE_DEVICES={os.environ.get('CUDA_VISIBLE_DEVICES', '(지정 안 함)')!r}, "
          f"torch.cuda.is_available()={torch.cuda.is_available()}, node.device={self.device}, torch 스레드 {torch.get_num_threads()}, "
          f"OMP_NUM_THREADS={os.environ.get('OMP_NUM_THREADS', '(지정 안 함)')}", flush=True)
    if os.environ.get("STEP4_NO_FIND_WORST") == "1":
        print(f"[wrapper] find_worst_vis_enabled: renat.yaml 값 {self.find_worst_vis_enabled} -> False (이 실행에서만), "
              f"num_directions {self.find_worst_num_directions}, target {self.find_worst_target_index}", flush=True)
        self.find_worst_vis_enabled = False


_node.DynamicsNode.__init__ = _node_init

print(f"[wrapper] node module {_node.__file__} (pid {os.getpid()}), PYTHONDONTWRITEBYTECODE={os.environ.get('PYTHONDONTWRITEBYTECODE')} "
      f"ROS_LOG_DIR={os.environ.get('ROS_LOG_DIR')} ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}", flush=True)
try:
    rc = _node.main()
finally:
    print(f"[wrapper] 종료, 차단된 쓰기 시도 {len(BLOCKED)}건", flush=True)
sys.exit(rc)

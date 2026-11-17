#!/usr/bin/env bash
# run_isaac.sh — Isaac Sim 내장 Python으로 스크립트를 "깨끗한 환경"에서 실행한다.
#
#   사용법:  run_isaac.sh <script.py> [args...]
#            run_isaac.sh --print-env           # 실행 환경만 출력
#            run_isaac.sh --shell                # 같은 환경의 bash 셸 (디버그용)
#
#   원칙 (1단계 규칙):
#   - env -i 로 모든 환경 변수를 비우고 시작 → PYTHONPATH, LD_LIBRARY_PATH, CUDA_HOME,
#     ROS_DISTRO, AMENT_PREFIX_PATH, ROS_VERSION 등 ~/.bashrc 가 넣은 값이 전달되지 않음
#   - PYTHONNOUSERSITE=1 → ~/.local 의 torch, usd-core 차단
#   - PATH 는 시스템 기본값만 (/usr/local/cuda/bin 제외)
#   - GUI 를 위해 DISPLAY, XAUTHORITY, XDG_RUNTIME_DIR 만 통과
#   - 이 스크립트는 ~/.bashrc 나 시스템 설정을 전혀 바꾸지 않는다.
set -euo pipefail

ISAAC_ROOT="${ISAAC_ROOT:-$HOME/effmass_mpc/isaacsim}"
PY="$ISAAC_ROOT/python.sh"

CLEAN_ENV=(
  env -i
  HOME="$HOME"
  USER="${USER:-$(id -un)}"
  LOGNAME="${LOGNAME:-$(id -un)}"
  SHELL=/bin/bash
  TERM="${TERM:-xterm-256color}"
  LANG="${LANG:-C.UTF-8}"
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  PYTHONNOUSERSITE=1
  PYTHONUNBUFFERED=1   # SimulationApp.close() 가 프로세스를 바로 끝내므로, 파일로 리다이렉트할 때 print 가 버려지지 않게 함
  ISAAC_ROOT="$ISAAC_ROOT"
  ROS_LOG_DIR="$HOME/effmass_mpc/env_check/ros_log"   # rclpy 를 쓰는 경우 로그 위치 (3단계 사용자 결정)
  STEP_ROBOT="${STEP_ROBOT:-panda}"   # (goal1) 로봇 선택 panda|fr3 (step2_scene.robot_name)
  STEP_FR3_ARM_GAINS="${STEP_FR3_ARM_GAINS:-625,10}"   # (goal1) FR3 팔 구동 이득 (step2_scene.set_fr3_arm_gains)
)
# GUI/X11 용 변수만 선택적으로 통과
#   RUN_ISAAC_NO_DISPLAY=1 이면 DISPLAY 를 넘기지 않음 (headless 자동 실행 시 zenity 경고창이 기동을 막는 것을 방지)
# (goal1 1-2d 2안, 2026-11-16) Isaac 안 numpy(OpenBLAS64 pthreads) LAPACK 호출 뒤 스레드 바쁜 대기 방지: BLAS·OpenMP 1스레드. 이 실행 환경에만 적용
#   EFFMASS_THREADS1=0 이면 넣지 않음 (비교 측정용). warp 커널 캐시는 ~/effmass_mpc/cache/warp (기존 ~/.cache/warp 는 그대로 둠)
if [[ "${EFFMASS_THREADS1:-1}" == "1" ]]; then CLEAN_ENV+=(OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1); fi
CLEAN_ENV+=(WARP_CACHE_PATH="$HOME/effmass_mpc/cache/warp")
for v in DISPLAY XAUTHORITY XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS; do
  if [[ "${RUN_ISAAC_NO_DISPLAY:-0}" == "1" && ( "$v" == "DISPLAY" || "$v" == "XAUTHORITY" ) ]]; then continue; fi
  [[ -n "${!v:-}" ]] && CLEAN_ENV+=("$v=${!v}")
done
# 실행 스크립트 안에서만 지정하는 Isaac 관련 변수 (필요 시 여기서 추가)
# 예: CLEAN_ENV+=(OMNI_KIT_ALLOW_ROOT=1)

case "${1:-}" in
  --print-env)
    exec "${CLEAN_ENV[@]}" bash -c 'env | sort';;
  --shell)
    exec "${CLEAN_ENV[@]}" bash --noprofile --norc;;
  "")
    echo "사용법: $0 <script.py> [args...] | --print-env | --shell" >&2; exit 1;;
esac

if [[ ! -x "$PY" ]]; then
  echo "[run_isaac] $PY 가 없거나 실행 권한이 없습니다. Isaac Sim 이 $ISAAC_ROOT 에 풀려 있는지 확인하세요." >&2
  exit 2
fi

cd "$ISAAC_ROOT"
exec "${CLEAN_ENV[@]}" "$PY" "$@"

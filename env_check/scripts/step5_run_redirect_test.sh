#!/usr/bin/env bash
# step5_run_redirect_test.sh <out_dir> (step5_run_sensing.sh 와 같은 환경으로 step5_redirect_test.py 실행) — sensing_node 실행 (시스템 Python, ~/.bashrc 의 ROS 부분과 같은 source)
#   PYTHONDONTWRITEBYTECODE=1, ROS_LOG_DIR=~/effmass_mpc/env_check/ros_log, ROS_DOMAIN_ID=30, ROS_LOCALHOST_ONLY=1
#   DISPLAY·XAUTHORITY 는 넘김 (체형 추정 중 cv2.imshow("body") 가 항상 호출되고, renat.yaml imshow: True)
#   작업 폴더 = <out_dir> (상대 경로 쓰기가 있으면 그 아래). 쓰기는 step5_redirect_test.py 가 <out_dir> 로 돌림
set -euo pipefail
OUT="$(realpath -m "${1:?out_dir}")"; mkdir -p "$OUT"
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" SHELL=/bin/bash TERM="${TERM:-xterm}" \
  LANG="${LANG:-C.UTF-8}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin PYTHONUNBUFFERED=1 \
  DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
  PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$HOME/effmass_mpc/env_check/ros_log" \
  STEP5_SENSING_OUT="$OUT" \
  bash -c 'cd "$STEP5_SENSING_OUT"; set +u; source /opt/ros/humble/setup.bash; source "$HOME/hrc_ws/install/setup.bash"; set -u; exec /usr/bin/python3 "$HOME/effmass_mpc/env_check/scripts/step5_redirect_test.py"'

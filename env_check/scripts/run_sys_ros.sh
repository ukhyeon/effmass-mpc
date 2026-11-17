#!/usr/bin/env bash
# run_sys_ros.sh — 시스템 Python(/usr/bin/python3)을 이 PC 의 ROS 2 환경(~/.bashrc 의 ROS 부분과 같은 source)으로 실행.
#   ~/.bashrc 를 읽지 않고 env -i 후 setup.bash 2개만 source. ROS_DOMAIN_ID=30, ROS_LOCALHOST_ONLY=1 (bashrc 값).
#   홈 폴더에서 source 한다 (curobo/src/curobo 폴더 문제 회피, README 주의 참고).
set -euo pipefail
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" SHELL=/bin/bash TERM="${TERM:-xterm}" \
  LANG="${LANG:-C.UTF-8}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin PYTHONUNBUFFERED=1 \
  ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$HOME/effmass_mpc/env_check/ros_log" \
  bash -c 'cd "$HOME"; set +u; source /opt/ros/humble/setup.bash; source "$HOME/hrc_ws/install/setup.bash"; set -u; exec /usr/bin/python3 "$@"' _ "$@"

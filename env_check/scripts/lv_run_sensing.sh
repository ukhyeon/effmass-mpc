#!/usr/bin/env bash
# lv_run_sensing.sh <out_dir> - live prep: hrc_ws sensing_node with the real Kinect through lv_sensing_wrapper.py
#   (writes redirected to <out_dir>, visualization saving off). Same environment as step5_run_sensing.sh:
#   env -i, /opt/ros/humble + ~/hrc_ws/install, PYTHONDONTWRITEBYTECODE=1, ROS_LOG_DIR=~/effmass_mpc/env_check/ros_log,
#   ROS_DOMAIN_ID=${LV_DOMAIN:-30}, localhost only, DISPLAY passed (sensing calls cv2.imshow("body") during shape estimation).
set -euo pipefail
OUT="$(realpath -m "${1:?out_dir}")"; mkdir -p "$OUT"
exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" SHELL=/bin/bash TERM="${TERM:-xterm}" \
  LANG="${LANG:-C.UTF-8}" PATH=/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin PYTHONUNBUFFERED=1 \
  LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/lib \
  DISPLAY="${DISPLAY:-:0}" XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}" \
  PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID="${LV_DOMAIN:-30}" ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$HOME/effmass_mpc/env_check/ros_log" \
  LV_SENSING_OUT="$OUT" LV_SENSING_NOVIS="${LV_SENSING_NOVIS:-1}" \
  bash -c 'cd "$LV_SENSING_OUT"; set +u; source /opt/ros/humble/setup.bash; source "$HOME/hrc_ws/install/setup.bash"; set -u; exec /usr/bin/python3 "$HOME/effmass_mpc/env_check/scripts/lv_sensing_wrapper.py"'

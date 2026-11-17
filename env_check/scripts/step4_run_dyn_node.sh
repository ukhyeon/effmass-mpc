#!/usr/bin/env bash
# step4_run_dyn_node.sh — human_dynamics_node 실행 (시스템 Python, ~/.bashrc 의 ROS 부분과 같은 source)
#   PYTHONDONTWRITEBYTECODE=1 (~/hrc_ws/install 에 __pycache__ 생성 방지), ROS_LOG_DIR=~/effmass_mpc/env_check/ros_log
#   ROS_DOMAIN_ID=30, ROS_LOCALHOST_ONLY=1. 노드는 step4_dyn_node_wrapper.py 를 거쳐 실행 (성능 CSV 경로만 변경)
set -euo pipefail
# (4-3c) STEP4_NODE_CPU=1: 이 노드 프로세스 환경에만 CUDA_VISIBLE_DEVICES= (빈 값) → 노드가 CPU 로 계산
# (4-3) STEP4_NO_FIND_WORST=1: 래퍼에서 64방향 최악 유효질량 끔. STEP4_TASKSET="taskset -c <코어>": (D) 코어 지정
CVD=(); [[ "${STEP4_NODE_CPU:-0}" == "1" ]] && CVD=(CUDA_VISIBLE_DEVICES=)
# (4-3d) STEP4_NODE_THREADS=n: 이 노드 프로세스 환경에만 OMP_NUM_THREADS=n MKL_NUM_THREADS=n, 래퍼가 torch.set_num_threads(n)
[[ -n "${STEP4_NODE_THREADS:-}" ]] && CVD+=(OMP_NUM_THREADS=$STEP4_NODE_THREADS MKL_NUM_THREADS=$STEP4_NODE_THREADS STEP4_TORCH_THREADS=$STEP4_NODE_THREADS)
exec env -i "${CVD[@]}" STEP4_TASKSET="${STEP4_TASKSET:-}" HOME="$HOME" USER="${USER:-$(id -un)}" LOGNAME="${LOGNAME:-$(id -un)}" SHELL=/bin/bash TERM="${TERM:-xterm}" \
  LANG="${LANG:-C.UTF-8}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin PYTHONUNBUFFERED=1 \
  PYTHONDONTWRITEBYTECODE=1 STEP4_NO_FIND_WORST="${STEP4_NO_FIND_WORST:-0}" ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$HOME/effmass_mpc/env_check/ros_log" \
  bash -c 'cd "$HOME"; set +u; source /opt/ros/humble/setup.bash; source "$HOME/hrc_ws/install/setup.bash"; set -u; exec ${STEP4_TASKSET:-} /usr/bin/python3 "$HOME/effmass_mpc/env_check/scripts/step4_dyn_node_wrapper.py"'

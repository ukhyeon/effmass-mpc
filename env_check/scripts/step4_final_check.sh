#!/usr/bin/env bash
# step4_final_check.sh — 4-4 마무리 확인 (읽기만). 기준 시각 = 3단계 최종 확인(2026-11-15 23:21) 이후
set -u
REF="${REF:-2026-11-15 23:21}"
D=$HOME/effmass_mpc/env_check
echo "== 기준 시각 $REF 이후 바뀐 파일 수"
for p in "$HOME/.local/lib/python3.10/site-packages" "$HOME/.local/bin" "$HOME/.local/share/applications" /opt/ros/humble /usr/local \
         "$HOME/hrc_ws/src" "$HOME/hrc_ws/install" "$HOME/hrc_ws/build" "$HOME/hrc_ws/analysis" "$HOME/hrc_ws/pose_npz" \
         "$HOME/indy_ws/src" "$HOME/indy_ws/install" "$HOME/indy_ws/build" "$HOME/.ros"; do
  echo "  $p: $(find "$p" -newermt "$REF" \( -type f -o -type l \) 2>/dev/null | wc -l)"
done
echo "  ~/hrc_ws 전체 (.git 제외): $(find $HOME/hrc_ws -path '*/.git' -prune -o \( -type f -newermt "$REF" \) -print 2>/dev/null | wc -l)"
echo "  ~/indy_ws 전체 (.git 제외): $(find $HOME/indy_ws -path '*/.git' -prune -o \( -type f -newermt "$REF" \) -print 2>/dev/null | wc -l)"
echo "  ~/.ros/log 파일 수: $(find $HOME/.ros/log -type f | wc -l)"
echo "== 0번 기준값"
echo "  nvcc: $(which nvcc 2>/dev/null || echo '(PATH 에 없음)') / $(/usr/local/cuda/bin/nvcc --version | grep -o 'V[0-9.]*')"
echo "  /usr/local/cuda -> $(readlink /usr/local/cuda)"
echo "  시스템 torch: $(cd /tmp && /usr/bin/python3 -B -c 'import torch;print(torch.__version__, torch.version.cuda, torch.cuda.is_available())' 2>&1 | tail -1)"
echo "  드라이버: $(nvidia-smi --query-gpu=driver_version --format=csv,noheader)"
echo "  ~/.bashrc sha256: $(sha256sum $HOME/.bashrc | cut -c1-8)…$(sha256sum $HOME/.bashrc | cut -c59-64)"
echo "  HumanPose import (시스템): $($D/scripts/run_sys_ros.sh -B -c 'from hrc_interfaces.msg import HumanPose, HumanDynamicsState; print("OK")' 2>&1 | tail -1)"
echo "== 작업공간 git (index 를 쓰지 않는 --no-optional-locks)"
echo "  hrc_ws 미커밋: $(git --no-optional-locks -C $HOME/hrc_ws status --porcelain 2>/dev/null | wc -l), indy_ws 미커밋: $(git --no-optional-locks -C $HOME/indy_ws status --porcelain 2>/dev/null | wc -l)"
echo "== Isaac pip 목록 (2단계 env_lock 대비)"
env -i HOME=$HOME PATH=/usr/bin:/bin PYTHONNOUSERSITE=1 $HOME/effmass_mpc/isaacsim/python.sh -m pip list --format=freeze 2>/dev/null > $D/logs/step4_isaac_pip_now.txt
echo "  차이 줄 수: $(diff $D/step2/env_lock/isaac_pip_list.txt $D/logs/step4_isaac_pip_now.txt | grep -c '^[<>]')"
echo "== 남은 프로세스·공유 메모리"
echo "  ROS 데몬·rclpy 스크립트·Isaac·감시: $(ps -eo cmd | grep -E '_ros2_daemon|kit/python|isaacsim|step4_|step3_|step2b|vram_monitor|human_dynamics' | grep -v -e grep -e step4_final_check | wc -l)"
echo "  /dev/shm Fast DDS: $(ls /dev/shm | grep -c fastrtps)"
echo "  Slack pid 136205: $(ps -o pid=,etime= -p 136205 2>/dev/null || echo 없음) 현재 CPU% $(top -b -n 2 -d 1 -p 136205 2>/dev/null | awk 'END{print $9}')"

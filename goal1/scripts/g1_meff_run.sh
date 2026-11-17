#!/usr/bin/env bash
# g1_meff_run.sh <tag> [step2b 추가 인자...] — 1-2d 실시간 측정: 장면 g1, fr3v2_1, 4-3d (F) 설정
#   (F) = 재생 HumanPose 30 Hz + 사람 노드 CPU 1스레드 + ROS + 사람 표시(COM 19 PointInstancer)·GPU 텐서, 예열 10, 점검 끔, headless
#   노드 = effmass_human 사본 (g1_run_human.sh node), Isaac 사람 출처 = HumanInertiaState (--human-source inertia)
#   채널은 추가 인자로: --meff on [--meff-impl warp ...]. 기록: 프로세스별 CPU%·VRAM(step4_sys_monitor), VRAM, GPU 클럭(nvidia-smi dmon), JointState 수신
set -u
# 남은 프로세스 수: 명령 이름이 bash·grep 인 것 제외 (검사 문자열을 담은 셸 자신이 잡히지 않게, 1-3)
left() { ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|effmass_human.human_inertia_node|step4_replay|step4_js_sink|step4_sys_monitor|vram_monitor|nvidia-smi dmon|kit/python'; }
TAG="${1:?tag}"; shift
E=$HOME/effmass_mpc/env_check; S=$E/scripts; G=$HOME/effmass_mpc/goal1; O=$G/logs/meff_isaac; mkdir -p $O; P=$O/$TAG
NPZ=$HOME/hrc_ws/pose_npz/human_state_1781524848.npz
# (B1) 측정 전 점검: 남은 프로세스·/dev/shm(ad_ 제외) 이 0 이 아니면 시작하지 않음
if [[ $(left) != 0 || $(ls /dev/shm | grep -vc '^ad_') != 0 ]]; then echo "[pre-check] 중단: 남은 프로세스 $(left), /dev/shm $(ls /dev/shm | grep -v '^ad_' | tr '\n' ' ')"; exit 3; fi
rm -f $P.ready $P.done $P.end $P.stop
$S/step4_hrc_snapshot.sh ${P}_hrc_before > /dev/null; touch ${P}_t0
WARP0=$(find $HOME/.cache/warp -type f | wc -l)
echo "[g1d] tag $TAG 추가 isaac 인자 '$*' (시작 $(date +%T))"
/usr/bin/python3 -B $S/step4_sys_monitor.py --proc isaac=step2b_mpc_isaac.py --proc node=effmass_human.human_inertia_node --proc replay=step4_replay.py --out ${P}_sys_monitor.csv --stop-file $P.stop &
SM=$!
$S/vram_monitor.sh ${P}_vram.csv & VM=$!
nvidia-smi dmon -s pc -d 1 -o T > ${P}_dmon.txt 2>&1 & DM=$!
$S/run_sys_ros.sh -B $S/step4_js_sink.py --out ${P}_sys_recv_js.csv --stop-file $P.stop > ${P}_sink.log 2>&1 &
SK=$!
EFFMASS_WS=1 RUN_ISAAC_NO_DISPLAY="${RUN_ISAAC_NO_DISPLAY:-1}" ROS_METHOD=min $S/run_isaac_ros.sh $S/step2b_mpc_isaac.py --scene g1 --robot fr3v2_1 --no-diag --warmup 10 \
  --tag $TAG --ros --ros-pose-topic none --human --human-mode full --human-source inertia --ready-file $P.ready --done-file $P.done --ros-out $P --u-def com "$@" > $P.log 2>&1 &   # u-def com = A-stage/1-2d definition
IP=$!
$G/scripts/g1_run_human.sh node -p perf_csv:=${P}_nodeperf.csv > ${P}_node.log 2>&1 &
NP=$!
if [[ "${THREAD_SNAP:-0}" == "1" ]]; then   # 루프 중간에 Isaac 프로세스 스레드별 CPU (top -H, 밖에서 읽기만)
  ( while [[ ! -f $P.ready ]]; do sleep 0.5; done; sleep 16; IPID=$(pgrep -n -f "step2b_mpc_isaac.py --scene"); echo "pid $IPID $(date +%T.%N)" > ${P}_threads.txt
    top -H -b -n 2 -d 2 -w 200 -p $IPID >> ${P}_threads.txt 2>&1 ) &
fi
$S/run_sys_ros.sh -B $S/step4_replay.py --npz $NPZ ${REPLAY_ARGS:-} --ready-file $P.ready --end-file $P.end --out $P --start-wait 8 > ${P}_replay.log 2>&1
echo "replay exit=$?"
wait $IP; echo "isaac exit=$?"
kill -INT $NP; wait $NP; echo "node exit=$?"
touch $P.stop; wait $SM $SK 2>/dev/null
sleep 1; kill $VM $DM 2>/dev/null; wait $VM $DM 2>/dev/null
$S/step4_hrc_snapshot.sh ${P}_hrc_after > /dev/null
[[ $(left) == 0 ]] && rm -f /dev/shm/sem.carbonite-sharedmemory
# GUI window closed early -> Isaac exits without ROS cleanup and leaves fastrtps shm. Remove only entries created after this run started, when no related process is left.
[[ $(left) == 0 ]] && find /dev/shm -maxdepth 1 \( -name "fastrtps_*" -o -name "sem.fastrtps_*" \) -newer ${P}_t0 -print -delete | sed "s/^/[cleanup] removed /"   # (1-3) Kit 세마포어
echo "--- ~/hrc_ws 변경: $(diff ${P}_hrc_before_files.tsv ${P}_hrc_after_files.tsv | grep -c '^[<>]')줄, ~/.ros 새 파일: $(find $HOME/.ros -type f -newer ${P}_t0 | wc -l), /dev/shm fastrtps: $(ls /dev/shm | grep -c fastrtps), warp 캐시 파일 $WARP0 → $(find $HOME/.cache/warp -type f | wc -l)"
echo "--- 남은 프로세스: $(left)"
grep -a "^\[replay\]" ${P}_replay.log | tail -1
grep -aE "^\[step2b\] (예열|ROS|사람|\(1-2d\) (import|유효)|solver_ms|realtime|loop_ms|infeasible|arrivals|graph)" $P.log | cut -c1-260
grep -aE "Traceback" -A6 $P.log ${P}_node.log | head -14

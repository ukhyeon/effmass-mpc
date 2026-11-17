#!/usr/bin/env bash
# g13_pre_run.sh <tag> [step2b 추가 인자...] — 1-3 비교용 실행: 장면 g1, fr3v2_1, 사람 = 미리 계산한 프레임 (--human-source precomputed, 1안 결정적 재생)
#   재생·사람 노드·JointState 수신 없음. Isaac 은 g1_meff_run.sh 와 같은 인자 (ROS 켬, 사람 표시 COM 19, 예열 10, 점검 끔, headless)
#   시간 판정에 쓰지 않음. 기록: 프로세스 CPU%·VRAM, GPU 클럭 (dmon)
set -u
# 남은 프로세스 수: 명령 이름이 bash·grep 인 것 제외 (검사 문자열을 담은 셸 자신이 잡히지 않게, 1-3)
left() { ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|effmass_human.human_inertia_node|step4_replay|step4_js_sink|step4_sys_monitor|vram_monitor|nvidia-smi dmon|kit/python'; }
TAG="${1:?tag}"; shift
E=$HOME/effmass_mpc/env_check; S=$E/scripts; G=$HOME/effmass_mpc/goal1; O=$G/logs/meff_isaac; mkdir -p $O; P=$O/$TAG
# (B1) 측정 전 점검: 남은 프로세스·/dev/shm(ad_ 제외) 이 0 이 아니면 시작하지 않음
if [[ $(left) != 0 || $(ls /dev/shm | grep -vc '^ad_') != 0 ]]; then echo "[pre-check] 중단: 남은 프로세스 $(left), /dev/shm $(ls /dev/shm | grep -v '^ad_' | tr '\n' ' ')"; exit 3; fi
rm -f $P.ready $P.done $P.end $P.stop
$S/step4_hrc_snapshot.sh ${P}_hrc_before > /dev/null; touch ${P}_t0
echo "[g13pre] tag $TAG 추가 isaac 인자 '$*' (시작 $(date +%T)), 측정 전 남은 프로세스 $(left), /dev/shm 항목 $(ls /dev/shm | grep -vc '^ad_')"
/usr/bin/python3 -B $S/step4_sys_monitor.py --proc isaac=step2b_mpc_isaac.py --out ${P}_sys_monitor.csv --stop-file $P.stop &
SM=$!
$S/vram_monitor.sh ${P}_vram.csv & VM=$!
nvidia-smi dmon -s pc -d 1 -o T > ${P}_dmon.txt 2>&1 & DM=$!
EFFMASS_WS=1 RUN_ISAAC_NO_DISPLAY="${RUN_ISAAC_NO_DISPLAY:-1}" ROS_METHOD=min $S/run_isaac_ros.sh $S/step2b_mpc_isaac.py --scene g1 --robot fr3v2_1 --no-diag --warmup 10 \
  --tag $TAG --ros --ros-pose-topic none --human --human-mode full --human-source precomputed --ready-file $P.ready --done-file $P.done --ros-out $P "$@" > $P.log 2>&1
echo "isaac exit=$?"
touch $P.stop; wait $SM 2>/dev/null
sleep 1; kill $VM $DM 2>/dev/null; wait $VM $DM 2>/dev/null
$S/step4_hrc_snapshot.sh ${P}_hrc_after > /dev/null
# Isaac(Kit carbonite) 가 실행마다 남기는 32바이트 세마포어: Kit 프로세스가 없을 때만 지움 (1-3에서 확인)
[[ $(left) == 0 ]] && rm -f /dev/shm/sem.carbonite-sharedmemory
# GUI window closed early -> Isaac exits without ROS cleanup and leaves fastrtps shm. Remove only entries created after this run started, when no related process is left.
[[ $(left) == 0 ]] && find /dev/shm -maxdepth 1 \( -name "fastrtps_*" -o -name "sem.fastrtps_*" \) -newer ${P}_t0 -print -delete | sed "s/^/[cleanup] removed /"
echo "--- ~/hrc_ws 변경: $(diff ${P}_hrc_before_files.tsv ${P}_hrc_after_files.tsv | grep -c '^[<>]')줄, ~/.ros 새 파일: $(find $HOME/.ros -type f -newer ${P}_t0 | wc -l), /dev/shm fastrtps: $(ls /dev/shm | grep -c fastrtps), /dev/shm 기타(ad_ 제외): $(ls /dev/shm | grep -v '^ad_' | tr '\n' ' ')"
echo "--- 남은 프로세스: $(left)"
grep -aE "^\[step2b\] (예열|ROS|사람|\(1-2d\) (import|유효)|\(1-3\)|solver_ms|realtime|loop_ms|infeasible|arrivals|graph)" $P.log | cut -c1-260
grep -aE "Traceback" -A8 $P.log | head -14

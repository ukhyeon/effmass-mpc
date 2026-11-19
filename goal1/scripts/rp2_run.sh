#!/usr/bin/env bash
# rp2_run.sh <name> <profile yml> <tau_ms> [K] [duration_s] - real_prep2: fake robot (1 kHz joint states, first-order lag tau) + Isaac-free
#   MPC (rp2_mpc_rt.py, wall-clock 60 Hz). ROS_DOMAIN_ID 30, ROS_LOCALHOST_ONLY 1 for both. Joint-state topic of the test:
#   /rp2/fake/joint_states (the control-PC format /NS_1/franka/joint_states is the runner's default). Results goal1/real_prep2/<name>_*.
#   No Isaac: refuses to start while an Isaac process is running (timing would be disturbed).
#   Env FAKE_HZ: joint-state rate of the fake robot (default 1000). JS_SRC=shm: a separate receiver process (rp2_js_relay.py) writes the
#   latest joint state to shared memory and the MPC reads it there (default: the MPC subscribes itself). MPC_EXTRA: more runner options
#   (e.g. "--age-comp 1 --js-qos best_effort"). OUT_DIR: result folder (default goal1/real_prep2). FAKE_QOS: QoS of the fake robot's joint-state publisher (reliable | best_effort, default reliable).
set -u
NAME=${1:?name}; PF=${2:?profile}; TAU=${3:?tau_ms}; K=${4:-1}; DUR=${5:-180}
GG=$HOME/effmass_mpc/goal1; SC=$GG/scripts; OUT=${OUT_DIR:-$GG/real_prep2}; mkdir -p $OUT; JS=/rp2/fake/joint_states
n=$(ps -eo comm=,args= | awk '$1!="bash" && $1!="grep" && $1!="sh"' | grep -cE 'step2b_mpc_isaac|kit/python|rp2_mpc_rt')
[[ $n == 0 ]] || { echo "an Isaac / MPC process is running; stop it first"; exit 3; }
SJ=$GG/live/live3m/corrected/cal/m1b/scene_AB.json
Q0=$(python3 -c "import json;print(','.join(str(x) for x in json.load(open('$SJ'))['q_start']))")
$SC/g1_run_human.sh py $SC/rp2_fake_robot.py --js-topic $JS --tau-ms $TAU --rate-hz ${FAKE_HZ:-1000} --js-qos ${FAKE_QOS:-reliable} --q0 $Q0 --duration $(( DUR + 240 )) --out $OUT/$NAME > $OUT/${NAME}_fake.log 2>&1 &
FP=$!
sleep 2
RP=""
if [[ ${JS_SRC:-topic} == shm ]]; then
  $SC/g1_run_human.sh py $SC/rp2_js_relay.py --js-topic $JS --shm-name rp2_js --duration $(( DUR + 240 )) --stop-file $OUT/${NAME}.relaystop > $OUT/${NAME}_relay.log 2>&1 &
  RP=$!; sleep 2
fi
/usr/bin/python3 -B $HOME/effmass_mpc/env_check/scripts/step4_sys_monitor.py --proc mpc=rp2_mpc_rt.py --proc fake=rp2_fake_robot.py --proc relay=rp2_js_relay.py --out $OUT/${NAME}_sys.csv --stop-file $OUT/${NAME}.stop > /dev/null 2>&1 &
SM=$!
ROS_METHOD=min $HOME/effmass_mpc/env_check/scripts/run_isaac_ros.sh $SC/rp2_mpc_rt.py --profile $PF --js-topic $JS --k $K --duration $DUR --scene-json $SJ --out $OUT/$NAME --js-source ${JS_SRC:-topic} ${MPC_EXTRA:-} > $OUT/${NAME}_mpc.log 2>&1
echo "mpc exit $?"
[[ -n $RP ]] && { touch $OUT/${NAME}.relaystop; wait $RP; rm -f $OUT/${NAME}.relaystop; tail -1 $OUT/${NAME}_relay.log; }
wait $FP; touch $OUT/${NAME}.stop; wait $SM 2>/dev/null; rm -f $OUT/${NAME}.stop
grep -a "^\[rp2\]\|Traceback\|Error" $OUT/${NAME}_mpc.log | tail -4 | cut -c1-400; tail -1 $OUT/${NAME}_fake.log

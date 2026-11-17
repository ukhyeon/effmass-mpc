"""step3_msgs.py — 3단계 통신 시험용 가짜 메시지 생성·검증 (시스템 Python 과 Isaac Python 이 같이 import)

numpy·표준 라이브러리만 사용. 메시지 정의(hrc_interfaces)는 바꾸지 않는다.

순번·시각 표기 (메시지 정의를 바꾸지 않고 기존 필드 활용):
  header.stamp     = 발행 직전 time.time_ns() (같은 PC 의 벽시계)
  header.frame_id  = "step3/<kind>/<seq>"   (시험 표시용. 실제 sensing 노드는 "azure_kinect_base")

값 규칙: float32 에서 정확히 표현되는 값만 쓴다 (곱 0.5·정수 덧셈) → 받은 값과 기대값을 비트 단위로 비교 가능.
HumanPose 크기는 실제 sensing_node.py 가 채우는 크기와 같다 (0단계 보고서 6.3절):
  body_full_pose 66, global_trans 3, smpl_joint 135, smpl_vertices 20670, smpl_betas 10, centorid_24 0, image 비움
HumanDynamicsState: 19 body part (human_dynamics_node.py 와 같은 개수), positions/velocities 각 57
JointState: Franka 9 관절 이름
"""
import array
import time

import numpy as np

POSE_SIZES = [("body_full_pose", 66), ("global_trans", 3), ("smpl_joint", 135), ("smpl_vertices", 20670),
              ("smpl_betas", 10)]
BODY_PARTS = 19
FRANKA_JOINTS = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6",
                 "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"]
BEST_EFFORT_QOS_DEPTH = 5  # sensing_node.py 의 /hrc/human_pose QoS (KEEP_LAST 5, BEST_EFFORT, VOLATILE)


def _vals(n, seq, field_idx, dtype=np.float32):
    return (np.arange(n, dtype=dtype) * dtype(0.5) + dtype(seq % 1000) + dtype(field_idx * 0.25)).astype(dtype)


def _farr(np_arr):
    a = array.array("f")
    a.frombytes(np.ascontiguousarray(np_arr, dtype=np.float32).tobytes())
    return a


def frame_id(kind, seq):
    return f"step3/{kind}/{seq}"


def parse_frame_id(fid):
    try:
        _, kind, seq = fid.split("/")
        return kind, int(seq)
    except Exception:
        return None, -1


def stamp_now(header):
    t = time.time_ns()
    header.stamp.sec = t // 1_000_000_000
    header.stamp.nanosec = t % 1_000_000_000
    return t


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


# ---------------- HumanPose ----------------
def fill_pose(msg, seq, kind="pose"):
    msg.header.frame_id = frame_id(kind, seq)
    for k, (name, n) in enumerate(POSE_SIZES):
        setattr(msg, name, _farr(_vals(n, seq, k)))
    msg.centorid_24 = array.array("f")
    return msg


def check_pose(msg, seq, full=True):
    """full=True: 모든 원소 비교. False: 길이 + 각 필드 처음·중간·끝 원소만 (MPC 루프 부하를 줄이는 가벼운 검사)."""
    for k, (name, n) in enumerate(POSE_SIZES):
        got = np.frombuffer(getattr(msg, name), dtype=np.float32) if len(getattr(msg, name)) else np.zeros(0, np.float32)
        if got.size != n:
            return False
        exp = _vals(n, seq, k)
        if full:
            if not np.array_equal(got, exp):
                return False
        else:
            idx = [0, n // 2, n - 1]
            if not np.array_equal(got[idx], exp[idx]):
                return False
    return len(msg.centorid_24) == 0 and len(msg.image.data) == 0


# ---------------- HumanDynamicsState ----------------
def fill_dyn(msg, seq, kind="dyn"):
    msg.header.frame_id = frame_id(kind, seq)
    msg.body_part_indices = array.array("I", [(seq + i) % 100000 for i in range(BODY_PARTS)])
    msg.body_part_names = [f"part{i}" for i in range(BODY_PARTS)]
    msg.body_part_positions = _farr(_vals(BODY_PARTS * 3, seq, 0))
    msg.body_part_velocities = _farr(_vals(BODY_PARTS * 3, seq, 1))
    return msg


def check_dyn(msg, seq):
    return (list(msg.body_part_indices) == [(seq + i) % 100000 for i in range(BODY_PARTS)]
            and list(msg.body_part_names) == [f"part{i}" for i in range(BODY_PARTS)]
            and np.array_equal(np.frombuffer(msg.body_part_positions, np.float32), _vals(BODY_PARTS * 3, seq, 0))
            and np.array_equal(np.frombuffer(msg.body_part_velocities, np.float32), _vals(BODY_PARTS * 3, seq, 1)))


# ---------------- JointState (3-1: 규칙 값) ----------------
def fill_js(msg, seq, kind="js", positions=None, velocities=None):
    msg.header.frame_id = frame_id(kind, seq)
    msg.name = list(FRANKA_JOINTS)
    if positions is None:
        positions = (np.arange(9, dtype=np.float64) * 0.25 + (seq % 1000) * 0.5).tolist()
        velocities = (np.arange(9, dtype=np.float64) * 0.125 + (seq % 1000) * 0.25).tolist()
    msg.position = [float(x) for x in positions]
    msg.velocity = [float(x) for x in velocities]
    msg.effort = []
    return msg


def check_js_rule(msg, seq):
    return (list(msg.name) == FRANKA_JOINTS
            and list(msg.position) == (np.arange(9, dtype=np.float64) * 0.25 + (seq % 1000) * 0.5).tolist()
            and list(msg.velocity) == (np.arange(9, dtype=np.float64) * 0.125 + (seq % 1000) * 0.25).tolist())


def best_effort_qos():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=BEST_EFFORT_QOS_DEPTH,
                      reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)

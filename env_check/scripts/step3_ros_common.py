"""step3_ros_common.py — 3단계 공통: ROS 2 경로 추가와 import 경로 점검 도우미"""
import os
import sys

HOME = os.path.expanduser("~")
ISAAC = os.path.join(HOME, "effmass_mpc", "isaacsim")


def add_ros_paths():
    """ROS_METHOD=min 일 때 STEP3_ROS_PYPATHS 를 sys.path '맨 뒤'에 붙인다 (Isaac 경로가 항상 먼저 검색되게). 붙인 경로 목록 반환."""
    added = []
    for p in [x for x in os.environ.get("STEP3_ROS_PYPATHS", "").split(":") if x]:
        if p not in sys.path:
            sys.path.append(p)
            added.append(p)
    return added


def label(p):
    if p.startswith(ISAAC):
        return "isaac"
    if p.startswith("/opt/ros"):
        return "ros"
    if p.startswith(os.path.join(HOME, "hrc_ws")):
        return "hrc_ws"
    if p.startswith(os.path.join(HOME, ".local")):
        return "LOCAL!"
    if p.startswith(os.path.join(HOME, "effmass_mpc")):
        return "effmass"
    return "other"


def short(p):
    return str(p).replace(HOME, "~")

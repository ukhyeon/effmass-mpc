"""step5_redirect_test.py — Kinect 연결 없이 sensing_node 쓰기 경로 시험 (실제 __init__ 실행, 타이머는 돌리지 않음)"""
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import step5_sensing_redirect as RD
OUT = os.environ["STEP5_SENSING_OUT"]
os.makedirs(f"{OUT}/vis_output", exist_ok=True)
open(f"{OUT}/vis_output/dummy_old.jpg", "w").write("x")     # 시작 시 삭제가 여기서 일어나는지 보는 표지
S = RD.install(OUT)
H = os.path.expanduser("~/hrc_ws")
import rclpy
rclpy.init()
n = S.HRCSensingNode()                                      # 실제 __init__: vis_output 만들기·기존 jpg 삭제·PerfCsvLogger·CLIFF 적재
print("[test] 시작 시 삭제 후 표지 파일 남음?", os.path.exists(f"{OUT}/vis_output/dummy_old.jpg"))
img = np.zeros((8, 8, 3), np.uint8)
# body_crop: timer_callback 과 같은 고정 경로로 호출
S.os.makedirs("/home/robotics/hrc_ws/body_crop", exist_ok=True)
print("[test] body_crop imwrite", S.cv2.imwrite(os.path.join("/home/robotics/hrc_ws/body_crop", "body_crop_test.jpg"), img))
# vis_output: 렌더 스레드와 같은 save_path 생성 (self.vis_dp 사용) + 원래 고정 경로로도
print("[test] vis_output imwrite (vis_dp)", S.cv2.imwrite(os.path.join(n.vis_dp, "000000.jpg"), img),
      "(원래 경로)", S.cv2.imwrite("/home/robotics/hrc_ws/vis_output/000001.jpg", img))
# 성능 CSV 1줄
n.perf_logger.write(t_ros_sec="0", frame_index=0, capture_ms="0", pose_inference_ms="0", visualization_ms="0", publish_ms="0",
                    total_operation_ms="0", nominal_deadline_ms="33.3", deadline_miss=0)
# pose_npz (destroy_node 경로 그대로)
n.log_buffer["pose"].append(np.zeros(66, np.float32)); n.log_buffer["trans"].append(np.zeros(3, np.float32))
n.destroy_node()
rclpy.shutdown()
# 감사 훅 판정만 시험 (실제 쓰기 없음)
for path in (f"{H}/analysis/realtime_perf/x.csv", os.path.expanduser("~/realtime_smpl/x"), "/opt/ros/humble/x"):
    try:
        RD._audit("open", (path, "w", 0)); print("[test] 훅 통과(문제)", path)
    except PermissionError:
        print("[test] 훅 차단 OK", path)
RD.BLOCKED.clear()
print("[test] 경로 바꾼 호출", RD.REWRITES)
for root, dirs, files in os.walk(OUT):
    for f in files:
        print("[test] OUT 파일", os.path.relpath(os.path.join(root, f), OUT), os.path.getsize(os.path.join(root, f)))

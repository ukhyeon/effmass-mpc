"""step5_sensing_wrapper.py — 설치된 sensing_node.main() 을 그대로 실행하되 쓰기만 step5/<tag>/ 로 (step5_sensing_redirect)
  환경: STEP5_SENSING_OUT=<폴더> (step5_run_sensing.sh 가 지정)
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import step5_sensing_redirect as RD
S = RD.install(os.environ["STEP5_SENSING_OUT"])
try:
    rc = S.main()
finally:
    print(f"[sensing-wrapper] 종료, 경로 바꾼 호출 {RD.REWRITES}, 차단된 쓰기 시도 {len(RD.BLOCKED)}건 {RD.BLOCKED[:5]}", flush=True)
sys.exit(rc)

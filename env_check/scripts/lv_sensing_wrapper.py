"""lv_sensing_wrapper.py - live prep: run the installed hrc_ws sensing_node unchanged, writes redirected to $LV_SENSING_OUT.

Uses step5_sensing_redirect.install (audit hook blocks writes under ~/hrc_ws, ~/realtime_smpl, ~/indy_ws, ~/.local, /opt/ros;
vis_output / body_crop / pose_npz / PerfCsvLogger paths moved to <out>; vis_output is cleared there, not in ~/hrc_ws).
Added for the current sensing_node (IMU camera pose, 2026-11-18): the IMU measurement log (write_measurement_log, default
~/hrc_ws/debug_log) goes to <out>/debug_log. LV_SENSING_NOVIS=1 (default here): imshow and imsave off, render packets dropped
(vertices still computed and published), as step5 option B.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("STEP5_SENSING_NOVIS", os.environ.get("LV_SENSING_NOVIS", "1"))
import step5_sensing_redirect as RD

OUT = os.environ["LV_SENSING_OUT"]
S = RD.install(OUT)
_orig_log = S.write_measurement_log


def _log_to_out(data, directory=None):
    return _orig_log(data, directory=os.path.join(OUT, "debug_log"))


S.write_measurement_log = _log_to_out
print(f"[lv-sensing] IMU measurement log -> {OUT}/debug_log, NOVIS {os.environ.get('STEP5_SENSING_NOVIS')}", flush=True)
try:
    rc = S.main()
finally:
    print(f"[lv-sensing] end, rewrites {RD.REWRITES}, blocked writes {len(RD.BLOCKED)} {RD.BLOCKED[:5]}", flush=True)
sys.exit(rc)

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
SAVEVIS = os.environ.get("LV_SENSING_SAVEVIS", "0") == "1"
# (live prep2) LV_SENSING_SAVEVIS=1: save the Kinect colour image of every frame sensing hands to its render thread (the packet's
# bg_img, no SMPL overlay render: render packets are still dropped as NOVIS) as <out>/vis_output/<index>.jpg with the frame time
# drawn on it, written by a separate thread (queue of 90 frames; frames are dropped and counted if it is full). Frame time = wall
# time when get_capture2 returned that frame index (the instance method is wrapped on the first timer call after the camera is
# opened; the very first frame uses the submit time). <out>/vis_times.csv: file, wall_time (capture), submit_time, write_time.
os.environ.setdefault("STEP5_SENSING_NOVIS", os.environ.get("LV_SENSING_NOVIS", "1"))
import step5_sensing_redirect as RD

OUT = os.environ["LV_SENSING_OUT"]
S = RD.install(OUT)
_orig_log = S.write_measurement_log


def _log_to_out(data, directory=None):
    return _orig_log(data, directory=os.path.join(OUT, "debug_log"))


S.write_measurement_log = _log_to_out
if SAVEVIS:
    import queue as _queue
    import threading as _threading
    import time as _time
    import cv2 as _cv
    _CAP = {}; _Q = _queue.Queue(maxsize=90); _ST = dict(saved=0, dropped=0)
    _tfile = open(os.path.join(OUT, "vis_times.csv"), "w"); _tfile.write("file,wall_time,submit_time,write_time\n")
    _prev_init = S.HRCSensingNode.__init__

    def _writer():
        while True:
            it = _Q.get()
            if it is None:
                return
            path, img, tc, ts = it; name = os.path.basename(path)
            txt = _time.strftime("%H:%M:%S", _time.localtime(tc)) + f".{int((tc % 1) * 1000):03d}  #{name[:-4]}"
            _cv.putText(img, txt, (12, img.shape[0] - 14), _cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, _cv.LINE_AA)
            _cv.putText(img, txt, (12, img.shape[0] - 14), _cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, _cv.LINE_AA)
            S.cv2.imwrite(path, img)       # step5 proxy: ~/hrc_ws/vis_output -> <out>/vis_output
            _tfile.write(f"{name},{tc:.6f},{ts:.6f},{_time.time():.6f}\n"); _tfile.flush(); _ST["saved"] += 1

    _threading.Thread(target=_writer, daemon=True, name="lv_color_writer").start()

    def _init_savevis(self, *a, **k):
        _prev_init(self, *a, **k)
        self.vis_config["imshow"] = False; self.vis_config["imsave"] = True

        def _save_packet(packet, _self=self):
            sp = packet.get("save_path"); img = packet.get("bg_img")
            if sp is None or img is None:
                return
            ts = _time.time(); idx = int(os.path.basename(sp)[:-4]); tc = _CAP.pop(idx, ts)
            for _k in [k for k in list(_CAP) if k < idx - 300]:
                _CAP.pop(_k, None)
            try:
                _Q.put_nowait((sp, img, tc, ts))
            except _queue.Full:
                _ST["dropped"] += 1
                if _ST["dropped"] % 30 == 1:
                    print(f"[lv-sensing] colour writer busy: dropped {_ST['dropped']} frames (saved {_ST['saved']})", flush=True)
        self._submit_render_packet = _save_packet
        print(f"[lv-sensing] colour frames saved (no overlay render): enable {self.visualization_enabled}, vis_smpl {self.vis_config.get('vis_smpl')}", flush=True)

    S.HRCSensingNode.__init__ = _init_savevis
    _prev_tc = S.HRCSensingNode.timer_callback

    def _tc_stamp(self):
        k = getattr(self, "k4a", None)
        if k is not None and getattr(self, "_lv_k4a", None) is not k:
            _g = k.get_capture2

            def _g2(*a, **kw):
                d = _g(*a, **kw); _CAP[self.index] = _time.time(); return d
            try:
                k.get_capture2 = _g2
            except Exception as e:
                print(f"[lv-sensing] capture time wrap failed ({e}); submit times used", flush=True)
            self._lv_k4a = k
        return _prev_tc(self)

    S.HRCSensingNode.timer_callback = _tc_stamp
    import atexit as _atexit
    def _fin():
        t_end = _time.time() + 8.0
        while not _Q.empty() and _time.time() < t_end:   # let the writer finish the queued frames
            _time.sleep(0.05)
        print(f"[lv-sensing] colour frames saved {_ST['saved']}, dropped {_ST['dropped']}, left in queue {_Q.qsize()}", flush=True)
    _atexit.register(_fin)
print(f"[lv-sensing] IMU measurement log -> {OUT}/debug_log, NOVIS {os.environ.get('STEP5_SENSING_NOVIS')}, SAVEVIS {SAVEVIS}", flush=True)
try:
    rc = S.main()
finally:
    print(f"[lv-sensing] end, rewrites {RD.REWRITES}, blocked writes {len(RD.BLOCKED)} {RD.BLOCKED[:5]}", flush=True)
sys.exit(rc)

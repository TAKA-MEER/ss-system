"""`mount_calib` の試験（偽のカメラ・偽の検出器。学習済みモデルは使わない）。"""
import json
import math

import numpy as np
import pytest

from ss_locator import geometry as G
from ss_locator.locate_core import CameraState
from ss_locator.mount_calib import main

K = np.array([[1000.0, 0.0, 640.0], [0.0, 1000.0, 360.0], [0.0, 0.0, 1.0]])
DIST = np.zeros(5)
TOP_HEIGHT_M = 1.0
HEIGHT_M = 2.6
MOUNT = dict(mount_x_m=0.0, mount_y_m=0.0, forward_offset_m=0.08,
             optical_offset_m=0.06)
LUGS = np.array([[3.0, -0.8], [3.0, 0.8], [4.6, 0.0]])
POSES_DEG = [(-10.0, 25.0), (-4.0, 30.0), (0.0, 22.0), (4.0, 28.0), (10.0, 24.0)]
TRUE = dict(pitch_offset_rad=math.radians(-2.6),
            yaw_offset_rad=math.radians(-5.4),
            roll_rad=math.radians(4.1))


class FakeCalibCamera:
    def __init__(self, pose_known=True, take_over_after=None):
        self.yaw_rad = 0.0
        self.pitch_down_rad = 0.0
        self.pose_known = pose_known
        self.take_over_after = take_over_after
        self.points = []
        self.released = False

    def point(self, yaw_rad, pitch_down_rad):
        self.points.append((float(yaw_rad), float(pitch_down_rad)))
        self.yaw_rad = float(yaw_rad)
        self.pitch_down_rad = float(pitch_down_rad)

    def release(self):
        self.released = True

    def state(self):
        taken = (self.take_over_after is not None
                 and len(self.points) >= self.take_over_after)
        return CameraState(self.yaw_rad, self.pitch_down_rad, False,
                           self.pose_known, True, taken, False)

    def capture(self, on_done):
        on_done("use", {"image": np.zeros((720, 1280, 3), np.uint8),
                        "yaw_rad": self.yaw_rad,
                        "pitch_down_rad": self.pitch_down_rad})


class FakeCalibDetector:
    def __init__(self, camera):
        self.camera = camera

    def detect(self, image):
        T = G.camera_T(HEIGHT_M, self.camera.yaw_rad,
                       self.camera.pitch_down_rad, **MOUNT, **TRUE)
        pts = np.c_[LUGS, np.full(len(LUGS), TOP_HEIGHT_M)]
        uv, _ok = G.project(pts, K, DIST, T)
        kp = np.zeros((len(uv), 4, 2), np.float32)
        kp[:, 0] = uv
        return kp, np.ones(len(uv), np.float32)


def _args(tmp_path, camera, extra=()):
    camera_holder = {"camera": camera}
    detector = FakeCalibDetector(camera)
    lugs = ";".join("%.2f,%.2f" % (x, y) for x, y in LUGS)
    poses = ",".join("%.1f:%.1f" % p for p in POSES_DEG)
    params = {
        "camera_K": K.ravel().tolist(), "camera_dist": DIST.tolist(),
        "camera_mount_x_m": 0.0, "camera_mount_y_m": 0.0,
        "camera_forward_offset_m": 0.08, "camera_optical_offset_m": 0.06,
        "camera_height_offset_m": 0.0, "settle_s": 0.0, "aim_timeout_s": 1.0,
    }
    params_path = tmp_path / "params.yaml"
    import yaml
    params_path.write_text(yaml.safe_dump({"locator_node": {"ros__parameters": params}}))
    argv = ["--lugs", lugs, "--top-height-m", str(TOP_HEIGHT_M),
            "--lift-height-m", str(HEIGHT_M), "--poses=" + poses,
            "--params-file", str(params_path), "--out", str(tmp_path / "out")]
    argv += list(extra)
    return argv, lambda: camera_holder["camera"], lambda: detector


def test_calib_offline(tmp_path, capsys):
    observations = []
    for yaw_deg, pitch_deg in POSES_DEG:
        T = G.camera_T(HEIGHT_M, math.radians(yaw_deg), math.radians(pitch_deg),
                       **MOUNT, **TRUE)
        pts = np.c_[LUGS, np.full(len(LUGS), TOP_HEIGHT_M)]
        uv, ok = G.project(pts, K, DIST, T, 1280, 720)
        assert ok.all()
        observations.append({"yaw_rad": math.radians(yaw_deg),
                             "pitch_down_rad": math.radians(pitch_deg),
                             "pixels": uv.tolist(), "photo": "frame.jpg"})
    data = {"lugs": LUGS.tolist(), "top_height_m": TOP_HEIGHT_M,
            "lift_height_m": HEIGHT_M, "height_m": HEIGHT_M,
            "camera_K": K.ravel().tolist(), "camera_dist": DIST.tolist(),
            "camera_mount_x_m": 0.0, "camera_mount_y_m": 0.0,
            "camera_forward_offset_m": 0.08, "camera_optical_offset_m": 0.06,
            "camera_height_offset_m": 0.0, "observations": observations}
    (tmp_path / "observations.json").write_text(json.dumps(data))
    code = main(["--from", str(tmp_path), "--params-file", str(tmp_path / "none.yaml")])
    assert code == 0
    lines = {ln.split(":")[0]: float(ln.split(":")[1])
             for ln in capsys.readouterr().out.splitlines()
             if ln.startswith("camera_") and ":" in ln}
    assert abs(lines["camera_pitch_offset_deg"] - (-2.6)) < 0.05
    assert abs(lines["camera_yaw_offset_deg"] - (-5.4)) < 0.05
    assert abs(lines["camera_roll_deg"] - 4.1) < 0.05


def test_calib_pose_unknown(tmp_path):
    camera = FakeCalibCamera(pose_known=False)
    argv, cam_f, det_f = _args(tmp_path, camera)
    code = main(argv, camera_factory=cam_f, detector_factory=det_f)
    assert code != 0
    assert camera.points == []


def test_calib_restores(tmp_path):
    camera = FakeCalibCamera()
    argv, cam_f, det_f = _args(tmp_path, camera)
    code = main(argv, camera_factory=cam_f, detector_factory=det_f)
    assert code == 0
    # 撮り終えたら、撮る前の向きへ point してから release
    assert camera.points[-1] == (0.0, 0.0)
    assert camera.released
    assert (tmp_path / "out" / "observations.json").exists()
    assert (tmp_path / "out" / "frame_000.jpg").exists()


def test_calib_taken_over_stops(tmp_path):
    camera = FakeCalibCamera(take_over_after=1)
    argv, cam_f, det_f = _args(tmp_path, camera)
    code = main(argv, camera_factory=cam_f, detector_factory=det_f)
    assert len(camera.points) == 1  # それ以上 point しない


def test_calib_offline_fit_height_override(tmp_path, capsys):
    # --fit-height は json の値（ここでは false）より優先し、高さも当てはめる。
    # 画素は高さ 2.9 で生成し、json の名目高さは 2.6。
    observations = []
    for yaw_deg, pitch_deg in POSES_DEG:
        T = G.camera_T(2.9, math.radians(yaw_deg), math.radians(pitch_deg),
                       **MOUNT, **TRUE)
        pts = np.c_[LUGS, np.full(len(LUGS), TOP_HEIGHT_M)]
        uv, ok = G.project(pts, K, DIST, T, 1280, 720)
        assert ok.all()
        observations.append({"yaw_rad": math.radians(yaw_deg),
                             "pitch_down_rad": math.radians(pitch_deg),
                             "pixels": uv.tolist(), "photo": "frame.jpg"})
    data = {"lugs": LUGS.tolist(), "top_height_m": TOP_HEIGHT_M,
            "lift_height_m": 2.6, "height_m": 2.6,
            "camera_K": K.ravel().tolist(), "camera_dist": DIST.tolist(),
            "camera_mount_x_m": 0.0, "camera_mount_y_m": 0.0,
            "camera_forward_offset_m": 0.08, "camera_optical_offset_m": 0.06,
            "camera_height_offset_m": 0.0, "fit_height": False,
            "observations": observations}
    (tmp_path / "observations.json").write_text(json.dumps(data))
    code = main(["--from", str(tmp_path), "--fit-height",
                 "--params-file", str(tmp_path / "none.yaml")])
    assert code == 0
    out = capsys.readouterr().out
    heights = [ln for ln in out.splitlines() if ln.startswith("height_m:")]
    assert len(heights) == 1
    assert abs(float(heights[0].split(":")[1]) - 2.9) < 0.01
    lines = {ln.split(":")[0]: float(ln.split(":")[1])
             for ln in out.splitlines()
             if ln.startswith("camera_") and ":" in ln}
    assert abs(lines["camera_pitch_offset_deg"] - (-2.6)) < 0.05
    assert abs(lines["camera_yaw_offset_deg"] - (-5.4)) < 0.05
    assert abs(lines["camera_roll_deg"] - 4.1) < 0.05

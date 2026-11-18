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


def _offsets_from_output(text):
    lines = {ln.split(":")[0]: float(ln.split(":")[1])
             for ln in text.splitlines()
             if ln.startswith("camera_") and ":" in ln}
    return lines


def test_calib_real_mock_outlier(tmp_path, capsys):
    # 2026-11-18 の実機の観測（DetailedDesign-packets.md WP-CAL-03）。
    # (340, 110) が外され、ヨー −1.5°・ロール +3.2° 付近になり 0 で終わる。
    raw = [
        (0.0, 25.0, [(170, 609)]),
        (13.5, 25.0, [(258, 529), (591, 547), (340, 110)]),
        (23.1, 25.0, [(449, 503), (777, 539)]),
        (35.7, 25.0, [(752, 482), (1088, 550)]),
        (46.7, 25.0, [(1075, 489)]),
        (24.1, 20.0, [(492, 578), (820, 616)]),
        (23.5, 30.0, [(465, 375), (790, 416)]),
    ]
    data = {
        "lugs": [[1.989, 0.575], [1.989, 1.095]],
        "top_height_m": 0.78, "lift_height_m": 0.0, "height_m": 1.82,
        "camera_K": [1482.80, 0.0, 638.13, 0.0, 1482.66, 367.04, 0.0, 0.0, 1.0],
        "camera_dist": [0.02116, -0.27909, 0.0, 0.0, 0.0],
        "camera_mount_x_m": 0.0, "camera_mount_y_m": 0.0,
        "camera_forward_offset_m": 0.08, "camera_optical_offset_m": 0.06,
        "camera_height_offset_m": 1.82,
        "observations": [
            {"yaw_rad": math.radians(yaw), "pitch_down_rad": math.radians(pitch),
             "pixels": [list(p) for p in pixels], "photo": "frame.jpg"}
            for yaw, pitch, pixels in raw
        ],
    }
    (tmp_path / "observations.json").write_text(json.dumps(data))
    code = main(["--from", str(tmp_path),
                 "--params-file", str(tmp_path / "none.yaml")])
    assert code == 0
    out = capsys.readouterr().out
    lines = _offsets_from_output(out)
    assert abs(lines["camera_yaw_offset_deg"] - (-1.5)) < 0.3
    assert abs(lines["camera_roll_deg"] - 3.2) < 0.3
    rejected_lines = [ln for ln in out.splitlines() if ln.startswith("rejected:")]
    assert len(rejected_lines) == 1
    assert "340.0" in rejected_lines[0] and "110.0" in rejected_lines[0]


def test_calib_reject_m_passed(tmp_path, capsys):
    # --reject-m 5.0（既定と違う値）で回すと、遠い検出を外さない。
    # 道具が引数を渡さない変異（既定 0.3 で動く）を落とす。
    observations = []
    for yaw_deg, pitch_deg in POSES_DEG:
        T = G.camera_T(HEIGHT_M, math.radians(yaw_deg), math.radians(pitch_deg),
                       **MOUNT, **TRUE)
        pts = np.c_[LUGS, np.full(len(LUGS), TOP_HEIGHT_M)]
        uv, ok = G.project(pts, K, DIST, T, 1280, 720)
        assert ok.all()
        uv = np.vstack([uv, [[1279.0, 360.0]]])
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
    code = main(["--from", str(tmp_path), "--reject-m", "5.0",
                 "--params-file", str(tmp_path / "none.yaml")])
    assert code == 0
    out = capsys.readouterr().out
    assert "rejected:" not in out
    # 既定（0.3）なら同じ観測で外す。
    code = main(["--from", str(tmp_path),
                 "--params-file", str(tmp_path / "none.yaml")])
    assert code == 0
    out = capsys.readouterr().out
    assert out.count("rejected:") == len(POSES_DEG)


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

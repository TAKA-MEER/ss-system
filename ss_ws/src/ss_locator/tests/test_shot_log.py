"""撮影の記録（`WP-TEST-03`）の試験。ノードの手本は `test_locator_node.py`。"""
import json
import math
from pathlib import Path

import cv2
import numpy as np
import pytest
import rclpy

from camera_stub_server import CameraStubServer
from ss_locator import locator_node, shot_log
from ss_locator.locate_core import FrameDone
from ss_locator.locator_node import LocatorNode
from test_locator_node import (
    REAL_LIFT_STATE,
    Rig,
    _jpeg,
    _real_parameters,
    _start_lift_server,
    parameters,
)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class _RecordingNode(LocatorNode):
    """`_frame_events` が返した出来事を残す。"""

    def __init__(self, **kwargs):
        self.frame_events = []
        super().__init__(**kwargs)

    def _frame_events(self, verdict, captured):
        events = super()._frame_events(verdict, captured)
        self.frame_events.append(events)
        return events


def _read_shots(directory):
    path = Path(directory) / "shots.jsonl"
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def _done_events(node):
    return [
        e
        for events in node.frame_events
        for e in events
        if isinstance(e, FrameDone) and e.ok
    ]


def _lug_points(event):
    return [[float(lug["p"][0]), float(lug["p"][1])] for lug in event.lugs]


def test_shot_log_off_by_default(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    rig = Rig(node_class=_RecordingNode)
    try:
        result = rig.get_result(rig.send_goal())
        assert result.reason == "completed"
        assert _done_events(rig.node)
    finally:
        rig.close()
    assert list(tmp_path.iterdir()) == []


def test_shot_log_writes_lines(tmp_path):
    shots = tmp_path / "shots"
    rig = Rig(node_class=_RecordingNode, parameter_overrides=parameters(shot_log_dir=str(shots)))
    try:
        result = rig.get_result(rig.send_goal())
        assert result.reason == "completed"
        done = _done_events(rig.node)
    finally:
        rig.close()
    rows = _read_shots(shots)
    assert done and len(rows) == len(done)
    assert [r["shot"] for r in rows] == list(range(len(done)))
    keys = {
        "time", "shot", "lift_height_m", "camera_height_m", "yaw_rad", "pitch_down_rad",
        "keypoints", "confidences", "lugs", "photo", "camera_pitch_offset_deg",
        "camera_yaw_offset_deg", "camera_roll_deg", "camera_height_offset_m",
        "top_height_default_m",
    }
    for row, event in zip(rows, done):
        assert keys <= set(row)
        assert row["lift_height_m"] is None and row["camera_height_m"] is None
        assert row["photo"] is None
        assert row["top_height_default_m"] == 2.35
        assert row["lugs"] == _lug_points(event)
        assert len(row["keypoints"]) == len(row["confidences"])
        assert all(len(k) == 4 and len(k[0]) == 2 for k in row["keypoints"])
        assert row["yaw_rad"] == pytest.approx(event.yaw_rad)
    assert any(r["lugs"] for r in rows)


class _OneLugDetector:
    """float32 の検出を 1 つ返す偽の検出器（numpy の型が JSON に出ないことも見る）。"""

    def __init__(self, model_file, model_sha256, imgsz, conf):
        pass

    def detect(self, image):
        kp = np.array([[[600, 300], [640, 300], [620, 280], [620, 320]]], np.float32)
        return kp, np.array([0.9], np.float32)


def _real_shot_run(tmp_path, monkeypatch, **extra):
    monkeypatch.setattr(locator_node, "Detector", _OneLugDetector)
    shots = tmp_path / "shots"
    lift_server = _start_lift_server()
    camera_server = CameraStubServer()
    camera_server.start()
    camera_server.set_auto_state(
        {
            "t": "state", "seq": 1, "boot_id": "boot-1", "pitch_deg": -20.0,
            "yaw_deg": 10.0, "yaw_zeroed": True, "yaw_dps": 10.0, "owner": None,
            "owner_name": None, "owner_operator": None, "aim": None,
        },
        arrive_on_aim=True,
    )
    camera_server.set_snapshots(
        [
            (
                200,
                {
                    "X-Boot-Id": "boot-1", "X-Yaw-Deg": 30.0, "X-Pitch-Deg": -35.0,
                    "X-Still-Ms": 1000, "X-Frame-Age-Ms": 10,
                },
                _jpeg(),
            )
        ]
    )
    overrides = _real_parameters(
        lift_server,
        "127.0.0.1:%d" % camera_server.port,
        shot_log_dir=str(shots),
        camera_pitch_offset_deg=-2.6,
        **extra,
    )
    rig = Rig(node_class=_RecordingNode, parameter_overrides=overrides)
    try:
        rig.wait_until(
            lambda: rig.node._core._lift is not None
            and rig.node._core._lift.height_m is not None
            and rig.node._core._camera is not None
            and rig.node._core._camera.pose_known,
            10.0,
            "昇降部・カメラの状態が届かない",
        )
        goal_handle = rig.send_goal(azimuth_from_rad=0.0, azimuth_span_rad=0.3)
        assert goal_handle.accepted

        def gotos():
            return [m for m in lift_server.messages if m.get("t") == "goto"]

        rig.wait_until(lambda: gotos(), 10.0, "goto が届かない")
        target_mm = gotos()[0]["target_mm"]
        arrived = dict(REAL_LIFT_STATE)
        arrived["height_mm"] = target_mm
        arrived["goto"] = {
            "by": "ss", "press": gotos()[0]["press"], "target_mm": target_mm,
            "status": "ARRIVED", "reason": "NONE",
        }
        lift_server.set_auto_state(arrived)
        rig.get_result(goal_handle, timeout=20.0)
        done = _done_events(rig.node)
    finally:
        rig.close()
        camera_server.stop()
        lift_server.stop()
    return shots, done


def test_shot_log_real_path_fields(tmp_path, monkeypatch):
    shots, done = _real_shot_run(tmp_path, monkeypatch)
    rows = _read_shots(shots)
    assert done and len(rows) == len(done)
    row = rows[0]
    assert row["lift_height_m"] == pytest.approx(0.15)
    assert row["camera_height_m"] == pytest.approx(0.15 + 0.05)
    assert row["yaw_rad"] == pytest.approx(math.radians(30.0))
    assert row["pitch_down_rad"] == pytest.approx(math.radians(35.0))
    assert row["camera_pitch_offset_deg"] == pytest.approx(-2.6)
    assert row["camera_height_offset_m"] == pytest.approx(0.05)
    assert row["keypoints"] == [[[600.0, 300.0], [640.0, 300.0], [620.0, 280.0], [620.0, 320.0]]]
    assert row["confidences"] == [pytest.approx(0.9)]
    assert row["lugs"] == _lug_points(done[0])


@pytest.mark.parametrize("save", [False, True])
def test_shot_images_only_when_asked(tmp_path, monkeypatch, save):
    shots, _ = _real_shot_run(tmp_path, monkeypatch, shot_save_images=save)
    rows = _read_shots(shots)
    assert rows
    jpgs = sorted(p.name for p in shots.glob("*.jpg"))
    if save:
        assert rows[0]["photo"] == "shot_000000.jpg"
        assert "shot_000000.jpg" in jpgs
        assert cv2.imread(str(shots / "shot_000000.jpg")) is not None
    else:
        assert jpgs == []
        assert all(r["photo"] is None for r in rows)


def _fake_frame(node):
    node._camera.point(0.0, 0.3)
    captured = []
    node._camera.capture(lambda verdict, c: captured.append((verdict, c)))
    return captured[0]


def test_shot_log_failure_does_not_break(tmp_path):
    blocker = tmp_path / "blocker"
    blocker.write_text("file")
    plain = LocatorNode(parameter_overrides=parameters())
    broken = LocatorNode(parameter_overrides=parameters(shot_log_dir=str(blocker)))
    try:
        expected = repr(plain._frame_events(*_fake_frame(plain)))
        actual = repr(broken._frame_events(*_fake_frame(broken)))
        assert actual == expected
        assert "FrameDone" in actual and "True" in actual
        # 2 枚目も流れる（警告は 1 回で、以後は黙る）
        assert repr(broken._frame_events(*_fake_frame(broken))) == expected
        assert broken._shot_log_warned
    finally:
        plain.destroy_node()
        broken.destroy_node()


def test_shot_record_is_plain_json():
    record = shot_log.shot_record(
        3,
        lift_height_m=np.float32(0.5),
        camera_height_m=None,
        yaw_rad=np.float64(1.0),
        pitch_down_rad=None,
        keypoints=np.zeros((1, 4, 2), np.float32),
        confidences=np.array([0.5], np.float32),
        lugs=[{"p": np.array([1.0, 2.0])}],
        photo=None,
        fit_parameters={"camera_roll_deg": np.float32(1.0)},
    )
    json.dumps(record)
    assert record["lugs"] == [[1.0, 2.0]]

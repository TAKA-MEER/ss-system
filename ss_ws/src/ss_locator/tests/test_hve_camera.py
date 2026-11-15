"""`HveCamera` の試験。試験の中で起こす小さなサーバ（偽のカメラモジュール・昇降部）だけを相手にする。"""
import math
import threading
import time

import cv2
import numpy as np
import pytest

from ss_locator.hve_camera import HveCamera, host_from_modules
from ss_locator.hve_lift import HveLift

from camera_stub_server import CameraStubServer
from lift_stub_server import LiftStubServer

BOOT = "boot-1"


def wait_until(condition, timeout=5.0, interval=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = condition()
        if value:
            return value
        time.sleep(interval)
    raise AssertionError("条件が満たされない")


def jpeg(width=1280, height=720):
    ok, encoded = cv2.imencode(".jpg", np.full((height, width, 3), 128, np.uint8))
    assert ok
    return encoded.tobytes()


def state(aim=None, **extra):
    base = {
        "t": "state",
        "seq": 1,
        "boot_id": BOOT,
        "pitch_deg": -20.0,
        "yaw_deg": 10.0,
        "yaw_zeroed": True,
        "yaw_dps": 10.0,
        "pitch_min_deg": -45.0,
        "pitch_max_deg": 45.0,
        "owner": None,
        "owner_name": None,
        "owner_operator": None,
        "aim": aim,
    }
    base.update(extra)
    return base


def headers(yaw=30.0, pitch=-35.0, still=1000, age=10, boot=BOOT):
    return {
        "X-Boot-Id": boot,
        "X-Yaw-Deg": yaw,
        "X-Pitch-Deg": pitch,
        "X-Still-Ms": still,
        "X-Frame-Age-Ms": age,
        "Content-Type": "image/jpeg",
    }


@pytest.fixture
def server():
    srv = CameraStubServer()
    srv.start()
    yield srv
    srv.stop()


@pytest.fixture
def make_camera(server):
    cameras = []

    def factory(host=None, **kwargs):
        params = dict(
            cmd_period_ms=50,
            state_timeout_ms=500,
            snapshot_min_interval_ms=200,
            image_timeout_s=3.0,
        )
        params.update(kwargs)
        camera = HveCamera(
            "127.0.0.1:%d" % server.port if host is None else host, **params
        )
        cameras.append(camera)
        return camera

    yield factory
    for camera in cameras:
        camera.close()


def aims(server):
    return [m for m in server.messages if m.get("t") == "aim"]


def capture(camera, timeout=8.0):
    done = threading.Event()
    box = {}

    def on_done(verdict, result):
        box["verdict"] = verdict
        box["result"] = result
        done.set()

    camera.capture(on_done)
    assert done.wait(timeout), "静止画の結果が来ない"
    return box["verdict"], box["result"]


def test_hve_camera_hello_first(server, make_camera):
    server.set_auto_state(state())
    make_camera()
    first = wait_until(lambda: server.connections and server.connections[0])
    assert first[0]["t"] == "hello"
    assert first[0]["name"] == "ss"
    assert "ceiling_sensor" not in first[0]
    # 切って繋ぎ直させても、また hello から
    server.close_connections()
    second = wait_until(
        lambda: len(server.connections) >= 2 and server.connections[1], timeout=6.0
    )
    assert second[0]["t"] == "hello"


def test_hve_camera_aim_and_release(server, make_camera):
    server.set_auto_state(state())
    camera = make_camera()
    wait_until(lambda: camera.state().pose_known)
    camera.point(0.5, 0.3)
    wait_until(lambda: len(aims(server)) >= 3)  # 続けて届く
    first = aims(server)[0]
    assert first["press"] == 1
    assert first["yaw_deg"] == pytest.approx(math.degrees(0.5))
    assert first["pitch_deg"] == pytest.approx(-math.degrees(0.3))  # 見上げが正
    assert camera.state().arrived is False

    server.set_auto_state(
        state(
            aim={"by": "ss", "press": 1, "pitch_deg": first["pitch_deg"],
                 "yaw_deg": first["yaw_deg"], "status": "ARRIVED", "reason": "NONE"}
        )
    )
    wait_until(lambda: camera.state().arrived)
    time.sleep(0.15)
    count = len(aims(server))
    time.sleep(0.3)
    assert len(aims(server)) == count  # aim が止まる

    camera.release()
    wait_until(lambda: any(m.get("t") == "release" for m in server.messages))
    releases = [m for m in server.messages if m.get("t") == "release"]
    assert releases == [{"t": "release", "press": 1}]


def test_hve_camera_capture_uses_headers(server, make_camera):
    server.set_auto_state(state())  # state の向きは yaw 10°・pitch -20°
    server.set_snapshots([(200, headers(yaw=30.0, pitch=-35.0), jpeg())])
    camera = make_camera()
    wait_until(lambda: camera.state().pose_known)
    camera.point(0.5, 0.3)
    verdict, result = capture(camera)
    assert verdict == "use"
    assert result["image"].shape == (720, 1280, 3)
    assert result["yaw_rad"] == pytest.approx(math.radians(30.0))
    assert result["pitch_down_rad"] == pytest.approx(math.radians(35.0))  # 符号を逆に
    # 要求には名乗り（?by=ss）が付く
    assert server.snapshot_requests[0][1] == "/api/snapshot?by=ss"


def test_hve_camera_capture_retries(server, make_camera):
    server.set_auto_state(state())
    server.set_snapshots(
        [
            (503, {}, b""),
            (200, headers(still=50, age=100), jpeg()),  # 動いている間のコマ
            (200, headers(yaw=45.0, pitch=-10.0), jpeg()),
        ]
    )
    camera = make_camera(snapshot_min_interval_ms=200)
    wait_until(lambda: camera.state().pose_known)
    camera.point(0.5, 0.3)
    verdict, result = capture(camera)
    assert verdict == "use"
    assert result["yaw_rad"] == pytest.approx(math.radians(45.0))
    times = [t for t, _ in server.snapshot_requests]
    assert len(times) == 3
    for before, after in zip(times, times[1:]):
        assert after - before >= 0.2 - 0.01


def test_hve_camera_capture_times_out(server, make_camera):
    server.set_auto_state(state())
    server.set_snapshots([(503, {}, b"")])  # 503 が続く
    camera = make_camera(snapshot_min_interval_ms=100, image_timeout_s=0.6)
    wait_until(lambda: camera.state().pose_known)
    camera.point(0.5, 0.3)
    start = time.monotonic()
    verdict, result = capture(camera)
    elapsed = time.monotonic() - start
    assert verdict == "fail"
    assert result is None
    assert 0.5 <= elapsed < 3.0


def test_hve_camera_capture_pose_unknown_and_bad_images(server, make_camera):
    server.set_auto_state(state())
    camera = make_camera(snapshot_min_interval_ms=50)
    wait_until(lambda: camera.state().pose_known)
    camera.point(0.5, 0.3)

    server.set_snapshots([(200, headers(boot="other"), jpeg())])  # 再起動した
    assert capture(camera) == ("pose_unknown", None)
    server.set_snapshots([(200, headers(), b"not a jpeg")])
    assert capture(camera) == ("fail", None)
    server.set_snapshots([(200, headers(), jpeg(640, 480))])  # 1280×720 でない
    assert capture(camera) == ("fail", None)


def test_hve_camera_host_from_lift():
    camera_server = CameraStubServer()
    camera_server.start()
    lift_server = LiftStubServer()
    lift_server.start()
    camera_server.set_auto_state(state())
    lift_server.set_settings(
        {
            "lift_up": {"min": 10, "max": 60, "init": 30},
            "lift_down": {"min": 10, "max": 60, "init": 20},
        }
    )
    lift_server.set_auto_state(
        {
            "t": "state",
            "height_mm": 1000,
            "height_ok": True,
            "owner": None,
            "owner_name": None,
            "owner_operator": None,
            "goto": None,
            "modules": {"hve-cam": {"ip": "127.0.0.1:%d" % camera_server.port}},
        }
    )
    lift = HveLift("127.0.0.1:%d" % lift_server.port, cmd_period_ms=50, state_timeout_ms=500)
    camera = HveCamera("", lift=lift, cmd_period_ms=50, state_timeout_ms=500)
    try:
        first = wait_until(
            lambda: camera_server.connections and camera_server.connections[0]
        )
        assert first[0]["t"] == "hello"
        wait_until(lambda: camera.state().pose_known)
    finally:
        camera.close()
        lift.close()
        camera_server.stop()
        lift_server.stop()


def test_hve_camera_no_host_does_not_connect():
    camera = HveCamera("", lift=None, cmd_period_ms=50)
    try:
        time.sleep(0.3)
        assert camera.state().pose_known is False
        camera.point(0.0, 0.1)
        assert camera.state().failed is True
    finally:
        camera.close()


def test_host_from_modules_shapes():
    assert host_from_modules({"hve-cam": {"ip": "10.0.0.5"}}) == "10.0.0.5"
    assert host_from_modules([{"name": "hve-cam", "ip": "10.0.0.6"}]) == "10.0.0.6"
    assert host_from_modules({"hve-lift": {"ip": "10.0.0.7"}}) is None
    assert host_from_modules({"hve-cam": {"ip": ""}}) is None
    assert host_from_modules(None) is None

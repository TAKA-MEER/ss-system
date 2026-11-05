import json
import math
from pathlib import Path

import numpy as np
import pytest

from ss_locator import assemble, scan_plan
from ss_locator.fakes import FakeCamera, FakeDetector, FakeLift
from ss_locator.locate_core import (
    CameraPoint,
    Cancel,
    FrameDone,
    GoalRequest,
    LiftGoto,
    LocateCore,
    RequestFrame,
    SendCleanupDone,
    SendFeedback,
    SendResult,
    Tick,
)

FIXTURE = Path(__file__).with_name("fixtures") / "proto_9265d2d.json"
TAU = 2.0 * math.pi


def _fixture():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def _hfov():
    first = _fixture()["scenes"][0]["views"][0]
    return scan_plan.hfov_from_fx(float(first["K"][0][0]))


def make_params(**overrides):
    params = {
        "max_distance_default_m": 5.0,
        "top_height_default_m": 2.35,
        "scan_camera_height_m": 3.10,
        "min_camera_height_m": 2.95,
        "scan_pitch_down_deg": 20.0,
        "scan_overlap_ratio": 0.1,
        "settle_s": 0.5,
        "image_timeout_s": 5.0,
        "cleanup_timeout_s": 30.0,
        "camera_height_offset_m": 0.0,
        "hfov_rad": _hfov(),
    }
    params.update(overrides)
    return params


def goal(**overrides):
    values = {
        "azimuth_from_rad": 0.0,
        "azimuth_span_rad": TAU,
        "max_distance_m": 0.0,
        "time_limit_s": 120.0,
    }
    values.update(overrides)
    return GoalRequest(**values)


class EmptyDetector:
    def detect(self, view):
        return np.empty((0, 4, 2)), np.empty((0,))


class OneLugOnceDetector:
    """最初の 1 回だけ金具を 1 個返す。盤には組めない。"""

    def __init__(self):
        self._detector = FakeDetector(FIXTURE, scene=0)
        self._used = False

    def detect(self, view):
        if self._used:
            return np.empty((0, 4, 2)), np.empty((0,))
        self._used = True
        kp, conf = self._detector.detect(view)
        return kp[:1], conf[:1]


class Rig:
    """偽の昇降機・カメラ・検出器と `LocateCore` をつなぐ小さな回し役。"""

    def __init__(self, params=None, lift=None, camera=None, detector=None, scene=0):
        self.params = make_params() if params is None else params
        self.core = LocateCore(self.params)
        self.lift = lift or FakeLift(
            height_m=self.params["scan_camera_height_m"]
            - self.params["camera_height_offset_m"],
            speed_m_s=10.0,
        )
        self.camera = camera or FakeCamera(FIXTURE, scene=scene)
        self.detector = detector or FakeDetector(FIXTURE, scene=scene)
        self.now = 0.0
        self.queue = []
        self.commands = []

    def post(self, event):
        self.queue.append(event)

    def prime(self):
        self.post(self.lift.state())
        self.post(self.camera.state())
        self.flush()

    def accept(self, req):
        return self.core.accept(req, self.now)

    def flush(self):
        produced = []
        while self.queue:
            event = self.queue.pop(0)
            commands = self.core.step(event, self.now)
            self.commands.extend(commands)
            produced.extend(commands)
            for command in commands:
                self._handle(command)
        return produced

    def advance(self, dt):
        self.now += float(dt)
        self.post(self.lift.advance(dt))
        self.post(Tick())
        return self.flush()

    def _handle(self, command):
        if isinstance(command, (LiftGoto,)):
            self.lift.apply(command)
        elif isinstance(command, RequestFrame):
            captured = self.camera.capture()
            if captured is None:
                self.post(FrameDone(self.camera.yaw_rad, [], False))
                return
            kp, conf = self.detector.detect(captured["view"])
            obs = assemble.lift(
                kp,
                conf,
                (captured["K"], captured["dist"], captured["T"]),
                captured["view"],
                self.params["top_height_default_m"],
            )
            self.post(FrameDone(self.camera.yaw_rad, obs, True))
        elif isinstance(command, CameraPoint):
            self.camera.point(command.yaw_rad, command.pitch_down_rad)

    def start_scan(self):
        self.advance(0.0)  # 昇降部へ
        target = self.params["scan_camera_height_m"] - self.params["camera_height_offset_m"]
        distance = abs(target - self.lift.height_m)
        dt = distance / self.lift.speed_m_s if self.lift.speed_m_s > 0.0 else 0.0
        self.advance(dt + 1e-9)  # 着いて、最初のヨーを向く

    def finish_cleanup(self, dt=1.0, steps=100):
        for _ in range(steps):
            if self.core.state == LocateCore.IDLE:
                break
            self.advance(dt)

    def results(self):
        return [c for c in self.commands if isinstance(c, SendResult)]

    def cleanup_dones(self):
        return [c for c in self.commands if isinstance(c, SendCleanupDone)]


def run_one_frame(rig, dt=None):
    rig.start_scan()
    rig.advance(rig.params["settle_s"] if dt is None else dt)


# --------------------------------------------------------------------- 受け付け
def test_core_reject_bad_range():
    rig = Rig()
    rig.prime()
    assert rig.accept(goal(azimuth_span_rad=0.0)) == "bad_range"
    assert rig.accept(goal(azimuth_span_rad=-0.1)) == "bad_range"
    assert rig.accept(goal(azimuth_span_rad=TAU + 0.01)) == "bad_range"
    assert rig.core.state == LocateCore.IDLE


def test_core_reject_bad_distance():
    rig = Rig()
    rig.prime()
    assert rig.accept(goal(max_distance_m=-0.1)) == "bad_distance"
    assert rig.core.state == LocateCore.IDLE


def test_core_reject_bad_time_limit():
    rig = Rig()
    rig.prime()
    assert rig.accept(goal(time_limit_s=0.0)) == "bad_time_limit"
    assert rig.accept(goal(time_limit_s=-1.0)) == "bad_time_limit"
    assert rig.core.state == LocateCore.IDLE


def test_core_reject_busy():
    rig = Rig(lift=FakeLift(height_m=0.0, speed_m_s=0.2))
    rig.prime()
    assert rig.accept(goal()) is None
    rig.advance(0.1)
    assert rig.core.state == LocateCore.RUNNING
    assert rig.accept(goal()) == "busy"

    # 全部撮り終えて、後片付け中でも拒否する
    rig.start_scan()
    for _ in range(len(rig.core._yaws)):
        rig.advance(1.0)
    assert rig.core.state == LocateCore.CLEANING
    assert rig.accept(goal()) == "busy"


def test_core_accept_distance_clip():
    rig = Rig()
    rig.prime()
    assert rig.accept(goal(max_distance_m=0.0)) is None
    assert rig.core._max_distance == rig.params["max_distance_default_m"]

    rig = Rig()
    rig.prime()
    assert rig.accept(goal(max_distance_m=100.0)) is None
    assert rig.core._max_distance == rig.params["max_distance_default_m"]

    rig = Rig()
    rig.prime()
    assert rig.accept(goal(max_distance_m=2.0)) is None
    assert rig.core._max_distance == 2.0


# --------------------------------------------------------------------- reason
def test_core_reason_completed():
    rig = Rig()
    rig.prime()
    assert rig.accept(goal()) is None
    rig.start_scan()
    for _ in range(len(rig.core._yaws)):
        rig.advance(rig.params["settle_s"])
    rig.finish_cleanup(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "completed"
    assert len(results[0].panels) >= 1
    assert results[0].seen_span[0] == pytest.approx(TAU)


def test_core_reason_no_lugs():
    rig = Rig(detector=EmptyDetector())
    rig.prime()
    assert rig.accept(goal()) is None
    rig.start_scan()
    for _ in range(len(rig.core._yaws)):
        rig.advance(rig.params["settle_s"])
    rig.finish_cleanup(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "no_lugs"
    assert results[0].panels == []


def test_core_reason_lugs_ungrouped():
    rig = Rig(detector=OneLugOnceDetector())
    rig.prime()
    assert rig.accept(goal()) is None
    rig.start_scan()
    for _ in range(len(rig.core._yaws)):
        rig.advance(rig.params["settle_s"])
    rig.finish_cleanup(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "lugs_ungrouped"
    assert results[0].panels == []


def test_core_reason_no_image():
    rig = Rig(camera=FakeCamera(FIXTURE, scene=0, fail=True))
    rig.prime()
    assert rig.accept(goal()) is None
    rig.start_scan()
    rig.advance(rig.params["settle_s"])
    rig.finish_cleanup(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "no_image"


def test_core_reason_lift_refused():
    rig = Rig(lift=FakeLift(height_m=0.0, speed_m_s=10.0, refuse=True))
    rig.prime()
    assert rig.accept(goal()) is None
    rig.advance(0.0)
    rig.advance(0.0)
    rig.finish_cleanup(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "lift_refused"
    dones = rig.cleanup_dones()
    assert len(dones) == 1
    assert dones[0].lift_restored is False


def test_core_reason_lift_refused_without_height():
    rig = Rig()
    # 高さがまだ届いていない
    rig.post(rig.camera.state())
    rig.flush()
    assert rig.accept(goal()) is None
    rig.advance(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "lift_refused"
    assert not any(isinstance(c, LiftGoto) for c in rig.commands)
    dones = rig.cleanup_dones()
    assert len(dones) == 1
    assert dones[0].lift_restored is False


def test_core_reason_camera_too_low():
    rig = Rig(lift=FakeLift(height_m=0.0, speed_m_s=10.0, max_height_m=2.0))
    rig.prime()
    assert rig.accept(goal()) is None
    rig.advance(0.0)
    rig.advance(1.0)
    rig.finish_cleanup(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "camera_too_low"


def test_core_reason_canceled():
    rig = Rig()
    rig.prime()
    assert rig.accept(goal()) is None
    run_one_frame(rig)
    rig.post(Cancel())
    rig.flush()

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "canceled"


def test_core_reason_timeout():
    rig = Rig(params=make_params(time_limit_s=1.0))
    rig.prime()
    assert rig.accept(goal(time_limit_s=1.0)) is None
    run_one_frame(rig)
    rig.advance(1.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "timeout"


def test_core_reason_human_override():
    rig = Rig()
    rig.prime()
    assert rig.accept(goal()) is None
    run_one_frame(rig)
    rig.lift.take_over()
    rig.advance(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "human_override"


# ---------------------------------------------------------------- 途中で終わる
@pytest.mark.parametrize("stopper", ["canceled", "timeout", "human_override"])
def test_core_partial(stopper):
    if stopper == "timeout":
        rig = Rig(params=make_params(time_limit_s=1.0))
        req = goal(time_limit_s=1.0)
    else:
        rig = Rig()
        req = goal()
    rig.prime()
    assert rig.accept(req) is None
    run_one_frame(rig)

    if stopper == "canceled":
        rig.post(Cancel())
        rig.flush()
    elif stopper == "timeout":
        rig.advance(1.0)
    else:
        rig.lift.take_over()
        rig.advance(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == stopper
    assert len(results[0].panels) >= 1
    assert results[0].seen_from
    assert results[0].seen_span and results[0].seen_span[0] > 0.0


# ---------------------------------------------------------------- 後片付け
def test_core_cleanup_restores():
    origin_height = 1.5
    origin_yaw, origin_pitch = 0.3, 0.1
    rig = Rig(params=make_params(settle_s=0.1),
              lift=FakeLift(height_m=origin_height, speed_m_s=10.0))
    rig.camera.point(origin_yaw, origin_pitch)
    rig.prime()
    assert rig.accept(goal()) is None
    rig.start_scan()
    for _ in range(len(rig.core._yaws)):
        rig.advance(rig.params["settle_s"])
    rig.finish_cleanup(1.0)

    results = rig.results()
    dones = rig.cleanup_dones()
    assert len(results) == 1
    assert len(dones) == 1
    assert dones[0].lift_restored is True

    result_index = rig.commands.index(results[0])
    done_index = rig.commands.index(dones[0])
    assert done_index > result_index
    assert sum(isinstance(c, SendCleanupDone) for c in rig.commands) == 1

    after = rig.commands[result_index + 1:]
    assert any(isinstance(c, LiftGoto) and c.height_m == pytest.approx(origin_height)
               for c in after)
    assert any(isinstance(c, CameraPoint)
               and c.yaw_rad == pytest.approx(origin_yaw)
               and c.pitch_down_rad == pytest.approx(origin_pitch)
               for c in after)


def test_core_human_no_restore():
    rig = Rig()
    rig.camera.point(0.3, 0.1)
    rig.prime()
    assert rig.accept(goal()) is None
    run_one_frame(rig)
    rig.lift.take_over()
    rig.advance(0.0)

    result = rig.results()[0]
    assert result.reason == "human_override"
    after = rig.commands[rig.commands.index(result) + 1:]
    assert not any(isinstance(c, LiftGoto) for c in after)
    assert not any(isinstance(c, CameraPoint) for c in after)
    dones = rig.cleanup_dones()
    assert len(dones) == 1
    assert dones[0].lift_restored is False


def test_core_cleanup_timeout():
    origin_height = 0.0
    rig = Rig(params=make_params(cleanup_timeout_s=0.1, settle_s=0.1),
              lift=FakeLift(height_m=origin_height, speed_m_s=0.2))
    rig.prime()
    assert rig.accept(goal()) is None
    rig.start_scan()  # 3.1 m 上がる
    for _ in range(len(rig.core._yaws)):
        rig.advance(rig.params["settle_s"])
    assert rig.core.state == LocateCore.CLEANING
    rig.advance(0.2)

    dones = rig.cleanup_dones()
    assert len(dones) == 1
    assert dones[0].lift_restored is False
    assert rig.core.state == LocateCore.IDLE


# ---------------------------------------------------------------- 再起動
def test_core_restart_discards():
    rig = Rig(params=make_params(settle_s=0.5))
    rig.prime()
    assert rig.accept(goal()) is None
    rig.start_scan()
    first_yaw = rig.core._yaws[0]
    assert rig.core._scan_index == 0

    # 撮る前にカメラが再起動した
    rig.camera.restart()
    rig.post(rig.camera.state())
    commands = rig.flush()
    repointed = [c for c in commands if isinstance(c, CameraPoint)]
    assert len(repointed) == 1
    assert repointed[0].yaw_rad == pytest.approx(first_yaw)
    assert repointed[0].pitch_down_rad == pytest.approx(
        math.radians(rig.params["scan_pitch_down_deg"])
    )

    # 再起動の印のあいだに来た FrameDone は捨てる
    rig.post(FrameDone(first_yaw, [], True))
    commands = rig.flush()
    assert rig.core._scan_index == 0
    assert not any(isinstance(c, CameraPoint) for c in commands)

    # 向け直したあとは、そのヨーを撮り直して次へ進む
    rig.advance(0.5)
    assert rig.core._scan_index == 1
    next_point = [c for c in rig.commands if isinstance(c, CameraPoint)][-1]
    assert next_point.yaw_rad == pytest.approx(rig.core._yaws[1])


def test_core_end_to_end_fixture():
    rig = Rig()
    rig.prime()
    assert rig.accept(goal()) is None
    rig.start_scan()
    for _ in range(len(rig.core._yaws)):
        rig.advance(rig.params["settle_s"])
    rig.finish_cleanup(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "completed"
    assert len(results[0].panels) >= 1
    assert results[0].seen_from and results[0].seen_span
    assert rig.core.state == LocateCore.IDLE
    dones = rig.cleanup_dones()
    assert len(dones) == 1
    assert rig.commands.index(dones[0]) > rig.commands.index(results[0])

    phases = [c.phase for c in rig.commands if isinstance(c, SendFeedback)]
    assert phases[0] == "RAISING"
    assert "SCANNING" in phases
    assert "ESTIMATING" in phases

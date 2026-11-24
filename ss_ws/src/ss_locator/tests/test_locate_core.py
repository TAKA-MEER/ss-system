import json
import math
from pathlib import Path

import numpy as np
import pytest

from ss_locator import assemble, scan_plan
from ss_locator.fakes import FakeCamera, FakeDetector, FakeLift
from ss_locator.locate_core import (
    CameraPoint,
    CameraRelease,
    CameraState,
    Cancel,
    FrameDone,
    GoalRequest,
    LiftGoto,
    LiftRelease,
    LiftState,
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
        "two_lug_spacing_min_m": 0.5,
        "two_lug_spacing_max_m": 0.8,
        "two_lug_depth_default_m": 0.86,
        "two_lug_depth_std_m": 0.046,
        "panel_depth_min_m": 0.70,
        "panel_depth_max_m": 1.02,
        "back_to_back_gap_min_m": 0.05,
        "back_to_back_gap_max_m": 0.30,
        "merge_tol_m": 0.10,
        "back_to_back_angle_tol_deg": 10.0,
        "back_to_back_overlap_ratio": 0.5,
        "scan_camera_height_m": 3.10,
        "min_camera_height_m": 2.95,
        "scan_pitch_down_deg": 20.0,
        "scan_overlap_ratio": 0.1,
        "settle_s": 0.5,
        "image_timeout_s": 5.0,
        "aim_timeout_s": 60.0,
        "cleanup_timeout_s": 30.0,
        "cleanup_margin_s": 10.0,
        "cleanup_limit_max_s": 200.0,
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
            # 偽のカメラは向けた次の state() で着く。ノードと同じく、すぐ受け取る
            self.post(self.camera.state())

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
    # 1 枚失敗しても同じ向きで 2 回まで撮り直す（34cc2c8）ので、時間を進めて尽くす
    rig.finish_cleanup(1.0)

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


def test_core_height_none_refused():
    rig = Rig()
    # 高さが「読めない」（None）で届いている
    rig.post(LiftState(None))
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


class UnknownCeilingLift(FakeLift):
    """上昇が天井の値なしで止まる（`ceiling_unknown`）。"""

    def advance(self, dt_s):
        state = super().advance(dt_s)
        if self._arrived and self._at_ceiling:
            return LiftState(state.height_m, False, "ceiling_unknown")
        return state


def test_core_ceiling_unknown():
    rig = Rig(lift=UnknownCeilingLift(height_m=0.0, speed_m_s=10.0, max_height_m=2.0))
    rig.prime()
    assert rig.accept(goal()) is None
    rig.advance(0.0)
    rig.advance(1.0)
    rig.finish_cleanup(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "ceiling_unknown"
    # 昇降機は元の高さへ戻す
    after = rig.commands[rig.commands.index(results[0]) + 1:]
    assert any(isinstance(c, LiftGoto) and c.height_m == pytest.approx(0.0) for c in after)

    # 天井の値があって届かないときは今までどおり camera_too_low
    rig = Rig(lift=FakeLift(height_m=0.0, speed_m_s=10.0, max_height_m=2.0))
    rig.prime()
    assert rig.accept(goal()) is None
    rig.advance(0.0)
    rig.advance(1.0)
    assert rig.results()[0].reason == "camera_too_low"


def test_core_ceiling_unknown_enough_height_scans():
    # 光学中心が min_camera_height_m に届いていれば、ceiling_unknown でも走査へ進む
    rig = Rig(lift=UnknownCeilingLift(height_m=0.0, speed_m_s=10.0, max_height_m=3.0))
    rig.prime()
    assert rig.accept(goal()) is None
    rig.advance(0.0)
    rig.advance(1.0)
    assert rig.core.stage == LocateCore.SCANNING


def test_core_pose_unknown_at_accept():
    rig = Rig()
    rig.post(rig.lift.state())
    rig.post(CameraState(0.0, 0.0, False, False))
    rig.flush()
    assert rig.accept(goal()) is None
    rig.advance(0.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "camera_pose_unknown"
    assert results[0].cleanup_limit_s == 0.0
    assert not any(isinstance(c, (LiftGoto, CameraPoint)) for c in rig.commands)
    assert rig.cleanup_dones()[0].lift_restored is False
    assert rig.core.state == LocateCore.IDLE


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
class NoSpeedLift(FakeLift):
    """速さを知らせない昇降部（`LiftState` の速さが `None`）。"""

    def __init__(self, *args, report_speed=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.report_speed = report_speed

    def state(self):
        base = super().state()
        return LiftState(base.height_m, base.arrived, base.stopped_reason,
                         self.report_speed, self.report_speed)


class SlowDownLift(FakeLift):
    """動きは速いが、下降の速さだけ別の値を知らせる。"""

    def __init__(self, *args, down_speed=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.down_speed = down_speed

    def state(self):
        base = super().state()
        return LiftState(base.height_m, base.arrived, base.stopped_reason,
                         base.up_speed_m_s, self.down_speed)


def _scan_to_cleaning(rig):
    assert rig.accept(goal()) is None
    rig.start_scan()
    for _ in range(len(rig.core._yaws)):
        rig.advance(rig.params["settle_s"])
    assert rig.core.state == LocateCore.CLEANING


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
    # 速さが分からない昇降部なので、上限は cleanup_timeout_s
    rig = Rig(params=make_params(cleanup_timeout_s=0.1, settle_s=0.1),
              lift=NoSpeedLift(height_m=origin_height, speed_m_s=0.2))
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
def test_core_restart_ends_request():
    origin_height, origin_pitch = 1.5, 0.1
    rig = Rig(params=make_params(settle_s=0.1),
              lift=FakeLift(height_m=origin_height, speed_m_s=10.0))
    rig.camera.point(0.3, origin_pitch)
    rig.prime()
    assert rig.accept(goal()) is None
    rig.start_scan()
    rig.advance(0.1)  # 1 枚撮る
    assert rig.core._scan_index >= 1
    n_commands = len(rig.commands)

    rig.camera.restart()
    rig.post(rig.camera.state())
    rig.flush()
    rig.finish_cleanup(1.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "camera_pose_unknown"
    assert results[0].seen_from and results[0].seen_span
    after_restart = rig.commands[n_commands:]
    # 撮り直さない
    assert not any(isinstance(c, RequestFrame) for c in after_restart)
    # ヨーを戻す命令を出さない（ピッチは戻す）
    points = [c for c in after_restart if isinstance(c, CameraPoint)]
    assert points and all(c.yaw_rad is None for c in points)
    assert any(c.pitch_down_rad == pytest.approx(origin_pitch) for c in points)
    # 昇降機は戻す
    assert any(isinstance(c, LiftGoto) and c.height_m == pytest.approx(origin_height)
               for c in after_restart)
    assert rig.cleanup_dones()[0].lift_restored is True
    assert rig.core.state == LocateCore.IDLE


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


# ---------------------------------------------------------------- 後片付けの上限（版 2）
def test_core_cleanup_limit_from_speed():
    # 元の高さ 1.9 m・走査の高さ 3.1 m。差 1.2 m ÷ 0.03 m/s = 40 s、余裕 10 s で 50 s
    rig = Rig(params=make_params(settle_s=0.1),
              lift=SlowDownLift(height_m=1.9, speed_m_s=10.0, down_speed=0.03))
    rig.prime()
    _scan_to_cleaning(rig)
    assert rig.results()[0].cleanup_limit_s == pytest.approx(50.0)

    # 50 s の手前では待つ
    rig.advance(0.0)
    rig.lift._target = None  # 着かせない
    rig.advance(48.0)
    assert rig.core.state == LocateCore.CLEANING
    assert not rig.cleanup_dones()
    # 過ぎたら手を離して lift_restored: false
    rig.advance(3.0)
    dones = rig.cleanup_dones()
    assert len(dones) == 1
    assert dones[0].lift_restored is False
    assert rig.core.state == LocateCore.IDLE


@pytest.mark.parametrize("speed", [None, 0.0])
def test_core_cleanup_limit_fallback(speed):
    rig = Rig(params=make_params(settle_s=0.1, cleanup_timeout_s=33.0),
              lift=NoSpeedLift(height_m=1.9, speed_m_s=10.0, report_speed=speed))
    rig.prime()
    _scan_to_cleaning(rig)
    assert rig.results()[0].cleanup_limit_s == pytest.approx(33.0)


def test_core_cleanup_limit_capped():
    # 差 1.8 m ÷ 0.005 m/s = 360 s ＋ 10 s = 370 s は max の 200 s に切り詰める
    rig = Rig(params=make_params(settle_s=0.1),
              lift=SlowDownLift(height_m=1.3, speed_m_s=10.0, down_speed=0.005))
    rig.prime()
    _scan_to_cleaning(rig)
    assert rig.results()[0].cleanup_limit_s == pytest.approx(200.0)

    rig.advance(0.0)
    rig.lift._target = None
    rig.advance(198.0)
    assert rig.core.state == LocateCore.CLEANING
    rig.advance(3.0)
    dones = rig.cleanup_dones()
    assert len(dones) == 1
    assert dones[0].lift_restored is False


def test_core_cleanup_limit_zero_when_no_restore():
    rig = Rig()
    rig.prime()
    assert rig.accept(goal()) is None
    run_one_frame(rig)
    rig.lift.take_over()
    rig.advance(0.0)
    result = rig.results()[0]
    assert result.reason == "human_override"
    assert result.cleanup_limit_s == 0.0


# ---------------------------------------------------------------- カメラの着いた・取って代わられた（WP-HVE-02）
class Drive:
    """出来事を 1 つずつ手で渡して `LocateCore` の命令を見る（偽のカメラを使わない）。"""

    def __init__(self, origin_height=1.9, origin_yaw=0.0, lift_speed=10.0,
                 scanning=True, **params):
        self.params = make_params(**params)
        self.core = LocateCore(self.params)
        self.commands = []
        self.step(LiftState(origin_height, False, "none", lift_speed, lift_speed), 0.0)
        self.step(CameraState(origin_yaw, 0.1), 0.0)
        assert self.core.accept(goal(), 0.0) is None
        self.step(Tick(), 0.0)  # 昇降機へ
        if not scanning:
            assert self.core.stage == LocateCore.RAISING
            return
        target = self.params["scan_camera_height_m"]
        self.step(LiftState(target, True, "none", lift_speed, lift_speed), 1.0)
        assert self.core.stage == LocateCore.SCANNING
        assert isinstance(self.commands[-1], CameraPoint)

    def step(self, event, now):
        commands = self.core.step(event, now)
        self.commands.extend(commands)
        return commands

    def camera(self, now, **kwargs):
        values = {"yaw_rad": 0.0, "pitch_down_rad": 0.1}
        values.update(kwargs)
        return self.step(CameraState(**values), now)

    def result(self):
        results = [c for c in self.commands if isinstance(c, SendResult)]
        assert len(results) == 1
        return results[0]

    def cleanup_dones(self):
        return [c for c in self.commands if isinstance(c, SendCleanupDone)]


def has(commands, kind):
    return any(isinstance(c, kind) for c in commands)


def test_core_settle_after_arrived():
    drive = Drive(settle_s=0.5)
    # 向けた命令の時刻から settle_s たっても、着くまで撮らない
    assert not has(drive.step(Tick(), 1.6), RequestFrame)
    assert not has(drive.camera(10.0, arrived=False), RequestFrame)
    assert not has(drive.step(Tick(), 20.0), RequestFrame)
    # 着いた
    assert not has(drive.camera(30.0, arrived=True), RequestFrame)
    # 着いてから settle_s の手前では撮らない
    assert not has(drive.step(Tick(), 30.4), RequestFrame)
    # 着いたあとの状態が何度届いても、数え直さない
    assert not has(drive.camera(30.45, arrived=True), RequestFrame)
    # settle_s たって初めて撮る
    assert has(drive.step(Tick(), 30.5), RequestFrame)


def test_core_settle_zero_requests_on_arrival():
    drive = Drive(settle_s=0.0)
    assert has(drive.camera(5.0, arrived=True), RequestFrame)


def test_core_aim_timeout():
    drive = Drive(aim_timeout_s=60.0)  # 向けたのは 1.0 s
    assert not has(drive.step(Tick(), 60.9), SendResult)
    assert drive.core.state == LocateCore.RUNNING
    drive.step(Tick(), 61.0)
    assert drive.result().reason == "no_image"
    assert not has(drive.commands, RequestFrame)  # 撮ってはいない

    # 着いたあとに待つ上限は image_timeout_s のまま（aim_timeout_s ではない）
    drive = Drive(aim_timeout_s=60.0, settle_s=0.5, image_timeout_s=5.0)
    drive.camera(2.0, arrived=True)
    assert has(drive.step(Tick(), 2.5), RequestFrame)
    assert not has(drive.step(Tick(), 7.4), SendResult)
    drive.step(Tick(), 7.5)
    assert drive.result().reason == "no_image"


def test_core_camera_taken_over():
    rig = Rig()
    rig.camera.point(0.3, 0.1)
    rig.prime()
    assert rig.accept(goal()) is None
    run_one_frame(rig)
    assert rig.core.stage == LocateCore.SCANNING
    n_commands = len(rig.commands)

    rig.camera.take_over()
    rig.post(rig.camera.state())
    rig.flush()

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "human_override"
    assert results[0].cleanup_limit_s == 0.0
    after = rig.commands[n_commands:]
    # 昇降機もカメラも戻さない（手を離すだけ）
    assert not any(isinstance(c, (LiftGoto, CameraPoint)) for c in after)
    assert has(after, LiftRelease)
    assert has(after, CameraRelease)
    dones = rig.cleanup_dones()
    assert len(dones) == 1 and dones[0].lift_restored is False
    assert rig.core.state == LocateCore.IDLE


def test_core_camera_taken_over_while_raising():
    drive = Drive(scanning=False)
    drive.camera(0.5, taken_over=True)
    assert drive.result().reason == "human_override"
    assert drive.core.state == LocateCore.IDLE


def test_core_camera_taken_over_while_cleaning():
    drive = Drive()
    drive.step(Cancel(), 2.0)
    assert drive.core.state == LocateCore.CLEANING
    assert drive.result().reason == "canceled"
    n_commands = len(drive.commands)

    drive.camera(3.0, taken_over=True)
    assert drive.core.state == LocateCore.IDLE
    after = drive.commands[n_commands:]
    assert not any(isinstance(c, (LiftGoto, CameraPoint)) for c in after)
    assert has(after, LiftRelease) and has(after, CameraRelease)
    dones = drive.cleanup_dones()
    assert len(dones) == 1
    assert dones[0].reason == "canceled"
    assert dones[0].lift_restored is False


def test_core_camera_failed():
    rig = Rig(lift=FakeLift(height_m=1.5, speed_m_s=10.0),
              camera=FakeCamera(FIXTURE, scene=0, fail_aim=True))
    rig.prime()
    assert rig.accept(goal()) is None
    rig.start_scan()
    rig.finish_cleanup(1.0)

    results = rig.results()
    assert len(results) == 1
    assert results[0].reason == "no_image"
    assert not has(rig.commands, RequestFrame)
    # 後片付けはする（昇降機を元の高さへ戻し、カメラも向け直す。カメラが向けられなくても待ち続けない）
    after = rig.commands[rig.commands.index(results[0]) + 1:]
    assert any(isinstance(c, LiftGoto) and c.height_m == pytest.approx(1.5)
               for c in after)
    assert has(after, CameraPoint)
    dones = rig.cleanup_dones()
    assert len(dones) == 1
    assert dones[0].lift_restored is True
    assert rig.core.state == LocateCore.IDLE


def test_core_cleanup_waits_camera():
    drive = Drive(origin_height=1.9)
    drive.step(Cancel(), 2.0)
    assert drive.core.state == LocateCore.CLEANING
    assert has(drive.commands[-3:], CameraPoint)

    # 昇降機が先に着いても、カメラが着くまで完了を出さない
    drive.step(LiftState(1.9, True, "none", 10.0, 10.0), 3.0)
    assert drive.cleanup_dones() == []
    drive.camera(3.5, arrived=False)
    drive.step(Tick(), 4.0)
    assert drive.cleanup_dones() == []
    assert drive.core.state == LocateCore.CLEANING

    # カメラが着いたら出す
    drive.camera(5.0, arrived=True)
    dones = drive.cleanup_dones()
    assert len(dones) == 1
    assert dones[0].lift_restored is True
    assert drive.core.state == LocateCore.IDLE

    # カメラが先に着いたときは、昇降機が着くまで出さない
    drive = Drive(origin_height=1.9)
    drive.step(Cancel(), 2.0)
    drive.camera(3.0, arrived=True)
    assert drive.cleanup_dones() == []
    drive.step(LiftState(1.9, True, "none", 10.0, 10.0), 4.0)
    assert len(drive.cleanup_dones()) == 1


def test_core_cleanup_limit_includes_yaw():
    # 昇降機の分 1.2 m ÷ 0.06 m/s = 20 s、ヨーを戻す分 180° ÷ 5°/s = 36 s、余裕 10 s
    speed = math.radians(5.0)
    drive = Drive(origin_height=1.9, origin_yaw=0.0, lift_speed=0.06,
                  cleanup_margin_s=10.0)
    drive.camera(2.0, yaw_rad=math.pi, yaw_speed_rad_s=speed)
    drive.step(Cancel(), 2.5)
    assert drive.result().cleanup_limit_s == pytest.approx(46.0)

    # 46 s の手前では待ち、過ぎたら手を離す
    drive.step(Tick(), 2.5 + 45.0)
    assert drive.core.state == LocateCore.CLEANING
    drive.step(Tick(), 2.5 + 46.0)
    assert drive.core.state == LocateCore.IDLE
    dones = drive.cleanup_dones()
    assert len(dones) == 1 and dones[0].lift_restored is False

    # ヨーの速さが分からなければ昇降機の分だけ（20 s + 10 s）
    drive = Drive(origin_height=1.9, origin_yaw=0.0, lift_speed=0.06)
    drive.camera(2.0, yaw_rad=math.pi, yaw_speed_rad_s=None)
    drive.step(Cancel(), 2.5)
    assert drive.result().cleanup_limit_s == pytest.approx(30.0)

    # ヨーを戻さない終わり方（向きが読めない）も昇降機の分だけ
    drive = Drive(origin_height=1.9, origin_yaw=0.0, lift_speed=0.06)
    drive.camera(2.0, yaw_rad=math.pi, yaw_speed_rad_s=speed,
                 restarted=True, pose_known=False)
    assert drive.result().reason == "camera_pose_unknown"
    assert drive.result().cleanup_limit_s == pytest.approx(30.0)

    # 昇降機の分のほうが長ければ、そちら（180° ÷ 20°/s = 9 s < 20 s）
    drive = Drive(origin_height=1.9, origin_yaw=0.0, lift_speed=0.06)
    drive.camera(2.0, yaw_rad=math.pi, yaw_speed_rad_s=math.radians(20.0))
    drive.step(Cancel(), 2.5)
    assert drive.result().cleanup_limit_s == pytest.approx(30.0)


def test_two_lug_in_core():
    # `make_panel_tops` の結果を返す盤にする（本番の経路の試験）
    drive = Drive(settle_s=0.0)
    drive.camera(2.0, arrived=True)
    assert has(drive.commands, RequestFrame)
    obs = [
        {"view": 0, "p": np.array([2.0, -0.265]), "axis": 41.0, "s_axis": 3.0,
         "s_pos": 0.003, "conf": 0.9, "dist": 3.0},
        {"view": 0, "p": np.array([2.0, 0.265]), "axis": 139.0, "s_axis": 3.0,
         "s_pos": 0.003, "conf": 0.9, "dist": 3.0},
    ]
    drive.step(FrameDone(0.0, obs, True), 2.5)
    drive.step(Cancel(), 3.0)
    result = drive.result()
    assert result.reason == "canceled"
    assert len(result.panels) == 1
    panel = result.panels[0]
    assert panel["lugs"] == 2
    assert panel["width_m"] == pytest.approx(0.53 + 2 * 0.0675)
    assert panel["depth_m"] == pytest.approx(0.86)
    assert panel["depth_measured"] is False
    assert panel["front_known"] is False

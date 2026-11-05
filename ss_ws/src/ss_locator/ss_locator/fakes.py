"""試験用の偽の hve（昇降部・カメラ）と検出器。純 Python。

材料は試作の出力 `tests/fixtures/proto_9265d2d.json`。偽の外部パラメータは材料の `T` を
そのまま使う（`base_link` = 試作のワールド）。本物のカメラでは `geometry.camera_T` で作る。
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from .locate_core import CameraState, LiftGoto, LiftRelease, LiftState

TAU = 2.0 * math.pi


def _load_fixture(fixture):
    path = Path(fixture)
    return json.loads(path.read_text(encoding="utf-8"))


def _view_yaw_rad(view):
    """カメラの z 軸（`T` の 3 行目）の base_link での方位。"""
    return math.atan2(float(view["T"][2][1]), float(view["T"][2][0]))


def _angle_diff(a, b):
    return (a - b + math.pi) % TAU - math.pi


class FakeLift:
    """目標の高さへ一定の速さで動く偽の昇降部。"""

    def __init__(self, height_m=0.0, speed_m_s=0.2, max_height_m=None, refuse=False):
        self.height_m = float(height_m)
        self.speed_m_s = float(speed_m_s)
        self.max_height_m = None if max_height_m is None else float(max_height_m)
        self.refuse = bool(refuse)
        self._target = None
        self._arrived = False
        self._stopped_reason = "none"
        self._at_ceiling = False

    def apply(self, command):
        """`LiftGoto`／`LiftRelease` を受ける。"""
        if isinstance(command, LiftGoto):
            if self.refuse:
                self._target = None
                self._arrived = False
                self._at_ceiling = False
                self._stopped_reason = "refused"
                return
            target = float(command.height_m)
            self._at_ceiling = False
            if self.max_height_m is not None and target > self.max_height_m:
                target = self.max_height_m
                self._at_ceiling = True
            self._target = target
            self._arrived = False
            self._stopped_reason = "none"
        elif isinstance(command, LiftRelease):
            self._target = None
            self._arrived = False
            self._at_ceiling = False
            self._stopped_reason = "none"

    def advance(self, dt_s):
        """時間を `dt_s` 進め、今の `LiftState` を返す。"""
        dt = max(0.0, float(dt_s))
        if self._target is not None and not self._arrived:
            diff = self._target - self.height_m
            step = self.speed_m_s * dt
            if abs(diff) <= step + 1e-12:
                self.height_m = self._target
                self._arrived = True
            else:
                self.height_m += math.copysign(step, diff)
        if self._arrived:
            self._stopped_reason = "ceiling" if self._at_ceiling else "none"
        return self.state()

    def state(self):
        return LiftState(float(self.height_m), bool(self._arrived), self._stopped_reason)

    def take_over(self):
        """人に取って代わられた。"""
        self._target = None
        self._arrived = False
        self._at_ceiling = False
        self._stopped_reason = "human"


class FakeCamera:
    """材料の 1 シーンを持つ偽のカメラ。向けたヨーに最も近い視点を返す。"""

    def __init__(self, fixture, scene=0, fail=False):
        data = _load_fixture(fixture)
        self.views = data["scenes"][int(scene)]["views"]
        self.fail = bool(fail)
        self.yaw_rad = 0.0
        self.pitch_down_rad = 0.0
        self.restarted = False

    def point(self, yaw_rad, pitch_down_rad):
        self.yaw_rad = float(yaw_rad)
        self.pitch_down_rad = float(pitch_down_rad)
        self.restarted = False

    def capture(self):
        """向けたヨーに最も近い視点の `K`・`dist`・`T`・視点の番号を返す。失敗なら `None`。"""
        if self.fail:
            return None
        view = min(self.views, key=lambda v: abs(_angle_diff(_view_yaw_rad(v), self.yaw_rad)))
        return {
            "view": int(view["view"]),
            "K": np.array(view["K"], float),
            "dist": np.array(view["dist"], float),
            "T": np.array(view["T"], float),
            "yaw_rad": self.yaw_rad,
        }

    def restart(self):
        """再起動の印を立て、ピッチの目標角を 0 にする。"""
        self.restarted = True
        self.pitch_down_rad = 0.0

    def state(self):
        return CameraState(self.yaw_rad, self.pitch_down_rad, self.restarted)


class FakeDetector:
    """視点の番号を受け、材料のその視点の検出を返す。"""

    def __init__(self, fixture, scene=0):
        data = _load_fixture(fixture)
        self.views = {
            int(view["view"]): view for view in data["scenes"][int(scene)]["views"]
        }

    def detect(self, view):
        v = self.views[int(view)]
        return np.array(v["kp"], float), np.array(v["conf"], float)

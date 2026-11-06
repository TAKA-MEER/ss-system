"""依頼の流れ（純関数）。時刻と出来事を受け、次にやる命令の並びを返すだけ。

ROS 2 も ultralytics も読み込まない。画像の検出と金具を床に戻す処理はここでは行わず、
つなぐ層（`locator_node.py`）が結果を `FrameDone` として渡す。

状態（`IDLE`／`RUNNING`／`CLEANING`）と、`RUNNING` の中の段
（`RAISING`／`SCANNING`／`ESTIMATING`）を持つ。詳細設計:
`docs/plan/detailed/DetailedDesign-core.md` §5、`DetailedDesign-locate.md` §2・§3。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from . import assemble, panels, scan_plan

TAU = 2.0 * math.pi


@dataclass(frozen=True)
class GoalRequest:
    """依頼 4 つの値。受け付けの判定は `LocateCore.accept` が行う。"""

    azimuth_from_rad: float
    azimuth_span_rad: float
    max_distance_m: float
    time_limit_s: float


@dataclass(frozen=True)
class Cancel:
    """取り消し。"""


@dataclass(frozen=True)
class LiftState:
    """昇降部の高さ（配信値）・目標に着いたか・止まった理由。"""

    height_m: float
    arrived: bool = False
    stopped_reason: str = "none"  # none / ceiling / refused / human


@dataclass(frozen=True)
class CameraState:
    """今のヨー・ピッチの目標角と、再起動した印。"""

    yaw_rad: float
    pitch_down_rad: float
    restarted: bool = False


@dataclass(frozen=True)
class FrameDone:
    """撮って検出し床に戻した結果。失敗なら `ok: false`。"""

    yaw_rad: float
    lugs: list
    ok: bool = True


@dataclass(frozen=True)
class Tick:
    """時刻だけ。"""


@dataclass(frozen=True)
class LiftGoto:
    height_m: float


@dataclass(frozen=True)
class LiftRelease:
    pass


@dataclass(frozen=True)
class CameraPoint:
    yaw_rad: float
    pitch_down_rad: float


@dataclass(frozen=True)
class CameraRelease:
    pass


@dataclass(frozen=True)
class RequestFrame:
    pass


@dataclass(frozen=True)
class SendFeedback:
    phase: str


@dataclass(frozen=True)
class SendResult:
    reason: str
    panels: list
    seen_from: list
    seen_span: list


@dataclass(frozen=True)
class SendCleanupDone:
    reason: str
    lift_restored: bool


class LocateCore:
    """依頼の受け付けと状態遷移。"""

    IDLE = "IDLE"
    RUNNING = "RUNNING"
    CLEANING = "CLEANING"

    RAISING = "RAISING"
    SCANNING = "SCANNING"
    ESTIMATING = "ESTIMATING"

    _SETTLING = "SETTLING"
    _WAITING_FRAME = "WAITING_FRAME"

    def __init__(self, params):
        self.params = dict(params)
        self.state = self.IDLE
        self.stage = None

        self._lift = None
        self._camera = None

        self._goal = None
        self._accepted_at = 0.0
        self._max_distance = 0.0
        self._origin_height = None
        self._origin_yaw = 0.0
        self._origin_pitch = 0.0

        self._started = False
        self._yaws = []
        self._scan_index = 0
        self._yaws_done = []
        self._obs = []

        self._phase = None
        self._settle_deadline = None
        self._frame_requested_at = None
        self._awaiting_frame = False
        self._restart_active = False

        self._cleanup_reason = None
        self._cleanup_start = None
        self._cleanup_done = False

    # ------------------------------------------------------------------ 設定
    def _get(self, name, default):
        return self.params.get(name, default)

    def _scan_goto_height(self):
        return float(self._get("scan_camera_height_m", 3.10)) - float(
            self._get("camera_height_offset_m", 0.0)
        )

    def _pitch_down_rad(self):
        return math.radians(float(self._get("scan_pitch_down_deg", 20.0)))

    def _hfov_rad(self):
        return float(self._get("hfov_rad", 0.0))

    # --------------------------------------------------------------- 受け付け
    def accept(self, goal, now):
        """依頼を受けたら `None`、拒否したら理由（`busy`・`bad_range`・`bad_distance`・`bad_time_limit`）。"""
        if self.state != self.IDLE:
            return "busy"

        span = float(goal.azimuth_span_rad)
        # 依頼の幅は float32。2π は float32 にすると TAU より約 1.7e-7 大きくなるので、
        # その分を許す（全周の依頼を bad_range で弾かない）
        if not math.isfinite(span) or span <= 0.0 or span > TAU + 1e-6:
            return "bad_range"

        distance = float(goal.max_distance_m)
        if not math.isfinite(distance) or distance < 0.0:
            return "bad_distance"

        time_limit = float(goal.time_limit_s)
        if not math.isfinite(time_limit) or time_limit <= 0.0:
            return "bad_time_limit"

        default_distance = float(self._get("max_distance_default_m", 5.0))
        if distance == 0.0:
            distance = default_distance
        elif distance > default_distance:
            distance = default_distance

        self._goal = goal
        self._accepted_at = float(now)
        self._max_distance = distance
        self._origin_height = None if self._lift is None else float(self._lift.height_m)
        self._origin_yaw = 0.0 if self._camera is None else float(self._camera.yaw_rad)
        self._origin_pitch = 0.0 if self._camera is None else float(self._camera.pitch_down_rad)

        self._started = False
        self._yaws = []
        self._scan_index = 0
        self._yaws_done = []
        self._obs = []
        self._phase = None
        self._settle_deadline = None
        self._frame_requested_at = None
        self._awaiting_frame = False
        self._restart_active = False
        self._cleanup_reason = None
        self._cleanup_start = None
        self._cleanup_done = False

        self.state = self.RUNNING
        self.stage = self.RAISING
        return None

    # ------------------------------------------------------------------ 状態遷移
    def step(self, event, now):
        """出来事を受けて、次にやる命令のリストを返す。"""
        now = float(now)
        if isinstance(event, LiftState):
            self._lift = event
        elif isinstance(event, CameraState):
            self._camera = event

        if self.state == self.IDLE:
            return []
        if self.state == self.CLEANING:
            return self._step_cleaning(event, now)
        return self._step_running(event, now)

    def _step_running(self, event, now):
        cmds = []

        if not self._started:
            self._started = True
            cmds.append(SendFeedback(self.RAISING))
            if self._origin_height is None:
                # 受け付けた時点の高さがまだ届いていない。動かさずに終わる
                return cmds + self._finish("lift_refused", now)
            cmds.append(LiftGoto(self._scan_goto_height()))
            return cmds

        if isinstance(event, Cancel):
            return self._finish("canceled", now)
        if isinstance(event, LiftState) and event.stopped_reason == "human":
            return self._finish("human_override", now)

        time_limit = float(self._goal.time_limit_s)
        if now - self._accepted_at >= time_limit - 1e-12:
            return self._finish("timeout", now)

        if self.stage == self.RAISING:
            if isinstance(event, LiftState):
                reason = event.stopped_reason
                if reason == "refused":
                    return self._finish("lift_refused", now)
                if reason == "ceiling":
                    optical = float(event.height_m) + float(
                        self._get("camera_height_offset_m", 0.0)
                    )
                    if optical < float(self._get("min_camera_height_m", 2.95)):
                        return self._finish("camera_too_low", now)
                    return self._begin_scanning(now)
                if event.arrived:
                    return self._begin_scanning(now)
            return cmds

        if self.stage == self.SCANNING:
            if isinstance(event, CameraState) and event.restarted:
                return self._on_restart(now)
            if isinstance(event, FrameDone):
                return self._on_frame(event, now)
            return self._poll_scan(now)

        return cmds

    def _begin_scanning(self, now):
        self.stage = self.SCANNING
        cmds = [SendFeedback(self.SCANNING)]
        self._yaws = scan_plan.plan_yaws(
            self._goal.azimuth_from_rad,
            self._goal.azimuth_span_rad,
            self._hfov_rad(),
            float(self._get("scan_overlap_ratio", 0.1)),
        )
        self._scan_index = 0
        if not self._yaws:
            return cmds + self._finish(None, now)
        return cmds + self._point_current(now)

    def _point_current(self, now):
        yaw = self._yaws[self._scan_index]
        self._phase = self._SETTLING
        self._settle_deadline = now + float(self._get("settle_s", 0.5))
        self._frame_requested_at = None
        self._awaiting_frame = False
        return [CameraPoint(yaw, self._pitch_down_rad())]

    def _poll_scan(self, now):
        if self._phase == self._SETTLING:
            if now >= self._settle_deadline - 1e-12:
                self._phase = self._WAITING_FRAME
                self._frame_requested_at = now
                self._awaiting_frame = True
                self._restart_active = False
                return [RequestFrame()]
            return []
        if self._phase == self._WAITING_FRAME:
            timeout = float(self._get("image_timeout_s", 5.0))
            if now - self._frame_requested_at >= timeout - 1e-12:
                return self._finish("no_image", now)
        return []

    def _on_restart(self, now):
        self._restart_active = True
        if self.stage != self.SCANNING or self._scan_index >= len(self._yaws):
            return []
        return self._point_current(now)

    def _on_frame(self, event, now):
        if self._restart_active:
            # 再起動の印のあいだの画像は捨て、そのヨーを撮り直す
            return []
        if not event.ok:
            return self._finish("no_image", now)
        if not self._awaiting_frame:
            return []

        self._obs.extend(event.lugs)
        self._yaws_done.append(float(event.yaw_rad))
        self._awaiting_frame = False
        self._phase = None
        self._scan_index += 1

        if self._scan_index >= len(self._yaws):
            self.stage = self.ESTIMATING
            return [SendFeedback(self.ESTIMATING)] + self._finish(None, now)
        return self._point_current(now)

    # ------------------------------------------------------------------ 終わり
    def _finish(self, reason, now):
        raw_panels, lugs = assemble.assemble(self._obs)
        top_height = float(self._get("top_height_default_m", 2.35))
        tops = [panels.to_panel_top(p, lugs, top_height) for p in raw_panels]
        found = panels.filter_panels(
            tops,
            self._goal.azimuth_from_rad,
            self._goal.azimuth_span_rad,
            self._max_distance,
        )
        seen_from, seen_span = scan_plan.seen_ranges(
            self._yaws_done,
            self._hfov_rad(),
            self._goal.azimuth_from_rad,
            self._goal.azimuth_span_rad,
        )

        if reason is None:
            n_lugs = panels.count_lugs_in_range(
                lugs,
                self._goal.azimuth_from_rad,
                self._goal.azimuth_span_rad,
                self._max_distance,
            )
            reason = panels.finish_reason(n_lugs, len(found))

        cmds = [SendResult(reason, found, seen_from, seen_span)]
        self.stage = None
        self._phase = None
        self._awaiting_frame = False
        self._restart_active = False
        self._cleanup_reason = reason
        self._cleanup_start = now
        self._cleanup_done = False

        if reason == "human_override":
            # 人の操作。戻さず、すぐ手を離す
            self.state = self.IDLE
            self._cleanup_done = True
            return cmds + [LiftRelease(), CameraRelease(),
                           SendCleanupDone(reason, False)]

        if self._origin_height is None:
            # 元の高さが分からない（受け付け時の高さが未着）。戻せない
            self.state = self.IDLE
            self._cleanup_done = True
            return cmds + [LiftRelease(), CameraRelease(),
                           SendCleanupDone(reason, False)]

        self.state = self.CLEANING
        return cmds + [
            LiftGoto(self._origin_height),
            CameraPoint(self._origin_yaw, self._origin_pitch),
        ]

    def _step_cleaning(self, event, now):
        if self._cleanup_done:
            return []

        if isinstance(event, LiftState):
            if event.stopped_reason in ("refused", "human"):
                return self._cleanup_end(False)
            if event.arrived:
                return self._cleanup_end(True)

        timeout = float(self._get("cleanup_timeout_s", 30.0))
        if now - self._cleanup_start >= timeout - 1e-12:
            return self._cleanup_end(False)
        return []

    def _cleanup_end(self, restored):
        if self._cleanup_done:
            return []
        self._cleanup_done = True
        reason = self._cleanup_reason
        self.state = self.IDLE
        self.stage = None
        self._cleanup_reason = None
        return [LiftRelease(), CameraRelease(),
                SendCleanupDone(reason, bool(restored))]

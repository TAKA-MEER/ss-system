"""カメラモジュールとつなぐ部分の純関数（時刻は引数で受ける）。

受け取った `state`・`locate_core` の命令・時刻から、送る文と今の `CameraState` を返す。
静止画の応答の扱い（`frame_verdict`）も決める。
WebSocket・HTTP・ROS 2・スレッド・`time` は読み込まない。詳細設計:
`docs/plan/detailed/DetailedDesign-hve.md` §7.1・§7.2、
`docs/plan/detailed/DetailedDesign-names.md` §10。
"""
from __future__ import annotations

import math

from .hve_lift_core import LIFT_BUSY_LABEL, LIFT_NAME
from .locate_core import CameraPoint, CameraRelease, CameraState


def camera_hello_message(fw=None):
    """`hello` の dict を返す（`ceiling_sensor` は持たない）。`fw` を省くとパッケージの版。"""
    if fw is None:
        try:
            from importlib.metadata import version

            fw = version("ss_locator")
        except Exception:
            fw = ""
    return {
        "t": "hello",
        "name": LIFT_NAME,
        "role": "control",
        "operator": "auto",
        "busy_label": LIFT_BUSY_LABEL,
        "fw": str(fw),
    }


def _is_number(value):
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _wrap_yaw_deg(deg):
    """-180 以上 180 未満に直す。"""
    return (float(deg) + 180.0) % 360.0 - 180.0


class CameraLink:
    """送る文と今の `CameraState` を決める。WebSocket は持たない。"""

    def __init__(self, cmd_period_ms=100, state_timeout_ms=600, fw=None, link_grace_ms=0):
        self._cmd_period_s = max(0.0, float(cmd_period_ms) / 1000.0)
        self._state_timeout_s = max(0.0, float(state_timeout_ms) / 1000.0)
        self._fw = fw
        # 通信が途切れても、この時間までは命令をやめずに繋ぎ直して出し直す（0 なら今までどおり）
        self._grace_s = max(0.0, float(link_grace_ms) / 1000.0)
        self._good_at = None  # 最後に `state` を受けた時刻（命令の始めも含む）
        self._retries = 0
        self._repress_at = None

        self._connected = False
        self._last_state = None
        self._last_state_at = None
        self._queue = []

        self._press = 0
        self._command = None
        self._boot_ref = None  # 命令を始めたときの boot_id
        self._last_aim_at = None
        self._arrived = False
        self._taken_over = False
        self._failed = False
        self._pose_lost = False

    # ------------------------------------------------------------------ 出来事
    def on_connected(self, now):
        """繋がった。次に送る文の先頭に `hello` を積む。"""
        self._connected = True
        self._last_state = None
        self._last_state_at = float(now)
        self._queue.insert(0, camera_hello_message(self._fw))
        if self._grace_s > 0.0 and self._command is not None and not self._stopped():
            # 繋ぎ直した。前の `press` の `aim` は持ち主が外れて終わっているので、新しい番号で出し直す
            self._repress()

    def on_closed(self, now):
        """切れた。命令の途中なら、その命令をやめる（`failed`）。"""
        self._connected = False
        self._last_state = None
        self._queue.clear()
        if self._command is not None and not self._stopped() and not self._in_grace(now):
            self._failed = True

    def on_state(self, msg, now):
        """受けた `state` と時刻を覚える。`t` が `state` でないものは捨てる。"""
        if not isinstance(msg, dict) or msg.get("t") != "state":
            return
        self._last_state = msg
        self._last_state_at = float(now)
        self._good_at = float(now)
        if self._command is not None and self._boot_ref is None:
            self._boot_ref = msg.get("boot_id")

    def on_command(self, command, now):
        """`locate_core` の命令を受ける。"""
        if isinstance(command, CameraPoint):
            self._press += 1
            yaw_deg = None
            if command.yaw_rad is not None:
                yaw_deg = _wrap_yaw_deg(math.degrees(float(command.yaw_rad)))
            # 符号の変換はここだけ（hve は見上げが正、本システムは見下ろしが正）
            pitch_deg = -math.degrees(float(command.pitch_down_rad)) + 0.0
            self._command = {
                "press": self._press,
                "pitch_deg": pitch_deg,
                "yaw_deg": yaw_deg,
            }
            self._boot_ref = (
                None if self._last_state is None else self._last_state.get("boot_id")
            )
            self._reset_flags()
            if self._good_at is None or float(now) - self._good_at > self._grace_s:
                self._good_at = float(now)
            self._retries = 0
            self._repress_at = None
        elif isinstance(command, CameraRelease):
            if self._command is None:
                return
            self._queue.append({"t": "release", "press": self._command["press"]})
            self._command = None
            self._boot_ref = None
            self._reset_flags()

    # ------------------------------------------------------------------ 問い合わせ
    def poll(self, now):
        """送る文の並びと、繋ぎ直すかを返す。"""
        now = float(now)
        self._evaluate(now)

        messages = list(self._queue)
        self._queue.clear()
        if not self._connected:
            return messages, False
        if self._is_stale(now):
            return messages, True

        if (
            self._command is not None
            and not self._stopped()
            and not self._arrived
            and (
                self._last_aim_at is None
                or now - self._last_aim_at >= self._cmd_period_s - 1e-12
            )
        ):
            aim = {
                "t": "aim",
                "press": self._command["press"],
                "pitch_deg": self._command["pitch_deg"],
            }
            if self._command["yaw_deg"] is not None:
                aim["yaw_deg"] = self._command["yaw_deg"]
            messages.append(aim)
            self._last_aim_at = now
        return messages, False

    def camera_state(self, now):
        """今の `CameraState` を返す。"""
        now = float(now)
        self._evaluate(now)
        state = self._last_state
        yaw_rad = None
        pitch_down_rad = 0.0
        yaw_speed = None
        if state is not None:
            if _is_number(state.get("yaw_deg")):
                yaw_rad = math.radians(float(state["yaw_deg"]))
            if _is_number(state.get("pitch_deg")):
                pitch_down_rad = -math.radians(float(state["pitch_deg"])) + 0.0
            dps = state.get("yaw_dps")
            if _is_number(dps) and float(dps) > 0.0:
                yaw_speed = math.radians(float(dps))
        return CameraState(
            yaw_rad,
            pitch_down_rad,
            self._restarted(),
            self._pose_known(now),
            self._arrived,
            self._taken_over,
            self._failed,
            yaw_speed,
        )

    # ------------------------------------------------------------------ 中身
    def _reset_flags(self):
        self._last_aim_at = None
        self._arrived = False
        self._taken_over = False
        self._failed = False
        self._pose_lost = False

    def _in_grace(self, now):
        return (
            self._grace_s > 0.0
            and self._good_at is not None
            and float(now) - self._good_at <= self._grace_s
        )

    def _repress(self):
        self._press += 1
        self._command["press"] = self._press
        self._last_aim_at = None
        self._arrived = False

    def _stopped(self):
        return self._taken_over or self._failed or self._pose_lost

    def _is_stale(self, now):
        return (
            self._last_state_at is None
            or now - self._last_state_at > self._state_timeout_s
        )

    def _restarted(self):
        if self._command is None or self._boot_ref is None or self._last_state is None:
            return False
        return self._last_state.get("boot_id") != self._boot_ref

    def _pose_known(self, now):
        state = self._last_state
        if not self._connected or self._is_stale(now) or state is None:
            return False
        if self._pose_lost or self._restarted():
            return False
        return state.get("yaw_zeroed") is True and _is_number(state.get("yaw_deg"))

    def _evaluate(self, now):
        if self._command is None or self._taken_over:
            return
        # 3（後半）: 繋がっていない・`state` が古い
        if not self._connected or self._is_stale(now):
            if not self._pose_lost and not self._in_grace(now):
                self._failed = True
            return

        state = self._last_state
        if state is None:  # 繋がってからまだ `state` を受けていない（時間切れまでは待つ）
            return

        aim = state.get("aim")
        own = (
            isinstance(aim, dict)
            and aim.get("by") == LIFT_NAME
            and aim.get("press") == self._command["press"]
        )
        status = aim.get("status") if own else None
        reason = aim.get("reason") if own else None

        # 1: 人・取って代わった相手・ほかの持ち主
        if (
            state.get("owner_operator") == "human"
            or (own and status == "NOT_ARRIVED" and reason == "PREEMPTED")
            or (state.get("owner") is not None and state.get("owner_name") != LIFT_NAME)
        ):
            self._taken_over = True
            return

        # 2: 向きが読めない。ヨーを向けない命令（ピッチだけ戻す後片付け）は、
        # ヨーの原点が無くても動かせるので、ヨーに関わるものは見ない
        if self._restarted():
            self._pose_lost = True
            return
        if self._command["yaw_deg"] is not None:
            if state.get("yaw_zeroed") is not True or (
                own and status == "NOT_ARRIVED" and reason == "YAW_UNKNOWN"
            ):
                self._pose_lost = True
                return

        if not own or self._arrived:
            return
        if status == "ARRIVED":
            self._arrived = True
        elif status == "NOT_ARRIVED":
            if self._grace_s > 0.0 and self._retries < 10 and self._in_grace(now):
                # 途切れで持ち主が外れた・指令が途絶えたなど。少し待って新しい番号で出し直す
                if self._repress_at is None or now - self._repress_at >= 0.5:
                    self._retries += 1
                    self._repress()
                    self._repress_at = now
                return
            self._failed = True  # 3


def _header(headers, name):
    """大文字小文字を区別せずにヘッダの値を引く。無ければ `None`。"""
    target = name.lower()
    for key, value in (headers or {}).items():
        if str(key).lower() == target:
            return value
    return None


def _header_float(headers, name):
    value = _header(headers, name)
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def frame_verdict(status, headers, boot_id, since_arrived_ms):
    """静止画の応答の扱いを返す: `use`／`retry`／`pose_unknown`／`fail`。

    JPEG が読めるか・大きさの確認はここではしない（応答の番号とヘッダだけで決める）。
    `since_arrived_ms`（向け終えてからの経過）は今は使わない（`X-Still-Ms` が同じ役目をする）。
    """
    if status in (429, 503):
        return "retry"
    if status != 200:
        return "fail"

    if _header(headers, "X-Boot-Id") != boot_id:
        return "pose_unknown"
    if _header_float(headers, "X-Yaw-Deg") is None:
        return "pose_unknown"

    still_ms = _header_float(headers, "X-Still-Ms")
    age_ms = _header_float(headers, "X-Frame-Age-Ms")
    pitch = _header_float(headers, "X-Pitch-Deg")
    if still_ms is None or age_ms is None or pitch is None:
        return "fail"  # 使うのに要るヘッダが無い
    # 向きが最後に変わってからの経過が、撮ってからの経過より短い = 動いている間か動く前のコマ
    if still_ms < age_ms:
        return "retry"
    return "use"

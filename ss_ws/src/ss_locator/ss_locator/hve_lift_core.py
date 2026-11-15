"""昇降部とつなぐ部分の純関数（時刻は引数で受ける）。

受け取った `state`・`locate_core` の命令・時刻から、送る文と今の `LiftState` を返す。
WebSocket・ROS 2・スレッド・`time` は読み込まない。詳細設計:
`docs/plan/detailed/DetailedDesign-hve.md` §2〜§4、
`docs/plan/detailed/DetailedDesign-names.md` §7。
"""
from __future__ import annotations

from .locate_core import LiftGoto, LiftRelease, LiftState

LIFT_NAME = "ss"
LIFT_BUSY_LABEL = "画像認識で操作中"

_CEILING_REASONS = ("CEILING_NEAR", "CEILING_STALE", "TOP", "TARGET_RANGE")


def hello_message(fw=None):
    """`hello` の dict を返す。`fw` を省くとパッケージ `ss_locator` の版を使う。"""
    if fw is None:
        try:
            from importlib.metadata import PackageNotFoundError, version

            try:
                fw = version("ss_locator")
            except PackageNotFoundError:
                fw = ""
        except Exception:
            fw = ""
    return {
        "t": "hello",
        "name": LIFT_NAME,
        "role": "control",
        "operator": "auto",
        "ceiling_sensor": False,
        "busy_label": LIFT_BUSY_LABEL,
        "fw": str(fw),
    }


class LiftLink:
    """送る文と今の `LiftState` を決める。WebSocket は持たない。"""

    def __init__(self, speed_mm_s=40, cmd_period_ms=100, state_timeout_ms=600, fw=None):
        self._speed_mm_s = int(speed_mm_s)
        self._cmd_period_s = max(0.0, float(cmd_period_ms) / 1000.0)
        self._state_timeout_s = max(0.0, float(state_timeout_ms) / 1000.0)
        self._fw = fw

        self._connected = False
        self._last_state = None
        self._last_state_at = None
        self._queue = []

        self._press = 0
        self._command = None
        self._last_goto_at = None
        self._arrived = False
        self._stopped_reason = "none"

    # ------------------------------------------------------------------ 出来事
    def on_connected(self, now):
        """繋がった。次に送る文の先頭に `hello` を積む。"""
        self._connected = True
        self._last_state = None
        self._last_state_at = float(now)
        self._queue.insert(0, hello_message(self._fw))

    def on_closed(self, now):
        """切れた。命令の途中なら、その命令をやめる。"""
        self._connected = False
        self._last_state = None
        self._queue.clear()
        if self._command is not None and self._stopped_reason == "none":
            self._stopped_reason = "refused"

    def on_state(self, msg, now):
        """受けた `state` と時刻を覚える。`t` が `state` でないものは捨てる。"""
        if not isinstance(msg, dict) or msg.get("t") != "state":
            return
        self._last_state = msg
        self._last_state_at = float(now)

    def on_command(self, command, now):
        """`locate_core` の命令を受ける。"""
        now = float(now)
        if isinstance(command, LiftGoto):
            self._press += 1
            target_mm = int(round(float(command.height_m) * 1000.0))
            current_mm = self._known_height_mm()
            self._command = {
                "press": self._press,
                "target_mm": target_mm,
                "speed": self._speed_mm_s,
                "descending": current_mm is not None and target_mm < current_mm,
            }
            self._last_goto_at = None
            self._arrived = False
            self._stopped_reason = "none"
        elif isinstance(command, LiftRelease):
            if self._command is None:
                return
            self._queue.append({"t": "release", "press": self._command["press"]})
            self._command = None
            self._last_goto_at = None
            self._arrived = False
            self._stopped_reason = "none"

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
            and self._stopped_reason == "none"
            and not self._arrived
            and (
                self._last_goto_at is None
                or now - self._last_goto_at >= self._cmd_period_s - 1e-12
            )
        ):
            messages.append(
                {
                    "t": "goto",
                    "press": self._command["press"],
                    "target_mm": self._command["target_mm"],
                    "speed": self._command["speed"],
                }
            )
            self._last_goto_at = now
        return messages, False

    def lift_state(self, now):
        """今の `LiftState` を返す。"""
        now = float(now)
        self._evaluate(now)
        return LiftState(self._height_m(now), self._arrived, self._stopped_reason)

    # ------------------------------------------------------------------ 中身
    def _is_stale(self, now):
        return (
            self._last_state_at is None
            or now - self._last_state_at > self._state_timeout_s
        )

    def _known_height_mm(self):
        if not self._connected or self._last_state is None:
            return None
        if not bool(self._last_state.get("height_ok")):
            return None
        mm = self._last_state.get("height_mm")
        if isinstance(mm, bool) or not isinstance(mm, (int, float)):
            return None
        return int(round(float(mm)))

    def _height_m(self, now):
        if not self._connected or self._is_stale(now):
            return None
        mm = self._known_height_mm()
        return None if mm is None else mm / 1000.0

    def _evaluate(self, now):
        if self._command is None or self._stopped_reason != "none":
            return
        if not self._connected or self._is_stale(now):
            self._stopped_reason = "refused"
            return

        state = self._last_state
        if state.get("owner_operator") == "human":
            self._stopped_reason = "human"
            return

        goto = state.get("goto")
        own = (
            isinstance(goto, dict)
            and goto.get("by") == LIFT_NAME
            and goto.get("press") == self._command["press"]
            and goto.get("target_mm") == self._command["target_mm"]
        )
        if (
            own
            and goto.get("status") == "NOT_ARRIVED"
            and goto.get("reason") == "PREEMPTED"
        ):
            self._stopped_reason = "human"
            return

        if state.get("owner") is not None and state.get("owner_name") != LIFT_NAME:
            self._stopped_reason = "human"
            return

        if not own:
            return
        status = goto.get("status")
        reason = goto.get("reason")
        if status == "ARRIVED":
            self._arrived = True
            return
        if status != "NOT_ARRIVED":
            return

        if reason in _CEILING_REASONS:
            self._stopped_reason = "ceiling"
        elif reason == "BOTTOM" and self._command["descending"]:
            self._arrived = True
        else:
            self._stopped_reason = "refused"

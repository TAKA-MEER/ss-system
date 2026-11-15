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
LIFT_MAX_RUN_RETRY = 3  # 暫定。`MAX_RUN` で `press` を増やして送り直す回数の上限

# 上昇の `goto` がここで止まったら `ceiling`（-hve.md §4 の 4）
_CEILING_REASONS = ("CEILING_NEAR", "TOP", "TARGET_RANGE")


def hello_message(fw=None):
    """`hello` の dict を返す。`fw` を省くとパッケージ `ss_locator` の版を使う。"""
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
        "ceiling_sensor": False,
        "busy_label": LIFT_BUSY_LABEL,
        "fw": str(fw),
    }


class LiftLink:
    """送る文と今の `LiftState` を決める。WebSocket は持たない。"""

    def __init__(self, cmd_period_ms=100, state_timeout_ms=600, fw=None):
        self._cmd_period_s = max(0.0, float(cmd_period_ms) / 1000.0)
        self._state_timeout_s = max(0.0, float(state_timeout_ms) / 1000.0)
        self._fw = fw

        self._speeds = None  # (上昇, 下降) [mm/s]。取れていなければ None
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
    def set_speeds(self, up_mm_s, down_mm_s):
        """昇降部の設定から取った速さを渡す。`None` を渡すと「取れていない」に戻す。"""
        if up_mm_s is None or down_mm_s is None:
            self._speeds = None
        else:
            self._speeds = (int(up_mm_s), int(down_mm_s))

    def on_connected(self, now):
        """繋がった。次に送る文の先頭に `hello` を積む。"""
        self._connected = True
        self._last_state = None
        self._last_state_at = float(now)
        self._queue.insert(0, hello_message(self._fw))

    def on_closed(self, now):
        """切れた。命令の途中なら、その命令をやめる（`refused`）。"""
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
            current_mm = self._known_height_mm(now)
            self._last_goto_at = None
            self._arrived = False
            self._stopped_reason = "none"
            if current_mm is None or self._speeds is None or not self._connected:
                # 高さも速さも分からないまま動かさない
                self._command = {
                    "press": self._press,
                    "target_mm": target_mm,
                    "speed": 0,
                    "descending": False,
                    "retries": 0,
                }
                self._stopped_reason = "refused"
                return
            descending = target_mm < current_mm
            self._command = {
                "press": self._press,
                "target_mm": target_mm,
                "speed": self._speeds[1] if descending else self._speeds[0],
                "descending": descending,
                "retries": 0,
            }
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
        up = down = None
        if self._speeds is not None:
            up = self._speeds[0] / 1000.0
            down = self._speeds[1] / 1000.0
        return LiftState(
            self._height_m(now), self._arrived, self._stopped_reason, up, down
        )

    # ------------------------------------------------------------------ 中身
    def _is_stale(self, now):
        return (
            self._last_state_at is None
            or now - self._last_state_at > self._state_timeout_s
        )

    def _known_height_mm(self, now):
        if not self._connected or self._is_stale(now) or self._last_state is None:
            return None
        if self._last_state.get("height_ok") is not True:
            return None
        mm = self._last_state.get("height_mm")
        if isinstance(mm, bool) or not isinstance(mm, (int, float)):
            return None
        return int(round(float(mm)))

    def _height_m(self, now):
        mm = self._known_height_mm(now)
        return None if mm is None else mm / 1000.0

    def _evaluate(self, now):
        if self._command is None or self._stopped_reason != "none":
            return
        # 7: 繋がっていない・`state` が古い
        if not self._connected or self._is_stale(now):
            self._stopped_reason = "refused"
            return

        state = self._last_state
        if state is None:  # 繋がってからまだ `state` を受けていない（時間切れまでは待つ）
            return

        # 1: 人が動かしている
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
        # 2: 取って代わられた
        if (
            own
            and goto.get("status") == "NOT_ARRIVED"
            and goto.get("reason") == "PREEMPTED"
        ):
            self._stopped_reason = "human"
            return

        # 3: ほかの持ち主が動かしている
        if state.get("owner") is not None and state.get("owner_name") != LIFT_NAME:
            self._stopped_reason = "human"
            return

        if self._arrived or not own:
            return
        status = goto.get("status")
        reason = goto.get("reason")
        if status == "ARRIVED":
            self._arrived = True
            return
        if status != "NOT_ARRIVED":
            return

        descending = self._command["descending"]
        if not descending and reason in _CEILING_REASONS:
            self._stopped_reason = "ceiling"  # 4
        elif not descending and reason == "CEILING_STALE":
            self._stopped_reason = "ceiling_unknown"  # 4a
        elif descending and reason == "BOTTOM":
            self._arrived = True  # 5
        elif reason == "MAX_RUN":
            # 5a: 新しい `press` で同じ目標を送り直す
            if self._command["retries"] >= LIFT_MAX_RUN_RETRY:
                self._stopped_reason = "refused"
            else:
                self._command["retries"] += 1
                self._press += 1
                self._command["press"] = self._press
                self._last_goto_at = None
        else:
            self._stopped_reason = "refused"  # 6

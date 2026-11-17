"""依頼を手で試すコマンド `locate_cli`（ROS 2 のクライアントと流れ）。

口（アクション `/ss/locate/panels`・トピック `/ss/locate/cleanup_done`・
`/ss/locate/status`）だけを使う。`locator_node`・`locate_core` は読み込まない。
本番外の道具。昇降機・カメラ・機体は自分では動かさない。

終了コード: 0 結果と後片付けの完了を受けた（または依頼を出さずに終わった） /
2 引数の誤り / 3 応えない・版が合わない / 4 依頼できない・拒否 /
5 後片付けの完了が届かない / 130 待たずに終わった。

詳細設計: `docs/plan/detailed/DetailedDesign-tester.md`。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import signal
import sys
import threading
import time

import cv2
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.utilities import remove_ros_args
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ss_msgs.action import LocatePanels
from ss_msgs.msg import CleanupDone, LocatorStatus

from . import locate_view

ACTION_NAME = "/ss/locate/panels"
CLEANUP_TOPIC = "/ss/locate/cleanup_done"
STATUS_TOPIC = "/ss/locate/status"

CONFIRM_PROMPT = "昇降機とカメラが動きます。よろしいですか？ [y/N] "
KEEP_STILL = "後片付けの完了まで、機体と昇降機を動かさないでください。"

CLEANUP_QOS = QoSProfile(
    depth=10,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
STATUS_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

POLL_S = 0.02


class InterruptFlag:
    """取り消しの印。Ctrl+C のたびに `count` が増える（試験は `hit()` を直接呼ぶ）。"""

    def __init__(self):
        self.count = 0

    def hit(self):
        self.count += 1


class _Abort(Exception):
    def __init__(self, code):
        super().__init__(code)
        self.code = code


def _uuid_hex(uuid_msg):
    return bytes(uuid_msg.uuid).hex()


def _status_to_dict(message):
    return {
        "interface_version": message.interface_version,
        "phase": message.phase,
        "goal_id": _uuid_hex(message.goal_id),
        "last_reject_reason": message.last_reject_reason,
    }


def _cleanup_to_dict(message):
    return {
        "goal_id": _uuid_hex(message.goal_id),
        "reason": message.reason,
        "lift_restored": bool(message.lift_restored),
    }


def _panel_to_dict(panel):
    return {
        "x_m": panel.x_m,
        "y_m": panel.y_m,
        "top_height_m": panel.top_height_m,
        "height_measured": bool(panel.height_measured),
        "yaw_rad": panel.yaw_rad,
        "width_m": panel.width_m,
        "depth_m": panel.depth_m,
        "pos_std_m": panel.pos_std_m,
        "yaw_std_deg": panel.yaw_std_deg,
        "lugs": int(panel.lugs),
        "front_known": bool(panel.front_known),
        "front_yaw_rad": panel.front_yaw_rad,
    }


def _result_to_dict(result):
    return {
        "reason": result.reason,
        "panels": [_panel_to_dict(p) for p in result.panels],
        "seen_from_rad": [float(v) for v in result.seen_from_rad],
        "seen_span_rad": [float(v) for v in result.seen_span_rad],
        "cleanup_limit_s": float(result.cleanup_limit_s),
    }


def build_parser():
    parser = argparse.ArgumentParser(
        prog="locate_cli", description="盤の位置を探す依頼を手で出して、結果を見る（テスト用）"
    )
    parser.add_argument("--from-deg", type=float, default=-180.0,
                        help="方向の範囲の開始 [°]（機体の前が 0・左が正）")
    parser.add_argument("--span-deg", type=float, default=360.0,
                        help="方向の範囲の幅 [°]（0 より大きく 360 以下）")
    parser.add_argument("--max-distance-m", type=float, default=0.0,
                        help="探す最大の距離 [m]。0 は本システムの既定値")
    parser.add_argument("--time-limit-s", type=float, default=120.0, help="制限時間 [s]")
    parser.add_argument("--yes", action="store_true", help="依頼の前の確認を省く")
    parser.add_argument("--status", action="store_true", help="状態を 1 回出して終わる")
    parser.add_argument("--plot", metavar="PATH", help="上から見た図を PNG で書き出す")
    parser.add_argument("--json", metavar="PATH", help="結果と後片付けの完了を JSON で書き出す")
    parser.add_argument("--wait-s", type=float, default=5.0,
                        help="本システムの状態が届くのを待つ上限 [s]")
    parser.add_argument("--cleanup-margin-s", type=float, default=10.0,
                        help="後片付けの完了を待つ上限に足す余裕 [s]")
    return parser


class _Cli:
    def __init__(self, args, confirm, interrupt, out):
        self.args = args
        self.confirm = confirm
        self.interrupt = interrupt
        self.out = out
        self.lock = threading.Lock()
        self.status = None
        self.cleanups = []
        self.feedback = []
        self.node = None
        self.executor = None
        self.thread = None
        self.client = None

    # -- 補助
    def _sleep_check(self):
        time.sleep(POLL_S)
        if self.interrupt.count >= 2:
            raise _Abort(130)

    def _wait_until(self, predicate, timeout_s):
        """条件が成り立てば True、`timeout_s` を過ぎれば False。2 回目の取り消しで 130。"""
        deadline = time.monotonic() + timeout_s
        while True:
            if predicate():
                return True
            if time.monotonic() >= deadline:
                return False
            self._sleep_check()

    def _on_status(self, message):
        with self.lock:
            self.status = _status_to_dict(message)

    def _on_cleanup(self, message):
        with self.lock:
            self.cleanups.append(_cleanup_to_dict(message))

    def _status_snapshot(self):
        with self.lock:
            return None if self.status is None else dict(self.status)

    # -- 準備と後始末
    def open(self):
        name = f"locate_cli_{os.getpid()}_{random.randrange(10**6)}"
        self.node = rclpy.create_node(name)
        # 購読は依頼の前から（QoS は口の取り決めどおり）
        self.node.create_subscription(LocatorStatus, STATUS_TOPIC, self._on_status, STATUS_QOS)
        self.node.create_subscription(CleanupDone, CLEANUP_TOPIC, self._on_cleanup, CLEANUP_QOS)
        self.client = ActionClient(self.node, LocatePanels, ACTION_NAME)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def close(self):
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.0)
        if self.thread is not None:
            self.thread.join(timeout=3.0)
        if self.client is not None:
            self.client.destroy()
        if self.node is not None:
            self.node.destroy_node()

    # -- 流れ
    def run(self, goal):
        out = self.out
        # 1. 状態が届くのを待つ
        if not self._wait_until(lambda: self._status_snapshot() is not None, self.args.wait_s):
            out("本システムが応えません（状態が届きません）。")
            return 3
        status = self._status_snapshot()
        # 2. 版の照合
        if status["interface_version"] != locate_view.INTERFACE_VERSION:
            out(
                f"口の版が合いません（本システム \"{status['interface_version']}\"・"
                f"この道具 \"{locate_view.INTERFACE_VERSION}\"）。"
            )
            return 3
        # 3. 状態を出す
        out(locate_view.format_status(status))
        if self.args.status:
            return 0
        # 4. 受けられる状態か
        if status["phase"] != "IDLE":
            out(f"別の依頼の最中です（phase={status['phase']}）。依頼を出しません。")
            return 4
        if not self.client.wait_for_server(timeout_sec=self.args.wait_s):
            out("本システムが応えません（アクションが見つかりません）。")
            return 3
        # 5. 確認
        out(
            "依頼: 開始 {:+.1f}°  幅 {:.1f}°  最大の距離 {} m  制限時間 {:.0f} s".format(
                math.degrees(goal["azimuth_from_rad"]),
                math.degrees(goal["azimuth_span_rad"]),
                goal["max_distance_m"] if goal["max_distance_m"] > 0 else "既定",
                goal["time_limit_s"],
            )
        )
        if not self.args.yes:
            try:
                answer = self.confirm(CONFIRM_PROMPT)
            except (KeyboardInterrupt, EOFError):
                out("確認の途中で終わりました。依頼を出していません。")
                return 0
            if self.interrupt.count >= 1:
                out("確認の途中で取り消されました。依頼を出していません。")
                return 0
            if str(answer).strip().lower() not in ("y", "yes"):
                out("依頼を出さずに終わります。")
                return 0
        # 6. 依頼
        message = LocatePanels.Goal()
        message.azimuth_from_rad = goal["azimuth_from_rad"]
        message.azimuth_span_rad = goal["azimuth_span_rad"]
        message.max_distance_m = goal["max_distance_m"]
        message.time_limit_s = goal["time_limit_s"]
        started = time.monotonic()
        stamp = time.time()

        def on_feedback(fb):
            with self.lock:
                self.feedback.append(fb.feedback.phase)

        send_future = self.client.send_goal_async(message, feedback_callback=on_feedback)
        if not self._wait_until(send_future.done, self.args.wait_s + 5.0):
            out("依頼への返事が届きません。")
            return 3
        handle = send_future.result()
        if not handle.accepted:
            reason = ""
            self._wait_until(
                lambda: (self._status_snapshot() or {}).get("last_reject_reason", "") != "", 1.0
            )
            reason = (self._status_snapshot() or {}).get("last_reject_reason", "")
            out(f"依頼が拒否されました（last_reject_reason={reason or '-'}）。")
            return 4
        goal_id = _uuid_hex(handle.goal_id)
        out("依頼を受け付けました。")

        # 7. 途中経過。取り消しの印があればアクションの取り消しを送る
        result_future = handle.get_result_async()
        printed = 0
        cancel_sent = False
        limit = goal["time_limit_s"] + 2 * self.args.cleanup_margin_s + 30.0
        while not result_future.done():
            with self.lock:
                new = self.feedback[printed:]
                printed += len(new)
            for phase in new:
                out(f"[{time.monotonic() - started:6.1f} s] {phase}")
            if self.interrupt.count >= 1 and not cancel_sent:
                out("取り消します。結果と後片付けの完了を待ちます（もう一度 Ctrl+C で待たずに終わる）。")
                handle.cancel_goal_async()
                cancel_sent = True
            if time.monotonic() - started > limit:
                out("結果が届きません。")
                return 5
            self._sleep_check()
        with self.lock:
            rest = self.feedback[printed:]
        for phase in rest:
            out(f"[{time.monotonic() - started:6.1f} s] {phase}")
        result = _result_to_dict(result_future.result().result)

        # 8. 結果
        out(locate_view.format_result(result))
        if self.args.plot:
            cv2.imwrite(self.args.plot, locate_view.draw_top_view(result, goal))
            out(f"図を書き出しました: {self.args.plot}")
        out(KEEP_STILL)

        # 9. 後片付けの完了（依頼の ID が同じものだけ）
        wait_s = locate_view.cleanup_wait_limit_s(result, self.args.cleanup_margin_s)
        out(f"後片付けの完了を待ちます（上限 {wait_s:.1f} s）。")

        def mine():
            with self.lock:
                return next((c for c in self.cleanups if c["goal_id"] == goal_id), None)

        found = self._wait_until(lambda: mine() is not None, wait_s)
        cleanup = mine()
        if self.args.json:
            record = locate_view.result_to_dict(goal, result, cleanup, accepted_stamp=stamp)
            with open(self.args.json, "w", encoding="utf-8") as f:
                json.dump(record, f, ensure_ascii=False, indent=2)
            out(f"記録を書き出しました: {self.args.json}")
        if not found:
            out("後片付けの完了が届きません。昇降機の状態を確かめてください。")
            return 5
        out(locate_view.format_cleanup(cleanup))
        return 0


def run_cli(argv, confirm=input, interrupt=None, out=print):
    """コマンドの本体。`rclpy` を初期化済みであること。終了コードを返す。

    `confirm`（確認の入力）と `interrupt`（取り消しの印）は差し替えられる。
    """
    parser = build_parser()
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        return int(exc.code) if isinstance(exc.code, int) else 2
    try:
        goal = locate_view.goal_from_args(
            args.from_deg, args.span_deg, args.max_distance_m, args.time_limit_s
        )
    except ValueError as exc:
        out(f"引数の誤り: {exc}")
        return 2
    interrupt = interrupt if interrupt is not None else InterruptFlag()
    cli = _Cli(args, confirm, interrupt, out)
    cli.open()
    try:
        return cli.run(goal)
    except _Abort as exc:
        if exc.code == 130:
            out("後片付けの完了を見ていません。機体と昇降機を動かす前に状態を確かめてください。")
        return exc.code
    finally:
        cli.close()


def main(args=None):
    argv = sys.argv[1:] if args is None else list(args)
    rclpy.init(args=argv)
    flag = InterruptFlag()
    signal.signal(signal.SIGINT, lambda _signum, _frame: flag.hit())
    try:
        code = run_cli(remove_ros_args(argv), interrupt=flag)
    finally:
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()

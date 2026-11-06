"""ノードの配線の試験。ノードを同じプロセスで起動し、rclpy のクライアントから依頼する。"""
import math
import threading
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ss_locator.locator_node import LocatorNode
from ss_msgs.action import LocatePanels
from ss_msgs.msg import CleanupDone, LocatorStatus

FIXTURE = Path(__file__).with_name("fixtures") / "proto_9265d2d.json"

# 試験を速くするため、昇降機の移動量・待ち時間・状態の周期だけを小さくする。
BASE_PARAMETERS = {
    "use_fakes": True,
    "fake_fixture": str(FIXTURE),
    "fake_scene": 0,
    "scan_camera_height_m": 0.2,
    "min_camera_height_m": 0.0,
    "camera_height_offset_m": 0.0,
    "status_period_ms": 100,
    "settle_s": 0.05,
    "cleanup_timeout_s": 10.0,
}


def parameters(**overrides):
    values = dict(BASE_PARAMETERS)
    values.update(overrides)
    return [Parameter(name, value=value) for name, value in values.items()]


PARAMETERS = parameters()

CLEANUP_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class HeldResultNode(LocatorNode):
    """結果を返す直前で止め、cleanup_done との順序を確かめられるようにする。"""

    def __init__(self, **kwargs):
        self.result_ready = threading.Event()
        self.release_result = threading.Event()
        super().__init__(**kwargs)

    def _execute(self, goal_handle):
        goal_id = bytes(goal_handle.goal_id.uuid)
        with self._lock:
            event = self._result_events.setdefault(goal_id, threading.Event())
        while not event.wait(0.1):
            if self._shutdown:
                return LocatePanels.Result()
        self.result_ready.set()
        self.release_result.wait(5.0)
        with self._lock:
            result = self._results.pop(goal_id, None)
            self._result_events.pop(goal_id, None)
            self._result_returned_at[goal_id] = time.monotonic()
        return result if result is not None else LocatePanels.Result()

    def destroy_node(self):
        self.release_result.set()
        super().destroy_node()


class Rig:
    def __init__(self, node_class=LocatorNode, parameter_overrides=None):
        overrides = PARAMETERS if parameter_overrides is None else parameter_overrides
        self.node = node_class(parameter_overrides=overrides)
        self.client_node = rclpy.create_node("locator_test_client")
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.executor.add_node(self.client_node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

        self.lock = threading.Lock()
        self.cleanups = []
        self.statuses = []
        self.result_times = {}

        self.client = ActionClient(self.client_node, LocatePanels, "/ss/locate/panels")
        self.cleanup_sub = self.client_node.create_subscription(
            CleanupDone, "/ss/locate/cleanup_done", self._on_cleanup, CLEANUP_QOS
        )
        self.status_sub = self.client_node.create_subscription(
            LocatorStatus, "/ss/locate/status", self._on_status, STATUS_QOS
        )

        start = time.monotonic()
        while not self.client.server_is_ready():
            if time.monotonic() - start > 10.0:
                raise AssertionError("action server が見つからない")
            time.sleep(0.05)

    def _on_cleanup(self, message):
        with self.lock:
            self.cleanups.append((time.monotonic(), message))

    def _on_status(self, message):
        with self.lock:
            self.statuses.append((time.monotonic(), message))

    def wait_future(self, future, timeout):
        start = time.monotonic()
        while not future.done():
            if time.monotonic() - start > timeout:
                raise AssertionError("future が終わらない")
            time.sleep(0.01)
        return future.result()

    def wait_until(self, predicate, timeout, message):
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            with self.lock:
                if predicate():
                    return
            time.sleep(0.01)
        raise AssertionError(message)

    def send_goal(self, **overrides):
        goal = LocatePanels.Goal()
        goal.azimuth_from_rad = float(overrides.get("azimuth_from_rad", -math.pi))
        goal.azimuth_span_rad = float(overrides.get("azimuth_span_rad", 2.0 * math.pi))
        goal.max_distance_m = float(overrides.get("max_distance_m", 0.0))
        goal.time_limit_s = float(overrides.get("time_limit_s", 30.0))
        return self.wait_future(self.client.send_goal_async(goal), 10.0)

    def get_result(self, goal_handle, timeout=30.0):
        goal_id = bytes(goal_handle.goal_id.uuid)
        future = goal_handle.get_result_async()

        def record_time(_):
            with self.lock:
                self.result_times[goal_id] = time.monotonic()

        future.add_done_callback(record_time)
        return self.wait_future(future, timeout).result

    def close(self):
        self.executor.shutdown(timeout_sec=5.0)
        self.thread.join(timeout=5.0)
        self.client.destroy()
        self.client_node.destroy_node()
        self.node.destroy_node()


def test_node_cleanup_after_result():
    rig = Rig()
    try:
        goal_handle = rig.send_goal()
        assert goal_handle.accepted

        result = rig.get_result(goal_handle)
        assert result.reason == "completed"
        assert len(result.panels) >= 1

        rig.wait_until(lambda: len(rig.cleanups) >= 1, 10.0, "cleanup_done が来ない")
        with rig.lock:
            cleanup_time, cleanup = rig.cleanups[0]
            result_time = rig.result_times[bytes(goal_handle.goal_id.uuid)]

        assert bytes(cleanup.goal_id.uuid) == bytes(goal_handle.goal_id.uuid)
        assert cleanup.reason == result.reason
        assert cleanup.lift_restored is True
        assert cleanup_time > result_time

        time.sleep(0.5)
        with rig.lock:
            assert len(rig.cleanups) == 1
    finally:
        rig.close()


def test_node_busy_reject():
    rig = Rig()
    try:
        first = rig.send_goal()
        assert first.accepted

        running = rig.send_goal()
        assert not running.accepted

        result = rig.get_result(first)
        assert result.reason == "completed"

        cleaning = rig.send_goal()
        assert not cleaning.accepted

        rig.wait_until(
            lambda: any(message.last_reject_reason == "busy" for _, message in rig.statuses),
            5.0,
            "last_reject_reason が busy にならない",
        )

        with rig.lock:
            phases = [message.phase for _, message in rig.statuses]
        # phase が変わった時点でも status を出す
        assert "RUNNING" in phases
        assert "CLEANING" in phases
    finally:
        rig.close()


def test_node_status_period():
    rig = Rig()
    try:
        rig.wait_until(lambda: len(rig.statuses) >= 8, 3.0, "status が来ない")
        with rig.lock:
            samples = list(rig.statuses)

        assert all(message.interface_version == "1" for _, message in samples)
        assert samples[-1][1].phase == "IDLE"

        times = [stamp for stamp, _ in samples]
        intervals = [b - a for a, b in zip(times, times[1:])]
        # 最初の 1 件は transient_local で残っていたものをすぐ受け取るので、2 件目以降で見る
        steady = intervals[1:]
        assert steady
        assert max(steady) < 0.4
        mean = sum(steady) / len(steady)
        assert 0.05 <= mean <= 0.2
    finally:
        rig.close()


def test_node_cleanup_immediate_ordering():
    # 人の割り込みなら、結果と後片付けの完了が同じ step で出る（昇降機は戻さない）
    rig = Rig(
        node_class=HeldResultNode,
        parameter_overrides=parameters(status_period_ms=1000),
    )
    try:
        goal_handle = rig.send_goal()
        assert goal_handle.accepted
        goal_start_ns = rig.node.get_clock().now().nanoseconds

        # 昇降機を動かし始めてから人の割り込みを入れる
        rig.wait_until(
            lambda: any(message.phase == "RUNNING" for _, message in rig.statuses),
            5.0,
            "RUNNING にならない",
        )
        with rig.node._lock:
            rig.node._lift.take_over()

        assert rig.node.result_ready.wait(5.0), "結果が用意されない"

        # 結果をまだ返していないあいだは cleanup_done を出さない
        time.sleep(1.2)
        with rig.lock:
            assert rig.cleanups == []

        # 後片付けが終わる前に出した依頼は busy で拒否する
        blocked = rig.send_goal()
        assert not blocked.accepted

        rig.node.release_result.set()
        result = rig.get_result(goal_handle)
        assert result.reason == "human_override"

        # 結果を受け取った直後も、cleanup_done までは受け付けない
        rejected = rig.send_goal()
        assert not rejected.accepted

        rig.wait_until(
            lambda: any(message.last_reject_reason == "busy" for _, message in rig.statuses),
            5.0,
            "last_reject_reason が busy にならない",
        )
        rig.wait_until(lambda: len(rig.cleanups) >= 1, 10.0, "cleanup_done が来ない")

        with rig.lock:
            cleanup_time, cleanup = rig.cleanups[0]
            result_time = rig.result_times[bytes(goal_handle.goal_id.uuid)]
            statuses = list(rig.statuses)

        assert cleanup_time > result_time
        assert bytes(cleanup.goal_id.uuid) == bytes(goal_handle.goal_id.uuid)
        assert cleanup.reason == "human_override"
        assert cleanup.lift_restored is False

        cleanup_ns = stamp_ns(cleanup.header.stamp)
        for status_time, status in statuses:
            status_ns = stamp_ns(status.header.stamp)
            if status_ns < goal_start_ns:
                continue
            # cleanup_done より前に IDLE を出していない。
            # 配信順が入れ替わっても見分けられるよう、刻印で比べる。
            if status_time < cleanup_time and status.phase == "IDLE":
                assert status_ns >= cleanup_ns
        assert any(
            status.phase == "IDLE" and stamp_ns(status.header.stamp) >= cleanup_ns
            for _, status in statuses
        )
        assert any(status.last_reject_reason == "busy" for _, status in statuses)
    finally:
        rig.close()

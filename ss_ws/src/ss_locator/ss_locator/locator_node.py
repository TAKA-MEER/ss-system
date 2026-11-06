"""盤の位置を探すノード（ROS 2 の配線）。

`locate_core` の命令を実行し、昇降機・カメラ・検出器の出来事を渡す。
撮った画像の検出と `assemble.lift` はこのノードで行う。
本物の hve-system につなぐ部分は `WP-HVE-01` まで無いので、`use_fakes` が false のときは
起動時にエラーで止まる。

詳細設計: `docs/plan/detailed/DetailedDesign-locate.md`、
`docs/plan/detailed/DetailedDesign-core.md` §5、`docs/plan/detailed/DetailedDesign-names.md` §3・§5。
"""
from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ss_msgs.action import LocatePanels
from ss_msgs.msg import CleanupDone, LocatorStatus, PanelTop
from unique_identifier_msgs.msg import UUID

from . import assemble, geometry, scan_plan
from .fakes import FakeCamera, FakeDetector, FakeLift
from .locate_core import (
    CameraPoint,
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

INTERFACE_VERSION = "1"

ACTION_NAME = "/ss/locate/panels"
CLEANUP_TOPIC = "/ss/locate/cleanup_done"
STATUS_TOPIC = "/ss/locate/status"

# 材料（tests/fixtures/proto_9265d2d.json）の合成データの値。
# 本物の値は WP-CAL-01 で校正する（names.md §5）。
DEFAULT_CAMERA_K = [
    1082.4794514973958, 0.0, 640.0,
    0.0, 1082.4794514973958, 360.0,
    0.0, 0.0, 1.0,
]
DEFAULT_CAMERA_DIST = [-0.018133423603158887, 0.0, 0.0, 0.0, 0.0]

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


class LocatorNode(Node):
    """`/ss/locate/panels` のアクションと `/ss/locate/cleanup_done`・`/ss/locate/status` を持つ。"""

    def __init__(self, **kwargs):
        super().__init__("locator_node", **kwargs)

        self._declare_parameters()
        self._read_parameters()

        if not self.use_fakes:
            raise RuntimeError(
                "use_fakes=false: 本物の hve-system とつなぐ部分は WP-HVE-01 まで無い"
            )

        self._lock = threading.RLock()
        self._shutdown = False

        self._core = LocateCore(self._core_parameters())

        self._lift = FakeLift()
        self._camera = FakeCamera(self.fake_fixture, scene=self.fake_scene)
        self._detector = FakeDetector(self.fake_fixture, scene=self.fake_scene)
        self._last_lift = self._lift.state()

        self._events = []
        self._results = {}
        self._result_events = {}
        self._result_returned_at = {}
        self._pending_cleanup = []

        self._active_goal_id = None
        self._active_goal_uuid = None
        self._active_handle = None
        self._last_reject_reason = ""
        self._accepted_stamp = None

        self._last_tick = time.monotonic()
        self._last_published_phase = None
        self._status_published_this_tick = False

        self._cleanup_pub = self.create_publisher(CleanupDone, CLEANUP_TOPIC, CLEANUP_QOS)
        self._status_pub = self.create_publisher(LocatorStatus, STATUS_TOPIC, STATUS_QOS)

        # 受け付けの判定が昇降機・カメラの今の状態を見られるように、最初の状態を渡しておく
        with self._lock:
            self._events.append(self._lift.state())
            self._events.append(self._camera.state())
            self._drain(self._last_tick)

        self._action_server = ActionServer(
            self,
            LocatePanels,
            ACTION_NAME,
            self._execute,
            goal_callback=self._on_goal,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=self._on_cancel,
        )

        self._timer = self.create_timer(self.status_period_s, self._on_timer)

    # ------------------------------------------------------------------ 設定
    def _declare_parameters(self):
        self.declare_parameter("model_file", "model/lug_v3_s.pt")
        self.declare_parameter(
            "model_sha256",
            "5a4689a9d49d549f61dddb1f2fb375583e2a98ca0333791776e6b140f4e83227",
        )
        self.declare_parameter("detect_imgsz", 1280)
        self.declare_parameter("detect_conf", 0.25)

        self.declare_parameter("max_distance_default_m", 5.0)
        self.declare_parameter("top_height_default_m", 2.35)
        self.declare_parameter("status_period_ms", 200)
        self.declare_parameter("cleanup_timeout_s", 30.0)

        self.declare_parameter("scan_camera_height_m", 3.10)
        self.declare_parameter("min_camera_height_m", 2.95)
        self.declare_parameter("scan_pitch_down_deg", 20.0)
        self.declare_parameter("scan_overlap_ratio", 0.1)
        self.declare_parameter("settle_s", 0.5)
        self.declare_parameter("image_timeout_s", 5.0)

        self.declare_parameter("camera_mount_x_m", 0.0)
        self.declare_parameter("camera_mount_y_m", 0.0)
        self.declare_parameter("camera_forward_offset_m", 0.0)
        self.declare_parameter("camera_height_offset_m", 0.0)
        self.declare_parameter("camera_K", DEFAULT_CAMERA_K)
        self.declare_parameter("camera_dist", DEFAULT_CAMERA_DIST)

        self.declare_parameter("use_fakes", False)
        self.declare_parameter(
            "fake_fixture",
            "ss_ws/src/ss_locator/tests/fixtures/proto_9265d2d.json",
        )
        self.declare_parameter("fake_scene", 0)

    def _read_parameters(self):
        def value(name):
            return self.get_parameter(name).value

        self.model_file = str(value("model_file"))
        self.model_sha256 = str(value("model_sha256"))
        self.detect_imgsz = int(value("detect_imgsz"))
        self.detect_conf = float(value("detect_conf"))

        self.max_distance_default_m = float(value("max_distance_default_m"))
        self.top_height_default_m = float(value("top_height_default_m"))
        self.status_period_ms = int(value("status_period_ms"))
        self.cleanup_timeout_s = float(value("cleanup_timeout_s"))

        self.scan_camera_height_m = float(value("scan_camera_height_m"))
        self.min_camera_height_m = float(value("min_camera_height_m"))
        self.scan_pitch_down_deg = float(value("scan_pitch_down_deg"))
        self.scan_overlap_ratio = float(value("scan_overlap_ratio"))
        self.settle_s = float(value("settle_s"))
        self.image_timeout_s = float(value("image_timeout_s"))

        self.camera_mount_x_m = float(value("camera_mount_x_m"))
        self.camera_mount_y_m = float(value("camera_mount_y_m"))
        self.camera_forward_offset_m = float(value("camera_forward_offset_m"))
        self.camera_height_offset_m = float(value("camera_height_offset_m"))
        self._camera_K = np.asarray(value("camera_K"), float).reshape(3, 3)
        self._camera_dist = np.asarray(value("camera_dist"), float)

        self.use_fakes = bool(value("use_fakes"))
        self.fake_fixture = str(value("fake_fixture"))
        self.fake_scene = int(value("fake_scene"))

        self.status_period_s = max(0.001, self.status_period_ms / 1000.0)

    def _core_parameters(self):
        return {
            "max_distance_default_m": self.max_distance_default_m,
            "top_height_default_m": self.top_height_default_m,
            "scan_camera_height_m": self.scan_camera_height_m,
            "min_camera_height_m": self.min_camera_height_m,
            "scan_pitch_down_deg": self.scan_pitch_down_deg,
            "scan_overlap_ratio": self.scan_overlap_ratio,
            "settle_s": self.settle_s,
            "image_timeout_s": self.image_timeout_s,
            "cleanup_timeout_s": self.cleanup_timeout_s,
            "camera_height_offset_m": self.camera_height_offset_m,
            "hfov_rad": scan_plan.hfov_from_fx(float(self._camera_K[0, 0])),
        }

    # ------------------------------------------------------------------ アクション
    def _on_goal(self, goal_request):
        goal = GoalRequest(
            azimuth_from_rad=float(goal_request.azimuth_from_rad),
            azimuth_span_rad=float(goal_request.azimuth_span_rad),
            max_distance_m=float(goal_request.max_distance_m),
            time_limit_s=float(goal_request.time_limit_s),
        )
        now = time.monotonic()
        with self._lock:
            # cleanup_done を出すまでは、依頼する側から見てまだ後片付け中
            if self._pending_cleanup:
                self._last_reject_reason = "busy"
                return GoalResponse.REJECT
            reason = self._core.accept(goal, now)
            if reason is not None:
                self._last_reject_reason = reason
                return GoalResponse.REJECT
            self._accepted_stamp = self.get_clock().now().to_msg()
        return GoalResponse.ACCEPT

    def _on_accepted(self, goal_handle):
        goal_id = bytes(goal_handle.goal_id.uuid)
        with self._lock:
            self._active_handle = goal_handle
            self._active_goal_id = goal_id
            self._active_goal_uuid = goal_handle.goal_id
            self._result_events.setdefault(goal_id, threading.Event())
        goal_handle.execute()

    def _on_cancel(self, goal_handle):
        goal_id = bytes(goal_handle.goal_id.uuid)
        with self._lock:
            if self._active_goal_id != goal_id:
                return CancelResponse.REJECT
            self._events.append(Cancel())
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        """`goal_handle.execute()` から呼ばれる。結果が出るまで待って返す。"""
        goal_id = bytes(goal_handle.goal_id.uuid)
        with self._lock:
            event = self._result_events.setdefault(goal_id, threading.Event())
        while not event.wait(0.1):
            if self._shutdown:
                return LocatePanels.Result()
        with self._lock:
            result = self._results.pop(goal_id, None)
            self._result_events.pop(goal_id, None)
            self._result_returned_at[goal_id] = time.monotonic()
        return result if result is not None else LocatePanels.Result()

    # ------------------------------------------------------------------ 時計
    def _on_timer(self):
        now = time.monotonic()
        with self._lock:
            dt = now - self._last_tick
            self._last_tick = now
            if self._lift is not None:
                self._last_lift = self._lift.advance(dt)
                self._events.append(self._last_lift)
            else:
                self._events.append(Tick())
            self._status_published_this_tick = False
            self._drain(now)
            self._flush_cleanup(now)
            if not self._status_published_this_tick:
                self._publish_status(now)

    def _drain(self, now):
        while self._events:
            event = self._events.pop(0)
            for command in self._core.step(event, now):
                self._execute_command(command)
            # phase が変わったら、周期を待たずに出す
            if self._core.state != self._last_published_phase:
                self._publish_status(now)

    # ------------------------------------------------------------------ 命令
    def _execute_command(self, command):
        if isinstance(command, (LiftGoto, LiftRelease)):
            self._lift.apply(command)
        elif isinstance(command, CameraPoint):
            self._camera.point(command.yaw_rad, command.pitch_down_rad)
            self._events.append(self._camera.state())
        elif isinstance(command, RequestFrame):
            self._handle_frame()
        elif isinstance(command, SendFeedback):
            self._send_feedback(command.phase)
        elif isinstance(command, SendResult):
            self._send_result(command)
        elif isinstance(command, SendCleanupDone):
            self._send_cleanup_done(command)

    def _handle_frame(self):
        captured = self._camera.capture()
        if captured is None:
            self._events.append(FrameDone(float(self._camera.yaw_rad), [], False))
            return
        keypoints, confidences = self._detector.detect(captured["view"])
        camera = self._camera_parameters(captured)
        lugs = assemble.lift(
            keypoints,
            confidences,
            camera,
            captured["view"],
            self.top_height_default_m,
        )
        self._events.append(FrameDone(float(captured["yaw_rad"]), lugs, True))

    def _camera_parameters(self, captured):
        """偽のときだけ材料の `T` を使い、本物では `geometry.camera_T` で作る（分岐はここだけ）。"""
        if self.use_fakes:
            return captured["K"], captured["dist"], captured["T"]

        # 本物の hve-system につなぐ部分は WP-HVE-01。今はここへ来ない
        lift = self._last_lift if self._last_lift is not None else LiftState(0.0)
        height = float(lift.height_m) + self.camera_height_offset_m
        yaw = float(self._camera.yaw_rad) if self._camera is not None else 0.0
        pitch = (
            float(self._camera.pitch_down_rad) if self._camera is not None else 0.0
        )
        T = geometry.camera_T(
            height,
            yaw,
            pitch,
            self.camera_mount_x_m,
            self.camera_mount_y_m,
            self.camera_forward_offset_m,
        )
        return self._camera_K, self._camera_dist, T

    def _send_feedback(self, phase):
        if self._active_handle is None:
            return
        feedback = LocatePanels.Feedback()
        feedback.phase = str(phase)
        self._active_handle.publish_feedback(feedback)

    def _send_result(self, command):
        goal_id = self._active_goal_id
        if goal_id is None:
            return
        self._results[goal_id] = self._make_result(command)
        event = self._result_events.setdefault(goal_id, threading.Event())
        event.set()

        handle = self._active_handle
        if handle is None:
            return
        if command.reason == "canceled" or handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.succeed()

    def _make_result(self, command):
        result = LocatePanels.Result()
        if self._accepted_stamp is not None:
            result.header.stamp = self._accepted_stamp
        else:
            result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = "base_link"
        result.reason = str(command.reason)
        for top in command.panels:
            panel = PanelTop()
            panel.x_m = float(top["x_m"])
            panel.y_m = float(top["y_m"])
            panel.top_height_m = float(top["top_height_m"])
            panel.height_measured = bool(top["height_measured"])
            panel.yaw_rad = float(top["yaw_rad"])
            panel.width_m = float(top["width_m"])
            panel.depth_m = float(top["depth_m"])
            panel.pos_std_m = float(top["pos_std_m"])
            panel.yaw_std_deg = float(top["yaw_std_deg"])
            panel.lugs = int(top["lugs"])
            panel.front_known = bool(top["front_known"])
            panel.front_yaw_rad = float(top["front_yaw_rad"])
            result.panels.append(panel)
        result.seen_from_rad = [float(v) for v in command.seen_from]
        result.seen_span_rad = [float(v) for v in command.seen_span]
        return result

    def _send_cleanup_done(self, command):
        goal_id = self._active_goal_id
        goal_uuid = self._active_goal_uuid
        if goal_id is None or goal_uuid is None:
            return
        self._pending_cleanup.append(
            (goal_id, goal_uuid, str(command.reason), bool(command.lift_restored))
        )

    def _flush_cleanup(self, now):
        if not self._pending_cleanup:
            return
        remaining = []
        published = False
        for goal_id, goal_uuid, reason, lift_restored in self._pending_cleanup:
            returned_at = self._result_returned_at.get(goal_id)
            # 結果が相手に届く前に cleanup_done を出さない（結果のあとに 1 回だけ）
            if returned_at is None or now - returned_at < self.status_period_s:
                remaining.append((goal_id, goal_uuid, reason, lift_restored))
                continue

            message = CleanupDone()
            message.header.stamp = self.get_clock().now().to_msg()
            message.goal_id = goal_uuid
            message.reason = reason
            message.lift_restored = lift_restored
            self._cleanup_pub.publish(message)

            self._result_returned_at.pop(goal_id, None)
            if self._active_goal_id == goal_id:
                self._active_goal_id = None
                self._active_goal_uuid = None
                self._active_handle = None
            published = True
        self._pending_cleanup = remaining
        if published:
            # cleanup_done を出した時点で、依頼する側から見て IDLE に戻る
            self._publish_status(now)

    # ------------------------------------------------------------------ 状態
    def _publish_status(self, now=None):
        message = LocatorStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.interface_version = INTERFACE_VERSION
        if self._pending_cleanup:
            # cleanup_done を出すまでは、phase も goal_id も後片付け中のまま見せる
            message.phase = LocateCore.CLEANING
            message.goal_id = self._pending_cleanup[0][1]
        else:
            message.phase = self._core.state
            if message.phase != LocateCore.IDLE and self._active_goal_uuid is not None:
                message.goal_id = self._active_goal_uuid
            else:
                message.goal_id = UUID()
        message.last_reject_reason = self._last_reject_reason
        self._status_pub.publish(message)
        self._last_published_phase = self._core.state
        self._status_published_this_tick = True

    # ------------------------------------------------------------------ 終了
    def destroy_node(self):
        self._shutdown = True
        for event in list(self._result_events.values()):
            event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LocatorNode()
    except Exception:
        rclpy.shutdown()
        raise

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

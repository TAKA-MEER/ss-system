"""locate_cli の試験。`use_fakes` の locator_node を同じプロセスで起こして相手にする。

確認の入力と Ctrl+C（取り消しの印）は `run_cli` に渡す関数・印で差し替え、
実端末・実シグナルには頼らない。
"""
import json
import re
import threading
import time
import uuid
from pathlib import Path

import cv2
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from ss_locator import locator_node
from ss_locator.fakes import FakeLift
from ss_locator.locate_cli import InterruptFlag, run_cli
from ss_locator.locator_node import LocatorNode
from ss_msgs.msg import CleanupDone
from unique_identifier_msgs.msg import UUID

FIXTURE = Path(__file__).with_name("fixtures") / "proto_9265d2d.json"

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
    "cleanup_margin_s": 1.0,
}

CLEANUP_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

PHASE_RE = re.compile(r"\] (RAISING|SCANNING|ESTIMATING)")


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class _NoCleanupPublisher:
    def publish(self, _message):
        pass


class SilentCleanupNode(LocatorNode):
    """cleanup_done を出さないノード（待つ上限を確かめる）。"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._cleanup_pub = _NoCleanupPublisher()


class HeldLift:
    """試験が放すまで着かない偽の昇降機（`test_cli_busy` の手本）。"""

    def __init__(self, release):
        self._release = release
        self._inner = FakeLift()

    def apply(self, command):
        self._inner.apply(command)

    def advance(self, dt_s):
        if not self._release.is_set():
            return self._inner.state()
        return self._inner.advance(dt_s)

    def state(self):
        return self._inner.state()

    def take_over(self):
        self._inner.take_over()


class HeldLiftNode(LocatorNode):
    """昇降機を差し替えたノード（試験が放すまで最初の依頼が IDLE に戻らない）。"""

    def __init__(self, release, **kwargs):
        super().__init__(**kwargs)
        self._lift = HeldLift(release)
        self._last_lift = self._lift.advance(0.0)


class ServerRig:
    def __init__(self, node_class=LocatorNode, node_kwargs=None, **overrides):
        values = dict(BASE_PARAMETERS)
        values.update(overrides)
        params = [Parameter(k, value=v) for k, v in values.items()]
        self.node = node_class(**(node_kwargs or {}), parameter_overrides=params)
        self.helper = rclpy.create_node("locate_cli_test_helper")
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.executor.add_node(self.helper)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def idle(self):
        return self.node._active_goal_id is None

    def wait_idle(self, timeout=20.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.idle():
                return
            time.sleep(0.05)
        raise AssertionError("ノードが IDLE に戻らない")

    def close(self):
        try:
            self.wait_idle(15.0)
        except AssertionError:
            pass
        # 失敗で途中のまま来ても、後続の試験へ残さない（各段を独立に片付ける）
        for step in (
            lambda: self.executor.shutdown(timeout_sec=5.0),
            lambda: self.thread.join(timeout=5.0),
            lambda: self.helper.destroy_node(),
            lambda: self.node.destroy_node(),
        ):
            try:
                step()
            except Exception:
                pass


@pytest.fixture
def rig():
    r = ServerRig()
    yield r
    r.close()


def run(argv, confirm=None, interrupt=None, on_line=None):
    lines = []

    def out(text):
        lines.append(text)
        if on_line is not None:
            on_line(text)

    code = run_cli(
        argv,
        confirm=confirm if confirm is not None else (lambda _prompt: "y"),
        interrupt=interrupt if interrupt is not None else InterruptFlag(),
        out=out,
    )
    return code, lines


def index_of(lines, predicate):
    for i, line in enumerate(lines):
        if predicate(line):
            return i
    raise AssertionError("行が無い: " + "\n".join(lines))


def is_table(line):
    return line.startswith("結果:")


def is_cleanup(line):
    return line.startswith("後片付けの完了:")


def test_cli_completes(rig):
    code, lines = run(["--yes"])
    assert code == 0
    phase = index_of(lines, lambda l: PHASE_RE.search(l) is not None)
    table = index_of(lines, is_table)
    cleanup = index_of(lines, is_cleanup)
    assert phase < table < cleanup
    assert "completed" in lines[table]
    assert "reason=completed" in lines[cleanup]
    assert "lift_restored=True" in lines[cleanup]
    assert any("動かさないでください" in l for l in lines)


def test_cli_confirm_no_sends_nothing(rig):
    prompts = []

    def confirm(prompt):
        prompts.append(prompt)
        return "n"

    code, lines = run([], confirm=confirm)
    assert code == 0
    assert len(prompts) == 1
    time.sleep(0.5)
    assert rig.idle()
    assert not any(is_table(l) for l in lines)


def test_cli_cancel_waits_cleanup(rig):
    flag = InterruptFlag()
    fired = []

    def on_line(text):
        if not fired and PHASE_RE.search(text):
            fired.append(True)
            flag.hit()

    code, lines = run(["--yes"], interrupt=flag, on_line=on_line)
    assert code == 0
    table = index_of(lines, is_table)
    cleanup = index_of(lines, is_cleanup)
    assert "canceled" in lines[table]
    assert table < cleanup
    assert "reason=canceled" in lines[cleanup]


def test_cli_second_interrupt(rig):
    flag = InterruptFlag()

    def on_line(text):
        if PHASE_RE.search(text) and flag.count == 0:
            flag.hit()
        elif text.startswith("取り消します") and flag.count == 1:
            flag.hit()

    started = time.monotonic()
    code, lines = run(["--yes"], interrupt=flag, on_line=on_line)
    assert code == 130
    assert time.monotonic() - started < 5.0
    assert not any(is_cleanup(l) for l in lines)
    assert any("後片付けの完了を見ていません" in l for l in lines)


def test_cli_version_mismatch(monkeypatch):
    monkeypatch.setattr(locator_node, "INTERFACE_VERSION", "2")
    r = ServerRig()
    try:
        code, lines = run(["--yes"])
        assert code == 3
        assert any("版が合いません" in l for l in lines)
        time.sleep(0.3)
        assert r.idle()
        assert not any("受け付けました" in l for l in lines)
    finally:
        r.close()


def test_cli_cleanup_wait_from_limit():
    r = ServerRig(node_class=SilentCleanupNode)
    try:
        measured = []
        for margin in (0.5, 2.5):
            start = []

            def on_line(text, start=start):
                if text.startswith("後片付けの完了を待ちます"):
                    start.append(time.monotonic())

            code, lines = run(["--yes", "--cleanup-margin-s", str(margin)], on_line=on_line)
            elapsed = time.monotonic() - start[0]
            assert code == 5
            table = lines[index_of(lines, is_table):]
            limit = float(
                re.search(r"後片付けの上限: ([\d.]+) s", "\n".join(table)).group(1)
            )
            waited = float(
                re.search(r"上限 ([\d.]+) s", lines[index_of(lines, lambda l: l.startswith("後片付けの完了を待ちます"))]).group(1)
            )
            # 待つ上限は、結果の cleanup_limit_s ＋ 余裕
            assert waited == pytest.approx(limit + margin, abs=0.11)
            assert waited - 0.3 <= elapsed <= waited + 2.0
            measured.append(elapsed)
            r.wait_idle()
        assert measured[1] - measured[0] > 1.0
    finally:
        r.close()


def test_cli_no_server():
    code, lines = run(["--yes", "--wait-s", "0.5"])
    assert code == 3
    assert not any("受け付けました" in l for l in lines)


def test_cli_busy():
    # 試験が放すまで最初の依頼を IDLE に戻さない（時間を長くするだけでは揺れが残る）
    release = threading.Event()
    rig = ServerRig(node_class=HeldLiftNode, node_kwargs={"release": release})
    first = {}

    def first_run():
        first["code"], first["lines"] = run(["--yes"])

    thread = threading.Thread(target=first_run)
    thread.start()
    try:
        # 最初の依頼が受け付けられただけでなく、状態が RUNNING になるまで待つ
        # （受け付け直後は2つ目の依頼が古い IDLE の状態を読むことがある）
        end = time.monotonic() + 10.0
        while time.monotonic() < end:
            _code, status_lines = run(["--status"])
            if any("phase=RUNNING" in line for line in status_lines):
                break
            time.sleep(0.05)
        else:
            raise AssertionError("最初の依頼が RUNNING にならない")
        assert not rig.idle()
        code, lines = run(["--yes"])
        assert code == 4
        assert any("別の依頼" in l for l in lines)
        assert not rig.idle()
    finally:
        release.set()
        thread.join(timeout=60.0)
        rig.close()
    assert first["code"] == 0


def test_cli_status_only(rig):
    code, lines = run(["--status"])
    assert code == 0
    assert any("IDLE" in l and "interface_version=3" in l for l in lines)
    time.sleep(0.3)
    assert rig.idle()
    assert not any("受け付けました" in l for l in lines)


def test_cli_ignores_other_cleanup(rig):
    publisher = rig.helper.create_publisher(CleanupDone, "/ss/locate/cleanup_done", CLEANUP_QOS)

    def foreign():
        message = CleanupDone()
        message.goal_id = UUID(uuid=list(uuid.uuid4().bytes))
        message.reason = "human_override"
        message.lift_restored = False
        publisher.publish(message)

    foreign()
    time.sleep(0.3)

    sent = []

    def on_line(text):
        if not sent and PHASE_RE.search(text):
            sent.append(True)
            foreign()

    code, lines = run(["--yes"], on_line=on_line)
    assert code == 0
    cleanup = lines[index_of(lines, is_cleanup)]
    assert "reason=completed" in cleanup
    assert "human_override" not in cleanup


def test_cli_writes_plot_and_json(rig, tmp_path):
    plot = tmp_path / "top.png"
    record = tmp_path / "result.json"
    code, lines = run(["--yes", "--plot", str(plot), "--json", str(record)])
    assert code == 0
    image = cv2.imread(str(plot))
    assert image.shape[:2] == (800, 800)
    data = json.loads(record.read_text(encoding="utf-8"))
    reason = re.search(r"結果: (\S+)", lines[index_of(lines, is_table)]).group(1)
    assert data["result"]["reason"] == reason
    assert data["cleanup_done"]["reason"] == reason
    assert len(data["result"]["panels"]) >= 1

    rig.wait_idle()
    before = sorted(p.name for p in tmp_path.iterdir())
    code, _ = run(["--yes"])
    assert code == 0
    assert sorted(p.name for p in tmp_path.iterdir()) == before == ["result.json", "top.png"]


def test_cli_json_has_depth_measured(rig, tmp_path):
    record = tmp_path / "result.json"
    code, lines = run(["--yes", "--json", str(record)])
    assert code == 0
    data = json.loads(record.read_text(encoding="utf-8"))
    assert len(data["result"]["panels"]) >= 1
    for panel in data["result"]["panels"]:
        assert isinstance(panel["depth_measured"], bool)


def test_cli_bad_args():
    code, lines = run(["--span-deg", "0"])
    assert code == 2

"""`hve_camera_core`（`CameraLink`・`frame_verdict`・`camera_hello_message`）の試験。

時刻は引数で渡し、実時間では待たない。
"""
import math

import pytest

from ss_locator.hve_camera_core import CameraLink, camera_hello_message, frame_verdict
from ss_locator.locate_core import CameraPoint, CameraRelease


def make_state(pitch_deg=0.0, yaw_deg=0.0, yaw_zeroed=True, boot_id="b1", aim=None,
               owner=None, owner_name=None, owner_operator=None, yaw_dps=10.0):
    return {
        "t": "state",
        "boot_id": boot_id,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg,
        "yaw_zeroed": yaw_zeroed,
        "yaw_dps": yaw_dps,
        "owner": owner,
        "owner_name": owner_name,
        "owner_operator": owner_operator,
        "aim": aim,
    }


def own_aim(press, status, reason="NONE", by="ss"):
    return {
        "by": by,
        "press": press,
        "pitch_deg": 0.0,
        "yaw_deg": 0.0,
        "status": status,
        "reason": reason,
    }


def make_link(**state_kwargs):
    link = CameraLink(cmd_period_ms=100, state_timeout_ms=600)
    link.on_connected(0.0)
    link.poll(0.0)  # hello を捨てる
    link.on_state(make_state(**state_kwargs), 0.0)
    return link


def aims(messages):
    return [m for m in messages if m["t"] == "aim"]


def test_camera_hello_fields():
    message = camera_hello_message("0.1.0")
    assert message["t"] == "hello"
    assert message["name"] == "ss"
    assert message["role"] == "control"
    assert message["operator"] == "auto"
    assert "ceiling_sensor" not in message
    assert message["busy_label"] == "画像認識で操作中"
    assert len(message["busy_label"].encode("utf-8")) <= 48
    assert message["fw"] == "0.1.0"
    # 接続の最初に送る文
    link = CameraLink()
    link.on_connected(0.0)
    messages, reconnect = link.poll(0.0)
    assert messages[0]["t"] == "hello"
    assert reconnect is False


def test_aim_converts_angles():
    link = make_link()
    link.on_command(CameraPoint(math.radians(90.0), math.radians(20.0)), 0.0)
    messages, _ = link.poll(0.0)
    assert len(aims(messages)) == 1
    aim = aims(messages)[0]
    assert aim["press"] == 1
    assert aim["yaw_deg"] == pytest.approx(90.0)
    assert aim["pitch_deg"] == pytest.approx(-20.0)

    # ヨー 190° は -170° に直る
    link.on_command(CameraPoint(math.radians(190.0), 0.0), 0.2)
    aim = aims(link.poll(0.2)[0])[0]
    assert aim["press"] == 2
    assert aim["yaw_deg"] == pytest.approx(-170.0)
    assert aim["pitch_deg"] == 0.0


def test_aim_without_yaw():
    link = make_link()
    link.on_command(CameraPoint(None, math.radians(10.0)), 0.0)
    aim = aims(link.poll(0.0)[0])[0]
    assert "yaw_deg" not in aim
    assert aim["pitch_deg"] == pytest.approx(-10.0)


def test_camera_state_converts_back():
    link = make_link(pitch_deg=-20.0, yaw_deg=88.5, yaw_dps=10.0)
    state = link.camera_state(0.0)
    assert state.pitch_down_rad == pytest.approx(math.radians(20.0))
    assert state.yaw_rad == pytest.approx(math.radians(88.5))
    assert state.yaw_speed_rad_s == pytest.approx(math.radians(10.0))
    assert state.pose_known is True
    assert state.arrived is False and state.taken_over is False and state.failed is False

    link.on_state(make_state(pitch_deg=5.0, yaw_deg=None, yaw_dps=None), 0.1)
    state = link.camera_state(0.1)
    assert state.yaw_rad is None
    assert state.pitch_down_rad == pytest.approx(math.radians(-5.0))
    assert state.yaw_speed_rad_s is None
    assert state.pose_known is False


def test_camera_arrived_needs_same_aim():
    link = make_link()
    link.on_command(CameraPoint(0.0, 0.0), 0.0)  # press 1
    link.poll(0.0)

    # press が違う ARRIVED
    link.on_state(make_state(aim=own_aim(0, "ARRIVED")), 0.05)
    assert link.camera_state(0.05).arrived is False
    # 名前が違う ARRIVED
    link.on_state(make_state(aim=own_aim(1, "ARRIVED", by="other")), 0.06)
    assert link.camera_state(0.06).arrived is False
    # 動いている間
    link.on_state(make_state(aim=own_aim(1, "MOVING")), 0.07)
    assert link.camera_state(0.07).arrived is False
    assert aims(link.poll(0.1)[0])  # 着くまで送り続ける

    link.on_state(make_state(aim=own_aim(1, "ARRIVED")), 0.15)
    assert link.camera_state(0.15).arrived is True
    assert aims(link.poll(0.2)[0]) == []  # 着いたら送らない

    # 新しい命令の直後は、前の press の ARRIVED が残っていても着いたとみなさない
    link.on_command(CameraPoint(1.0, 0.0), 0.3)
    assert link.camera_state(0.3).arrived is False


TAKEN_OVER_STATES = {
    "human": lambda: make_state(owner="ui", owner_operator="human"),
    "preempted": lambda: make_state(aim=own_aim(1, "NOT_ARRIVED", "PREEMPTED")),
    "other_owner": lambda: make_state(owner="module", owner_name="arm",
                                      owner_operator="auto"),
}


@pytest.mark.parametrize("case", sorted(TAKEN_OVER_STATES))
@pytest.mark.parametrize("arrived_first", [False, True])
def test_camera_taken_over(case, arrived_first):
    link = make_link()
    link.on_command(CameraPoint(0.0, 0.0), 0.0)
    assert aims(link.poll(0.0)[0])
    if arrived_first:
        link.on_state(make_state(aim=own_aim(1, "ARRIVED")), 0.05)
        assert link.camera_state(0.05).arrived is True
    assert link.camera_state(0.05).taken_over is False

    link.on_state(TAKEN_OVER_STATES[case](), 0.1)
    state = link.camera_state(0.1)
    assert state.taken_over is True
    assert state.failed is False
    assert aims(link.poll(0.1)[0]) == []
    assert aims(link.poll(0.2)[0]) == []  # 周期が来ても返さない

    # 手を離したあと（命令が無い間）は見ない
    link.on_command(CameraRelease(), 0.3)
    link.on_state(TAKEN_OVER_STATES[case](), 0.3)
    assert link.camera_state(0.3).taken_over is False


def test_camera_own_owner_is_not_taken_over():
    link = make_link()
    link.on_command(CameraPoint(0.0, 0.0), 0.0)
    link.on_state(
        make_state(owner="module", owner_name="ss", owner_operator="auto",
                   aim=own_aim(1, "MOVING")),
        0.05,
    )
    assert link.camera_state(0.05).taken_over is False


@pytest.mark.parametrize("case", ["boot_id", "yaw_zeroed", "yaw_unknown"])
def test_camera_pose_unknown(case):
    link = make_link()
    link.on_command(CameraPoint(0.5, 0.0), 0.0)
    link.poll(0.0)
    state = link.camera_state(0.0)
    assert state.pose_known is True and state.restarted is False

    if case == "boot_id":
        link.on_state(make_state(boot_id="b2"), 0.1)
    elif case == "yaw_zeroed":
        link.on_state(make_state(yaw_zeroed=False, yaw_deg=None), 0.1)
    else:
        link.on_state(make_state(aim=own_aim(1, "NOT_ARRIVED", "YAW_UNKNOWN")), 0.1)
    state = link.camera_state(0.1)
    assert state.pose_known is False
    assert state.failed is False  # 3 ではなく 2
    assert state.taken_over is False
    assert state.restarted is (case == "boot_id")
    assert aims(link.poll(0.1)[0]) == []


def test_camera_pitch_only_command_ignores_yaw_origin():
    # ヨーを向けない命令（再起動のあとのピッチの後片付け）は、ヨーの原点が無くても動かす
    link = make_link(yaw_zeroed=False, yaw_deg=None)
    link.on_command(CameraPoint(None, math.radians(10.0)), 0.0)
    assert aims(link.poll(0.0)[0])
    link.on_state(make_state(yaw_zeroed=False, yaw_deg=None,
                             aim=own_aim(1, "ARRIVED")), 0.05)
    state = link.camera_state(0.05)
    assert state.arrived is True
    assert state.failed is False


@pytest.mark.parametrize(
    "reason",
    ["TARGET_RANGE", "IO_LOST", "CMD_TIMEOUT", "CMD_STOP", "OWNER_GONE", "SOMETHING_NEW"],
)
def test_camera_failed(reason):
    link = make_link()
    link.on_command(CameraPoint(0.0, 0.0), 0.0)
    link.poll(0.0)
    assert link.camera_state(0.0).failed is False

    link.on_state(make_state(aim=own_aim(1, "NOT_ARRIVED", reason)), 0.1)
    state = link.camera_state(0.1)
    assert state.failed is True
    assert state.pose_known is True
    assert state.taken_over is False
    assert aims(link.poll(0.1)[0]) == []


def test_camera_failed_when_stale_or_closed():
    # state が古い
    link = make_link()
    link.on_command(CameraPoint(0.0, 0.0), 0.0)
    link.poll(0.0)
    assert link.camera_state(0.5).failed is False
    state = link.camera_state(0.7)
    assert state.failed is True
    assert state.pose_known is False
    messages, reconnect = link.poll(0.7)
    assert reconnect is True and aims(messages) == []

    # 切れた
    link = make_link()
    link.on_command(CameraPoint(0.0, 0.0), 0.0)
    link.poll(0.0)
    link.on_closed(0.1)
    assert link.camera_state(0.1).failed is True

    # 命令が無い間は見ない
    link = make_link()
    assert link.camera_state(5.0).failed is False


def test_camera_release_sent_once():
    link = make_link()
    link.on_command(CameraRelease(), 0.0)  # 命令が無ければ何も積まない
    assert link.poll(0.0)[0] == []
    link.on_command(CameraPoint(0.0, 0.0), 0.0)
    link.on_command(CameraRelease(), 0.1)
    messages, _ = link.poll(0.1)
    assert messages == [{"t": "release", "press": 1}]
    assert link.poll(0.2)[0] == []


def headers(**overrides):
    values = {
        "X-Boot-Id": "b1",
        "X-Frame-Age-Ms": "100",
        "X-Still-Ms": "700",
        "X-Pitch-Deg": "-20.0",
        "X-Yaw-Deg": "88.5",
    }
    values.update(overrides)
    return {k: v for k, v in values.items() if v is not None}


def test_frame_verdict_table():
    def verdict(status=200, boot_id="b1", since=800, **overrides):
        return frame_verdict(status, headers(**overrides), boot_id, since)

    # 200 で、上のどれでもない
    assert verdict() == "use"
    # 向きが最後に変わってからの経過が、撮ってからの経過より短い（動いている間か動く前のコマ）
    assert verdict(**{"X-Still-Ms": "99", "X-Frame-Age-Ms": "100"}) == "retry"
    # 等しい・大きいは使う
    assert verdict(**{"X-Still-Ms": "100", "X-Frame-Age-Ms": "100"}) == "use"
    assert verdict(**{"X-Still-Ms": "101", "X-Frame-Age-Ms": "100"}) == "use"
    # 起動の番号が命令を始めたときと違う
    assert verdict(boot_id="b0") == "pose_unknown"
    assert verdict(**{"X-Boot-Id": None}) == "pose_unknown"
    # ヨーの向きが無い（null なら付かない）
    assert verdict(**{"X-Yaw-Deg": None}) == "pose_unknown"
    # 上の行が先
    assert verdict(boot_id="b0", **{"X-Still-Ms": "1"}) == "pose_unknown"
    # 429・503
    assert verdict(status=429) == "retry"
    assert verdict(status=503) == "retry"
    # それ以外
    assert verdict(status=500) == "fail"
    assert verdict(status=404) == "fail"
    assert verdict(status=204) == "fail"
    # 使うのに要るヘッダが無い（表に無い。取り直しても直らないので fail）
    assert verdict(**{"X-Pitch-Deg": None}) == "fail"
    assert verdict(**{"X-Still-Ms": None}) == "fail"
    assert verdict(**{"X-Frame-Age-Ms": "abc"}) == "fail"
    # ヘッダ名の大文字小文字は区別しない
    lowered = {k.lower(): v for k, v in headers().items()}
    assert frame_verdict(200, lowered, "b1", 800) == "use"

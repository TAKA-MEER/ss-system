"""`hve_lift_core.LiftLink` の試験。時刻は引数で渡し、実時間では待たない。"""
import pytest

from ss_locator.hve_lift_core import (
    LIFT_MAX_RUN_RETRY,
    LiftLink,
    hello_message,
)
from ss_locator.locate_core import LiftGoto, LiftRelease

PERIOD_S = 0.1
TIMEOUT_S = 0.6


def make_state(height_mm=1000, goto=None, owner=None, owner_name=None,
               owner_operator=None, height_ok=True):
    return {
        "t": "state",
        "height_mm": height_mm,
        "height_ok": height_ok,
        "owner": owner,
        "owner_name": owner_name,
        "owner_operator": owner_operator,
        "goto": goto,
    }


def own_goto(press, target_mm, status, reason="NONE", by="ss"):
    return {
        "by": by,
        "press": press,
        "target_mm": target_mm,
        "status": status,
        "reason": reason,
    }


def make_link(height_mm=1000, speeds=(30, 20)):
    link = LiftLink(cmd_period_ms=100, state_timeout_ms=600)
    if speeds is not None:
        link.set_speeds(*speeds)
    link.on_connected(0.0)
    link.poll(0.0)  # hello を捨てる
    link.on_state(make_state(height_mm), 0.0)
    return link


def gotos(messages):
    return [m for m in messages if m["t"] == "goto"]


def test_hello_fields():
    message = hello_message("0.1.0")
    assert message["t"] == "hello"
    assert message["name"] == "ss"
    assert message["role"] == "control"
    assert message["operator"] == "auto"
    assert message["ceiling_sensor"] is False
    assert message["busy_label"] == "画像認識で操作中"
    assert len(message["busy_label"].encode("utf-8")) <= 48
    assert message["fw"] == "0.1.0"
    # 接続の最初に送る文
    link = LiftLink()
    link.on_connected(0.0)
    messages, reconnect = link.poll(0.0)
    assert messages[0]["t"] == "hello"
    assert reconnect is False


def test_goto_repeats_same_press():
    link = make_link(1000)
    link.on_command(LiftGoto(1.2), 0.0)
    first, _ = link.poll(0.0)
    assert first == [{"t": "goto", "press": 1, "target_mm": 1200, "speed": 30}]
    assert "ceiling" not in first[0]
    # 周期の途中では返さない
    mid, _ = link.poll(0.05)
    assert mid == []
    link.on_state(make_state(1000), 0.05)
    second, _ = link.poll(0.1)
    assert second == first
    link.on_state(make_state(1000), 0.15)
    third, _ = link.poll(0.2)
    assert third == first


def test_new_goto_increments_press():
    link = make_link(1000)
    link.on_command(LiftGoto(1.2), 0.0)
    assert gotos(link.poll(0.0)[0])[0]["press"] == 1
    link.on_command(LiftGoto(1.3), 0.01)
    assert gotos(link.poll(0.01)[0])[0]["press"] == 2
    # 繋ぎ直しても戻らない
    link.on_closed(0.5)
    link.on_connected(1.0)
    link.on_state(make_state(1000), 1.0)
    link.set_speeds(30, 20)
    link.on_command(LiftGoto(1.4), 1.0)
    assert gotos(link.poll(1.0)[0])[0]["press"] == 3


def test_arrived_needs_same_goto():
    link = make_link(1000)
    link.on_command(LiftGoto(1.2), 0.0)
    link.poll(0.0)
    # 前の press で着いたままの state
    link.on_state(make_state(1200, own_goto(0, 1200, "ARRIVED")), 0.01)
    assert link.lift_state(0.01).arrived is False
    # target_mm が違う
    link.on_state(make_state(1200, own_goto(1, 1100, "ARRIVED")), 0.02)
    assert link.lift_state(0.02).arrived is False
    # by が違う（持ち主は null のまま）
    link.on_state(make_state(1200, own_goto(1, 1200, "ARRIVED", by="x")), 0.03)
    assert link.lift_state(0.03).arrived is False
    # 全部同じ
    link.on_state(make_state(1200, own_goto(1, 1200, "ARRIVED")), 0.04)
    assert link.lift_state(0.04).arrived is True


def test_arrived_not_carried_to_next_goto():
    link = make_link(1000)
    link.on_command(LiftGoto(1.2), 0.0)
    link.poll(0.0)
    link.on_state(make_state(1200, own_goto(1, 1200, "ARRIVED")), 0.01)
    assert link.lift_state(0.01).arrived is True
    # 次の LiftGoto の直後に、前の press で着いたままの state を渡す
    link.on_command(LiftGoto(1.0), 0.02)
    link.on_state(make_state(1200, own_goto(1, 1200, "ARRIVED")), 0.03)
    assert link.lift_state(0.03).arrived is False


def test_stops_sending_after_arrived():
    link = make_link(1000)
    link.on_command(LiftGoto(1.2), 0.0)
    assert gotos(link.poll(0.0)[0])
    link.on_state(make_state(1200, own_goto(1, 1200, "ARRIVED")), 0.05)
    assert link.lift_state(0.05).arrived is True
    assert link.poll(0.1)[0] == []
    link.on_state(make_state(1200), 0.15)
    assert link.poll(0.2)[0] == []


def _human_cases():
    return {
        "owner_operator_human": make_state(
            1000, own_goto(1, 1200, "MOVING"), owner="ui", owner_operator="human"
        ),
        "preempted": make_state(
            1000, own_goto(1, 1200, "NOT_ARRIVED", "PREEMPTED"),
            owner="ui", owner_name=None, owner_operator="auto",
        ),
        "other_module_owner": make_state(
            1000, own_goto(1, 1200, "MOVING"),
            owner="module", owner_name="arm", owner_operator="auto",
        ),
    }


@pytest.mark.parametrize("case", ["owner_operator_human", "preempted", "other_module_owner"])
def test_human_takeover_stops_sending(case):
    link = make_link(1000)
    link.on_command(LiftGoto(1.2), 0.0)
    assert gotos(link.poll(0.0)[0])
    link.on_state(_human_cases()[case], 0.05)
    assert link.lift_state(0.05).stopped_reason == "human"
    assert link.poll(0.1)[0] == []
    link.on_state(_human_cases()[case], 0.15)
    assert link.poll(0.2)[0] == []


def test_human_takeover_after_arrived_stops():
    # 3 は着いたあと（送っていない間）でも
    link = make_link(1000)
    link.on_command(LiftGoto(1.2), 0.0)
    link.poll(0.0)
    link.on_state(make_state(1200, own_goto(1, 1200, "ARRIVED")), 0.05)
    assert link.lift_state(0.05).stopped_reason == "none"
    link.on_state(
        make_state(1200, own_goto(1, 1200, "ARRIVED"),
                   owner="module", owner_name="arm", owner_operator="auto"),
        0.1,
    )
    assert link.lift_state(0.1).stopped_reason == "human"
    assert link.poll(0.2)[0] == []


def test_idle_ignores_owner():
    link = make_link(1000)
    link.on_state(
        make_state(1000, owner="ui", owner_operator="human"), 0.05
    )
    assert link.lift_state(0.05).stopped_reason == "none"
    link.on_state(
        make_state(1000, owner="module", owner_name="arm", owner_operator="auto"), 0.1
    )
    assert link.lift_state(0.1).stopped_reason == "none"
    assert link.poll(0.1)[0] == []


@pytest.mark.parametrize(
    "reason,expected",
    [
        ("CEILING_NEAR", "ceiling"),
        ("TOP", "ceiling"),
        ("TARGET_RANGE", "ceiling"),
        ("CEILING_STALE", "ceiling_unknown"),
        ("HEIGHT_UNKNOWN", "refused"),
        ("ENCODER_STALL", "refused"),
        ("ARM_LINK", "refused"),
        ("CMD_TIMEOUT", "refused"),
        ("OWNER_GONE", "refused"),
        ("CMD_STOP", "refused"),
        ("BOTTOM", "refused"),  # 上昇の BOTTOM
        ("SOMETHING_NEW", "refused"),  # 知らない名前
    ],
)
def test_reason_table(reason, expected):
    link = make_link(1000)
    link.on_command(LiftGoto(1.2), 0.0)
    link.poll(0.0)
    link.on_state(make_state(1000, own_goto(1, 1200, "NOT_ARRIVED", reason)), 0.05)
    state = link.lift_state(0.05)
    assert state.stopped_reason == expected
    assert state.arrived is False
    assert link.poll(0.1)[0] == []


def test_reason_table_down_bottom_is_arrived():
    link = make_link(1000)
    link.on_command(LiftGoto(0.5), 0.0)
    link.poll(0.0)
    link.on_state(make_state(0, own_goto(1, 500, "NOT_ARRIVED", "BOTTOM")), 0.05)
    state = link.lift_state(0.05)
    assert state.arrived is True
    assert state.stopped_reason == "none"
    assert link.poll(0.1)[0] == []


def test_speed_from_settings():
    link = make_link(1000, speeds=(30, 20))
    link.on_command(LiftGoto(1.2), 0.0)
    assert gotos(link.poll(0.0)[0])[0]["speed"] == 30
    link.on_command(LiftGoto(0.8), 0.01)
    assert gotos(link.poll(0.01)[0])[0]["speed"] == 20

    # set_speeds の前の LiftGoto は goto を返さず refused
    link = make_link(1000, speeds=None)
    link.on_command(LiftGoto(1.2), 0.0)
    assert gotos(link.poll(0.0)[0]) == []
    assert link.lift_state(0.0).stopped_reason == "refused"
    assert gotos(link.poll(0.2)[0]) == []


def test_max_run_retries():
    link = make_link(1000)
    link.on_command(LiftGoto(1.2), 0.0)
    assert gotos(link.poll(0.0)[0])[0]["press"] == 1
    now = 0.0
    for i in range(1, LIFT_MAX_RUN_RETRY + 1):
        now += 0.05
        link.on_state(
            make_state(1100, own_goto(i, 1200, "NOT_ARRIVED", "MAX_RUN")), now
        )
        sent = gotos(link.poll(now)[0])
        assert sent == [
            {"t": "goto", "press": i + 1, "target_mm": 1200, "speed": 30}
        ]
        assert link.lift_state(now).stopped_reason == "none"
    # 超えたら refused
    now += 0.05
    link.on_state(
        make_state(1100, own_goto(LIFT_MAX_RUN_RETRY + 1, 1200, "NOT_ARRIVED", "MAX_RUN")),
        now,
    )
    assert gotos(link.poll(now)[0]) == []
    assert link.lift_state(now).stopped_reason == "refused"


def test_target_range_down_refused():
    link = make_link(1000)
    link.on_command(LiftGoto(0.5), 0.0)
    link.poll(0.0)
    link.on_state(make_state(1000, own_goto(1, 500, "NOT_ARRIVED", "TARGET_RANGE")), 0.05)
    assert link.lift_state(0.05).stopped_reason == "refused"
    # 上昇は ceiling
    link = make_link(1000)
    link.on_command(LiftGoto(1.5), 0.0)
    link.poll(0.0)
    link.on_state(make_state(1000, own_goto(1, 1500, "NOT_ARRIVED", "TARGET_RANGE")), 0.05)
    assert link.lift_state(0.05).stopped_reason == "ceiling"


def test_height_unknown():
    link = LiftLink(cmd_period_ms=100, state_timeout_ms=600)
    # 繋がっていない
    assert link.lift_state(0.0).height_m is None
    link.on_connected(0.0)
    # state をまだ受けていない
    assert link.lift_state(0.1).height_m is None
    # height_ok が false
    link.on_state(make_state(1000, height_ok=False), 0.2)
    assert link.lift_state(0.2).height_m is None
    # 正常
    link.on_state(make_state(1234), 0.3)
    assert link.lift_state(0.3).height_m == pytest.approx(1.234)
    # lift_state_timeout_ms より古い
    assert link.lift_state(0.3 + TIMEOUT_S + 0.01).height_m is None


def test_stale_requests_reconnect():
    link = make_link(1000)
    link.on_command(LiftGoto(1.2), 0.0)
    assert gotos(link.poll(0.0)[0])
    link.on_state(make_state(1000), 0.1)
    messages, reconnect = link.poll(0.1 + TIMEOUT_S - 0.01)
    assert reconnect is False
    messages, reconnect = link.poll(0.1 + TIMEOUT_S + 0.01)
    assert reconnect is True
    assert gotos(messages) == []
    assert link.lift_state(0.8).stopped_reason == "refused"
    # 繋ぎ直して新しい state が来ても、その命令の goto は返さない
    link.on_closed(0.8)
    link.on_connected(1.8)
    link.on_state(make_state(1000), 1.8)
    messages, reconnect = link.poll(1.9)
    assert gotos(messages) == []
    assert [m["t"] for m in messages] == ["hello"]
    assert link.lift_state(1.9).stopped_reason == "refused"
    link.on_state(make_state(1000), 2.0)
    assert gotos(link.poll(2.1)[0]) == []


def test_release_once():
    link = make_link(1000)
    # 命令が無ければ出ない
    link.on_command(LiftRelease(), 0.0)
    assert link.poll(0.0)[0] == []
    link.on_command(LiftGoto(1.2), 0.0)
    link.poll(0.0)
    link.on_command(LiftGoto(1.3), 0.01)
    link.poll(0.01)
    link.on_command(LiftRelease(), 0.02)
    messages, _ = link.poll(0.02)
    assert messages == [{"t": "release", "press": 2}]
    assert link.poll(0.1)[0] == []
    link.on_command(LiftRelease(), 0.2)
    assert link.poll(0.2)[0] == []

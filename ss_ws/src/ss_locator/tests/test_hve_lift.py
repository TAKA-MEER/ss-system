"""`HveLift` の試験。試験の中で起こす小さな WebSocket のサーバだけを相手にする。"""
import time

import pytest

from ss_locator.hve_lift import HveLift
from ss_locator.locate_core import LiftGoto, LiftRelease

from lift_stub_server import LiftStubServer

SETTINGS = {
    "lift_up": {"min": 10, "max": 60, "init": 30},
    "lift_down": {"min": 10, "max": 60, "init": 20},
}


def wait_until(condition, timeout=5.0, interval=0.02):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = condition()
        if value:
            return value
        time.sleep(interval)
    raise AssertionError("条件が満たされない")


def state(height_mm=1000, goto=None, **extra):
    base = {
        "t": "state",
        "height_mm": height_mm,
        "height_ok": True,
        "owner": None,
        "owner_name": None,
        "owner_operator": None,
        "goto": goto,
    }
    base.update(extra)
    return base


@pytest.fixture
def server():
    srv = LiftStubServer()
    srv.start()
    srv.set_settings(SETTINGS)
    yield srv
    srv.stop()


@pytest.fixture
def make_lift(server):
    lifts = []

    def factory(**kwargs):
        params = dict(cmd_period_ms=50, state_timeout_ms=500)
        params.update(kwargs)
        lift = HveLift("127.0.0.1:%d" % server.port, **params)
        lifts.append(lift)
        return lift

    yield factory
    for lift in lifts:
        lift.close()


def gotos(server):
    return [m for m in server.messages if m.get("t") == "goto"]


def test_hve_lift_hello_first(server, make_lift):
    server.set_auto_state(state())
    make_lift()
    first = wait_until(lambda: server.connections and server.connections[0])
    assert first[0]["t"] == "hello"
    assert first[0]["name"] == "ss"
    assert first[0]["ceiling_sensor"] is False
    # 切って繋ぎ直させても、また hello から
    server.close_connections()
    second = wait_until(
        lambda: len(server.connections) >= 2 and server.connections[1]
    )
    assert second[0]["t"] == "hello"


def test_hve_lift_reads_settings(server, make_lift):
    server.set_settings(
        {
            "lift_up": {"min": 10, "max": 60, "init": 70},  # max を超える
            "lift_down": {"min": 15, "max": 60, "init": 5},  # min を下回る
        }
    )
    server.set_auto_state(state(1000))
    lift = make_lift()
    wait_until(lambda: lift.advance(0.0).height_m is not None)
    lift.apply(LiftGoto(1.2))
    wait_until(lambda: gotos(server))
    assert gotos(server)[0]["speed"] == 60
    lift.apply(LiftGoto(0.5))
    wait_until(lambda: any(g["target_mm"] == 500 for g in gotos(server)))
    assert [g for g in gotos(server) if g["target_mm"] == 500][0]["speed"] == 15


def test_hve_lift_settings_unavailable_refuses(server, make_lift):
    server.set_settings(None)
    server.set_auto_state(state(1000))
    lift = make_lift()
    wait_until(lambda: lift.advance(0.0).height_m is not None)
    lift.apply(LiftGoto(1.2))
    assert lift.advance(0.0).stopped_reason == "refused"
    time.sleep(0.3)
    assert gotos(server) == []
    # 設定が取れるようになれば、次の LiftGoto から動かせる
    server.set_settings(SETTINGS)
    lift.apply(LiftRelease())

    def retry():
        lift.apply(LiftGoto(1.2))
        return gotos(server)

    wait_until(retry, timeout=5.0, interval=0.1)
    assert gotos(server)[0]["speed"] == 30


def test_hve_lift_goto_and_release(server, make_lift):
    server.set_auto_state(state(1000))
    lift = make_lift()
    wait_until(lambda: lift.advance(0.0).height_m is not None)
    lift.apply(LiftGoto(1.2))
    wait_until(lambda: len(gotos(server)) >= 3)  # 続けて届く
    first = gotos(server)[0]
    assert first == {"t": "goto", "press": 1, "target_mm": 1200, "speed": 30}
    assert lift.advance(0.0).arrived is False

    server.set_auto_state(
        state(
            1200,
            {"by": "ss", "press": 1, "target_mm": 1200,
             "status": "ARRIVED", "reason": "NONE"},
        )
    )
    wait_until(lambda: lift.advance(0.0).arrived)
    time.sleep(0.15)
    count = len(gotos(server))
    time.sleep(0.3)
    assert len(gotos(server)) == count  # goto が止まる

    lift.apply(LiftRelease())
    wait_until(lambda: any(m.get("t") == "release" for m in server.messages))
    releases = [m for m in server.messages if m.get("t") == "release"]
    assert releases == [{"t": "release", "press": 1}]


def test_hve_lift_reconnects_when_silent(server, make_lift):
    server.set_auto_state(state(1000))
    lift = make_lift(state_timeout_ms=300)
    wait_until(lambda: lift.advance(0.0).height_m is not None)
    assert len(server.connections) == 1
    server.set_auto_state(None)  # state を送るのをやめる
    second = wait_until(
        lambda: len(server.connections) >= 2 and server.connections[1], timeout=6.0
    )
    assert second[0]["t"] == "hello"
    assert lift.advance(0.0).height_m is None

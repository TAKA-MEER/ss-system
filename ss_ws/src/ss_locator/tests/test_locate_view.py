"""locate_view（ROS 2 なしの純関数）の試験。"""
import math

import numpy as np
import pytest

from ss_locator import locate_view as lv


def panel(**overrides):
    values = {
        "x_m": 2.413, "y_m": 0.310, "top_height_m": 2.35, "height_measured": False,
        "depth_measured": True,
        "yaw_rad": math.radians(178.2), "width_m": 0.8, "depth_m": 0.6,
        "pos_std_m": 0.04, "yaw_std_deg": 2.1, "lugs": 4,
        "front_known": False, "front_yaw_rad": 0.0,
    }
    values.update(overrides)
    return values


def result(panels=(), reason="completed", seen=((-math.pi, 2 * math.pi),), limit=25.0):
    return {
        "reason": reason,
        "panels": list(panels),
        "seen_from_rad": [s[0] for s in seen],
        "seen_span_rad": [s[1] for s in seen],
        "cleanup_limit_s": limit,
    }


GOAL = lv.goal_from_args(-180.0, 360.0, 0.0, 120.0)


def test_goal_from_args_defaults():
    goal = lv.goal_from_args(-180.0, 360.0, 0.0, 120.0)
    assert goal["azimuth_from_rad"] == pytest.approx(-math.pi)
    assert goal["azimuth_span_rad"] == pytest.approx(2 * math.pi)
    assert goal["max_distance_m"] == 0.0
    assert goal["time_limit_s"] == 120.0


def test_goal_from_args_degrees():
    goal = lv.goal_from_args(170.0, 40.0, 3.0, 60.0)
    assert goal["azimuth_from_rad"] == pytest.approx(math.radians(170.0))
    assert goal["azimuth_span_rad"] == pytest.approx(math.radians(40.0))
    assert lv.goal_from_args(270.0, 40.0, 0.0, 1.0)["azimuth_from_rad"] == pytest.approx(
        math.radians(-90.0)
    )


@pytest.mark.parametrize(
    "args",
    [(0, 0, 0, 120), (0, 361, 0, 120), (0, 90, -1, 120), (0, 90, 0, 0)],
)
def test_goal_from_args_rejects(args):
    with pytest.raises(ValueError):
        lv.goal_from_args(*args)


def test_format_result_table():
    p1 = panel()
    p2 = panel(x_m=2.398, y_m=-0.512, yaw_rad=math.radians(179.0), pos_std_m=0.07,
               yaw_std_deg=3.4, lugs=2)
    text = lv.format_result(result([p1, p2], reason="completed"))
    lines = text.splitlines()
    assert "completed" in lines[0]
    rows = [l for l in lines if l.startswith(" 1 ") or l.startswith(" 2 ")]
    assert len(rows) == 2
    bearing = math.degrees(math.atan2(0.310, 2.413))
    distance = math.hypot(2.413, 0.310)
    assert f"{bearing:.1f}" in rows[0]
    assert f"{distance:.2f}" in rows[0]
    assert "178.2" in rows[0]
    assert f"{math.degrees(math.atan2(-0.512, 2.398)):.1f}" in rows[1]
    assert "25.0" in text  # 後片付けの上限


def test_format_result_unknown_yaw():
    p_unknown = panel(yaw_std_deg=90.0, front_known=False)
    p_front = panel(yaw_std_deg=3.0, front_known=True, front_yaw_rad=math.radians(181.5))
    rows = lv.format_result(result([p_unknown, p_front])).splitlines()
    rows = [l for l in rows if l.startswith(" 1 ") or l.startswith(" 2 ")]
    assert "不明" in rows[0]
    assert rows[0].split()[-1] == "-"
    assert "不明" not in rows[1]
    assert rows[1].split()[-1] == "181.5"


def test_format_result_empty():
    text = lv.format_result(result([], reason="no_lugs"))
    assert "no_lugs" in text
    assert lv.REASON_DESCRIPTIONS["no_lugs"] in text
    assert "x[m]" not in text
    unknown = lv.format_result(result([], reason="something_new"))
    assert "something_new" in unknown


def test_format_result_depth_mark():
    unmeasured = panel(depth_m=0.86, depth_measured=False)
    measured = panel(depth_measured=True)
    text = lv.format_result(result([unmeasured, measured]))
    rows = [l for l in text.splitlines() if l.startswith(" 1 ") or l.startswith(" 2 ")]
    assert "0.86*" in rows[0]
    assert "0.60*" not in rows[1]
    assert "*" in text.splitlines()[-2]  # 後片付けの上限の行の前
    assert text.splitlines()[-2] == "* 奥行きは測っていない（既定値）"

    text_all = lv.format_result(result([measured]))
    assert "*" not in text_all


def test_format_seen_ranges():
    text = lv.format_seen_ranges([0.0, math.radians(90)], [math.radians(30), math.radians(45)])
    assert "+0.0°〜+30.0°" in text
    assert "+90.0°〜+135.0°" in text
    assert lv.format_seen_ranges([], []) == "なし"


def test_format_result_v2_reasons():
    for reason in ("ceiling_unknown", "camera_pose_unknown"):
        text = lv.format_result(result([], reason=reason))
        assert reason in text
        assert lv.REASON_DESCRIPTIONS[reason] in text
    assert len(lv.REASON_DESCRIPTIONS) == 11


def test_cleanup_wait_limit_s():
    assert lv.cleanup_wait_limit_s({"cleanup_limit_s": 25.0}, 10.0) == 35.0
    assert lv.cleanup_wait_limit_s({"cleanup_limit_s": 0.0}, 10.0) == 10.0
    assert lv.cleanup_wait_limit_s({"cleanup_limit_s": 60.0}, 0.5) == 60.5


def test_format_cleanup_and_status():
    c = lv.format_cleanup({"reason": "completed", "lift_restored": True})
    assert "completed" in c and "True" in c
    s = lv.format_status({"interface_version": "3", "phase": "IDLE", "last_reject_reason": ""})
    assert "3" in s and "IDLE" in s


def _diff(a, b):
    return np.any(a != b, axis=2)


def test_draw_top_view_axes():
    empty = lv.draw_top_view(result([]), GOAL)
    assert empty.shape == (800, 800, 3)
    front = lv.draw_top_view(result([panel(x_m=3.0, y_m=0.0, yaw_rad=math.pi)]), GOAL)
    d = _diff(empty, front)
    assert d[:400].any()
    assert not d[400:].any()
    cols = np.where(d.any(axis=0))[0]
    assert abs(cols.mean() - 400) < 60
    left = lv.draw_top_view(result([panel(x_m=0.0, y_m=3.0, yaw_rad=0.0)]), GOAL)
    dl = _diff(empty, left)
    assert dl[:, :400].any()
    assert not dl[:, 400:].any()


def test_draw_top_view_base_edge():
    empty = lv.draw_top_view(result([]), GOAL)
    p = panel(x_m=3.0, y_m=0.0, yaw_rad=0.0, width_m=0.8, depth_m=0.6)

    def counts(yaw):
        img = lv.draw_top_view(result([dict(p, yaw_rad=yaw)]), GOAL)
        d = _diff(empty, img)
        scale = 400.0 / 5.5
        # 基準の辺（+n 側）の中点と、反対の辺の中点
        out = []
        for sign in (+1, -1):
            x = 3.0 + sign * 0.3 * math.cos(yaw)
            y = 0.0 + sign * 0.3 * math.sin(yaw)
            col = int(round(400 - y * scale))
            row = int(round(400 - x * scale))
            out.append(int(d[row - 4:row + 5, col - 4:col + 5].sum()))
        return out

    base0, other0 = counts(0.0)
    assert base0 > other0
    base1, other1 = counts(math.pi)
    assert base1 > other1


def test_result_to_dict_keeps_values():
    p = panel(x_m=2.413123456789, yaw_rad=3.1100000001)
    r = result([p])
    record = lv.result_to_dict(GOAL, r, {"reason": "completed", "lift_restored": True}, 123.5)
    assert record["result"]["panels"][0]["x_m"] == 2.413123456789
    assert record["result"]["panels"][0]["yaw_rad"] == 3.1100000001
    assert record["result"]["reason"] == "completed"
    assert record["cleanup_done"]["lift_restored"] is True
    assert record["goal"]["azimuth_span_rad"] == pytest.approx(2 * math.pi)
    assert record["accepted_stamp"] == 123.5

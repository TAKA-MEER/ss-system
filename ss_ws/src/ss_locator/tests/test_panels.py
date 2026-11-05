import math

import numpy as np
import pytest

from ss_locator import assemble as A
from ss_locator import panels as P
from ss_locator import scan_plan as S

LUG_INSET = A.LUG_INSET


def rect_lugs(center, phi, width, depth, s_pos=0.01):
    """中心・向き・寸法から、長方形の四隅に置いた金具を作る（fit に渡す用）。"""
    cx, cy = center
    u = np.array([math.cos(phi), math.sin(phi)])
    v = np.array([-math.sin(phi), math.cos(phi)])
    pts = [np.array([cx, cy]) + sx * width / 2.0 * u + sy * depth / 2.0 * v
           for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return [{"p": p, "axis": math.degrees(phi) % 180.0, "s_pos": s_pos,
             "s_axis": 1.0, "conf": 1.0} for p in pts]


def base_edge_length(center, phi, width, depth):
    """機体側の辺の長さ（LUG_INSET 込み）。実装とは独立に幾何から求める。"""
    cx, cy = center
    to_origin = math.atan2(-cy, -cx)
    sides = [(phi, depth), (phi + math.pi, depth),
             (phi + math.pi / 2.0, width), (phi - math.pi / 2.0, width)]
    _, length = min(sides, key=lambda nd: abs(S.normalize(nd[0] - to_origin)))
    return length + 2 * LUG_INSET


def test_base_edge_faces_robot():
    for center in ((3.0, 0.0), (0.0, 3.0), (-3.0, 0.0)):
        for phi_deg in (0.0, 30.0, 90.0):
            phi = math.radians(phi_deg)
            lugs = rect_lugs(center, phi, 1.2, 0.8)
            panel = A.fit(lugs, [0, 1, 2, 3])
            top = P.to_panel_top(panel, lugs, 2.35)

            cx, cy = center
            to_origin = math.atan2(-cy, -cx)
            assert abs(S.normalize(top["yaw_rad"] - to_origin)) <= math.radians(45.0), \
                f"center={center} phi={phi_deg}: yaw_rad が機体の向きから 45° を超える"

            # width_m は基準の辺（機体側の辺）の長さ
            assert top["width_m"] == pytest.approx(base_edge_length(center, phi, 1.2, 0.8)), \
                f"center={center} phi={phi_deg}: width_m が基準の辺の長さでない"
            got = sorted([top["width_m"], top["depth_m"]])
            want = sorted([1.2 + 2 * LUG_INSET, 0.8 + 2 * LUG_INSET])
            assert got == pytest.approx(want)


def test_width_depth_swap():
    center = (3.0, 0.0)
    inset = 2 * LUG_INSET

    # 長い辺が機体側を向く: 基準の辺は fit の width
    lugs_a = rect_lugs(center, math.radians(90.0), 1.2, 0.8)
    panel_a = A.fit(lugs_a, [0, 1, 2, 3])
    top_a = P.to_panel_top(panel_a, lugs_a, 2.35)
    assert top_a["width_m"] == pytest.approx(1.2 + inset)
    assert top_a["depth_m"] == pytest.approx(0.8 + inset)

    # 同じ矩形を 90° 回す。機体側の辺は短い辺になるが、その長さが幅のまま
    lugs_b = rect_lugs(center, 0.0, 1.2, 0.8)
    panel_b = A.fit(lugs_b, [0, 1, 2, 3])
    top_b = P.to_panel_top(panel_b, lugs_b, 2.35)
    assert top_b["width_m"] == pytest.approx(0.8 + inset)
    assert top_b["depth_m"] == pytest.approx(1.2 + inset)
    # fit の width（長い辺）をそのまま返していない
    assert top_b["width_m"] != pytest.approx(panel_b["width"])

    # 同じ物理的な矩形のまま、fit の幅・奥行の呼び方だけ 90° 回しても、
    # 機体側の辺の長さが幅のまま（入れ替わらない）
    panel_c = dict(panel_a)
    panel_c.update(front_yaw=(panel_a["front_yaw"] - 90.0) % 360.0,
                   width=panel_a["depth"], depth=panel_a["width"])
    top_c = P.to_panel_top(panel_c, lugs_a, 2.35)
    assert top_c["width_m"] == pytest.approx(top_a["width_m"])
    assert top_c["depth_m"] == pytest.approx(top_a["depth_m"])


def top_at(x, y):
    return {"x_m": x, "y_m": y}


def test_filter_range_distance():
    tops = [top_at(1, 1), top_at(-1, 1), top_at(-1, -1), top_at(1, -1), top_at(10, 0)]

    # 0° から反時計回りに 180°
    got = P.filter_panels(tops, 0.0, math.pi, 5.0)
    assert got == [tops[0], tops[1]]

    # 0 をまたぐ範囲（170° から 40°）と距離
    around = [top_at(-4, 0.1), top_at(-4, -0.5), top_at(4, 0.1), top_at(0, -4)]
    got = P.filter_panels(around, math.radians(170), math.radians(40), 6.0)
    assert got == [around[0], around[1]]

    # 全周はすべて通す（距離以内）
    got = P.filter_panels(tops[:-1], 0.0, 2 * math.pi, 5.0)
    assert got == tops[:-1]

    # 距離は「以下」（ちょうどは通す）
    assert P.filter_panels([top_at(0, 5)], 0.0, math.pi, 5.0) == [top_at(0, 5)]


def test_no_lugs_vs_ungrouped():
    assert P.finish_reason(0, 0) == "no_lugs"
    assert P.finish_reason(1, 0) == "lugs_ungrouped"
    assert P.finish_reason(3, 0) == "lugs_ungrouped"
    assert P.finish_reason(0, 1) == "completed"
    assert P.finish_reason(4, 2) == "completed"

    lugs = [{"p": np.array([1.0, 0.0])}, {"p": np.array([0.0, 1.0])},
            {"p": np.array([-1.0, 0.0])}, {"p": np.array([10.0, 0.0])}]
    # 範囲 0〜90°・距離 2 m 以内は 2 個
    n = P.count_lugs_in_range(lugs, 0.0, math.pi / 2.0, 2.0)
    assert n == 2
    assert P.finish_reason(n, 0) == "lugs_ungrouped"

    # 範囲 100〜170°・距離 2 m 以内は 0 個
    n = P.count_lugs_in_range(lugs, math.radians(100.0), math.radians(70.0), 2.0)
    assert n == 0
    assert P.finish_reason(n, 0) == "no_lugs"


def test_diagonal_pair():
    lugs = [
        {"p": np.array([-0.5, -0.5]), "axis": 45.0, "s_pos": 0.01, "s_axis": 1.0, "conf": 1.0},
        {"p": np.array([0.5, 0.5]), "axis": 45.0, "s_pos": 0.01, "s_axis": 1.0, "conf": 1.0},
    ]
    panel = A.fit(lugs, [0, 1])
    assert panel.get("diagonal", False)

    top = P.to_panel_top(panel, lugs, 2.35)
    assert top["yaw_std_deg"] == pytest.approx(90.0)
    assert top["front_known"] is False
    assert top["front_yaw_rad"] == 0.0
    assert top["lugs"] == 2
    assert (top["x_m"], top["y_m"]) == pytest.approx((0.0, 0.0))
    # 2 個の距離 ÷ √2
    assert top["width_m"] == pytest.approx(1.0)
    assert top["depth_m"] == pytest.approx(top["width_m"])

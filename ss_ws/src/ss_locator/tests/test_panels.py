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


def _front_pair(s_axis):
    """背中合わせの盤の正面の 2 個。リング面は中心 (0, 3.4) を通る。"""
    return [
        {"p": np.array([-0.4, 3.0]), "axis": 45.0, "s_pos": 0.01, "s_axis": s_axis, "conf": 1.0},
        {"p": np.array([0.4, 3.0]), "axis": 135.0, "s_pos": 0.01, "s_axis": s_axis, "conf": 1.0},
    ]


def test_pos_std_two_lugs_adds_axis_term():
    """金具 2 個で正面が分かる盤は、r̄ × radians(s̄_axis) を位置の標準偏差に足す（-core.md §4）。"""
    lugs1 = _front_pair(4.0)
    lugs0 = _front_pair(1e-6)
    p1 = A.fit(lugs1, [0, 1])
    p0 = A.fit(lugs0, [0, 1])
    assert p1["front_known"] and not p1.get("diagonal", False)
    np.testing.assert_allclose(p1["center"], p0["center"], atol=1e-6)

    top1 = P.to_panel_top(p1, lugs1, 2.35)
    top0 = P.to_panel_top(p0, lugs0, 2.35)
    r_mean = np.mean([np.linalg.norm(np.asarray(p1["center"]) - L["p"]) for L in lugs1])
    a = r_mean * math.radians(4.0)
    assert top1["pos_std_m"] ** 2 - top0["pos_std_m"] ** 2 == pytest.approx(a * a, rel=1e-3)


def _lug_toward(p, target, s_pos=0.01, s_axis=1.0, conf=1.0):
    axis = math.degrees(math.atan2(float(target[1] - p[1]), float(target[0] - p[0]))) % 180.0
    return {"p": np.asarray(p, float), "axis": axis, "s_pos": s_pos,
            "s_axis": s_axis, "conf": conf}


def _far_pair(pa, pb, far=True, **kwargs):
    """2 個の中点から向こう側（far）／手前側へ向かう向きに輪の向きを合わせた 2 個。"""
    pa, pb = np.asarray(pa, float), np.asarray(pb, float)
    mid = (pa + pb) / 2.0
    e = pb - pa
    e = e / np.linalg.norm(e)
    n = np.array([-e[1], e[0]])
    if float(mid @ n) <= 0.0:
        n = -n
    target = mid + (0.3 if far else -0.3) * n
    return [_lug_toward(pa, target, **kwargs), _lug_toward(pb, target, **kwargs)]


PARAMS = dict(spacing_min_m=0.5, spacing_max_m=0.8, depth_default_m=0.86,
              depth_std_m=0.046, depth_min_m=0.70, depth_max_m=1.02,
              gap_min_m=0.05, gap_max_m=0.30, merge_tol_m=0.10,
              angle_tol_deg=10.0, overlap_ratio=0.5)


def _tops(raw, lugs, **overrides):
    values = dict(PARAMS)
    values.update(overrides)
    return P.make_panel_tops(raw, lugs, 2.35, **values)


def _pair_at(x, y_lo=-0.265, y_hi=0.265, target=None, **kwargs):
    """x=const の列に y_lo/y_hi の 2 個。輪の向きは target へ向く。"""
    pa, pb = np.array([x, y_lo], float), np.array([x, y_hi], float)
    if target is None:
        target = (x + 0.3, 0.0)
    return [_lug_toward(pa, target, **kwargs), _lug_toward(pb, target, **kwargs)]


def _rect_panel(lugs_all, center, phi, width, depth, s_pos=0.01):
    """矩形の 4 個を lugs_all に足し、`assemble.fit` の盤を返す。"""
    local = rect_lugs(center, phi, width, depth, s_pos=s_pos)
    base = len(lugs_all)
    lugs_all.extend(local)
    panel = A.fit(local, [0, 1, 2, 3])
    panel = dict(panel)
    panel["lugs"] = [base + i for i in panel["lugs"]]
    return panel


# 盤の幾何（x が前後・y が左右。LUG_INSET 込みの盤の寸法は lug 間隔 + 2*LUG_INSET）
# 面 A（手前）: 中心 x=2.0、lug の前後 0.765（盤の奥行き 0.9）。継ぎ目の列 x=2.3825
# 面 B（奥）: 継ぎ目の列 x=2.5175、遠い列 x=3.2825、中心 x=2.9
A_CENTER = (2.0, 0.0)
B_CENTER = (2.9, 0.0)
LUG_W = 0.53
LUG_D = 0.765


def test_two_lug_near_edge():
    lugs = _pair_at(2.0)  # 輪は向こう側（+x）を向く
    tops = _tops([], lugs)
    assert len(tops) == 1
    top = tops[0]
    assert (top["x_m"], top["y_m"]) == pytest.approx((2.0 + 0.86 / 2.0 - LUG_INSET, 0.0))
    assert top["width_m"] == pytest.approx(0.53 + 2 * LUG_INSET)
    assert top["depth_m"] == pytest.approx(0.86)
    assert top["depth_measured"] is False
    assert top["lugs"] == 2
    assert top["front_known"] is False


def test_two_lug_far_edge():
    lugs = _pair_at(2.0, target=(1.7, 0.0))  # 輪は手前（-x）を向く
    tops = _tops([], lugs)
    assert len(tops) == 1
    top = tops[0]
    assert top["x_m"] == pytest.approx(2.0 - (0.86 / 2.0 - LUG_INSET))
    assert top["x_m"] < 2.0
    assert top["depth_m"] == pytest.approx(0.86)
    assert top["depth_measured"] is False
    assert top["lugs"] == 2


@pytest.mark.parametrize("spacing,expected", [(0.45, 0), (0.85, 0), (0.5, 1), (0.8, 1)])
def test_two_lug_spacing_gate(spacing, expected):
    lugs = _far_pair((2.0, -spacing / 2.0), (2.0, spacing / 2.0))
    assert len(_tops([], lugs)) == expected


def test_two_lug_skewed_rings():
    shifted = [
        {"p": np.array([1.751, 1.370]), "axis": 40.0, "s_pos": 0.003, "s_axis": 4.0, "conf": 0.94},
        {"p": np.array([1.834, 0.847]), "axis": 63.0, "s_pos": 0.003, "s_axis": 3.1, "conf": 0.87},
    ]
    tops = _tops([], shifted)
    assert len(tops) == 1
    assert (tops[0]["x_m"], tops[0]["y_m"]) == pytest.approx((2.151, 1.165), abs=0.01)
    assert tops[0]["depth_m"] == pytest.approx(0.86)


def test_two_lug_real_mock():
    lugs = [
        {"p": np.array([1.751, 1.370]), "axis": 139.0, "s_pos": 0.003, "s_axis": 4.0, "conf": 0.94},
        {"p": np.array([1.834, 0.847]), "axis": 36.0, "s_pos": 0.003, "s_axis": 3.1, "conf": 0.87},
    ]
    tops = _tops([], lugs)
    assert len(tops) == 1
    top = tops[0]
    assert top["width_m"] == pytest.approx(0.665, abs=0.005)
    assert top["depth_m"] == pytest.approx(0.86)
    assert (top["x_m"], top["y_m"]) == pytest.approx((2.151, 1.165), abs=0.01)
    assert top["lugs"] == 2
    assert top["front_known"] is False
    assert top["depth_measured"] is False


def test_back_to_back_rect_rect():
    lugs = []
    raw = [_rect_panel(lugs, A_CENTER, 0.0, LUG_D, LUG_W),
           _rect_panel(lugs, B_CENTER, 0.0, LUG_D, LUG_W)]
    tops = _tops(raw, lugs)
    assert len(tops) == 1
    top = tops[0]
    assert top["depth_m"] == pytest.approx(1.8, abs=0.02)
    assert top["lugs"] == 8
    assert top["depth_measured"] is True
    assert top["x_m"] == pytest.approx(2.45, abs=0.02)


def test_back_to_back_pair_pair_outer():
    lugs = _pair_at(1.6175, target=(2.0, 0.0)) + _pair_at(3.2825, target=(2.9, 0.0))
    tops = _tops([], lugs)
    assert len(tops) == 1
    top = tops[0]
    assert top["depth_m"] == pytest.approx(1.8, abs=0.02)
    assert top["depth_measured"] is True
    assert top["lugs"] == 4


def test_single_face_from_two_pairs():
    lugs = _pair_at(1.6175, target=(2.0, 0.0)) + _pair_at(2.3825, target=(2.0, 0.0))
    tops = _tops([], lugs)
    assert len(tops) == 1
    top = tops[0]
    assert top["depth_m"] == pytest.approx(0.9, abs=0.02)
    assert top["depth_measured"] is True
    assert top["lugs"] == 4


def test_back_to_back_one_full():
    lugs = []
    raw = [_rect_panel(lugs, A_CENTER, 0.0, LUG_D, LUG_W)]
    lugs += _pair_at(3.2825, target=(2.9, 0.0))
    tops = _tops(raw, lugs)
    assert len(tops) == 1
    assert tops[0]["depth_measured"] is True
    assert tops[0]["lugs"] == 6


def test_back_to_back_seam_only():
    lugs = _pair_at(2.3825, target=(2.0, 0.0)) + _pair_at(2.5175, target=(2.9, 0.0))
    tops = _tops([], lugs)
    assert len(tops) == 1
    top = tops[0]
    assert top["depth_m"] == pytest.approx(2 * 0.86, abs=0.02)
    assert top["depth_measured"] is False
    # 測っていない一番外側の列 2 本ぶんのばらつき
    assert top["pos_std_m"] >= math.sqrt(2) * 0.046 - 1e-9


def test_back_to_back_rect_pair():
    lugs = []
    raw = [_rect_panel(lugs, A_CENTER, 0.0, LUG_D, LUG_W)]
    lugs += _pair_at(2.5175, target=(2.9, 0.0))
    tops = _tops(raw, lugs)
    assert len(tops) == 1
    assert tops[0]["depth_measured"] is False
    assert tops[0]["lugs"] == 6


def test_seam_diagonal_four():
    # 継ぎ目の 4 個。斜めの組の確信度を高くしても、列ごとの組が選ばれる
    lugs = _pair_at(2.3825, target=(2.0, 0.0), conf=0.1) + _pair_at(2.5175, target=(2.9, 0.0), conf=0.1)
    lugs[0]["conf"] = 0.9
    lugs[3]["conf"] = 0.9
    tops = _tops([], lugs)
    assert len(tops) == 1
    assert tops[0]["depth_m"] == pytest.approx(2 * 0.86, abs=0.02)
    assert tops[0]["lugs"] == 4


def test_seam_diagonal_three():
    # 継ぎ目の 3 個。斜めの組は捨てられ、直角の組（片方の列の 2 個）が選ばれる
    lugs = _pair_at(2.3825, target=(2.0, 0.0))
    lugs.append(_lug_toward((2.5175, 0.265), (2.9, 0.265)))
    # 斜めの組（lugs[0] と lugs[2]）の確信度を高くする（①だけでは拾えない）
    lugs[0]["conf"] = 0.9
    lugs[2]["conf"] = 0.9
    lugs[1]["conf"] = 0.1
    tops = _tops([], lugs)
    assert len(tops) == 1
    assert tops[0]["lugs"] == 2
    assert tops[0]["depth_m"] == pytest.approx(0.86)
    # 直角の組（片方の列の 2 個）の中心。斜めの組なら (2.10, 0.09) になる
    assert (tops[0]["x_m"], tops[0]["y_m"]) == pytest.approx((2.02, 0.0), abs=0.02)


def test_back_to_back_gates():
    # 向きの差 15°
    lugs = []
    raw = [_rect_panel(lugs, A_CENTER, 0.0, LUG_D, LUG_W),
           _rect_panel(lugs, B_CENTER, math.radians(15.0), LUG_D, LUG_W)]
    assert len(_tops(raw, lugs)) == 2
    # 継ぎ目 0.4 m
    lugs = []
    raw = [_rect_panel(lugs, A_CENTER, 0.0, LUG_D, LUG_W),
           _rect_panel(lugs, (3.165, 0.0), 0.0, LUG_D, LUG_W)]
    assert len(_tops(raw, lugs)) == 2
    # 重なり 30%
    lugs = []
    raw = [_rect_panel(lugs, A_CENTER, 0.0, LUG_D, LUG_W),
           _rect_panel(lugs, (2.9, 0.371), 0.0, LUG_D, LUG_W)]
    assert len(_tops(raw, lugs)) == 2
    # k=2 で測った列の間 1.1 m（1 面にも 2 面にも合わない）
    lugs = _pair_at(1.6175, target=(2.0, 0.0)) + _pair_at(2.7175, target=(2.5, 0.0))
    assert len(_tops([], lugs)) == 2


def test_single_rect_unchanged():
    lugs = rect_lugs((0.0, 3.0), 0.0, 1.0, 0.6)
    panel = A.fit(lugs, [0, 1, 2, 3])
    want = P.to_panel_top(panel, lugs, 2.35)
    tops = _tops([panel], lugs)
    assert len(tops) == 1
    got = tops[0]
    for key in ("x_m", "y_m", "yaw_rad", "width_m", "depth_m", "pos_std_m", "yaw_std_deg"):
        assert got[key] == pytest.approx(want[key], abs=1e-9), key
    assert got["depth_measured"] is True


def test_side_by_side_row():
    # 横に並んだ盤 3 面の手前の列だけ（6 個）。金具ごとに標準偏差 1 cm の誤差を
    # 足して 200 回（乱数の種を決める）→ 毎回 3 面
    base = []
    for panel in range(3):
        y0 = panel * 0.735
        base += _pair_at(2.0, y_lo=y0, y_hi=y0 + 0.6, target=(2.3, y0 + 0.3))
    rng = np.random.default_rng(12)
    for _ in range(200):
        lugs = []
        for lug in base:
            noisy = dict(lug)
            noisy["p"] = np.asarray(lug["p"], float) + rng.normal(0.0, 0.01, size=2)
            lugs.append(noisy)
        assert len(_tops([], lugs)) == 3


def test_two_lug_between():
    lugs = _far_pair((2.0, -0.265), (2.0, 0.265))
    lugs.append(_lug_toward((2.0, 0.0), (2.3, 0.0)))
    assert _tops([], lugs) == []


def test_two_lug_pick_by_conf():
    target = (2.3, 0.55)
    lugs = [_lug_toward((2.0, 0.0), target, conf=0.9),
            _lug_toward((2.0, 0.55), target, conf=0.9),
            _lug_toward((2.0, 1.1), target, conf=0.1)]
    tops = _tops([], lugs)
    assert len(tops) == 1
    assert tops[0]["y_m"] == pytest.approx(0.275)


def test_pos_std_four_lugs_has_no_axis_term():
    """金具 4 個の盤には足さない（リング面の向きのばらつきで位置の標準偏差が変わらない）。"""
    tops = []
    for s_axis in (1.0, 4.0):
        lugs = rect_lugs((0.0, 3.0), 0.0, 1.0, 0.6)
        for L in lugs:
            L["s_axis"] = s_axis
        panel = A.fit(lugs, [0, 1, 2, 3])
        tops.append(P.to_panel_top(panel, lugs, 2.35))
    assert tops[0]["pos_std_m"] == pytest.approx(tops[1]["pos_std_m"])

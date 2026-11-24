import math

import pytest

from ss_locator import scan_plan as S

TAU = 2.0 * math.pi
D = math.radians


def test_plan_yaws():
    hfov = D(60.0)
    overlap = 0.1
    step = hfov * (1.0 - overlap)

    # 幅が画角以下なら 1 枚（範囲の中央）
    got = S.plan_yaws(D(10.0), D(30.0), hfov, overlap)
    assert got == [pytest.approx(S.normalize(D(25.0)))]

    # 幅 90°・画角 60°・重なり 10%: 両端が範囲に合い、間隔は step 以下
    start = D(0.0)
    span = D(90.0)
    got = S.plan_yaws(start, span, hfov, overlap)
    assert len(got) >= 2
    assert S.normalize(got[0] - hfov / 2.0 - start) == pytest.approx(0.0, abs=1e-9)
    assert S.normalize(got[-1] + hfov / 2.0 - (start + span)) == pytest.approx(0.0, abs=1e-9)
    assert S.normalize(got[1] - got[0]) > 0.0
    for a, b in zip(got, got[1:]):
        assert 0.0 < S.normalize(b - a) <= step + 1e-9

    # 全周: ceil(2π/step) 枚を開始の向きから等間隔
    got = S.plan_yaws(D(20.0), TAU, hfov, overlap)
    n = int(math.ceil(TAU / step))
    assert len(got) == n
    assert got[0] == pytest.approx(S.normalize(D(20.0)))
    diffs = [S.normalize(b - a) for a, b in zip(got, got[1:])]
    assert all(d == pytest.approx(TAU / n) for d in diffs)

    # 値は −π〜π
    assert all(-math.pi <= a < math.pi for a in got)


def test_hfov_from_fx():
    assert S.hfov_from_fx(640.0) == pytest.approx(math.pi / 2.0)
    assert S.hfov_from_fx(640.0, width_px=1280) == pytest.approx(math.pi / 2.0)


def test_vfov_from_fy():
    assert S.vfov_from_fy(360.0) == pytest.approx(math.pi / 2.0)
    assert S.vfov_from_fy(360.0, height_px=720) == pytest.approx(math.pi / 2.0)


def test_plan_pitches_real_camera():
    # fy 1482.66（実機。WP-CAL-01）・上 20°・下端 70°・重なり 0.1 → 3 段
    vfov = S.vfov_from_fy(1482.66)
    got = S.plan_pitches(D(20.0), D(70.0), vfov, 0.1)
    assert len(got) == 3
    assert got[0] == pytest.approx(D(20.0))
    assert got[1] == pytest.approx(D(38.2), abs=D(0.05))
    assert got[2] == pytest.approx(D(56.35), abs=D(0.05))
    # 隣の段の縦の画角が重なり、一番深い段の下端がちょうど下端 70°
    assert got[-1] + vfov / 2.0 == pytest.approx(D(70.0))
    for a, b in zip(got, got[1:]):
        assert b - a <= vfov * 0.9 + 1e-9


def test_plan_pitches_sim_camera():
    # fy 1082（合成データ）・上 20°・下端 70°・重なり 0.1 → 2 段
    vfov = S.vfov_from_fy(1082.0)
    got = S.plan_pitches(D(20.0), D(70.0), vfov, 0.1)
    assert len(got) == 2
    assert got[0] == pytest.approx(D(20.0))
    assert got[1] == pytest.approx(D(51.6), abs=D(0.05))


def test_plan_pitches_single():
    # vfov 0 → 1 段だけ
    assert S.plan_pitches(D(20.0), D(70.0), 0.0, 0.1) == [pytest.approx(D(20.0))]
    # 下端 58°・fy 1482.66 → 2 段（20°・約 44.35°）
    vfov = S.vfov_from_fy(1482.66)
    got = S.plan_pitches(D(20.0), D(58.0), vfov, 0.1)
    assert len(got) == 2
    assert got[0] == pytest.approx(D(20.0))
    assert got[1] == pytest.approx(D(44.35), abs=D(0.05))
    # 下端が浅すぎて一番深い段が上以下 → 1 段
    assert S.plan_pitches(D(20.0), D(30.0), vfov, 0.1) == [pytest.approx(D(20.0))]


def test_plan_shots_serpentine():
    a, b, c = D(10.0), D(20.0), D(30.0)
    p1, p2 = D(20.0), D(50.0)
    got = S.plan_shots([a, b, c], [p1, p2])
    assert got == [(a, p1), (b, p1), (c, p1), (c, p2), (b, p2), (a, p2)]
    pitches = [p for _, p in got]
    assert pitches == sorted(pitches)  # ピッチは単調に深くなる


def test_seen_ranges():
    hfov = D(60.0)

    # 1 枚だけ撮った: 依頼の範囲で切る
    got = S.seen_ranges([D(30.0)], hfov, D(0.0), D(90.0))
    assert got == ([pytest.approx(0.0)], [pytest.approx(D(60.0))])

    # 2 枚が重なって依頼の範囲を全部見た
    got = S.seen_ranges([D(30.0), D(60.0)], hfov, D(0.0), D(90.0))
    assert got == ([pytest.approx(0.0)], [pytest.approx(D(90.0))])

    # 0 をまたぐ範囲
    got = S.seen_ranges([D(-150.0)], hfov, D(-170.0), D(40.0))
    assert got == ([pytest.approx(S.normalize(D(-170.0)))], [pytest.approx(D(40.0))])

    # 全周を全部見た
    yaws = [D(i * 60.0) for i in range(6)]
    got = S.seen_ranges(yaws, hfov, D(10.0), TAU)
    assert got == ([pytest.approx(S.normalize(D(10.0)))], [pytest.approx(TAU)])

    # 重なり・接しているものをまとめる（依頼の範囲では切る）
    got = S.seen_ranges([D(0.0), D(60.0)], hfov, D(-60.0), D(200.0))
    assert got == ([pytest.approx(S.normalize(D(-30.0)))], [pytest.approx(D(120.0))])

    # 撮っていなければ空
    assert S.seen_ranges([], hfov, D(0.0), D(90.0)) == ([], [])

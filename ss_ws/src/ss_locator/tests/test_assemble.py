import json
import math
from pathlib import Path

import numpy as np
import pytest

from ss_locator import assemble as A
from ss_locator import geometry as G
from ss_locator import panels as P

FIXTURE = Path(__file__).with_name("fixtures") / "proto_9265d2d.json"
FK_SHOTS = Path(__file__).with_name("fixtures") / "fk_sim_20261126_shots.jsonl"


def load_fixture():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def cam_of(view):
    return np.array(view["K"], float), np.array(view["dist"], float), np.array(view["T"], float)


def test_assemble_matches_proto():
    fixture = load_fixture()
    top = fixture["top_height_m"]
    for scene in fixture["scenes"]:
        obs = []
        for view in scene["views"]:
            obs += A.lift(np.array(view["kp"], float), np.array(view["conf"], float),
                          cam_of(view), view["view"], top)
        fused = A.fuse(obs)
        want_count = sum(panel["n_lugs"] for panel in scene["panels"])
        assert len(fused) >= want_count, f"scene {scene['scene']}: fused lug count"

        # The proto fixture has fitted geometry for complete rectangles. Rebuild
        # their four expected lug locations and match those against fused points;
        # grouping itself intentionally no longer has to match the old proto.
        expected = []
        inset = A.LUG_INSET
        for panel in scene["panels"]:
            if panel["n_lugs"] != 4:
                continue
            front = math.radians(panel["front_yaw"])
            width_axis = np.array([-math.sin(front), math.cos(front)])
            depth_axis = np.array([math.cos(front), math.sin(front)])
            center = np.asarray(panel["center"], float)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    expected.append(center + sx * (panel["width"] / 2 - inset) * width_axis
                                    + sy * (panel["depth"] / 2 - inset) * depth_axis)
        remaining = [np.asarray(lug["p"], float) for lug in fused]
        for point in expected:
            distances = [np.linalg.norm(candidate - point) for candidate in remaining]
            nearest = int(np.argmin(distances))
            assert distances[nearest] < 0.07, f"scene {scene['scene']}: fused lug position"
            remaining.pop(nearest)


def _fk_cam(record):
    return G.camera_T(
        record["camera_height_m"], record["yaw_rad"], record["pitch_down_rad"],
        0.0, 0.0, 0.08, optical_offset_m=0.06,
        pitch_offset_rad=math.radians(record["camera_pitch_offset_deg"]),
        yaw_offset_rad=math.radians(record["camera_yaw_offset_deg"]),
        roll_rad=math.radians(record["camera_roll_deg"]),
    )


def _fk_replay(records, run_index):
    K = np.array([1482.80, 0.0, 638.13, 0.0, 1482.66, 367.04, 0.0, 0.0, 1.0]).reshape(3, 3)
    dist = np.array([0.02116, -0.27909, 0.0, 0.0, 0.0])
    obs = []
    for shot_index, record in enumerate(records):
        if not record["keypoints"]:
            continue
        camera = (K, dist, _fk_cam(record))
        obs.extend(A.lift(
            np.asarray(record["keypoints"], float), np.asarray(record["confidences"], float),
            camera, run_index * 15 + shot_index + 1, record["top_height_default_m"],
        ))
    raw, lugs = A.assemble(obs)
    tops = P.make_panel_tops(raw, lugs, 2.35, 0.5, 0.8, 0.86, 0.046,
                             0.70, 1.02, 0.05, 0.30, 0.10, 10.0, 0.5)
    return raw, lugs, tops


def test_fk_sim_shots_group_four():
    records = [json.loads(line) for line in FK_SHOTS.read_text(encoding="utf-8").splitlines()]
    assert len(records) == 19 * 15
    for run_index in range(19):
        raw, lugs, tops = _fk_replay(records[run_index * 15:(run_index + 1) * 15], run_index)
        assert len(raw) == 1, f"run {run_index}: raw panels"
        assert [len(panel["lugs"]) for panel in raw] == [4], f"run {run_index}: grouping"
        assert len(tops) == 1, f"run {run_index}: panel tops"
        top = tops[0]
        assert (top["x_m"], top["y_m"]) == pytest.approx((1.488, 0.0), abs=0.02)
        assert top["width_m"] == pytest.approx(1.0, abs=0.03)
        assert top["depth_m"] == pytest.approx(1.2, abs=0.03)


def test_top_height_param():
    fixture = load_fixture()
    view = fixture["scenes"][0]["views"][0]
    K, dist, T = cam_of(view)
    kp = np.array(view["kp"], float)
    conf = np.array(view["conf"], float)

    hi = A.lift(kp, conf, (K, dist, T), view["view"], 2.35)
    lo = A.lift(kp, conf, (K, dist, T), view["view"], 2.0)
    assert hi and lo
    assert len(hi) == len(lo)

    # 2.0 で戻した根元は、z = 2.0 の平面に乗る（元の画素へ戻る）
    for o in lo:
        uv, ok = G.project(np.array([[o["p"][0], o["p"][1], 2.0]]), K, dist, T)
        assert ok[0]
        assert np.min(np.linalg.norm(kp[:, 0, :2] - uv[0], axis=1)) < 0.5

    # 高さを変えると根元の位置が変わる
    assert max(np.linalg.norm(a["p"] - b["p"]) for a, b in zip(hi, lo)) > 0.01


def _position_lug(x, y, axis=0.0):
    return {"p": np.array([x, y], float), "axis": axis,
            "s_axis": 1.0, "s_pos": 0.01, "conf": 1.0}


def _rectangle_lugs(cx=0.0, cy=0.0, width=0.6, depth=0.6, axes=None):
    points = [(cx - width / 2, cy - depth / 2), (cx + width / 2, cy - depth / 2),
              (cx + width / 2, cy + depth / 2), (cx - width / 2, cy + depth / 2)]
    axes = axes or [0.0] * 4
    return [_position_lug(x, y, axis) for (x, y), axis in zip(points, axes)]


def test_group_ignores_ring_axis():
    lugs = _rectangle_lugs(axes=[7.0, 43.0, 91.0, 158.0])
    # A competing valid 3-lug rectangle shares one corner with the 4-lug one.
    # The 4-lug candidate must win even though the 3-lug candidate is exact too.
    lugs.extend([_position_lug(-0.6, 0.0, 119.0), _position_lug(0.0, -0.6, 11.0)])
    assert A.group(lugs) == [[0, 1, 2, 3]]


def test_group_three_lugs():
    right = [_position_lug(0.0, 0.0), _position_lug(0.6, 0.0), _position_lug(0.0, 0.6)]
    assert A.group(right) == [[0, 1, 2]]

    cosine = 0.2
    skewed = [_position_lug(0.0, 0.0), _position_lug(0.6, 0.0),
              _position_lug(0.6 * cosine, 0.6 * math.sqrt(1 - cosine ** 2))]
    assert A.group(skewed) == []


def test_group_side_range():
    too_close = [_position_lug(0.0, 0.0), _position_lug(0.6, 0.0),
                 _position_lug(0.0, 0.135), _position_lug(0.6, 0.135)]
    assert A.group(too_close) == []
    too_wide = _rectangle_lugs(width=1.2, depth=0.6)
    assert A.group(too_wide) == []


def test_group_row_of_panels():
    lugs = []
    for panel in range(3):
        lugs.extend(_rectangle_lugs(cy=panel * 0.62))
    groups = A.group(lugs)
    assert len(groups) == 3
    assert all(len(group) == 4 for group in groups)
    assert {tuple(group) for group in groups} == {
        (0, 1, 2, 3), (4, 5, 6, 7), (8, 9, 10, 11),
    }


def test_group_duplicate_lug():
    lugs = _rectangle_lugs()
    lugs.append(_position_lug(-0.46, -0.45))  # 4 cm from the first corner
    assert A.group(lugs) == [[0, 1, 2, 3]]


def test_group_rejects_interior_lug():
    lugs = _rectangle_lugs()
    lugs.append(_position_lug(0.0, 0.0))
    assert [0, 1, 2, 3] not in A.group(lugs)


def test_group_speed():
    import time

    records = [json.loads(line) for line in FK_SHOTS.read_text(encoding="utf-8").splitlines()]
    _raw, lugs, _tops = _fk_replay(records[7 * 15:8 * 15], 7)
    started = time.perf_counter()
    A.group(lugs)
    assert time.perf_counter() - started < 1.0

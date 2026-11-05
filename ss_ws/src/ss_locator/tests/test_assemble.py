import json
import math
from pathlib import Path

import numpy as np

from ss_locator import assemble as A
from ss_locator import geometry as G

FIXTURE = Path(__file__).with_name("fixtures") / "proto_9265d2d.json"


def load_fixture():
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def cam_of(view):
    return np.array(view["K"], float), np.array(view["dist"], float), np.array(view["T"], float)


def test_assemble_matches_proto():
    fixture = load_fixture()
    top = fixture["top_height_m"]
    n_panels = 0
    for scene in fixture["scenes"]:
        obs = []
        for view in scene["views"]:
            obs += A.lift(np.array(view["kp"], float), np.array(view["conf"], float),
                          cam_of(view), view["view"], top)
        got, _ = A.assemble(obs)
        want = scene["panels"]
        assert len(got) == len(want), f"scene {scene['scene']}: 盤の数"
        for g, w in zip(got, want):
            assert g["n_lugs"] == w["n_lugs"], f"scene {scene['scene']}: 金具の数"
            assert bool(g["front_known"]) == w["front_known"], f"scene {scene['scene']}: front_known"
            assert bool(g.get("diagonal", False)) == w["diagonal"], f"scene {scene['scene']}: diagonal"
            np.testing.assert_allclose(g["center"], w["center"], atol=1e-3, rtol=0)
            np.testing.assert_allclose([g["width"], g["depth"]], [w["width"], w["depth"]],
                                       atol=1e-3, rtol=0)
            diff = abs((g["front_yaw"] - w["front_yaw"] + 180.0) % 360.0 - 180.0)
            assert diff <= 0.01, f"scene {scene['scene']}: front_yaw {diff}°"
            n_panels += 1
    assert n_panels > 0


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

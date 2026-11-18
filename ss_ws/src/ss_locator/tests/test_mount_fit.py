"""`mount_fit` の試験（純関数。モデル・カメラは使わない）。

観測は、決めたずれで金具を射影した画素で作る（雑音なし）。当てはめが元のずれに
0.05° 以内で戻ること、その他の振る舞い（割り当ての入れ替え・高さを動かさない・
少なくて駄目）を確かめる。
"""
import math

import numpy as np
import pytest

from ss_locator import geometry as G
from ss_locator.mount_fit import FitError, fit_mount_offsets

K = np.array([[1000.0, 0.0, 640.0], [0.0, 1000.0, 360.0], [0.0, 0.0, 1.0]])
DIST = np.zeros(5)
TOP_HEIGHT_M = 1.0
HEIGHT_M = 2.6
MOUNT = dict(mount_x_m=0.0, mount_y_m=0.0, forward_offset_m=0.08,
             optical_offset_m=0.06)
LUGS = np.array([[3.0, -0.8], [3.0, 0.8], [4.6, 0.0]])
POSES = [(-10.0, 25.0), (-4.0, 30.0), (0.0, 22.0), (4.0, 28.0), (10.0, 24.0)]

TRUE = dict(pitch_offset_rad=math.radians(-2.6),
            yaw_offset_rad=math.radians(-5.4),
            roll_rad=math.radians(4.1))


def _observations(true=TRUE, height_m=HEIGHT_M, noise_px=0.0, seed=0, shuffle=False):
    rng = np.random.default_rng(seed)
    obs = []
    for yaw_deg, pitch_deg in POSES:
        T = G.camera_T(height_m, math.radians(yaw_deg), math.radians(pitch_deg),
                       **MOUNT, **true)
        pts = np.c_[LUGS, np.full(len(LUGS), TOP_HEIGHT_M)]
        uv, ok = G.project(pts, K, DIST, T, 1280, 720)
        assert ok.all(), (yaw_deg, pitch_deg, uv, ok)
        if noise_px:
            uv = uv + rng.normal(0.0, noise_px, uv.shape)
        if shuffle:
            uv = uv[::-1].copy()
        obs.append({"yaw_rad": math.radians(yaw_deg),
                    "pitch_down_rad": math.radians(pitch_deg),
                    "pixels": uv})
    return obs


def _fit(obs, **kw):
    return fit_mount_offsets(obs, LUGS, K, DIST, height_m=kw.pop("height_m", HEIGHT_M),
                             top_height_m=TOP_HEIGHT_M, **MOUNT, **kw)


def test_fit_recovers_offsets():
    result = _fit(_observations())
    assert math.degrees(abs(result.pitch_offset_rad - TRUE["pitch_offset_rad"])) < 0.05
    assert math.degrees(abs(result.yaw_offset_rad - TRUE["yaw_offset_rad"])) < 0.05
    assert math.degrees(abs(result.roll_rad - TRUE["roll_rad"])) < 0.05
    assert result.mean_m < 1e-6


def test_fit_recovers_offsets_with_noise():
    result = _fit(_observations(noise_px=0.5, seed=7))
    assert math.degrees(abs(result.pitch_offset_rad - TRUE["pitch_offset_rad"])) < 0.3
    assert math.degrees(abs(result.yaw_offset_rad - TRUE["yaw_offset_rad"])) < 0.3
    assert math.degrees(abs(result.roll_rad - TRUE["roll_rad"])) < 0.3


def test_fit_assignment():
    true = dict(pitch_offset_rad=math.radians(4.9),
                yaw_offset_rad=math.radians(-7.9),
                roll_rad=math.radians(3.0))
    result = _fit(_observations(true=true, shuffle=True))
    assert math.degrees(abs(result.pitch_offset_rad - true["pitch_offset_rad"])) < 0.05
    assert math.degrees(abs(result.yaw_offset_rad - true["yaw_offset_rad"])) < 0.05
    assert math.degrees(abs(result.roll_rad - true["roll_rad"])) < 0.05


def test_fit_fixed_height():
    # 既定では高さを動かさない
    result = _fit(_observations(height_m=2.9), height_m=2.6)
    assert result.height_m == 2.6
    # fit_height=True で高さも戻る（雑音なし）
    result = _fit(_observations(height_m=2.9), height_m=2.6, fit_height=True)
    assert abs(result.height_m - 2.9) < 1e-3
    assert math.degrees(abs(result.pitch_offset_rad - TRUE["pitch_offset_rad"])) < 0.05
    assert math.degrees(abs(result.yaw_offset_rad - TRUE["yaw_offset_rad"])) < 0.05
    assert math.degrees(abs(result.roll_rad - TRUE["roll_rad"])) < 0.05


def test_fit_too_few():
    obs = _observations()
    with pytest.raises(FitError, match="at least 2"):
        fit_mount_offsets(obs, LUGS[:1], K, DIST, height_m=HEIGHT_M,
                          top_height_m=TOP_HEIGHT_M, **MOUNT)
    with pytest.raises(FitError, match="at least 2 pixels"):
        _fit([{"yaw_rad": 0.0, "pitch_down_rad": 0.4, "pixels": obs[0]["pixels"][:1]}])
    with pytest.raises(FitError, match="at least 2 pixels"):
        _fit([])

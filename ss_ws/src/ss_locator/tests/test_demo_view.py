"""`demo_view.py` の純関数の試験。"""

from __future__ import annotations

import numpy as np
import pytest

from ss_locator.demo_view import LatestFrame, RateMeter, draw_lugs, fit_to_canvas


def _blank(w, h):
    return np.zeros((h, w, 3), dtype=np.uint8)


def test_fit_to_canvas_sizes() -> None:
    cases = [
        ((640, 480), 1.5, (160, 0)),
        ((1920, 1080), 2.0 / 3.0, (0, 0)),
        ((1280, 720), 1.0, (0, 0)),
    ]
    for (w, h), scale, pad in cases:
        canvas, actual_scale, actual_pad = fit_to_canvas(_blank(w, h))
        assert canvas.shape == (720, 1280, 3)
        assert actual_scale == pytest.approx(scale)
        assert actual_pad == pad

    canvas, scale, (pad_x, pad_y) = fit_to_canvas(_blank(720, 1280))
    assert canvas.shape == (720, 1280, 3)
    assert scale == pytest.approx(720 / 1280)
    assert pad_y == 0
    assert pad_x > 0


def test_fit_to_canvas_keeps_aspect() -> None:
    image = _blank(640, 480)
    image[150:250, 200:300] = 255
    canvas, _, _ = fit_to_canvas(image)
    ys, xs = np.nonzero(canvas[:, :, 0] > 127)
    assert xs.size > 0
    width = int(xs.max() - xs.min() + 1)
    height = int(ys.max() - ys.min() + 1)
    assert abs(width - height) <= 1


def test_draw_lugs_does_not_modify_input() -> None:
    canvas = np.full((720, 1280, 3), 128, dtype=np.uint8)
    original = canvas.copy()
    out = draw_lugs(
        canvas,
        np.zeros((0, 4), np.float32),
        np.zeros((0, 4, 2), np.float32),
        np.zeros((0,), np.float32),
        [],
    )
    assert np.array_equal(canvas, original)
    assert out.shape == canvas.shape


def test_draw_lugs_draws_marks() -> None:
    canvas = np.full((720, 1280, 3), 128, dtype=np.uint8)
    boxes = np.array([[100.0, 100.0, 200.0, 200.0]], np.float32)
    keypoints = np.array(
        [[[110.0, 110.0], [150.0, 150.0], [180.0, 100.0], [180.0, 180.0]]],
        np.float32,
    )
    out = draw_lugs(canvas, boxes, keypoints, np.array([0.9], np.float32), ["lugs: 1"])

    assert out[100, 150, 0] != canvas[100, 150, 0]
    for x, y in [(110, 110), (150, 150), (180, 100), (180, 180)]:
        assert not np.array_equal(out[y, x], canvas[y, x])
    assert not np.array_equal(out[0:30, 0:200], canvas[0:30, 0:200])


def test_latest_frame_keeps_newest() -> None:
    frame = LatestFrame()
    assert frame.take(0.01) is None

    frame.put(1)
    frame.put(2)
    frame.put(3)
    assert frame.dropped == 2
    assert frame.take(0.01) == 3
    assert frame.take(0.01) is None

    frame.put(4)
    frame.close()
    assert frame.take(0.01) is None


def test_latest_frame_counts_received() -> None:
    frame = LatestFrame()
    assert frame.received == 0

    for value in (1, 2, 3):
        frame.put(value)

    assert frame.received == 3
    assert frame.take(0.01) == 3
    assert frame.received == 3


def test_rate_meter() -> None:
    meter = RateMeter(window_s=1.0)

    time_s = 0.0
    while time_s < 1.0 - 1e-9:
        meter.tick(time_s)
        assert meter.rate(time_s) is None
        time_s = round(time_s + 0.1, 10)

    meter.tick(1.0)
    assert meter.rate(1.0) == pytest.approx(10.0, abs=0.5)

    assert meter.rate(2.0) == pytest.approx(0.0, abs=1e-9)

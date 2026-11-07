"""金具認識のリアルタイム表示の純関数（ROS 2・ultralytics を読み込まない）。

詳細設計: `docs/plan/detailed/DetailedDesign-demo.md` §2。
"""

from __future__ import annotations

import threading
import time
from collections.abc import Sequence

import cv2
import numpy as np

ROOT_COLOR = (0, 0, 255)
RING_COLOR = (0, 255, 0)
END_COLOR = (255, 0, 0)
BOX_COLOR = (0, 255, 255)
CONF_COLOR = (255, 255, 255)
INFO_COLOR = (255, 255, 255)


def fit_to_canvas(
    image: np.ndarray, width: int = 1280, height: int = 720
) -> tuple[np.ndarray, float, tuple[int, int]]:
    """縦横比を保って `width`×`height` の中央に置き、倍率と余白を返す。"""
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    h, w = image.shape[:2]
    scale = min(width / w, height / h)
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    if (new_w, new_h) != (w, h):
        image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    pad_x = (width - new_w) // 2
    pad_y = (height - new_h) // 2
    canvas[pad_y : pad_y + new_h, pad_x : pad_x + new_w] = image
    return canvas, float(scale), (pad_x, pad_y)


def draw_lugs(
    canvas: np.ndarray,
    boxes: np.ndarray,
    keypoints: np.ndarray,
    confidences: np.ndarray,
    info: Sequence[str],
) -> np.ndarray:
    """枠・キーポイント・確信度・左上の情報を描いた写しを返す。"""
    out = canvas.copy()

    for i in range(len(boxes)):
        x1, y1, x2, y2 = (int(round(float(v))) for v in boxes[i][:4])
        cv2.rectangle(out, (x1, y1), (x2, y2), BOX_COLOR, 2)
        label = f"{float(confidences[i]):.2f}"
        cv2.putText(
            out,
            label,
            (x1, max(12, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            CONF_COLOR,
            1,
            cv2.LINE_AA,
        )

    for points in keypoints:
        colors = (ROOT_COLOR, RING_COLOR, END_COLOR, END_COLOR)
        for j, point in enumerate(points[:4]):
            x, y = int(round(float(point[0]))), int(round(float(point[1])))
            cv2.circle(out, (x, y), 4, colors[j], -1)
        if len(points) >= 4:
            p2 = (int(round(float(points[2][0]))), int(round(float(points[2][1]))))
            p3 = (int(round(float(points[3][0]))), int(round(float(points[3][1]))))
            cv2.line(out, p2, p3, END_COLOR, 1, cv2.LINE_AA)

    for i, line in enumerate(info or []):
        cv2.putText(
            out,
            str(line),
            (8, 24 + 22 * i),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            INFO_COLOR,
            1,
            cv2.LINE_AA,
        )

    return out


class RateMeter:
    """直近 `window_s` 秒の出来事の数を数え、1 秒あたりの速さを返す。"""

    def __init__(self, window_s: float = 1.0) -> None:
        self._window_s = float(window_s)
        self._first = None
        self._events: list[tuple[float, int]] = []

    def tick(self, now: float, count: int = 1) -> None:
        now = float(now)
        if self._first is None:
            self._first = now
        self._events = [
            (at, number)
            for at, number in self._events
            if at > now - self._window_s
        ]
        self._events.append((now, int(count)))

    def rate(self, now: float) -> float | None:
        now = float(now)
        if self._first is None or now - self._first < self._window_s:
            return None
        cutoff = now - self._window_s
        total = sum(number for at, number in self._events if cutoff < at <= now)
        return total / self._window_s


class LatestFrame:
    """スレッドをまたいで最新の 1 コマだけを持つ箱。"""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._frame = None
        self._closed = False
        self.dropped = 0
        self.received = 0

    def put(self, frame) -> None:
        with self._condition:
            if self._closed:
                return
            self.received += 1
            if self._frame is not None:
                self.dropped += 1
            self._frame = frame
            self._condition.notify()

    def take(self, timeout_s: float):
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            if self._closed:
                return None
            while self._frame is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
                if self._closed:
                    return None
            frame = self._frame
            self._frame = None
            return frame

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._frame = None
            self._condition.notify_all()

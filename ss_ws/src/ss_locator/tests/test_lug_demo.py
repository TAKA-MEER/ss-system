"""`lug_demo.py` のつなぐ層の試験。偽の検出器とメモリの中のコマで回す。"""

from __future__ import annotations

import cv2
import numpy as np
import pytest

from ss_locator import lug_demo
from ss_locator.demo_view import LatestFrame


def _marked(value):
    return np.full((720, 1280, 3), int(value), dtype=np.uint8)


def _empty_detections():
    return (
        np.zeros((0, 4, 2), np.float32),
        np.zeros((0,), np.float32),
        np.zeros((0, 4), np.float32),
    )


def _make_fake_detector():
    class FakeDetector:
        instances = []

        def __init__(self, model_file, model_sha256, imgsz, conf):
            self.conf = conf
            self.seen = []
            type(self).instances.append(self)

        def detect_with_boxes(self, image):
            self.seen.append(int(image[0, 0, 0]))
            return _empty_detections()

    return FakeDetector


def _write_image(path, value):
    assert cv2.imwrite(str(path), _marked(value))


class _AddingDetector:
    def __init__(self, source):
        self.source = source
        self.seen = []

    def detect_with_boxes(self, image):
        marker = int(image[0, 0, 0])
        self.seen.append(marker)
        for step in (1, 2, 3):
            self.source.put(_marked(marker + step))
        return _empty_detections()


def test_run_demo_skips_stale_frames() -> None:
    source = LatestFrame()
    source.put(_marked(0))
    detector = _AddingDetector(source)
    results = []

    def show(image, result):
        results.append(result)
        return True

    lug_demo.run_demo(source, detector, show, 0.25, max_frames=3)

    assert detector.seen == [0, 3, 6]
    assert [result["frame"] for result in results] == [0, 3, 6]
    assert results[-1]["dropped"] > 0


def test_run_demo_draws_on_inferred_frame() -> None:
    inferred_value = 50
    newer_value = 200
    frame = _marked(inferred_value)
    source = LatestFrame()
    source.put(frame)
    received = []
    shown = []

    class Detector:
        def detect_with_boxes(self, image):
            received.append(np.array(image, copy=True))
            source.put(_marked(newer_value))
            boxes = np.array([[10.0, 100.0, 60.0, 150.0]], np.float32)
            keypoints = np.array(
                [[[15.0, 105.0], [35.0, 125.0], [55.0, 105.0], [55.0, 145.0]]],
                np.float32,
            )
            return keypoints, np.array([0.8], np.float32), boxes

    def show(image, result):
        shown.append(image)
        return True

    lug_demo.run_demo(source, Detector(), show, 0.25, max_frames=1)

    assert len(received) == 1
    assert np.array_equal(received[0], frame)
    background = shown[0][300:400, 300:400]
    assert np.all(background == inferred_value)
    assert not np.any(background == newer_value)


def test_folder_source_in_order_no_drop(tmp_path, monkeypatch, capsys) -> None:
    folder = tmp_path / "in"
    folder.mkdir()
    _write_image(folder / "b.PNG", 20)
    _write_image(folder / "a.jpg", 10)
    _write_image(folder / "c.jpeg", 30)
    (folder / "note.txt").write_text("not an image", encoding="utf-8")

    fake = _make_fake_detector()
    monkeypatch.setattr(lug_demo, "Detector", fake)

    assert lug_demo.main(["--source", str(folder), "--headless"]) == 0

    assert fake.instances[-1].seen == [10, 20, 30]
    lines = capsys.readouterr().out.strip().splitlines()
    assert len(lines) == 3
    assert all("dropped=0" in line for line in lines)


def test_main_honours_max_frames(tmp_path, monkeypatch, capsys) -> None:
    folder = tmp_path / "in"
    folder.mkdir()
    for index, name in enumerate(["a.png", "b.png", "c.png", "d.png", "e.png"]):
        _write_image(folder / name, 10 * (index + 1))

    fake = _make_fake_detector()
    monkeypatch.setattr(lug_demo, "Detector", fake)

    assert (
        lug_demo.main(
            ["--source", str(folder), "--headless", "--max-frames", "2"]
        )
        == 0
    )

    assert fake.instances[-1].seen == [10, 20]
    assert len(capsys.readouterr().out.strip().splitlines()) == 2


def test_conf_reaches_detector(tmp_path, monkeypatch) -> None:
    folder = tmp_path / "empty"
    folder.mkdir()
    fake = _make_fake_detector()
    monkeypatch.setattr(lug_demo, "Detector", fake)

    assert lug_demo.main(["--source", str(folder), "--headless", "--conf", "0.6"]) == 0
    assert fake.instances[-1].conf == pytest.approx(0.6)

    assert lug_demo.main(["--source", str(folder), "--headless"]) == 0
    assert fake.instances[-1].conf == pytest.approx(
        lug_demo.load_defaults()["detect_conf"]
    )


def test_main_writes_no_files(tmp_path, monkeypatch) -> None:
    work = tmp_path / "work"
    folder = tmp_path / "in"
    work.mkdir()
    folder.mkdir()
    _write_image(folder / "a.png", 10)
    _write_image(folder / "b.png", 20)

    fake = _make_fake_detector()
    monkeypatch.setattr(lug_demo, "Detector", fake)
    monkeypatch.chdir(work)

    before_work = {path for path in work.rglob("*")}
    before_folder = {path for path in folder.rglob("*")}

    assert lug_demo.main(["--source", str(folder), "--headless"]) == 0

    assert {path for path in work.rglob("*")} == before_work
    assert {path for path in folder.rglob("*")} == before_folder

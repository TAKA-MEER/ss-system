"""Detector の回帰試験。

材料（学習済みモデル・試作の画像と preds.json）が無いときは skip する。
置き場所は環境変数 SS_MODEL_DIR（既定 /ws/model）と SS_PROTO_DIR（既定 /proto）で変えられる。
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import cv2
import numpy as np
import pytest
from scipy.optimize import linear_sum_assignment

from ss_locator import detector
from ss_locator.detector import Detector

MODEL_DIR = Path(os.environ.get("SS_MODEL_DIR", "/ws/model"))
PROTO_DIR = Path(os.environ.get("SS_PROTO_DIR", "/proto"))
MODEL_FILE = MODEL_DIR / "lug_v3_s.pt"
MODEL_SHA256 = "5a4689a9d49d549f61dddb1f2fb375583e2a98ca0333791776e6b140f4e83227"
PROTO_RUN = PROTO_DIR / "data" / "synthetic" / "robot_v3_test"
PROTO_PREDS = PROTO_DIR / "results" / "lug_v3_on_v3_test" / "preds.json"
N_IMAGES = 5
CONF = 0.25
IMGSZ = 1280


def _require_model() -> None:
    if not MODEL_FILE.is_file():
        pytest.skip(f"model file not found: {MODEL_FILE}")


def test_detector_matches_proto() -> None:
    """試作 preds.json の最初の 5 枚と、検出の数・キーポイントを比べる。"""
    _require_model()
    if not PROTO_PREDS.is_file():
        pytest.skip(f"preds.json not found: {PROTO_PREDS}")
    if not PROTO_RUN.is_dir():
        pytest.skip(f"test images not found: {PROTO_RUN}")

    preds = json.loads(PROTO_PREDS.read_text(encoding="utf-8"))
    names = sorted(preds)[:N_IMAGES]
    det = Detector(MODEL_FILE, MODEL_SHA256, IMGSZ, CONF)

    for name in names:
        image = cv2.imread(str(PROTO_RUN / name))
        assert image is not None, f"image not readable: {PROTO_RUN / name}"
        kp, conf = det.detect(image)

        proto_kp = np.array(preds[name]["kp"], dtype=np.float64).reshape(-1, 4, 3)
        proto_conf = np.array(preds[name]["conf"], dtype=np.float64)
        proto_kp = proto_kp[proto_conf >= CONF, :, :2]

        assert len(conf) == len(proto_kp), (
            f"{name}: detection count differs (detector {len(conf)}, proto {len(proto_kp)})"
        )
        if len(conf) == 0:
            continue

        distance = np.max(
            np.linalg.norm(proto_kp[:, None, :, :] - kp[None, :, :, :], axis=-1), axis=-1
        )
        rows, cols = linear_sum_assignment(distance)
        worst = float(distance[rows, cols].max())
        assert worst <= 1.0, f"{name}: keypoint differs by {worst:.3f} px (> 1 px)"


def test_detector_rejects_bad_hash(monkeypatch: pytest.MonkeyPatch) -> None:
    """SHA-256 が違えば例外を出し、モデルを読まない。"""
    _require_model()
    loaded: list[str] = []
    monkeypatch.setattr(detector, "_load_yolo", lambda path: loaded.append(path))

    with pytest.raises(ValueError):
        Detector(MODEL_FILE, "0" * 64, IMGSZ, CONF)

    assert loaded == []

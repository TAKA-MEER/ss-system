"""学習済みモデルを読み込み、画像から吊り金具のキーポイントを検出する。

推論の呼び方は試作の ``src/eval_lug.py`` に揃える（``imgsz``・確信度・``device``）。
重みは読む前に SHA-256 を照合し、一致しなければモデルを読まない。
"""

from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

DEFAULT_IMGSZ = 1280
DEFAULT_CONF = 0.25


def sha256_file(path: str | Path) -> str:
    """ファイルの SHA-256 を小文字の 16 進で返す。"""
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_yolo(model_file: str) -> object:
    """重みを読む。試験から差し替えられるよう関数に切り出す。"""
    import os

    os.environ.setdefault("YOLO_AUTOINSTALL", "false")
    from ultralytics import YOLO

    return YOLO(model_file)


class Detector:
    """重みを 1 つ持ち、1 枚ずつ検出する。"""

    def __init__(self, model_file: str | Path, model_sha256: str, imgsz: int, conf: float):
        self.model_file = Path(model_file)
        if not self.model_file.is_file():
            raise FileNotFoundError(f"model file not found: {self.model_file}")
        actual = sha256_file(self.model_file)
        if actual.lower() != str(model_sha256).lower():
            raise ValueError(
                f"model SHA-256 mismatch: expected {model_sha256}, got {actual}"
            )
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self._model = _load_yolo(str(self.model_file))

    def detect_with_boxes(
        self, image_bgr: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """BGR 画像 1 枚を検出し、キーポイント (N,4,2)・確信度 (N,)・枠 (N,4) xyxy を返す。"""
        result = self._model.predict(
            image_bgr,
            imgsz=self.imgsz,
            conf=self.conf,
            device="cpu",
            verbose=False,
        )[0]
        if result.keypoints is None or result.boxes is None or len(result.boxes) == 0:
            return (
                np.zeros((0, 4, 2), dtype=np.float32),
                np.zeros((0,), dtype=np.float32),
                np.zeros((0, 4), dtype=np.float32),
            )
        keypoints = result.keypoints.data.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()
        boxes = result.boxes.xyxy.cpu().numpy()
        return (
            keypoints[:, :, :2].astype(np.float32),
            confidences.astype(np.float32),
            boxes.astype(np.float32),
        )

    def detect(self, image_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """BGR 画像 1 枚を検出し、キーポイント (N,4,2) と確信度 (N,) を返す。"""
        keypoints, confidences, _ = self.detect_with_boxes(image_bgr)
        return keypoints, confidences

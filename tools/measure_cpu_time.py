#!/usr/bin/env python3
"""CPU で 1 枚あたりの検出時間を測る。

コンテナの中（GPU なし）で、画像を名前順に ``--n`` 枚 Detector にかけ、
最初の 1 枚を除いた 1 枚あたりの時間の中央値と 90% を出す。
モデルと推論の条件は ``ss_ws/src/ss_locator/config/locator.yaml`` の既定値を使う。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_FILE = REPO_ROOT / "ss_ws" / "src" / "ss_locator" / "config" / "locator.yaml"
sys.path.insert(0, str(REPO_ROOT / "ss_ws" / "src" / "ss_locator"))

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def load_params() -> dict:
    import yaml

    config = yaml.safe_load(CONFIG_FILE.read_text(encoding="utf-8"))
    return config["locator_node"]["ros__parameters"]


def resolve(path: str | Path) -> Path:
    path = Path(path)
    return path if path.is_absolute() else REPO_ROOT / path


def cpu_name() -> str:
    try:
        with open("/proc/cpuinfo", encoding="utf-8") as f:
            for line in f:
                if line.startswith("model name"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return "unknown"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", required=True, help="画像のディレクトリ")
    ap.add_argument("--n", type=int, default=50, help="使う枚数（既定 50）")
    args = ap.parse_args()

    import cv2
    import torch

    from ss_locator.detector import Detector

    params = load_params()
    paths = sorted(p for p in Path(args.images).iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    paths = paths[: args.n]
    if len(paths) < 2:
        ap.error("2 枚以上必要（最初の 1 枚は暖機として除く）")

    det = Detector(
        resolve(params["model_file"]),
        params["model_sha256"],
        params["detect_imgsz"],
        params["detect_conf"],
    )

    times = []
    for i, path in enumerate(paths):
        image = cv2.imread(str(path))
        if image is None:
            raise RuntimeError(f"image not readable: {path}")
        t0 = time.perf_counter()
        det.detect(image)
        elapsed = time.perf_counter() - t0
        if i > 0:
            times.append(elapsed)

    print(f"images: {len(paths)}（最初の 1 枚を除く {len(times)} 枚で集計）")
    print(f"cpu: {cpu_name()}")
    print(f"cores: {os.cpu_count()}")
    print(f"torch: {torch.__version__}")
    print(f"torch threads: {torch.get_num_threads()}")
    print(f"median: {np.median(times):.4f} s")
    print(f"p90: {np.percentile(times, 90):.4f} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

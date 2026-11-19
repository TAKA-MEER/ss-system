"""撮影の記録（1 枚ごとの向き・高さ・検出）を書く小さな関数。

詳細設計: `docs/plan/detailed/DetailedDesign-tester.md` §7a。
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import cv2
import numpy as np


def _opt(value):
    return None if value is None else float(value)


def shot_record(
    shot,
    *,
    lift_height_m,
    camera_height_m,
    yaw_rad,
    pitch_down_rad,
    keypoints,
    confidences,
    lugs,
    photo,
    fit_parameters,
    now=None,
):
    """1 枚ぶんの記録を JSON にできる dict にする（numpy の型は float・list に直す）。

    `lugs` は `assemble.lift` の観測（`p` が床の [x, y]）の並び。
    """
    record = {
        "time": float(time.time() if now is None else now),
        "shot": int(shot),
        "lift_height_m": _opt(lift_height_m),
        "camera_height_m": _opt(camera_height_m),
        "yaw_rad": float(yaw_rad),
        "pitch_down_rad": _opt(pitch_down_rad),
        "keypoints": np.asarray(keypoints, float)[..., :2].tolist(),
        "confidences": np.asarray(confidences, float).reshape(-1).tolist(),
        "lugs": [[float(lug["p"][0]), float(lug["p"][1])] for lug in lugs],
        "photo": photo,
    }
    for name, value in fit_parameters.items():
        record[name] = float(value)
    return record


def photo_name(shot):
    return "shot_%06d.jpg" % int(shot)


def append_shot(directory, record, image=None):
    """`<directory>/shots.jsonl` に 1 行足す。`image` があれば `record["photo"]` の名前で書く。"""
    folder = Path(directory)
    folder.mkdir(parents=True, exist_ok=True)
    if image is not None and record.get("photo"):
        if not cv2.imwrite(str(folder / record["photo"]), image):
            raise OSError("静止画を書けない: %s" % (folder / record["photo"]))
    with open(folder / "shots.jsonl", "a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, ensure_ascii=False) + "\n")

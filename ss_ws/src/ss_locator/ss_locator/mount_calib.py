"""カメラの取り付けの角度のずれを求める道具 `mount_calib`（本番の流れは通さない）。

撮る: カメラの向きが分からないなら動かさずに終わる。指定した向きへ順に向け、
着いたら `settle_s` 待って静止画を撮り、検出器で金具の画素を得る。人の割り込み・
失敗が来たらそこで止める（それまでの写真で当てはめる）。撮り終えたら撮る前の向きへ
戻してから手を離す。写真と観測の json を保存し、json だけから当てはめ直せる。

終わり方: 当てはめた → 0、向きが分からず動かさなかった・金具が 2 個に割り当たら
なかった → 0 以外。設定ファイルは書き換えず、写す行を出す。

カメラは `HveCamera`、検出は `Detector` を使う。試験で偽物に差し替えられるよう、
`main` は工場関数を引数で受けられる。

詳細設計: `docs/plan/detailed/DetailedDesign-core.md` §2.1、
`docs/plan/detailed/DetailedDesign-names.md` §11（`WP-CAL-02`、
外れた検出を捨てる `WP-CAL-03`）。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np

from .mount_fit import FitError, fit_mount_offsets

DEFAULT_PARAMS_FILE = str(
    Path(__file__).resolve().parent.parent / "config" / "locator.yaml"
)
DEFAULT_HOST = "192.168.5.129"
REJECT_M = 0.3


class CalibError(Exception):
    """撮れない（向きが分からないなど。0 以外で終わる）。"""


# ------------------------------------------------------------------ 引数の読み方
def parse_lugs(text):
    """`"x,y;x,y"` → (M,2)。"""
    lugs = []
    for part in str(text).split(";"):
        part = part.strip()
        if not part:
            continue
        x, y = part.split(",")
        lugs.append([float(x), float(y)])
    return np.array(lugs, float)


def parse_poses(text):
    """`"ヨー:ピッチ,…"`（度）→ [(ヨー rad, ピッチ rad)]。"""
    poses = []
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        yaw, pitch = part.split(":")
        poses.append((math.radians(float(yaw)), math.radians(float(pitch))))
    return poses


def load_params(path):
    """`locator.yaml` → フラットな dict。無ければ空。"""
    if not path:
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    except FileNotFoundError:
        return {}
    if isinstance(data, dict) and "locator_node" in data:
        return dict(data["locator_node"].get("ros__parameters") or {})
    return dict(data) if isinstance(data, dict) else {}


def _float(params, name, default=0.0):
    try:
        return float(params.get(name, default))
    except (TypeError, ValueError):
        return default


# ------------------------------------------------------------------ 撮る
def _wait_arrived(camera, timeout_s, sleep):
    deadline = time.monotonic() + timeout_s
    while True:
        state = camera.state()
        if getattr(state, "taken_over", False):
            return state
        if getattr(state, "arrived", False) or getattr(state, "failed", False):
            return state
        if time.monotonic() >= deadline:
            return state
        sleep(0.02)


def _capture_one(camera, timeout_s=10.0):
    done = threading.Event()
    holder = {}

    def _done(verdict, result):
        holder["verdict"] = verdict
        holder["result"] = result
        done.set()

    camera.capture(_done)
    if not done.wait(timeout_s):
        return "fail", None
    return holder.get("verdict", "fail"), holder.get("result")


def capture_observations(camera, detector, poses, *, settle_s=0.0,
                         sleep=time.sleep, aim_timeout_s=30.0):
    """指定した向きへ向けて撮り、観測の並びを返す。

    観測は `{"yaw_rad", "pitch_down_rad", "pixels", "image"}`。向きが分からないときは
    何も向けず `CalibError`。人の割り込み・失敗ではそこで止め、それまでを返す。
    撮り終えたら撮る前の向きへ戻してから手を離す（割り込みのときは戻さない）。
    """
    state = camera.state()
    if not getattr(state, "pose_known", True) or state.yaw_rad is None:
        raise CalibError("camera pose unknown: not moving the camera")
    origin = (float(state.yaw_rad), float(state.pitch_down_rad))

    observations = []
    taken_over = False
    try:
        for yaw, pitch in poses:
            state = camera.state()
            if getattr(state, "taken_over", False):
                taken_over = True
                break
            if not getattr(state, "pose_known", True):
                break
            camera.point(yaw, pitch)
            state = _wait_arrived(camera, aim_timeout_s, sleep)
            if getattr(state, "taken_over", False):
                taken_over = True
                break
            if getattr(state, "failed", False):
                break
            if settle_s:
                sleep(settle_s)
            verdict, captured = _capture_one(camera)
            if verdict != "use" or captured is None:
                break
            keypoints, _conf = detector.detect(captured["image"])
            keypoints = np.asarray(keypoints, float)
            pixels = keypoints[:, 0, :2] if keypoints.size else np.zeros((0, 2))
            observations.append({
                "yaw_rad": float(captured["yaw_rad"]),
                "pitch_down_rad": float(captured["pitch_down_rad"]),
                "pixels": pixels,
                "image": captured["image"],
            })
    finally:
        if not taken_over:
            try:
                camera.point(origin[0], origin[1])
            except Exception:
                pass
        camera.release()
    return observations


# ------------------------------------------------------------------ 当てる・出す
def fit_from_data(data, params, fit_height=None, reject_m=None):
    """`observations.json` の中身とパラメータ → `FitResult`。

    `fit_height` を渡すと json の値より優先する（コマンドラインの `--fit-height`）。
    `reject_m` を渡すと json の値（`reject_m`）より優先する（`--reject-m`）。
    """
    K = np.asarray(params.get("camera_K", data.get("camera_K")), float)
    dist = np.asarray(params.get("camera_dist", data.get("camera_dist")), float)
    height_m = _float(data, "height_m", 0.0)
    if "camera_height_offset_m" in params or "camera_height_offset_m" in data:
        height_m = _float(data, "lift_height_m", 0.0) + _float(
            params, "camera_height_offset_m",
            _float(data, "camera_height_offset_m", 0.0))
    observations = [
        {"yaw_rad": o["yaw_rad"], "pitch_down_rad": o["pitch_down_rad"],
         "pixels": np.asarray(o["pixels"], float)}
        for o in data.get("observations", [])
    ]
    return fit_mount_offsets(
        observations,
        np.asarray(data["lugs"], float),
        K, dist,
        height_m=height_m,
        top_height_m=float(data["top_height_m"]),
        mount_x_m=_float(params, "camera_mount_x_m",
                         _float(data, "camera_mount_x_m", 0.0)),
        mount_y_m=_float(params, "camera_mount_y_m",
                         _float(data, "camera_mount_y_m", 0.0)),
        forward_offset_m=_float(params, "camera_forward_offset_m",
                                _float(data, "camera_forward_offset_m", 0.0)),
        optical_offset_m=_float(params, "camera_optical_offset_m",
                                _float(data, "camera_optical_offset_m", 0.0)),
        fit_height=(bool(data.get("fit_height", False))
                    if fit_height is None else bool(fit_height)),
        reject_m=float(data.get("reject_m", REJECT_M))
        if reject_m is None else float(reject_m),
    )


def report(result, fit_height=False):
    """3 つの値と、観測ごとの差・平均・最大と、`locator.yaml` に写す行を出す。

    高さを当てはめたときだけ、当てはめた高さ（ピッチの回転軸の床からの高さ）も出す。
    """
    if fit_height:
        print("height_m: %.3f" % result.height_m)
    print("camera_pitch_offset_deg: %.2f" % math.degrees(result.pitch_offset_rad))
    print("camera_yaw_offset_deg: %.2f" % math.degrees(result.yaw_offset_rad))
    print("camera_roll_deg: %.2f" % math.degrees(result.roll_rad))
    for i, r in enumerate(result.residuals):
        print("observation %d: n=%d mean=%.4f m max=%.4f m"
              % (i, r["n"], r["mean_m"], r["max_m"]))
    print("mean=%.4f m max=%.4f m" % (result.mean_m, result.max_m))
    for rej in getattr(result, "rejected", []):
        pixel = rej.get("pixel", [])
        print("rejected: observation %s pixel (%.1f, %.1f) distance=%.4f m"
              % (rej.get("observation"), float(pixel[0]), float(pixel[1]),
                 float(rej.get("distance_m"))))


def _save_observations(out_dir, observations, data):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    saved = []
    for i, obs in enumerate(observations):
        photo = "frame_%03d.jpg" % i
        image = obs.get("image")
        if image is not None:
            try:
                import cv2
                cv2.imwrite(str(out / photo), image)
            except Exception:
                pass
        saved.append({"yaw_rad": obs["yaw_rad"],
                      "pitch_down_rad": obs["pitch_down_rad"],
                      "pixels": np.asarray(obs["pixels"], float).tolist(),
                      "photo": photo})
    data = dict(data)
    data["observations"] = saved
    (out / "observations.json").write_text(
        json.dumps(data, indent=2), encoding="utf-8")
    return data


# ------------------------------------------------------------------ 入口
def _build_parser():
    p = argparse.ArgumentParser(prog="mount_calib")
    p.add_argument("--lugs", default="")
    p.add_argument("--top-height-m", type=float, default=0.0)
    p.add_argument("--lift-height-m", type=float, default=0.0)
    p.add_argument("--poses", default="")
    p.add_argument("--params-file", default=DEFAULT_PARAMS_FILE)
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--out", default="")
    p.add_argument("--from", dest="from_dir", default="")
    p.add_argument("--fit-height", action="store_true")
    p.add_argument("--reject-m", type=float, default=REJECT_M)
    return p


def main(argv=None, camera_factory=None, detector_factory=None):
    args = _build_parser().parse_args(argv)
    params = load_params(args.params_file)

    if args.from_dir:
        path = Path(args.from_dir) / "observations.json"
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            print("cannot read observations: %s" % error, file=sys.stderr)
            return 1
        fit_height = bool(args.fit_height) or bool(data.get("fit_height", False))
        try:
            result = fit_from_data(data, params, fit_height=fit_height,
                                   reject_m=args.reject_m)
        except FitError as error:
            print(str(error), file=sys.stderr)
            return 1
        report(result, fit_height=fit_height)
        return 0

    if not args.lugs or not args.poses:
        print("--lugs and --poses are required when capturing", file=sys.stderr)
        return 2
    lugs = parse_lugs(args.lugs)
    poses = parse_poses(args.poses)
    if len(lugs) < 2:
        print("measured lugs too few: need at least 2", file=sys.stderr)
        return 1

    if camera_factory is None:
        from .hve_camera import HveCamera

        def camera_factory():
            return HveCamera(
                host=args.host,
                cmd_period_ms=int(_float(params, "camera_cmd_period_ms", 100)),
                state_timeout_ms=int(_float(params, "camera_state_timeout_ms", 600)))
    if detector_factory is None:
        from .detector import Detector

        def detector_factory():
            return Detector(
                params.get("model_file", "model/lug_v3_s.pt"),
                params.get("model_sha256", ""),
                int(_float(params, "detect_imgsz", 1280)),
                _float(params, "detect_conf", 0.25))
    camera = camera_factory()
    detector = detector_factory()

    height_m = args.lift_height_m + _float(params, "camera_height_offset_m", 0.0)
    data = {
        "lugs": lugs.tolist(),
        "top_height_m": args.top_height_m,
        "lift_height_m": args.lift_height_m,
        "height_m": height_m,
        "camera_K": list(np.asarray(params.get("camera_K", []), float)),
        "camera_dist": list(np.asarray(params.get("camera_dist", []), float)),
        "camera_mount_x_m": _float(params, "camera_mount_x_m", 0.0),
        "camera_mount_y_m": _float(params, "camera_mount_y_m", 0.0),
        "camera_forward_offset_m": _float(params, "camera_forward_offset_m", 0.0),
        "camera_optical_offset_m": _float(params, "camera_optical_offset_m", 0.0),
        "camera_height_offset_m": _float(params, "camera_height_offset_m", 0.0),
        "fit_height": bool(args.fit_height),
        "reject_m": float(args.reject_m),
        "observations": [],
    }
    try:
        observations = capture_observations(
            camera, detector, poses,
            settle_s=_float(params, "settle_s", 0.0),
            aim_timeout_s=_float(params, "aim_timeout_s", 30.0))
    except CalibError as error:
        print(str(error), file=sys.stderr)
        return 1
    if args.out:
        data = _save_observations(args.out, observations, data)
    else:
        data["observations"] = [
            {"yaw_rad": o["yaw_rad"], "pitch_down_rad": o["pitch_down_rad"],
             "pixels": np.asarray(o["pixels"], float).tolist()}
            for o in observations]
    try:
        result = fit_from_data(data, params, fit_height=bool(args.fit_height),
                               reject_m=args.reject_m)
    except FitError as error:
        print(str(error), file=sys.stderr)
        return 1
    report(result, fit_height=bool(args.fit_height))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

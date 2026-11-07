"""金具認識のリアルタイム表示（本番外の道具）の入口。

詳細設計: `docs/plan/detailed/DetailedDesign-demo.md` §1・§3。
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import cv2
import yaml

from .demo_view import LatestFrame, RateMeter, draw_lugs, fit_to_canvas
from .detector import Detector

TAKE_TIMEOUT_S = 0.1
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def _format_rate(rate: float | None) -> str:
    return "-" if rate is None else f"{rate:.1f}"


def load_defaults(yaml_path: str | Path | None = None) -> dict:
    """`locator.yaml` の `locator_node` からデモの既定値を読む。"""
    if yaml_path is None:
        from ament_index_python.packages import get_package_share_directory

        yaml_path = Path(get_package_share_directory("ss_locator")) / "config" / "locator.yaml"
    data = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8"))
    params = data["locator_node"]["ros__parameters"]
    return {
        name: params[name]
        for name in ("model_file", "model_sha256", "detect_imgsz", "detect_conf")
    }


def run_demo(
    source,
    detector,
    show,
    conf,
    max_frames=None,
    clock=time.monotonic,
) -> None:
    """`source` から最新のコマを取り、推定してそのコマの上に描く。"""
    inferred = 0
    frame_no = -1
    last_dropped = 0
    last_received = 0
    out_meter = RateMeter()
    in_meter = RateMeter()

    while max_frames is None or inferred < max_frames:
        frame = source.take(TAKE_TIMEOUT_S)
        if frame is None:
            break

        dropped = int(getattr(source, "dropped", 0))
        frame_no += (dropped - last_dropped) + 1
        last_dropped = dropped

        src_h, src_w = frame.shape[:2]
        canvas, _, _ = fit_to_canvas(frame)
        started = clock()
        keypoints, confidences, boxes = detector.detect_with_boxes(canvas)
        infer_ms = int(round((clock() - started) * 1000))

        now = clock()
        out_meter.tick(now)
        fps = out_meter.rate(now)

        received = getattr(source, "received", None)
        if received is None:
            in_fps = None
        else:
            in_meter.tick(now, int(received) - last_received)
            last_received = int(received)
            in_fps = in_meter.rate(now)

        lugs = int(len(confidences))
        info = [
            f"lugs: {lugs}",
            f"infer: {infer_ms} ms",
            f"conf >= {float(conf):g}",
            f"dropped: {dropped}",
            f"fps: {_format_rate(fps)}",
            f"in fps: {_format_rate(in_fps)}",
            f"src: {src_w}x{src_h}",
        ]
        image = draw_lugs(canvas, boxes, keypoints, confidences, info)
        result = {
            "frame": frame_no,
            "lugs": lugs,
            "infer_ms": infer_ms,
            "dropped": dropped,
            "fps": fps,
            "in_fps": in_fps,
            "src_size": (src_w, src_h),
            "boxes": boxes,
            "keypoints": keypoints,
            "confidences": confidences,
        }
        if show(image, result) is False:
            break
        inferred += 1


class _FolderSource:
    """フォルダ直下の画像を名前順に 1 枚ずつ返す。捨てない。"""

    def __init__(self, folder: Path) -> None:
        self._paths = sorted(
            (
                path
                for path in folder.iterdir()
                if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
            ),
            key=lambda path: path.name,
        )
        self._index = 0
        self.dropped = 0

    def take(self, timeout_s: float):
        while self._index < len(self._paths):
            path = self._paths[self._index]
            self._index += 1
            image = cv2.imread(str(path))
            if image is None:
                print(f"skip unreadable image: {path}", file=sys.stderr)
                continue
            return image
        return None

    def close(self) -> None:
        return


class _LatestSource:
    """別スレッドで読み続け、最新の 1 コマだけを渡す。"""

    def __init__(self, capture, fps=None) -> None:
        self._capture = capture
        self._frames = LatestFrame()
        self._eof = threading.Event()
        self._stop = threading.Event()
        self._fps = fps
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    @property
    def dropped(self) -> int:
        return self._frames.dropped

    @property
    def received(self) -> int:
        return self._frames.received

    def _read(self) -> None:
        interval = 1.0 / self._fps if self._fps and self._fps > 0 else 0.0
        next_at = time.monotonic()
        while not self._stop.is_set():
            ok, frame = self._capture.read()
            if not ok:
                break
            self._frames.put(frame)
            if interval:
                next_at += interval
                delay = next_at - time.monotonic()
                if delay > 0:
                    self._stop.wait(delay)
                else:
                    next_at = time.monotonic()
        self._eof.set()

    def take(self, timeout_s: float):
        while True:
            frame = self._frames.take(timeout_s)
            if frame is not None:
                return frame
            if self._eof.is_set():
                return None

    def close(self) -> None:
        self._stop.set()
        self._capture.release()
        self._frames.close()


def _open_source(spec: str):
    if spec.startswith("usb:"):
        index = int(spec.split(":", 1)[1])
        capture = cv2.VideoCapture(index)
        if not capture.isOpened():
            raise RuntimeError(f"cannot open USB camera: {spec}")
        return _LatestSource(capture)

    path = Path(spec)
    if path.is_dir():
        return _FolderSource(path)

    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open video: {spec}")
    fps = capture.get(cv2.CAP_PROP_FPS)
    return _LatestSource(capture, fps=fps)


def _parse_args(argv):
    parser = argparse.ArgumentParser(prog="lug_demo")
    parser.add_argument("--source", required=True)
    parser.add_argument("--conf", type=float, default=None)
    parser.add_argument("--model", default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=None)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    defaults = load_defaults()
    conf = float(args.conf if args.conf is not None else defaults["detect_conf"])
    model_file = args.model if args.model is not None else defaults["model_file"]

    try:
        source = _open_source(args.source)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    try:
        detector = Detector(
            model_file, defaults["model_sha256"], defaults["detect_imgsz"], conf
        )
    except BaseException:
        source.close()
        raise

    if args.headless:

        def show(image, result) -> bool:
            print(
                f"frame={result['frame']} lugs={result['lugs']} "
                f"infer_ms={result['infer_ms']} dropped={result['dropped']}",
                flush=True,
            )
            return True

    else:

        def show(image, result) -> bool:
            cv2.imshow("lug_demo", image)
            key = cv2.waitKey(1) & 0xFF
            return key not in (ord("q"), 27)

    try:
        run_demo(source, detector, show, conf, args.max_frames)
    finally:
        source.close()
        if not args.headless:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())

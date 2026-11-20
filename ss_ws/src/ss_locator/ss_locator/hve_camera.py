"""hve-system のカメラモジュールとつなぐ（WebSocket と静止画の HTTP）。純関数は `hve_camera_core.py`。

WebSocket は `HveLift` と同じ作り（自分のスレッド 1 本で繋ぎ、`CameraLink` が返した文を送り、
受けた `state` を渡す）。静止画は別のスレッドで取り、結果を呼び出し側の関数へ渡す
（ノードの周期を止めない）。向きは `state` ではなく応答ヘッダの値を使う。
詳細設計: `docs/plan/detailed/DetailedDesign-hve.md` §7.1・§7.2・§7.4、
`docs/plan/detailed/DetailedDesign-names.md` §10。
"""
from __future__ import annotations

import asyncio
import json
import math
import threading
import time
import urllib.error
import urllib.request

import cv2
import numpy as np
import websockets

from .hve_camera_core import CameraLink, frame_verdict
from .locate_core import CameraPoint, CameraRelease

CAMERA_WS_MODULE_PATH = "/ws/module"
CAMERA_SNAPSHOT_PATH = "/api/snapshot"

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
CAMERA_MODULE_NAME = "hve-cam"

_REQUEST_TIMEOUT_S = 3.0


def host_from_modules(modules):
    """昇降部の `state.modules` から `hve-cam` の `ip` を返す。無ければ `None`。

    `modules` は名前をキーにした dict（`{"hve-cam": {"ip": ...}}`）か、
    `name` を持つ dict の並びのどちらでもよい。
    """
    entry = None
    if isinstance(modules, dict):
        entry = modules.get(CAMERA_MODULE_NAME)
    elif isinstance(modules, (list, tuple)):
        for item in modules:
            if isinstance(item, dict) and item.get("name") == CAMERA_MODULE_NAME:
                entry = item
                break
    if not isinstance(entry, dict):
        return None
    ip = entry.get("ip")
    if isinstance(ip, str) and ip.strip():
        return ip.strip()
    return None


def _header_float(headers, name):
    target = name.lower()
    for key, value in headers.items():
        if str(key).lower() == target:
            try:
                number = float(value)
            except (TypeError, ValueError):
                return None
            return number if math.isfinite(number) else None
    return None


class HveCamera:
    """hve-system のカメラモジュールにつなぐ。`FakeCamera` と差し替えられる形。"""

    def __init__(
        self,
        host="",
        lift=None,
        cmd_period_ms=100,
        state_timeout_ms=600,
        snapshot_min_interval_ms=200,
        image_timeout_s=5.0,
        fw=None,
        link_grace_ms=0,
    ):
        self._host = str(host or "").strip()
        self._lift = lift  # `modules()` を持つもの（`HveLift`）。`host` が空のときだけ使う
        self._link = CameraLink(
            cmd_period_ms=cmd_period_ms,
            state_timeout_ms=state_timeout_ms,
            fw=fw,
            link_grace_ms=link_grace_ms,
        )
        self._cmd_period_s = max(0.001, float(cmd_period_ms) / 1000.0)
        self._retry_s = self._cmd_period_s * 10.0
        self._snapshot_min_interval_s = max(0.0, float(snapshot_min_interval_ms) / 1000.0)
        self._image_timeout_s = float(image_timeout_s)
        self._lock = threading.RLock()
        self._loop = None
        self._ws = None
        self._active_host = None
        self._closed = False

        self._latest_boot = None
        self._cmd_boot = None
        self._cmd_boot_pending = False
        self._last_request_at = None

        self._thread = threading.Thread(
            target=self._run, name="hve_camera", daemon=True
        )
        self._thread.start()

    # ------------------------------------------------------------------ 外から
    def point(self, yaw_rad, pitch_down_rad):
        """向ける。`yaw_rad` が `None` ならヨーは動かさない。"""
        now = time.monotonic()
        with self._lock:
            self._cmd_boot = self._latest_boot
            self._cmd_boot_pending = self._latest_boot is None
            self._link.on_command(CameraPoint(yaw_rad, pitch_down_rad), now)
            messages, _ = self._link.poll(now)
            loop = self._loop
            ws = self._ws
        self._send(messages, loop, ws)

    def release(self):
        """持ち主を離す。その場で `release` を送る。"""
        now = time.monotonic()
        with self._lock:
            self._link.on_command(CameraRelease(), now)
            messages, _ = self._link.poll(now)
            loop = self._loop
            ws = self._ws
        self._send(messages, loop, ws)

    def state(self):
        """今の `CameraState` を返す。"""
        with self._lock:
            return self._link.camera_state(time.monotonic())

    def capture(self, on_done):
        """静止画を別スレッドで取り、終わったら `on_done(verdict, result)` を呼ぶ（すぐ戻る）。

        `verdict` は `use`・`pose_unknown`・`fail`。`use` のとき `result` は
        `{image, yaw_rad, pitch_down_rad}`（向きは応答ヘッダの値。ピッチは見下ろしが正）、
        それ以外は `None`。
        """
        start = time.monotonic()
        with self._lock:
            boot = self._cmd_boot if self._cmd_boot is not None else self._latest_boot
        thread = threading.Thread(
            target=self._capture_worker,
            args=(on_done, start, boot),
            name="hve_camera_capture",
            daemon=True,
        )
        thread.start()

    def close(self):
        """命令の途中なら `release` を送ってから閉じる。"""
        now = time.monotonic()
        with self._lock:
            self._link.on_command(CameraRelease(), now)
            messages, _ = self._link.poll(now)
            loop = self._loop
            ws = self._ws
            self._closed = True
        futures = self._send([m for m in messages if m.get("t") == "release"], loop, ws)
        for future in futures:
            try:
                future.result(timeout=1.0)
            except Exception:
                pass
        if loop is not None and ws is not None:
            try:
                asyncio.run_coroutine_threadsafe(ws.close(), loop)
            except Exception:
                pass
        self._thread.join(timeout=3.0)

    # ------------------------------------------------------------------ 静止画
    def _resolve_host(self):
        if self._host:
            return self._host
        lift = self._lift
        if lift is None:
            return None
        try:
            return host_from_modules(lift.modules())
        except Exception:
            return None

    def _capture_worker(self, on_done, start, boot):
        try:
            verdict, result = self._capture_loop(start, boot)
        except Exception:
            verdict, result = "fail", None
        try:
            on_done(verdict, result)
        except Exception:
            pass

    def _capture_loop(self, start, boot):
        deadline = start + self._image_timeout_s
        while not self._closed:
            with self._lock:
                host = self._active_host
            if host is None:
                host = self._resolve_host()
            if host is None:
                return "fail", None

            # 前の要求から snapshot_min_interval_ms 以上空ける
            with self._lock:
                last = self._last_request_at
                now = time.monotonic()
                wait = 0.0 if last is None else last + self._snapshot_min_interval_s - now
                self._last_request_at = now + max(0.0, wait)
            if wait > 0.0:
                time.sleep(wait)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return "fail", None

            status, headers, body = self._fetch(host, min(_REQUEST_TIMEOUT_S, remaining))
            if status is None:
                # 繋がらない（無線の途切れなど）。期限までは少し待って撮り直す
                if time.monotonic() + 0.3 >= deadline:
                    return "fail", None
                time.sleep(0.3)
                continue
            verdict = frame_verdict(status, headers, boot, None)
            if verdict == "use":
                return self._decode(headers, body)
            if verdict == "retry":
                if time.monotonic() >= deadline:
                    return "fail", None
                continue
            return verdict, None  # pose_unknown / fail
        return "fail", None

    def _fetch(self, host, timeout_s):
        """`(番号, ヘッダ, 本文)`。繋がらないときは `(None, {}, b"")`。"""
        url = "http://" + host + CAMERA_SNAPSHOT_PATH + "?by=ss"
        try:
            with urllib.request.urlopen(url, timeout=max(0.1, timeout_s)) as response:
                return response.status, dict(response.headers.items()), response.read()
        except urllib.error.HTTPError as error:
            try:
                return error.code, dict(error.headers.items()), b""
            finally:
                error.close()
        except Exception:
            return None, {}, b""

    def _decode(self, headers, body):
        yaw_deg = _header_float(headers, "X-Yaw-Deg")
        pitch_deg = _header_float(headers, "X-Pitch-Deg")
        if yaw_deg is None or pitch_deg is None:
            return "fail", None
        image = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (IMAGE_HEIGHT, IMAGE_WIDTH):
            return "fail", None
        return "use", {
            "image": image,
            "yaw_rad": math.radians(yaw_deg),
            # hve は見上げが正、本システムは見下ろしが正
            "pitch_down_rad": -math.radians(pitch_deg) + 0.0,
        }

    # ------------------------------------------------------------------ スレッド
    def _run(self):
        try:
            asyncio.run(self._main())
        except Exception:
            pass

    async def _sleep(self, seconds):
        end = time.monotonic() + seconds
        while not self._closed:
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                return
            await asyncio.sleep(min(0.05, remaining))

    async def _main(self):
        while not self._closed:
            host = self._resolve_host()
            if host is None:
                await self._sleep(self._cmd_period_s)
                continue
            try:
                url = "ws://" + host + CAMERA_WS_MODULE_PATH
                async with websockets.connect(url, open_timeout=5.0) as ws:
                    loop = asyncio.get_running_loop()
                    with self._lock:
                        self._loop = loop
                        self._ws = ws
                        self._active_host = host
                        now = time.monotonic()
                        self._link.on_connected(now)
                        messages, _ = self._link.poll(now)
                    for message in messages:
                        await ws.send(json.dumps(message, ensure_ascii=False))

                    receiver = asyncio.create_task(self._receive(ws))
                    while not self._closed:
                        await asyncio.sleep(self._cmd_period_s)
                        if receiver.done():
                            break
                        with self._lock:
                            if self._ws is not ws:
                                break
                            messages, reconnect = self._link.poll(time.monotonic())
                        for message in messages:
                            await ws.send(json.dumps(message, ensure_ascii=False))
                        if reconnect:
                            break

                    receiver.cancel()
                    try:
                        await receiver
                    except (asyncio.CancelledError, Exception):
                        pass
            except asyncio.CancelledError:
                raise
            except Exception:
                pass
            finally:
                with self._lock:
                    self._loop = None
                    self._ws = None
                    self._active_host = None
                    self._link.on_closed(time.monotonic())
            await self._sleep(self._retry_s)

    async def _receive(self, ws):
        try:
            async for raw in ws:
                try:
                    message = json.loads(raw)
                except Exception:
                    continue
                with self._lock:
                    self._link.on_state(message, time.monotonic())
                    if isinstance(message, dict) and message.get("t") == "state":
                        self._latest_boot = message.get("boot_id")
                        if self._cmd_boot_pending and self._latest_boot is not None:
                            self._cmd_boot = self._latest_boot
                            self._cmd_boot_pending = False
        except asyncio.CancelledError:
            raise
        except Exception:
            pass

    def _send(self, messages, loop, ws):
        futures = []
        if not messages or loop is None or ws is None:
            return futures
        for message in messages:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    ws.send(json.dumps(message, ensure_ascii=False)), loop
                )
                future.add_done_callback(lambda f: f.exception())
                futures.append(future)
            except Exception:
                pass
        return futures

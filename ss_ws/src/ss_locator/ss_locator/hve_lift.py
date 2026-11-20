"""hve-system の昇降部とつなぐ（WebSocket）。純関数は `hve_lift_core.py`。

自分のスレッド 1 本で繋ぎ、`LiftLink` が返した文を送り、受けた `state` を渡す。
`FakeLift` と同じ `apply`・`advance` を持ち、`locator_node` は差し替えるだけで使える。
詳細設計: `docs/plan/detailed/DetailedDesign-hve.md` §2.2・§5、
`docs/plan/detailed/DetailedDesign-names.md` §7。
"""
from __future__ import annotations

import asyncio
import json
import threading
import time
import urllib.request

import websockets

from .hve_lift_core import LiftLink
from .locate_core import LiftRelease

LIFT_WS_MODULE_PATH = "/ws/module"
LIFT_SETTINGS_PATH = "/api/settings"

_SETTINGS_TIMEOUT_S = 1.0


def _clamped_init(axis):
    """設定の 1 軸（`min`・`max`・`init`）から、`min`〜`max` に収めた `init` を返す。"""
    lo, hi, init = axis["min"], axis["max"], axis["init"]
    for value in (lo, hi, init):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError("設定の値が数でない")
    return int(round(max(lo, min(hi, init))))


def speeds_from_settings(body):
    """`GET /api/settings` の本文（dict）から (上昇, 下降) の速さ [mm/s] を返す。読めなければ例外。"""
    settings = body.get("settings", body)
    return _clamped_init(settings["lift_up"]), _clamped_init(settings["lift_down"])


class HveLift:
    """hve-system の昇降部（`/ws/module`）につなぐ。"""

    def __init__(self, host, cmd_period_ms=100, state_timeout_ms=600, fw=None,
                 link_grace_ms=0):
        self._host = str(host)
        self._url = "ws://" + self._host + LIFT_WS_MODULE_PATH
        self._settings_url = "http://" + self._host + LIFT_SETTINGS_PATH
        self._link = LiftLink(
            cmd_period_ms=cmd_period_ms,
            state_timeout_ms=state_timeout_ms,
            fw=fw,
            link_grace_ms=link_grace_ms,
        )
        self._cmd_period_s = max(0.001, float(cmd_period_ms) / 1000.0)
        self._retry_s = self._cmd_period_s * 10.0
        self._lock = threading.RLock()
        self._loop = None
        self._ws = None
        self._closed = False
        self._modules = None
        self._thread = threading.Thread(
            target=self._run, name="hve_lift", daemon=True
        )
        self._thread.start()

    # ------------------------------------------------------------------ 外から
    def apply(self, command):
        """`LiftGoto`／`LiftRelease` を受ける。`LiftRelease` はその場で送る。"""
        now = time.monotonic()
        with self._lock:
            self._link.on_command(command, now)
            messages, _ = self._link.poll(now)
            loop = self._loop
            ws = self._ws
        self._send(messages, loop, ws)

    def advance(self, dt_s):
        """今の `LiftState` を返す（`dt_s` は使わない）。"""
        with self._lock:
            return self._link.lift_state(time.monotonic())

    def modules(self):
        """最後に受けた `state` の `modules`（読み取り専用。無ければ `None`）。"""
        with self._lock:
            return self._modules

    def close(self):
        """命令の途中なら `release` を送ってから閉じる。"""
        now = time.monotonic()
        with self._lock:
            self._link.on_command(LiftRelease(), now)
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

    # ------------------------------------------------------------------ スレッド
    def _run(self):
        try:
            asyncio.run(self._main())
        except Exception:
            pass

    def _fetch_speeds(self):
        """設定を読んで (上昇, 下降)。取れなければ `None`。"""
        try:
            with urllib.request.urlopen(
                self._settings_url, timeout=_SETTINGS_TIMEOUT_S
            ) as response:
                body = json.loads(response.read().decode("utf-8"))
            return speeds_from_settings(body)
        except Exception:
            return None

    async def _refresh_speeds(self):
        speeds = await asyncio.to_thread(self._fetch_speeds)
        with self._lock:
            if speeds is None:
                self._link.set_speeds(None, None)
            else:
                self._link.set_speeds(*speeds)
        return speeds is not None

    async def _sleep(self, seconds):
        end = time.monotonic() + seconds
        while not self._closed:
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                return
            await asyncio.sleep(min(0.05, remaining))

    async def _main(self):
        while not self._closed:
            try:
                # 繋ぐたびに速さを読み直す（取れなければ、繋いだあとも周期的に試す）
                have_speeds = await self._refresh_speeds()
                last_try = time.monotonic()
                async with websockets.connect(self._url, open_timeout=5.0) as ws:
                    loop = asyncio.get_running_loop()
                    with self._lock:
                        self._loop = loop
                        self._ws = ws
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
                        if (
                            not have_speeds
                            and time.monotonic() - last_try >= self._retry_s
                        ):
                            have_speeds = await self._refresh_speeds()
                            last_try = time.monotonic()
                        with self._lock:
                            if self._ws is not ws:
                                break
                            messages, reconnect = self._link.poll(
                                time.monotonic()
                            )
                        for message in messages:
                            await ws.send(
                                json.dumps(message, ensure_ascii=False)
                            )
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
                    self._modules = None
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
                        self._modules = message.get("modules")
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

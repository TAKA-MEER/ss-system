"""hve-system の昇降部とつなぐ（WebSocket）。純関数は `hve_lift_core.py`。

自分のスレッド 1 本で繋ぎ、`LiftLink` が返した文を送り、受けた `state` を渡す。
`FakeLift` と同じ `apply`・`advance` を持ち、`locator_node` は差し替えるだけで使える。
詳細設計: `docs/plan/detailed/DetailedDesign-hve.md` §5、
`docs/plan/detailed/DetailedDesign-names.md` §7。
"""
from __future__ import annotations

import asyncio
import json
import threading
import time

import websockets

from .hve_lift_core import LiftLink
from .locate_core import LiftRelease

LIFT_WS_MODULE_PATH = "/ws/module"


class HveLift:
    """hve-system の昇降部（`/ws/module`）につなぐ。"""

    def __init__(
        self,
        host,
        speed_mm_s=40,
        cmd_period_ms=100,
        state_timeout_ms=600,
        fw=None,
    ):
        self._url = "ws://" + str(host) + LIFT_WS_MODULE_PATH
        self._link = LiftLink(
            speed_mm_s=speed_mm_s,
            cmd_period_ms=cmd_period_ms,
            state_timeout_ms=state_timeout_ms,
            fw=fw,
        )
        self._cmd_period_s = max(0.001, float(cmd_period_ms) / 1000.0)
        self._lock = threading.RLock()
        self._loop = None
        self._ws = None
        self._closed = False
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

    def close(self):
        """命令の途中なら `release` を送ってから閉じる。"""
        now = time.monotonic()
        with self._lock:
            self._link.on_command(LiftRelease(), now)
            messages, _ = self._link.poll(now)
            loop = self._loop
            ws = self._ws
            self._closed = True
        self._send([m for m in messages if m.get("t") == "release"], loop, ws)
        if loop is not None and ws is not None:
            try:
                asyncio.run_coroutine_threadsafe(ws.close(), loop)
            except Exception:
                pass
        self._thread.join(timeout=2.0)

    # ------------------------------------------------------------------ スレッド
    def _run(self):
        try:
            asyncio.run(self._main())
        except Exception:
            pass

    async def _main(self):
        while not self._closed:
            try:
                async with websockets.connect(self._url, open_timeout=5.0) as ws:
                    loop = asyncio.get_running_loop()
                    with self._lock:
                        self._loop = loop
                        self._ws = ws
                        self._link.on_connected(time.monotonic())
                        messages, _ = self._link.poll(time.monotonic())
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
                    self._link.on_closed(time.monotonic())
            if self._closed:
                break
            await asyncio.sleep(self._cmd_period_s * 10.0)

    async def _receive(self, ws):
        try:
            async for raw in ws:
                try:
                    message = json.loads(raw)
                except Exception:
                    continue
                with self._lock:
                    self._link.on_state(message, time.monotonic())
        except asyncio.CancelledError:
            raise
        except Exception:
            pass

    def _send(self, messages, loop, ws):
        if not messages or loop is None or ws is None:
            return
        for message in messages:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    ws.send(json.dumps(message, ensure_ascii=False)), loop
                )
                future.add_done_callback(lambda f: f.exception())
            except Exception:
                pass

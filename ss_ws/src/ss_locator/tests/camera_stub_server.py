"""試験の中だけで使う小さなサーバ（偽のカメラモジュール）。

`127.0.0.1` の空いているポートで待つ。`/ws/module`（WebSocket）では受けた文を覚え、
渡された `state` を送る。`/api/snapshot`（HTTP）では、渡された応答（番号・ヘッダ・本文）を
順に返す（最後の 1 つは繰り返す）。同じポートで両方に答える（`lift_stub_server.py` と同じ作り）。
`ceiling_sensor` を名乗る接続は扱わない。
"""
from __future__ import annotations

import asyncio
import json
import threading
import time
from http import HTTPStatus

import websockets


class CameraStubServer:
    """試験プロセスの中で起こす偽のカメラモジュール。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._connections = []
        self._clients = set()
        self._auto_state = None
        self._arrive_on_aim = False
        self._last_aim = None
        self._snapshots = []
        self._snapshot_index = 0
        self._snapshot_requests = []
        self._state_period_s = 0.05
        self._port = None
        self._loop = None
        self._stop_event = None
        self._thread = None
        self._ready = threading.Event()

    @property
    def port(self):
        return self._port

    @property
    def connections(self):
        """接続ごとの、受けた文の並び（接続した順）。"""
        with self._lock:
            return [list(messages) for messages in self._connections]

    @property
    def messages(self):
        with self._lock:
            return [message for messages in self._connections for message in messages]

    @property
    def snapshot_requests(self):
        """`(受けた時刻 time.monotonic(), パスとクエリ)` の並び。"""
        with self._lock:
            return list(self._snapshot_requests)

    def start(self):
        self._thread = threading.Thread(
            target=self._run, name="camera_stub_server", daemon=True
        )
        self._thread.start()
        if not self._ready.wait(5.0):
            raise AssertionError("camera stub server が起動しない")
        return self._port

    def stop(self):
        if self._loop is not None and self._stop_event is not None:
            try:
                self._loop.call_soon_threadsafe(self._stop_event.set)
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    def set_auto_state(self, state, period_s=0.05, arrive_on_aim=False):
        """同じ `state` を周期的に送る。`None` で止める。

        `arrive_on_aim` が true なら、受けた最後の `aim` を `ARRIVED` にして `state.aim` へ載せる。
        """
        with self._lock:
            self._auto_state = None if state is None else dict(state)
            self._state_period_s = float(period_s)
            self._arrive_on_aim = bool(arrive_on_aim)

    def set_snapshots(self, responses):
        """`/api/snapshot` の応答を順に返す。各要素は `(番号, ヘッダの dict, 本文 bytes)`。"""
        with self._lock:
            self._snapshots = [(int(s), dict(h), bytes(b)) for s, h, b in responses]
            self._snapshot_index = 0

    def close_connections(self):
        if self._loop is None:
            return
        try:
            asyncio.run_coroutine_threadsafe(self._close_all(), self._loop)
        except Exception:
            pass

    # ------------------------------------------------------------------ 中身
    def _run(self):
        asyncio.run(self._main())

    async def _main(self):
        self._loop = asyncio.get_running_loop()
        self._stop_event = asyncio.Event()
        server = await websockets.serve(
            self._handler, "127.0.0.1", 0, process_request=self._process_request
        )
        self._port = server.sockets[0].getsockname()[1]
        state_task = asyncio.create_task(self._state_loop())
        self._ready.set()
        try:
            await self._stop_event.wait()
        finally:
            state_task.cancel()
            server.close()
            await server.wait_closed()
            await self._close_all()

    def _process_request(self, connection, request):
        """WebSocket でない `GET /api/snapshot` に答える。"""
        if not request.path.startswith("/api/snapshot"):
            return None
        with self._lock:
            self._snapshot_requests.append((time.monotonic(), request.path))
            if not self._snapshots:
                status, headers, body = 503, {}, b""
            else:
                status, headers, body = self._snapshots[
                    min(self._snapshot_index, len(self._snapshots) - 1)
                ]
                self._snapshot_index += 1
        response = connection.respond(HTTPStatus(status), "")
        response.body = body
        del response.headers["Content-Length"]
        response.headers["Content-Length"] = str(len(body))
        for key, value in headers.items():
            if key in response.headers:
                del response.headers[key]
            response.headers[key] = str(value)
        return response

    async def _state_loop(self):
        while True:
            await asyncio.sleep(self._state_period_s)
            with self._lock:
                state = None if self._auto_state is None else dict(self._auto_state)
                if state is not None and self._arrive_on_aim and self._last_aim:
                    aim = self._last_aim
                    state["aim"] = {
                        "by": "ss",
                        "press": aim.get("press"),
                        "pitch_deg": aim.get("pitch_deg"),
                        "yaw_deg": aim.get("yaw_deg"),
                        "status": "ARRIVED",
                        "reason": "NONE",
                    }
            if state is not None:
                await self._broadcast(state)

    async def _handler(self, ws):
        messages = []
        with self._lock:
            self._connections.append(messages)
            self._clients.add(ws)
        try:
            async for raw in ws:
                try:
                    message = json.loads(raw)
                except Exception:
                    continue
                with self._lock:
                    messages.append(message)
                    if message.get("t") == "aim":
                        self._last_aim = message
        except Exception:
            pass
        finally:
            with self._lock:
                self._clients.discard(ws)

    async def _broadcast(self, state):
        with self._lock:
            clients = list(self._clients)
        for ws in clients:
            try:
                await ws.send(json.dumps(state, ensure_ascii=False))
            except Exception:
                pass

    async def _close_all(self):
        with self._lock:
            clients = list(self._clients)
        for ws in clients:
            try:
                await ws.close()
            except Exception:
                pass

"""試験の中だけで使う小さな WebSocket のサーバ（偽の昇降部）。

`127.0.0.1` の空いているポートで待ち、受けた文を覚え、渡された `state` を送る。
天井の値は扱わない（`ceiling_sensor: true` を名乗る試験は作らない）。
"""
from __future__ import annotations

import asyncio
import json
import threading
from http import HTTPStatus

import websockets


class LiftStubServer:
    """試験プロセスの中で起こす偽の昇降部。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._connections = []
        self._clients = set()
        self._auto_state = None
        self._settings = None
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

    def start(self):
        self._thread = threading.Thread(
            target=self._run, name="lift_stub_server", daemon=True
        )
        self._thread.start()
        if not self._ready.wait(5.0):
            raise AssertionError("lift stub server が起動しない")
        return self._port

    def stop(self):
        if self._loop is not None and self._stop_event is not None:
            try:
                self._loop.call_soon_threadsafe(self._stop_event.set)
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    def set_auto_state(self, state, period_s=0.05):
        """同じ `state` を周期的に送る。`None` で止める。"""
        with self._lock:
            self._auto_state = None if state is None else dict(state)
            self._state_period_s = float(period_s)

    def set_settings(self, settings):
        """`GET /api/settings` の `settings` を渡す。`None` なら 404 を返す（取れない）。"""
        with self._lock:
            self._settings = None if settings is None else json.loads(json.dumps(settings))

    def send_state(self, state):
        if self._loop is None:
            return
        try:
            asyncio.run_coroutine_threadsafe(
                self._broadcast(dict(state)), self._loop
            )
        except Exception:
            pass

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
        """WebSocket でない `GET /api/settings` に答える。"""
        if request.path == "/api/settings":
            with self._lock:
                settings = self._settings
            if settings is None:
                return connection.respond(HTTPStatus.NOT_FOUND, "no settings")
            response = connection.respond(HTTPStatus.OK, json.dumps({"settings": settings}))
            response.headers["Content-Type"] = "application/json"
            return response
        return None

    async def _state_loop(self):
        while True:
            await asyncio.sleep(self._state_period_s)
            with self._lock:
                state = None if self._auto_state is None else dict(self._auto_state)
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

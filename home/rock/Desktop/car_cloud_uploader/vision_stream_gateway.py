#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


ROOT_DIR = Path(__file__).resolve().parent
TEST_SOURCE = ROOT_DIR / "ros2_tools" / "vision_stream_test_source.py"


class StreamState:
    def __init__(self, max_clients: int):
        self.condition = threading.Condition()
        self.active = False
        self.paused = False
        self.mode = "idle"
        self.session_id = ""
        self.stage = ""
        self.pause_reason = ""
        self.jpeg = b""
        self.sequence = 0
        self.started_at = 0.0
        self.frame_at = 0.0
        self.detection_age_s: float | None = None
        self.clients = 0
        self.max_clients = max(1, int(max_clients))
        self.fps_target = 5.0
        self.width = 1280
        self.height = 720
        self.frame_times: list[float] = []
        self.test_process: subprocess.Popen[Any] | None = None

    def status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.condition:
            recent = [value for value in self.frame_times if now - value <= 2.0]
            fps = 0.0 if len(recent) < 2 else (len(recent) - 1) / max(0.001, recent[-1] - recent[0])
            return {
                "ok": True,
                "active": self.active,
                "paused": self.paused,
                "mode": self.mode,
                "session_id": self.session_id,
                "stage": self.stage,
                "frame_age_s": None if self.frame_at <= 0 else round(now - self.frame_at, 3),
                "detection_age_s": self.detection_age_s,
                "fps": round(fps, 2),
                "fps_target": self.fps_target,
                "width": self.width,
                "height": self.height,
                "clients": self.clients,
                "max_clients": self.max_clients,
            }

    def begin(self, payload: dict[str, Any]) -> None:
        with self.condition:
            self.active = True
            self.paused = False
            self.mode = str(payload.get("mode") or "inspection")
            self.session_id = str(payload.get("session_id") or "")
            self.fps_target = float(payload.get("fps_target") or 5.0)
            self.stage = ""
            self.pause_reason = ""
            self.jpeg = b""
            self.started_at = time.monotonic()
            self.frame_at = 0.0
            self.detection_age_s = None
            self.width = 1280
            self.height = 720
            self.frame_times = []
            self.condition.notify_all()

    def end(self, session_id: str) -> None:
        with self.condition:
            if session_id and self.session_id and session_id != self.session_id:
                return
            self.active = False
            self.paused = False
            self.mode = "idle"
            self.session_id = ""
            self.stage = ""
            self.jpeg = b""
            self.frame_at = 0.0
            self.detection_age_s = None
            self.frame_times = []
            self.condition.notify_all()

    def set_frame(self, jpeg: bytes, headers: Any) -> bool:
        session_id = str(headers.get("X-Session-Id") or "")
        with self.condition:
            if not self.active or self.paused or session_id != self.session_id:
                return False
            now = time.monotonic()
            self.jpeg = jpeg
            self.sequence += 1
            self.frame_at = now
            self.stage = str(headers.get("X-Stage") or "")
            raw_age = str(headers.get("X-Detection-Age") or "").strip()
            try:
                self.detection_age_s = float(raw_age) if raw_age else None
            except ValueError:
                self.detection_age_s = None
            for attribute, header in (("width", "X-Frame-Width"), ("height", "X-Frame-Height")):
                try:
                    dimension = int(headers.get(header) or 0)
                except (TypeError, ValueError):
                    dimension = 0
                if 16 <= dimension <= 8192:
                    setattr(self, attribute, dimension)
            self.frame_times.append(now)
            self.frame_times = self.frame_times[-20:]
            self.condition.notify_all()
            return True


class PublicHandler(BaseHTTPRequestHandler):
    server_version = "CarVision/1.0"

    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Private-Network", "true")
        self.send_header("Cross-Origin-Resource-Policy", "cross-origin")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")

    def _json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        state: StreamState = self.server.state
        if self.path.split("?", 1)[0] in {"/healthz", "/status"}:
            self._json(200, state.status())
            return
        if self.path.split("?", 1)[0] == "/snapshot.jpg":
            with state.condition:
                jpeg = state.jpeg
                fresh = state.active and state.frame_at > 0 and time.monotonic() - state.frame_at <= 1.0
            if not jpeg or not fresh:
                self._json(404, {"ok": False, "error": "frame unavailable"})
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self._cors()
            self.end_headers()
            self.wfile.write(jpeg)
            return
        if self.path.split("?", 1)[0] != "/stream.mjpg":
            self._json(404, {"ok": False, "error": "not found"})
            return
        with state.condition:
            if not state.active:
                self._json(409, {"ok": False, "error": "stream inactive"})
                return
            if state.clients >= state.max_clients:
                self._json(503, {"ok": False, "error": "client limit reached"})
                return
            state.clients += 1
        try:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self._cors()
            self.end_headers()
            sequence = -1
            idle_since = 0.0
            while True:
                with state.condition:
                    state.condition.wait_for(lambda: state.sequence != sequence or not state.active, timeout=1.0)
                    if not state.active:
                        break
                    jpeg = state.jpeg
                    current = state.sequence
                    fresh = state.frame_at > 0 and time.monotonic() - state.frame_at <= 1.0
                if not jpeg or current == sequence or not fresh:
                    idle_since = idle_since or time.monotonic()
                    if time.monotonic() - idle_since > 15.0:
                        break
                    continue
                idle_since = 0.0
                sequence = current
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            with state.condition:
                state.clients = max(0, state.clients - 1)

    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self._cors()
        self.end_headers()

    def log_message(self, _format: str, *_args: Any) -> None:
        return


class ControlHandler(BaseHTTPRequestHandler):
    def _reply(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json_body(self) -> dict[str, Any]:
        length = min(65536, int(self.headers.get("Content-Length") or 0))
        if length <= 0:
            return {}
        value = json.loads(self.rfile.read(length).decode("utf-8"))
        return value if isinstance(value, dict) else {}

    def _authorized(self) -> bool:
        return str(self.headers.get("X-Control-Token") or "") == self.server.control_token

    def _stop_test(self) -> None:
        state: StreamState = self.server.state
        process = state.test_process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        state.test_process = None
        if state.mode == "test":
            state.end("")

    def do_POST(self) -> None:
        if not self._authorized():
            self._reply(401, {"ok": False, "error": "unauthorized"})
            return
        state: StreamState = self.server.state
        path = self.path.split("?", 1)[0]
        if path == "/frame":
            length = min(2 * 1024 * 1024, int(self.headers.get("Content-Length") or 0))
            jpeg = self.rfile.read(length)
            ok = bool(jpeg) and state.set_frame(jpeg, self.headers)
            self._reply(200 if ok else 409, {"ok": ok})
            return
        try:
            payload = self._json_body()
            if path == "/session/start":
                requested_mode = str(payload.get("mode") or "inspection")
                if requested_mode == "inspection":
                    self._stop_test()
                elif state.active and state.mode == "inspection":
                    raise RuntimeError("inspection is active")
                state.begin(payload)
            elif path == "/session/end":
                state.end(str(payload.get("session_id") or ""))
            elif path == "/session/pause":
                with state.condition:
                    state.paused = True
                    state.pause_reason = str(payload.get("reason") or "")
            elif path == "/session/resume":
                with state.condition:
                    state.paused = False
                    state.pause_reason = ""
            elif path == "/test/start":
                if state.active and state.mode == "inspection":
                    raise RuntimeError("inspection is active")
                self._stop_test()
                timeout_s = max(5, min(120, int(payload.get("timeout_s") or 120)))
                state.test_process = subprocess.Popen(
                    [sys.executable, str(TEST_SOURCE), "--timeout-s", str(timeout_s)],
                    cwd=str(ROOT_DIR),
                    start_new_session=True,
                )
            elif path == "/test/stop":
                self._stop_test()
            else:
                self._reply(404, {"ok": False, "error": "not found"})
                return
            self._reply(200, {**state.status(), "ok": True})
        except Exception as exc:
            self._reply(409, {"ok": False, "error": str(exc)})

    def log_message(self, _format: str, *_args: Any) -> None:
        return


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--public-host", default="192.168.1.1")
    parser.add_argument("--public-port", type=int, default=8766)
    parser.add_argument("--control-host", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=8767)
    parser.add_argument("--max-clients", type=int, default=2)
    args = parser.parse_args()
    state = StreamState(args.max_clients)
    public = ThreadingHTTPServer((args.public_host, args.public_port), PublicHandler)
    control = ThreadingHTTPServer((args.control_host, args.control_port), ControlHandler)
    public.state = state
    control.state = state
    control.control_token = os.environ.get("CAR_CLOUD_UPLOAD_TOKEN", "car-cloud-upload")
    threads = [
        threading.Thread(target=public.serve_forever, daemon=True),
        threading.Thread(target=control.serve_forever, daemon=True),
    ]
    for thread in threads:
        thread.start()
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_args: stop.set())
    while not stop.wait(0.5):
        process = state.test_process
        if process is not None and process.poll() is not None:
            state.test_process = None
            if state.mode == "test":
                state.end("")
    public.shutdown()
    control.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

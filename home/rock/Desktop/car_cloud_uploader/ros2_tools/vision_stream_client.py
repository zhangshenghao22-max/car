#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import threading
import time
from typing import Any
from urllib.request import Request, urlopen

import cv2
import numpy as np


CONTROL_URL = os.environ.get("CAR_VISION_STREAM_CONTROL_URL", "http://127.0.0.1:8767").rstrip("/")
CONTROL_TOKEN = os.environ.get("CAR_CLOUD_UPLOAD_TOKEN", "car-cloud-upload")


def control_request(path: str, payload: dict[str, Any] | None = None, timeout: float = 2.0) -> dict[str, Any]:
    body = json.dumps(payload or {}, ensure_ascii=False).encode("utf-8")
    request = Request(
        f"{CONTROL_URL}{path}",
        data=body,
        method="POST",
        headers={"Content-Type": "application/json", "X-Control-Token": CONTROL_TOKEN},
    )
    with urlopen(request, timeout=timeout) as response:
        value = json.loads(response.read().decode("utf-8"))
    return value if isinstance(value, dict) else {}


class VisionStreamPublisher:
    def __init__(self, session_id: str, mode: str, fps: float = 8.0, jpeg_quality: int = 75):
        self.session_id = str(session_id or f"session-{int(time.time())}")
        self.mode = str(mode)
        self.period_s = 1.0 / max(1.0, float(fps))
        self.jpeg_quality = max(40, min(90, int(jpeg_quality)))
        self.active = False
        self.paused = False
        self.last_frame_at = 0.0
        self.last_offer_at = 0.0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._wake_event = threading.Event()
        self._sender: threading.Thread | None = None
        self._annotated_frame: np.ndarray | None = None
        self._clean_frame: np.ndarray | None = None
        self._detection_at: float | None = None
        self._stage = ""

    def start(self) -> None:
        result = control_request(
            "/session/start",
            {"session_id": self.session_id, "mode": self.mode, "fps_target": round(1.0 / self.period_s, 2)},
        )
        if not bool(result.get("ok")):
            raise RuntimeError(str(result.get("error") or "vision stream start failed"))
        with self._lock:
            self.active = True
            self.paused = False
            self.last_frame_at = 0.0
            self.last_offer_at = 0.0
            self._annotated_frame = None
            self._clean_frame = None
            self._detection_at = None
            self._stage = ""
        self._stop_event.clear()
        self._wake_event.clear()
        self._sender = threading.Thread(
            target=self._sender_loop,
            name=f"vision-stream-{self.mode}",
            daemon=True,
        )
        self._sender.start()

    def due(self, *, force: bool = False) -> bool:
        with self._lock:
            return self.active and not self.paused and (
                force or time.monotonic() - self.last_offer_at >= self.period_s
            )

    def publish(
        self,
        frame: np.ndarray,
        *,
        detection_age_s: float | None,
        stage: str = "",
        force: bool = False,
        clean_frame: np.ndarray | None = None,
    ) -> bool:
        if not self.due(force=force):
            return False
        now = time.monotonic()
        with self._lock:
            if not self.active or self.paused:
                return False
            self._annotated_frame = frame.copy()
            self._clean_frame = (clean_frame if clean_frame is not None else frame).copy()
            self._detection_at = None if detection_age_s is None else now - max(0.0, float(detection_age_s))
            self._stage = str(stage)
            self.last_offer_at = now
        self._wake_event.set()
        return True

    def _post_frame(self, frame: np.ndarray, detection_age_s: float | None, stage: str) -> bool:
        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not ok:
            return False
        request = Request(
            f"{CONTROL_URL}/frame",
            data=encoded.tobytes(),
            method="POST",
            headers={
                "Content-Type": "image/jpeg",
                "X-Control-Token": CONTROL_TOKEN,
                "X-Session-Id": self.session_id,
                "X-Detection-Age": "" if detection_age_s is None else f"{float(detection_age_s):.3f}",
                "X-Stage": str(stage),
                "X-Frame-Width": str(int(frame.shape[1])),
                "X-Frame-Height": str(int(frame.shape[0])),
            },
        )
        try:
            with urlopen(request, timeout=1.0) as response:
                response.read(256)
        except Exception:
            return False
        return True

    def _sender_loop(self) -> None:
        next_send_at = time.monotonic()
        while not self._stop_event.is_set():
            wait_s = max(0.0, next_send_at - time.monotonic())
            self._wake_event.wait(wait_s)
            self._wake_event.clear()
            if self._stop_event.is_set():
                break
            now = time.monotonic()
            if now < next_send_at:
                continue
            with self._lock:
                if not self.active:
                    break
                paused = self.paused
                annotated = self._annotated_frame
                clean = self._clean_frame
                detected_at = self._detection_at
                stage = self._stage
            if not paused and annotated is not None:
                detection_age_s = None if detected_at is None else max(0.0, now - detected_at)
                frame = clean if detection_age_s is not None and detection_age_s > 0.6 and clean is not None else annotated
                if self._post_frame(frame, detection_age_s, stage):
                    with self._lock:
                        self.last_frame_at = time.monotonic()
            next_send_at += self.period_s
            now = time.monotonic()
            if next_send_at <= now:
                missed_periods = int((now - next_send_at) / self.period_s) + 1
                next_send_at += missed_periods * self.period_s

    def pause(self, reason: str) -> None:
        with self._lock:
            active = self.active
            self.paused = active
        if active:
            try:
                control_request("/session/pause", {"session_id": self.session_id, "reason": str(reason)})
            except Exception:
                pass

    def resume(self) -> None:
        with self._lock:
            active = self.active
        if active:
            try:
                control_request("/session/resume", {"session_id": self.session_id})
                with self._lock:
                    self.paused = False
                self._wake_event.set()
            except Exception:
                pass

    def stop(self) -> None:
        with self._lock:
            active = self.active
            self.active = False
            self.paused = False
        self._stop_event.set()
        self._wake_event.set()
        sender = self._sender
        if sender is not None and sender.is_alive():
            sender.join(timeout=2.0)
        self._sender = None
        if active:
            try:
                control_request("/session/end", {"session_id": self.session_id})
            except Exception:
                pass

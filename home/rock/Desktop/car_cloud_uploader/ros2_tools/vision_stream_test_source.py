#!/usr/bin/env python3
from __future__ import annotations

import argparse
import fcntl
import json
import signal
import socket
import tempfile
import time
from pathlib import Path
from typing import Any

import cv2
import yaml

from arm_apriltag_s0_search import CameraPipeline
from cabinet_apriltag_inspection import detection_box, detection_label
from vision_stream_client import VisionStreamPublisher


ROOT_DIR = Path(__file__).resolve().parent.parent
CONFIG_PATH = ROOT_DIR / "config" / "arm_apriltag_s0_search.yaml"
PROFILE_PATH = ROOT_DIR / "cabinet_inspection_profiles" / "fixed_scan.yaml"


def load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    return value if isinstance(value, dict) else {}


def worker_request(socket_path: Path, payload: dict[str, Any], timeout_s: float = 5.0) -> dict[str, Any]:
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout_s)
    try:
        client.connect(str(socket_path))
        client.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        data = b""
        while b"\n" not in data:
            chunk = client.recv(65536)
            if not chunk:
                break
            data += chunk
        result = json.loads(data.split(b"\n", 1)[0].decode("utf-8"))
        if not bool(result.get("ok")):
            raise RuntimeError(str(result.get("error") or "worker failed"))
        return result
    finally:
        client.close()


def draw(frame: Any, detections: list[dict[str, Any]]) -> Any:
    display = frame.copy()
    for item in detections:
        box = detection_box(item)
        if box is None:
            continue
        x1, y1, x2, y2 = [int(round(value)) for value in box]
        cv2.rectangle(display, (x1, y1), (x2, y2), (255, 180, 0), 2)
        cv2.putText(display, f"{detection_label(item)} {float(item.get('confidence') or 0.0):.2f}", (x1, max(18, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 180, 0), 1, cv2.LINE_AA)
    return display


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=int, default=120)
    args = parser.parse_args()
    config = load_yaml(CONFIG_PATH)
    profile = load_yaml(PROFILE_PATH)
    lock_path = Path(config["safety"]["lock_file"])
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    stop = {"value": False}
    signal.signal(signal.SIGINT, lambda *_args: stop.__setitem__("value", True))
    signal.signal(signal.SIGTERM, lambda *_args: stop.__setitem__("value", True))
    camera = CameraPipeline(config)
    publisher = VisionStreamPublisher(f"test-{int(time.time())}", "test", fps=8.0, jpeg_quality=75)
    with lock_path.open("w", encoding="ascii") as lock_handle:
        fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        camera.open()
        try:
            publisher.start()
            for _ in range(int(config["camera"].get("warmup_frames", 8))):
                camera.read()
            worker_socket = ROOT_DIR / str(profile["recognition"]["worker_socket"])
            models = [ROOT_DIR / str(value) for value in profile["recognition"]["models"]]
            detections: list[dict[str, Any]] = []
            detected_at = 0.0
            next_infer_at = 0.0
            deadline = time.monotonic() + max(5, min(120, int(args.timeout_s)))
            temp_path = Path(tempfile.gettempdir()) / "car_vision_stream_test.jpg"
            while not stop["value"] and time.monotonic() < deadline:
                frame = camera.read()
                now = time.monotonic()
                if now >= next_infer_at:
                    inference_started_at = now
                    cv2.imwrite(str(temp_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    current: list[dict[str, Any]] = []
                    for model in models:
                        result = worker_request(worker_socket, {"action": "infer", "image_path": str(temp_path), "model_path": str(model.resolve()), "conf": float(profile["recognition"].get("conf", 0.22)), "iou": float(profile["recognition"].get("iou", 0.45))})
                        current.extend(item for item in result.get("detections", []) if isinstance(item, dict))
                    detections = current
                    detected_at = time.monotonic()
                    next_infer_at = max(inference_started_at + 0.125, detected_at)
                age = None if detected_at <= 0 else time.monotonic() - detected_at
                display = draw(frame, detections if age is not None and age <= 0.6 else [])
                publisher.publish(
                    display,
                    detection_age_s=age,
                    stage="",
                    force=False,
                    clean_frame=frame,
                )
                time.sleep(0.02)
        finally:
            publisher.stop()
            camera.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

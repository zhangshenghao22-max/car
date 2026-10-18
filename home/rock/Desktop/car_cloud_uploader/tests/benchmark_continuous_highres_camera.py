#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import resource
import socket
import sys
import threading
import time
from pathlib import Path

import cv2
import yaml


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_tools"))

from cabinet_apriltag_inspection import ContinuousHighResolutionCamera, atomic_write_jpeg  # noqa: E402


def cpu_times() -> tuple[int, int]:
    values = Path("/proc/stat").read_text(encoding="ascii").splitlines()[0].split()[1:]
    ticks = [int(value) for value in values]
    idle = ticks[3] + (ticks[4] if len(ticks) > 4 else 0)
    return sum(ticks), idle


def maximum_temperature_c() -> float:
    values = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            value = float(path.read_text(encoding="ascii").strip())
        except (OSError, ValueError):
            continue
        values.append(value / 1000.0 if value > 1000.0 else value)
    return max(values, default=0.0)


def worker_request(socket_path: Path, image_path: Path, model_path: Path) -> dict:
    payload = {
        "action": "infer",
        "image_path": str(image_path),
        "model_path": str(model_path),
        "conf": 0.22,
        "iou": 0.45,
    }
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(2.0)
    try:
        client.connect(str(socket_path))
        client.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        chunks = []
        while True:
            chunk = client.recv(65536)
            if not chunk:
                break
            chunks.append(chunk)
            if b"\n" in chunk:
                break
        result = json.loads(b"".join(chunks).split(b"\n", 1)[0].decode("utf-8"))
        if not result.get("ok"):
            raise RuntimeError(str(result.get("error") or "worker inference failed"))
        return result
    finally:
        client.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "arm_apriltag_s0_search.yaml")
    parser.add_argument("--profile", type=Path, default=ROOT / "cabinet_inspection_profiles" / "fixed_scan.yaml")
    parser.add_argument("--worker-socket", type=Path, default=Path(""))
    parser.add_argument("--model", type=Path, default=Path(""))
    args = parser.parse_args()
    camera_config = yaml.safe_load(args.config.read_text(encoding="utf-8"))["camera"]
    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    camera = ContinuousHighResolutionCamera(camera_config, profile["capture"])
    stream = profile["stream"]
    observation = profile.get("observation", {})
    frame_wait_timeout_s = float(observation.get("frame_wait_timeout_s", 1.5))
    max_camera_age_s = float(observation.get("max_camera_age_s", 1.0))
    period_s = 1.0 / float(stream["fps"])
    total_start, idle_start = cpu_times()
    started_at = time.monotonic()
    next_encode_at = started_at
    encoded_count = 0
    encoded_bytes = 0
    maximum_temp = maximum_temperature_c()
    error = ""
    inference_stats = {"count": 0, "ages": [], "elapsed": [], "error": ""}
    inference_stop = threading.Event()
    inference_thread = None
    try:
        camera.open()
        if str(args.model) not in {"", "."}:
            worker_frame = Path("/dev/shm/car2_highres_benchmark_latest.jpg")

            def inference_loop() -> None:
                sequence = camera.current_sequence()
                while not inference_stop.is_set():
                    try:
                        frame, sequence, captured_at = camera.wait_latest(
                            sequence,
                            timeout_s=frame_wait_timeout_s,
                            copy_frame=True,
                        )
                        request_started = time.monotonic()
                        inference_frame = cv2.resize(
                            frame,
                            (
                                int(profile["recognition"].get("input_width", 1280)),
                                int(profile["recognition"].get("input_height", 960)),
                            ),
                            interpolation=cv2.INTER_AREA,
                        )
                        atomic_write_jpeg(worker_frame, inference_frame, int(profile["capture"].get("worker_jpeg_quality", 88)))
                        worker_request(args.worker_socket, worker_frame, args.model)
                        completed_at = time.monotonic()
                        inference_stats["count"] += 1
                        inference_stats["ages"].append(completed_at - captured_at)
                        inference_stats["elapsed"].append(completed_at - request_started)
                    except Exception as exc:
                        if not inference_stop.is_set():
                            inference_stats["error"] = str(exc)
                        break

            inference_thread = threading.Thread(target=inference_loop, daemon=True)
            inference_thread.start()
        while time.monotonic() - started_at < float(args.duration):
            latest = camera.latest(copy_frame=False)
            if latest is None:
                time.sleep(0.005)
                continue
            frame, _sequence, captured_at = latest
            now = time.monotonic()
            if now - captured_at > max_camera_age_s:
                raise RuntimeError(f"camera frame stale: {now - captured_at:.3f}s")
            if now >= next_encode_at:
                resized = cv2.resize(
                    frame,
                    (int(stream["width"]), int(stream["height"])),
                    interpolation=cv2.INTER_AREA,
                )
                ok, jpeg = cv2.imencode(
                    ".jpg",
                    resized,
                    [cv2.IMWRITE_JPEG_QUALITY, int(stream["jpeg_quality"])],
                )
                if not ok:
                    raise RuntimeError("JPEG encoding failed")
                encoded_count += 1
                encoded_bytes += int(jpeg.size)
                next_encode_at += period_s
                if next_encode_at <= now:
                    next_encode_at = now + period_s
            maximum_temp = max(maximum_temp, maximum_temperature_c())
            time.sleep(0.002)
    except Exception as exc:
        error = str(exc)
    finally:
        inference_stop.set()
        if inference_thread is not None:
            inference_thread.join(timeout=3.0)
        capture_fps = camera.actual_fps()
        camera.close()
        try:
            Path("/dev/shm/car2_highres_benchmark_latest.jpg").unlink()
        except OSError:
            pass
    elapsed_s = max(0.001, time.monotonic() - started_at)
    total_end, idle_end = cpu_times()
    total_delta = max(1, total_end - total_start)
    cpu_percent = 100.0 * (1.0 - (idle_end - idle_start) / total_delta)
    payload = {
        "ok": not error,
        "error": error,
        "duration_s": round(elapsed_s, 3),
        "capture_resolution": [camera.width, camera.height],
        "capture_fps": round(capture_fps, 3),
        "max_camera_age_s": round(max_camera_age_s, 3),
        "camera_diagnostics": camera.diagnostics(),
        "stream_resolution": [int(stream["width"]), int(stream["height"])],
        "encode_fps": round(encoded_count / elapsed_s, 3),
        "average_jpeg_bytes": 0 if not encoded_count else round(encoded_bytes / encoded_count),
        "system_cpu_percent": round(cpu_percent, 3),
        "process_max_rss_mb": round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0, 3),
        "maximum_temperature_c": round(maximum_temp, 3),
        "inference_count": int(inference_stats["count"]),
        "inference_fps": round(float(inference_stats["count"]) / elapsed_s, 3),
        "average_detection_age_s": round(sum(inference_stats["ages"]) / len(inference_stats["ages"]), 4)
        if inference_stats["ages"]
        else None,
        "maximum_detection_age_s": round(max(inference_stats["ages"]), 4) if inference_stats["ages"] else None,
        "average_inference_elapsed_s": round(sum(inference_stats["elapsed"]) / len(inference_stats["elapsed"]), 4)
        if inference_stats["elapsed"]
        else None,
        "inference_error": str(inference_stats["error"]),
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2), flush=True)
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

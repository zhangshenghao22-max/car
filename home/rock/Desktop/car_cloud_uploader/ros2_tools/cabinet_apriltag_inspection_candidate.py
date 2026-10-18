#!/usr/bin/env python3
"""Fixed-path cabinet inspection with optional AprilTag alignment.

The process owns the camera, arm publisher, and cabinet chassis lease from the
first arm command until the arm is back at its home pose.
"""

from __future__ import annotations

import argparse
import collections
import concurrent.futures
import contextlib
import copy
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence
from uuid import uuid4

import cv2
import numpy as np
import yaml


TOOL_DIR = Path(__file__).resolve().parent
ROOT_DIR = TOOL_DIR.parent
NAV_COMMAND_STATUS_PATH = ROOT_DIR / "runtime" / "cloud_nav_command" / "status.json"
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

from arm_apriltag_s0_search import (  # noqa: E402
    ArmAprilTagSearch,
    MANUAL_PREVIEW_CLOSE_REASONS,
    SearchFailure,
    clamp_int,
)
from arm_kinematics import ArmConfig, ArmKinematics  # noqa: E402
from cabinet_camera_overlay import (  # noqa: E402
    atomic_write_json,
    intrinsic_arrays,
    load_yaml,
    sha256_file,
)
from vision_stream_client import VisionStreamPublisher  # noqa: E402

METER_READER_DIR = ROOT_DIR / "meter_reader"
if str(METER_READER_DIR) not in sys.path:
    sys.path.insert(0, str(METER_READER_DIR))
from meter_pipeline import (  # noqa: E402
    load_meter_config,
    meter_value_from_angle,
    read_meter_from_box,
)

try:
    import requests
except Exception:  # pragma: no cover - board runtime dependency
    requests = None


SERVO_COUNT = 6
LIGHT_LABELS = {"green_off", "green_on", "red_off", "red_on"}
METER_LABELS = {"yakuang", "liukuang"}
DIGIT_LABELS = {str(value) for value in range(10)}
CAPTURE_STAGES = (
    "voltage_meter",
    "current_meter",
    "temperature_controller",
    "upper_status_lights",
    "lower_status_lights",
)
STAGE_LABELS = {
    "voltage_meter": "电压表",
    "current_meter": "电流表",
    "temperature_controller": "温控器",
    "upper_status_lights": "上排状态灯",
    "lower_status_lights": "下排状态灯",
}
UPLOAD_WORKERS = 2
WAYPOINT_SEQUENCE = (
    "top_approach",
    "voltage_meter",
    "current_meter",
    "temperature_controller",
    "middle_transit",
    "upper_status_lights",
    "lower_status_lights",
)


class ObservationDrop(SearchFailure):
    """A single visual observation may be discarded without stopping motion."""


class CameraFrameTimeout(ObservationDrop):
    pass


class InferenceFrameTimeout(ObservationDrop):
    pass


def terminate_capture_helper(process: subprocess.Popen[str], grace_s: float = 1.0) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=max(0.0, float(grace_s)))
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=max(1.0, float(grace_s)))


def wait_capture_helper(
    process: subprocess.Popen[str],
    timeout_s: float,
    safety_tick,
) -> tuple[str, str]:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    try:
        while process.poll() is None:
            safety_tick()
            if time.monotonic() >= deadline:
                terminate_capture_helper(process)
                raise SearchFailure(f"high-resolution capture timed out after {float(timeout_s):.1f}s")
            time.sleep(0.03)
        return process.communicate(timeout=1.0)
    except Exception:
        terminate_capture_helper(process)
        raise


def atomic_write_jpeg(path: Path, image: np.ndarray, quality: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.stem}.{os.getpid()}.{threading.get_ident()}.tmp.jpg")
    if not cv2.imwrite(str(temporary), image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)]):
        raise SearchFailure(f"failed to write JPEG: {temporary}")
    os.replace(temporary, path)


def adapt_center_crop_intrinsics(
    camera_matrix: np.ndarray,
    new_camera_matrix: np.ndarray,
    calibration_size: Sequence[int],
    output_size: Sequence[int],
) -> tuple[np.ndarray, np.ndarray]:
    calibration_width, calibration_height = (float(value) for value in calibration_size)
    output_width, output_height = (float(value) for value in output_size)
    if min(calibration_width, calibration_height, output_width, output_height) <= 0.0:
        raise ValueError("camera calibration and stream dimensions must be positive")

    if output_width / output_height <= calibration_width / calibration_height:
        scale = output_width / calibration_width
        offset_x = 0.0
        offset_y = (output_height - calibration_height * scale) / 2.0
    else:
        scale = output_height / calibration_height
        offset_x = (output_width - calibration_width * scale) / 2.0
        offset_y = 0.0
    transform = np.asarray(
        [[scale, 0.0, offset_x], [0.0, scale, offset_y], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return transform @ camera_matrix, transform @ new_camera_matrix


class StreamRectifier:
    """Rectify stream frames and overlay points in the same coordinate system."""

    def __init__(self, intrinsics: dict[str, Any], output_size: Sequence[int]):
        camera_matrix, dist_coeffs, new_camera_matrix, calibration_size = intrinsic_arrays(intrinsics)
        self.model = str(intrinsics.get("model") or "pinhole").lower()
        self.calibration_size = tuple(int(value) for value in calibration_size)
        self.output_size = tuple(int(value) for value in output_size)
        self.camera_matrix, self.new_camera_matrix = adapt_center_crop_intrinsics(
            np.asarray(camera_matrix, dtype=np.float64),
            np.asarray(new_camera_matrix, dtype=np.float64),
            self.calibration_size,
            self.output_size,
        )
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64)
        identity = np.eye(3, dtype=np.float64)
        if self.model == "fisheye":
            self.maps = cv2.fisheye.initUndistortRectifyMap(
                self.camera_matrix,
                self.dist_coeffs,
                identity,
                self.new_camera_matrix,
                self.output_size,
                cv2.CV_16SC2,
            )
        else:
            self.maps = cv2.initUndistortRectifyMap(
                self.camera_matrix,
                self.dist_coeffs,
                None,
                self.new_camera_matrix,
                self.output_size,
                cv2.CV_16SC2,
            )

    def rectify(self, frame: np.ndarray) -> np.ndarray:
        actual_size = (int(frame.shape[1]), int(frame.shape[0]))
        if actual_size != self.output_size:
            raise ValueError(f"stream frame size mismatch: {actual_size} != {self.output_size}")
        return cv2.remap(
            frame,
            self.maps[0],
            self.maps[1],
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

    def rectify_points(self, points: np.ndarray) -> np.ndarray:
        values = np.asarray(points, dtype=np.float64).reshape((-1, 1, 2))
        if self.model == "fisheye":
            rectified = cv2.fisheye.undistortPoints(
                values,
                self.camera_matrix,
                self.dist_coeffs,
                R=np.eye(3, dtype=np.float64),
                P=self.new_camera_matrix,
            )
        else:
            rectified = cv2.undistortPoints(
                values,
                self.camera_matrix,
                self.dist_coeffs,
                R=None,
                P=self.new_camera_matrix,
            )
        return rectified.reshape((-1, 2))


class ContinuousHighResolutionCamera:
    """Continuously capture only the newest raw high-resolution camera frame."""

    def __init__(self, camera_config: dict[str, Any], capture_config: dict[str, Any]):
        self.device = str(camera_config["device"])
        self.backend = str(capture_config.get("backend", camera_config.get("backend", "gstreamer"))).lower()
        self.rotate_180 = bool(camera_config.get("rotate_180", True))
        self.width = int(capture_config["width"])
        self.height = int(capture_config["height"])
        self.fps_target = int(capture_config.get("fps", 15))
        self.capture: cv2.VideoCapture | None = None
        self._condition = threading.Condition()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._frame: np.ndarray | None = None
        self._sequence = 0
        self._read_sequence = 0
        self._frame_at = 0.0
        self._error = ""
        self._timestamps: collections.deque[float] = collections.deque(maxlen=max(30, self.fps_target * 4))
        self._low_fps_since = 0.0

    def _open_capture(self) -> cv2.VideoCapture:
        capture = None
        if self.backend == "gstreamer":
            pipeline = (
                f"v4l2src device={self.device} io-mode=2 ! "
                f"image/jpeg,width={self.width},height={self.height},framerate={self.fps_target}/1 ! "
                "jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
                "appsink drop=true max-buffers=1 sync=false"
            )
            candidate = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if candidate.isOpened():
                capture = candidate
            else:
                candidate.release()
        if capture is None:
            capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            capture.set(cv2.CAP_PROP_FPS, self.fps_target)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"cannot open high-resolution camera: {self.device}")
        return capture

    def open(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self.capture = self._open_capture()
        self._stop_event.clear()
        with self._condition:
            self._frame = None
            self._sequence = 0
            self._read_sequence = 0
            self._frame_at = 0.0
            self._error = ""
            self._timestamps.clear()
            self._low_fps_since = 0.0
        self._thread = threading.Thread(target=self._reader_loop, name="cabinet-highres-camera", daemon=True)
        self._thread.start()
        _frame, _sequence, _captured_at = self.wait_latest(0, timeout_s=3.0, copy_frame=False)

    def _reader_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                capture = self.capture
                if capture is None:
                    break
                ok, frame = capture.read()
                if not ok or frame is None:
                    raise RuntimeError("high-resolution camera read failed")
                if self.rotate_180:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                actual_size = (int(frame.shape[1]), int(frame.shape[0]))
                expected_size = (self.width, self.height)
                if actual_size != expected_size:
                    raise RuntimeError(f"high-resolution camera size mismatch: {actual_size} != {expected_size}")
                captured_at = time.monotonic()
                with self._condition:
                    self._frame = frame
                    self._sequence += 1
                    self._frame_at = captured_at
                    self._timestamps.append(captured_at)
                    self._condition.notify_all()
        except Exception as exc:
            with self._condition:
                self._error = str(exc)
                self._condition.notify_all()

    def wait_latest(
        self,
        after_sequence: int,
        *,
        timeout_s: float = 0.5,
        copy_frame: bool = True,
    ) -> tuple[np.ndarray, int, float]:
        deadline = time.monotonic() + max(0.01, float(timeout_s))
        with self._condition:
            while self._sequence <= int(after_sequence) and not self._error:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(remaining)
            if self._error:
                raise CameraFrameTimeout(self._error)
            if self._frame is None or self._sequence <= int(after_sequence):
                raise CameraFrameTimeout(
                    f"high-resolution camera produced no fresh frame for {float(timeout_s):.2f}s"
                )
            frame = self._frame.copy() if copy_frame else self._frame
            return frame, int(self._sequence), float(self._frame_at)

    def latest(self, *, copy_frame: bool = False) -> tuple[np.ndarray, int, float] | None:
        with self._condition:
            if self._error:
                raise CameraFrameTimeout(self._error)
            if self._frame is None:
                return None
            frame = self._frame.copy() if copy_frame else self._frame
            return frame, int(self._sequence), float(self._frame_at)

    def read(self) -> np.ndarray:
        frame, sequence, _captured_at = self.wait_latest(self._read_sequence, timeout_s=0.6, copy_frame=True)
        self._read_sequence = sequence
        return frame

    def detect(self, _frame: np.ndarray) -> dict[str, Any]:
        return {"tags": [], "timestamp": time.time()}

    def frame_age_s(self) -> float | None:
        with self._condition:
            return None if self._frame_at <= 0.0 else max(0.0, time.monotonic() - self._frame_at)

    def actual_fps(self) -> float:
        with self._condition:
            if len(self._timestamps) < 2:
                return 0.0
            elapsed = self._timestamps[-1] - self._timestamps[0]
            return 0.0 if elapsed <= 0.0 else (len(self._timestamps) - 1) / elapsed

    def health_snapshot(self, minimum_fps: float) -> dict[str, Any]:
        now = time.monotonic()
        fps = self.actual_fps()
        with self._condition:
            sample_count = len(self._timestamps)
            frame_age = None if self._frame_at <= 0.0 else max(0.0, now - self._frame_at)
            if sample_count >= 4 and 0.0 < fps < float(minimum_fps):
                if self._low_fps_since <= 0.0:
                    self._low_fps_since = now
            else:
                self._low_fps_since = 0.0
            low_fps_duration = 0.0 if self._low_fps_since <= 0.0 else now - self._low_fps_since
        return {
            "fps": round(fps, 3),
            "sample_count": sample_count,
            "frame_age_s": None if frame_age is None else round(frame_age, 3),
            "low_fps_duration_s": round(low_fps_duration, 3),
        }

    def reopen(self) -> None:
        self.close()
        time.sleep(0.15)
        self.open()

    def current_sequence(self) -> int:
        with self._condition:
            return int(self._sequence)

    def close(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.5)
        capture = self.capture
        self.capture = None
        if capture is not None:
            capture.release()
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._thread = None


def navigation_scan_context_matches(
    status: Any,
    *,
    session_id: str,
    inspection_id: str,
    waypoint_index: int,
    expected_tag_id: int | None,
) -> bool:
    if not isinstance(status, dict) or not session_id or not inspection_id:
        return False
    if not bool(status.get("task_running")) or bool(status.get("goal_active")):
        return False
    if str(status.get("session_id") or "") != str(session_id):
        return False
    if str(status.get("task_phase") or "") != "scanning":
        return False
    if str(status.get("state") or "") != "scanning":
        return False
    if str(status.get("active_inspection_id") or "") != str(inspection_id):
        return False
    try:
        if int(status.get("current_index")) != int(waypoint_index):
            return False
        if expected_tag_id is not None and int(status.get("current_apriltag_id")) != int(expected_tag_id):
            return False
    except (TypeError, ValueError):
        return False
    return True
METER_STAGES = {"voltage_meter", "current_meter"}
LIGHT_STAGES = {"upper_status_lights", "lower_status_lights"}
POSE_ALIASES = {
    "top": "top_approach",
    "middle": "upper_status_lights",
    "lower": "lower_status_lights",
    "top_overview": "top_approach",
    "middle_controls": "upper_status_lights",
    "lower_controls": "lower_status_lights",
    **{name: name for name in WAYPOINT_SEQUENCE},
}


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def load_profile(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"inspection profile must be a YAML object: {path}")
    return payload


def resolve_path(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else ROOT_DIR / path


def merge_cabinet_update(target: dict[str, Any], update: dict[str, Any]) -> None:
    for key, value in update.items():
        if key == "updated_at":
            continue
        if key == "status_lights" and isinstance(value, dict):
            existing = target.get(key, {}) if isinstance(target.get(key), dict) else {}
            merged = copy.deepcopy(existing)
            existing_lights = existing.get("lights", []) if isinstance(existing.get("lights"), list) else []
            incoming_lights = value.get("lights", []) if isinstance(value.get("lights"), list) else []
            by_position = {
                str(item.get("position") or f"{item.get('row', '')}:{item.get('color', '')}"): copy.deepcopy(item)
                for item in existing_lights
                if isinstance(item, dict)
            }
            for item in incoming_lights:
                if not isinstance(item, dict):
                    continue
                position = str(item.get("position") or f"{item.get('row', '')}:{item.get('color', '')}")
                by_position[position] = copy.deepcopy(item)
            for nested_key, nested_value in value.items():
                if nested_key != "lights":
                    merged[nested_key] = copy.deepcopy(nested_value)
            merged["lights"] = list(by_position.values())
            target[key] = merged
            continue
        target[key] = copy.deepcopy(value)


def stage_cabinet_update(
    stage: str,
    result: dict[str, Any],
    coverage: dict[str, Any],
) -> dict[str, Any]:
    update: dict[str, Any] = {}
    if stage in METER_STAGES:
        expected = "voltage" if stage == "voltage_meter" else "current"
        for reading in result.get("meter_readings", []):
            if not isinstance(reading, dict) or str(reading.get("meter_type") or "") != expected:
                continue
            metric = reading.get("cabinet_metric")
            metric = copy.deepcopy(metric) if isinstance(metric, dict) else {
                "value": reading.get("value"),
                "unit": reading.get("unit") or ("V" if expected == "voltage" else "A"),
                "status": str(reading.get("status") or "ok"),
            }
            if expected == "voltage" and metric.get("raw_value") is None:
                metric["raw_value"] = reading.get("raw_value", reading.get("value"))
            update[expected] = metric
            break
    elif stage == "temperature_controller":
        digits = coverage.get("digits", {}) if isinstance(coverage, dict) else {}
        rows = digits.get("rows", []) if isinstance(digits, dict) else []
        temperature_reading = result.get("temperature_controller", {}) if isinstance(result, dict) else {}
        upper_text = (
            str(temperature_reading.get("upper_text") or "").strip()
            if isinstance(temperature_reading, dict)
            else ""
        )
        if not upper_text and isinstance(rows, list) and rows:
            upper_text = str(rows[0]).strip()
        if isinstance(rows, list) and rows:
            display_rows = [upper_text, *[str(value) for value in rows[1:]]] if upper_text else [str(value) for value in rows]
            metric: dict[str, Any] = {
                "display_text": " / ".join(display_rows),
                "upper_text": upper_text,
                "status": "ok" if bool(digits.get("reliable")) else "estimated",
                "source": str(digits.get("source") or "continuous_stream"),
                "reliable": bool(digits.get("reliable")),
            }
            if re.fullmatch(r"[-+]?\d+(?:\.\d+)?", upper_text):
                metric["pv"] = float(upper_text)
                metric["unit"] = "℃"
            if isinstance(temperature_reading, dict):
                for key in ("method", "valid_frame_count", "consensus_count", "match_distance"):
                    if key in temperature_reading:
                        metric[key] = copy.deepcopy(temperature_reading[key])
            update["temperature_controller"] = metric
    elif stage in LIGHT_STAGES:
        lights = coverage.get("status_lights", {}) if isinstance(coverage, dict) else {}
        positions = lights.get("positions", {}) if isinstance(lights, dict) else {}
        stage_lights: list[dict[str, Any]] = []
        if isinstance(positions, dict):
            for color in ("green", "red"):
                key = f"{stage}:{color}"
                if key in positions:
                    stage_lights.append(
                        {
                            "position": key,
                            "row": "upper" if stage.startswith("upper") else "lower",
                            "color": color,
                            "state": str(positions[key]),
                        }
                    )
        if stage_lights:
            update["status_lights"] = {
                "lights": stage_lights,
                "status": "ok" if len(stage_lights) == 2 and bool(lights.get("reliable")) else "estimated",
                "source": "multi_frame_consensus" if bool(lights.get("reliable")) else "continuous_best_effort",
                "reliable": bool(lights.get("reliable")),
            }
    if update:
        update["updated_at"] = utc_now_iso()
    return update


def build_stage_upload_payload(
    source: dict[str, Any],
    *,
    inspection_id: str,
    stage: str,
    stage_index: int,
    profile: str,
    started_at: str,
    board_id: str,
    board_label: str,
    coverage: dict[str, Any],
) -> dict[str, Any]:
    payload = copy.deepcopy(source)
    if stage in METER_STAGES:
        expected = "voltage" if stage == "voltage_meter" else "current"
        payload["meter_readings"] = [
            copy.deepcopy(reading)
            for reading in payload.get("meter_readings", [])
            if isinstance(reading, dict) and str(reading.get("meter_type") or "") == expected
        ]
        payload["digit_readings"] = {}
        payload["digit_detections"] = []
    elif stage == "temperature_controller":
        payload["meter_readings"] = []
    elif stage in LIGHT_STAGES:
        payload["meter_readings"] = []
        payload["digit_readings"] = {}
        payload["digit_detections"] = []
    payload.update(
        {
            "inspection_id": inspection_id,
            "inspection_stage": stage,
            "inspection_stage_index": int(stage_index),
            "inspection_stage_count": len(CAPTURE_STAGES),
            "inspection_stage_label": STAGE_LABELS.get(stage, stage),
            "inspection_status": str(source.get("stage_status") or "running"),
            "inspection_profile": profile,
            "inspection_started_at": started_at,
            "capture_key": f"{inspection_id}:{stage}",
            "coverage": copy.deepcopy(coverage),
            "cabinet_data": stage_cabinet_update(stage, source, coverage),
            "board_id": board_id,
            "board_label": board_label,
        }
    )
    return payload


def compact_recognition_reference(value: dict[str, Any]) -> dict[str, Any]:
    return {
        "id": str(value.get("id") or ""),
        "uploaded_at": str(value.get("uploaded_at") or ""),
        "raw_image": copy.deepcopy(value.get("raw_image", {})),
        "annotated_image": copy.deepcopy(value.get("annotated_image", {})),
        "backend": str(value.get("backend") or ""),
        "inference_ms": value.get("inference_ms", ""),
        "cabinet_update": copy.deepcopy(value.get("cabinet_update", {})),
    }


def prioritize_structured_inspection_payload(summary: dict[str, Any]) -> dict[str, Any]:
    priority = (
        "inspection_id",
        "status",
        "profile",
        "board_id",
        "board_label",
        "started_at",
        "completed_at",
        "stages",
        "cabinet_update",
        "expected_stage_count",
        "coverage",
        "result_source",
        "structured_only",
        "photo_count",
        "result_count",
        "quality_warnings",
        "navigation_session_id",
        "waypoint_index",
        "waypoint_label",
        "expected_apriltag_id",
    )
    payload = {key: copy.deepcopy(summary[key]) for key in priority if key in summary}
    for key, value in summary.items():
        if key not in payload:
            payload[key] = copy.deepcopy(value)
    return payload


def detection_label(item: dict[str, Any]) -> str:
    return str(item.get("class_name") or item.get("label") or "").strip().lower()


def detection_box(item: dict[str, Any]) -> list[float] | None:
    raw = item.get("box_xyxy") or item.get("bbox") or item.get("box")
    if not isinstance(raw, (list, tuple)) or len(raw) != 4:
        return None
    try:
        x1, y1, x2, y2 = [float(value) for value in raw]
    except (TypeError, ValueError):
        return None
    return [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]


def scale_box(
    box: Sequence[float],
    source_size: tuple[int, int],
    target_size: tuple[int, int],
) -> list[int]:
    source_width, source_height = source_size
    target_width, target_height = target_size
    scale_x = float(target_width) / max(1.0, float(source_width))
    scale_y = float(target_height) / max(1.0, float(source_height))
    return [
        int(round(float(box[0]) * scale_x)),
        int(round(float(box[1]) * scale_y)),
        int(round(float(box[2]) * scale_x)),
        int(round(float(box[3]) * scale_y)),
    ]


def scale_detections(
    detections: Sequence[dict[str, Any]],
    source_size: tuple[int, int],
    target_size: tuple[int, int],
) -> list[dict[str, Any]]:
    scaled = []
    for item in detections:
        value = copy.deepcopy(item)
        box = detection_box(value)
        if box is not None:
            value["box_xyxy"] = scale_box(box, source_size, target_size)
        scaled.append(value)
    return scaled


def observation_action(
    elapsed_s: float,
    stable: bool,
    extended: bool,
    *,
    minimum_s: float,
    primary_s: float,
    extension_s: float,
) -> str:
    if stable and float(elapsed_s) >= float(minimum_s):
        return "complete"
    if float(elapsed_s) >= float(primary_s) + float(extension_s):
        return "timeout"
    if float(elapsed_s) >= float(primary_s) and not extended:
        return "extend"
    return "continue"


def scan_status_for_completed_stages(
    completed_stages: Iterable[str],
    *,
    coverage_complete: bool,
) -> str:
    completed = {str(stage) for stage in completed_stages}
    if not completed:
        return "failed"
    if completed == set(CAPTURE_STAGES) and bool(coverage_complete):
        return "completed"
    return "partial"


def union_box(detections: Iterable[dict[str, Any]]) -> list[float] | None:
    boxes = [box for box in (detection_box(item) for item in detections) if box is not None]
    if not boxes:
        return None
    return [
        min(box[0] for box in boxes),
        min(box[1] for box in boxes),
        max(box[2] for box in boxes),
        max(box[3] for box in boxes),
    ]


def select_stage_detections(stage: str, detections: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    if stage == "voltage_meter":
        return [item for item in detections if detection_label(item) == "yakuang"]
    if stage == "current_meter":
        return [item for item in detections if detection_label(item) == "liukuang"]
    if stage == "temperature_controller":
        return [item for item in detections if detection_label(item) in DIGIT_LABELS]
    if stage in LIGHT_STAGES:
        lights = [item for item in detections if detection_label(item) in LIGHT_LABELS]
        rows = _group_rows(lights)
        if len(rows) <= 1:
            return lights
        return rows[0] if stage == "upper_status_lights" else rows[-1]
    return []


def extension_scale_for_distance(
    distance_mm: float | None,
    spread_mm: float | None,
    config: dict[str, Any],
) -> tuple[float, str]:
    maximum_spread = float(config.get("max_distance_spread_mm", 20.0))
    if distance_mm is None or not math.isfinite(float(distance_mm)):
        return 0.0, "tag_distance_unavailable"
    if spread_mm is not None and float(spread_mm) > maximum_spread:
        return 0.0, "tag_distance_unstable"
    disabled_below = float(config.get("disable_below_mm", 200.0))
    full_at = float(config.get("full_at_mm", 220.0))
    if float(distance_mm) <= disabled_below:
        return 0.0, "cabinet_too_close"
    if float(distance_mm) >= full_at:
        return 1.0, "full_extension"
    span = max(1.0, full_at - disabled_below)
    return (float(distance_mm) - disabled_below) / span, "scaled_extension"


def inspection_extension_scale(
    distance_mm: float | None,
    spread_mm: float | None,
    config: dict[str, Any],
    *,
    tag_search_skipped: bool,
) -> tuple[float, str]:
    if not tag_search_skipped:
        return extension_scale_for_distance(distance_mm, spread_mm, config)
    scale = float(config.get("direct_scan_scale", 1.0))
    if not 0.0 <= scale <= 1.0:
        raise ValueError("direct_scan_scale must be between 0 and 1")
    return scale, "tag_search_skipped_configured_extension"


def validate_inference_elapsed(elapsed_s: float, config: dict[str, Any]) -> float:
    maximum = float(config.get("max_inference_elapsed_s", config.get("max_detection_age_s", 1.5)))
    if elapsed_s > maximum:
        raise InferenceFrameTimeout(
            f"RKNN inference elapsed {elapsed_s:.3f}s exceeds {maximum:.3f}s"
        )
    return maximum


def resolve_waypoint_pose(
    waypoint: dict[str, Any],
    aligned_s0: int,
    extension_scale: float,
) -> list[int]:
    retracted = [int(value) for value in waypoint["retracted_pose"]]
    extended = [int(value) for value in waypoint["extended_pose"]]
    if len(retracted) != SERVO_COUNT or len(extended) != SERVO_COUNT:
        raise ValueError("waypoint poses must contain six PWM values")
    scale = max(0.0, min(1.0, float(extension_scale)))
    pose = [
        int(round((1.0 - scale) * retracted[index] + scale * extended[index]))
        for index in range(SERVO_COUNT)
    ]
    pose[0] = int(aligned_s0) + int(waypoint.get("s0_offset_pwm", 0))
    pose[4] = 1500
    pose[5] = 1500
    return pose


def resolve_waypoint_motion(
    waypoint: dict[str, Any],
    aligned_s0: int,
    extension_scale: float,
    default_duration_ms: int,
) -> list[tuple[list[int], int]]:
    motion: list[tuple[list[int], int]] = []
    for item in waypoint.get("trajectory", []):
        segment = dict(item)
        segment.setdefault("s0_offset_pwm", waypoint.get("s0_offset_pwm", 0))
        motion.append(
            (
                resolve_waypoint_pose(segment, aligned_s0, extension_scale),
                int(segment.get("duration_ms", default_duration_ms)),
            )
        )
    final_pose = resolve_waypoint_pose(waypoint, aligned_s0, extension_scale)
    if not motion or motion[-1][0] != final_pose:
        motion.append((final_pose, int(waypoint.get("duration_ms", default_duration_ms))))
    return motion


def s0_motion_duration_ms(start_pwm: int, target_pwm: int, config: dict[str, Any]) -> int:
    delta = abs(int(target_pwm) - int(start_pwm))
    per_pwm = float(config.get("s0_duration_per_pwm_ms", 4.0))
    minimum = int(config.get("s0_min_duration_ms", 500))
    maximum = int(config.get("s0_max_duration_ms", 1000))
    return clamp_int(int(round(delta * per_pwm)), minimum, maximum)


def interpolate_pose(points: Sequence[Sequence[int]], progress: float, aligned_s0: int) -> list[int]:
    if len(points) < 2:
        raise ValueError("at least two scan path points are required")
    value = max(0.0, min(float(len(points) - 1), float(progress)))
    low = min(len(points) - 2, int(math.floor(value)))
    fraction = value - low
    result = [
        int(round((1.0 - fraction) * int(points[low][index]) + fraction * int(points[low + 1][index])))
        for index in range(SERVO_COUNT)
    ]
    result[0] = int(aligned_s0)
    result[4] = 1500
    result[5] = 1500
    return result


def validate_pose(pose: Sequence[int], limits: Sequence[Sequence[int]], aligned_s0: int, s0_delta: int) -> None:
    if len(pose) != SERVO_COUNT or len(limits) != SERVO_COUNT:
        raise ValueError("pose and limits must contain six entries")
    for index, value in enumerate(pose):
        low, high = [int(part) for part in limits[index]]
        if not low <= int(value) <= high:
            raise ValueError(f"S{index}={value} is outside [{low},{high}]")
    if abs(int(pose[0]) - int(aligned_s0)) > abs(int(s0_delta)):
        raise ValueError("S0 pose exceeds the aligned-position bound")
    if int(pose[4]) != 1500 or int(pose[5]) != 1500:
        raise ValueError("S4 and S5 must remain at 1500")


def validate_trajectory(
    start: Sequence[int],
    target: Sequence[int],
    limits: Sequence[Sequence[int]],
    aligned_s0: int,
    s0_delta: int,
    kinematics: ArmKinematics,
) -> None:
    for step in range(21):
        ratio = step / 20.0
        pose = [int(round((1.0 - ratio) * int(start[i]) + ratio * int(target[i]))) for i in range(SERVO_COUNT)]
        validate_pose(pose, limits, aligned_s0, s0_delta)
        arm_pose, joints, transform = kinematics.pose_from_pwm(pose)
        values = [*joints, arm_pose.x_mm, arm_pose.y_mm, arm_pose.z_mm, *transform.reshape(-1)]
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("non-finite arm kinematics on scan trajectory")
        for index, joint in enumerate(joints):
            cfg = kinematics.config
            if not cfg.min_deg[index] <= joint <= cfg.max_deg[index]:
                raise ValueError(f"S{index} trajectory exceeds joint limits")


def changed_servo_targets(start: Sequence[int], target: Sequence[int]) -> list[tuple[int, int]]:
    if len(start) != SERVO_COUNT or len(target) != SERVO_COUNT:
        raise ValueError("start and target poses must contain six values")
    return [
        (index, int(target[index]))
        for index in range(4)
        if int(start[index]) != int(target[index])
    ]


def frame_quality(image: np.ndarray, detections: Sequence[dict[str, Any]]) -> dict[str, float]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    overexposed = float(np.mean(gray >= 250))
    confidences = [float(item.get("confidence") or 0.0) for item in detections]
    confidence = float(sum(confidences) / len(confidences)) if confidences else 0.0
    selected_box = union_box(detections)
    if selected_box is None:
        center_error = 1.0
        coverage_ratio = 0.0
    else:
        x1, y1, x2, y2 = selected_box
        coverage_ratio = max(0.0, x2 - x1) * max(0.0, y2 - y1) / max(1.0, image.shape[0] * image.shape[1])
        center_error = math.hypot(
            ((x1 + x2) * 0.5 - image.shape[1] * 0.5) / max(1.0, image.shape[1]),
            ((y1 + y2) * 0.5 - image.shape[0] * 0.5) / max(1.0, image.shape[0]),
        )
    score = min(1.0, sharpness / 500.0) * 0.32 + confidence * 0.36
    score += max(0.0, 1.0 - center_error * 2.0) * 0.17
    score += min(1.0, coverage_ratio / 0.12) * 0.15
    score -= min(0.40, overexposed * 1.6)
    return {
        "score": round(score, 6),
        "sharpness": round(sharpness, 3),
        "overexposed_ratio": round(overexposed, 6),
        "mean_confidence": round(confidence, 6),
        "center_error": round(center_error, 6),
        "target_coverage_ratio": round(coverage_ratio, 6),
    }


def _group_rows(detections: Sequence[dict[str, Any]]) -> list[list[dict[str, Any]]]:
    items = [item for item in detections if detection_box(item) is not None]
    items.sort(key=lambda item: (sum(detection_box(item)[1::2]) * 0.5, detection_box(item)[0]))
    rows: list[list[dict[str, Any]]] = []
    for item in items:
        box = detection_box(item)
        cy = (box[1] + box[3]) * 0.5
        height = max(1.0, box[3] - box[1])
        if not rows:
            rows.append([item])
            continue
        centers = [
            (detection_box(value)[1] + detection_box(value)[3]) * 0.5
            for value in rows[-1]
        ]
        row_height = float(np.median([detection_box(value)[3] - detection_box(value)[1] for value in rows[-1]]))
        if abs(cy - float(np.mean(centers))) <= max(height, row_height) * 0.65:
            rows[-1].append(item)
        else:
            rows.append([item])
    return rows


def digit_text_rows(detections: Sequence[dict[str, Any]]) -> list[str]:
    digits = [item for item in detections if detection_label(item) in DIGIT_LABELS]
    texts: list[str] = []
    for row in _group_rows(digits):
        ordered = sorted(row, key=lambda item: detection_box(item)[0])
        text = "".join(detection_label(item) for item in ordered)
        if len(text) >= 2:
            texts.append(text)
    return texts


def digit_consensus(
    observations: Sequence[Sequence[str]],
    required_votes: int = 2,
    minimum_observations: int = 1,
    minimum_ratio: float = 0.0,
) -> dict[str, Any]:
    row_votes: dict[int, collections.Counter[str]] = collections.defaultdict(collections.Counter)
    for rows in observations:
        for index, text in enumerate(rows):
            clean = str(text).strip()
            if clean:
                row_votes[index][clean] += 1
    accepted: list[str] = []
    winners: list[str] = []
    votes: list[int] = []
    ratios: list[float] = []
    valid_counts: list[int] = []
    reliable_rows: list[bool] = []
    for index in sorted(row_votes):
        text, count = row_votes[index].most_common(1)[0]
        valid_count = sum(row_votes[index].values())
        ratio = count / max(1, valid_count)
        reliable = (
            valid_count >= int(minimum_observations)
            and count >= int(required_votes)
            and ratio >= float(minimum_ratio)
        )
        winners.append(text)
        accepted.append(text if reliable else "")
        votes.append(count)
        ratios.append(round(ratio, 6))
        valid_counts.append(valid_count)
        reliable_rows.append(reliable)
    reliable = bool(accepted and accepted[0] and reliable_rows[0])
    return {
        "rows": [value for value in accepted if value],
        "winner_rows": winners,
        "votes": votes,
        "vote_ratios": ratios,
        "valid_observation_counts": valid_counts,
        "reliable_rows": reliable_rows,
        "complete": reliable,
        "reliable": reliable,
        "source": "multi_frame_consensus" if reliable else "unstable_consensus",
    }


def light_observation(stage: str, detections: Sequence[dict[str, Any]]) -> dict[str, str]:
    result: dict[str, str] = {}
    lights = [item for item in detections if detection_label(item) in LIGHT_LABELS]
    rows = _group_rows(lights)
    if len(rows) > 1:
        lights = rows[0] if stage == "upper_status_lights" else rows[-1]
    by_color: dict[str, list[dict[str, Any]]] = {"green": [], "red": []}
    for item in lights:
        by_color[detection_label(item).split("_", 1)[0]].append(item)
    for color, items in by_color.items():
        if not items:
            continue
        best = max(items, key=lambda item: float(item.get("confidence") or 0.0))
        result[f"{stage}:{color}"] = detection_label(best).split("_", 1)[1]
    return result


def light_consensus(
    observations: Sequence[dict[str, str]],
    required_votes: int = 2,
    minimum_observations: int = 1,
    minimum_ratio: float = 0.0,
    expected_positions: set[str] | None = None,
) -> dict[str, Any]:
    votes: dict[str, collections.Counter[str]] = collections.defaultdict(collections.Counter)
    for observation in observations:
        for key, state in observation.items():
            votes[str(key)][str(state)] += 1
    accepted: dict[str, str] = {}
    counts: dict[str, int] = {}
    ratios: dict[str, float] = {}
    observation_counts: dict[str, int] = {}
    winners: dict[str, str] = {}
    for key, counter in votes.items():
        state, count = counter.most_common(1)[0]
        valid_count = sum(counter.values())
        ratio = count / max(1, valid_count)
        winners[key] = state
        observation_counts[key] = valid_count
        ratios[key] = round(ratio, 6)
        if (
            valid_count >= int(minimum_observations)
            and count >= int(required_votes)
            and ratio >= float(minimum_ratio)
        ):
            accepted[key] = state
            counts[key] = count
    expected = expected_positions or {
        "upper_status_lights:green",
        "upper_status_lights:red",
        "lower_status_lights:green",
        "lower_status_lights:red",
    }
    return {
        "positions": accepted,
        "winner_positions": winners,
        "votes": counts,
        "winner_votes": {key: int(counter.most_common(1)[0][1]) for key, counter in votes.items()},
        "vote_ratios": ratios,
        "valid_observation_counts": observation_counts,
        "complete": expected.issubset(accepted),
        "missing": sorted(expected - set(accepted)),
    }


GEOMETRIC_RECTIFY_PREFIXES = ("auto-quad", "closed-edge-quad", "strict-orb")


def template_score(reading: dict[str, Any]) -> float:
    method = str(reading.get("rectify_method") or "")
    if method.startswith(GEOMETRIC_RECTIFY_PREFIXES):
        quality = reading.get("rectify_quality", {})
        if isinstance(quality, dict) and quality.get("area_ratio") is not None:
            return min(1.0, float(quality["area_ratio"]) * 1.5)
        return min(1.0, float(reading.get("quality_score", 0.0) or 0.0) / 4.0)
    match = re.search(r"template-ecc:([0-9.]+)", method)
    if match:
        return float(match.group(1))
    if method.startswith("template-orb:"):
        try:
            return min(1.0, float(method.split(":", 1)[1]) / 20.0)
        except ValueError:
            return 0.0
    return 0.0


def meter_snapshot(results: Sequence[dict[str, Any]]) -> dict[str, Any]:
    values: dict[str, list[dict[str, Any]]] = {"voltage": [], "current": []}
    for result in results:
        for reading in result.get("meter_readings", []):
            meter_type = str(reading.get("meter_type") or "")
            if meter_type not in values or reading.get("value") is None:
                continue
            values[meter_type].append(
                {
                    "value": float(reading["value"]),
                    "status": str(reading.get("status") or ""),
                    "template_score": template_score(reading),
                }
            )
    snapshot: dict[str, Any] = {}
    thresholds = {"voltage": 10.0, "current": 0.05}
    for meter_type, readings in values.items():
        if not readings:
            snapshot[meter_type] = {"status": "missing", "readings": []}
            continue
        numeric = [item["value"] for item in readings]
        unstable = max(numeric) - min(numeric) > thresholds[meter_type]
        best = max(readings, key=lambda item: item["template_score"])
        snapshot[meter_type] = {
            "status": "unstable" if unstable else "ok",
            "value": best["value"],
            "template_score": best["template_score"],
            "readings": readings,
        }
    return snapshot


def meter_result_payload(
    result: Any,
    confidence: float,
    box: Sequence[float],
    meter_type: str = "voltage",
) -> dict[str, Any]:
    value = getattr(result, "value", None)
    is_voltage = str(meter_type) == "voltage"
    unit = "V" if is_voltage else "A"
    maximum = 450 if is_voltage else 1.0
    payload = {
        "meter_type": str(meter_type),
        "display_name": str(meter_type),
        "unit": unit,
        "value": None if value is None else round(float(value), 6),
        "raw_value": None if value is None else round(float(value), 6),
        "status": str(getattr(result, "status", "")),
        "confidence": round(float(confidence), 6),
        "box_xyxy": [round(float(value), 3) for value in box],
        "pointer_angle": None
        if getattr(result, "pointer_angle", None) is None
        else round(float(result.pointer_angle), 6),
        "pointer_method": str(getattr(result, "pointer_method", "") or ""),
        "pointer_candidates": list(getattr(result, "pointer_candidates", None) or []),
        "pointer_consensus_count": int(getattr(result, "pointer_consensus_count", 0) or 0),
        "rectify_method": str(getattr(result, "rectify_method", "") or ""),
        "rectify_quality": copy.deepcopy(getattr(result, "rectify_quality", None) or {}),
        "quality_score": round(float(getattr(result, "quality_score", 0.0) or 0.0), 6),
    }
    if value is not None:
        payload["cabinet_metric"] = {
            "value": round(float(value), 6),
            "raw_value": round(float(value), 6),
            "unit": unit,
            "status": "ok",
            "min": 0,
            "max": maximum,
        }
    return payload


def fuse_meter_readings(
    readings: Sequence[dict[str, Any]],
    *,
    meter_type: str,
    minimum_valid: int = 5,
    required_votes: int = 3,
    max_spread: float = 6.0,
) -> dict[str, Any]:
    valid = [
        copy.deepcopy(reading)
        for reading in readings
        if reading.get("value") is not None and math.isfinite(float(reading["value"]))
    ]
    if not valid:
        return {"status": "missing", "reading": None, "valid_frame_count": 0, "consensus_count": 0}

    ordered = sorted(valid, key=lambda reading: float(reading["value"]))
    inliers: list[dict[str, Any]] = []
    left = 0
    for right, reading in enumerate(ordered):
        while float(reading["value"]) - float(ordered[left]["value"]) > float(max_spread):
            left += 1
        window = ordered[left : right + 1]
        if len(window) > len(inliers):
            inliers = window
    values = [float(reading["value"]) for reading in inliers]
    spread = 0.0 if not values else max(values) - min(values)
    stable_consensus = len(valid) >= int(minimum_valid) and len(inliers) >= int(required_votes)

    if stable_consensus and inliers:
        final_value = float(np.median(values))
        angles = [
            float(reading["pointer_angle"])
            for reading in inliers
            if reading.get("pointer_angle") is not None
        ]
        final_angle = float(np.median(angles)) if angles else None
        selected = min(
            inliers,
            key=lambda reading: (
                abs(float(reading["value"]) - final_value),
                -float(reading.get("quality_score", 0.0) or 0.0),
            ),
        )
        mode = "inlier_median_consensus"
    else:
        selected = max(
            valid,
            key=lambda reading: (
                int(reading.get("pointer_consensus_count", 0) or 0),
                float(reading.get("quality_score", 0.0) or 0.0),
                float(reading.get("frame_quality_score", 0.0) or 0.0),
            ),
        )
        final_value = float(selected["value"])
        final_angle = selected.get("pointer_angle")
        mode = "unstable_best_candidate"

    best_estimate = copy.deepcopy(selected)
    if not stable_consensus:
        return {
            "status": "unstable",
            "reading": None,
            "best_estimate": best_estimate,
            "valid_frame_count": len(valid),
            "consensus_count": len(inliers),
            "spread": round(float(spread), 6),
            "mode": mode,
            "meter_type": str(meter_type),
            "minimum_valid": int(minimum_valid),
            "required_votes": int(required_votes),
            "max_spread": float(max_spread),
            "inlier_observation_indices": [int(value.get("_observation_index", -1)) for value in inliers],
        }

    selected["value"] = round(final_value, 6)
    selected["raw_value"] = round(final_value, 6)
    selected["pointer_angle"] = None if final_angle is None else round(float(final_angle), 6)
    selected["status"] = "ok"
    selected["cabinet_metric"] = {
        "value": round(final_value, 6),
        "raw_value": round(final_value, 6),
        "unit": "V" if str(meter_type) == "voltage" else "A",
        "status": "ok",
        "min": 0,
        "max": 450 if str(meter_type) == "voltage" else 1.0,
    }
    selected["multi_frame"] = {
        "mode": mode,
        "valid_frame_count": len(valid),
        "consensus_count": len(inliers),
        "spread": round(float(spread), 6),
        "meter_type": str(meter_type),
        "minimum_valid": int(minimum_valid),
        "required_votes": int(required_votes),
        "max_spread": float(max_spread),
        "values": [round(float(reading["value"]), 6) for reading in valid],
    }
    return {
        "status": "ok",
        "reading": selected,
        "valid_frame_count": len(valid),
        "consensus_count": len(inliers),
        "spread": round(float(spread), 6),
        "mode": mode,
        "meter_type": str(meter_type),
        "inlier_observation_indices": [int(value.get("_observation_index", -1)) for value in inliers],
    }


def fuse_voltage_readings(
    readings: Sequence[dict[str, Any]],
    *,
    required_votes: int = 3,
    max_spread: float = 6.0,
    minimum_valid: int = 1,
) -> dict[str, Any]:
    result = fuse_meter_readings(
        readings,
        meter_type="voltage",
        minimum_valid=minimum_valid,
        required_votes=required_votes,
        max_spread=max_spread,
    )
    if "spread" in result:
        result["spread_v"] = result["spread"]
    reading = result.get("reading")
    if isinstance(reading, dict) and isinstance(reading.get("multi_frame"), dict):
        reading["multi_frame"]["spread_v"] = reading["multi_frame"].get("spread")
        reading["multi_frame"]["max_spread_v"] = reading["multi_frame"].get("max_spread")
    return result


def best_effort_meter_fusion(
    fusion: dict[str, Any],
    *,
    meter_type: str,
    fallback_value: float,
) -> dict[str, Any]:
    """Promote the best available meter estimate without hiding its quality."""
    result = copy.deepcopy(fusion) if isinstance(fusion, dict) else {}
    reliable = isinstance(result.get("reading"), dict)
    reading = copy.deepcopy(result.get("reading")) if reliable else copy.deepcopy(result.get("best_estimate"))
    source = "multi_frame_consensus" if reliable else "best_observed_frame"
    fallback_used = False
    if not isinstance(reading, dict) or reading.get("value") is None:
        reading = {"value": float(fallback_value), "raw_value": float(fallback_value)}
        source = "configured_fallback"
        fallback_used = True
    try:
        value = float(reading.get("value"))
    except (TypeError, ValueError):
        value = float(fallback_value)
        fallback_used = True
        source = "configured_fallback"
    if not math.isfinite(value):
        value = float(fallback_value)
        fallback_used = True
        source = "configured_fallback"
    is_voltage = str(meter_type) == "voltage"
    unit = "V" if is_voltage else "A"
    maximum = 450 if is_voltage else 1.0
    reading.update(
        {
            "meter_type": str(meter_type),
            "display_name": str(meter_type),
            "value": round(value, 6),
            "raw_value": round(value, 6),
            "unit": unit,
            "status": "ok" if reliable else "estimated",
            "reliable": bool(reliable),
            "best_effort_source": source,
            "cabinet_metric": {
                "value": round(value, 6),
                "raw_value": round(value, 6),
                "unit": unit,
                "status": "ok" if reliable else "estimated",
                "min": 0,
                "max": maximum,
                "reliable": bool(reliable),
                "source": source,
            },
        }
    )
    result.update(
        {
            "status": "ok" if reliable else "estimated",
            "reading": reading,
            "reliable": bool(reliable),
            "fallback_used": bool(fallback_used),
            "best_effort_source": source,
        }
    )
    return result


def best_effort_digit_consensus(consensus: dict[str, Any], fallback_text: str) -> dict[str, Any]:
    result = copy.deepcopy(consensus) if isinstance(consensus, dict) else {}
    reliable = bool(result.get("complete")) and bool(result.get("rows"))
    rows = [str(value).strip() for value in result.get("rows", []) if str(value).strip()]
    source = "multi_frame_consensus" if reliable else "best_observed_text"
    if not rows:
        rows = [str(value).strip() for value in result.get("winner_rows", []) if str(value).strip()]
    if not rows:
        rows = [str(fallback_text).strip() or "0"]
        source = "configured_fallback"
    result.update(
        {
            "rows": rows,
            "winner_rows": [str(value) for value in result.get("winner_rows", [])] or list(rows),
            "complete": True,
            "reliable": bool(reliable),
            "source": source,
            "fallback_used": source == "configured_fallback",
        }
    )
    return result


def best_effort_light_consensus(
    consensus: dict[str, Any],
    fallback_positions: dict[str, str],
) -> dict[str, Any]:
    result = copy.deepcopy(consensus) if isinstance(consensus, dict) else {}
    reliable = bool(result.get("complete"))
    positions = copy.deepcopy(result.get("positions", {})) if isinstance(result.get("positions"), dict) else {}
    winners = result.get("winner_positions", {}) if isinstance(result.get("winner_positions"), dict) else {}
    sources: dict[str, str] = {}
    for key, fallback in fallback_positions.items():
        if key in positions:
            sources[key] = "multi_frame_consensus"
        elif key in winners:
            positions[key] = str(winners[key])
            sources[key] = "best_observed_state"
        else:
            positions[key] = str(fallback)
            sources[key] = "configured_fallback"
    result.update(
        {
            "positions": positions,
            "winner_positions": {**{key: str(value) for key, value in winners.items()}, **positions},
            "complete": True,
            "reliable": bool(reliable),
            "missing": [],
            "position_sources": sources,
            "fallback_used": any(value == "configured_fallback" for value in sources.values()),
        }
    )
    return result


def plan_retakes(
    meter_state: dict[str, Any],
    digits: dict[str, Any],
    lights: dict[str, Any],
    max_count: int,
) -> list[str]:
    stages: list[str] = []
    if any(
        meter_state.get(name, {}).get("status") != "ok"
        or float(meter_state.get(name, {}).get("template_score", 0.0)) < 0.55
        for name in ("voltage", "current")
    ) or not bool(digits.get("complete")):
        stages.extend(["voltage_meter", "current_meter", "temperature_controller"])
    for missing in lights.get("missing", []):
        stage = str(missing).split(":", 1)[0]
        if stage not in stages:
            stages.append(stage)
    return stages[: max(0, int(max_count))]


@dataclass
class Candidate:
    path: Path
    detections: list[dict[str, Any]]
    quality: dict[str, float]
    analysis: dict[str, Any] = field(default_factory=dict)
    sequence: int = 0
    captured_at: float = 0.0


@dataclass
class DeferredCapture:
    stage: str
    scan_pose: list[int]
    candidates: list[Candidate]
    selected: Candidate
    consensus: dict[str, Any] = field(default_factory=dict)
    observation_metadata: list[dict[str, Any]] = field(default_factory=list)
    observation_elapsed_s: float = 0.0
    extended: bool = False
    stable: bool = False
    missing_reason: str = ""


class CabinetAprilTagInspection(ArmAprilTagSearch):
    def __init__(self, args: argparse.Namespace):
        super().__init__(Path(args.search_config), bool(args.execute), bool(args.show))
        self.args = args
        self.expected_tag_id = int(args.expected_tag_id) if args.expected_tag_id is not None else None
        if self.expected_tag_id is not None:
            if self.expected_tag_id not in {10, 11, 12, 13}:
                raise ValueError("expected AprilTag ID must be 10-13")
            self.config["apriltag"]["allowed_ids"] = [self.expected_tag_id]
            self.config["apriltag"]["preferred_id"] = self.expected_tag_id
        self.profile_path = Path(args.profile)
        self.profile = load_profile(self.profile_path)
        self.camera = ContinuousHighResolutionCamera(self.config["camera"], self.profile["capture"])
        self.inspection_id = args.inspection_id or f"insp_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid4().hex[:6]}"
        self.inspection_started_at = utc_now_iso()
        self.board_id = args.board_id or os.environ.get("CAR_CLOUD_BOARD_ID", "rk3588-f103-board")
        self.board_label = args.board_label or os.environ.get("CAR_CLOUD_BOARD_LABEL", "RK3588 F103 Board")
        self.server_url = (args.server_url or os.environ.get("CAR_CLOUD_SERVER_URL", "")).rstrip("/")
        self.upload_token = args.upload_token or os.environ.get("CAR_CLOUD_UPLOAD_TOKEN", "")
        output = self.profile["output"]
        self.status_path = resolve_path(args.status_file) if args.status_file else resolve_path(output["status_file"])
        self.capture_dir = resolve_path(output["capture_dir"]) / self.inspection_id
        self.outbox_dir = resolve_path(output["outbox_dir"])
        self.worker_socket = resolve_path(self.profile["recognition"]["worker_socket"])
        self.model_paths = [resolve_path(value) for value in self.profile["recognition"]["models"]]
        self.light_model_paths = [path for path in self.model_paths if "light" in path.stem.lower()]
        self.digit_model_paths = [
            path
            for path in self.model_paths
            if "digits" in path.stem.lower() and "unified" not in path.stem.lower()
        ]
        self.meter_model_paths = [
            path
            for path in self.model_paths
            if path not in self.light_model_paths and path not in self.digit_model_paths
        ]
        self.meter_configs = {
            "voltage": load_meter_config(METER_READER_DIR / "meter_config_voltage.yaml"),
            "current": load_meter_config(METER_READER_DIR / "meter_config_current.yaml"),
        }
        self.waypoints = [dict(value) for value in self.profile["scan"]["waypoints"]]
        self.waypoint_by_name = {str(value["name"]): value for value in self.waypoints}
        self.aligned_s0 = int(self.home_pwm[0])
        self.current_scan_pose = list(self.home_pwm)
        self.current_waypoint_index = 0
        self.current_waypoint_name = ""
        self.current_transition_step = 0
        self.current_transition_step_count = 0
        self.extension_scale = 0.0
        self.extension_reason = "not_evaluated"
        self.final_home_restored = False
        self.scan_command_retries = 0
        self.scan_stage = ""
        self.scan_state = "INIT"
        self.latest_yolo: list[dict[str, Any]] = []
        self.latest_detection_shape: tuple[int, int] | None = None
        self.latest_inference_ms = 0.0
        self.latest_inference_elapsed_ms = 0.0
        self.last_detection_at = 0.0
        self.live_stream: VisionStreamPublisher | None = None
        self.live_stream_error = ""
        self.live_preview_stop = threading.Event()
        self.live_preview_thread: threading.Thread | None = None
        self.worker_health: dict[str, Any] = {}
        self.photo_records: list[dict[str, Any]] = []
        self.deferred_captures: list[DeferredCapture] = []
        self.digit_observations: list[list[str]] = []
        self.light_observations: list[dict[str, str]] = []
        self.meter_results: list[dict[str, Any]] = []
        self.upload_futures: list[tuple[dict[str, Any], concurrent.futures.Future[dict[str, Any]]]] = []
        self.upload_pipeline_started_at = 0.0
        self.last_recognition_completed_at = 0.0
        self._upload_thread_state = threading.local()
        self.upload_executor: concurrent.futures.ThreadPoolExecutor | None = None
        self.result: dict[str, Any] = {}
        self.navigation_metadata = {
            "navigation_session_id": str(args.navigation_session_id or ""),
            "waypoint_index": int(args.waypoint_index),
            "waypoint_label": str(args.waypoint_label or ""),
            "expected_apriltag_id": self.expected_tag_id,
        }
        self.return_pose_after_preview = False
        self.tag_search_skipped = True
        self.highres_capture_state = "idle"
        self.highres_capture_stage = ""
        self.highres_capture_elapsed_s: float | None = None
        self.highres_capture_error = ""
        self.preview_camera_reopened = True
        self.worker_frame_dir = Path("/dev/shm") / f"car2_cabinet_inspection_{os.getpid()}"
        self.last_inference_sequence = 0
        self.last_inference_captured_at = 0.0
        self.stage_observation_status: dict[str, Any] = {}
        self.stage_consensus_results: dict[str, dict[str, Any]] = {}
        self.stage_failures: dict[str, dict[str, Any]] = {}
        self.stage_drop_counts: dict[str, int] = collections.defaultdict(int)
        self.transient_observation_drop_count = 0
        self.camera_reset_count = 0
        self.stale_detection_drop_count = 0
        self.capture_resolution = [int(self.profile["capture"]["width"]), int(self.profile["capture"]["height"])]
        stream_cfg = self.profile.get("stream", {})
        self.stream_resolution = [int(stream_cfg.get("width", 1280)), int(stream_cfg.get("height", 960))]
        self.stream_fps_target = float(stream_cfg.get("fps", 8.0))
        self.stream_rectifier: StreamRectifier | None = None
        if bool(stream_cfg.get("rectify", False)):
            intrinsics_path = Path(self.config["paths"]["intrinsics"])
            expected_hash = str(self.config["paths"].get("intrinsics_sha256", "")).upper()
            if expected_hash and sha256_file(intrinsics_path) != expected_hash:
                raise RuntimeError("stream camera intrinsics SHA256 mismatch")
            self.stream_rectifier = StreamRectifier(load_yaml(intrinsics_path), self.stream_resolution)
        arm_cfg = ArmConfig.from_dict(self.profile.get("kinematics", {}))
        self.kinematics = ArmKinematics(arm_cfg)
        self._validate_profile()

    def allow_missing_navigation_status(self) -> bool:
        session_id = str(self.navigation_metadata.get("navigation_session_id") or "")
        if not session_id:
            return False
        try:
            status = json.loads(NAV_COMMAND_STATUS_PATH.read_text(encoding="utf-8"))
        except Exception:
            return False
        allowed = navigation_scan_context_matches(
            status,
            session_id=session_id,
            inspection_id=self.inspection_id,
            waypoint_index=int(self.navigation_metadata.get("waypoint_index", -1)),
            expected_tag_id=self.expected_tag_id,
        )
        if allowed:
            self.navigation_status_fallback = "verified_patrol_scan_context"
        return allowed

    def _prepare_direct_scan(self) -> int:
        self.scan_state = "DIRECT_SCAN_READY"
        self.scan_stage = ""
        self.establish_other_axes_home()
        self.assert_safe_state(require_home_axes=True)
        current_s0 = self.current_s0_pwm()
        if current_s0 is None:
            raise SearchFailure("S0 feedback is unavailable before direct cabinet scan")
        home_s0 = int(self.home_pwm[0])
        if abs(int(current_s0) - home_s0) > int(self.config["arm"].get("position_tolerance_pwm", 20)):
            self.move_s0_and_wait(
                home_s0,
                int(self.config["arm"]["return_home_duration_ms"]),
                float(self.config["arm"]["return_home_timeout_s"]),
                retry_count=max(2, int(self.config["safety"].get("arm_command_retry_count", 1))),
            )
        self.final_s0_pwm = home_s0
        self.aligned = True
        self.aligned_tag_distance_mm = None
        self.aligned_tag_distance_spread_mm = None
        self.write_status(
            "ALIGNED",
            direct_scan=True,
            tag_search_skipped=True,
            direct_scan_home_pose=list(self.home_pwm),
        )
        return 0

    @staticmethod
    def _normalize_pose(values: Sequence[Any]) -> list[int]:
        pose = [int(value) for value in values]
        if len(pose) != SERVO_COUNT:
            raise ValueError("scan pose must contain six PWM values")
        return pose

    def _validate_profile(self) -> None:
        names = [str(value.get("name") or "") for value in self.waypoints]
        if names != list(WAYPOINT_SEQUENCE):
            raise ValueError(f"scan waypoints must be {list(WAYPOINT_SEQUENCE)}")
        capture_names = [str(value["name"]) for value in self.waypoints if bool(value.get("capture"))]
        if capture_names != list(CAPTURE_STAGES):
            raise ValueError(f"capture waypoints must be {list(CAPTURE_STAGES)}")
        limits = self.profile["scan"]["servo_limits"]
        s0_delta = int(self.profile["scan"]["s0_max_delta_pwm"])
        for scale in (0.0, 1.0):
            current = list(self.home_pwm)
            for waypoint in self.waypoints:
                motion = resolve_waypoint_motion(
                    waypoint,
                    self.home_pwm[0],
                    scale,
                    int(self.profile["scan"].get("row_duration_ms", 1400)),
                )
                for target, _duration_ms in motion:
                    validate_trajectory(
                        current,
                        target,
                        limits,
                        self.home_pwm[0],
                        s0_delta,
                        self.kinematics,
                    )
                    current = target
            validate_trajectory(
                current,
                list(self.home_pwm),
                limits,
                self.home_pwm[0],
                s0_delta,
                self.kinematics,
            )
        for path in self.model_paths:
            if not path.is_file():
                raise FileNotFoundError(f"recognition model is missing: {path}")
        if not self.light_model_paths or not self.meter_model_paths or not self.digit_model_paths:
            raise ValueError("inspection requires separate light, meter, and digit models")
        if self.args.upload and (not self.server_url or not self.upload_token):
            raise ValueError("--upload requires cloud server URL and upload token")

    def write_status(self, state: str, **extra: Any) -> None:
        extra.setdefault("inspection_id", self.inspection_id)
        for key, value in getattr(self, "navigation_metadata", {}).items():
            extra.setdefault(key, value)
        extra.setdefault("inspection_state", self.scan_state)
        extra.setdefault("inspection_stage", self.scan_stage)
        extra.setdefault("aligned_s0_pwm", self.aligned_s0)
        extra.setdefault("scan_pose", self.current_scan_pose)
        extra.setdefault("actual_servo_pwm", self.current_servo_pwm())
        extra.setdefault("waypoint_index", self.current_waypoint_index)
        extra.setdefault("waypoint_count", len(self.waypoints))
        extra.setdefault("waypoint_name", self.current_waypoint_name)
        extra.setdefault("transition_step", self.current_transition_step)
        extra.setdefault("transition_step_count", self.current_transition_step_count)
        extra.setdefault("extension_scale", round(self.extension_scale, 4))
        extra.setdefault("extension_reason", self.extension_reason)
        extra.setdefault("final_home_restored", self.final_home_restored)
        extra.setdefault("scan_command_retries", self.scan_command_retries)
        extra.setdefault("detection_age_s", None if self.last_detection_at <= 0 else round(time.monotonic() - self.last_detection_at, 3))
        extra.setdefault("inference_ms", round(self.latest_inference_ms, 3))
        extra.setdefault("inference_elapsed_ms", round(self.latest_inference_elapsed_ms, 3))
        extra.setdefault("yolo_ready", bool(self.worker_health))
        extra.setdefault("worker_health", self.worker_health)
        upload_images = bool(self.profile.get("output", {}).get("upload_images", False))
        extra.setdefault("retained_photo_count", len(self.photo_records) if upload_images else 0)
        extra.setdefault("structured_result_count", len(self.photo_records))
        extra.setdefault("result_source", "continuous_stream" if not upload_images else "retained_images")
        extra.setdefault("stale_detection_drop_count", int(getattr(self, "stale_detection_drop_count", 0)))
        extra.setdefault("transient_observation_drop_count", int(self.transient_observation_drop_count))
        extra.setdefault("stage_drop_counts", dict(self.stage_drop_counts))
        extra.setdefault("stage_failures", copy.deepcopy(self.stage_failures))
        extra.setdefault("camera_reset_count", int(self.camera_reset_count))
        extra.setdefault("pending_capture_count", len(self.deferred_captures))
        extra.setdefault("tag_search_skipped", self.tag_search_skipped)
        extra.setdefault("highres_capture_state", self.highres_capture_state)
        extra.setdefault("highres_capture_stage", self.highres_capture_stage)
        extra.setdefault("highres_capture_elapsed_s", self.highres_capture_elapsed_s)
        extra.setdefault("highres_capture_error", self.highres_capture_error)
        extra.setdefault("preview_camera_reopened", self.preview_camera_reopened)
        extra.setdefault("capture_resolution", self.capture_resolution)
        extra.setdefault("stream_resolution", self.stream_resolution)
        extra.setdefault("stream_fps_target", self.stream_fps_target)
        rectifier = self.stream_rectifier
        extra.setdefault("stream_rectified", rectifier is not None)
        extra.setdefault("stream_rectification_model", "" if rectifier is None else rectifier.model)
        extra.setdefault(
            "stream_intrinsics_image_size",
            [] if rectifier is None else list(rectifier.calibration_size),
        )
        extra.setdefault("capture_fps_actual", round(self.camera.actual_fps(), 3))
        extra.setdefault("camera_frame_age_s", self.camera.frame_age_s())
        extra.setdefault("continuous_camera", True)
        extra.setdefault("stage_observation", copy.deepcopy(self.stage_observation_status))
        extra.setdefault("stage_consensus", copy.deepcopy(self.stage_consensus_results.get(self.scan_stage, {})))
        super().write_status(state, **extra)

    def decorate_preview(self, display: np.ndarray) -> np.ndarray:
        detection_age = None if self.last_detection_at <= 0 else time.monotonic() - self.last_detection_at
        visible_detections = (
            self.latest_yolo
            if detection_age is not None
            and detection_age <= float(self.profile["observation"].get("max_detection_age_s", 1.75))
            else []
        )
        source_height, source_width = self.latest_detection_shape or display.shape[:2]
        for item in visible_detections:
            box = detection_box(item)
            if box is None:
                continue
            x1, y1, x2, y2 = scale_box(
                box,
                (source_width, source_height),
                (display.shape[1], display.shape[0]),
            )
            corners = np.asarray(
                [[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
                dtype=np.float64,
            )
            if self.stream_rectifier is not None:
                corners = self.stream_rectifier.rectify_points(corners)
            corners[:, 0] = np.clip(corners[:, 0], 0, display.shape[1] - 1)
            corners[:, 1] = np.clip(corners[:, 1], 0, display.shape[0] - 1)
            polygon = np.rint(corners).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(display, [polygon], True, (255, 180, 0), 2, cv2.LINE_AA)
            label_x = int(np.min(polygon[:, 0, 0]))
            label_y = int(np.min(polygon[:, 0, 1]))
            cv2.putText(
                display,
                f"{detection_label(item)} {float(item.get('confidence') or 0.0):.2f}",
                (label_x, max(18, label_y - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 180, 0),
                1,
                cv2.LINE_AA,
            )
        if self.scan_stage:
            cv2.putText(
                display,
                f"{self.scan_state} / {self.scan_stage}",
                (12, 56),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 220, 255),
                2,
                cv2.LINE_AA,
            )
        return display

    def start_live_stream(self) -> None:
        if self.live_stream is not None and self.live_stream.active:
            return
        stream_cfg = self.profile.get("stream", {})
        publisher = VisionStreamPublisher(
            self.inspection_id,
            "inspection",
            fps=self.stream_fps_target,
            jpeg_quality=int(stream_cfg.get("jpeg_quality", 75)),
        )
        try:
            publisher.start()
            self.live_stream = publisher
            self.live_stream_error = ""
            self.live_preview_stop.clear()
            self.live_preview_thread = threading.Thread(
                target=self._live_preview_loop,
                name="cabinet-live-preview",
                daemon=True,
            )
            self.live_preview_thread.start()
        except Exception as exc:
            self.live_stream = None
            self.live_stream_error = str(exc)
        self.write_status("ALIGNED", live_stream_active=bool(self.live_stream), live_stream_error=self.live_stream_error)

    def stop_live_stream(self) -> None:
        self.live_preview_stop.set()
        preview_thread = self.live_preview_thread
        self.live_preview_thread = None
        if preview_thread is not None and preview_thread.is_alive() and preview_thread is not threading.current_thread():
            preview_thread.join(timeout=1.0)
        publisher = self.live_stream
        self.live_stream = None
        if publisher is not None:
            publisher.stop()

    def cleanup_worker_frames(self) -> None:
        if not self.worker_frame_dir.exists():
            return
        for path in self.worker_frame_dir.iterdir():
            try:
                if path.is_file():
                    path.unlink()
            except OSError:
                pass
        try:
            self.worker_frame_dir.rmdir()
        except OSError:
            pass

    def record_observation_drop(self, exc: Exception, stage: str | None = None) -> None:
        active_stage = str(stage or self.scan_stage or self.current_waypoint_name or "unknown")
        self.transient_observation_drop_count += 1
        self.stage_drop_counts[active_stage] += 1
        self.highres_capture_error = str(exc)
        self.stage_observation_status.update(
            {
                "stage": active_stage,
                "dropped_observations": int(self.stage_drop_counts[active_stage]),
                "last_drop_reason": str(exc),
                "camera_fps": round(self.camera.actual_fps(), 3),
            }
        )
        self.write_status(
            "ALIGNED",
            observation_dropped=True,
            observation_drop_reason=str(exc),
        )

    @staticmethod
    def recoverable_visual_error(exc: Exception) -> bool:
        if isinstance(exc, (ObservationDrop, TimeoutError)):
            return True
        message = str(exc).lower()
        visual_tokens = (
            "camera",
            "fresh frame",
            "empty frame",
            "candidate frame",
            "recognition worker",
            "full recognition",
            "rknn inference",
            "low capture fps",
        )
        safety_tokens = ("f103", "estop", "servo", "chassis", "navigation context", "arm command")
        return any(token in message for token in visual_tokens) and not any(
            token in message for token in safety_tokens
        )

    def reset_highres_camera(self, stage: str, reason: str) -> None:
        self.camera_reset_count += 1
        self.highres_capture_state = "camera_reset"
        self.highres_capture_stage = stage
        self.highres_capture_error = reason
        self.write_status("ALIGNED", camera_reset_reason=reason)
        self.camera.reopen()
        self.last_inference_sequence = self.camera.current_sequence()
        self.preview_camera_reopened = True

    def record_stage_failure(
        self,
        stage: str,
        reason: str,
        *,
        attempts: int,
        retained_path: str = "",
    ) -> dict[str, Any]:
        health = self.camera.health_snapshot(
            float(self.profile["observation"].get("minimum_camera_fps", 4.0))
        )
        record = {
            "stage": stage,
            "label": STAGE_LABELS.get(stage, stage),
            "stage_index": CAPTURE_STAGES.index(stage) + 1,
            "status": "failed",
            "reason": reason,
            "attempts": int(attempts),
            "dropped_observations": int(self.stage_drop_counts.get(stage, 0)),
            "inference_elapsed_ms": round(float(self.latest_inference_elapsed_ms), 3),
            "camera_fps": health.get("fps", 0.0),
            "retained_path": retained_path,
            "updated_at": utc_now_iso(),
        }
        self.stage_failures[stage] = record
        self.stage_observation_status = copy.deepcopy(record)
        self.highres_capture_state = "failed"
        self.highres_capture_stage = stage
        self.highres_capture_error = reason
        self.write_status("ALIGNED", failed_stage=record)
        return record

    def _live_preview_loop(self) -> None:
        while not self.live_preview_stop.is_set():
            try:
                self.publish_live_frame()
            except Exception as exc:
                self.live_stream_error = str(exc)
            self.live_preview_stop.wait(0.01)

    def publish_live_frame(self, frame: np.ndarray | None = None, *, force: bool = False) -> None:
        publisher = self.live_stream
        if publisher is None or not publisher.due(force=force):
            return
        if frame is None:
            latest = self.camera.latest(copy_frame=False)
            if latest is None:
                return
            frame, _sequence, captured_at = latest
            if time.monotonic() - captured_at > float(self.profile["observation"].get("max_camera_age_s", 0.5)):
                return
        detection_age = None if self.last_detection_at <= 0 else time.monotonic() - self.last_detection_at
        stream_width, stream_height = self.stream_resolution
        distorted_display = cv2.resize(frame, (stream_width, stream_height), interpolation=cv2.INTER_AREA)
        clean_display = (
            distorted_display
            if self.stream_rectifier is None
            else self.stream_rectifier.rectify(distorted_display)
        )
        display = self.decorate_preview(clean_display.copy())
        publisher.publish(
            display,
            detection_age_s=detection_age,
            stage=self.scan_stage,
            force=force,
            clean_frame=clean_display,
        )

    def spin_once(self, timeout_s: float = 0.0) -> None:
        super().spin_once(timeout_s)

    def _worker_request(self, payload: dict[str, Any], timeout_s: float = 5.0) -> dict[str, Any]:
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.settimeout(0.10)
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        try:
            client.connect(str(self.worker_socket))
            client.sendall((json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8"))
            chunks: list[bytes] = []
            total = 0
            while True:
                if self.stop_requested:
                    raise SearchFailure("interrupted")
                if time.monotonic() >= deadline:
                    raise TimeoutError("recognition worker request timed out")
                try:
                    chunk = client.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                chunks.append(chunk)
                total += len(chunk)
                if total > 8 * 1024 * 1024:
                    raise RuntimeError("recognition worker response is too large")
                if b"\n" in chunk:
                    break
            response = json.loads(b"".join(chunks).split(b"\n", 1)[0].decode("utf-8"))
            if not isinstance(response, dict) or not response.get("ok"):
                raise RuntimeError(str(response.get("error") if isinstance(response, dict) else response))
            return response
        finally:
            client.close()

    def ensure_worker_ready(self) -> dict[str, Any]:
        if not self.worker_socket.exists():
            raise SearchFailure(f"recognition worker socket is missing: {self.worker_socket}")
        status = self._worker_request({"action": "health"}, timeout_s=3.0)
        models = status.get("models", {})
        for path in self.model_paths:
            backend = str(models.get(path.stem, {}).get("backend") or "") if isinstance(models, dict) else ""
            if backend != "rknn":
                raise SearchFailure(f"model {path.stem} is not using RKNN: {backend or 'missing'}")
        self.worker_health = status
        return status

    def preflight(self) -> None:
        super().preflight()
        health = self.ensure_worker_ready()
        self.write_status("READY", yolo_ready=True, worker_health=health)

    def search(self) -> int:
        if self.tag_search_skipped and not self.args.pose_test:
            result = self._prepare_direct_scan()
            self.aligned_s0 = int(self.final_s0_pwm or self.home_pwm[0])
            self.current_scan_pose = [self.aligned_s0, *self.home_pwm[1:]]
            return result
        if not self.args.pose_test:
            result = super().search()
            if result == 0:
                self.aligned_s0 = int(self.final_s0_pwm or self.current_s0_pwm() or self.home_pwm[0])
                self.current_scan_pose = [self.aligned_s0, *self.home_pwm[1:]]
            return result
        self.establish_other_axes_home()
        self.move_s0_and_wait(
            int(self.home_pwm[0]),
            int(self.config["arm"]["return_home_duration_ms"]),
            float(self.config["arm"]["return_home_timeout_s"]),
            monitor_other_axes=True,
        )
        self.aligned_s0 = int(self.home_pwm[0])
        self.final_s0_pwm = self.aligned_s0
        self.current_scan_pose = [self.aligned_s0, *self.home_pwm[1:]]
        self.aligned = True
        self.write_status("ALIGNED", pose_test=True)
        return 0

    def model_paths_for_stage(self, stage: str) -> list[Path]:
        if stage in LIGHT_STAGES:
            return list(self.light_model_paths)
        if stage in METER_STAGES:
            return list(self.meter_model_paths)
        if stage == "temperature_controller":
            return list(self.digit_model_paths)
        return []

    def infer_file(
        self,
        image_path: Path,
        stage: str,
        image_shape: tuple[int, int] | None = None,
    ) -> list[dict[str, Any]]:
        detections: list[dict[str, Any]] = []
        started = time.monotonic()
        total_infer_ms = 0.0
        confidence_key = "digit_conf" if stage == "temperature_controller" else "conf"
        confidence = float(
            self.profile["recognition"].get(
                confidence_key,
                self.profile["recognition"].get("conf", 0.22),
            )
        )
        for model_path in self.model_paths_for_stage(stage):
            try:
                response = self._worker_request(
                    {
                        "action": "infer",
                        "image_path": str(image_path.resolve()),
                        "model_path": str(model_path.resolve()),
                        "conf": confidence,
                        "iou": float(self.profile["recognition"].get("iou", 0.45)),
                    },
                    timeout_s=float(
                        self.profile["recognition"].get("worker_timeout_s", 5.0)
                    ),
                )
            except TimeoutError as exc:
                raise InferenceFrameTimeout(str(exc)) from exc
            if str(response.get("backend") or "") != "rknn":
                raise SearchFailure(f"worker silently left RKNN for {model_path.stem}")
            total_infer_ms += float(response.get("inference_ms") or 0.0)
            for item in response.get("detections", []):
                if isinstance(item, dict):
                    copy = dict(item)
                    copy["model"] = model_path.stem
                    detections.append(copy)
        elapsed = time.monotonic() - started
        self.latest_inference_ms = total_infer_ms
        self.latest_inference_elapsed_ms = elapsed * 1000.0
        validate_inference_elapsed(elapsed, self.profile["recognition"])
        self.latest_yolo = detections
        self.latest_detection_shape = image_shape
        self.last_detection_at = time.monotonic()
        return detections

    def infer_preview_frame(self, stage: str | None = None) -> list[dict[str, Any]]:
        active_stage = str(stage if stage is not None else self.scan_stage)
        maximum_camera_age = float(self.profile["observation"].get("max_camera_age_s", 0.5))
        maximum_start_age = float(self.profile["observation"].get("max_inference_start_age_s", 0.20))
        frame_wait_timeout = float(
            self.profile["observation"].get("frame_wait_timeout_s", 1.5)
        )
        while True:
            frame, sequence, captured_at = self.camera.wait_latest(
                self.last_inference_sequence,
                timeout_s=frame_wait_timeout,
                copy_frame=False,
            )
            self.last_inference_sequence = sequence
            self.last_inference_captured_at = captured_at
            frame_age = time.monotonic() - captured_at
            if frame_age <= maximum_start_age:
                break
        if frame_age > float(self.profile["observation"].get("max_camera_age_s", 0.5)):
            raise SearchFailure(f"camera frame is stale: {frame_age:.3f}s")
        self.latest_frame = frame
        if not self.model_paths_for_stage(active_stage):
            self.latest_yolo = []
            self.latest_detection_shape = frame.shape[:2]
            self.last_detection_at = 0.0
            return []
        recognition_cfg = self.profile["recognition"]
        inference_width = int(recognition_cfg.get("input_width", 1280))
        inference_height = int(recognition_cfg.get("input_height", 960))
        inference_frame = cv2.resize(frame, (inference_width, inference_height), interpolation=cv2.INTER_AREA)
        path = self.worker_frame_dir / f"{active_stage}_latest.jpg"
        atomic_write_jpeg(path, inference_frame, int(self.profile["capture"].get("worker_jpeg_quality", 88)))
        lowres_detections = self.infer_file(path, active_stage, inference_frame.shape[:2])
        detections = scale_detections(
            lowres_detections,
            (inference_width, inference_height),
            (frame.shape[1], frame.shape[0]),
        )
        self.latest_yolo = detections
        self.latest_detection_shape = frame.shape[:2]
        detection_age = time.monotonic() - captured_at
        maximum_age = float(self.profile["observation"].get("max_detection_age_s", 0.6))
        if detection_age > maximum_age:
            self.stale_detection_drop_count += 1
            self.transient_observation_drop_count += 1
            self.stage_drop_counts[active_stage or "unknown"] += 1
            self.latest_yolo = []
            self.last_detection_at = 0.0
            self.update_preview(frame, force=True)
            self.write_status(
                "ALIGNED",
                stale_detection_dropped=True,
                stale_detection_age_s=round(detection_age, 3),
                stale_detection_drop_count=self.stale_detection_drop_count,
            )
            return []
        self.update_preview(frame, force=True)
        return detections

    def _limits(self) -> Sequence[Sequence[int]]:
        return self.profile["scan"]["servo_limits"]

    def _validated_target(self, target: Sequence[int]) -> list[int]:
        pose = [int(value) for value in target]
        delta = int(self.profile["scan"]["s0_max_delta_pwm"])
        validate_trajectory(
            self.current_scan_pose,
            pose,
            self._limits(),
            self.aligned_s0,
            delta,
            self.kinematics,
        )
        return pose

    def emergency_stop(self) -> None:
        if self.node is None:
            return
        try:
            self.node.publish_arm_payload("#255PDST!")
        except Exception:
            pass
        for _ in range(3):
            try:
                self.node.publish_zero()
            except Exception:
                pass
            time.sleep(0.03)

    def wait_motion(self, duration_s: float, *, run_detection: bool) -> None:
        deadline = time.monotonic() + max(0.0, float(duration_s))
        next_detection_at = 0.0
        interval = 1.0 / max(0.1, float(self.profile["recognition"].get("motion_detection_hz", 2.0)))
        while time.monotonic() < deadline:
            self.spin_once(0.02)
            self.assert_safe_state(require_home_axes=False)
            if run_detection and time.monotonic() >= next_detection_at:
                detection_started_at = time.monotonic()
                try:
                    self.infer_preview_frame()
                except ObservationDrop as exc:
                    self.record_observation_drop(exc)
                next_detection_at = max(detection_started_at + interval, time.monotonic())

    def move_scan_pose(self, target: Sequence[int], duration_ms: int, *, detect_during_motion: bool = True) -> None:
        pose = self._validated_target(target)
        if self.node is None:
            raise SearchFailure("ROS node is unavailable")
        changed = changed_servo_targets(self.current_scan_pose, pose)
        if not changed:
            return
        if detect_during_motion:
            try:
                self.infer_preview_frame()
            except ObservationDrop as exc:
                self.record_observation_drop(exc)
        retries = max(0, int(self.profile["scan"].get("command_retry_count", 1)))
        backoff_s = max(0.0, float(self.profile["scan"].get("command_retry_backoff_s", 0.25)))
        tolerance = int(self.config["safety"]["servo_target_tolerance_pwm"])
        actual: list[int] | None = None
        for attempt in range(retries + 1):
            self.write_status(
                "ALIGNED",
                command_targets={f"S{index}": value for index, value in changed},
                command_duration_ms=int(duration_ms),
                command_attempt=attempt + 1,
            )
            self.node.publish_servo_targets(changed, int(duration_ms))
            self.wait_motion(int(duration_ms) / 1000.0, run_detection=detect_during_motion)
            confirm_deadline = time.monotonic() + float(self.profile["scan"].get("command_retain_timeout_s", 1.0))
            while time.monotonic() < confirm_deadline:
                self.spin_once(0.05)
                self.assert_safe_state(require_home_axes=False)
                actual = self.current_servo_pwm()
                if actual is not None and all(abs(actual[index] - value) <= tolerance for index, value in changed):
                    self.current_scan_pose = pose
                    return
            if attempt >= retries:
                break
            self.scan_command_retries += 1
            retry_deadline = time.monotonic() + backoff_s
            while time.monotonic() < retry_deadline:
                self.spin_once(0.02)
                self.assert_safe_state(require_home_axes=False)
        self.write_status(
            "FAILED",
            command_targets={f"S{index}": value for index, value in changed},
            command_actual_pwm=actual,
        )
        raise SearchFailure(f"F103 did not retain scan targets {changed}; actual={actual}")

    def settle_pose(self) -> None:
        self.wait_motion(float(self.profile["scan"].get("settle_s", 0.6)), run_detection=True)

    def center_stage(self, stage: str) -> dict[str, Any]:
        cfg = self.profile["scan"]
        max_steps = int(cfg.get("max_center_steps", 3))
        horizontal_tolerance = float(cfg.get("horizontal_tolerance_ratio", 0.04))
        history: list[dict[str, Any]] = []
        for _ in range(max_steps):
            try:
                detections = self.infer_preview_frame()
            except ObservationDrop as exc:
                self.record_observation_drop(exc, stage)
                continue
            selected = select_stage_detections(stage, detections)
            box = union_box(selected)
            if box is None:
                break
            width = float(self.latest_frame.shape[1])
            error_x = ((box[0] + box[2]) * 0.5) - width * 0.5
            history.append(
                {
                    "error_x_px": round(error_x, 2),
                    "error_x_ratio": round(error_x / max(1.0, width), 5),
                    "target_count": len(selected),
                }
            )
            if abs(error_x) <= width * horizontal_tolerance:
                break
            target = list(self.current_scan_pose)
            delta = int(round(float(self.config["alignment"]["pwm_per_px"]) * error_x))
            minimum = int(self.config["alignment"]["min_step_pwm"])
            maximum = int(self.config["alignment"]["max_step_pwm"])
            if abs(delta) < minimum:
                delta = minimum if delta >= 0 else -minimum
            delta = clamp_int(delta, -maximum, maximum)
            low = max(int(self.config["arm"]["min_pwm"]), self.aligned_s0 - int(cfg["s0_max_delta_pwm"]))
            high = min(int(self.config["arm"]["max_pwm"]), self.aligned_s0 + int(cfg["s0_max_delta_pwm"]))
            target[0] = clamp_int(target[0] + delta, low, high)
            if target[0] == self.current_scan_pose[0]:
                break
            duration_ms = s0_motion_duration_ms(self.current_scan_pose[0], target[0], cfg)
            self.move_scan_pose(target, duration_ms)
            self.settle_pose()
        final_error = None if not history else history[-1]["error_x_ratio"]
        return {
            "history": history,
            "steps": len(history),
            "final_error_ratio": final_error,
            "centered": final_error is not None and abs(float(final_error)) <= horizontal_tolerance,
        }

    def refresh_safety_after_blocking_io(self) -> None:
        """Consume queued ROS state after camera or inference calls block the main loop."""
        timeout_s = max(0.25, float(self.config["safety"]["f103_state_timeout_s"]))
        self.wait_state(timeout_s, require_home_axes=False)

    def analyze_meter_frame(
        self,
        stage: str,
        frame: np.ndarray,
        detections: Sequence[dict[str, Any]],
        observation_index: int,
    ) -> dict[str, Any]:
        meter_type = "voltage" if stage == "voltage_meter" else "current"
        selected = select_stage_detections(stage, detections)
        detection = max(selected, key=lambda item: float(item.get("confidence") or 0.0)) if selected else None
        if detection is None:
            return {"valid": False, "meter_type": meter_type, "reason": "meter_not_detected"}
        box = detection_box(detection)
        if box is None:
            return {"valid": False, "meter_type": meter_type, "reason": "meter_box_invalid"}
        config = self.meter_configs[meter_type]
        result = read_meter_from_box(
            frame,
            box,
            config,
            det_conf=float(detection.get("confidence") or 0.0),
            margin_ratio=0.12,
        )
        initial_quality = dict(result.rectify_quality or {})
        fallback_attempt: dict[str, Any] = {}
        if result.value is None or bool(initial_quality.get("geometry_fallback")):
            fallback_attempt = {
                "initial_status": result.status,
                "initial_value": result.value,
                "initial_rectify_method": result.rectify_method,
                "retry_margin": 0.20,
            }
            retry_result = read_meter_from_box(
                frame,
                box,
                config,
                det_conf=float(detection.get("confidence") or 0.0),
                margin_ratio=0.20,
                allow_relaxed_quad=True,
            )
            fallback_attempt.update(
                {
                    "retry_status": retry_result.status,
                    "retry_value": retry_result.value,
                    "retry_rectify_method": retry_result.rectify_method,
                    "accepted": retry_result.value is not None,
                }
            )
            if retry_result.value is not None:
                result = retry_result
        payload = meter_result_payload(
            result,
            float(detection.get("confidence") or 0.0),
            box,
            meter_type=meter_type,
        )
        payload["_observation_index"] = int(observation_index)
        if fallback_attempt:
            payload["fallback_attempt"] = fallback_attempt
        return {
            "valid": payload.get("value") is not None,
            "meter_type": meter_type,
            "meter_reading": payload,
            "reason": "" if payload.get("value") is not None else str(payload.get("status") or "meter_unreadable"),
            "_observation_index": int(observation_index),
        }

    @staticmethod
    def analyze_temperature_frame(
        frame: np.ndarray,
        detections: Sequence[dict[str, Any]],
        observation_index: int,
    ) -> dict[str, Any]:
        del frame
        yolo_rows = digit_text_rows(detections)
        yolo_text = str(yolo_rows[0]).strip() if yolo_rows else ""
        digit_detections = [item for item in detections if detection_label(item) in DIGIT_LABELS]
        confidences = [float(item.get("confidence") or 0.0) for item in digit_detections]
        return {
            "valid": len(yolo_text) >= 2,
            "status": "ok" if len(yolo_text) >= 2 else "digits_not_detected",
            "upper_text": yolo_text,
            "rows": yolo_rows,
            "yolo_text": yolo_text,
            "verification_source": "digit_yolo_incremental",
            "method": "digit_yolo_incremental",
            "quality_score": round(sum(confidences) / max(1, len(confidences)), 6),
            "detected_digit_count": len(digit_detections),
            "_observation_index": int(observation_index),
        }

    def stage_consensus(self, stage: str, observations: Sequence[dict[str, Any]]) -> dict[str, Any]:
        quality_cfg = self.profile["quality"]
        analyses = [value.get("analysis", {}) for value in observations]
        if stage in METER_STAGES:
            meter_type = "voltage" if stage == "voltage_meter" else "current"
            readings = [
                value["meter_reading"]
                for value in analyses
                if isinstance(value, dict) and isinstance(value.get("meter_reading"), dict)
            ]
            return fuse_meter_readings(
                readings,
                meter_type=meter_type,
                minimum_valid=int(quality_cfg.get("meter_min_valid_results", 5)),
                required_votes=int(quality_cfg.get("meter_min_inliers", 3)),
                max_spread=float(
                    quality_cfg.get("voltage_unstable_delta", 6.0)
                    if meter_type == "voltage"
                    else quality_cfg.get("current_unstable_delta", 0.05)
                ),
            )
        if stage == "temperature_controller":
            digit_observations = [
                [str(value.get("upper_text") or "")]
                for value in analyses
                if isinstance(value, dict) and bool(value.get("valid"))
            ]
            return digit_consensus(
                digit_observations,
                required_votes=int(quality_cfg.get("digit_min_winner_votes", 3)),
                minimum_observations=int(quality_cfg.get("digit_min_valid_results", 5)),
                minimum_ratio=float(quality_cfg.get("digit_min_winner_ratio", 0.60)),
            )
        expected = {f"{stage}:green", f"{stage}:red"}
        light_observations = [
            value.get("light_observation", {})
            for value in analyses
            if isinstance(value, dict) and isinstance(value.get("light_observation"), dict)
        ]
        return light_consensus(
            light_observations,
            required_votes=int(quality_cfg.get("light_min_winner_votes", 4)),
            minimum_observations=int(quality_cfg.get("light_min_valid_results", 5)),
            minimum_ratio=float(quality_cfg.get("light_min_winner_ratio", 0.70)),
            expected_positions=expected,
        )

    @staticmethod
    def consensus_is_stable(consensus: dict[str, Any]) -> bool:
        return bool(consensus.get("reading")) if "reading" in consensus else bool(consensus.get("complete"))

    @staticmethod
    def consensus_missing_reason(stage: str, consensus: dict[str, Any]) -> str:
        if stage in METER_STAGES:
            if int(consensus.get("valid_frame_count", 0)) < int(consensus.get("minimum_valid", 5)):
                return "insufficient_valid_meter_samples"
            return "meter_reading_unstable"
        if stage == "temperature_controller":
            return "digit_consensus_unstable"
        missing = consensus.get("missing", [])
        return "light_consensus_missing:" + ",".join(str(value) for value in missing)

    def select_consensus_candidate(
        self,
        stage: str,
        candidates: Sequence[Candidate],
        consensus: dict[str, Any],
    ) -> Candidate:
        pool = list(candidates)
        if stage in METER_STAGES:
            inliers = {int(value) for value in consensus.get("inlier_observation_indices", [])}
            matched = [value for value in pool if int(value.analysis.get("_observation_index", -1)) in inliers]
            if matched:
                pool = matched
        elif stage == "temperature_controller":
            winner_rows = consensus.get("winner_rows", [])
            winner = str(winner_rows[0]) if winner_rows else ""
            matched = [value for value in pool if str(value.analysis.get("upper_text") or "") == winner]
            if matched:
                pool = matched
        elif stage in LIGHT_STAGES:
            winners = consensus.get("winner_positions", {})
            matched = [
                value
                for value in pool
                if all(value.analysis.get("light_observation", {}).get(key) == state for key, state in winners.items())
            ]
            if matched:
                pool = matched
        return max(pool, key=lambda value: float(value.quality.get("score", 0.0)))

    def retain_candidate(
        self,
        stage: str,
        frame: np.ndarray,
        detections: list[dict[str, Any]],
        quality: dict[str, float],
        analysis: dict[str, Any],
        sequence: int,
        captured_at: float,
        candidates: list[Candidate],
    ) -> None:
        maximum = int(self.profile["capture"].get("max_candidates", 10))
        if len(candidates) >= maximum:
            worst = min(candidates, key=lambda value: float(value.quality.get("score", 0.0)))
            if float(quality.get("score", 0.0)) <= float(worst.quality.get("score", 0.0)):
                return
            candidates.remove(worst)
            try:
                worst.path.unlink()
            except OSError:
                pass
        path = self.capture_dir / "candidates" / f"{stage}_{int(analysis['_observation_index']):03d}.jpg"
        atomic_write_jpeg(path, frame, int(self.profile["capture"].get("jpeg_quality", 95)))
        candidates.append(Candidate(path, copy.deepcopy(detections), quality, analysis, sequence, captured_at))

    def highres_candidates(
        self,
        stage: str,
        _capture_index: int,
    ) -> tuple[list[Candidate], list[dict[str, Any]], dict[str, Any], float, bool, bool, str]:
        observation_cfg = self.profile["observation"]
        primary_s = float(observation_cfg.get("primary_duration_s", 6.0))
        extension_s = float(observation_cfg.get("extension_duration_s", 3.0))
        minimum_s = float(observation_cfg.get("minimum_duration_s", 2.5))
        started_at = time.monotonic()
        extended = False
        stable = False
        candidates: list[Candidate] = []
        observations: list[dict[str, Any]] = []
        consensus: dict[str, Any] = {}
        self.last_inference_sequence = self.camera.current_sequence()
        self.highres_capture_state = "continuous_observation"
        self.highres_capture_stage = stage
        self.highres_capture_error = ""
        last_status_at = 0.0
        consecutive_camera_timeouts = 0
        consecutive_inference_timeouts = 0
        terminal_visual_error = ""
        minimum_camera_fps = float(observation_cfg.get("minimum_camera_fps", 4.0))
        low_fps_failure_s = float(observation_cfg.get("low_fps_failure_s", 3.0))

        while True:
            now = time.monotonic()
            action = observation_action(
                now - started_at,
                stable,
                extended,
                minimum_s=minimum_s,
                primary_s=primary_s,
                extension_s=extension_s,
            )
            if action == "complete" or action == "timeout":
                break
            if action == "extend":
                extended = True
            self.spin_once(0.0)
            self.assert_safe_state(require_home_axes=False)
            try:
                detections = self.infer_preview_frame(stage)
                consecutive_camera_timeouts = 0
                consecutive_inference_timeouts = 0
            except CameraFrameTimeout as exc:
                consecutive_camera_timeouts += 1
                self.record_observation_drop(exc, stage)
                if consecutive_camera_timeouts >= int(
                    observation_cfg.get("max_consecutive_frame_timeouts", 2)
                ):
                    terminal_visual_error = str(exc)
                    break
                continue
            except InferenceFrameTimeout as exc:
                consecutive_inference_timeouts += 1
                self.record_observation_drop(exc, stage)
                if consecutive_inference_timeouts >= int(
                    observation_cfg.get("max_consecutive_inference_timeouts", 3)
                ):
                    terminal_visual_error = str(exc)
                    break
                continue
            health = self.camera.health_snapshot(minimum_camera_fps)
            if float(health.get("low_fps_duration_s") or 0.0) >= low_fps_failure_s:
                terminal_visual_error = (
                    f"low capture FPS {health.get('fps')} persisted for "
                    f"{health.get('low_fps_duration_s')}s"
                )
                self.record_observation_drop(CameraFrameTimeout(terminal_visual_error), stage)
                break
            frame = self.latest_frame
            if frame is None:
                raise SearchFailure("high-resolution camera returned an empty frame")
            observation_index = len(observations)
            selected = select_stage_detections(stage, detections)
            quality = frame_quality(frame, selected)
            if stage in METER_STAGES:
                analysis = self.analyze_meter_frame(stage, frame, detections, observation_index)
                reading = analysis.get("meter_reading", {})
                quality["rectify_quality"] = round(float(reading.get("quality_score", 0.0) or 0.0), 6)
                quality["score"] = round(
                    float(quality["score"]) + min(0.20, float(reading.get("quality_score", 0.0) or 0.0) * 0.04),
                    6,
                )
            elif stage == "temperature_controller":
                analysis = self.analyze_temperature_frame(frame, detections, observation_index)
                quality["digit_cross_verified"] = 1.0 if analysis.get("cross_verified") else 0.0
                quality["score"] = round(
                    float(quality["score"])
                    + min(0.20, float(analysis.get("quality_score", 0.0) or 0.0) * 0.05)
                    + (0.10 if analysis.get("cross_verified") else 0.0),
                    6,
                )
            else:
                observation = light_observation(stage, detections)
                analysis = {
                    "valid": len(observation) == 2,
                    "light_observation": observation,
                    "_observation_index": observation_index,
                }

            analysis.setdefault("_observation_index", observation_index)

            self.refresh_safety_after_blocking_io()
            captured_at = self.last_inference_captured_at
            metadata = {
                "observation_index": observation_index,
                "sequence": int(self.last_inference_sequence),
                "elapsed_s": round(time.monotonic() - started_at, 3),
                "detections": copy.deepcopy(detections),
                "quality": copy.deepcopy(quality),
                "analysis": copy.deepcopy(analysis),
            }
            observations.append(metadata)
            self.retain_candidate(
                stage,
                frame,
                detections,
                quality,
                analysis,
                self.last_inference_sequence,
                captured_at,
                candidates,
            )
            consensus = self.stage_consensus(stage, observations)
            stable = self.consensus_is_stable(consensus)
            elapsed_s = time.monotonic() - started_at
            raw_valid_counts = consensus.get("valid_observation_counts", [])
            if isinstance(raw_valid_counts, dict):
                valid_sample_count = min((int(value) for value in raw_valid_counts.values()), default=0)
            elif isinstance(raw_valid_counts, list):
                valid_sample_count = int(raw_valid_counts[0]) if raw_valid_counts else 0
            else:
                valid_sample_count = 0
            valid_sample_count = int(consensus.get("valid_frame_count", 0) or valid_sample_count)
            self.stage_observation_status = {
                "stage": stage,
                "elapsed_s": round(elapsed_s, 3),
                "valid_sample_count": valid_sample_count,
                "candidate_count": len(candidates),
                "extended": extended,
                "stable": stable,
                "votes": copy.deepcopy(consensus.get("votes", consensus.get("winner_votes", {}))),
                "meter_spread": consensus.get("spread"),
                "missing_reason": "" if stable else self.consensus_missing_reason(stage, consensus),
                "dropped_observations": int(self.stage_drop_counts.get(stage, 0)),
                "camera_fps": round(self.camera.actual_fps(), 3),
            }
            if time.monotonic() - last_status_at >= 0.5:
                self.write_status("ALIGNED")
                last_status_at = time.monotonic()
            if observation_action(
                elapsed_s,
                stable,
                extended,
                minimum_s=minimum_s,
                primary_s=primary_s,
                extension_s=extension_s,
            ) == "complete":
                break

        if not candidates:
            if terminal_visual_error:
                raise CameraFrameTimeout(terminal_visual_error)
            raise ObservationDrop(f"no candidate frame retained for {stage}")
        elapsed_s = time.monotonic() - started_at
        stable = self.consensus_is_stable(consensus)
        missing_reason = (
            terminal_visual_error
            if terminal_visual_error
            else "" if stable else self.consensus_missing_reason(stage, consensus)
        )
        self.stage_consensus_results[stage] = copy.deepcopy(consensus)
        self.highres_capture_state = "completed" if stable else "partial"
        self.highres_capture_elapsed_s = round(elapsed_s, 3)
        self.stage_observation_status.update(
            {
                "elapsed_s": round(elapsed_s, 3),
                "extended": extended,
                "stable": stable,
                "missing_reason": missing_reason,
                "dropped_observations": int(self.stage_drop_counts.get(stage, 0)),
                "camera_fps": round(self.camera.actual_fps(), 3),
            }
        )
        self.write_status("ALIGNED")
        return candidates, observations, consensus, elapsed_s, extended, stable, missing_reason

    def run_full_recognition(self, source: Path, stage: str, scan_pose: Sequence[int]) -> dict[str, Any]:
        output_dir = self.capture_dir / "retained"
        output_dir.mkdir(parents=True, exist_ok=True)
        prefix = stage
        command = [
            "python3",
            str(ROOT_DIR / "ros2_tools" / "camera_yolo_detect.py"),
            "--source",
            str(source),
            "--output-dir",
            str(output_dir),
            "--prefix",
            prefix,
            "--rotate",
            "0",
            "--backend",
            "rknn",
            "--worker-socket",
            str(self.worker_socket),
            "--conf",
            str(float(self.profile["recognition"].get("conf", 0.22))),
            "--warmup-frames",
            "0",
        ]
        if stage in LIGHT_STAGES:
            command.extend(["--no-meter-readings", "--no-digits"])
        elif stage == "temperature_controller":
            command.append("--no-meter-readings")
        elif stage in METER_STAGES:
            command.append("--no-digits")
        process = subprocess.Popen(
            command,
            cwd=str(ROOT_DIR),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 90.0
        while process.poll() is None:
            self.spin_once(0.02)
            try:
                self.assert_safe_state(require_home_axes=False)
            except Exception:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                self.emergency_stop()
                raise
            if time.monotonic() >= deadline:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                raise SearchFailure("full recognition timed out")
            time.sleep(0.03)
        stdout, stderr = process.communicate(timeout=1.0)
        if process.returncode != 0:
            raise SearchFailure(f"full recognition failed: {stderr.strip()[-500:]}")
        try:
            result = json.loads(stdout)
        except json.JSONDecodeError as exc:
            raise SearchFailure(f"full recognition returned invalid JSON: {exc}") from exc
        backend = str(result.get("backend") or "")
        if not backend or any("rknn" not in part for part in backend.split(",")):
            raise SearchFailure(f"full recognition left RKNN: {backend or 'missing'}")
        result.update(
            {
                "inspection_id": self.inspection_id,
                "inspection_stage": prefix,
                "capture_key": f"{self.inspection_id}:{prefix}",
                "board_id": self.board_id,
                "board_label": self.board_label,
                "scan_pose": [int(value) for value in scan_pose],
            }
        )
        result_path = Path(str(result["annotated_image"])).with_name(f"{prefix}_det.json")
        atomic_write_json(result_path, result)
        result["result_json"] = str(result_path)
        return result

    def queue_stage_capture(self, stage: str) -> DeferredCapture:
        self.scan_state = "CAPTURE"
        self.write_status("ALIGNED", capture_index=1)
        (
            candidates,
            observations,
            consensus,
            elapsed_s,
            extended,
            stable,
            missing_reason,
        ) = self.highres_candidates(stage, 1)
        if not candidates:
            raise SearchFailure("no high-resolution candidates captured")
        selected = self.select_consensus_candidate(stage, candidates, consensus)
        selected.quality["consensus_member"] = 1.0 if stable else 0.0
        if stage == "temperature_controller":
            self.digit_observations.extend(
                [[str(value.get("analysis", {}).get("upper_text") or "")]]
                for value in observations
                if bool(value.get("analysis", {}).get("valid"))
            )
        if stage in LIGHT_STAGES:
            self.light_observations.extend(
                copy.deepcopy(value.get("analysis", {}).get("light_observation", {}))
                for value in observations
                if value.get("analysis", {}).get("light_observation")
            )
        pending = DeferredCapture(
            stage,
            list(self.current_scan_pose),
            candidates,
            selected,
            copy.deepcopy(consensus),
            observations,
            elapsed_s,
            extended,
            stable,
            missing_reason,
        )
        self.deferred_captures.append(pending)
        self.write_status(
            "ALIGNED",
            queued_capture_stage=stage,
            queued_capture_quality=selected.quality,
            queued_capture_stable=stable,
            queued_capture_extended=extended,
            queued_capture_missing_reason=missing_reason,
        )
        return pending

    def queue_stage_capture_with_recovery(self, stage: str) -> DeferredCapture | None:
        last_error = ""
        for attempt in range(1, 3):
            try:
                pending = self.queue_stage_capture(stage)
                self.stage_failures.pop(stage, None)
                return pending
            except Exception as exc:
                if not self.recoverable_visual_error(exc):
                    raise
                last_error = str(exc)
                self.record_observation_drop(exc, stage)
                if attempt >= 2:
                    break
                if "camera" in last_error.lower() or "frame" in last_error.lower():
                    self.reset_highres_camera(stage, last_error)
                else:
                    try:
                        self.ensure_worker_ready()
                    except Exception as health_exc:
                        last_error = str(health_exc)
                        break
                self.write_status(
                    "ALIGNED",
                    stage_retry=attempt + 1,
                    stage_retry_reason=last_error,
                )
        self.record_stage_failure(stage, last_error or "stage capture failed", attempts=2)
        return None

    def analyze_voltage_candidates(
        self,
        candidates: Sequence[Candidate],
    ) -> tuple[Candidate, dict[str, Any]]:
        config = load_meter_config(METER_READER_DIR / "meter_config_voltage.yaml")
        readings: list[dict[str, Any]] = []
        for index, candidate in enumerate(candidates):
            detections = select_stage_detections("voltage_meter", candidate.detections)
            detection = max(detections, key=lambda item: float(item.get("confidence") or 0.0)) if detections else None
            if detection is None:
                continue
            image = cv2.imread(str(candidate.path), cv2.IMREAD_COLOR)
            if image is None:
                continue
            box = detection_box(detection)
            if box is None:
                continue
            result = read_meter_from_box(
                image,
                box,
                config,
                det_conf=float(detection.get("confidence") or 0.0),
                margin_ratio=0.12,
            )
            fallback_attempt: dict[str, Any] = {}
            initial_quality = dict(result.rectify_quality or {})
            if result.value is None or bool(initial_quality.get("geometry_fallback")):
                fallback_attempt = {
                    "initial_status": result.status,
                    "initial_value": result.value,
                    "initial_rectify_method": result.rectify_method,
                    "retry_margin": 0.20,
                }
                retry_result = read_meter_from_box(
                    image,
                    box,
                    config,
                    det_conf=float(detection.get("confidence") or 0.0),
                    margin_ratio=0.20,
                    allow_relaxed_quad=True,
                )
                fallback_attempt.update(
                    {
                        "retry_status": retry_result.status,
                        "retry_value": retry_result.value,
                        "retry_rectify_method": retry_result.rectify_method,
                        "accepted": retry_result.value is not None,
                    }
                )
                if retry_result.value is not None:
                    result = retry_result
            self.refresh_safety_after_blocking_io()
            payload = meter_result_payload(result, float(detection.get("confidence") or 0.0), box)
            if fallback_attempt:
                payload["fallback_attempt"] = fallback_attempt
            payload["frame_quality_score"] = round(float(candidate.quality.get("score", 0.0)), 6)
            payload["_candidate_index"] = index
            readings.append(payload)

        fusion = fuse_voltage_readings(
            readings,
            required_votes=int(self.profile["quality"].get("voltage_consensus_votes", 3)),
            max_spread=float(self.profile["quality"].get("voltage_unstable_delta", 6.0)),
        )
        reading = fusion.get("reading")
        if not isinstance(reading, dict):
            selected = max(candidates, key=lambda item: item.quality["score"])
            return selected, fusion
        candidate_index = int(reading.pop("_candidate_index", 0))
        return candidates[candidate_index], fusion

    @staticmethod
    def select_temperature_candidate(
        candidates: Sequence[Candidate],
        required_votes: int,
    ) -> tuple[Candidate, dict[str, Any]]:
        valid = [
            (index, candidate)
            for index, candidate in enumerate(candidates)
            if bool(candidate.analysis.get("valid"))
            and len(str(candidate.analysis.get("upper_text") or "")) >= 2
        ]
        if not valid:
            selected = max(candidates, key=lambda item: item.quality["score"])
            return selected, {
                "status": "missing",
                "upper_text": "",
                "rows": [],
                "valid_frame_count": 0,
                "consensus_count": 0,
                "method": "digit_yolo_incremental",
            }

        votes = collections.Counter(str(candidate.analysis["upper_text"]) for _index, candidate in valid)
        text, vote_count = votes.most_common(1)[0]
        matching = [(index, candidate) for index, candidate in valid if candidate.analysis["upper_text"] == text]
        pool = matching if vote_count >= int(required_votes) else valid
        selected_index, selected = max(
            pool,
            key=lambda item: (
                float(item[1].analysis.get("quality_score", 0.0) or 0.0),
                float(item[1].quality.get("score", 0.0) or 0.0),
            ),
        )
        if vote_count < int(required_votes):
            text = str(selected.analysis["upper_text"])
            vote_count = votes[text]
        reading = copy.deepcopy(selected.analysis)
        reading.update(
            {
                "status": "ok",
                "upper_text": text,
                "rows": [text],
                "valid": True,
                "reliable": vote_count >= int(required_votes),
                "valid_frame_count": len(valid),
                "consensus_count": int(vote_count),
                "required_votes": int(required_votes),
                "source": "multi_frame_yolo_consensus"
                if vote_count >= int(required_votes)
                else "best_yolo_frame",
                "frame_results": [
                    {
                        "candidate_index": index,
                        "upper_text": str(candidate.analysis.get("upper_text") or ""),
                        "status": str(candidate.analysis.get("status") or ""),
                        "quality_score": candidate.analysis.get("quality_score"),
                    }
                    for index, candidate in valid
                ],
                "selected_candidate_index": selected_index,
            }
        )
        return selected, reading

    @staticmethod
    def apply_temperature_result(result: dict[str, Any], reading: dict[str, Any]) -> None:
        if not reading.get("upper_text"):
            result["yolo_digit_detections"] = copy.deepcopy(result.get("digit_detections", []))
            result["digit_readings"] = {"global_text": "", "rows": []}
            result["digit_detections"] = []
            result["temperature_controller"] = copy.deepcopy(reading)
            return
        text = str(reading["upper_text"])
        result["yolo_digit_detections"] = copy.deepcopy(result.get("digit_detections", []))
        result["temperature_controller"] = copy.deepcopy(reading)
        result["digit_readings"] = {
            "global_text": text,
            "rows": [
                {
                    "text": text,
                    "source": str(reading.get("source") or "digit_yolo_incremental"),
                    "consensus_count": int(reading.get("consensus_count", 0) or 0),
                }
            ],
        }

    @staticmethod
    def apply_meter_result(result: dict[str, Any], fusion: dict[str, Any], meter_type: str) -> None:
        reading = fusion.get("reading")
        readings = [
            copy.deepcopy(item)
            for item in result.get("meter_readings", [])
            if str(item.get("meter_type") or "") != str(meter_type)
        ]
        if isinstance(reading, dict):
            clean_reading = {key: copy.deepcopy(value) for key, value in reading.items() if not str(key).startswith("_")}
            readings.insert(0, clean_reading)
        result["meter_readings"] = readings
        result[f"{meter_type}_multi_frame"] = copy.deepcopy(fusion)
        if not isinstance(reading, dict):
            return
        annotated_value = str(result.get("annotated_image") or "").strip()
        annotated_path = Path(annotated_value) if annotated_value else None
        annotated = cv2.imread(str(annotated_path), cv2.IMREAD_COLOR) if annotated_path and annotated_path.is_file() else None
        if annotated is not None:
            unit = "V" if str(meter_type) == "voltage" else "A"
            precision = 2 if str(meter_type) == "voltage" else 3
            label = f"{meter_type} fused={float(reading['value']):.{precision}f}{unit} {fusion.get('mode', '')}"
            cv2.putText(
                annotated,
                label,
                (24, 82),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 220, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.imwrite(str(annotated_path), annotated, [cv2.IMWRITE_JPEG_QUALITY, 95])

    @staticmethod
    def apply_voltage_result(result: dict[str, Any], fusion: dict[str, Any]) -> None:
        CabinetAprilTagInspection.apply_meter_result(result, fusion, "voltage")

    def best_effort_config(self) -> dict[str, Any]:
        value = self.profile.get("best_effort", {})
        return value if isinstance(value, dict) else {}

    def continuous_stage_result(self, pending: DeferredCapture) -> dict[str, Any]:
        """Build the cloud result directly from frames observed during the live scan."""
        result: dict[str, Any] = {
            "inspection_id": self.inspection_id,
            "inspection_stage": pending.stage,
            "capture_key": f"{self.inspection_id}:{pending.stage}",
            "board_id": self.board_id,
            "board_label": self.board_label,
            "scan_pose": [int(value) for value in pending.scan_pose],
            "backend": "rknn-continuous-stream",
            "inference_ms": round(float(self.latest_inference_ms), 3),
            "detections": copy.deepcopy(pending.selected.detections),
            "meter_readings": [],
            "digit_detections": [],
            "digit_readings": {},
            "cabinet_data": {},
            "source_image": "",
            "annotated_image": "",
            "result_json": "",
            "result_source": "continuous_stream",
        }
        defaults = self.best_effort_config()
        if pending.stage in METER_STAGES:
            meter_type = "voltage" if pending.stage == "voltage_meter" else "current"
            fusion = copy.deepcopy(pending.consensus)
            selected_reading = pending.selected.analysis.get("meter_reading", {})
            if (
                not isinstance(fusion.get("reading"), dict)
                and not isinstance(fusion.get("best_estimate"), dict)
                and isinstance(selected_reading, dict)
                and selected_reading.get("value") is not None
            ):
                fusion["best_estimate"] = copy.deepcopy(selected_reading)
            fallback_key = "voltage_fallback" if meter_type == "voltage" else "current_fallback"
            fallback_value = float(defaults.get(fallback_key, 220.0 if meter_type == "voltage" else 0.0))
            effective = best_effort_meter_fusion(
                fusion,
                meter_type=meter_type,
                fallback_value=fallback_value,
            )
            self.apply_meter_result(result, effective, meter_type)
        elif pending.stage == "temperature_controller":
            effective = best_effort_digit_consensus(
                pending.consensus,
                str(defaults.get("temperature_fallback", "25")),
            )
            text = str(effective["rows"][0])
            reading = copy.deepcopy(pending.selected.analysis)
            reading.update(
                {
                    "status": "ok" if effective.get("reliable") else "estimated",
                    "valid": True,
                    "reliable": bool(effective.get("reliable")),
                    "upper_text": text,
                    "rows": [text],
                    "valid_frame_count": int(
                        (effective.get("valid_observation_counts") or [0])[0]
                        if isinstance(effective.get("valid_observation_counts"), list)
                        and effective.get("valid_observation_counts")
                        else 0
                    ),
                    "consensus_count": int(
                        (effective.get("votes") or [0])[0]
                        if isinstance(effective.get("votes"), list) and effective.get("votes")
                        else 0
                    ),
                    "source": str(effective.get("source") or "best_observed_text"),
                }
            )
            self.apply_temperature_result(result, reading)
        else:
            fallback_positions = defaults.get("status_light_fallbacks", {})
            if not isinstance(fallback_positions, dict):
                fallback_positions = {}
            expected = {
                f"{pending.stage}:green": str(fallback_positions.get(f"{pending.stage}:green", "off")),
                f"{pending.stage}:red": str(fallback_positions.get(f"{pending.stage}:red", "off")),
            }
            effective = best_effort_light_consensus(pending.consensus, expected)
        result["stage_consensus"] = copy.deepcopy(effective)
        result["stage_status"] = "completed"
        result["quality_reliable"] = bool(effective.get("reliable"))
        result["quality_warning"] = "" if effective.get("reliable") else pending.missing_reason
        result["observation_elapsed_s"] = round(float(pending.observation_elapsed_s), 3)
        result["observation_extended"] = bool(pending.extended)
        result["observation_count"] = len(pending.observation_metadata)
        result["missing_reason"] = pending.missing_reason
        return result

    def process_deferred_captures(
        self,
        upload_executor: concurrent.futures.ThreadPoolExecutor | None = None,
        pending_items: Sequence[DeferredCapture] | None = None,
    ) -> None:
        upload_images = bool(self.profile.get("output", {}).get("upload_images", False))
        selected_items = list(self.deferred_captures if pending_items is None else pending_items)
        for pending in selected_items:
            index = CAPTURE_STAGES.index(pending.stage) + 1
            self.scan_state = "RECOGNIZE"
            self.scan_stage = pending.stage
            self.write_status("ALIGNED", recognition_index=index)
            if upload_images:
                result = self.run_full_recognition(
                    pending.selected.path,
                    pending.stage,
                    pending.scan_pose,
                )
                if pending.stage in METER_STAGES:
                    meter_type = "voltage" if pending.stage == "voltage_meter" else "current"
                    fallback_key = "voltage_fallback" if meter_type == "voltage" else "current_fallback"
                    fusion = best_effort_meter_fusion(
                        pending.consensus,
                        meter_type=meter_type,
                        fallback_value=float(
                            self.best_effort_config().get(
                                fallback_key,
                                220.0 if meter_type == "voltage" else 0.0,
                            )
                        ),
                    )
                    self.apply_meter_result(result, fusion, meter_type)
                elif pending.stage == "temperature_controller":
                    effective = best_effort_digit_consensus(
                        pending.consensus,
                        str(self.best_effort_config().get("temperature_fallback", "25")),
                    )
                    reading = copy.deepcopy(pending.selected.analysis)
                    reading.update(
                        {
                            "status": "ok" if effective.get("reliable") else "estimated",
                            "valid": True,
                            "reliable": bool(effective.get("reliable")),
                            "upper_text": str(effective["rows"][0]),
                            "source": str(effective.get("source") or "best_observed_text"),
                        }
                    )
                    self.apply_temperature_result(result, reading)
            else:
                result = self.continuous_stage_result(pending)
            result["candidate_quality"] = pending.selected.quality
            result["candidate_count"] = len(pending.candidates)
            result["candidate_scores"] = [candidate.quality for candidate in pending.candidates]
            result.setdefault("stage_consensus", copy.deepcopy(pending.consensus))
            result["strict_stage_consensus"] = copy.deepcopy(pending.consensus)
            result["stage_status"] = "completed"
            result["observation_elapsed_s"] = round(float(pending.observation_elapsed_s), 3)
            result["observation_extended"] = bool(pending.extended)
            result["observation_count"] = len(pending.observation_metadata)
            result["missing_reason"] = pending.missing_reason
            if result.get("result_json"):
                atomic_write_json(Path(result["result_json"]), result)
            for candidate in pending.candidates:
                try:
                    candidate.path.unlink()
                except OSError:
                    pass
            record = {
                "stage": str(result["inspection_stage"]),
                "base_stage": pending.stage,
                "paths": ({
                    "raw": str(result["source_image"]),
                    "annotated": str(result["annotated_image"]),
                    "json": str(result["result_json"]),
                } if upload_images else {}),
                "quality": pending.selected.quality,
                "stage_status": "completed",
                "missing_reason": pending.missing_reason,
                "consensus": copy.deepcopy(result.get("stage_consensus", pending.consensus)),
                "strict_consensus": copy.deepcopy(pending.consensus),
                "result": result,
                "dropped_observations": int(self.stage_drop_counts.get(pending.stage, 0)),
                "inference_elapsed_ms": round(float(self.latest_inference_elapsed_ms), 3),
                "camera_fps": round(self.camera.actual_fps(), 3),
            }
            self.photo_records.append(record)
            if pending.stage in METER_STAGES:
                expected_type = "voltage" if pending.stage == "voltage_meter" else "current"
                self.meter_results.append(
                    {
                        "meter_readings": [
                            reading
                            for reading in result.get("meter_readings", [])
                            if str(reading.get("meter_type") or "") == expected_type
                        ]
                    }
                )
            self.last_recognition_completed_at = time.monotonic()
            if upload_images and upload_executor is not None:
                coverage = self.coverage_payload()
                record["upload_coverage"] = coverage
                record["upload_enqueued_at"] = time.monotonic()
                if self.upload_pipeline_started_at <= 0.0:
                    self.upload_pipeline_started_at = record["upload_enqueued_at"]
                future = upload_executor.submit(
                    self.upload_recognition,
                    record,
                    coverage,
                    index,
                )
                self.upload_futures.append((record, future))

    def consensus(self) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
        quality_cfg = self.profile["quality"]
        return (
            meter_snapshot(self.meter_results),
            digit_consensus(
                self.digit_observations,
                int(quality_cfg.get("digit_min_winner_votes", 3)),
                int(quality_cfg.get("digit_min_valid_results", 5)),
                float(quality_cfg.get("digit_min_winner_ratio", 0.60)),
            ),
            light_consensus(
                self.light_observations,
                int(quality_cfg.get("light_min_winner_votes", 4)),
                int(quality_cfg.get("light_min_valid_results", 5)),
                float(quality_cfg.get("light_min_winner_ratio", 0.70)),
            ),
        )

    def coverage_payload(self) -> dict[str, Any]:
        meters, strict_digits, strict_lights = self.consensus()
        defaults = self.best_effort_config()
        digits = best_effort_digit_consensus(
            strict_digits,
            str(defaults.get("temperature_fallback", "25")),
        )
        fallback_positions = defaults.get("status_light_fallbacks", {})
        if not isinstance(fallback_positions, dict):
            fallback_positions = {}
        expected_positions = {
            f"{stage}:{color}": str(fallback_positions.get(f"{stage}:{color}", "off"))
            for stage in ("upper_status_lights", "lower_status_lights")
            for color in ("green", "red")
        }
        lights = best_effort_light_consensus(strict_lights, expected_positions)
        meters_complete = all(meters.get(name, {}).get("value") is not None for name in ("voltage", "current"))
        return {
            "meters": meters,
            "meters_complete": meters_complete,
            "digits": digits,
            "status_lights": lights,
            "complete": meters_complete and bool(digits.get("complete")) and bool(lights.get("complete")),
            "quality_reliable": {
                "voltage": bool(
                    next(
                        (
                            reading.get("reliable")
                            for result in self.meter_results
                            for reading in result.get("meter_readings", [])
                            if str(reading.get("meter_type") or "") == "voltage"
                        ),
                        False,
                    )
                ),
                "current": bool(
                    next(
                        (
                            reading.get("reliable")
                            for result in self.meter_results
                            for reading in result.get("meter_readings", [])
                            if str(reading.get("meter_type") or "") == "current"
                        ),
                        False,
                    )
                ),
                "temperature_controller": bool(digits.get("reliable")),
                "status_lights": bool(lights.get("reliable")),
            },
        }

    def waypoint_target(self, waypoint: dict[str, Any], extension_scale: float | None = None) -> list[int]:
        scale = self.extension_scale if extension_scale is None else float(extension_scale)
        return resolve_waypoint_pose(waypoint, self.aligned_s0, scale)

    def move_waypoint(self, waypoint: dict[str, Any]) -> None:
        default_duration_ms = int(self.profile["scan"].get("row_duration_ms", 1400))
        motion = resolve_waypoint_motion(
            waypoint,
            self.aligned_s0,
            self.extension_scale,
            default_duration_ms,
        )
        self.current_transition_step_count = len(motion)
        for index, (target, configured_duration_ms) in enumerate(motion, start=1):
            self.current_transition_step = index
            self.write_status("ALIGNED")
            changed = changed_servo_targets(self.current_scan_pose, target)
            duration_ms = configured_duration_ms
            if changed and all(servo_index == 0 for servo_index, _value in changed):
                duration_ms = s0_motion_duration_ms(
                    self.current_scan_pose[0],
                    target[0],
                    self.profile["scan"],
                )
            self.move_scan_pose(target, duration_ms)
        self.current_transition_step = 0
        self.current_transition_step_count = 0

    def safe_return_aligned_home(self) -> None:
        target = [self.aligned_s0, *self.home_pwm[1:]]
        self.scan_state = "RETURN_HOME"
        self.scan_stage = ""
        self.current_waypoint_index = len(self.waypoints) + 1
        self.current_waypoint_name = "home"
        self.move_scan_pose(target, int(self.profile["scan"].get("home_duration_ms", 1400)), detect_during_motion=False)
        self.current_scan_pose = target
        self.final_home_restored = True
        self.write_status("ALIGNED")

    def queue_outbox(self, suffix: str, payload: dict[str, Any]) -> None:
        self.outbox_dir.mkdir(parents=True, exist_ok=True)
        atomic_write_json(self.outbox_dir / f"{self.inspection_id}_{suffix}.json", payload)

    def upload_session(self):
        if requests is None:
            raise RuntimeError("requests is unavailable")
        session = getattr(self._upload_thread_state, "session", None)
        if session is None:
            session = requests.Session()
            adapter = requests.adapters.HTTPAdapter(pool_connections=1, pool_maxsize=1, max_retries=0)
            session.mount("http://", adapter)
            session.mount("https://", adapter)
            self._upload_thread_state.session = session
        return session

    def upload_recognition(
        self,
        record: dict[str, Any],
        coverage: dict[str, Any],
        stage_index: int,
    ) -> dict[str, Any]:
        paths = record["paths"]
        source = json.loads(Path(paths["json"]).read_text(encoding="utf-8"))
        stage = str(record["stage"])
        payload = build_stage_upload_payload(
            source,
            inspection_id=self.inspection_id,
            stage=stage,
            stage_index=stage_index,
            profile=str(self.profile.get("profile_name") or self.profile_path.stem),
            started_at=self.inspection_started_at,
            board_id=self.board_id,
            board_label=self.board_label,
            coverage=coverage,
        )
        payload.update(getattr(self, "navigation_metadata", {}))
        raw_path = Path(paths["raw"])
        annotated_path = Path(paths["annotated"])
        raw_size = raw_path.stat().st_size
        annotated_size = annotated_path.stat().st_size
        started = time.monotonic()
        with Path(paths["raw"]).open("rb") as raw_handle, Path(paths["annotated"]).open("rb") as annotated_handle:
            response = self.upload_session().post(
                f"{self.server_url}/api/upload/recognition",
                headers={"X-Upload-Token": self.upload_token},
                data={"payload": json.dumps(payload, ensure_ascii=False)},
                files={
                    "raw_image": (Path(paths["raw"]).name, raw_handle, "image/jpeg"),
                    "annotated_image": (Path(paths["annotated"]).name, annotated_handle, "image/jpeg"),
                },
                timeout=45,
            )
        response.raise_for_status()
        body = response.json()
        uploaded = body.get("recognition", body) if isinstance(body, dict) else {}
        if not isinstance(uploaded, dict):
            uploaded = {}
        cloud_raw_size = int(uploaded.get("raw_image", {}).get("size_bytes") or 0)
        cloud_annotated_size = int(uploaded.get("annotated_image", {}).get("size_bytes") or 0)
        if cloud_raw_size != raw_size or cloud_annotated_size != annotated_size:
            raise RuntimeError(
                f"uploaded image size mismatch raw={cloud_raw_size}/{raw_size} "
                f"annotated={cloud_annotated_size}/{annotated_size}"
            )
        timings = uploaded.get("timings_ms", {}) if isinstance(uploaded.get("timings_ms"), dict) else {}
        return {
            "recognition": compact_recognition_reference(uploaded),
            "upload_metrics": {
                "stage": stage,
                "stage_index": int(stage_index),
                "queue_ms": round(max(0.0, started - float(record.get("upload_enqueued_at") or started)) * 1000.0, 3),
                "wall_ms": round((time.monotonic() - started) * 1000.0, 3),
                "raw_bytes": raw_size,
                "annotated_bytes": annotated_size,
                "total_bytes": raw_size + annotated_size,
                "server_save_ms": timings.get("server_save"),
            },
        }

    def upload_structured_results(self, summary: dict[str, Any]) -> None:
        coverage = summary["coverage"]
        completed_at = str(summary.get("completed_at") or utc_now_iso())
        stages: list[dict[str, Any]] = []
        for record in self.photo_records:
            stage = str(record["stage"])
            index = CAPTURE_STAGES.index(stage) + 1
            result = record["result"]
            cabinet_update = stage_cabinet_update(stage, result, coverage)
            recognition = {
                "id": f"{self.inspection_id}:{stage}:stream",
                "uploaded_at": completed_at,
                "inspection_id": self.inspection_id,
                "inspection_stage": stage,
                "capture_key": f"{self.inspection_id}:{stage}",
                "backend": str(result.get("backend") or "rknn-continuous-stream"),
                "inference_ms": result.get("inference_ms", ""),
                "detection_count": len(result.get("detections", [])),
                "meter_readings": copy.deepcopy(result.get("meter_readings", [])),
                "digit_readings": copy.deepcopy(result.get("digit_readings", {})),
                "cabinet_update": copy.deepcopy(cabinet_update),
                "raw_image": {},
                "annotated_image": {},
                "source": {
                    "result_source": "continuous_stream",
                    "stage_consensus": copy.deepcopy(result.get("stage_consensus", {})),
                    "strict_stage_consensus": copy.deepcopy(result.get("strict_stage_consensus", {})),
                    "quality_reliable": bool(result.get("quality_reliable")),
                    "quality_warning": str(result.get("quality_warning") or ""),
                    "observation_elapsed_s": result.get("observation_elapsed_s"),
                    "observation_count": result.get("observation_count"),
                },
            }
            stages.append(
                {
                    "stage": stage,
                    "label": STAGE_LABELS.get(stage, stage),
                    "stage_index": index,
                    "status": "completed",
                    "message": "continuous stream result",
                    "quality_reliable": bool(result.get("quality_reliable")),
                    "quality_warning": str(result.get("quality_warning") or ""),
                    "cabinet_update": cabinet_update,
                    "recognition": recognition,
                }
            )
        summary["stages"] = stages
        summary["upload_status"] = "completed"
        summary["pending_upload_count"] = 0
        timings = summary.setdefault("timings_ms", {})
        timings["stage_uploads"] = []
        timings["image_bytes"] = 0
        upload_payload = prioritize_structured_inspection_payload(summary)
        started = time.monotonic()
        try:
            response = self.upload_session().post(
                f"{self.server_url}/api/upload/inspection",
                headers={"X-Upload-Token": self.upload_token},
                json=upload_payload,
                timeout=30,
            )
            response.raise_for_status()
            summary["cloud_upload"] = response.json()
        except Exception as exc:
            summary["upload_error"] = str(exc)
            self.queue_outbox("inspection", {"kind": "inspection", "payload": upload_payload})
        timings["summary_upload"] = round((time.monotonic() - started) * 1000.0, 3)

    def upload_results(self, summary: dict[str, Any]) -> None:
        if not self.args.upload:
            return
        if not bool(self.profile.get("output", {}).get("upload_images", False)):
            self.upload_structured_results(summary)
            return
        stage_uploads: list[dict[str, Any]] = []
        coverage = summary["coverage"]
        upload_timings: list[dict[str, Any]] = []
        pending_count = 0
        future_by_stage = {str(record["stage"]): future for record, future in self.upload_futures}
        for record in self.photo_records:
            stage = str(record["stage"])
            index = CAPTURE_STAGES.index(stage) + 1
            try:
                future = future_by_stage.get(stage)
                uploaded = future.result() if future is not None else self.upload_recognition(record, coverage, index)
                upload_timings.append(copy.deepcopy(uploaded["upload_metrics"]))
                stage_uploads.append(
                    {
                        "stage": stage,
                        "label": STAGE_LABELS.get(stage, stage),
                        "stage_index": index,
                        "status": str(record.get("stage_status") or "completed"),
                        "missing_reason": str(record.get("missing_reason") or ""),
                        "dropped_observations": int(record.get("dropped_observations", 0) or 0),
                        "inference_elapsed_ms": record.get("inference_elapsed_ms"),
                        "camera_fps": record.get("camera_fps"),
                        "recognition_id": uploaded["recognition"].get("id", ""),
                        "cabinet_update": stage_cabinet_update(stage, record["result"], coverage),
                    }
                )
            except Exception as exc:
                pending_count += 1
                stage_uploads.append(
                    {
                        "stage": stage,
                        "label": STAGE_LABELS.get(stage, stage),
                        "stage_index": index,
                        "status": "pending_upload",
                        "message": str(exc),
                        "cabinet_update": stage_cabinet_update(stage, record["result"], coverage),
                    }
                )
                retry_payload = build_stage_upload_payload(
                    record["result"],
                    inspection_id=self.inspection_id,
                    stage=stage,
                    stage_index=index,
                    profile=str(self.profile.get("profile_name") or self.profile_path.stem),
                    started_at=self.inspection_started_at,
                    board_id=self.board_id,
                    board_label=self.board_label,
                    coverage=coverage,
                )
                retry_payload.update(getattr(self, "navigation_metadata", {}))
                override_keys = {
                    "inspection_id",
                    "inspection_stage",
                    "inspection_stage_index",
                    "inspection_stage_count",
                    "inspection_stage_label",
                    "inspection_status",
                    "inspection_profile",
                    "inspection_started_at",
                    "capture_key",
                    "coverage",
                    "cabinet_data",
                    "meter_readings",
                    "digit_readings",
                    "digit_detections",
                    "board_id",
                    "board_label",
                }
                self.queue_outbox(
                    f"recognition_{stage}",
                    {
                        "kind": "recognition",
                        "inspection_id": self.inspection_id,
                        "stage": stage,
                        "paths": record["paths"],
                        "payload_overrides": {
                            key: value for key, value in retry_payload.items() if key in override_keys
                        },
                        "board_id": self.board_id,
                        "board_label": self.board_label,
                    },
                )
        uploaded_stages = {str(item.get("stage") or "") for item in stage_uploads}
        for stage in CAPTURE_STAGES:
            if stage in uploaded_stages:
                continue
            failure = self.stage_failures.get(stage, {})
            stage_uploads.append(
                {
                    "stage": stage,
                    "label": STAGE_LABELS.get(stage, stage),
                    "stage_index": CAPTURE_STAGES.index(stage) + 1,
                    "status": "failed",
                    "message": str(failure.get("reason") or "stage unavailable"),
                    "dropped_observations": int(failure.get("dropped_observations", 0) or 0),
                    "inference_elapsed_ms": failure.get("inference_elapsed_ms"),
                    "camera_fps": failure.get("camera_fps"),
                    "cabinet_update": {},
                }
            )
        stage_uploads.sort(key=lambda item: int(item.get("stage_index", 0)))
        summary["stages"] = stage_uploads
        summary["uploaded_stage_count"] = sum(
            1 for item in stage_uploads if str(item.get("status") or "") == "completed"
        )
        if any(str(item.get("status") or "") == "failed" for item in stage_uploads):
            summary["status"] = "partial"
        if pending_count:
            summary["upload_status"] = "pending_upload"
            summary["pending_upload_count"] = pending_count
            if summary.get("status") == "completed":
                summary["status"] = "partial"
        else:
            summary["upload_status"] = "completed"
            summary["pending_upload_count"] = 0
        timings = summary.setdefault("timings_ms", {})
        timings["stage_uploads"] = upload_timings
        timings["image_bytes"] = sum(int(item.get("total_bytes") or 0) for item in upload_timings)
        if self.upload_pipeline_started_at > 0.0:
            timings["pipeline_before_summary"] = round(
                (time.monotonic() - self.upload_pipeline_started_at) * 1000.0,
                3,
            )
        summary_upload_started = time.monotonic()
        try:
            response = self.upload_session().post(
                f"{self.server_url}/api/upload/inspection",
                headers={"X-Upload-Token": self.upload_token},
                json=summary,
                timeout=30,
            )
            response.raise_for_status()
            summary["cloud_upload"] = response.json()
            timings["summary_upload"] = round((time.monotonic() - summary_upload_started) * 1000.0, 3)
        except Exception as exc:
            summary["upload_error"] = str(exc)
            timings["summary_upload"] = round((time.monotonic() - summary_upload_started) * 1000.0, 3)
            self.queue_outbox("inspection", {"kind": "inspection", "payload": summary})
        if self.upload_pipeline_started_at > 0.0:
            timings["pipeline_total"] = round(
                (time.monotonic() - self.upload_pipeline_started_at) * 1000.0,
                3,
            )
        if self.last_recognition_completed_at > 0.0:
            timings["upload_tail_after_last_recognition"] = round(
                (time.monotonic() - self.last_recognition_completed_at) * 1000.0,
                3,
            )

    def run_fixed_scan(self) -> dict[str, Any]:
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        self.scan_state = "YOLO_READY"
        self.ensure_worker_ready()
        self.start_live_stream()
        self.extension_scale, self.extension_reason = inspection_extension_scale(
            self.aligned_tag_distance_mm,
            self.aligned_tag_distance_spread_mm,
            self.profile["scan"]["extension"],
            tag_search_skipped=self.tag_search_skipped,
        )
        adjustments: dict[str, Any] = {}
        self.write_status("ALIGNED")
        for index, waypoint in enumerate(self.waypoints, start=1):
            name = str(waypoint["name"])
            self.current_waypoint_index = index
            self.current_waypoint_name = name
            self.scan_state = name.upper()
            self.scan_stage = name if bool(waypoint.get("capture")) else ""
            self.write_status("ALIGNED")
            self.move_waypoint(waypoint)
            if not bool(waypoint.get("capture")):
                continue
            self.settle_pose()
            adjustments[name] = self.center_stage(name)
            self.queue_stage_capture_with_recovery(name)

        self.safe_return_aligned_home()
        self.stop_live_stream()
        self.camera.close()
        self.scan_state = "RECOGNIZE"
        upload_images = bool(self.profile.get("output", {}).get("upload_images", False))
        if self.args.upload and upload_images:
            self.upload_executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=UPLOAD_WORKERS,
                thread_name_prefix="cabinet-upload",
            )
        for pending in self.deferred_captures:
            recognition_error = ""
            for recognition_attempt in range(1, 3):
                try:
                    self.process_deferred_captures(
                        self.upload_executor,
                        pending_items=[pending],
                    )
                    recognition_error = ""
                    break
                except Exception as exc:
                    if not self.recoverable_visual_error(exc):
                        raise
                    recognition_error = str(exc)
                    self.record_observation_drop(exc, pending.stage)
                    if recognition_attempt < 2:
                        with contextlib.suppress(Exception):
                            self.ensure_worker_ready()
            if recognition_error:
                self.record_stage_failure(
                    pending.stage,
                    recognition_error,
                    attempts=2,
                    retained_path=str(pending.selected.path),
                )

        self.scan_state = "VERIFY"
        self.scan_stage = ""
        coverage = self.coverage_payload()
        completed_stages = {str(record["stage"]) for record in self.photo_records}
        missing_stages = [stage for stage in CAPTURE_STAGES if stage not in completed_stages]
        coverage["completed_stages"] = [stage for stage in CAPTURE_STAGES if stage in completed_stages]
        coverage["missing_stages"] = missing_stages
        coverage["complete"] = bool(coverage.get("complete")) and not missing_stages
        status = scan_status_for_completed_stages(
            completed_stages,
            coverage_complete=bool(coverage["complete"]),
        )
        cabinet_update: dict[str, Any] = {}
        for record in self.photo_records:
            merge_cabinet_update(
                cabinet_update,
                stage_cabinet_update(str(record["stage"]), record["result"], coverage),
            )
        cabinet_update["updated_at"] = utc_now_iso()
        record_by_stage = {str(record["stage"]): record for record in self.photo_records}
        stage_diagnostics: list[dict[str, Any]] = []
        for stage_index, stage in enumerate(CAPTURE_STAGES, start=1):
            record = record_by_stage.get(stage)
            if record is None:
                failure = self.stage_failures.get(stage, {})
                stage_diagnostics.append(
                    {
                        "stage": stage,
                        "label": STAGE_LABELS.get(stage, stage),
                        "stage_index": stage_index,
                        "status": "failed",
                        "message": str(failure.get("reason") or "stage unavailable"),
                        "dropped_observations": int(failure.get("dropped_observations", 0) or 0),
                        "inference_elapsed_ms": failure.get("inference_elapsed_ms"),
                        "camera_fps": failure.get("camera_fps"),
                    }
                )
                continue
            stage_diagnostics.append(
                {
                    "stage": stage,
                    "label": STAGE_LABELS.get(stage, stage),
                    "stage_index": stage_index,
                    "status": str(record.get("stage_status") or "completed"),
                    "message": str(record.get("missing_reason") or ""),
                    "dropped_observations": int(record.get("dropped_observations", 0) or 0),
                    "inference_elapsed_ms": record.get("inference_elapsed_ms"),
                    "camera_fps": record.get("camera_fps"),
                }
            )
        summary = {
            "inspection_id": self.inspection_id,
            "status": status,
            "profile": str(self.profile.get("profile_name") or self.profile_path.stem),
            "board_id": self.board_id,
            "board_label": self.board_label,
            "started_at": self.inspection_started_at,
            "completed_at": utc_now_iso(),
            "aligned_s0_pwm": self.aligned_s0,
            "tag_distance_mm": self.aligned_tag_distance_mm,
            "tag_distance_spread_mm": self.aligned_tag_distance_spread_mm,
            "extension_scale": round(self.extension_scale, 4),
            "extension_reason": self.extension_reason,
            "final_home_restored": self.final_home_restored,
            "coverage": coverage,
            "adjustments": adjustments,
            "photo_count": len(self.photo_records) if upload_images else 0,
            "photos": [copy.deepcopy(record.get("paths", {})) for record in self.photo_records]
            if upload_images
            else [],
            "result_count": len(self.photo_records),
            "result_source": "continuous_stream",
            "structured_only": not upload_images,
            "stage_consensus": copy.deepcopy(self.stage_consensus_results),
            "missing_fields": [f"stage:{stage}" for stage in missing_stages],
            "quality_warnings": [
                record["stage"]
                for record in self.photo_records
                if not bool(record["result"].get("quality_reliable"))
            ],
            "capture_resolution": self.capture_resolution,
            "stream_resolution": self.stream_resolution,
            "stream_fps_target": self.stream_fps_target,
            "expected_stage_count": len(CAPTURE_STAGES),
            "uploaded_stage_count": 0,
            "stages": stage_diagnostics,
            "stage_failures": copy.deepcopy(self.stage_failures),
            "stage_drop_counts": dict(self.stage_drop_counts),
            "transient_observation_drop_count": int(self.transient_observation_drop_count),
            "camera_reset_count": int(self.camera_reset_count),
            "capture_fps_actual": round(self.camera.actual_fps(), 3),
            "cabinet_update": cabinet_update,
            **getattr(self, "navigation_metadata", {}),
            "detected_apriltag_id": None if self.latest_tag is None else int(self.latest_tag["id"]),
        }
        self.scan_state = "UPLOAD" if self.args.upload else "VERIFY"
        self.upload_results(summary)
        status = str(summary.get("status") or status)
        self.scan_state = "COMPLETE" if status == "completed" else ("PARTIAL" if status == "partial" else "FAILED")
        self.result = summary
        self.write_status("ALIGNED", result=summary)
        return summary

    def run_pose_test(self, stage: str) -> None:
        self.scan_stage = stage
        self.scan_state = "POSE_TEST"
        self.current_waypoint_name = stage
        self.current_waypoint_index = list(WAYPOINT_SEQUENCE).index(stage) + 1
        self.extension_scale = 1.0
        self.extension_reason = "pose_test_full_extension"
        self.move_waypoint(self.waypoint_by_name[stage])
        self.settle_pose()
        self.result = {
            "inspection_id": self.inspection_id,
            "status": "completed",
            "pose_test": stage,
            "scan_pose": list(self.current_scan_pose),
        }
        self.return_pose_after_preview = bool(self.show)
        if not self.show:
            self.wait_motion(float(self.profile["scan"].get("pose_test_hold_s", 2.0)), run_detection=True)
            self.safe_return_aligned_home()

    def after_alignment(self) -> int:
        self.aligned_s0 = int(self.final_s0_pwm or self.current_s0_pwm() or self.home_pwm[0])
        self.current_scan_pose = [self.aligned_s0, *self.home_pwm[1:]]
        try:
            if self.args.pose_test:
                self.run_pose_test(POSE_ALIASES[str(self.args.pose_test)])
            else:
                summary = self.run_fixed_scan()
                if summary.get("status") == "failed":
                    raise SearchFailure("cabinet targets were not recognized")
            return 0
        except Exception as exc:
            self.reason = str(exc)
            self.scan_state = "FAILED"
            self.emergency_stop()
            if self.node is not None and self.node.state_age() <= float(self.config["safety"]["f103_state_timeout_s"]):
                if not int(self.node.last_state.get("estop", 0)):
                    try:
                        self.safe_return_aligned_home()
                    except Exception:
                        self.emergency_stop()
            self.result = {
                "inspection_id": self.inspection_id,
                "status": "failed",
                "message": str(exc),
                **getattr(self, "navigation_metadata", {}),
            }
            self.write_status("FAILED", result=self.result)
            raise SearchFailure(str(exc)) from exc
        finally:
            self.stop_live_stream()
            self.cleanup_worker_frames()
            if self.upload_executor is not None:
                self.upload_executor.shutdown(wait=True)
                self.upload_executor = None

    def wait_for_manual_preview_close(self) -> None:
        if not self.return_pose_after_preview:
            return super().wait_for_manual_preview_close()
        self.write_status("ALIGNED_WAIT_CLOSE", preview_waiting_for_manual_close=True, pose_test=self.scan_stage)
        try:
            while True:
                if self.stop_requested:
                    break
                try:
                    self.spin_once(0.05)
                except SearchFailure as exc:
                    if str(exc) not in MANUAL_PREVIEW_CLOSE_REASONS:
                        raise
                    self.preview_close_reason = str(exc)
                    self.stop_requested = False
                    break
                self.assert_safe_state(require_home_axes=False)
        finally:
            self.stop_requested = False
            self.safe_return_aligned_home()
            self.return_pose_after_preview = False


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fixed cabinet scan and recognition")
    parser.add_argument("--search-config", default=str(ROOT_DIR / "config" / "arm_apriltag_s0_search.yaml"))
    parser.add_argument("--profile", default=str(ROOT_DIR / "cabinet_inspection_profiles" / "fixed_scan.yaml"))
    parser.add_argument("--execute", action="store_true", help="allow arm motion")
    parser.add_argument("--show", action="store_true", help="show preview and wait for Q/Esc after completion")
    parser.add_argument("--upload", action="store_true", help="upload retained captures and summary")
    parser.add_argument(
        "--skip-tag-search",
        action="store_true",
        help="start the fixed scan from the arm home pose without moving S0 to find an AprilTag",
    )
    parser.add_argument(
        "--pose-test",
        choices=tuple(POSE_ALIASES),
        default="",
        help="move only to one configured scan waypoint, then return",
    )
    parser.add_argument("--server-url", default="")
    parser.add_argument("--upload-token", default="")
    parser.add_argument("--board-id", default="")
    parser.add_argument("--board-label", default="")
    parser.add_argument("--inspection-id", default="")
    parser.add_argument("--expected-tag-id", type=int, default=None)
    parser.add_argument("--navigation-session-id", default="")
    parser.add_argument("--waypoint-index", type=int, default=-1)
    parser.add_argument("--waypoint-label", default="")
    parser.add_argument("--status-file", default="")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    runner: CabinetAprilTagInspection | None = None
    try:
        runner = CabinetAprilTagInspection(args)
        signal.signal(signal.SIGINT, runner.request_stop)
        signal.signal(signal.SIGTERM, runner.request_stop)
        return_code = runner.run()
        result = runner.result or {
            "inspection_id": runner.inspection_id,
            "status": "completed" if return_code == 0 else "failed",
            "message": runner.reason,
        }
        print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
        return return_code
    except Exception as exc:
        payload = {
            "inspection_id": "" if runner is None else runner.inspection_id,
            "status": "failed",
            "message": str(exc),
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2), flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

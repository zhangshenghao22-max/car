#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
import threading
import time
from pathlib import Path
from typing import Any, Sequence

import cv2
import numpy as np
import yaml


WINDOW_NAME = "Cabinet Vision"
ACTIVE_NAV_ACTION_STATUSES = {1, 2, 3}
ACTIVE_NAV_MANAGER_PHASES = {
    "PLANNING",
    "REPLANNING",
    "TURN_IN_PLACE",
    "DRIVE_TO_CORNER",
    "DRIVE_TO_GOAL",
}
TERMINAL_NAV_MANAGER_PHASES = {
    "IDLE",
    "DONE",
    "FAILED",
    "SAFETY_STOP",
    "CANCELED",
    "CANCELLED",
}


def load_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def atomic_write_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(path))


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_write_bytes(
        path,
        json.dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("utf-8"),
    )


def letterbox(image: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[int, int]]:
    height, width = image.shape[:2]
    scale = min(float(size) / float(width), float(size) / float(height))
    resized_width = int(round(width * scale))
    resized_height = int(round(height * scale))
    resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    left = int(round((size - resized_width) / 2.0 - 0.1))
    top = int(round((size - resized_height) / 2.0 - 0.1))
    canvas[top : top + resized_height, left : left + resized_width] = resized
    return canvas, scale, (left, top)


def decode_yolov8(
    output: np.ndarray,
    image_shape: tuple[int, int],
    classes: Sequence[str],
    scale: float,
    pad: tuple[int, int],
    confidence_threshold: float,
    iou_threshold: float,
) -> list[dict[str, Any]]:
    rows = np.asarray(output)
    while rows.ndim > 2 and rows.shape[0] == 1:
        rows = rows[0]
    if rows.ndim == 1:
        rows = rows.reshape(1, -1)
    if rows.ndim != 2:
        raise ValueError(f"unexpected YOLO output shape: {np.asarray(output).shape}")
    features = 4 + len(classes)
    if rows.shape[0] == features and rows.shape[1] != features:
        rows = rows.T
    elif rows.shape[1] != features:
        raise ValueError(f"YOLO output does not contain {features} features: {rows.shape}")

    image_height, image_width = image_shape
    boxes: list[list[int]] = []
    scores: list[float] = []
    class_ids: list[int] = []
    for row in rows:
        class_scores = row[4 : 4 + len(classes)]
        class_id = int(np.argmax(class_scores))
        confidence = float(class_scores[class_id])
        if confidence < confidence_threshold:
            continue
        center_x, center_y, width, height = [float(value) for value in row[:4]]
        x1 = int(round((center_x - width / 2.0 - pad[0]) / scale))
        y1 = int(round((center_y - height / 2.0 - pad[1]) / scale))
        x2 = int(round((center_x + width / 2.0 - pad[0]) / scale))
        y2 = int(round((center_y + height / 2.0 - pad[1]) / scale))
        x1 = max(0, min(image_width - 1, x1))
        y1 = max(0, min(image_height - 1, y1))
        x2 = max(0, min(image_width - 1, x2))
        y2 = max(0, min(image_height - 1, y2))
        if x2 <= x1 or y2 <= y1:
            continue
        boxes.append([x1, y1, x2 - x1, y2 - y1])
        scores.append(confidence)
        class_ids.append(class_id)

    selected = cv2.dnn.NMSBoxes(boxes, scores, confidence_threshold, iou_threshold)
    indices = np.asarray(selected).reshape(-1).tolist() if len(selected) else []
    detections: list[dict[str, Any]] = []
    for index in indices:
        x, y, width, height = boxes[index]
        detections.append(
            {
                "class_id": class_ids[index],
                "class_name": classes[class_ids[index]],
                "confidence": float(scores[index]),
                "box_xyxy": [x, y, x + width, y + height],
                "center_xy": [x + width / 2.0, y + height / 2.0],
            }
        )
    return sorted(detections, key=lambda item: item["confidence"], reverse=True)


class LatestFrameRknnWorker(threading.Thread):
    def __init__(self, config: dict[str, Any]):
        super().__init__(name="switch-rknn", daemon=True)
        self.config = config
        self._condition = threading.Condition()
        self._frame: np.ndarray | None = None
        self._stop_requested = False
        self._lock = threading.Lock()
        self._detections: list[dict[str, Any]] = []
        self._result_at = 0.0
        self._inference_ms = 0.0
        self._error = "starting"
        self._backend = "uninitialized"

    def submit(self, frame: np.ndarray) -> None:
        with self._condition:
            self._frame = frame.copy()
            self._condition.notify()

    def stop(self) -> None:
        with self._condition:
            self._stop_requested = True
            self._condition.notify()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "detections": [dict(item) for item in self._detections],
                "result_at": self._result_at,
                "inference_ms": self._inference_ms,
                "error": self._error,
                "backend": self._backend,
            }

    def run(self) -> None:
        runtime = None
        try:
            from rknnlite.api import RKNNLite

            model_path = Path(self.config["model"])
            if not model_path.is_file():
                raise FileNotFoundError(model_path)
            runtime = RKNNLite()
            code = runtime.load_rknn(str(model_path))
            if code != 0:
                raise RuntimeError(f"load_rknn returned {code}")
            code = runtime.init_runtime(core_mask=RKNNLite.NPU_CORE_AUTO)
            if code != 0:
                raise RuntimeError(f"init_runtime returned {code}")
            with self._lock:
                self._backend = "rknn"
                self._error = ""
            while True:
                with self._condition:
                    while self._frame is None and not self._stop_requested:
                        self._condition.wait(0.5)
                    if self._stop_requested:
                        return
                    frame = self._frame
                    self._frame = None
                if frame is None:
                    continue
                prepared, scale, pad = letterbox(frame, int(self.config.get("imgsz", 640)))
                tensor = np.expand_dims(cv2.cvtColor(prepared, cv2.COLOR_BGR2RGB), axis=0)
                started = time.monotonic()
                outputs = runtime.inference(inputs=[np.ascontiguousarray(tensor)])
                inference_ms = (time.monotonic() - started) * 1000.0
                if not outputs or len(outputs) != 1:
                    raise RuntimeError(f"RKNN output contract mismatch: {0 if not outputs else len(outputs)}")
                detections = decode_yolov8(
                    outputs[0],
                    frame.shape[:2],
                    [str(value) for value in self.config.get("classes", ["kai", "guan"])],
                    scale,
                    pad,
                    float(self.config.get("confidence", 0.20)),
                    float(self.config.get("iou", 0.45)),
                )
                now = time.time()
                for detection in detections:
                    detection["detected_at"] = now
                with self._lock:
                    self._detections = detections
                    self._result_at = now
                    self._inference_ms = inference_ms
                    self._error = ""
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
        finally:
            if runtime is not None:
                try:
                    runtime.release()
                except Exception:
                    pass


def intrinsic_arrays(intrinsics: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[int, int]]:
    camera_matrix = np.asarray(intrinsics["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.asarray(intrinsics["dist_coeffs"], dtype=np.float64)
    new_matrix = np.asarray(intrinsics.get("new_camera_matrix", camera_matrix), dtype=np.float64)
    image_size = tuple(int(value) for value in intrinsics["image_size"])
    return camera_matrix, dist_coeffs, new_matrix, image_size


def raw_points_to_rectified(points_xy: np.ndarray, intrinsics: dict[str, Any]) -> np.ndarray:
    camera_matrix, dist_coeffs, new_matrix, _ = intrinsic_arrays(intrinsics)
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 1, 2)
    if str(intrinsics["model"]) == "fisheye":
        output = cv2.fisheye.undistortPoints(
            points, camera_matrix, dist_coeffs, R=np.eye(3), P=new_matrix
        )
    else:
        output = cv2.undistortPoints(
            points, camera_matrix, dist_coeffs, R=np.eye(3), P=new_matrix
        )
    return output.reshape(-1, 2)


def rotation_x(angle: float) -> np.ndarray:
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.asarray([[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]])


def rotation_z(angle: float) -> np.ndarray:
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.asarray([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def camera_mount_transform(joint_deg: Sequence[float], arm: dict[str, Any]) -> np.ndarray:
    values = list(joint_deg) + [0.0] * max(0, 4 - len(joint_deg))
    base = math.radians(values[0])
    shoulder = math.radians(values[1] + 90.0)
    forearm = shoulder - math.radians(values[2])
    tool_pitch = forearm + math.radians(values[3])
    l0 = float(arm.get("l0_mm", 100.0))
    l1 = float(arm.get("l1_mm", 105.0))
    l2 = float(arm.get("l2_mm", 88.0))
    tool = float(arm.get("tool_length_mm", 155.0))
    radial = l1 * math.cos(shoulder) + l2 * math.cos(forearm) + tool * math.cos(tool_pitch)
    z = l0 + l1 * math.sin(shoulder) + l2 * math.sin(forearm) + tool * math.sin(tool_pitch)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation_z(-base) @ rotation_x(tool_pitch)
    transform[:3, 3] = [radial * math.sin(base), radial * math.cos(base), z]
    return transform


def pwm_to_joint_deg(pwm: Sequence[int], arm: dict[str, Any]) -> list[float]:
    home = list(arm.get("home_pwm", [1600, 1500, 1500, 1500, 1500, 1500]))
    direction = list(arm.get("direction", [-1, 1, 1, 1, 1, 1]))
    scale = list(arm.get("pwm_per_deg", [2000.0 / 270.0] * 6))
    values = list(pwm) + home[len(pwm) :]
    return [
        (float(values[index]) - float(home[index]))
        / (float(direction[index] or 1) * float(scale[index] or (2000.0 / 270.0)))
        for index in range(5)
    ]


def detection_center(detection: dict[str, Any]) -> np.ndarray:
    return np.asarray(detection["rectified_center"], dtype=np.float64).reshape(2)


def highest_confidence_detection(
    detections: Sequence[dict[str, Any]],
) -> dict[str, Any] | None:
    if not detections:
        return None
    return max(detections, key=lambda item: float(item.get("confidence", 0.0)))


def matching_detection(
    detections: Sequence[dict[str, Any]],
    target: dict[str, Any],
    max_distance_px: float,
) -> dict[str, Any] | None:
    target_class = str(target.get("class_name", ""))
    target_center = detection_center(target)
    candidates = [
        item for item in detections if str(item.get("class_name", "")) == target_class
    ]
    if not candidates:
        return None
    selected = min(
        candidates,
        key=lambda item: float(np.linalg.norm(detection_center(item) - target_center)),
    )
    if float(np.linalg.norm(detection_center(selected) - target_center)) > max_distance_px:
        return None
    return selected


class AlignmentController:
    ACTIVE_STATES = {"WAIT_TARGET", "ACQUIRE", "EVALUATE", "PULSE", "SETTLE"}

    def __init__(self, config: dict[str, Any], image_width: int):
        self.config = config
        self.image_center_x = float(image_width) * 0.5
        self.state = "DISABLED"
        self.reason = ""
        self.candidate: dict[str, Any] | None = None
        self.candidate_frames = 0
        self.target: dict[str, Any] | None = None
        self.last_processed_result_at = 0.0
        self.last_seen_at = 0.0
        self.enabled_at = 0.0
        self.acquire_started_at = 0.0
        self.pulse_started_at = 0.0
        self.pulse_motion_started_at = 0.0
        self.pulse_feedback_frames = 0
        self.pulse_feedback_seq = -1
        self.settle_started_at = 0.0
        self.settle_result_at = 0.0
        self.pulse_initial_error = 0.0
        self.divergence_count = 0
        self.centered_frames = 0
        self.attempts = 0
        self.error_px: float | None = None
        self.command_vy = 0.0
        self.lease_desired = False

    def _clear_tracking(self) -> None:
        self.candidate = None
        self.candidate_frames = 0
        self.target = None
        self.last_processed_result_at = 0.0
        self.last_seen_at = 0.0
        self.acquire_started_at = 0.0
        self.pulse_started_at = 0.0
        self.pulse_motion_started_at = 0.0
        self.pulse_feedback_frames = 0
        self.pulse_feedback_seq = -1
        self.settle_started_at = 0.0
        self.settle_result_at = 0.0
        self.pulse_initial_error = 0.0
        self.divergence_count = 0
        self.centered_frames = 0
        self.attempts = 0
        self.error_px = None
        self.command_vy = 0.0
        self.lease_desired = False

    def begin(self, now: float) -> None:
        self._clear_tracking()
        self.state = "WAIT_TARGET"
        self.reason = ""
        self.enabled_at = float(now)

    def cancel(self, reason: str = "CANCELLED") -> None:
        self.command_vy = 0.0
        self.lease_desired = False
        self.state = "DISABLED"
        self.reason = reason

    def fault(self, reason: str) -> None:
        self.command_vy = 0.0
        self.lease_desired = False
        self.state = "FAULT"
        self.reason = str(reason)

    def toggle(self, now: float) -> None:
        if self.state in self.ACTIVE_STATES:
            self.cancel()
        else:
            self.begin(now)

    @property
    def active(self) -> bool:
        return self.state in self.ACTIVE_STATES

    def _update_target(
        self,
        detections: Sequence[dict[str, Any]],
        now: float,
    ) -> bool:
        if self.target is None:
            return False
        matched = matching_detection(
            detections,
            self.target,
            float(self.config.get("target_match_distance_px", 100.0)),
        )
        if matched is None:
            return False
        self.target = dict(matched)
        self.last_seen_at = float(now)
        self.error_px = float(detection_center(matched)[0] - self.image_center_x)
        return True

    def _safety_fault(self, safety: dict[str, Any], inference_error: str) -> str:
        if inference_error:
            return f"INFERENCE ERROR: {inference_error}"
        if not bool(safety.get("ros_ready", False)):
            return "ROS NOT READY"
        if not bool(safety.get("f103_fresh", False)):
            return "F103 STATE STALE"
        if bool(safety.get("motion_busy", False)):
            return "MOTION BUSY"
        if bool(safety.get("control_busy", False)) and not bool(
            safety.get("lease_owned", False)
        ):
            return "CABINET CONTROL BUSY"
        return ""

    def update(
        self,
        detections: Sequence[dict[str, Any]],
        result_at: float,
        now: float,
        safety: dict[str, Any],
        inference_error: str = "",
    ) -> tuple[float, bool]:
        now = float(now)
        result_at = float(result_at)
        fresh_result = result_at > 0.0 and result_at != self.last_processed_result_at
        if fresh_result:
            self.last_processed_result_at = result_at

        if not self.active:
            self.command_vy = 0.0
            self.lease_desired = False
            return self.command_vy, self.lease_desired

        safety_fault = self._safety_fault(safety, inference_error)
        if safety_fault:
            self.fault(safety_fault)
            return self.command_vy, self.lease_desired

        if self.state != "WAIT_TARGET" and bool(safety.get("lease_error")):
            self.fault(f"LEASE ERROR: {safety['lease_error']}")
            return self.command_vy, self.lease_desired

        if fresh_result and self.target is not None:
            self._update_target(detections, now)

        if self.target is not None and self.last_seen_at > 0.0:
            if now - self.last_seen_at > float(self.config.get("target_lost_timeout_s", 0.5)):
                self.fault("TARGET LOST")
                return self.command_vy, self.lease_desired

        if self.state == "WAIT_TARGET":
            self.command_vy = 0.0
            self.lease_desired = False
            if now - self.enabled_at > float(self.config.get("target_wait_timeout_s", 5.0)):
                self.fault("TARGET TIMEOUT")
                return self.command_vy, self.lease_desired
            if not fresh_result:
                return self.command_vy, self.lease_desired
            selected = highest_confidence_detection(detections)
            if selected is None:
                self.candidate = None
                self.candidate_frames = 0
                return self.command_vy, self.lease_desired
            if self.candidate is None:
                self.candidate = dict(selected)
                self.candidate_frames = 1
            else:
                matched = matching_detection(
                    detections,
                    self.candidate,
                    float(self.config.get("target_match_distance_px", 100.0)),
                )
                if matched is None:
                    self.candidate = dict(selected)
                    self.candidate_frames = 1
                else:
                    self.candidate = dict(matched)
                    self.candidate_frames += 1
            if self.candidate_frames >= int(self.config.get("stable_frames", 3)):
                self.target = dict(self.candidate)
                self.last_seen_at = now
                self.error_px = float(detection_center(self.target)[0] - self.image_center_x)
                self.state = "ACQUIRE"
                self.acquire_started_at = now
                self.lease_desired = True
            return self.command_vy, self.lease_desired

        if self.state == "ACQUIRE":
            self.command_vy = 0.0
            self.lease_desired = True
            if bool(safety.get("lease_owned", False)):
                self.state = "EVALUATE"
                return self.command_vy, self.lease_desired
            if now - self.acquire_started_at > float(self.config.get("lease_timeout_s", 2.0)):
                self.fault("LEASE TIMEOUT")
            return self.command_vy, self.lease_desired

        if not bool(safety.get("lease_owned", False)):
            self.fault("LEASE LOST")
            return self.command_vy, self.lease_desired

        if self.state == "PULSE":
            self.lease_desired = True
            feedback = safety.get("wheel_feedback_mm_s")
            feedback_seq = int(safety.get("wheel_feedback_seq", -1))
            wheel_targets = safety.get("wheel_targets_mm_s")
            feedback_available = (
                isinstance(feedback, (list, tuple))
                and len(feedback) >= 4
                and isinstance(wheel_targets, (list, tuple))
                and len(wheel_targets) >= 4
            )
            if self.pulse_motion_started_at <= 0.0 and feedback_available:
                if feedback_seq != self.pulse_feedback_seq:
                    self.pulse_feedback_seq = feedback_seq
                    threshold = abs(float(self.config.get("startup_feedback_mm_s", 2.0)))
                    active_feedback = [
                        abs(float(actual)) >= threshold
                        for target, actual in zip(wheel_targets, feedback)
                        if abs(float(target)) >= 1.0
                    ]
                    if len(active_feedback) == 4 and all(active_feedback):
                        self.pulse_feedback_frames += 1
                    else:
                        self.pulse_feedback_frames = 0
                    if self.pulse_feedback_frames >= int(
                        self.config.get("startup_feedback_frames", 2)
                    ):
                        self.pulse_motion_started_at = now
                if now - self.pulse_started_at >= float(
                    self.config.get("startup_timeout_s", 3.5)
                ):
                    self.fault("WHEEL START TIMEOUT")
                    return self.command_vy, self.lease_desired
                return self.command_vy, self.lease_desired

            pulse_reference = (
                self.pulse_motion_started_at
                if self.pulse_motion_started_at > 0.0
                else self.pulse_started_at
            )
            if now - pulse_reference < float(self.config.get("pulse_s", 0.3)):
                return self.command_vy, self.lease_desired
            self.command_vy = 0.0
            self.state = "SETTLE"
            self.settle_started_at = now
            self.settle_result_at = result_at
            return self.command_vy, self.lease_desired

        if self.state == "SETTLE":
            self.command_vy = 0.0
            self.lease_desired = True
            if now - self.settle_started_at < float(self.config.get("settle_s", 0.35)):
                return self.command_vy, self.lease_desired
            if not fresh_result or result_at == self.settle_result_at:
                return self.command_vy, self.lease_desired
            self.state = "EVALUATE"

        if self.state == "EVALUATE":
            self.command_vy = 0.0
            self.lease_desired = True
            if not fresh_result or self.target is None or self.error_px is None:
                return self.command_vy, self.lease_desired
            tolerance = float(self.config.get("center_tolerance_px", 20.0))
            absolute_error = abs(self.error_px)
            if self.pulse_initial_error > 0.0:
                divergence = float(self.config.get("divergence_px", 5.0))
                if absolute_error > self.pulse_initial_error + divergence:
                    self.divergence_count += 1
                else:
                    self.divergence_count = 0
                self.pulse_initial_error = 0.0
                if self.divergence_count >= int(self.config.get("divergence_pulses", 2)):
                    self.fault("DIRECTION MISMATCH")
                    return self.command_vy, self.lease_desired
            if absolute_error <= tolerance:
                self.centered_frames += 1
                if self.centered_frames >= int(self.config.get("centered_frames", 2)):
                    self.state = "ALIGNED"
                    self.reason = "CENTERED"
                    self.lease_desired = False
                return self.command_vy, self.lease_desired
            self.centered_frames = 0
            if self.attempts >= int(self.config.get("max_attempts", 10)):
                self.fault("MAX ATTEMPTS")
                return self.command_vy, self.lease_desired
            speed = abs(float(self.config.get("speed_m_s", 0.04)))
            lateral_sign = float(self.config.get("lateral_sign", -1.0))
            self.command_vy = lateral_sign * speed * (1.0 if self.error_px > 0.0 else -1.0)
            self.pulse_initial_error = absolute_error
            self.pulse_started_at = now
            self.pulse_motion_started_at = 0.0
            self.pulse_feedback_frames = 0
            self.pulse_feedback_seq = int(safety.get("wheel_feedback_seq", -1))
            self.attempts += 1
            self.state = "PULSE"
        return self.command_vy, self.lease_desired

    def snapshot(self) -> dict[str, Any]:
        target = None
        if self.target is not None:
            center = detection_center(self.target)
            target = {
                "class_name": str(self.target.get("class_name", "")),
                "confidence": round(float(self.target.get("confidence", 0.0)), 4),
                "rectified_center": [round(float(center[0]), 3), round(float(center[1]), 3)],
            }
        return {
            "state": self.state,
            "active": self.active,
            "reason": self.reason,
            "attempts": self.attempts,
            "max_attempts": int(self.config.get("max_attempts", 10)),
            "error_px": None if self.error_px is None else round(self.error_px, 3),
            "center_x_px": self.image_center_x,
            "tolerance_px": float(self.config.get("center_tolerance_px", 20.0)),
            "command_vy_m_s": round(self.command_vy, 4),
            "pulse_phase": (
                "STARTING"
                if self.state == "PULSE" and self.pulse_motion_started_at <= 0.0
                else "MOVING"
                if self.state == "PULSE"
                else ""
            ),
            "lease_desired": self.lease_desired,
            "target": target,
        }


class RosStateBridge:
    def __init__(self, ros_config: dict[str, Any], alignment_config: dict[str, Any]):
        self.ros_config = ros_config
        self.alignment_config = alignment_config
        self._lock = threading.Lock()
        self._payload: dict[str, Any] = {}
        self._received_at = 0.0
        self._received_monotonic = 0.0
        self._vision_payload: dict[str, Any] | None = None
        self._desired_vy = 0.0
        self._lease_desired = False
        self._lease_owned = False
        self._lease_pending = False
        self._lease_error = ""
        self._service_ready = False
        self._ros_ready = False
        self._ros_error = "starting"
        self._nav_action_active = False
        self._nav_manager_active = False
        self._nav_manager_phase = ""
        self._last_normal_motion_at = 0.0
        self._other_cabinet_publishers = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            import rclpy
            from action_msgs.msg import GoalStatusArray
            from geometry_msgs.msg import Twist
            from std_msgs.msg import String
            from std_srvs.srv import SetBool

            rclpy.init(args=None)
            node = rclpy.create_node("cabinet_camera_overlay")
            state_topic = str(self.ros_config["f103_state_topic"])
            vision_topic = str(self.ros_config["vision_state_topic"])
            cabinet_topic = str(self.ros_config.get("cabinet_cmd_topic", "/cmd_vel_cabinet"))
            lease_service = str(
                self.ros_config.get(
                    "cabinet_mode_service", "/f103_usb_ros2_bridge/cabinet_mode"
                )
            )
            publisher = node.create_publisher(String, vision_topic, 10)
            command_publisher = node.create_publisher(Twist, cabinet_topic, 10)
            lease_client = node.create_client(SetBool, lease_service)

            def on_message(message: String) -> None:
                try:
                    payload = json.loads(message.data)
                except Exception:
                    return
                with self._lock:
                    self._payload = payload
                    self._received_at = time.time()
                    self._received_monotonic = time.monotonic()

            def on_nav_status(message: GoalStatusArray) -> None:
                active = any(
                    int(item.status) in ACTIVE_NAV_ACTION_STATUSES for item in message.status_list
                )
                with self._lock:
                    self._nav_action_active = active

            def on_manager_status(message: String) -> None:
                phase = str(message.data or "").split(":", 1)[0].strip().upper()
                with self._lock:
                    if phase in ACTIVE_NAV_MANAGER_PHASES:
                        self._nav_manager_active = True
                    elif phase in TERMINAL_NAV_MANAGER_PHASES:
                        self._nav_manager_active = False
                    self._nav_manager_phase = phase

            def on_normal_twist(message: Twist) -> None:
                values = (
                    message.linear.x,
                    message.linear.y,
                    message.angular.z,
                )
                if any(abs(float(value)) > 1e-4 for value in values):
                    with self._lock:
                        self._last_normal_motion_at = time.monotonic()

            node.create_subscription(String, state_topic, on_message, 20)
            node.create_subscription(
                GoalStatusArray,
                str(self.ros_config.get("nav_status_topic", "/navigate_to_pose/_action/status")),
                on_nav_status,
                10,
            )
            node.create_subscription(
                String,
                str(self.ros_config.get("nav_manager_status_topic", "/maze_turn_manager/status")),
                on_manager_status,
                10,
            )
            for topic in self.ros_config.get("normal_cmd_topics", ["/cmd_vel", "/cmd_vel_cmd"]):
                if str(topic) and str(topic) != cabinet_topic:
                    node.create_subscription(Twist, str(topic), on_normal_twist, 20)
            with self._lock:
                self._ros_ready = True
                self._ros_error = ""
            last_sent = 0.0
            last_command_sent = 0.0
            last_graph_check = 0.0
            lease_future = None
            lease_request_value = False
            while rclpy.ok() and not self._stop.is_set():
                now = time.monotonic()
                service_ready = lease_client.service_is_ready()
                with self._lock:
                    self._service_ready = service_ready
                    desired_lease = self._lease_desired
                    lease_owned = self._lease_owned

                if lease_future is not None and lease_future.done():
                    try:
                        response = lease_future.result()
                        success = bool(response is not None and response.success)
                        message = "" if response is None else str(response.message)
                    except Exception as exc:
                        success = False
                        message = str(exc)
                    with self._lock:
                        if success:
                            self._lease_owned = bool(lease_request_value)
                            self._lease_error = ""
                        else:
                            self._lease_error = message or "cabinet mode request failed"
                        self._lease_pending = False
                    lease_future = None

                if lease_future is None:
                    with self._lock:
                        desired_lease = self._lease_desired
                        lease_owned = self._lease_owned
                    if desired_lease != lease_owned and service_ready:
                        request = SetBool.Request()
                        request.data = bool(desired_lease)
                        lease_request_value = bool(desired_lease)
                        lease_future = lease_client.call_async(request)
                        with self._lock:
                            self._lease_pending = True

                if now - last_command_sent >= 0.05:
                    with self._lock:
                        vy = self._desired_vy if self._lease_owned and self._lease_desired else 0.0
                    command = Twist()
                    command.linear.x = 0.0
                    command.linear.y = float(vy)
                    command.angular.z = 0.0
                    command_publisher.publish(command)
                    last_command_sent = now

                if now - last_sent >= 0.2:
                    with self._lock:
                        vision_payload = self._vision_payload
                    if vision_payload is not None:
                        output = String()
                        output.data = json.dumps(
                            vision_payload, ensure_ascii=True, separators=(",", ":")
                        )
                        publisher.publish(output)
                    last_sent = now
                if now - last_graph_check >= 0.5:
                    with self._lock:
                        self._other_cabinet_publishers = max(
                            0, int(node.count_publishers(cabinet_topic)) - 1
                        )
                    last_graph_check = now
                rclpy.spin_once(node, timeout_sec=0.02)
            zero = Twist()
            for _ in range(5):
                command_publisher.publish(zero)
                rclpy.spin_once(node, timeout_sec=0.02)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            with self._lock:
                self._ros_ready = False
                self._ros_error = str(exc)
                self._received_at = time.time()

    def snapshot(self) -> tuple[dict[str, Any], float]:
        with self._lock:
            return dict(self._payload), self._received_at

    def publish(self, payload: dict[str, Any]) -> None:
        with self._lock:
            self._vision_payload = payload

    def set_control(self, vy: float, lease_desired: bool) -> None:
        with self._lock:
            if lease_desired and not self._lease_desired:
                self._lease_error = ""
            self._desired_vy = float(vy) if lease_desired else 0.0
            self._lease_desired = bool(lease_desired)

    def control_snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            payload = dict(self._payload)
            age = (
                float("inf")
                if self._received_monotonic <= 0.0
                else max(0.0, now - self._received_monotonic)
            )
            lease_owned = self._lease_owned
            normal_motion_active = (
                self._last_normal_motion_at > 0.0
                and now - self._last_normal_motion_at
                <= float(self.alignment_config.get("motion_busy_timeout_s", 0.5))
            )
            nav_active = self._nav_action_active or self._nav_manager_active
            target_motion = any(
                abs(float(payload.get(key, 0.0) or 0.0)) > 0.5
                for key in ("target_vx_mm_s", "target_vy_mm_s", "target_wz_mrad_s")
            )
            reported_cabinet_mode = bool(payload.get("cabinet_mode_active", False))
            wheel_feedback = payload.get("last_wheel_mm_s", [0, 0, 0, 0])
            wheel_targets = payload.get("target_wheels_mm_s", [0, 0, 0, 0])
            other_publishers = self._other_cabinet_publishers
            return {
                "ros_ready": self._ros_ready,
                "ros_error": self._ros_error,
                "f103_age_s": None if not math.isfinite(age) else round(age, 3),
                "f103_fresh": age <= float(self.alignment_config.get("f103_state_timeout_s", 1.0)),
                "nav_active": nav_active,
                "nav_manager_phase": self._nav_manager_phase,
                "normal_motion_active": normal_motion_active,
                "motion_busy": bool(
                    nav_active or normal_motion_active or (target_motion and not lease_owned)
                ),
                "control_busy": bool(
                    (reported_cabinet_mode and not lease_owned)
                    or (other_publishers > 0 and not lease_owned)
                ),
                "other_cabinet_publishers": other_publishers,
                "reported_cabinet_mode": reported_cabinet_mode,
                "wheel_feedback_mm_s": list(wheel_feedback),
                "wheel_targets_mm_s": list(wheel_targets),
                "wheel_feedback_seq": int(payload.get("odom_count", -1)),
                "lease_owned": lease_owned,
                "lease_pending": self._lease_pending,
                "lease_error": self._lease_error,
                "service_ready": self._service_ready,
            }

    def stop(self) -> None:
        self.set_control(0.0, False)
        deadline = time.monotonic() + 2.0
        while self._thread.is_alive() and time.monotonic() < deadline:
            with self._lock:
                if not self._lease_owned and not self._lease_pending:
                    break
            time.sleep(0.05)
        self._stop.set()
        self._thread.join(timeout=2.0)


def create_tag_detector(dictionary_name: str):
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers
    return lambda image: cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)


def detect_tags(
    image: np.ndarray,
    intrinsics: dict[str, Any],
    tag_config: dict[str, Any],
    detect_markers,
) -> dict[str, Any]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners_values, ids, rejected = detect_markers(gray)
    _, _, new_matrix, _ = intrinsic_arrays(intrinsics)
    tag_size = float(tag_config.get("tag_size_mm", 50.0))
    half = tag_size * 0.5
    object_corners = np.asarray(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )
    allowed = {int(value) for value in tag_config.get("allowed_ids", [10, 11, 12, 13])}
    tags: list[dict[str, Any]] = []
    if ids is not None:
        for corners_value, marker_id_value in zip(corners_values, ids.reshape(-1)):
            marker_id = int(marker_id_value)
            if allowed and marker_id not in allowed:
                continue
            corners = np.asarray(corners_value, dtype=np.float64).reshape(4, 2)
            sides = np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)
            pose_ok, rotation_vector, translation_vector = cv2.solvePnP(
                object_corners,
                corners,
                new_matrix,
                np.zeros(5, dtype=np.float64),
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            reprojection_error = float("inf")
            transform = None
            if pose_ok:
                projected, _ = cv2.projectPoints(
                    object_corners,
                    rotation_vector,
                    translation_vector,
                    new_matrix,
                    np.zeros(5, dtype=np.float64),
                )
                reprojection_error = float(
                    np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - corners) ** 2, axis=1)))
                )
                rotation, _ = cv2.Rodrigues(rotation_vector)
                transform = np.eye(4, dtype=np.float64)
                transform[:3, :3] = rotation
                transform[:3, 3] = translation_vector.reshape(3)
            homography = cv2.getPerspectiveTransform(
                corners.astype(np.float32), object_corners[:, :2].astype(np.float32)
            )
            valid = bool(
                transform is not None
                and transform[2, 3] > 0.0
                and float(np.mean(sides)) >= float(tag_config.get("min_side_px", 24.0))
                and reprojection_error <= float(tag_config.get("max_reprojection_error_px", 2.0))
            )
            tags.append(
                {
                    "id": marker_id,
                    "valid": valid,
                    "corners": corners.tolist(),
                    "distance_mm": None if transform is None else float(np.linalg.norm(transform[:3, 3])),
                    "reprojection_error_px": reprojection_error,
                    "T_camera_tag": None if transform is None else transform.tolist(),
                    "homography": homography.tolist(),
                }
            )
    preferred = int(tag_config.get("preferred_id", 10))
    tags.sort(key=lambda item: (not item["valid"], item["id"] != preferred))
    reference = next((item for item in tags if item["valid"]), None)
    return {
        "detected_at": time.time(),
        "reference_id": None if reference is None else reference["id"],
        "tags": tags,
        "rejected_count": len(rejected),
    }


def enrich_switches(
    detections: Sequence[dict[str, Any]],
    reference: dict[str, Any] | None,
    intrinsics: dict[str, Any],
    transform_base_camera: np.ndarray | None,
    cabinet_rotation_deg: float,
) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    homography = None
    transform_base_tag = None
    reference_id = None
    if reference is not None:
        reference_id = int(reference["id"])
        homography = np.asarray(reference["homography"], dtype=np.float64)
        transform_camera_tag = np.asarray(reference["T_camera_tag"], dtype=np.float64)
        transform_base_tag = (
            None
            if transform_base_camera is None
            else transform_base_camera @ transform_camera_tag
        )
    angle = math.radians(cabinet_rotation_deg)
    cabinet_rotation = np.asarray(
        [[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]],
        dtype=np.float64,
    )
    for detection in detections:
        x1, y1, x2, y2 = [float(value) for value in detection["box_xyxy"]]
        raw_corners = np.asarray([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.float64)
        raw_center = np.asarray(detection["center_xy"], dtype=np.float64).reshape(1, 2)
        rectified_corners = raw_points_to_rectified(raw_corners, intrinsics)
        rectified_center = raw_points_to_rectified(raw_center, intrinsics).reshape(2)
        item = dict(detection)
        item.update(
            {
                "reference_tag_id": reference_id,
                "rectified_corners": rectified_corners.tolist(),
                "rectified_center": rectified_center.tolist(),
            }
        )
        if homography is not None:
            tag_xy = cv2.perspectiveTransform(
                rectified_center.reshape(1, 1, 2), homography
            ).reshape(2)
            cabinet_uv = cabinet_rotation @ tag_xy
            item["tag_xy_mm"] = tag_xy.tolist()
            item["cabinet_uvn_mm"] = [
                float(cabinet_uv[0]),
                float(cabinet_uv[1]),
                0.0,
            ]
            if transform_base_tag is not None:
                base_point = transform_base_tag @ np.asarray(
                    [tag_xy[0], tag_xy[1], 0.0, 1.0]
                )
                item["base_xyz_mm"] = base_point[:3].tolist()
        output.append(item)
    return output


def put_label(image: np.ndarray, text: str, origin: tuple[int, int], color: tuple[int, int, int]) -> None:
    x, y = origin
    (width, height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1)
    y = max(height + 5, min(image.shape[0] - baseline - 2, y))
    x = max(0, min(image.shape[1] - width - 6, x))
    cv2.rectangle(image, (x, y - height - 5), (x + width + 6, y + baseline + 2), (20, 20, 20), -1)
    cv2.putText(image, text, (x + 3, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)


def draw_overlay(
    image: np.ndarray,
    tags: dict[str, Any],
    switches: Sequence[dict[str, Any]],
    status: dict[str, Any],
) -> np.ndarray:
    display = image.copy()
    alignment = status.get("alignment", {})
    center_x = int(round(float(alignment.get("center_x_px", image.shape[1] * 0.5))))
    tolerance = int(round(float(alignment.get("tolerance_px", 20.0))))
    cv2.line(display, (center_x, 0), (center_x, image.shape[0] - 1), (230, 210, 40), 1)
    cv2.line(
        display,
        (center_x - tolerance, 0),
        (center_x - tolerance, image.shape[0] - 1),
        (100, 100, 40),
        1,
    )
    cv2.line(
        display,
        (center_x + tolerance, 0),
        (center_x + tolerance, image.shape[0] - 1),
        (100, 100, 40),
        1,
    )
    selected_target = alignment.get("target")
    for tag in tags.get("tags", []):
        corners = np.rint(np.asarray(tag["corners"])).astype(np.int32)
        color = (40, 210, 70) if tag["valid"] else (40, 80, 230)
        cv2.polylines(display, [corners], True, color, 2, cv2.LINE_AA)
        distance = tag.get("distance_mm")
        label = f"TAG {tag['id']}"
        if distance is not None:
            label += f" {distance:.0f}mm err={tag['reprojection_error_px']:.2f}px"
        put_label(display, label, tuple(corners[0]), color)

    colors = {"kai": (50, 210, 70), "guan": (50, 80, 235)}
    for item in switches:
        polygon = np.rint(np.asarray(item["rectified_corners"])).astype(np.int32)
        color = colors.get(str(item["class_name"]), (30, 190, 220))
        selected = False
        if isinstance(selected_target, dict):
            selected_center = np.asarray(
                selected_target.get("rectified_center", [-1000.0, -1000.0]),
                dtype=np.float64,
            )
            selected = bool(
                str(selected_target.get("class_name", "")) == str(item.get("class_name", ""))
                and np.linalg.norm(selected_center - detection_center(item)) < 4.0
            )
        cv2.polylines(display, [polygon], True, (0, 255, 255) if selected else color, 4 if selected else 2, cv2.LINE_AA)
        center = tuple(np.rint(np.asarray(item["rectified_center"])).astype(int))
        cv2.drawMarker(display, center, color, cv2.MARKER_CROSS, 14, 2)
        lines = [
            f"{str(item['class_name']).upper()} {item['confidence']:.2f}"
        ]
        if "tag_xy_mm" in item:
            tag_x, tag_y = item["tag_xy_mm"]
            cabinet_u, cabinet_v, cabinet_n = item["cabinet_uvn_mm"]
            lines[0] += f" TAG={item['reference_tag_id']}"
            lines.append(f"Tag X={tag_x:.1f} Y={tag_y:.1f} mm")
            lines.append(f"Cab U={cabinet_u:.1f} V={cabinet_v:.1f} N={cabinet_n:.1f}")
            if "base_xyz_mm" in item:
                base_x, base_y, base_z = item["base_xyz_mm"]
                lines.append(f"PROVISIONAL Base X={base_x:.1f} Y={base_y:.1f} Z={base_z:.1f}")
            else:
                lines.append("STALE Base X=-- Y=-- Z=--")
        else:
            lines.append("TAG LOST - IMAGE ALIGNMENT ONLY")
        start_y = max(80, center[1] - 55)
        for index, line in enumerate(lines):
            line_color = (60, 80, 255) if "Base" in line or "TAG LOST" in line else color
            put_label(display, line, (center[0] + 8, start_y + index * 22), line_color)

    top_lines = [
        f"Camera {status.get('display_fps', 0.0):.1f} FPS  RKNN {status.get('inference_fps', 0.0):.1f} FPS / {status.get('inference_ms', 0.0):.1f} ms",
        f"Tag {status.get('tag_state', 'LOST')}  Arm {status.get('arm_state', 'STALE')}  Calibration PROVISIONAL",
        (
            f"Align {alignment.get('state', 'DISABLED')}  "
            f"err={alignment.get('error_px') if alignment.get('error_px') is not None else '--'}px  "
            f"vy={alignment.get('command_vy_m_s', 0.0):+.3f}m/s  "
            f"try={alignment.get('attempts', 0)}/{alignment.get('max_attempts', 10)}"
        ),
    ]
    if alignment.get("reason"):
        top_lines.append(f"ALIGN {alignment['reason']}")
    if status.get("error"):
        top_lines.append(f"ERROR {status['error']}")
    for index, line in enumerate(top_lines):
        color = (230, 230, 230) if index == 0 else (60, 80, 255)
        put_label(display, line, (8, 24 + index * 24), color)
    return display


class CabinetCameraOverlay:
    def __init__(self, config_path: Path):
        self.config_path = config_path
        self.config = load_yaml(config_path)
        paths = self.config["paths"]
        self.intrinsics_path = Path(paths["intrinsics"])
        self.handeye_path = Path(paths["handeye"])
        expected_hash = str(paths.get("intrinsics_sha256", "")).upper()
        actual_hash = sha256_file(self.intrinsics_path)
        if expected_hash and actual_hash != expected_hash:
            raise RuntimeError(f"intrinsics SHA256 mismatch: {actual_hash}")
        self.intrinsics = load_yaml(self.intrinsics_path)
        handeye = load_yaml(self.handeye_path)
        self.handeye_accepted = bool(handeye.get("accepted", False))
        self.transform_camera_mount_camera = np.asarray(
            handeye["T_camera_mount_camera"]["matrix"], dtype=np.float64
        )
        if self.transform_camera_mount_camera.shape != (4, 4):
            raise ValueError("T_camera_mount_camera must be 4x4")
        camera_matrix, dist_coeffs, new_matrix, image_size = intrinsic_arrays(self.intrinsics)
        if str(self.intrinsics["model"]) == "fisheye":
            self.maps = cv2.fisheye.initUndistortRectifyMap(
                camera_matrix, dist_coeffs, np.eye(3), new_matrix, image_size, cv2.CV_32FC1
            )
        else:
            self.maps = cv2.initUndistortRectifyMap(
                camera_matrix, dist_coeffs, None, new_matrix, image_size, cv2.CV_32FC1
            )
        self.image_size = image_size
        self.detect_markers = create_tag_detector(self.config["apriltag"]["dictionary"])
        self.worker = LatestFrameRknnWorker(self.config["yolo"])
        self.alignment = AlignmentController(
            self.config.get("alignment", {}), self.image_size[0]
        )
        self.ros = RosStateBridge(
            self.config["ros"], self.config.get("alignment", {})
        )
        runtime_dir = Path(self.config["paths"]["runtime_dir"])
        self.frame_path = runtime_dir / "latest_raw.jpg"
        self.health_path = runtime_dir / "health.json"
        self.stop_requested = False
        self.fullscreen = False

    def request_stop(self, *_args) -> None:
        self.stop_requested = True

    def current_base_camera(self) -> tuple[np.ndarray | None, float | None]:
        payload, received_at = self.ros.snapshot()
        ik_state = payload.get("last_ik5_ok") or payload.get("last_ik6_ok")
        if not isinstance(ik_state, dict):
            return None, None
        age = payload.get("last_ik5_ok_age_s")
        if age is None:
            age = payload.get("last_ik6_ok_age_s")
        if age is None:
            age = max(0.0, time.time() - received_at) if received_at > 0.0 else float("inf")
        age = float(age)
        if age > float(self.config["arm"].get("state_timeout_s", 30.0)):
            return None, age
        joint_deg = ik_state.get("joint_deg")
        if not isinstance(joint_deg, list) or len(joint_deg) < 4:
            pwm = ik_state.get("servo_pwm")
            if not isinstance(pwm, list) or len(pwm) < 4:
                return None, age
            joint_deg = pwm_to_joint_deg(pwm, self.config["arm"])
        mount = camera_mount_transform(joint_deg, self.config["arm"])
        return mount @ self.transform_camera_mount_camera, age

    def open_camera(self) -> cv2.VideoCapture:
        camera_config = self.config["camera"]
        device = str(camera_config["device"])
        width = int(camera_config["width"])
        height = int(camera_config["height"])
        fps = int(camera_config["fps"])
        capture = None
        if str(camera_config.get("backend", "gstreamer")).lower() == "gstreamer":
            pipeline = (
                f"v4l2src device={device} io-mode=2 ! "
                f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
                "jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
                "appsink drop=true max-buffers=1 sync=false"
            )
            candidate = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if candidate.isOpened():
                capture = candidate
            else:
                candidate.release()
        if capture is None:
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f"cannot open camera: {camera_config['device']}")
        if capture.getBackendName() != "GSTREAMER":
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            capture.set(cv2.CAP_PROP_FPS, fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def run(self) -> int:
        signal.signal(signal.SIGINT, self.request_stop)
        signal.signal(signal.SIGTERM, self.request_stop)
        capture = self.open_camera()
        self.worker.start()
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, self.image_size[0], self.image_size[1])
        tag_result: dict[str, Any] = {"reference_id": None, "tags": []}
        last_yolo_submit = last_tag_run = last_shared_write = 0.0
        last_result_at = 0.0
        inference_result_count = 0
        frame_count = 0
        fps_started = time.monotonic()
        display_fps = inference_fps = 0.0
        last_health: dict[str, Any] = {}
        try:
            while not self.stop_requested:
                ok, frame = capture.read()
                if not ok or frame is None:
                    raise RuntimeError("camera read failed")
                if bool(self.config["camera"].get("rotate_180", True)):
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                actual_size = (frame.shape[1], frame.shape[0])
                if actual_size != self.image_size:
                    raise RuntimeError(
                        f"camera/intrinsics size mismatch: {actual_size} != {self.image_size}"
                    )
                now_mono = time.monotonic()
                if now_mono - last_yolo_submit >= float(self.config["yolo"].get("interval_s", 0.2)):
                    self.worker.submit(frame)
                    last_yolo_submit = now_mono
                rectified = cv2.remap(frame, self.maps[0], self.maps[1], cv2.INTER_LINEAR)
                if now_mono - last_tag_run >= float(self.config["apriltag"].get("interval_s", 0.2)):
                    tag_result = detect_tags(
                        rectified, self.intrinsics, self.config["apriltag"], self.detect_markers
                    )
                    last_tag_run = now_mono
                inference = self.worker.snapshot()
                if inference["result_at"] != last_result_at:
                    last_result_at = float(inference["result_at"])
                    inference_result_count += 1
                reference = next(
                    (
                        item
                        for item in tag_result.get("tags", [])
                        if item.get("valid") and item.get("id") == tag_result.get("reference_id")
                    ),
                    None,
                )
                transform_base_camera, arm_age = self.current_base_camera()
                switches = enrich_switches(
                    inference["detections"],
                    reference,
                    self.intrinsics,
                    transform_base_camera,
                    float(self.config["apriltag"].get("cabinet_mount_rotation_deg", 90.0)),
                )
                control_safety = self.ros.control_snapshot()
                command_vy, lease_desired = self.alignment.update(
                    switches,
                    float(inference["result_at"]),
                    now_mono,
                    control_safety,
                    str(inference["error"] or ""),
                )
                self.ros.set_control(command_vy, lease_desired)
                alignment_status = self.alignment.snapshot()
                alignment_status.update(
                    {
                        "lease_owned": bool(control_safety.get("lease_owned", False)),
                        "lease_pending": bool(control_safety.get("lease_pending", False)),
                        "navigation_active": bool(control_safety.get("nav_active", False)),
                        "motion_busy": bool(control_safety.get("motion_busy", False)),
                        "f103_fresh": bool(control_safety.get("f103_fresh", False)),
                        "f103_age_s": control_safety.get("f103_age_s"),
                        "wheel_targets_mm_s": list(
                            control_safety.get("wheel_targets_mm_s", [0, 0, 0, 0])
                        ),
                        "wheel_feedback_mm_s": list(
                            control_safety.get("wheel_feedback_mm_s", [0, 0, 0, 0])
                        ),
                    }
                )
                frame_count += 1
                elapsed = now_mono - fps_started
                if elapsed >= 1.0:
                    display_fps = frame_count / elapsed
                    inference_fps = inference_result_count / elapsed
                    frame_count = 0
                    inference_result_count = 0
                    fps_started = now_mono
                status = {
                    "display_fps": display_fps,
                    "inference_fps": inference_fps,
                    "inference_ms": float(inference["inference_ms"]),
                    "tag_state": "OK" if reference is not None else "LOST",
                    "arm_state": "OK" if transform_base_camera is not None else "STALE",
                    "arm_age_s": arm_age,
                    "error": inference["error"],
                    "alignment": alignment_status,
                }
                display = draw_overlay(rectified, tag_result, switches, status)
                cv2.imshow(WINDOW_NAME, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    self.alignment.fault("WINDOW CLOSED")
                    self.ros.set_control(0.0, False)
                    self.stop_requested = True
                elif key in (ord("f"), ord("F")):
                    self.fullscreen = not self.fullscreen
                    cv2.setWindowProperty(
                        WINDOW_NAME,
                        cv2.WND_PROP_FULLSCREEN,
                        cv2.WINDOW_FULLSCREEN if self.fullscreen else cv2.WINDOW_NORMAL,
                    )
                elif key in (ord("a"), ord("A")):
                    self.alignment.toggle(now_mono)
                    self.ros.set_control(0.0, self.alignment.lease_desired)
                elif key == ord(" "):
                    self.alignment.fault("EMERGENCY STOP")
                    self.ros.set_control(0.0, False)
                now_wall = time.time()
                payload = {
                    "schema_version": 2,
                    "running": True,
                    "updated_at": now_wall,
                    "camera": {
                        "device": self.config["camera"]["device"],
                        "width": actual_size[0],
                        "height": actual_size[1],
                        "rotate_180": bool(self.config["camera"].get("rotate_180", True)),
                        "display_fps": round(display_fps, 3),
                    },
                    "inference": {
                        "backend": inference["backend"],
                        "fps": round(inference_fps, 3),
                        "inference_ms": round(float(inference["inference_ms"]), 3),
                        "error": inference["error"],
                    },
                    "calibration": {
                        "handeye_accepted": self.handeye_accepted,
                        "coordinate_quality": "provisional_unaccepted",
                    },
                    "arm": {
                        "fresh": transform_base_camera is not None,
                        "age_s": arm_age,
                        "timeout_s": float(self.config["arm"].get("state_timeout_s", 30.0)),
                    },
                    "apriltag": tag_result,
                    "switches": switches,
                    "alignment": alignment_status,
                }
                self.ros.publish(payload)
                last_health = payload
                if now_mono - last_shared_write >= float(
                    self.config["sharing"].get("interval_s", 0.2)
                ):
                    encode_ok, encoded = cv2.imencode(
                        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(self.config["sharing"].get("jpeg_quality", 92))]
                    )
                    if encode_ok:
                        atomic_write_bytes(self.frame_path, encoded.tobytes())
                    atomic_write_json(self.health_path, payload)
                    last_shared_write = now_mono
        finally:
            self.alignment.fault("PROCESS STOPPED")
            self.ros.set_control(0.0, False)
            capture.release()
            self.worker.stop()
            self.worker.join(timeout=3.0)
            self.ros.stop()
            cv2.destroyAllWindows()
            last_health["running"] = False
            last_health["stopped_at"] = time.time()
            atomic_write_json(self.health_path, last_health)
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Live cabinet camera overlay for RK3588 HDMI desktop.")
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    return CabinetCameraOverlay(args.config).run()


if __name__ == "__main__":
    raise SystemExit(main())

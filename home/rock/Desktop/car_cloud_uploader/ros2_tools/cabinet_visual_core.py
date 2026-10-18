#!/usr/bin/env python3
from __future__ import annotations

import math
import random
import statistics
from collections import Counter, defaultdict, deque
from typing import Any, Callable, Iterable

import numpy as np


VOLTAGE_LABELS = {"dianya", "yakuang", "voltage", "voltage_meter"}
CURRENT_LABELS = {"dianliu", "liukuang", "current", "current_meter"}
LIGHT_LABELS = {"green_on", "green_off", "red_on", "red_off"}
DIGIT_LABELS = {str(value) for value in range(10)} | {"digit", "digits", "digital_display"}
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
VISUAL_GRACE_NAV_MANAGER_PHASES = {"DONE", "FAILED", "SAFETY_STOP"}


def navigation_manager_phase(status_text: str) -> str:
    return str(status_text or "").split(":", 1)[0].strip().upper()


def allows_post_navigation_visual_grace(manager_phase: str) -> bool:
    return str(manager_phase or "").strip().upper() in VISUAL_GRACE_NAV_MANAGER_PHASES


class NavigationActivityTracker:
    """Merge action and topic-originated managed navigation lifecycles."""

    def __init__(self, *, active_at_launch: bool = False):
        self.action_active = False
        self.action_status_seen = False
        self.manager_active = bool(active_at_launch)
        self.manager_status_seen = False
        self.manager_phase = "LAUNCH_ACTIVE" if active_at_launch else ""

    @property
    def active(self) -> bool:
        return bool(self.action_active or self.manager_active)

    @property
    def known(self) -> bool:
        return bool(
            self.action_status_seen
            or self.manager_status_seen
            or self.manager_phase == "LAUNCH_ACTIVE"
        )

    def update_action(self, statuses: Iterable[int]) -> bool:
        self.action_active = any(
            int(status) in ACTIVE_NAV_ACTION_STATUSES for status in statuses
        )
        self.action_status_seen = True
        return self.active

    def update_manager(self, status_text: str) -> bool:
        phase = navigation_manager_phase(status_text)
        if phase in ACTIVE_NAV_MANAGER_PHASES:
            self.manager_active = True
        elif phase in TERMINAL_NAV_MANAGER_PHASES:
            self.manager_active = False
        self.manager_phase = phase
        self.manager_status_seen = True
        return self.active

    def confirm_manager_inactive(self) -> None:
        self.manager_active = False
        self.manager_status_seen = True


class InferencePerformanceMonitor:
    def __init__(
        self,
        *,
        target_fps: float = 5.0,
        hard_min_fps: float = 4.0,
        hard_fail_hold_s: float = 5.0,
        window_frames: int = 30,
        warmup_frames: int = 30,
    ):
        self.target_fps = max(0.1, float(target_fps))
        self.hard_min_fps = max(0.1, min(float(hard_min_fps), self.target_fps))
        self.hard_fail_hold_s = max(0.0, float(hard_fail_hold_s))
        self.window_frames = max(3, int(window_frames))
        self.warmup_frames = max(3, min(int(warmup_frames), self.window_frames))
        self.timestamps: deque[float] = deque(maxlen=self.window_frames)
        self.hard_low_since: float | None = None

    def observe(self, timestamp: float) -> dict[str, Any]:
        now = float(timestamp)
        self.timestamps.append(now)
        return self.snapshot(now)

    def snapshot(self, timestamp: float | None = None) -> dict[str, Any]:
        now = float(timestamp if timestamp is not None else (self.timestamps[-1] if self.timestamps else 0.0))
        fps = 0.0
        if len(self.timestamps) >= 2:
            elapsed = self.timestamps[-1] - self.timestamps[0]
            if elapsed > 0.0:
                fps = (len(self.timestamps) - 1) / elapsed

        warmed_up = len(self.timestamps) >= self.warmup_frames
        hard_failed = False
        warning = ""
        if not warmed_up:
            state = "warming_up"
            self.hard_low_since = None
        elif fps >= self.target_fps:
            state = "normal"
            self.hard_low_since = None
        elif fps >= self.hard_min_fps:
            state = "degraded"
            warning = f"RKNN throughput below target: {fps:.2f} < {self.target_fps:.2f} FPS"
            self.hard_low_since = None
        else:
            if self.hard_low_since is None:
                self.hard_low_since = now
            low_duration = max(0.0, now - self.hard_low_since)
            hard_failed = low_duration >= self.hard_fail_hold_s
            state = "failed" if hard_failed else "hard_low_pending"
            warning = f"RKNN throughput below hard minimum: {fps:.2f} < {self.hard_min_fps:.2f} FPS"

        low_duration = 0.0 if self.hard_low_since is None else max(0.0, now - self.hard_low_since)
        return {
            "state": state,
            "fps": fps,
            "target_fps": self.target_fps,
            "hard_min_fps": self.hard_min_fps,
            "hard_low_duration_s": low_duration,
            "hard_fail_hold_s": self.hard_fail_hold_s,
            "warmed_up": warmed_up,
            "hard_failed": hard_failed,
            "warning": warning,
            "sample_count": len(self.timestamps),
        }


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def signed_minimum(value: float, minimum: float) -> float:
    if value == 0.0:
        return 0.0
    if abs(value) >= minimum:
        return value
    return minimum if value > 0.0 else -minimum


def trigger_evidence(
    result: dict[str, Any],
    label_getter: Callable[[Any], str] | None = None,
) -> dict[str, Any]:
    def default_label(item: Any) -> str:
        if not isinstance(item, dict):
            return ""
        return str(item.get("class_name") or item.get("label") or "").strip().lower()

    get_label = label_getter or default_label
    labels: list[str] = []
    seen: set[tuple[str, tuple[float, ...]]] = set()
    for item in list(result.get("detections", [])) + list(result.get("digit_detections", [])):
        if not isinstance(item, dict):
            continue
        label = get_label(item)
        raw_box = item.get("box_xyxy") or item.get("bbox") or item.get("box") or []
        box = tuple(round(float(value), 2) for value in raw_box) if len(raw_box) == 4 else ()
        key = (label, box)
        if key in seen:
            continue
        seen.add(key)
        labels.append(label)
    voltage = sum(label in VOLTAGE_LABELS for label in labels)
    current = sum(label in CURRENT_LABELS for label in labels)
    lights = sum(label in LIGHT_LABELS for label in labels)
    digits = sum(label in DIGIT_LABELS for label in labels)
    return {
        "matched": bool(
            (voltage > 0 and current > 0)
            or ((voltage + current) > 0 and lights > 0)
            or (digits >= 2 and lights > 0)
        ),
        "voltage_count": voltage,
        "current_count": current,
        "light_count": lights,
        "digit_count": digits,
    }


def fit_cabinet_plane(
    points: list[tuple[float, float]],
    *,
    iterations: int,
    inlier_threshold: float,
    min_inliers: int,
    min_span: float,
) -> dict[str, Any] | None:
    if len(points) < max(2, min_inliers):
        return None
    array = np.asarray(points, dtype=np.float64)
    rng = random.Random(0xCAB1)
    best_indices: np.ndarray | None = None
    best_residual = float("inf")
    for _ in range(max(8, iterations)):
        first, second = rng.sample(range(len(points)), 2)
        delta = array[second] - array[first]
        length = float(np.linalg.norm(delta))
        if length < 1e-6:
            continue
        direction = delta / length
        normal = np.asarray([-direction[1], direction[0]])
        residuals = np.abs((array - array[first]) @ normal)
        indices = np.flatnonzero(residuals <= inlier_threshold)
        if len(indices) < min_inliers:
            continue
        residual = float(np.mean(residuals[indices]))
        if best_indices is None or len(indices) > len(best_indices) or (
            len(indices) == len(best_indices) and residual < best_residual
        ):
            best_indices = indices
            best_residual = residual
    if best_indices is None:
        return None

    inliers = array[best_indices]
    center = np.mean(inliers, axis=0)
    _, _, vh = np.linalg.svd(inliers - center, full_matrices=False)
    direction = vh[0]
    normal = np.asarray([-direction[1], direction[0]])
    if float(normal @ center) < 0.0:
        normal = -normal
        direction = -direction
    projections = (inliers - center) @ direction
    span = float(np.max(projections) - np.min(projections))
    if span < min_span:
        return None
    distance = float(normal @ center)
    yaw_error = math.atan2(float(normal[1]), float(normal[0]))
    return {
        "distance_m": round(distance, 4),
        "yaw_error_rad": round(yaw_error, 5),
        "yaw_error_deg": round(math.degrees(yaw_error), 3),
        "inliers": int(len(inliers)),
        "span_m": round(span, 4),
        "residual_m": round(best_residual, 5),
    }


def docking_command(
    *,
    yaw_error: float,
    center_error: float,
    distance_error: float,
    yaw_tolerance: float,
    center_tolerance: float,
    distance_tolerance: float,
    yaw_gain: float,
    yaw_sign: float,
    lateral_gain: float,
    lateral_sign: float,
    distance_gain: float,
    min_angular: float,
    max_angular: float,
    min_lateral: float,
    max_lateral: float,
    min_linear: float,
    max_linear: float,
    max_reverse: float,
) -> tuple[float, float, float, str]:
    if abs(yaw_error) > yaw_tolerance:
        value = clamp(yaw_sign * yaw_gain * yaw_error, -max_angular, max_angular)
        return 0.0, 0.0, signed_minimum(value, min_angular), "yaw"
    if abs(center_error) > center_tolerance:
        value = clamp(lateral_sign * lateral_gain * center_error, -max_lateral, max_lateral)
        return 0.0, signed_minimum(value, min_lateral), 0.0, "lateral"
    if abs(distance_error) > distance_tolerance:
        value = clamp(distance_gain * distance_error, -max_reverse, max_linear)
        return signed_minimum(value, min_linear), 0.0, 0.0, "distance"
    return 0.0, 0.0, 0.0, "aligned"


class TemporalInspectionConsensus:
    def __init__(self, max_samples: int = 12):
        self.max_samples = max(3, int(max_samples))
        self.meters: dict[str, deque[dict[str, Any]]] = defaultdict(
            lambda: deque(maxlen=self.max_samples)
        )
        self.digits: deque[dict[str, Any]] = deque(maxlen=self.max_samples)
        self.lights: dict[str, deque[dict[str, Any]]] = defaultdict(
            lambda: deque(maxlen=self.max_samples)
        )

    def add_meter_readings(self, readings: Iterable[Any]) -> None:
        for item in readings:
            if not isinstance(item, dict):
                continue
            meter_type = str(item.get("meter_type") or "").strip().lower()
            if meter_type not in {"voltage", "current"}:
                continue
            try:
                value = float(item.get("value"))
            except (TypeError, ValueError):
                continue
            if not math.isfinite(value):
                continue
            sample = dict(item)
            sample["value"] = value
            self.meters[meter_type].append(sample)

    def add_digit_readings(self, readings: Any) -> None:
        if not isinstance(readings, dict):
            return
        rows = readings.get("rows", [])
        texts = tuple(
            str(row.get("text") or "").strip()
            for row in rows
            if isinstance(row, dict) and str(row.get("text") or "").strip()
        )
        if not texts:
            global_text = str(readings.get("global_text") or "").strip()
            texts = (global_text,) if global_text else ()
        if texts:
            self.digits.append({"texts": texts, "readings": dict(readings)})

    def add_light_slots(self, slots: dict[str, dict[str, Any]]) -> None:
        for slot, item in slots.items():
            if not slot or not isinstance(item, dict):
                continue
            label = str(item.get("class_name") or item.get("label") or "").strip().lower()
            if label not in LIGHT_LABELS:
                continue
            self.lights[str(slot)].append({"label": label, "detection": dict(item)})

    def snapshot(
        self,
        *,
        meter_min_samples: int = 3,
        digit_min_samples: int = 2,
        light_min_samples: int = 2,
        required_lights: int = 4,
    ) -> dict[str, Any]:
        meter_result: dict[str, Any] = {}
        for meter_type in ("voltage", "current"):
            samples = list(self.meters.get(meter_type, ()))
            if len(samples) < meter_min_samples:
                continue
            representative = dict(samples[-1])
            representative["value"] = round(
                float(statistics.median(float(item["value"]) for item in samples)), 4
            )
            representative["sample_count"] = len(samples)
            representative["aggregation"] = "median"
            representative["status"] = "ok"
            meter_result[meter_type] = representative

        digit_result: dict[str, Any] = {}
        if self.digits:
            counts = Counter(item["texts"] for item in self.digits)
            texts, count = counts.most_common(1)[0]
            if count >= digit_min_samples:
                representative = next(
                    item["readings"] for item in reversed(self.digits) if item["texts"] == texts
                )
                digit_result = {
                    "texts": list(texts),
                    "global_text": "".join(texts),
                    "sample_count": count,
                    "aggregation": "mode",
                    "readings": representative,
                }

        light_result: dict[str, Any] = {}
        for slot, values in self.lights.items():
            samples = list(values)
            if len(samples) < light_min_samples:
                continue
            counts = Counter(item["label"] for item in samples)
            label, count = counts.most_common(1)[0]
            if count * 2 <= len(samples):
                continue
            representative = next(
                item["detection"] for item in reversed(samples) if item["label"] == label
            )
            light_result[slot] = {
                "label": label,
                "sample_count": len(samples),
                "vote_count": count,
                "detection": representative,
            }

        return {
            "meters": meter_result,
            "meters_complete": all(item in meter_result for item in ("voltage", "current")),
            "digital_display": digit_result,
            "digital_display_complete": bool(digit_result),
            "status_lights": light_result,
            "status_lights_count": len(light_result),
            "status_lights_complete": len(light_result) >= required_lights,
        }

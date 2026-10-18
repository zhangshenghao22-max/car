#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable
from uuid import uuid4

try:
    import cv2
except Exception:  # pragma: no cover - board runtime dependency
    cv2 = None

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String as RosString
except Exception:  # pragma: no cover - board runtime dependency
    rclpy = None
    Twist = None
    Node = object
    LaserScan = None
    RosString = None
    qos_profile_sensor_data = None

ROOT_DIR = Path(__file__).resolve().parents[1]
TOOL_DIR = Path(__file__).resolve().parent
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

try:
    from cabinet_nav_takeover import (
        DEFAULT_CLASSES,
        DEFAULT_DIGIT_CLASSES,
        DEFAULT_DIGIT_MODEL,
        DEFAULT_MODEL,
        DEFAULT_TARGET_LABELS,
        STAGES,
        StreamDetector,
        apply_camera_controls,
        atomic_write_json,
        bbox_metrics,
        build_digit_readings,
        cabinet_data_from_result,
        draw_measurement_summary,
        draw_results,
        parse_device,
        rotate_frame,
        run_meter_readings,
        stage_score,
        target_bbox,
        utc_now_iso,
    )
except Exception as exc:  # pragma: no cover - import error is reported at runtime
    raise RuntimeError(f"failed to import cabinet_nav_takeover helpers: {exc}") from exc

try:
    from cabinet_inspection_runner import (
        SERVO_COUNT,
        SERVO_MAX_PWM,
        SERVO_MIN_PWM,
        build_arm_payload,
        clamp,
        clamp_int,
        load_profile,
        normalize_pwm,
    )
except Exception as exc:  # pragma: no cover - import error is reported at runtime
    raise RuntimeError(f"failed to import cabinet_inspection_runner helpers: {exc}") from exc


DEFAULT_PROFILE = ROOT_DIR / "cabinet_inspection_profiles" / "default.yaml"
DEFAULT_STATUS_FILE = ROOT_DIR / "runtime" / "cabinet_stream_adjust_test" / "status.json"
DEFAULT_CAPTURE_DIR = ROOT_DIR / "runtime" / "cabinet_stream_adjust_test" / "captures"
STATUS_LIGHT_LABELS = {"green_on", "green_off", "red_on", "red_off"}
STATUS_LIGHT_SLOTS = ("left_top", "right_top", "left_bottom", "right_bottom")


@dataclass
class BestFrame:
    score: float = 0.0
    image: Any = None
    result: dict[str, Any] = field(default_factory=dict)
    selected_count: int = 0
    updated_at: str = ""

    def snapshot(self) -> dict[str, Any]:
        return {
            "captured": self.image is not None,
            "score": round(float(self.score), 4),
            "selected_count": int(self.selected_count),
            "updated_at": self.updated_at,
        }


@dataclass
class EvaluatedFrame:
    stage: str
    score: float
    image: Any
    result: dict[str, Any]
    coverage: dict[str, Any]
    paths: dict[str, str] = field(default_factory=dict)


def parse_pwm_list(text: str) -> list[int]:
    values = [int(part.strip()) for part in str(text).split(",") if part.strip()]
    if len(values) != SERVO_COUNT:
        raise ValueError(f"expected {SERVO_COUNT} PWM values, got {len(values)}")
    return [clamp_int(value, SERVO_MIN_PWM, SERVO_MAX_PWM) for value in values]


def selected_detection_summary(result: dict[str, Any], limit: int = 12) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for item in list(result.get("detections", [])) + list(result.get("digit_detections", [])):
        if not isinstance(item, dict):
            continue
        box = item.get("box_xyxy") or item.get("bbox") or item.get("box") or []
        items.append(
            {
                "label": str(item.get("class_name") or item.get("display_name") or item.get("label") or ""),
                "confidence": round(float(item.get("confidence") or 0.0), 4),
                "box": [round(float(value), 2) for value in box[:4]] if isinstance(box, list) else [],
                "source": str(item.get("source") or ""),
            }
        )
        if len(items) >= limit:
            break
    return items


def detection_label(item: Any) -> str:
    if not isinstance(item, dict):
        return ""
    raw = str(item.get("class_name") or item.get("label") or item.get("display_name") or "").strip().lower()
    if ":" in raw:
        raw = raw.rsplit(":", 1)[-1].strip()
    return raw


def average_confidence(items: Iterable[dict[str, Any]]) -> float:
    values: list[float] = []
    for item in items:
        try:
            values.append(float(item.get("confidence") or 0.0))
        except Exception:
            pass
    if not values:
        return 0.0
    return sum(values) / float(len(values))


def detection_box(item: Any) -> list[float] | None:
    if not isinstance(item, dict):
        return None
    box = item.get("box_xyxy") or item.get("bbox") or item.get("box")
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        return None
    try:
        x1, y1, x2, y2 = [float(value) for value in box]
    except Exception:
        return None
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    if x2 <= x1 or y2 <= y1:
        return None
    return [x1, y1, x2, y2]


def box_center_distance(a: list[float], b: list[float]) -> float:
    acx = (a[0] + a[2]) * 0.5
    acy = (a[1] + a[3]) * 0.5
    bcx = (b[0] + b[2]) * 0.5
    bcy = (b[1] + b[3]) * 0.5
    return math.hypot(acx - bcx, acy - bcy)


def dedupe_by_box(items: list[dict[str, Any]], min_center_distance_px: float = 20.0) -> list[dict[str, Any]]:
    selected: list[dict[str, Any]] = []
    for item in sorted(items, key=lambda entry: float(entry.get("confidence") or 0.0), reverse=True):
        box = detection_box(item)
        if box is None:
            continue
        duplicate = False
        for kept in selected:
            kept_box = detection_box(kept)
            if kept_box is not None and box_center_distance(box, kept_box) < min_center_distance_px:
                duplicate = True
                break
        if not duplicate:
            selected.append(item)
    return selected


def status_light_items(result: dict[str, Any], min_conf: float = 0.0, image_shape: tuple[int, int] | None = None) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    for item in result.get("detections", []) if isinstance(result.get("detections"), list) else []:
        if detection_label(item) not in STATUS_LIGHT_LABELS:
            continue
        try:
            if float(item.get("confidence") or 0.0) < float(min_conf):
                continue
        except Exception:
            continue
        box = detection_box(item)
        if box is None:
            continue
        if image_shape is not None:
            height, width = image_shape
            cy = (box[1] + box[3]) * 0.5 / max(1.0, float(height))
            if cy < 0.45:
                continue
        items.append(item)
    return dedupe_by_box(items)


def status_light_slot_map(items: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    boxes = [(item, detection_box(item)) for item in items]
    boxes = [(item, box) for item, box in boxes if box is not None]
    if not boxes:
        return {}
    centers = [
        ((box[0] + box[2]) * 0.5, (box[1] + box[3]) * 0.5)
        for _, box in boxes
    ]
    min_x = min(center[0] for center in centers)
    max_x = max(center[0] for center in centers)
    min_y = min(center[1] for center in centers)
    max_y = max(center[1] for center in centers)
    x_spread = max_x - min_x
    y_spread = max_y - min_y
    x_mid = (min_x + max_x) * 0.5
    y_mid = (min_y + max_y) * 0.5
    slot_map: dict[str, dict[str, Any]] = {}
    if x_spread < 24.0 or y_spread < 24.0:
        axis = 0 if x_spread >= y_spread else 1
        ordered = sorted(boxes, key=lambda entry: ((entry[1][0] + entry[1][2]) if axis == 0 else (entry[1][1] + entry[1][3])))
        for index, (item, _) in enumerate(ordered[:4]):
            slot = f"slot_{index + 1}"
            current = slot_map.get(slot)
            if current is None or float(item.get("confidence") or 0.0) > float(current.get("confidence") or 0.0):
                slot_map[slot] = item
        return slot_map
    for item, box in boxes:
        cx = (box[0] + box[2]) * 0.5
        cy = (box[1] + box[3]) * 0.5
        slot = ("left" if cx <= x_mid else "right") + "_" + ("top" if cy <= y_mid else "bottom")
        current = slot_map.get(slot)
        if current is None or float(item.get("confidence") or 0.0) > float(current.get("confidence") or 0.0):
            slot_map[slot] = item
    return slot_map


def status_light_spread_score(items: list[dict[str, Any]]) -> float:
    boxes = [box for box in (detection_box(item) for item in items) if box is not None]
    if not boxes:
        return 0.0
    min_x = min(box[0] for box in boxes)
    max_x = max(box[2] for box in boxes)
    min_y = min(box[1] for box in boxes)
    max_y = max(box[3] for box in boxes)
    return max(0.0, max_x - min_x) * 0.002 + max(0.0, max_y - min_y) * 0.001


def digital_rows_from_result(result: dict[str, Any]) -> list[dict[str, Any]]:
    readings = result.get("digit_readings", {}) if isinstance(result.get("digit_readings"), dict) else {}
    rows = readings.get("rows", [])
    return rows if isinstance(rows, list) else []


def meter_status_from_result(result: dict[str, Any]) -> tuple[bool, bool, dict[str, Any]]:
    cabinet = result.get("cabinet_data", {}) if isinstance(result.get("cabinet_data"), dict) else {}
    voltage = cabinet.get("voltage", {}) if isinstance(cabinet.get("voltage"), dict) else {}
    current = cabinet.get("current", {}) if isinstance(cabinet.get("current"), dict) else {}
    voltage_ok = str(voltage.get("status") or "").lower() == "ok" and voltage.get("value") is not None
    current_ok = str(current.get("status") or "").lower() == "ok" and current.get("value") is not None
    return voltage_ok, current_ok, {"voltage": voltage, "current": current}


def active_motion_processes() -> list[str]:
    patterns = (
        "rt_robot_nav2_complete.launch.py",
        "controller_server",
        "planner_server",
        "bt_navigator",
        "behavior_server",
        "waypoint_follower",
        "velocity_smoother",
        "nav2_",
        "slam_toolbox",
        "slam_gmapping",
        "frontier_explorer",
        "start_board_f103_mapping.sh",
        "start_board_f103_navigation.sh",
    )
    try:
        completed = subprocess.run(
            ["pgrep", "-af", "|".join(patterns)],
            capture_output=True,
            text=True,
            check=False,
        )
    except Exception:
        return []
    lines = []
    for line in completed.stdout.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        if "cabinet_stream_adjust_test.py" in stripped:
            continue
        lines.append(stripped)
    return lines


class StreamAdjustNode(Node):
    def __init__(self, cmd_vel_topic: str, arm_topic: str, scan_topic: str, front_angle_deg: float, scan_valid_min_m: float):
        super().__init__("cabinet_stream_adjust_test")
        self.cmd_vel_topic = cmd_vel_topic
        self.arm_topic = arm_topic
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.arm_pub = self.create_publisher(RosString, arm_topic, 10)
        self.front_min = float("inf")
        self.last_scan_at = 0.0
        self.front_angle_rad = math.radians(max(5.0, min(60.0, float(front_angle_deg))))
        self.scan_valid_min_m = max(0.01, float(scan_valid_min_m))
        self.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg: LaserScan) -> None:
        front_ranges: list[float] = []
        for index, value in enumerate(msg.ranges):
            if not math.isfinite(float(value)):
                continue
            if value < self.scan_valid_min_m or value > msg.range_max:
                continue
            angle = float(msg.angle_min) + float(index) * float(msg.angle_increment)
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= self.front_angle_rad:
                front_ranges.append(float(value))
        self.front_min = min(front_ranges) if front_ranges else float("inf")
        self.last_scan_at = time.time()

    def wait_subscribers(self, topic: str, timeout_s: float) -> int:
        deadline = time.time() + max(0.0, float(timeout_s))
        count = 0
        while time.time() < deadline:
            if topic == self.arm_topic:
                count = self.count_subscribers(self.arm_topic)
            else:
                count = self.count_subscribers(self.cmd_vel_topic)
            if count > 0:
                return count
            rclpy.spin_once(self, timeout_sec=0.05)
        return count

    def wait_scan(self, timeout_s: float) -> bool:
        deadline = time.time() + max(0.0, float(timeout_s))
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.last_scan_at > 0.0:
                return True
        return False

    def publish_twist(self, linear_x: float, angular_z: float) -> dict[str, float]:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        return {"linear_x": round(float(linear_x), 5), "angular_z": round(float(angular_z), 5)}

    def stop(self, repeats: int = 4) -> None:
        for _ in range(max(1, int(repeats))):
            self.publish_twist(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def publish_arm(self, pwm: list[int], duration_ms: int) -> str:
        payload = build_arm_payload(pwm, duration_ms)
        msg = RosString()
        msg.data = payload
        self.arm_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.03)
        return payload


class CabinetStreamAdjustTest:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.status_file = Path(args.status_file)
        self.capture_dir = Path(args.capture_dir)
        self.labels = {
            str(item).strip().lower()
            for item in str(args.target_labels).split(",")
            if str(item).strip()
        } or set(DEFAULT_TARGET_LABELS)
        self.profile = load_profile(Path(args.profile))
        self.servo_min = int(self.profile.get("servo_min_pwm", SERVO_MIN_PWM) or SERVO_MIN_PWM)
        self.servo_max = int(self.profile.get("servo_max_pwm", SERVO_MAX_PWM) or SERVO_MAX_PWM)
        home_raw: Iterable[Any] = parse_pwm_list(args.home_pwm) if args.home_pwm else self.profile.get("home_pwm", [1500] * SERVO_COUNT)
        self.home_pwm = normalize_pwm(home_raw, low=self.servo_min, high=self.servo_max)
        self.current_pwm = list(self.home_pwm)
        self.center_pwm = list(self.home_pwm)

        closed_loop = self.profile.get("closed_loop", {}) if isinstance(self.profile.get("closed_loop"), dict) else {}
        self.pan_servo = int(args.pan_servo if args.pan_servo is not None else closed_loop.get("pan_servo", 0))
        self.tilt_servo = int(args.tilt_servo if args.tilt_servo is not None else closed_loop.get("tilt_servo", 3))
        self.pan_gain = float(args.pan_pwm_per_norm_error if args.pan_pwm_per_norm_error is not None else closed_loop.get("pan_pwm_per_norm_error", -120))
        self.tilt_gain = float(args.tilt_pwm_per_norm_error if args.tilt_pwm_per_norm_error is not None else closed_loop.get("tilt_pwm_per_norm_error", 120))

        self.state = "starting"
        self.target_seen_count = 0
        self.lost_count = 0
        self.started_at = time.time()
        self.test_id = f"stream_adjust_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid4().hex[:6]}"
        self.last_detection: dict[str, Any] = {}
        self.last_cmd_vel: dict[str, float] = {"linear_x": 0.0, "angular_z": 0.0}
        self.last_arm_payload = ""
        self.last_error = ""
        self.result: dict[str, Any] = {}
        self.logs: list[str] = []
        self.node: StreamAdjustNode | None = None
        self.detector: StreamDetector | None = None
        self.camera: Any = None
        self.last_arm_at = 0.0
        self.final_frame: Any = None
        self.final_result: dict[str, Any] = {}
        self.best_frames: dict[str, BestFrame] = {stage: BestFrame() for stage in STAGES}
        self.evaluated_frames: dict[str, EvaluatedFrame] = {}
        self.coverage: dict[str, Any] = {}
        self.missing_items: list[str] = []
        self.quality_score = 0.0
        self.status_light_candidates: list[dict[str, Any]] = []
        self.status_light_refine_attempts = 0
        self.status_light_roi_recheck_count = 0
        self.status_light_union_slots: dict[str, dict[str, Any]] = {}
        self.status_light_missing_slots: list[str] = list(STATUS_LIGHT_SLOTS)
        self.last_status_light_roi: dict[str, Any] = {}

    def log(self, message: str) -> None:
        entry = f"{utc_now_iso()} {message}"
        self.logs.append(entry)
        self.logs = self.logs[-100:]
        print(f"[cabinet-stream-adjust-test] {message}", file=sys.stderr, flush=True)

    def capture_progress(self) -> dict[str, Any]:
        return {stage: self.best_frames[stage].snapshot() for stage in STAGES}

    def update_status(self, **extra: Any) -> None:
        payload = {
            "state": self.state,
            "test_id": self.test_id,
            "target_seen_count": int(self.target_seen_count),
            "lost_count": int(self.lost_count),
            "front_min": None
            if self.node is None or not math.isfinite(float(self.node.front_min))
            else round(float(self.node.front_min), 3),
            "scan_age_s": None
            if self.node is None or self.node.last_scan_at <= 0.0
            else round(time.time() - self.node.last_scan_at, 3),
            "current_pwm": list(self.current_pwm),
            "home_pwm": list(self.home_pwm),
            "last_cmd_vel": dict(self.last_cmd_vel),
            "last_arm_payload": self.last_arm_payload,
            "capture_progress": self.capture_progress(),
            "coverage": dict(self.coverage),
            "missing_items": list(self.missing_items),
            "quality_score": round(float(self.quality_score), 4),
            "photo_count": int(self.result.get("photo_count", 0)) if isinstance(self.result, dict) else 0,
            "status_light_candidates": list(self.status_light_candidates),
            "status_light_refine_attempts": int(self.status_light_refine_attempts),
            "status_light_roi_recheck_count": int(self.status_light_roi_recheck_count),
            "status_light_union_count": int(len(self.status_light_union_slots)),
            "status_light_missing_slots": list(self.status_light_missing_slots),
            "status_light_conf_used": float(self.args.status_light_conf),
            "speedup_mode": True,
            "last_status_light_roi": dict(self.last_status_light_roi),
            "last_detection": dict(self.last_detection),
            "last_error": self.last_error,
            "result": dict(self.result),
            "logs": list(self.logs[-30:]),
            "pid": os.getpid(),
            "updated_at": utc_now_iso(),
        }
        payload.update(extra)
        atomic_write_json(self.status_file, payload)

    def ensure_runtime(self) -> None:
        if cv2 is None:
            raise RuntimeError("OpenCV is unavailable")
        if rclpy is None or Twist is None or LaserScan is None or RosString is None:
            raise RuntimeError("ROS2 Python packages are unavailable; source ROS2 before running")
        if not self.args.allow_nav_running:
            active = active_motion_processes()
            if active:
                raise RuntimeError(
                    "navigation/mapping process is active; stop it before standalone stream adjust test: "
                    + " | ".join(active[:6])
                )
        if not (0 <= self.pan_servo < SERVO_COUNT and 0 <= self.tilt_servo < SERVO_COUNT):
            raise RuntimeError(f"invalid pan/tilt servo index: pan={self.pan_servo}, tilt={self.tilt_servo}")

    def open_camera(self) -> None:
        if apply_camera_controls is not None:
            report = apply_camera_controls(self.args.device, self.args.camera_profile, {})
            for error in report.get("errors", []):
                self.log(f"camera control warning: {error}")
        source = parse_device(self.args.device)
        attempts = max(1, int(self.args.camera_open_retries))
        retry_delay = max(0.0, float(self.args.camera_open_retry_s))
        errors: list[str] = []
        for attempt in range(1, attempts + 1):
            camera = None
            try:
                if hasattr(cv2, "CAP_V4L2"):
                    camera = cv2.VideoCapture(source, cv2.CAP_V4L2)
                else:
                    camera = cv2.VideoCapture(source)
                if not camera.isOpened():
                    raise RuntimeError("device did not open")
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.args.preview_width))
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.args.preview_height))
                camera.set(cv2.CAP_PROP_FPS, int(self.args.fps))
                with contextlib_suppress():
                    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                first_frame = None
                for _ in range(5):
                    ok, frame = camera.read()
                    if ok and frame is not None:
                        first_frame = frame
                if first_frame is None:
                    raise RuntimeError("device opened but returned no frame")
                self.camera = camera
                self.log(f"camera ready via V4L2 attempt={attempt}/{attempts}")
                return
            except Exception as exc:
                errors.append(f"attempt {attempt}: {exc}")
                if camera is not None:
                    camera.release()
                if attempt < attempts:
                    time.sleep(retry_delay)
        raise RuntimeError(f"failed to open camera {self.args.device}: {'; '.join(errors)}")

    def read_frame(self) -> Any:
        if self.camera is None:
            raise RuntimeError("camera is not open")
        ok, frame = self.camera.read()
        if not ok or frame is None:
            raise RuntimeError(f"failed to read frame from camera: {self.args.device}")
        return rotate_frame(frame, int(self.args.rotate))

    def detect_frame(self, frame: Any) -> dict[str, Any]:
        if self.detector is None:
            raise RuntimeError("detector is not initialized")
        result = self.detector.detect(frame)
        self.recheck_status_light_roi(frame, result)
        return result

    def status_light_roi(self, frame: Any, result: dict[str, Any]) -> tuple[int, int, int, int]:
        height, width = frame.shape[:2]
        lights = status_light_items(result, min_conf=float(self.args.status_light_conf), image_shape=(height, width))
        boxes = [box for box in (detection_box(item) for item in lights) if box is not None]
        if boxes:
            x1 = min(box[0] for box in boxes)
            y1 = min(box[1] for box in boxes)
            x2 = max(box[2] for box in boxes)
            y2 = max(box[3] for box in boxes)
            pad_x = max(100.0, (x2 - x1) * 0.8)
            pad_y = max(90.0, (y2 - y1) * 0.9)
            return (
                max(0, int(x1 - pad_x)),
                max(0, int(y1 - pad_y)),
                min(width, int(x2 + pad_x)),
                min(height, int(y2 + pad_y)),
            )
        return (
            int(width * 0.08),
            int(height * float(self.args.status_light_roi_min_y)),
            int(width * 0.92),
            int(height * 0.98),
        )

    def recheck_status_light_roi(self, frame: Any, result: dict[str, Any]) -> None:
        if self.detector is None or not bool(self.args.status_light_crop_recheck):
            return
        height, width = frame.shape[:2]
        current_lights = status_light_items(result, min_conf=float(self.args.status_light_conf), image_shape=(height, width))
        if len(current_lights) >= int(self.args.required_status_lights):
            return
        x1, y1, x2, y2 = self.status_light_roi(frame, result)
        if x2 - x1 < 40 or y2 - y1 < 40:
            return
        crop = frame[y1:y2, x1:x2]
        if crop is None or crop.size == 0:
            return
        scale = max(1.0, float(self.args.status_light_crop_scale))
        if scale > 1.0:
            crop = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        added: list[dict[str, Any]] = []
        infer_ms = 0.0
        try:
            for model in self.detector.models:
                detections, elapsed_ms = self.detector._run_one(model, crop, float(self.args.status_light_crop_conf))
                infer_ms += float(elapsed_ms)
                for item in detections:
                    if detection_label(item) not in STATUS_LIGHT_LABELS:
                        continue
                    box = detection_box(item)
                    if box is None:
                        continue
                    mapped = [
                        int(round(x1 + box[0] / scale)),
                        int(round(y1 + box[1] / scale)),
                        int(round(x1 + box[2] / scale)),
                        int(round(y1 + box[3] / scale)),
                    ]
                    mapped_item = json.loads(json.dumps(item, ensure_ascii=False))
                    mapped_item["box_xyxy"] = mapped
                    mapped_item["bbox"] = mapped
                    mapped_item["box"] = mapped
                    mapped_item["source"] = "status_light_roi"
                    mapped_item["model"] = str(model.get("label") or mapped_item.get("model") or "")
                    added.append(mapped_item)
        except Exception as exc:
            self.last_error = f"status light roi recheck failed: {exc}"
            return
        if not added:
            self.last_status_light_roi = {
                "enabled": True,
                "roi": [x1, y1, x2, y2],
                "added": 0,
                "infer_ms": round(infer_ms, 3),
            }
            return
        existing = list(result.get("detections", [])) if isinstance(result.get("detections"), list) else []
        non_lights = [item for item in existing if detection_label(item) not in STATUS_LIGHT_LABELS]
        lights = [item for item in existing if detection_label(item) in STATUS_LIGHT_LABELS]
        lights.extend(added)
        result["detections"] = non_lights + dedupe_by_box(lights, min_center_distance_px=float(self.args.status_light_dedupe_px))
        result["inference_ms"] = round(float(result.get("inference_ms") or 0.0) + infer_ms, 3)
        self.status_light_roi_recheck_count += 1
        self.last_status_light_roi = {
            "enabled": True,
            "roi": [x1, y1, x2, y2],
            "added": len(added),
            "infer_ms": round(infer_ms, 3),
            "conf": float(self.args.status_light_crop_conf),
            "scale": scale,
        }

    def update_best_frames(self, frame: Any, result: dict[str, Any]) -> None:
        height, width = frame.shape[:2]
        for stage in STAGES:
            score, matches = stage_score(stage, result, width, height)
            if score <= 0.0:
                continue
            if stage == "status_lights":
                score += min(4, len(status_light_items(result))) * 1.2
            elif stage == "digital_display":
                digit_count = sum(1 for row in digital_rows_from_result(result) if str(row.get("text") or "").strip())
                score += min(2, digit_count) * 1.8
            current = self.best_frames[stage]
            if score > current.score:
                best = BestFrame(
                    score=score,
                    image=frame.copy(),
                    result=json.loads(json.dumps(result, ensure_ascii=False)),
                    selected_count=len(matches),
                    updated_at=utc_now_iso(),
                )
                self.best_frames[stage] = best
                self.evaluate_stage_candidate(stage, best)
        self.refresh_global_coverage()

    def evaluate_stage_candidate(self, stage: str, best: BestFrame) -> None:
        if best.image is None:
            return
        prefix = f"{self.test_id}_{stage}_candidate"
        try:
            result = self.enrich_stage_result(stage, best.image, best.result, prefix)
        except Exception as exc:
            self.last_error = str(exc)
            result = json.loads(json.dumps(best.result, ensure_ascii=False))
        if best.image is not None:
            height, width = best.image.shape[:2]
            result["_image_shape"] = [int(height), int(width)]
        coverage = self.coverage_for_stage(stage, result)
        score = self.score_for_stage(stage, coverage, best.score)
        current = self.evaluated_frames.get(stage)
        if current is None or score > current.score:
            self.evaluated_frames[stage] = EvaluatedFrame(
                stage=stage,
                score=score,
                image=best.image.copy(),
                result=json.loads(json.dumps(result, ensure_ascii=False)),
                coverage=coverage,
            )

    def coverage_for_stage(self, stage: str, result: dict[str, Any]) -> dict[str, Any]:
        if stage == "meters":
            voltage_ok, current_ok, meter_payload = meter_status_from_result(result)
            readings = result.get("meter_readings", []) if isinstance(result.get("meter_readings"), list) else []
            return {
                "stage": stage,
                "meters": bool(voltage_ok and current_ok),
                "voltage_ok": bool(voltage_ok),
                "current_ok": bool(current_ok),
                "meter_readings_count": len(readings),
                "meter_values": meter_payload,
            }
        if stage == "digital_display":
            rows = digital_rows_from_result(result)
            useful_rows = [row for row in rows if str(row.get("text") or "").strip()]
            return {
                "stage": stage,
                "digital_display": len(useful_rows) >= int(self.args.required_digital_rows),
                "digital_rows": len(useful_rows),
                "digital_text": str((result.get("digit_readings", {}) or {}).get("global_text") or ""),
                "digital_row_texts": [str(row.get("text") or "") for row in useful_rows],
            }
        if stage == "status_lights":
            image_shape = None
            if isinstance(result.get("_image_shape"), list) and len(result["_image_shape"]) >= 2:
                image_shape = (int(result["_image_shape"][0]), int(result["_image_shape"][1]))
            lights = status_light_items(result, min_conf=float(self.args.status_light_conf), image_shape=image_shape)
            labels = [detection_label(item) for item in lights]
            candidates = selected_detection_summary({"detections": lights}, limit=8)
            if len(lights) >= len(self.status_light_candidates):
                self.status_light_candidates = candidates
            return {
                "stage": stage,
                "status_lights": len(lights) >= int(self.args.required_status_lights),
                "status_lights_count": len(lights),
                "status_light_labels": labels,
                "status_light_avg_confidence": round(average_confidence(lights), 4),
                "status_light_candidates": candidates,
                "status_light_boxes": [
                    [round(float(value), 2) for value in detection_box(item) or []]
                    for item in lights
                ],
                "status_light_conf_used": float(self.args.status_light_conf),
            }
        return {"stage": stage}

    def score_for_stage(self, stage: str, coverage: dict[str, Any], base_score: float) -> float:
        score = float(base_score)
        if stage == "meters":
            if coverage.get("voltage_ok"):
                score += 5.0
            if coverage.get("current_ok"):
                score += 5.0
        elif stage == "digital_display":
            score += min(int(coverage.get("digital_rows", 0)), int(self.args.required_digital_rows)) * 4.0
        elif stage == "status_lights":
            count = int(coverage.get("status_lights_count", 0))
            score += min(count, int(self.args.required_status_lights)) * 10.0
            score += float(coverage.get("status_light_avg_confidence", 0.0))
            boxes = coverage.get("status_light_boxes", [])
            if isinstance(boxes, list):
                pseudo_items = [{"box_xyxy": box, "confidence": 1.0} for box in boxes if isinstance(box, list)]
                score += status_light_spread_score(pseudo_items)
        return round(score, 4)

    def refresh_global_coverage(self) -> None:
        meters = self.evaluated_frames.get("meters")
        digital = self.evaluated_frames.get("digital_display")
        lights = self.evaluated_frames.get("status_lights")
        union_slots = self.compute_status_light_union_slots()
        self.status_light_union_slots = union_slots
        if all(slot in STATUS_LIGHT_SLOTS for slot in union_slots):
            self.status_light_missing_slots = [slot for slot in STATUS_LIGHT_SLOTS if slot not in union_slots]
        else:
            missing_count = max(0, int(self.args.required_status_lights) - len(union_slots))
            self.status_light_missing_slots = [f"missing_light_{index + 1}" for index in range(missing_count)]
        coverage = {
            "meters": bool(meters and meters.coverage.get("meters")),
            "voltage_ok": bool(meters and meters.coverage.get("voltage_ok")),
            "current_ok": bool(meters and meters.coverage.get("current_ok")),
            "digital_display": bool(digital and digital.coverage.get("digital_display")),
            "digital_rows": int(digital.coverage.get("digital_rows", 0)) if digital else 0,
            "status_lights": bool((lights and lights.coverage.get("status_lights")) or len(union_slots) >= int(self.args.required_status_lights)),
            "status_lights_count": int(lights.coverage.get("status_lights_count", 0)) if lights else 0,
            "status_light_union_count": int(len(union_slots)),
            "status_light_missing_slots": list(self.status_light_missing_slots),
            "photo_count": len(self.select_capture_frames()),
        }
        missing: list[str] = []
        if not coverage["voltage_ok"]:
            missing.append("voltage_meter")
        if not coverage["current_ok"]:
            missing.append("current_meter")
        if coverage["digital_rows"] < int(self.args.required_digital_rows):
            missing.append(f"digital_rows_{coverage['digital_rows']}/{int(self.args.required_digital_rows)}")
        if coverage["status_light_union_count"] < int(self.args.required_status_lights):
            missing.append(f"status_lights_union_{coverage['status_light_union_count']}/{int(self.args.required_status_lights)}")
        self.coverage = coverage
        self.missing_items = missing
        self.quality_score = round(sum(frame.score for frame in self.evaluated_frames.values()), 4)

    def coverage_complete(self) -> bool:
        return bool(self.coverage.get("meters") and self.coverage.get("digital_display") and self.coverage.get("status_lights"))

    def compute_status_light_union_slots(self) -> dict[str, dict[str, Any]]:
        union: dict[str, dict[str, Any]] = {}
        frames = self.select_capture_frames(include_refresh=False)
        for frame in frames:
            image_shape = None
            if frame.image is not None:
                h, w = frame.image.shape[:2]
                image_shape = (int(h), int(w))
            lights = status_light_items(frame.result, min_conf=float(self.args.status_light_conf), image_shape=image_shape)
            slot_map = status_light_slot_map(lights)
            for slot, item in slot_map.items():
                current = union.get(slot)
                if current is None or float(item.get("confidence") or 0.0) > float(current.get("confidence") or 0.0):
                    union[slot] = item
        return union

    def select_capture_frames(self, include_refresh: bool = True) -> list[EvaluatedFrame]:
        selected: list[EvaluatedFrame] = []
        seen_keys: set[tuple[int, int]] = set()
        for stage in ("meters", "digital_display", "status_lights"):
            frame = self.evaluated_frames.get(stage)
            if frame is None:
                continue
            key = (id(frame.image), len(json.dumps(frame.result, ensure_ascii=False)))
            if key in seen_keys:
                continue
            selected.append(frame)
            seen_keys.add(key)
            if len(selected) >= int(self.args.max_photos):
                break
        return selected[: max(1, int(self.args.max_photos))]

    def start_ros(self) -> None:
        rclpy.init(args=None)
        self.node = StreamAdjustNode(
            self.args.cmd_vel_topic,
            self.args.arm_topic,
            self.args.scan_topic,
            self.args.front_angle_deg,
            self.args.scan_valid_min_m,
        )
        cmd_subs = self.node.wait_subscribers(self.args.cmd_vel_topic, float(self.args.subscriber_timeout_s))
        if cmd_subs <= 0:
            raise RuntimeError(f"no subscriber on {self.args.cmd_vel_topic}")
        arm_subs = self.node.wait_subscribers(self.args.arm_topic, float(self.args.subscriber_timeout_s))
        if arm_subs <= 0:
            raise RuntimeError(f"no subscriber on {self.args.arm_topic}")
        if not self.node.wait_scan(float(self.args.scan_timeout_s)):
            raise RuntimeError(f"no scan received on {self.args.scan_topic}")
        self.node.stop(repeats=5)
        self.log(f"ROS ready: cmd_vel subscribers={cmd_subs}, arm subscribers={arm_subs}")

    def move_arm(self, pwm: list[int], duration_ms: int) -> None:
        if self.node is None:
            raise RuntimeError("ROS node is not initialized")
        self.current_pwm = normalize_pwm(pwm, low=self.servo_min, high=self.servo_max)
        self.last_arm_payload = self.node.publish_arm(self.current_pwm, int(duration_ms))
        self.last_arm_at = time.time()
        self.update_status()

    def adjust_arm(self, metrics: dict[str, float]) -> bool:
        if self.node is None:
            return False
        if time.time() - self.last_arm_at < float(self.args.arm_command_interval_s):
            return False
        error_x = float(metrics.get("error_x", 0.0))
        error_y = float(metrics.get("error_y", 0.0))
        if abs(error_x) < float(self.args.arm_deadband) and abs(error_y) < float(self.args.arm_deadband):
            return False
        max_step = abs(int(self.args.max_arm_step_pwm))
        max_total = abs(int(self.args.max_arm_total_delta_pwm))
        pan_delta = 0 if abs(error_x) < float(self.args.arm_deadband) else clamp_int(round(error_x * self.pan_gain), -max_step, max_step)
        tilt_delta = 0 if abs(error_y) < float(self.args.arm_deadband) else clamp_int(round(error_y * self.tilt_gain), -max_step, max_step)
        next_pwm = list(self.current_pwm)
        for servo, delta in ((self.pan_servo, pan_delta), (self.tilt_servo, tilt_delta)):
            low = max(self.servo_min, self.center_pwm[servo] - max_total)
            high = min(self.servo_max, self.center_pwm[servo] + max_total)
            next_pwm[servo] = clamp_int(next_pwm[servo] + delta, low, high)
        if next_pwm == self.current_pwm:
            return False
        self.move_arm(next_pwm, int(self.args.arm_duration_ms))
        return True

    def search_target(self) -> bool:
        self.state = "searching_target"
        self.update_status()
        deadline = time.time() + max(3.0, float(self.args.search_timeout_s))
        period = 1.0 / max(0.2, float(self.args.search_rate_hz))
        while time.time() < deadline:
            loop_started = time.time()
            if self.node is not None:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            try:
                frame = self.read_frame()
                result = self.detect_frame(frame)
                self.update_best_frames(frame, result)
                box, count, confidence = target_bbox(result, self.labels)
            except Exception as exc:
                self.last_error = str(exc)
                box, count, confidence = None, 0, 0.0
                frame = None
                result = {}

            if box is None or frame is None:
                self.target_seen_count = 0
                self.last_detection = {"target_found": False, "selected_count": 0}
            else:
                self.target_seen_count += 1
                height, width = frame.shape[:2]
                metrics = bbox_metrics(box, width, height)
                self.last_detection = {
                    "target_found": True,
                    "selected_count": count,
                    "confidence": round(confidence, 4),
                    "bbox": [round(value, 2) for value in box],
                    "metrics": metrics,
                    "detections": selected_detection_summary(result),
                }
                self.final_frame = frame.copy()
                self.final_result = json.loads(json.dumps(result, ensure_ascii=False))
            self.update_status()
            if self.target_seen_count >= int(self.args.confirm_frames):
                self.state = "target_confirmed"
                self.log("target confirmed")
                self.update_status()
                return True
            time.sleep(max(0.02, period - (time.time() - loop_started)))
        self.state = "failed"
        self.result = {"message": "no cabinet target confirmed"}
        self.update_status()
        return False

    def scan_safe_for_motion(self) -> tuple[bool, str]:
        if self.node is None:
            return False, "ROS node unavailable"
        if self.node.last_scan_at <= 0.0:
            return False, "scan unavailable"
        scan_age = time.time() - self.node.last_scan_at
        if scan_age > float(self.args.max_scan_age_s):
            return False, f"scan stale: {scan_age:.2f}s"
        if self.node.front_min < float(self.args.front_emergency_m):
            return False, f"front obstacle too close: {self.node.front_min:.3f}m"
        return True, ""

    def sweep_direction(self) -> float:
        elapsed = max(0.0, time.time() - self.started_at)
        period = max(0.8, float(self.args.view_sweep_step_s))
        index = int(elapsed / period) % 6
        pattern = (1.0, 1.0, 0.0, -1.0, -1.0, 0.0)
        return pattern[index]

    def nudge_arm_for_missing_items(self) -> None:
        if self.node is None or time.time() - self.last_arm_at < float(self.args.arm_command_interval_s):
            return
        if self.coverage_complete():
            return
        missing_lights = any(item.startswith("status_lights") for item in self.missing_items)
        max_total = abs(int(self.args.status_light_pan_total_pwm if missing_lights else self.args.max_arm_total_delta_pwm))
        step = max(1, abs(int(self.args.status_light_arm_step_pwm if missing_lights else self.args.view_sweep_arm_step_pwm)))
        elapsed = max(0.0, time.time() - self.started_at)
        direction = -1 if int(elapsed / max(0.8, float(self.args.view_sweep_step_s))) % 2 else 1
        next_pwm = list(self.current_pwm)
        if missing_lights:
            servo = self.pan_servo
            low = max(self.servo_min, self.center_pwm[servo] - max_total)
            high = min(self.servo_max, self.center_pwm[servo] + max_total)
            next_pwm[servo] = clamp_int(next_pwm[servo] + direction * step, low, high)
            tilt_servo = self.tilt_servo
            tilt_total = abs(int(self.args.status_light_tilt_total_pwm))
            tilt_low = max(self.servo_min, self.center_pwm[tilt_servo] - tilt_total)
            tilt_high = min(self.servo_max, self.center_pwm[tilt_servo] + tilt_total)
            tilt_direction = -direction if int(elapsed / max(1.0, float(self.args.view_sweep_step_s) * 2.0)) % 2 else direction
            next_pwm[tilt_servo] = clamp_int(next_pwm[tilt_servo] + tilt_direction * max(1, step // 2), tilt_low, tilt_high)
        elif any(item.startswith("digital_rows") for item in self.missing_items):
            servo = self.tilt_servo
            low = max(self.servo_min, self.center_pwm[servo] - max_total)
            high = min(self.servo_max, self.center_pwm[servo] + max_total)
            next_pwm[servo] = clamp_int(next_pwm[servo] - direction * step, low, high)
        if next_pwm != self.current_pwm:
            self.move_arm(next_pwm, int(self.args.arm_duration_ms))

    def directed_status_light_direction(self) -> tuple[float, float]:
        missing = set(self.status_light_missing_slots)
        if not missing:
            return 0.0, 0.0
        left_missing = sum(1 for slot in missing if slot.startswith("left"))
        right_missing = sum(1 for slot in missing if slot.startswith("right"))
        top_missing = sum(1 for slot in missing if slot.endswith("top"))
        bottom_missing = sum(1 for slot in missing if slot.endswith("bottom"))
        pan_dir = 0.0
        tilt_dir = 0.0
        if left_missing != right_missing:
            pan_dir = -1.0 if left_missing > right_missing else 1.0
        if top_missing != bottom_missing:
            tilt_dir = -1.0 if top_missing > bottom_missing else 1.0
        return pan_dir, tilt_dir

    def nudge_arm_for_status_lights(self) -> None:
        if self.node is None or time.time() - self.last_arm_at < float(self.args.arm_command_interval_s):
            return
        if self.coverage.get("status_lights"):
            return
        pan_dir, tilt_dir = self.directed_status_light_direction()
        if pan_dir == 0.0:
            pan_dir = self.sweep_direction() or 1.0
        if tilt_dir == 0.0:
            elapsed = max(0.0, time.time() - self.started_at)
            tilt_dir = -1.0 if int(elapsed / max(1.0, float(self.args.view_sweep_step_s) * 2.0)) % 2 else 1.0
        step = max(1, abs(int(self.args.status_light_arm_step_pwm)))
        next_pwm = list(self.current_pwm)
        pan_low = max(self.servo_min, self.center_pwm[self.pan_servo] - abs(int(self.args.status_light_pan_total_pwm)))
        pan_high = min(self.servo_max, self.center_pwm[self.pan_servo] + abs(int(self.args.status_light_pan_total_pwm)))
        tilt_low = max(self.servo_min, self.center_pwm[self.tilt_servo] - abs(int(self.args.status_light_tilt_total_pwm)))
        tilt_high = min(self.servo_max, self.center_pwm[self.tilt_servo] + abs(int(self.args.status_light_tilt_total_pwm)))
        next_pwm[self.pan_servo] = clamp_int(next_pwm[self.pan_servo] + int(round(pan_dir * step)), pan_low, pan_high)
        next_pwm[self.tilt_servo] = clamp_int(next_pwm[self.tilt_servo] + int(round(tilt_dir * max(1, step // 2))), tilt_low, tilt_high)
        if next_pwm != self.current_pwm:
            self.move_arm(next_pwm, int(self.args.arm_duration_ms))

    def directed_status_light_yaw(self) -> float:
        pan_dir, _ = self.directed_status_light_direction()
        if pan_dir == 0.0:
            pan_dir = self.sweep_direction()
        return pan_dir * float(self.args.status_light_yaw_sweep_angular)

    def adjust_to_view(self) -> bool:
        if self.node is None:
            return False
        self.state = "adjusting_view"
        self.update_status()
        deadline = time.time() + max(5.0, float(self.args.adjust_timeout_s))
        collect_deadline = time.time() + max(5.0, float(self.args.max_approach_collect_s))
        period = 1.0 / max(1.0, float(self.args.control_rate_hz))
        ready_since = 0.0
        while time.time() < deadline:
            loop_started = time.time()
            rclpy.spin_once(self.node, timeout_sec=0.01)
            safe, reason = self.scan_safe_for_motion()
            if not safe:
                self.node.stop(repeats=6)
                self.state = "failed"
                self.result = {"message": reason}
                self.update_status(safety_block_reason=reason)
                return False

            try:
                frame = self.read_frame()
                result = self.detect_frame(frame)
                self.update_best_frames(frame, result)
                box, count, confidence = target_bbox(result, self.labels)
            except Exception as exc:
                self.last_error = str(exc)
                box, count, confidence = None, 0, 0.0
                frame = None
                result = {}

            if box is None or frame is None:
                self.lost_count += 1
                self.node.stop(repeats=2)
                self.last_detection = {"target_found": False, "selected_count": 0}
                self.update_status()
                if self.lost_count >= int(self.args.max_lost_frames):
                    self.state = "failed"
                    self.result = {"message": "target lost during adjustment"}
                    self.update_status()
                    return False
                time.sleep(period)
                continue

            self.lost_count = 0
            height, width = frame.shape[:2]
            metrics = bbox_metrics(box, width, height)
            area = float(metrics["area_ratio"])
            error_x = float(metrics["error_x"])
            error_y = float(metrics["error_y"])
            centered = abs(error_x) <= float(self.args.center_tolerance) and abs(error_y) <= float(self.args.arm_center_tolerance)
            image_large_enough = area >= float(self.args.target_area_min)
            near_limit = self.node.front_min <= float(self.args.front_stop_m)
            too_large = area >= float(self.args.target_area_max)
            self.last_detection = {
                "target_found": True,
                "selected_count": count,
                "confidence": round(confidence, 4),
                "bbox": [round(value, 2) for value in box],
                "metrics": metrics,
                "centered": bool(centered),
                "image_large_enough": bool(image_large_enough),
                "near_limit": bool(near_limit),
                "too_large": bool(too_large),
                "detections": selected_detection_summary(result),
            }
            self.final_frame = frame.copy()
            self.final_result = json.loads(json.dumps(result, ensure_ascii=False))

            self.adjust_arm(metrics)
            if self.coverage.get("meters") and self.coverage.get("digital_display") and not self.coverage.get("status_lights"):
                self.nudge_arm_for_status_lights()
            else:
                self.nudge_arm_for_missing_items()
            if self.coverage_complete() and time.time() - self.started_at >= float(self.args.min_collect_s):
                self.last_cmd_vel = self.node.publish_twist(0.0, 0.0)
                if ready_since <= 0.0:
                    ready_since = time.time()
                if time.time() - ready_since >= float(self.args.ready_hold_s):
                    self.node.stop(repeats=6)
                    self.state = "capturing"
                    self.update_status()
                    return True
            else:
                ready_since = 0.0
                angular = 0.0
                if abs(error_x) >= float(self.args.chassis_deadband):
                    angular = clamp(-float(self.args.angular_gain) * error_x, -float(self.args.max_angular), float(self.args.max_angular))
                elif not self.coverage_complete() and time.time() - self.started_at >= float(self.args.min_collect_s):
                    angular = self.directed_status_light_yaw() if any(item.startswith("status_lights") for item in self.missing_items) else self.sweep_direction() * float(self.args.view_sweep_angular)
                linear = 0.0
                if not image_large_enough and not too_large and not near_limit:
                    linear = float(self.args.max_linear)
                self.last_cmd_vel = self.node.publish_twist(linear, angular)
            self.update_status()
            if time.time() >= collect_deadline and self.coverage.get("meters") and self.coverage.get("digital_display"):
                self.node.stop(repeats=4)
                self.state = "capturing"
                self.result = {"message": "approach collection window ended; running targeted light refine if needed"}
                self.update_status()
                return True
            time.sleep(max(0.02, period - (time.time() - loop_started)))

        self.node.stop(repeats=6)
        self.state = "capturing"
        self.result = {"message": "view adjustment timed out; saving best partial coverage"}
        self.update_status()
        return True

    def status_light_refine(self) -> None:
        if self.node is None or self.coverage.get("status_lights"):
            return
        self.state = "status_light_refine"
        self.update_status()
        deadline = time.time() + max(5.0, float(self.args.status_light_refine_s))
        period = 1.0 / max(1.0, float(self.args.control_rate_hz))
        while time.time() < deadline and not self.coverage.get("status_lights"):
            loop_started = time.time()
            self.status_light_refine_attempts += 1
            rclpy.spin_once(self.node, timeout_sec=0.01)
            safe, reason = self.scan_safe_for_motion()
            if not safe:
                self.node.stop(repeats=6)
                self.last_error = reason
                self.update_status(safety_block_reason=reason)
                return
            try:
                frame = self.read_frame()
                result = self.detect_frame(frame)
                self.update_best_frames(frame, result)
                box, count, confidence = target_bbox(result, self.labels)
                if box is not None:
                    height, width = frame.shape[:2]
                    metrics = bbox_metrics(box, width, height)
                    self.last_detection = {
                        "target_found": True,
                        "selected_count": count,
                        "confidence": round(confidence, 4),
                        "bbox": [round(value, 2) for value in box],
                        "metrics": metrics,
                        "status_light_refine": True,
                        "detections": selected_detection_summary(result),
                    }
                self.final_frame = frame.copy()
                self.final_result = json.loads(json.dumps(result, ensure_ascii=False))
            except Exception as exc:
                self.last_error = str(exc)
            self.nudge_arm_for_status_lights()
            angular = self.directed_status_light_yaw()
            self.last_cmd_vel = self.node.publish_twist(0.0, angular)
            self.update_status()
            time.sleep(max(0.02, period - (time.time() - loop_started)))
        self.node.stop(repeats=6)
        self.update_status()

    def enrich_stage_result(self, stage: str, image: Any, result: dict[str, Any], prefix: str) -> dict[str, Any]:
        payload = json.loads(json.dumps(result, ensure_ascii=False))
        if (
            stage == "meters"
            and run_meter_readings is not None
            and not bool(payload.get("_temporally_enriched"))
        ):
            meter_args = argparse.Namespace(
                no_meter_readings=False,
                meter_margin=float(self.args.meter_margin),
                save_meter_debug=bool(self.args.save_meter_debug),
            )
            meter_readings, meter_warnings = run_meter_readings(
                image,
                payload.get("detections", []),
                meter_args,
                self.capture_dir,
                prefix,
            )
            payload["meter_readings"] = meter_readings
            payload["model_warnings"] = list(payload.get("model_warnings", [])) + list(meter_warnings)
        if build_digit_readings is not None:
            payload["digit_readings"] = build_digit_readings(
                payload.get("digit_detections", []),
                payload.get("meter_readings", []),
                image.shape[:2],
            )
        cabinet_update = cabinet_data_from_result(stage, payload)
        if cabinet_update:
            existing = payload.get("cabinet_data", {})
            if not isinstance(existing, dict):
                existing = {}
            existing.update(cabinet_update)
            payload["cabinet_data"] = existing
        return payload

    def save_image_pair(self, image: Any, result: dict[str, Any], prefix: str) -> dict[str, str]:
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        raw_path = self.capture_dir / f"{prefix}_raw.jpg"
        annotated_path = self.capture_dir / f"{prefix}_det.jpg"
        json_path = self.capture_dir / f"{prefix}.json"
        if not cv2.imwrite(str(raw_path), image):
            raise RuntimeError(f"failed to write {raw_path}")
        annotated = (
            draw_results(
                image,
                result.get("detections", []),
                float(result.get("inference_ms") or 0.0),
                str(result.get("backend") or ""),
            )
            if draw_results is not None
            else image.copy()
        )
        if draw_measurement_summary is not None:
            annotated = draw_measurement_summary(
                annotated,
                result.get("meter_readings", []),
                result.get("digit_readings", {}),
            )
        if not cv2.imwrite(str(annotated_path), annotated):
            raise RuntimeError(f"failed to write {annotated_path}")
        json_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        return {"raw": str(raw_path), "annotated": str(annotated_path), "json": str(json_path)}

    def save_captures(self) -> dict[str, Any]:
        self.state = "saving_captures"
        self.update_status()
        selected_frames = self.select_capture_frames()
        saved: dict[str, Any] = {"photos": []}
        cabinet_update: dict[str, Any] = {}
        if not selected_frames and self.final_frame is not None and self.final_result:
            result = self.enrich_stage_result("final", self.final_frame, self.final_result, f"{self.test_id}_final")
            result.update(
                {
                    "test_id": self.test_id,
                    "capture_type": "final",
                    "captured_at": utc_now_iso(),
                    "current_pwm": list(self.current_pwm),
                    "last_cmd_vel": dict(self.last_cmd_vel),
                }
            )
            paths = self.save_image_pair(self.final_frame, result, f"{self.test_id}_final")
            saved["photos"].append({"stage": "final", "paths": paths, "coverage": {}})

        stages_payload: dict[str, Any] = {}
        for frame in selected_frames:
            stage = frame.stage
            prefix = f"{self.test_id}_{stage}"
            result = json.loads(json.dumps(frame.result, ensure_ascii=False))
            result.update(
                {
                    "test_id": self.test_id,
                    "capture_type": stage,
                    "stream_best_score": round(float(frame.score), 4),
                    "stream_selected_count": int(self.best_frames.get(stage, BestFrame()).selected_count),
                    "coverage": dict(frame.coverage),
                    "captured_at": utc_now_iso(),
                }
            )
            paths = self.save_image_pair(frame.image, result, prefix)
            frame.paths = paths
            stages_payload[stage] = {"status": "captured", "paths": paths, "cabinet_data": result.get("cabinet_data", {})}
            saved["photos"].append({"stage": stage, "paths": paths, "coverage": dict(frame.coverage), "score": frame.score})
            if isinstance(result.get("cabinet_data"), dict):
                cabinet_update.update(result.get("cabinet_data", {}))
        for stage in STAGES:
            if stage not in stages_payload:
                stages_payload[stage] = {"status": "missing"}
        saved["stages"] = stages_payload
        saved["cabinet_update"] = cabinet_update
        self.refresh_global_coverage()
        status = "completed" if self.coverage_complete() else "partial"
        self.result = {
            "status": status,
            "captures": saved,
            "coverage": dict(self.coverage),
            "missing_items": list(self.missing_items),
            "quality_score": round(float(self.quality_score), 4),
            "photo_count": len(saved["photos"]),
            "current_pwm": list(self.current_pwm),
            "status_light_union_count": int(len(self.status_light_union_slots)),
            "status_light_missing_slots": list(self.status_light_missing_slots),
            "status_light_roi_recheck_count": int(self.status_light_roi_recheck_count),
        }
        self.state = status
        self.update_status()
        return saved

    def run(self) -> int:
        self.ensure_runtime()
        self.detector = StreamDetector(self.args)
        self.open_camera()
        self.start_ros()
        try:
            if not self.args.skip_initial_arm_home:
                self.state = "initial_arm_home"
                self.update_status()
                self.log(f"moving arm to home PWM {self.home_pwm}")
                self.move_arm(self.home_pwm, int(self.args.initial_arm_duration_ms))
                time.sleep(max(0.1, float(self.args.initial_arm_duration_ms) / 1000.0))
            if not self.search_target():
                return 1
            if not self.adjust_to_view():
                return 1
            self.status_light_refine()
            self.save_captures()
            return 0
        finally:
            if self.node is not None:
                self.node.stop(repeats=6)
                self.node.destroy_node()
                self.node = None
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
            if self.camera is not None:
                self.camera.release()
            if self.detector is not None:
                self.detector.close()


class contextlib_suppress:
    def __enter__(self):
        return None

    def __exit__(self, exc_type, exc, tb):
        return True


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Standalone streaming YOLO cabinet chassis + arm adjustment test.")
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE))
    parser.add_argument("--status-file", default=str(DEFAULT_STATUS_FILE))
    parser.add_argument("--capture-dir", default=str(DEFAULT_CAPTURE_DIR))
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--arm-topic", default="/arm_cmd")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--preview-width", type=int, default=1280)
    parser.add_argument("--preview-height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--camera-open-retries", type=int, default=3)
    parser.add_argument("--camera-open-retry-s", type=float, default=0.5)
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=180)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--backend", choices=("auto", "rknn", "onnxruntime", "opencv"), default="auto")
    parser.add_argument("--rknn-model-dir", type=Path, default=ROOT_DIR / "yolo_models" / "rknn")
    parser.add_argument("--require-rknn", action="store_true", default=False)
    parser.add_argument("--camera-profile", default="normal")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--classes", type=Path, default=DEFAULT_CLASSES)
    parser.add_argument("--extra-model", action="append", type=Path, default=[])
    parser.add_argument("--extra-classes", action="append", type=Path, default=[])
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--digit-model", type=Path, default=DEFAULT_DIGIT_MODEL)
    parser.add_argument("--digit-classes", type=Path, default=DEFAULT_DIGIT_CLASSES)
    parser.add_argument("--digit-conf", type=float, default=None)
    parser.add_argument("--digit-imgsz", type=int, default=None)
    parser.add_argument("--no-digits", action="store_true", default=False)
    parser.add_argument("--target-labels", default=",".join(sorted(DEFAULT_TARGET_LABELS)))
    parser.add_argument("--confirm-frames", type=int, default=3)
    parser.add_argument("--max-lost-frames", type=int, default=5)
    parser.add_argument("--search-rate-hz", type=float, default=2.0)
    parser.add_argument("--control-rate-hz", type=float, default=6.0)
    parser.add_argument("--search-timeout-s", type=float, default=120.0)
    parser.add_argument("--adjust-timeout-s", type=float, default=90.0)
    parser.add_argument("--max-linear", type=float, default=0.03)
    parser.add_argument("--max-angular", type=float, default=0.12)
    parser.add_argument("--angular-gain", type=float, default=0.55)
    parser.add_argument("--center-tolerance", type=float, default=0.08)
    parser.add_argument("--arm-center-tolerance", type=float, default=0.10)
    parser.add_argument("--chassis-deadband", type=float, default=0.025)
    parser.add_argument("--target-area-min", type=float, default=0.08)
    parser.add_argument("--target-area-max", type=float, default=0.45)
    parser.add_argument("--ready-hold-s", type=float, default=1.0)
    parser.add_argument("--front-angle-deg", type=float, default=18.0)
    parser.add_argument("--front-caution-m", type=float, default=0.35)
    parser.add_argument("--front-stop-m", type=float, default=0.06)
    parser.add_argument("--front-emergency-m", type=float, default=0.05)
    parser.add_argument("--scan-valid-min-m", type=float, default=0.05)
    parser.add_argument("--max-scan-age-s", type=float, default=1.0)
    parser.add_argument("--subscriber-timeout-s", type=float, default=5.0)
    parser.add_argument("--scan-timeout-s", type=float, default=5.0)
    parser.add_argument("--home-pwm", default="")
    parser.add_argument("--skip-initial-arm-home", action="store_true", default=False)
    parser.add_argument("--initial-arm-duration-ms", type=int, default=1200)
    parser.add_argument("--pan-servo", type=int, default=None)
    parser.add_argument("--tilt-servo", type=int, default=None)
    parser.add_argument("--pan-pwm-per-norm-error", type=float, default=None)
    parser.add_argument("--tilt-pwm-per-norm-error", type=float, default=None)
    parser.add_argument("--max-arm-step-pwm", type=int, default=30)
    parser.add_argument("--max-arm-total-delta-pwm", type=int, default=150)
    parser.add_argument("--arm-deadband", type=float, default=0.035)
    parser.add_argument("--arm-duration-ms", type=int, default=450)
    parser.add_argument("--arm-command-interval-s", type=float, default=0.45)
    parser.add_argument("--required-digital-rows", type=int, default=2)
    parser.add_argument("--required-status-lights", type=int, default=4)
    parser.add_argument("--max-photos", type=int, default=3)
    parser.add_argument("--min-collect-s", type=float, default=3.0)
    parser.add_argument("--max-approach-collect-s", type=float, default=25.0)
    parser.add_argument("--view-sweep-angular", type=float, default=0.045)
    parser.add_argument("--view-sweep-step-s", type=float, default=1.2)
    parser.add_argument("--view-sweep-arm-step-pwm", type=int, default=18)
    parser.add_argument("--status-light-conf", type=float, default=0.15)
    parser.add_argument("--status-light-refine-s", type=float, default=20.0)
    parser.add_argument("--status-light-crop-recheck", action="store_true", default=True)
    parser.add_argument("--no-status-light-crop-recheck", dest="status_light_crop_recheck", action="store_false")
    parser.add_argument("--status-light-crop-conf", type=float, default=0.10)
    parser.add_argument("--status-light-crop-scale", type=float, default=2.0)
    parser.add_argument("--status-light-roi-min-y", type=float, default=0.40)
    parser.add_argument("--status-light-dedupe-px", type=float, default=18.0)
    parser.add_argument("--status-light-arm-step-pwm", type=int, default=28)
    parser.add_argument("--status-light-pan-total-pwm", type=int, default=260)
    parser.add_argument("--status-light-tilt-total-pwm", type=int, default=90)
    parser.add_argument("--status-light-yaw-sweep-angular", type=float, default=0.035)
    parser.add_argument("--meter-margin", type=float, default=0.12)
    parser.add_argument("--save-meter-debug", action="store_true", default=False)
    parser.add_argument("--allow-nav-running", action="store_true", default=False)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    runner = CabinetStreamAdjustTest(args)
    try:
        return runner.run()
    except KeyboardInterrupt:
        runner.state = "stopped"
        runner.result = {"message": "stopped"}
        runner.update_status()
        return 130
    except Exception as exc:
        runner.state = "failed"
        runner.last_error = str(exc)
        runner.result = {"message": str(exc)}
        runner.update_status()
        print(json.dumps({"status": "failed", "message": str(exc), "status_file": str(runner.status_file)}, ensure_ascii=False), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

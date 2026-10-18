#!/usr/bin/env python3
from __future__ import annotations

import argparse
from concurrent.futures import Future, ThreadPoolExecutor
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable
from uuid import uuid4

try:
    import termios
    import tty
except Exception:  # pragma: no cover - Linux-only interactive calibration
    termios = None
    tty = None

try:
    import requests
except Exception:  # pragma: no cover - board runtime dependency
    requests = None

try:
    import yaml
except Exception:  # pragma: no cover - board runtime dependency
    yaml = None

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from std_msgs.msg import String as RosString
    from geometry_msgs.msg import Twist
except Exception:  # pragma: no cover - board runtime dependency
    rclpy = None
    Context = None
    Node = object
    RosString = None
    Twist = None


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = ROOT_DIR / "cabinet_inspection_profiles" / "default.yaml"
CAMERA_SCRIPT = ROOT_DIR / "ros2_tools" / "camera_yolo_detect.py"
SERVO_COUNT = 6
SERVO_MIN_PWM = 500
SERVO_MAX_PWM = 2500


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def clamp_int(value: int, low: int, high: int) -> int:
    return int(max(low, min(high, int(value))))


def print_json(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, ensure_ascii=False, indent=2), flush=True)


def load_profile(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise RuntimeError(f"profile not found: {path}")
    if yaml is None:
        raise RuntimeError("PyYAML is required to read cabinet inspection profiles")
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError(f"profile is not a YAML object: {path}")
    return payload


def save_profile(path: Path, payload: dict[str, Any]) -> None:
    if yaml is None:
        raise RuntimeError("PyYAML is required to write cabinet inspection profiles")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, allow_unicode=False, sort_keys=False), encoding="utf-8")


def parse_pwm_list(text: str) -> list[int]:
    values = [int(part.strip()) for part in str(text).split(",") if part.strip()]
    if len(values) != SERVO_COUNT:
        raise ValueError(f"expected {SERVO_COUNT} PWM values, got {len(values)}")
    return [clamp_int(value, SERVO_MIN_PWM, SERVO_MAX_PWM) for value in values]


def normalize_pwm(values: Iterable[Any], *, low: int, high: int) -> list[int]:
    pwm = [clamp_int(int(value), low, high) for value in list(values)[:SERVO_COUNT]]
    if len(pwm) < SERVO_COUNT:
        pwm.extend([1500] * (SERVO_COUNT - len(pwm)))
    return pwm


def configured_stage_names(profile: dict[str, Any], *, enabled_only: bool = True) -> list[str]:
    stages = profile.get("stages", {})
    if not isinstance(stages, dict):
        return []
    names: list[str] = []
    for name, cfg in stages.items():
        stage_name = str(name).strip()
        if not stage_name:
            continue
        if enabled_only and isinstance(cfg, dict) and not bool(cfg.get("enabled", True)):
            continue
        names.append(stage_name)
    return names


def build_arm_payload(pwm: Iterable[int], duration_ms: int) -> str:
    duration_ms = clamp_int(duration_ms, 20, 5000)
    parts = []
    for index, value in enumerate(list(pwm)[:SERVO_COUNT]):
        parts.append(f"#{index:03d}P{clamp_int(value, SERVO_MIN_PWM, SERVO_MAX_PWM):04d}T{duration_ms:04d}!")
    return "{" + "".join(parts) + "}"


def detection_label(item: Any) -> str:
    if not isinstance(item, dict):
        return ""
    raw = str(item.get("class_name") or item.get("label") or item.get("display_name") or "").strip().lower()
    if ":" in raw:
        raw = raw.rsplit(":", 1)[-1].strip()
    return raw


def detection_box(item: Any) -> list[float] | None:
    if not isinstance(item, dict):
        return None
    box = item.get("box_xyxy") or item.get("bbox") or item.get("box")
    if not isinstance(box, (list, tuple)) or len(box) != 4:
        return None
    try:
        x1, y1, x2, y2 = [float(value) for value in box]
    except (TypeError, ValueError):
        return None
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    return [x1, y1, x2, y2]


def union_bbox(items: list[dict[str, Any]]) -> list[float] | None:
    boxes = [box for box in (detection_box(item) for item in items) if box is not None]
    if not boxes:
        return None
    return [
        min(box[0] for box in boxes),
        min(box[1] for box in boxes),
        max(box[2] for box in boxes),
        max(box[3] for box in boxes),
    ]


def selected_target_bbox(result: dict[str, Any], stage_cfg: dict[str, Any]) -> tuple[list[float] | None, list[dict[str, Any]]]:
    labels = {str(item).strip().lower() for item in stage_cfg.get("target_labels", []) if str(item).strip()}
    selected: list[dict[str, Any]] = []
    if "__digits__" in labels:
        digits = result.get("digit_detections", [])
        if isinstance(digits, list):
            selected.extend(item for item in digits if isinstance(item, dict))
    detections = result.get("detections", [])
    if isinstance(detections, list):
        for item in detections:
            if isinstance(item, dict) and detection_label(item) in labels:
                selected.append(item)
    return union_bbox(selected), selected


def bbox_metrics(box: list[float], width: int, height: int, desired_center: tuple[float, float]) -> dict[str, float]:
    x1, y1, x2, y2 = box
    cx = ((x1 + x2) * 0.5) / max(1.0, float(width))
    cy = ((y1 + y2) * 0.5) / max(1.0, float(height))
    area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_ratio = area / max(1.0, float(width * height))
    return {
        "center_x": round(cx, 4),
        "center_y": round(cy, 4),
        "error_x": round(cx - desired_center[0], 4),
        "error_y": round(cy - desired_center[1], 4),
        "area_ratio": round(area_ratio, 6),
    }


def parse_float_text(text: str) -> float | None:
    clean = "".join(ch for ch in str(text) if ch.isdigit() or ch in ".-")
    if not clean or clean in {".", "-", "-."}:
        return None
    try:
        return float(clean)
    except ValueError:
        return None


def deep_merge_dict(base: dict[str, Any], update: dict[str, Any]) -> dict[str, Any]:
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge_dict(base[key], value)
        else:
            base[key] = value
    return base


def cabinet_update_from_result(stage: str, result: dict[str, Any]) -> dict[str, Any]:
    update: dict[str, Any] = {}
    cabinet_data = result.get("cabinet_data", {})
    if isinstance(cabinet_data, dict):
        update.update(cabinet_data)
    if stage in {"digital_display", "indicator_digit"}:
        readings = result.get("digit_readings", {})
        rows = readings.get("rows", []) if isinstance(readings, dict) else []
        texts = [str(row.get("text") or "").strip() for row in rows if isinstance(row, dict) and row.get("text")]
        global_text = str(readings.get("global_text") or "").strip() if isinstance(readings, dict) else ""
        if not texts and global_text:
            texts = [global_text]
        if texts:
            controller: dict[str, Any] = {
                "display_text": "/".join(texts),
                "status": "ok",
                "captured_at": utc_now_iso(),
            }
            first = parse_float_text(texts[0])
            if first is not None:
                controller["pv"] = first
            if len(texts) > 1:
                second = parse_float_text(texts[1])
                if second is not None:
                    controller["sv"] = second
            update["temperature_controller"] = controller
    if update:
        update["updated_at"] = utc_now_iso()
    return update


class MotionNode(Node):
    def __init__(self, arm_topic: str, cmd_vel_topic: str, context: Context | None = None):
        super().__init__("cabinet_inspection_runner", context=context)
        self.arm_topic = arm_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.arm_pub = self.create_publisher(RosString, arm_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

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

    def publish_arm(self, pwm: list[int], duration_ms: int) -> str:
        if self.wait_subscribers(self.arm_topic, 5.0) <= 0:
            raise RuntimeError(f"no subscriber on {self.arm_topic}")
        payload = build_arm_payload(pwm, duration_ms)
        msg = RosString()
        msg.data = payload
        self.arm_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)
        return payload

    def stop_chassis(self) -> None:
        if self.wait_subscribers(self.cmd_vel_topic, 0.5) <= 0:
            return
        msg = Twist()
        self.cmd_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.02)

    def pulse_chassis(self, linear_x: float, linear_y: float, angular_z: float, duration_s: float) -> None:
        if self.wait_subscribers(self.cmd_vel_topic, 2.0) <= 0:
            raise RuntimeError(f"no subscriber on {self.cmd_vel_topic}")
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        deadline = time.time() + max(0.02, float(duration_s))
        while time.time() < deadline:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)
        self.stop_chassis()


@dataclass
class StageResult:
    stage: str
    label: str
    status: str
    message: str
    recognition: dict[str, Any]
    cabinet_update: dict[str, Any]
    adjustment: dict[str, Any]
    upload_future: Future | None = None


class CabinetInspectionRunner:
    def __init__(self, args: argparse.Namespace, profile: dict[str, Any]):
        self.args = args
        self.profile = profile
        self.profile_path = Path(args.profile)
        self.inspection_id = args.inspection_id or f"insp_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{uuid4().hex[:6]}"
        self.board_id = args.board_id or os.environ.get("CAR_CLOUD_BOARD_ID", "rk3588-f103-board")
        self.board_label = args.board_label or os.environ.get("CAR_CLOUD_BOARD_LABEL", "RK3588 F103 Board")
        self.server_url = (args.server_url or os.environ.get("CAR_CLOUD_SERVER_URL", "")).rstrip("/")
        self.upload_token = args.upload_token or os.environ.get("CAR_CLOUD_UPLOAD_TOKEN", "")
        self.servo_min = int(profile.get("servo_min_pwm", SERVO_MIN_PWM) or SERVO_MIN_PWM)
        self.servo_max = int(profile.get("servo_max_pwm", SERVO_MAX_PWM) or SERVO_MAX_PWM)
        self.home_pwm = normalize_pwm(profile.get("home_pwm", [1500] * SERVO_COUNT), low=self.servo_min, high=self.servo_max)
        self.current_pwm = list(self.home_pwm)
        self.logs: list[str] = []
        self.node: MotionNode | None = None
        self.total_linear_m = 0.0
        self.total_yaw_rad = 0.0
        self.http = requests.Session() if requests is not None else None
        self.upload_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="inspection-upload")

    def log(self, message: str) -> None:
        text = f"{utc_now_iso()} {message}"
        self.logs.append(text)
        print(f"[cabinet-inspection] {message}", file=sys.stderr, flush=True)

    def ensure_runtime(self) -> None:
        if rclpy is None or RosString is None or Twist is None:
            raise RuntimeError("ROS2 Python packages are unavailable; source ROS2 before running")
        if not CAMERA_SCRIPT.exists():
            raise RuntimeError(f"camera script not found: {CAMERA_SCRIPT}")
        if not bool(self.profile.get("calibrated", False)) and not self.args.allow_uncalibrated:
            raise RuntimeError(
                f"profile is not calibrated: {self.profile_path}. "
                "Run calibration first or pass --allow-uncalibrated for bench testing."
            )
        if requests is None:
            raise RuntimeError("requests is required for cloud uploads")
        if not self.server_url:
            raise RuntimeError("missing server URL")
        if not self.upload_token:
            raise RuntimeError("missing upload token")

    def start_ros(self) -> None:
        rclpy.init(args=None)
        arm_topic = str(self.profile.get("arm_topic") or "/arm_cmd")
        cmd_topic = str(self.profile.get("cmd_vel_topic") or "/cmd_vel_cmd")
        self.node = MotionNode(arm_topic, cmd_topic)

    def destroy_ros(self) -> None:
        if self.node is not None:
            try:
                self.node.stop_chassis()
            finally:
                self.node.destroy_node()
                self.node = None
        if rclpy is not None:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def move_arm(self, pwm: list[int], duration_ms: int) -> str:
        if self.node is None:
            raise RuntimeError("ROS node is not initialized")
        self.current_pwm = normalize_pwm(pwm, low=self.servo_min, high=self.servo_max)
        payload = self.node.publish_arm(self.current_pwm, duration_ms)
        time.sleep(max(0.1, duration_ms / 1000.0))
        return payload

    def run_camera(self, stage: str, stage_cfg: dict[str, Any], *, preview: bool, iteration: int) -> dict[str, Any]:
        camera = self.profile.get("camera", {}) if isinstance(self.profile.get("camera"), dict) else {}
        width_key = "preview_width" if preview else "final_width"
        height_key = "preview_height" if preview else "final_height"
        width = int(stage_cfg.get(width_key) or camera.get(width_key) or (1280 if preview else 3264))
        height = int(stage_cfg.get(height_key) or camera.get(height_key) or (720 if preview else 2448))
        prefix = f"{self.inspection_id}_{stage}_{'preview' if preview else 'final'}_{iteration}"
        cmd = [
            "python3",
            str(CAMERA_SCRIPT),
            "--device",
            str(stage_cfg.get("device") or camera.get("device") or "/dev/video0"),
            "--prefix",
            prefix,
            "--width",
            str(width),
            "--height",
            str(height),
            "--fps",
            str(int(stage_cfg.get("fps") or camera.get("fps") or 15)),
            "--rotate",
            str(int(stage_cfg.get("rotate") or camera.get("rotate") or 180)),
            "--conf",
            f"{float(stage_cfg.get('conf', self.args.conf)):.3f}",
            "--backend",
            str(stage_cfg.get("backend") or "auto"),
            "--camera-profile",
            str(stage_cfg.get("camera_profile") or camera.get("profile") or "normal"),
        ]
        if bool(stage_cfg.get("no_digits", False)):
            cmd.append("--no-digits")
        if bool(stage_cfg.get("no_meter_readings", False)):
            cmd.append("--no-meter-readings")
        timeout_s = float(stage_cfg.get("timeout_s") or (45 if preview else 90))
        completed = subprocess.run(
            cmd,
            cwd=str(ROOT_DIR),
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                "camera recognition failed: "
                + (completed.stderr or completed.stdout or f"returncode={completed.returncode}")[-1800:]
            )
        payload = json.loads(completed.stdout)
        if not isinstance(payload, dict):
            raise RuntimeError("camera recognition output is not an object")
        payload["_capture_width"] = width
        payload["_capture_height"] = height
        payload["_stderr"] = completed.stderr[-1000:] if completed.stderr else ""
        return payload

    def maybe_adjust_arm(
        self,
        stage_cfg: dict[str, Any],
        metrics: dict[str, float],
        rough_pwm: list[int],
        adjustment: dict[str, Any],
    ) -> bool:
        loop_cfg = self.profile.get("closed_loop", {}) if isinstance(self.profile.get("closed_loop"), dict) else {}
        if not bool(stage_cfg.get("closed_loop_enabled", loop_cfg.get("enabled", True))):
            return False
        pan_servo = int(stage_cfg.get("pan_servo", loop_cfg.get("pan_servo", 0)))
        tilt_servo = int(stage_cfg.get("tilt_servo", loop_cfg.get("tilt_servo", 3)))
        max_step = abs(int(stage_cfg.get("max_step_pwm", loop_cfg.get("max_step_pwm", 60))))
        max_total = abs(int(stage_cfg.get("max_total_pwm_delta", loop_cfg.get("max_total_pwm_delta", 180))))
        pan_gain = float(stage_cfg.get("pan_pwm_per_norm_error", loop_cfg.get("pan_pwm_per_norm_error", -120)))
        tilt_gain = float(stage_cfg.get("tilt_pwm_per_norm_error", loop_cfg.get("tilt_pwm_per_norm_error", 120)))
        if not (0 <= pan_servo < SERVO_COUNT and 0 <= tilt_servo < SERVO_COUNT):
            return False
        pan_delta = clamp_int(round(metrics["error_x"] * pan_gain), -max_step, max_step)
        tilt_delta = clamp_int(round(metrics["error_y"] * tilt_gain), -max_step, max_step)
        next_pwm = list(self.current_pwm)
        for servo, delta in ((pan_servo, pan_delta), (tilt_servo, tilt_delta)):
            target = next_pwm[servo] + delta
            low = max(self.servo_min, rough_pwm[servo] - max_total)
            high = min(self.servo_max, rough_pwm[servo] + max_total)
            next_pwm[servo] = clamp_int(target, low, high)
        if next_pwm == self.current_pwm:
            return False
        payload = self.move_arm(next_pwm, int(stage_cfg.get("adjust_duration_ms", 450)))
        adjustment.setdefault("arm_steps", []).append(
            {
                "pwm": list(next_pwm),
                "payload": payload,
                "metrics": dict(metrics),
            }
        )
        return True

    def maybe_adjust_chassis(self, metrics: dict[str, float], adjustment: dict[str, Any]) -> bool:
        cfg = self.profile.get("chassis_adjust", {}) if isinstance(self.profile.get("chassis_adjust"), dict) else {}
        if not bool(cfg.get("enabled", True)) or self.args.no_chassis_adjust:
            return False
        max_linear = abs(float(cfg.get("max_linear", 0.025)))
        max_angular = abs(float(cfg.get("max_angular", 0.12)))
        pulse_s = clamp(float(cfg.get("pulse_s", 0.25)), 0.05, 0.30)
        max_total_linear = abs(float(cfg.get("max_total_linear_m", 0.20)))
        max_total_yaw = abs(float(cfg.get("max_total_yaw_rad", 0.18)))
        y = clamp(metrics["error_x"] * float(cfg.get("y_gain", -0.018)), -max_linear, max_linear)
        wz = clamp(metrics["error_x"] * float(cfg.get("yaw_gain", -0.08)), -max_angular, max_angular)
        x = 0.0
        area = float(metrics.get("area_ratio", 0.0))
        if area > 0.0 and area < float(cfg.get("min_area_ratio", 0.06)):
            x = abs(float(cfg.get("x_gain", 0.015)))
        elif area > float(cfg.get("max_area_ratio", 0.45)):
            x = -abs(float(cfg.get("x_gain", 0.015)))
        x = clamp(x, -max_linear, max_linear)
        if abs(x) < 1e-4 and abs(y) < 1e-4 and abs(wz) < 1e-4:
            return False
        linear_delta = math.hypot(x, y) * pulse_s
        yaw_delta = abs(wz) * pulse_s
        if self.total_linear_m + linear_delta > max_total_linear:
            x = 0.0
            y = 0.0
        if self.total_yaw_rad + yaw_delta > max_total_yaw:
            wz = 0.0
        if abs(x) < 1e-4 and abs(y) < 1e-4 and abs(wz) < 1e-4:
            return False
        if self.node is None:
            raise RuntimeError("ROS node is not initialized")
        self.node.pulse_chassis(x, y, wz, pulse_s)
        self.total_linear_m += math.hypot(x, y) * pulse_s
        self.total_yaw_rad += abs(wz) * pulse_s
        adjustment.setdefault("chassis_steps", []).append(
            {
                "linear_x": round(x, 5),
                "linear_y": round(y, 5),
                "angular_z": round(wz, 5),
                "duration_s": round(pulse_s, 3),
                "metrics": dict(metrics),
            }
        )
        time.sleep(0.5)
        return True

    def align_stage(self, stage: str, stage_cfg: dict[str, Any], rough_pwm: list[int]) -> dict[str, Any]:
        loop_cfg = self.profile.get("closed_loop", {}) if isinstance(self.profile.get("closed_loop"), dict) else {}
        if not bool(stage_cfg.get("closed_loop_enabled", loop_cfg.get("enabled", True))):
            return {
                "target_found": False,
                "centered": False,
                "preview_iterations": 0,
                "arm_steps": [],
                "chassis_steps": [],
                "last_message": "fixed pose capture",
            }
        max_iterations = int(stage_cfg.get("max_iterations", loop_cfg.get("max_iterations", 4)))
        tolerance = float(stage_cfg.get("center_tolerance", loop_cfg.get("center_tolerance", 0.10)))
        desired_raw = stage_cfg.get("desired_center", loop_cfg.get("desired_center", [0.5, 0.5]))
        desired = (float(desired_raw[0]), float(desired_raw[1])) if isinstance(desired_raw, list) and len(desired_raw) >= 2 else (0.5, 0.5)
        adjustment: dict[str, Any] = {
            "target_found": False,
            "centered": False,
            "preview_iterations": 0,
            "arm_steps": [],
            "chassis_steps": [],
        }
        for iteration in range(1, max(0, max_iterations) + 1):
            preview = self.run_camera(stage, stage_cfg, preview=True, iteration=iteration)
            box, selected = selected_target_bbox(preview, stage_cfg)
            adjustment["preview_iterations"] = iteration
            adjustment["selected_count"] = len(selected)
            if box is None:
                adjustment["last_message"] = "target not detected in preview"
                break
            metrics = bbox_metrics(box, int(preview["_capture_width"]), int(preview["_capture_height"]), desired)
            adjustment["target_found"] = True
            adjustment["last_bbox"] = [round(value, 2) for value in box]
            adjustment["last_metrics"] = metrics
            if abs(metrics["error_x"]) <= tolerance and abs(metrics["error_y"]) <= tolerance:
                adjustment["centered"] = True
                adjustment["last_message"] = "target centered"
                break
            if self.maybe_adjust_arm(stage_cfg, metrics, rough_pwm, adjustment):
                continue
            self.maybe_adjust_chassis(metrics, adjustment)
        return adjustment

    def upload_recognition(self, result: dict[str, Any]) -> dict[str, Any]:
        raw_path = Path(str(result.get("source_image") or ""))
        det_path = Path(str(result.get("annotated_image") or ""))
        if not raw_path.exists() or not det_path.exists():
            raise RuntimeError("recognition output images are missing")
        url = f"{self.server_url}/api/upload/recognition"
        headers = {"X-Upload-Token": self.upload_token}
        if self.http is None:
            raise RuntimeError("HTTP session is unavailable")
        started = time.perf_counter()
        last_error: Exception | None = None
        for attempt, delay_s in enumerate((0.0, 0.2, 0.5, 1.0), start=1):
            if delay_s:
                time.sleep(delay_s)
            try:
                with raw_path.open("rb") as raw_handle, det_path.open("rb") as det_handle:
                    response = self.http.post(
                        url,
                        headers=headers,
                        data={"payload": json.dumps(result, ensure_ascii=False)},
                        files={
                            "raw_image": (raw_path.name, raw_handle, "image/jpeg"),
                            "annotated_image": (det_path.name, det_handle, "image/jpeg"),
                        },
                        timeout=45,
                    )
                response.raise_for_status()
                payload = response.json()
                timings = result.setdefault("timings_ms", {})
                if isinstance(timings, dict):
                    timings["upload"] = round((time.perf_counter() - started) * 1000.0, 3)
                    timings["upload_attempts"] = attempt
                return payload.get("recognition", {}) if isinstance(payload, dict) else {}
            except Exception as exc:
                last_error = exc
        raise RuntimeError(f"recognition upload failed after retries: {last_error}")

    def upload_inspection(self, summary: dict[str, Any]) -> dict[str, Any]:
        if self.http is None:
            raise RuntimeError("HTTP session is unavailable")
        last_error: Exception | None = None
        for delay_s in (0.0, 0.2, 0.5, 1.0):
            if delay_s:
                time.sleep(delay_s)
            try:
                response = self.http.post(
                    f"{self.server_url}/api/upload/inspection",
                    headers={"X-Upload-Token": self.upload_token},
                    json=summary,
                    timeout=30,
                )
                response.raise_for_status()
                payload = response.json()
                return payload if isinstance(payload, dict) else {}
            except Exception as exc:
                last_error = exc
        raise RuntimeError(f"inspection upload failed after retries: {last_error}")

    def inspect_stage(self, stage: str, stage_cfg: dict[str, Any]) -> StageResult:
        label = str(stage_cfg.get("label") or stage)
        rough_pwm = normalize_pwm(stage_cfg.get("rough_pwm", self.home_pwm), low=self.servo_min, high=self.servo_max)
        self.log(f"stage {stage}: move arm to rough pose")
        self.move_arm(rough_pwm, int(stage_cfg.get("rough_duration_ms", 1200)))
        time.sleep(float(stage_cfg.get("settle_s", self.profile.get("closed_loop", {}).get("settle_s", 1.0))))
        adjustment = self.align_stage(stage, stage_cfg, rough_pwm)
        self.log(f"stage {stage}: final capture")
        final_result = self.run_camera(stage, stage_cfg, preview=False, iteration=0)
        final_result["inspection_id"] = self.inspection_id
        final_result["inspection_stage"] = stage
        final_result["inspection_stage_label"] = label
        final_result["board_id"] = self.board_id
        final_result["board_label"] = self.board_label
        final_result["adjustment"] = adjustment
        final_result["capture_key"] = f"{self.inspection_id}:{stage}"
        cabinet_update = cabinet_update_from_result(stage, final_result)
        if cabinet_update:
            existing = final_result.get("cabinet_data", {})
            if not isinstance(existing, dict):
                existing = {}
            existing.update(cabinet_update)
            final_result["cabinet_data"] = existing
        upload_future = self.upload_pool.submit(self.upload_recognition, final_result)
        final_has_data = bool(final_result.get("detections") or final_result.get("meter_readings"))
        digit_readings = final_result.get("digit_readings", {})
        if isinstance(digit_readings, dict):
            final_has_data = final_has_data or bool(digit_readings.get("global_text") or digit_readings.get("rows"))
        status = "completed" if adjustment.get("target_found") or final_has_data or stage in {"digital_display", "indicator_digit"} else "unreadable"
        message = str(adjustment.get("last_message") or "stage captured")
        return StageResult(
            stage=stage,
            label=label,
            status=status,
            message=message,
            recognition={},
            cabinet_update=cabinet_update,
            adjustment=adjustment,
            upload_future=upload_future,
        )

    def run(self) -> dict[str, Any]:
        self.ensure_runtime()
        stages_cfg = self.profile.get("stages", {})
        if not isinstance(stages_cfg, dict):
            raise RuntimeError("profile.stages must be an object")
        requested = self.args.stages or configured_stage_names(self.profile)
        if not requested:
            raise RuntimeError("no enabled inspection stages configured in profile")
        self.start_ros()
        started_at = utc_now_iso()
        stage_results: list[StageResult] = []
        cabinet_update: dict[str, Any] = {}
        try:
            for stage in requested:
                if stage not in stages_cfg:
                    raise RuntimeError(f"stage not configured: {stage}")
                stage_cfg = stages_cfg[stage]
                if not isinstance(stage_cfg, dict) or not bool(stage_cfg.get("enabled", True)):
                    continue
                if not bool(stage_cfg.get("calibrated", False)) and not self.args.allow_uncalibrated:
                    raise RuntimeError(f"stage is not calibrated: {stage}")
                try:
                    result = self.inspect_stage(stage, stage_cfg)
                except Exception as exc:
                    self.log(f"stage {stage} failed: {exc}")
                    result = StageResult(
                        stage=stage,
                        label=stage,
                        status="failed",
                        message=str(exc),
                        recognition={},
                        cabinet_update={},
                        adjustment={},
                    )
                    if self.args.abort_on_stage_failure:
                        stage_results.append(result)
                        break
                stage_results.append(result)
                for key, value in result.cabinet_update.items():
                    if isinstance(value, dict) and isinstance(cabinet_update.get(key), dict):
                        deep_merge_dict(cabinet_update[key], value)
                    else:
                        cabinet_update[key] = value
            try:
                self.move_arm(self.home_pwm, 1200)
            except Exception as exc:
                self.log(f"return home failed: {exc}")
        finally:
            self.destroy_ros()
        for item in stage_results:
            if item.upload_future is None:
                continue
            try:
                item.recognition = item.upload_future.result(timeout=90)
                recognition_update = (
                    item.recognition.get("cabinet_update", {})
                    if isinstance(item.recognition, dict)
                    else {}
                )
                if isinstance(recognition_update, dict) and recognition_update:
                    deep_merge_dict(item.cabinet_update, recognition_update)
            except Exception as exc:
                item.status = "failed"
                item.message = f"upload failed: {exc}"
                self.log(f"stage {item.stage} upload failed: {exc}")
        self.upload_pool.shutdown(wait=True)
        completed_at = utc_now_iso()
        status = "completed" if stage_results and all(item.status in {"completed", "unreadable"} for item in stage_results) else "failed"
        if stage_results and any(item.status == "completed" for item in stage_results):
            status = "completed"
        summary = {
            "inspection_id": self.inspection_id,
            "status": status,
            "profile": str(self.profile.get("profile_name") or self.profile_path.stem),
            "board_id": self.board_id,
            "board_label": self.board_label,
            "started_at": started_at,
            "completed_at": completed_at,
            "stages": [
                {
                    "stage": item.stage,
                    "label": item.label,
                    "status": item.status,
                    "message": item.message,
                    "recognition": item.recognition,
                    "cabinet_update": item.cabinet_update,
                    "adjustment": item.adjustment,
                }
                for item in stage_results
            ],
            "cabinet_update": cabinet_update,
            "logs": self.logs[-80:],
        }
        try:
            uploaded = self.upload_inspection(summary)
            if isinstance(uploaded, dict):
                summary["cloud_upload"] = uploaded.get("inspection", uploaded)
        except Exception as exc:
            summary["upload_error"] = str(exc)
        if self.http is not None:
            self.http.close()
        return summary


def active_motion_processes() -> list[str]:
    patterns = (
        "rt_robot_nav2_complete.launch.py",
        "start_board_f103_mapping.sh",
        "start_board_f103_navigation.sh",
        "slam_toolbox",
        "slam_gmapping",
    )
    try:
        completed = subprocess.run(["pgrep", "-af", "|".join(patterns)], capture_output=True, text=True, check=False)
    except Exception:
        return []
    lines = []
    for line in completed.stdout.splitlines():
        if "cabinet_inspection_runner.py" in line:
            continue
        lines.append(line.strip())
    return [line for line in lines if line]


def active_motion_tasks(processes: list[str]) -> list[str]:
    blockers: list[str] = []
    runtime_dir = ROOT_DIR / "runtime"
    for label, path in (
        ("navigation", runtime_dir / "cloud_nav_command" / "status.json"),
        ("mapping", runtime_dir / "cloud_mapping" / "session.json"),
    ):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            payload = {}
        if not isinstance(payload, dict):
            continue
        if label == "navigation" and (
            payload.get("task_running")
            or payload.get("goal_active")
            or str(payload.get("state") or "") in {"starting", "planning", "navigating", "stalled"}
        ):
            blockers.append("active navigation task")
        if label == "mapping" and payload.get("active"):
            blockers.append("active mapping session")

    mapping_tokens = ("use_slam:=true", "start_board_f103_mapping.sh", "slam_toolbox", "slam_gmapping")
    if any(any(token in process for token in mapping_tokens) for process in processes):
        blockers.append("active mapping process")
    return blockers


def calibrate_stage(args: argparse.Namespace, profile: dict[str, Any]) -> int:
    stage = args.calibrate_stage
    stages = profile.setdefault("stages", {})
    stage_cfg = stages.setdefault(stage, {})
    servo_min = int(profile.get("servo_min_pwm", SERVO_MIN_PWM) or SERVO_MIN_PWM)
    servo_max = int(profile.get("servo_max_pwm", SERVO_MAX_PWM) or SERVO_MAX_PWM)
    current = normalize_pwm(stage_cfg.get("rough_pwm", profile.get("home_pwm", [1500] * SERVO_COUNT)), low=servo_min, high=servo_max)
    if args.save_pwm:
        stage_cfg["rough_pwm"] = parse_pwm_list(args.save_pwm)
        stage_cfg["enabled"] = True
        stage_cfg["calibrated"] = True
        stages[stage] = stage_cfg
        stage_names = configured_stage_names(profile, enabled_only=False)
        if stage_names and all(isinstance(stages.get(name), dict) and stages[name].get("calibrated") for name in stage_names):
            profile["calibrated"] = True
        save_profile(Path(args.profile), profile)
        print_json({"ok": True, "message": "stage PWM saved", "stage": stage, "rough_pwm": stage_cfg["rough_pwm"]})
        return 0
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are unavailable; source ROS2 before calibration")
    if termios is None or tty is None:
        raise RuntimeError("interactive calibration requires a Linux TTY")
    rclpy.init(args=None)
    node = MotionNode(
        str(profile.get("arm_topic") or "/arm_cmd"),
        str(profile.get("cmd_vel_topic") or "/cmd_vel_cmd"),
    )
    selected = 0
    step = int(args.step_pwm)
    print("Calibration keys: 1-6 select, j/k decrease/increase, [/]=step, h home, s save, q quit")
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            print(f"\rstage={stage} joint={selected + 1} step={step} pwm={current}    ", end="", flush=True)
            ch = sys.stdin.read(1)
            if ch in "123456":
                selected = int(ch) - 1
            elif ch == "j":
                current[selected] = clamp_int(current[selected] - step, servo_min, servo_max)
                node.publish_arm(current, 350)
            elif ch == "k":
                current[selected] = clamp_int(current[selected] + step, servo_min, servo_max)
                node.publish_arm(current, 350)
            elif ch == "[":
                step = max(1, step // 2)
            elif ch == "]":
                step = min(200, step * 2)
            elif ch == "h":
                current = normalize_pwm(profile.get("home_pwm", current), low=servo_min, high=servo_max)
                node.publish_arm(current, 800)
            elif ch == "s":
                stage_cfg["rough_pwm"] = list(current)
                stage_cfg["enabled"] = True
                stage_cfg["calibrated"] = True
                stages[stage] = stage_cfg
                stage_names = configured_stage_names(profile, enabled_only=False)
                if stage_names and all(isinstance(stages.get(name), dict) and stages[name].get("calibrated") for name in stage_names):
                    profile["calibrated"] = True
                save_profile(Path(args.profile), profile)
                print(f"\nsaved {stage}: {current}")
            elif ch == "q":
                print("")
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Cabinet photo inspection with arm/camera/chassis micro-adjust.")
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE), help="inspection YAML profile")
    parser.add_argument("--server-url", default="", help="cloud server URL; defaults to CAR_CLOUD_SERVER_URL")
    parser.add_argument("--upload-token", default="", help="cloud upload token; defaults to CAR_CLOUD_UPLOAD_TOKEN")
    parser.add_argument("--board-id", default="", help="board id")
    parser.add_argument("--board-label", default="", help="board label")
    parser.add_argument("--inspection-id", default="", help="optional inspection id")
    parser.add_argument("--stages", nargs="*", help="profile stage names to run; defaults to enabled stages in profile order")
    parser.add_argument("--conf", type=float, default=0.25, help="fallback confidence threshold")
    parser.add_argument("--allow-uncalibrated", action="store_true", help="allow running with calibrated=false")
    parser.add_argument("--no-chassis-adjust", action="store_true", help="disable chassis micro-adjust")
    parser.add_argument("--abort-on-stage-failure", action="store_true", help="stop after first failed stage")
    parser.add_argument("--calibrate-stage", help="interactive calibration mode for one profile stage")
    parser.add_argument("--save-pwm", default="", help="non-interactive calibration PWM list, e.g. 2300,1450,1500,2200,1500,1700")
    parser.add_argument("--step-pwm", type=int, default=20, help="interactive calibration step")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        profile = load_profile(Path(args.profile))
        if args.calibrate_stage:
            return calibrate_stage(args, profile)
        motion_processes = active_motion_processes()
        if motion_processes and os.environ.get("CAR_CLOUD_ALLOW_INSPECTION_WITH_ROS_MOTION") != "1":
            if args.no_chassis_adjust:
                blockers = active_motion_tasks(motion_processes)
                if blockers:
                    raise RuntimeError(f"cabinet inspection blocked: {', '.join(blockers)}")
            else:
                raise RuntimeError("mapping/navigation process is active; stop it before cabinet inspection")
        summary = CabinetInspectionRunner(args, profile).run()
        print_json(summary)
        return 0 if summary.get("status") == "completed" else 1
    except Exception as exc:
        print_json({"status": "failed", "message": str(exc), "inspection_id": getattr(args, "inspection_id", "")})
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

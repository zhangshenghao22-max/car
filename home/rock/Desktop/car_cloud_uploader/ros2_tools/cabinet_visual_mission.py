#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

import cv2
import requests
import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String as RosString
from std_srvs.srv import SetBool, Trigger

from cabinet_inspection_runner import build_arm_payload, normalize_pwm
from cabinet_visual_core import (
    InferencePerformanceMonitor,
    NavigationActivityTracker,
    TemporalInspectionConsensus,
    allows_post_navigation_visual_grace,
    docking_command,
    fit_cabinet_plane,
    trigger_evidence,
)
from cabinet_nav_takeover import (
    DEFAULT_TARGET_LABELS,
    StreamDetector,
    atomic_write_json,
    bbox_metrics,
    detection_box,
    detection_label,
    rotate_frame,
    selected_detections,
    target_bbox,
    utc_now_iso,
)
from cabinet_stream_adjust_test import (
    CabinetStreamAdjustTest,
    STAGES,
    build_parser as build_stream_parser,
    selected_detection_summary,
    status_light_items,
    status_light_slot_map,
)


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_STATUS_FILE = ROOT_DIR / "runtime" / "cabinet_nav_takeover" / "status.json"
DEFAULT_CAPTURE_DIR = ROOT_DIR / "runtime" / "cabinet_nav_takeover" / "captures"
DEFAULT_OUTBOX_DIR = ROOT_DIR / "runtime" / "cabinet_nav_takeover" / "outbox"

DIGIT_LABELS = {str(value) for value in range(10)} | {"digit", "digits", "digital_display"}


class LatestFrameCamera:
    def __init__(self, capture: Any, rotate: int):
        self.capture = capture
        self.rotate = rotate
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.frame: Any = None
        self.frame_at = 0.0
        self.error = ""
        self.thread = threading.Thread(target=self._run, name="cabinet-camera", daemon=True)
        self.thread.start()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            ok, frame = self.capture.read()
            if not ok or frame is None:
                self.error = "camera read failed"
                time.sleep(0.05)
                continue
            rotated = rotate_frame(frame, self.rotate)
            with self.lock:
                self.frame = rotated
                self.frame_at = time.time()

    def read(self, timeout_s: float = 1.0) -> Any:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self.lock:
                if self.frame is not None and time.time() - self.frame_at <= 0.5:
                    return self.frame.copy()
            time.sleep(0.01)
        raise RuntimeError(self.error or "fresh camera frame unavailable")

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        self.capture.release()


class MissionNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("cabinet_visual_mission")
        self.args = args
        self.cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.arm_pub = self.create_publisher(RosString, args.arm_topic, 10)
        self.create_subscription(LaserScan, args.scan_topic, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            GoalStatusArray,
            args.nav_status_topic,
            self._on_nav_status,
            10,
        )
        self.create_subscription(
            RosString,
            args.nav_manager_status_topic,
            self._on_nav_manager_status,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.cancel_client = self.create_client(CancelGoal, args.nav_cancel_service)
        self.manager_cancel_client = self.create_client(
            Trigger,
            args.nav_manager_cancel_service,
        )
        self.lease_client = self.create_client(SetBool, args.cabinet_mode_service)
        self.last_scan_at = 0.0
        self.front_min = float("inf")
        self.plane: dict[str, Any] | None = None
        self.navigation = NavigationActivityTracker(
            active_at_launch=bool(args.navigation_active_at_launch)
        )
        self.last_nav_status_at = 0.0
        self.last_nav_manager_status_at = 0.0
        self.readiness: dict[str, Any] = {}

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        self.navigation.update_action(item.status for item in msg.status_list)
        self.last_nav_status_at = time.time()

    def _on_nav_manager_status(self, msg: RosString) -> None:
        self.navigation.update_manager(msg.data)
        self.last_nav_manager_status_at = time.time()

    @property
    def navigation_active(self) -> bool:
        return self.navigation.active

    @property
    def navigation_activity_known(self) -> bool:
        return self.navigation.known

    def _on_scan(self, msg: LaserScan) -> None:
        points: list[tuple[float, float]] = []
        front_values: list[float] = []
        half_angle = math.radians(float(self.args.plane_sector_deg))
        for index, raw in enumerate(msg.ranges):
            value = float(raw)
            if not math.isfinite(value) or value < max(msg.range_min, self.args.scan_valid_min_m):
                continue
            if value > min(msg.range_max, self.args.plane_max_range_m):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) > half_angle:
                continue
            points.append((value * math.cos(angle), value * math.sin(angle)))
            if abs(angle) <= math.radians(float(self.args.front_angle_deg)):
                front_values.append(value)
        self.front_min = min(front_values) if front_values else float("inf")
        self.plane = fit_cabinet_plane(
            points,
            iterations=int(self.args.plane_ransac_iterations),
            inlier_threshold=float(self.args.plane_inlier_threshold_m),
            min_inliers=int(self.args.plane_min_inliers),
            min_span=float(self.args.plane_min_span_m),
        )
        if self.plane is not None:
            offset = float(self.args.front_bumper_x_m) - float(self.args.laser_x_m)
            self.plane["front_clearance_m"] = round(
                float(self.plane["distance_m"]) - offset,
                4,
            )
        self.last_scan_at = time.time()

    def readiness_snapshot(self) -> dict[str, Any]:
        now = time.time()
        scan_age = None if self.last_scan_at <= 0.0 else max(0.0, now - self.last_scan_at)
        lease_ready = bool(
            self.lease_client.service_is_ready()
            if hasattr(self.lease_client, "service_is_ready")
            else self.lease_client.wait_for_service(timeout_sec=0.0)
        )
        manager_cancel_ready = bool(
            self.manager_cancel_client.service_is_ready()
            if hasattr(self.manager_cancel_client, "service_is_ready")
            else self.manager_cancel_client.wait_for_service(timeout_sec=0.0)
        )
        snapshot = {
            "cmd_vel_cabinet_subscribers": self.count_subscribers(self.args.cmd_vel_topic),
            "arm_cmd_subscribers": self.count_subscribers(self.args.arm_topic),
            "nav_status_publishers": self.count_publishers(self.args.nav_status_topic),
            "nav_manager_status_publishers": self.count_publishers(
                self.args.nav_manager_status_topic
            ),
            "nav_manager_cancel_service_ready": manager_cancel_ready,
            "cabinet_mode_service_ready": lease_ready,
            "scan_received": self.last_scan_at > 0.0,
            "scan_age_s": None if scan_age is None else round(scan_age, 3),
        }
        missing: list[str] = []
        if snapshot["cmd_vel_cabinet_subscribers"] <= 0:
            missing.append(self.args.cmd_vel_topic)
        if snapshot["arm_cmd_subscribers"] <= 0:
            missing.append(self.args.arm_topic)
        if (
            snapshot["nav_status_publishers"] <= 0
            and snapshot["nav_manager_status_publishers"] <= 0
        ):
            missing.append(
                f"{self.args.nav_status_topic}|{self.args.nav_manager_status_topic}"
            )
        if not self.args.standalone_inspection and not manager_cancel_ready:
            missing.append(self.args.nav_manager_cancel_service)
        if not lease_ready:
            missing.append(self.args.cabinet_mode_service)
        if scan_age is None or scan_age > float(self.args.max_scan_age_s):
            missing.append(self.args.scan_topic)
        snapshot["missing"] = missing
        snapshot["ready"] = not missing
        self.readiness = snapshot
        return snapshot

    def wait_ready(self, timeout_s: float, update_callback=None) -> None:
        deadline = time.time() + timeout_s
        last_update_at = 0.0
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            snapshot = self.readiness_snapshot()
            now = time.time()
            if update_callback is not None and now - last_update_at >= 1.0:
                update_callback(dict(snapshot))
                last_update_at = now
            if snapshot["ready"]:
                return
        snapshot = self.readiness_snapshot()
        raise RuntimeError(
            "cabinet ROS interfaces are not ready after "
            f"{timeout_s:.1f}s: {','.join(snapshot['missing'])}"
        )

    def publish_twist(self, linear_x: float, angular_z: float, linear_y: float = 0.0) -> dict[str, float]:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)
        return {
            "linear_x": round(float(linear_x), 5),
            "linear_y": round(float(linear_y), 5),
            "angular_z": round(float(angular_z), 5),
        }

    def stop(self, repeats: int = 5) -> None:
        for _ in range(max(1, repeats)):
            self.publish_twist(0.0, 0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.03)

    def publish_arm(self, pwm: list[int], duration_ms: int) -> str:
        payload = build_arm_payload(pwm, duration_ms)
        msg = RosString()
        msg.data = payload
        self.arm_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.03)
        return payload

    def set_cabinet_mode(self, enabled: bool, timeout_s: float = 3.0) -> bool:
        if not self.lease_client.wait_for_service(timeout_sec=timeout_s):
            return False
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self.lease_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        response = future.result() if future.done() else None
        return bool(response is not None and response.success)

    def cancel_navigation(self, timeout_s: float = 5.0) -> bool:
        if self.navigation_activity_known and not self.navigation_active:
            return True

        manager_request_succeeded = False
        if self.manager_cancel_client.wait_for_service(timeout_sec=timeout_s):
            future = self.manager_cancel_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            response = future.result() if future.done() else None
            manager_request_succeeded = bool(response is not None and response.success)
            if manager_request_succeeded and "no active" in str(response.message).lower():
                self.navigation.confirm_manager_inactive()

        if self.navigation.action_active:
            if not self.cancel_client.wait_for_service(timeout_sec=timeout_s):
                return False
            request = CancelGoal.Request()
            request.goal_info.goal_id.uuid = [0] * 16
            request.goal_info.stamp.sec = 0
            request.goal_info.stamp.nanosec = 0
            future = self.cancel_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            if not future.done() or future.result() is None:
                return False

        if not manager_request_succeeded and self.navigation.manager_active:
            return False
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if not self.navigation_active:
                return True
        return not self.navigation_active


class CabinetVisualMission(CabinetStreamAdjustTest):
    def __init__(self, args: argparse.Namespace):
        super().__init__(args)
        self.inspection_id = self.test_id.replace("stream_adjust_", "cabinet_")
        self.outbox_dir = Path(args.outbox_dir)
        self.camera_worker: LatestFrameCamera | None = None
        self.takeover_active = False
        self.control_was_acquired = False
        self.navigation_cancel_confirmed = False
        self.post_navigation_grace_active = False
        self.terminal_navigation_phase = ""
        self.skip_docking_due_to_navigation_failure = False
        self.failure_phase = ""
        self.failure_stage = ""
        self.docking_completed = False
        self.docking_degraded_reason = ""
        self._last_logged_status_signature: tuple[str, str, str] | None = None
        self.last_readiness: dict[str, Any] = {}
        self.trigger_window: deque[bool] = deque(maxlen=int(args.trigger_window_frames))
        self.last_inference_at = 0.0
        self.performance = InferencePerformanceMonitor(
            target_fps=float(args.target_inference_fps),
            hard_min_fps=float(args.min_inference_fps),
            hard_fail_hold_s=float(args.fps_hard_fail_hold_s),
            window_frames=int(args.fps_window_frames),
            warmup_frames=int(args.fps_warmup_frames),
        )
        self.performance_status = self.performance.snapshot()
        self.last_evidence: dict[str, Any] = {}
        self.last_plane: dict[str, Any] = {}
        self.consensus = TemporalInspectionConsensus(max_samples=int(args.consensus_max_samples))
        self.temporal_consensus: dict[str, Any] = {}
        self.monitor_pattern: list[int] = []
        self.monitor_pattern_index = 0
        self.monitor_detection_cache: dict[int, tuple[float, list[dict[str, Any]]]] = {}

    def update_status(self, **extra: Any) -> None:
        plane = dict(self.node.plane) if isinstance(self.node, MissionNode) and self.node.plane else {}
        if isinstance(self.node, MissionNode) and self.node.readiness:
            self.last_readiness = dict(self.node.readiness)
        readiness = dict(self.last_readiness)
        payload = {
            "state": self.state,
            "takeover_active": self.takeover_active,
            "control_was_acquired": self.control_was_acquired,
            "navigation_active": bool(
                isinstance(self.node, MissionNode) and self.node.navigation_active
            ),
            "navigation_manager_phase": (
                self.node.navigation.manager_phase
                if isinstance(self.node, MissionNode)
                else ""
            ),
            "navigation_cancel_confirmed": self.navigation_cancel_confirmed,
            "post_navigation_grace_active": self.post_navigation_grace_active,
            "terminal_navigation_phase": self.terminal_navigation_phase,
            "skip_docking_due_to_navigation_failure": (
                self.skip_docking_due_to_navigation_failure
            ),
            "inspection_id": self.inspection_id,
            "target_seen_count": self.target_seen_count,
            "trigger_window": list(self.trigger_window),
            "trigger_evidence": dict(self.last_evidence),
            "readiness": readiness,
            "inference_fps": round(float(self.performance_status.get("fps", 0.0)), 3),
            "performance_state": str(self.performance_status.get("state") or "warming_up"),
            "performance_warning": str(self.performance_status.get("warning") or ""),
            "fps_target": float(self.performance_status.get("target_fps", self.args.target_inference_fps)),
            "fps_hard_min": float(self.performance_status.get("hard_min_fps", self.args.min_inference_fps)),
            "fps_hard_low_duration_s": round(float(self.performance_status.get("hard_low_duration_s", 0.0)), 3),
            "inference_age_s": None if self.last_inference_at <= 0 else round(time.time() - self.last_inference_at, 3),
            "front_min": None if not isinstance(self.node, MissionNode) or not math.isfinite(self.node.front_min) else round(self.node.front_min, 3),
            "plane": plane,
            "last_detection": dict(self.last_detection),
            "last_cmd_vel": dict(self.last_cmd_vel),
            "current_pwm": list(self.current_pwm),
            "capture_progress": self.capture_progress(),
            "coverage": dict(self.coverage),
            "temporal_consensus": dict(self.temporal_consensus),
            "monitor_model_cache_age_s": {
                str(index): round(time.time() - value[0], 3)
                for index, value in self.monitor_detection_cache.items()
            },
            "missing_items": list(self.missing_items),
            "last_error": self.last_error,
            "failure_phase": self.failure_phase,
            "failure_stage": self.failure_stage,
            "docking_completed": self.docking_completed,
            "docking_degraded_reason": self.docking_degraded_reason,
            "result": dict(self.result),
            "logs": list(self.logs[-30:]),
            "pid": os.getpid(),
            "updated_at": utc_now_iso(),
        }
        payload.update(extra)
        atomic_write_json(self.status_file, payload)
        signature = (self.state, self.last_error, self.failure_stage)
        if signature != self._last_logged_status_signature:
            self.log(
                "mission status "
                + json.dumps(
                    {
                        "state": self.state,
                        "last_error": self.last_error,
                        "failure_stage": self.failure_stage,
                        "takeover_active": self.takeover_active,
                        "docking_completed": self.docking_completed,
                    },
                    ensure_ascii=False,
                    separators=(",", ":"),
                )
            )
            self._last_logged_status_signature = signature

    def ensure_runtime(self) -> None:
        if not Path(self.args.device).exists():
            raise RuntimeError(f"camera device unavailable: {self.args.device}")
        if self.args.backend != "rknn" or not self.args.require_rknn:
            raise RuntimeError("automatic cabinet takeover requires --backend rknn --require-rknn")

    def open_camera(self) -> None:
        super().open_camera()
        self.camera_worker = LatestFrameCamera(self.camera, int(self.args.rotate))

    def read_frame(self) -> Any:
        if self.camera_worker is None:
            raise RuntimeError("camera worker is not running")
        return self.camera_worker.read(timeout_s=1.0)

    def start_ros(self) -> None:
        rclpy.init(args=None)
        self.node = MissionNode(self.args)
        self.state = "WAIT_ROS_READY"
        self.update_status()
        self.node.wait_ready(
            float(self.args.ros_ready_timeout_s),
            update_callback=lambda readiness: self.update_status(readiness=readiness),
        )
        self.node.stop(repeats=5)
        self.log("cabinet mission ROS interfaces ready")

    def detect_fast(self, frame: Any) -> dict[str, Any]:
        if self.detector is None:
            raise RuntimeError("detector unavailable")
        if not self.monitor_pattern:
            raise RuntimeError("monitor model schedule is not configured")
        model_index = self.monitor_pattern[self.monitor_pattern_index % len(self.monitor_pattern)]
        self.monitor_pattern_index += 1
        result = self.detector.detect(
            frame,
            include_digits=False,
            model_indices={model_index},
        )
        now = time.time()
        current = [dict(item) for item in result.get("detections", []) if isinstance(item, dict)]
        self.monitor_detection_cache[model_index] = (now, current)
        combined: list[dict[str, Any]] = []
        cache_max_age = float(self.args.max_detection_age_s)
        for captured_at, detections in self.monitor_detection_cache.values():
            if now - captured_at <= cache_max_age:
                combined.extend(dict(item) for item in detections)
        result["detections"] = combined
        result["digit_detections"] = [
            dict(item) for item in combined if detection_label(item) in DIGIT_LABELS
        ]
        result["monitor_model_index"] = model_index
        self.performance_status = self.performance.observe(now)
        self.last_inference_at = now
        return result

    def configure_monitor_models(self) -> None:
        if self.detector is None:
            raise RuntimeError("detector unavailable")
        light_index = next(
            (
                index
                for index, model in enumerate(self.detector.models)
                if "lightlabel" in Path(model["model"]).stem.lower()
            ),
            None,
        )
        unified_index = next(
            (
                index
                for index, model in enumerate(self.detector.models)
                if "meter_digit_unified" in Path(model["model"]).stem.lower()
            ),
            None,
        )
        if light_index is None or unified_index is None:
            raise RuntimeError("automatic takeover requires lightlabel and meter_digit_unified RKNN models")
        self.monitor_pattern = [light_index, light_index, unified_index]

    def npu_fps(self) -> float:
        return float(self.performance_status.get("fps", 0.0))

    def enforce_performance_floor(self) -> None:
        if bool(self.performance_status.get("hard_failed")):
            raise RuntimeError(
                "RKNN throughput below hard minimum for "
                f"{float(self.performance_status.get('hard_low_duration_s', 0.0)):.1f}s: "
                f"{self.npu_fps():.2f} < {float(self.args.min_inference_fps):.2f} FPS"
            )

    def monitor_navigation(self) -> bool:
        assert isinstance(self.node, MissionNode)
        self.state = "MONITOR_NAV"
        self.update_status()
        monitor_started_at = time.time()
        deadline = time.time() + float(self.args.search_timeout_s)
        terminal_grace_deadline = 0.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if (
                self.node.navigation_activity_known
                and not self.node.navigation_active
            ):
                manager_phase = self.node.navigation.manager_phase
                if not allows_post_navigation_visual_grace(manager_phase):
                    self.state = "idle_no_active_goal"
                    self.update_status()
                    return False
                if terminal_grace_deadline <= 0.0:
                    terminal_grace_deadline = time.time() + float(
                        self.args.post_navigation_trigger_grace_s
                    )
                    self.post_navigation_grace_active = True
                    self.terminal_navigation_phase = manager_phase
                    self.skip_docking_due_to_navigation_failure = manager_phase in {
                        "FAILED",
                        "SAFETY_STOP",
                    }
                    self.state = "MONITOR_NAV_TERMINAL_GRACE"
                    self.trigger_window.clear()
                    self.update_status()
                elif time.time() >= terminal_grace_deadline:
                    self.post_navigation_grace_active = False
                    self.state = "idle_no_cabinet_after_navigation"
                    self.update_status()
                    return False
            elif self.node.navigation_active:
                terminal_grace_deadline = 0.0
                self.post_navigation_grace_active = False
                self.terminal_navigation_phase = ""
                self.skip_docking_due_to_navigation_failure = False
                if self.state != "MONITOR_NAV":
                    self.state = "MONITOR_NAV"
            try:
                frame = self.read_frame()
                result = self.detect_fast(frame)
                evidence = trigger_evidence(result, detection_label)
                plane = self.node.plane or {}
                clearance = float(plane.get("front_clearance_m", float("inf")))
                range_ok = float(self.args.trigger_min_clearance_m) <= clearance <= float(self.args.trigger_max_clearance_m)
                matched = bool(
                    evidence["matched"]
                    and range_ok
                    and (
                        self.node.navigation_active
                        or self.post_navigation_grace_active
                    )
                )
                self.trigger_window.append(matched)
                self.last_evidence = {**evidence, "range_ok": range_ok, "front_clearance_m": clearance}
                box, count, confidence = target_bbox(result, self.labels)
                self.last_detection = {
                    "target_found": box is not None,
                    "selected_count": count,
                    "confidence": round(confidence, 4),
                    "bbox": [] if box is None else [round(value, 2) for value in box],
                    "detections": selected_detection_summary(result),
                }
                self.last_error = ""
            except Exception as exc:
                self.last_error = str(exc)
                self.trigger_window.append(False)
            self.target_seen_count = sum(self.trigger_window)
            self.update_status()
            self.enforce_performance_floor()
            inference_reference = self.last_inference_at or monitor_started_at
            inference_age = time.time() - inference_reference
            if inference_age > float(self.args.max_detection_age_s):
                raise RuntimeError(
                    "cabinet detection stream is stale: "
                    f"{inference_age:.2f}s > {float(self.args.max_detection_age_s):.2f}s"
                )
            if self.target_seen_count >= int(self.args.trigger_required_frames):
                self.state = "CONFIRM_TARGET"
                self.update_status()
                return True
        return False

    def confirm_standalone_target(self) -> bool:
        assert isinstance(self.node, MissionNode)
        self.state = "CONFIRM_TARGET"
        self.trigger_window.clear()
        self.update_status(standalone_inspection=True)
        deadline = time.time() + float(self.args.standalone_confirm_timeout_s)
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            frame = self.read_frame()
            result = self.detect_fast(frame)
            evidence = trigger_evidence(result, detection_label)
            plane = self.node.plane or {}
            clearance = float(plane.get("front_clearance_m", float("inf")))
            range_ok = (
                float(self.args.trigger_min_clearance_m)
                <= clearance
                <= float(self.args.trigger_max_clearance_m)
            )
            inspection_only_match = bool(
                (self.args.skip_docking or self.args.relaxed_standalone_confirm)
                and int(evidence.get("voltage_count", 0)) >= 1
                and int(evidence.get("digit_count", 0)) >= 2
            )
            matched = bool((evidence["matched"] or inspection_only_match) and range_ok)
            self.trigger_window.append(matched)
            self.last_evidence = {
                **evidence,
                "inspection_only_match": inspection_only_match,
                "range_ok": range_ok,
                "front_clearance_m": clearance,
                "standalone_inspection": True,
            }
            box, count, confidence = target_bbox(result, self.labels)
            self.last_detection = {
                "target_found": box is not None,
                "selected_count": count,
                "confidence": round(confidence, 4),
                "bbox": [] if box is None else [round(value, 2) for value in box],
                "detections": selected_detection_summary(result),
            }
            self.target_seen_count = sum(self.trigger_window)
            self.update_status(standalone_inspection=True)
            self.enforce_performance_floor()
            if self.target_seen_count >= int(self.args.trigger_required_frames):
                return True
        self.last_error = "cabinet target was not confirmed for standalone inspection"
        return False

    def scan_safe_for_motion(self) -> tuple[bool, str]:
        if not isinstance(self.node, MissionNode):
            return False, "ROS node unavailable"
        scan_age = time.time() - self.node.last_scan_at
        if self.node.last_scan_at <= 0.0 or scan_age > float(self.args.max_scan_age_s):
            return False, f"scan stale: {scan_age:.2f}s"
        if self.node.plane is None:
            return False, "cabinet plane fit unavailable"
        if self.node.front_min < float(self.args.front_emergency_m):
            return False, f"front obstacle too close: {self.node.front_min:.3f}m"
        clearance = float(self.node.plane.get("front_clearance_m", float("inf")))
        if clearance < float(self.args.front_emergency_m):
            return False, f"front clearance too small: {clearance:.3f}m"
        return True, ""

    def dock(self) -> bool:
        assert isinstance(self.node, MissionNode)
        self.state = "DOCK"
        deadline = time.time() + float(self.args.adjust_timeout_s)
        ready_since = 0.0
        lost_since = 0.0
        plane_missing_since = 0.0
        active_axis_name = ""
        active_axis_started_at = 0.0
        active_axis_start_error = 0.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            safe, reason = self.scan_safe_for_motion()
            if not safe:
                self.node.stop(repeats=6)
                if reason == "cabinet plane fit unavailable":
                    if plane_missing_since <= 0.0:
                        plane_missing_since = time.time()
                    if time.time() - plane_missing_since < float(self.args.plane_lost_timeout_s):
                        self.update_status(safety_block_reason=reason)
                        time.sleep(0.05)
                        continue
                self.last_error = reason
                self.update_status(safety_block_reason=reason)
                return False
            plane_missing_since = 0.0
            try:
                frame = self.read_frame()
                result = self.detect_fast(frame)
                self.enforce_performance_floor()
                box, count, confidence = target_bbox(result, self.labels)
            except Exception as exc:
                self.last_error = str(exc)
                box, count, confidence, frame, result = None, 0, 0.0, None, {}
            if box is None or frame is None:
                if lost_since <= 0.0:
                    lost_since = time.time()
                self.node.stop(repeats=2)
                if time.time() - lost_since >= float(self.args.target_lost_timeout_s):
                    return False
                continue
            lost_since = 0.0
            self.update_best_frames(frame, result)
            height, width = frame.shape[:2]
            metrics = bbox_metrics(box, width, height)
            plane = dict(self.node.plane or {})
            clearance = float(plane.get("front_clearance_m", float("inf")))
            yaw_error = float(plane.get("yaw_error_rad", 0.0))
            control_yaw_error = 0.0 if self.args.disable_yaw_docking else yaw_error
            center_error = float(metrics["error_x"])
            control_center_error = 0.0 if self.args.disable_lateral_docking else center_error
            distance_error = clearance - float(self.args.target_clearance_m)
            control_distance_error = float(self.args.distance_sign) * distance_error
            aligned = (
                abs(control_yaw_error) <= math.radians(float(self.args.plane_yaw_tolerance_deg))
                and abs(control_center_error) <= float(self.args.center_tolerance)
                and abs(distance_error) <= float(self.args.clearance_tolerance_m)
            )
            self.last_detection = {
                "target_found": True,
                "selected_count": count,
                "confidence": round(confidence, 4),
                "bbox": [round(value, 2) for value in box],
                "metrics": metrics,
                "detections": selected_detection_summary(result),
                "aligned": aligned,
                "distance_sign": float(self.args.distance_sign),
                "yaw_docking_disabled": bool(self.args.disable_yaw_docking),
                "lateral_docking_disabled": bool(self.args.disable_lateral_docking),
            }
            self.final_frame = frame.copy()
            self.final_result = json.loads(json.dumps(result, ensure_ascii=False))
            vx, vy, wz, active_axis = docking_command(
                yaw_error=control_yaw_error,
                center_error=control_center_error,
                distance_error=control_distance_error,
                yaw_tolerance=math.radians(float(self.args.plane_yaw_tolerance_deg)),
                center_tolerance=float(self.args.center_tolerance),
                distance_tolerance=float(self.args.clearance_tolerance_m),
                yaw_gain=float(self.args.plane_yaw_gain),
                yaw_sign=float(self.args.plane_yaw_sign),
                lateral_gain=float(self.args.lateral_gain),
                lateral_sign=float(self.args.lateral_sign),
                distance_gain=float(self.args.distance_gain),
                min_angular=float(self.args.min_angular),
                max_angular=float(self.args.max_angular),
                min_lateral=float(self.args.min_lateral),
                max_lateral=float(self.args.max_lateral),
                min_linear=float(self.args.min_linear),
                max_linear=float(self.args.max_linear),
                max_reverse=float(self.args.max_reverse),
            )
            self.last_detection["active_axis"] = active_axis
            axis_error = {
                "yaw": abs(control_yaw_error),
                "lateral": abs(control_center_error),
                "distance": abs(distance_error),
            }.get(active_axis, 0.0)
            if active_axis != active_axis_name:
                active_axis_name = active_axis
                active_axis_started_at = time.time()
                active_axis_start_error = axis_error
            elif active_axis != "aligned" and time.time() - active_axis_started_at >= float(
                self.args.axis_divergence_timeout_s
            ):
                divergence_limit = {
                    "yaw": math.radians(float(self.args.yaw_divergence_deg)),
                    "lateral": float(self.args.center_divergence),
                    "distance": float(self.args.distance_divergence_m),
                }[active_axis]
                if axis_error > active_axis_start_error + divergence_limit:
                    self.node.stop(repeats=8)
                    self.last_error = (
                        f"{active_axis} correction diverged: "
                        f"{active_axis_start_error:.4f} -> {axis_error:.4f}"
                    )
                    self.update_status(safety_block_reason=self.last_error)
                    return False
            self.last_cmd_vel = self.node.publish_twist(vx, wz, vy)
            self.update_status()
            if aligned:
                if ready_since <= 0.0:
                    ready_since = time.time()
                if time.time() - ready_since >= float(self.args.ready_hold_s):
                    self.node.stop(repeats=8)
                    return True
            else:
                ready_since = 0.0
        self.node.stop(repeats=8)
        self.last_error = "cabinet docking timed out"
        return False

    def arm_scan(self) -> None:
        assert isinstance(self.node, MissionNode)
        self.node.stop(repeats=6)
        stages = self.profile.get("stages", {}) if isinstance(self.profile.get("stages"), dict) else {}
        for stage_name in ("meters", "indicator_digit"):
            cfg = stages.get(stage_name, {}) if isinstance(stages.get(stage_name), dict) else {}
            if not cfg or not bool(cfg.get("enabled", True)):
                continue
            self.state = f"ARM_SCAN_{stage_name.upper()}"
            rough = normalize_pwm(cfg.get("rough_pwm", self.home_pwm), low=self.servo_min, high=self.servo_max)
            self.move_arm(rough, int(cfg.get("rough_duration_ms", 1200)))
            self.center_pwm = list(rough)
            time.sleep(max(0.5, float(cfg.get("settle_s", 1.0))))
            labels = {str(item).lower() for item in cfg.get("target_labels", [])}
            if "__digits__" in labels:
                labels.remove("__digits__")
                labels.update(DIGIT_LABELS)
            deadline = time.time() + float(self.args.arm_stage_timeout_s)
            while time.time() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.01)
                frame = self.read_frame()
                result = self.detect_frame(frame)
                result = self.enrich_stage_result(
                    stage_name,
                    frame,
                    result,
                    f"{self.test_id}_{stage_name}_temporal",
                )
                result["_temporally_enriched"] = True
                self.record_temporal_result(result, frame.shape[:2])
                self.update_best_frames(frame, result)
                box, _, _ = target_bbox(result, labels or self.labels)
                if box is not None:
                    metrics = bbox_metrics(box, frame.shape[1], frame.shape[0])
                    self.adjust_arm(metrics)
                    self.last_detection = {
                        "target_found": True,
                        "metrics": metrics,
                        "detections": selected_detection_summary(result),
                    }
                else:
                    self.nudge_arm_for_missing_items()
                self.update_status()
                if stage_name == "meters" and self.consensus_snapshot().get("meters_complete"):
                    break
                if (
                    stage_name == "indicator_digit"
                    and self.consensus_snapshot().get("digital_display_complete")
                    and self.consensus_snapshot().get("status_lights_complete")
                ):
                    break
        if not self.consensus_snapshot().get("status_lights_complete"):
            self.state = "ARM_SCAN_STATUS_LIGHTS"
            deadline = time.time() + float(self.args.status_light_refine_s)
            while time.time() < deadline and not self.consensus_snapshot().get("status_lights_complete"):
                rclpy.spin_once(self.node, timeout_sec=0.01)
                self.node.stop(repeats=1)
                frame = self.read_frame()
                result = self.detect_frame(frame)
                result = self.enrich_stage_result(
                    "status_lights",
                    frame,
                    result,
                    f"{self.test_id}_status_lights_temporal",
                )
                result["_temporally_enriched"] = True
                self.record_temporal_result(result, frame.shape[:2])
                self.update_best_frames(frame, result)
                self.nudge_arm_for_status_lights()
                self.update_status()

    def record_temporal_result(self, result: dict[str, Any], image_shape: tuple[int, int]) -> None:
        self.consensus.add_meter_readings(result.get("meter_readings", []))
        self.consensus.add_digit_readings(result.get("digit_readings", {}))
        lights = status_light_items(
            result,
            min_conf=float(self.args.status_light_conf),
            image_shape=image_shape,
        )
        self.consensus.add_light_slots(status_light_slot_map(lights))
        self.temporal_consensus = self.consensus_snapshot()

    def consensus_snapshot(self) -> dict[str, Any]:
        return self.consensus.snapshot(
            meter_min_samples=int(self.args.meter_consensus_frames),
            digit_min_samples=int(self.args.digit_consensus_frames),
            light_min_samples=int(self.args.light_consensus_frames),
            required_lights=int(self.args.required_status_lights),
        )

    def apply_temporal_consensus(self) -> None:
        snapshot = self.consensus_snapshot()
        self.temporal_consensus = snapshot
        meters = snapshot.get("meters", {})
        meter_frame = self.evaluated_frames.get("meters")
        if meter_frame is not None and snapshot.get("meters_complete"):
            meter_frame.result["meter_readings"] = [dict(meters[name]) for name in ("voltage", "current")]
            cabinet = meter_frame.result.setdefault("cabinet_data", {})
            for name in ("voltage", "current"):
                item = meters[name]
                cabinet[name] = {
                    "value": item.get("value"),
                    "unit": item.get("unit") or ("V" if name == "voltage" else "A"),
                    "status": "ok",
                    "source": "temporal_median",
                    "sample_count": item.get("sample_count", 0),
                }
            meter_frame.coverage.update({"meters": True, "voltage_ok": True, "current_ok": True})

        digital = snapshot.get("digital_display", {})
        digital_frame = self.evaluated_frames.get("digital_display")
        if digital_frame is not None and snapshot.get("digital_display_complete"):
            digital_frame.result["digit_readings"] = dict(digital.get("readings", {}))
            digital_frame.result["digit_readings"]["temporal_sample_count"] = digital.get("sample_count", 0)
            digital_frame.coverage.update(
                {
                    "digital_display": True,
                    "digital_text": digital.get("global_text", ""),
                    "digital_rows": len(digital.get("texts", [])),
                }
            )

        lights = snapshot.get("status_lights", {})
        light_frame = self.evaluated_frames.get("status_lights")
        if light_frame is not None:
            light_frame.result["status_light_consensus"] = dict(lights)
            light_frame.coverage.update(
                {
                    "status_lights": bool(snapshot.get("status_lights_complete")),
                    "status_lights_count": int(snapshot.get("status_lights_count", 0)),
                }
            )
        for frame in self.evaluated_frames.values():
            frame.result["temporal_consensus"] = snapshot

        self.coverage.update(
            {
                "meters": bool(snapshot.get("meters_complete")),
                "voltage_ok": "voltage" in meters,
                "current_ok": "current" in meters,
                "digital_display": bool(snapshot.get("digital_display_complete")),
                "digital_rows": len(digital.get("texts", [])),
                "status_lights": bool(snapshot.get("status_lights_complete")),
                "status_lights_count": int(snapshot.get("status_lights_count", 0)),
            }
        )
        missing: list[str] = []
        if "voltage" not in meters:
            missing.append("voltage_meter")
        if "current" not in meters:
            missing.append("current_meter")
        if not snapshot.get("digital_display_complete"):
            missing.append("digital_display_consensus")
        if not snapshot.get("status_lights_complete"):
            missing.append(
                f"status_lights_consensus_{int(snapshot.get('status_lights_count', 0))}/{int(self.args.required_status_lights)}"
            )
        self.missing_items = missing

    def save_captures(self) -> dict[str, Any]:
        self.apply_temporal_consensus()
        saved = super().save_captures()
        saved["temporal_consensus"] = dict(self.temporal_consensus)
        return saved

    def upload_recognition(self, stage: str, paths: dict[str, str], coverage: dict[str, Any]) -> dict[str, Any]:
        raw_path = Path(paths["raw"])
        annotated_path = Path(paths["annotated"])
        payload_path = Path(paths["json"])
        payload = json.loads(payload_path.read_text(encoding="utf-8"))
        payload.update(
            {
                "inspection_id": self.inspection_id,
                "inspection_stage": stage,
                "capture_key": f"{self.inspection_id}:{stage}",
                "coverage": coverage,
                "board_id": self.args.board_id,
                "board_label": self.args.board_label,
            }
        )
        with raw_path.open("rb") as raw_handle, annotated_path.open("rb") as annotated_handle:
            response = requests.post(
                f"{self.args.server_url.rstrip('/')}/api/upload/recognition",
                headers={"X-Upload-Token": self.args.upload_token},
                data={"payload": json.dumps(payload, ensure_ascii=False)},
                files={
                    "raw_image": (raw_path.name, raw_handle, "image/jpeg"),
                    "annotated_image": (annotated_path.name, annotated_handle, "image/jpeg"),
                },
                timeout=45,
            )
        response.raise_for_status()
        body = response.json()
        return body.get("recognition", body)

    def queue_outbox(self, kind: str, payload: dict[str, Any]) -> None:
        self.outbox_dir.mkdir(parents=True, exist_ok=True)
        path = self.outbox_dir / f"{self.inspection_id}_{kind}.json"
        atomic_write_json(path, payload)

    def upload_results(self, saved: dict[str, Any]) -> None:
        self.state = "UPLOAD"
        stage_results: list[dict[str, Any]] = []
        for photo in saved.get("photos", []):
            stage = str(photo.get("stage") or "final")
            try:
                uploaded = self.upload_recognition(stage, photo["paths"], photo.get("coverage", {}))
                stage_results.append({"stage": stage, "status": "completed", "recognition": uploaded})
            except Exception as exc:
                stage_results.append({"stage": stage, "status": "pending_upload", "message": str(exc)})
                self.queue_outbox(
                    f"recognition_{stage}",
                    {
                        "kind": "recognition",
                        "inspection_id": self.inspection_id,
                        "stage": stage,
                        "paths": photo.get("paths", {}),
                        "coverage": photo.get("coverage", {}),
                        "board_id": self.args.board_id,
                        "board_label": self.args.board_label,
                    },
                )
        summary = {
            "inspection_id": self.inspection_id,
            "status": (
                "completed"
                if self.coverage_complete() and not self.docking_degraded_reason
                else "partial"
            ),
            "profile": "rknn_visual_navigation_takeover",
            "board_id": self.args.board_id,
            "board_label": self.args.board_label,
            "completed_at": utc_now_iso(),
            "stages": stage_results,
            "cabinet_update": saved.get("cabinet_update", {}),
            "coverage": dict(self.coverage),
            "missing_items": list(self.missing_items),
            "temporal_consensus": dict(self.temporal_consensus),
            "docking": {
                "completed": self.docking_completed,
                "degraded": bool(self.docking_degraded_reason),
                "reason": self.docking_degraded_reason,
            },
        }
        try:
            response = requests.post(
                f"{self.args.server_url.rstrip('/')}/api/upload/inspection",
                headers={"X-Upload-Token": self.args.upload_token},
                json=summary,
                timeout=30,
            )
            response.raise_for_status()
            summary["cloud_upload"] = response.json()
        except Exception as exc:
            summary["upload_error"] = str(exc)
            self.queue_outbox("inspection", {"kind": "inspection", "payload": summary})
        self.result = summary
        self.state = "COMPLETE" if summary["status"] == "completed" else "PARTIAL"
        self.update_status()

    def run(self) -> int:
        try:
            self.ensure_runtime()
            self.detector = StreamDetector(self.args)
            self.configure_monitor_models()
            self.open_camera()
            self.start_ros()
            assert isinstance(self.node, MissionNode)
            if self.args.standalone_inspection:
                if not self.confirm_standalone_target():
                    raise RuntimeError(self.last_error)
            elif not self.monitor_navigation():
                return 0
            self.state = "LOCK_CHASSIS"
            self.update_status()
            if not self.node.set_cabinet_mode(True):
                raise RuntimeError("failed to acquire cabinet chassis control")
            self.control_was_acquired = True
            self.takeover_active = True
            self.update_status(control_acquired=True)
            self.node.stop(repeats=8)
            if not self.args.standalone_inspection:
                self.state = "CANCEL_NAV"
                self.update_status()
                if not self.node.cancel_navigation():
                    raise RuntimeError("failed to confirm managed navigation cancellation")
                self.navigation_cancel_confirmed = True
                self.update_status()
            self.move_arm(self.home_pwm, int(self.args.initial_arm_duration_ms))
            time.sleep(max(0.3, float(self.args.initial_arm_duration_ms) / 1000.0))
            if self.args.skip_docking or self.skip_docking_due_to_navigation_failure:
                self.state = "DOCK_SKIPPED"
                self.node.stop(repeats=8)
                self.update_status(
                    docking_skipped=True,
                    docking_skip_reason=(
                        f"navigation terminal phase {self.terminal_navigation_phase}"
                        if self.skip_docking_due_to_navigation_failure
                        else "configured skip_docking"
                    ),
                )
            elif not self.dock():
                self.docking_degraded_reason = self.last_error or "cabinet docking failed"
                self.state = "DOCK_FAILED_SCAN_FALLBACK"
                self.node.stop(repeats=8)
                self.update_status(
                    docking_fallback=True,
                    docking_fallback_reason=self.docking_degraded_reason,
                )
            else:
                self.docking_completed = True
                self.update_status()
            self.arm_scan()
            self.state = "VERIFY"
            saved = self.save_captures()
            self.upload_results(saved)
            return 0
        except Exception as exc:
            self.failure_stage = self.state
            self.last_error = str(exc)
            self.failure_phase = "post_takeover" if self.control_was_acquired else "pre_takeover"
            self.state = "FAILED"
            self.result = {
                "message": str(exc),
                "failure_phase": self.failure_phase,
                "failure_stage": self.failure_stage,
                "control_was_acquired": self.control_was_acquired,
            }
            self.update_status()
            return 1
        finally:
            if isinstance(self.node, MissionNode):
                self.node.stop(repeats=8)
                if self.control_was_acquired:
                    if not self.args.standalone_inspection:
                        self.navigation_cancel_confirmed = bool(
                            self.navigation_cancel_confirmed
                            or self.node.cancel_navigation(timeout_s=5.0)
                        )
                    try:
                        self.move_arm(self.home_pwm, int(self.args.initial_arm_duration_ms))
                    except Exception:
                        pass
                    if self.args.standalone_inspection or self.navigation_cancel_confirmed:
                        self.node.set_cabinet_mode(False)
                    else:
                        self.last_error = (
                            self.last_error
                            or "cabinet mode retained because navigation cancellation was not confirmed"
                        )
                self.node.destroy_node()
                self.node = None
            self.takeover_active = False
            if rclpy.ok():
                rclpy.shutdown()
            if self.camera_worker is not None:
                self.camera_worker.close()
                self.camera_worker = None
                self.camera = None
            elif self.camera is not None:
                self.camera.release()
            if self.detector is not None:
                self.detector.close()
            self.update_status()


def build_parser() -> argparse.ArgumentParser:
    parser = build_stream_parser()
    parser.description = "Automatic RKNN cabinet visual takeover during active navigation."
    parser.set_defaults(
        status_file=str(DEFAULT_STATUS_FILE),
        capture_dir=str(DEFAULT_CAPTURE_DIR),
        cmd_vel_topic="/cmd_vel_cabinet",
        backend="rknn",
        require_rknn=True,
        allow_nav_running=True,
        max_linear=0.04,
        max_angular=0.10,
        center_tolerance=0.06,
        front_emergency_m=0.15,
        max_scan_age_s=0.30,
        max_arm_step_pwm=30,
        max_arm_total_delta_pwm=180,
        status_light_pan_total_pwm=180,
        status_light_tilt_total_pwm=180,
    )
    parser.add_argument("--server-url", default=os.environ.get("CAR_CLOUD_SERVER_URL", ""))
    parser.add_argument("--upload-token", default=os.environ.get("CAR_CLOUD_UPLOAD_TOKEN", ""))
    parser.add_argument("--board-id", default=os.environ.get("CAR_CLOUD_BOARD_ID", "rk3588-f103-board"))
    parser.add_argument("--board-label", default=os.environ.get("CAR_CLOUD_BOARD_LABEL", "RK3588 F103 Board"))
    parser.add_argument("--outbox-dir", default=str(DEFAULT_OUTBOX_DIR))
    parser.add_argument("--nav-status-topic", default="/navigate_to_pose/_action/status")
    parser.add_argument("--nav-cancel-service", default="/navigate_to_pose/_action/cancel_goal")
    parser.add_argument("--nav-manager-status-topic", default="/maze_turn_manager/status")
    parser.add_argument("--nav-manager-cancel-service", default="/maze_turn_manager/cancel")
    parser.add_argument("--navigation-active-at-launch", action="store_true", default=False)
    parser.add_argument("--post-navigation-trigger-grace-s", type=float, default=8.0)
    parser.add_argument("--cabinet-mode-service", default="/f103_usb_ros2_bridge/cabinet_mode")
    parser.add_argument("--trigger-window-frames", type=int, default=5)
    parser.add_argument("--trigger-required-frames", type=int, default=3)
    parser.add_argument("--trigger-min-clearance-m", type=float, default=0.25)
    parser.add_argument("--trigger-max-clearance-m", type=float, default=1.20)
    parser.add_argument("--target-inference-fps", type=float, default=5.0)
    parser.add_argument("--min-inference-fps", type=float, default=4.0)
    parser.add_argument("--fps-window-frames", type=int, default=30)
    parser.add_argument("--fps-warmup-frames", type=int, default=30)
    parser.add_argument("--fps-hard-fail-hold-s", type=float, default=5.0)
    parser.add_argument("--ros-ready-timeout-s", type=float, default=30.0)
    parser.add_argument("--target-clearance-m", type=float, default=0.20)
    parser.add_argument("--clearance-tolerance-m", type=float, default=0.03)
    parser.add_argument("--target-lost-timeout-s", type=float, default=1.0)
    parser.add_argument("--max-detection-age-s", type=float, default=0.50)
    parser.add_argument("--laser-x-m", type=float, default=0.13)
    parser.add_argument("--front-bumper-x-m", type=float, default=0.14)
    parser.add_argument("--plane-sector-deg", type=float, default=35.0)
    parser.add_argument("--plane-max-range-m", type=float, default=1.50)
    parser.add_argument("--plane-ransac-iterations", type=int, default=50)
    parser.add_argument("--plane-inlier-threshold-m", type=float, default=0.025)
    parser.add_argument("--plane-min-inliers", type=int, default=12)
    parser.add_argument("--plane-min-span-m", type=float, default=0.25)
    parser.add_argument("--plane-lost-timeout-s", type=float, default=1.0)
    parser.add_argument("--plane-yaw-tolerance-deg", type=float, default=4.0)
    parser.add_argument("--plane-yaw-gain", type=float, default=0.8)
    parser.add_argument("--plane-yaw-sign", type=float, default=1.0)
    parser.add_argument("--min-angular", type=float, default=0.06)
    parser.add_argument("--max-lateral", type=float, default=0.035)
    parser.add_argument("--min-lateral", type=float, default=0.03)
    parser.add_argument("--lateral-gain", type=float, default=0.20)
    parser.add_argument("--lateral-sign", type=float, default=-1.0)
    parser.add_argument("--min-linear", type=float, default=0.03)
    parser.add_argument("--max-reverse", type=float, default=0.03)
    parser.add_argument("--distance-gain", type=float, default=0.6)
    parser.add_argument("--distance-sign", type=float, default=1.0)
    parser.add_argument("--disable-yaw-docking", action="store_true", default=False)
    parser.add_argument("--disable-lateral-docking", action="store_true", default=False)
    parser.add_argument("--axis-divergence-timeout-s", type=float, default=2.0)
    parser.add_argument("--yaw-divergence-deg", type=float, default=2.0)
    parser.add_argument("--center-divergence", type=float, default=0.08)
    parser.add_argument("--distance-divergence-m", type=float, default=0.03)
    parser.add_argument("--arm-stage-timeout-s", type=float, default=20.0)
    parser.add_argument("--consensus-max-samples", type=int, default=12)
    parser.add_argument("--meter-consensus-frames", type=int, default=3)
    parser.add_argument("--digit-consensus-frames", type=int, default=2)
    parser.add_argument("--light-consensus-frames", type=int, default=2)
    parser.add_argument("--standalone-inspection", action="store_true", default=False)
    parser.add_argument("--standalone-confirm-timeout-s", type=float, default=20.0)
    parser.add_argument("--relaxed-standalone-confirm", action="store_true", default=False)
    parser.add_argument("--skip-docking", action="store_true", default=False)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    runner = CabinetVisualMission(args)
    try:
        return runner.run()
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())

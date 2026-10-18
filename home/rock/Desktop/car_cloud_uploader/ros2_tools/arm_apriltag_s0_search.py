#!/usr/bin/env python3
"""Search a cabinet AprilTag by sweeping only arm servo S0.

This tool is deliberately independent from navigation, YOLO, inspection, and
upload.  It requires an explicit --execute flag before sending arm commands.
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import os
import re
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import cv2
import numpy as np
import yaml


TOOL_DIR = Path(__file__).resolve().parent
ROOT_DIR = TOOL_DIR.parent
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

from cabinet_camera_overlay import (  # noqa: E402
    atomic_write_json,
    create_tag_detector,
    detect_tags,
    intrinsic_arrays,
    load_yaml,
    sha256_file,
)


SERVO_COUNT = 6
HOME_PWM = [1640, 1500, 1500, 2250, 1500, 1500]
STATE_PWM_RE = re.compile(r"(?:^|,)S([0-5])=(-?\d+)(?=,|!)")
ACTIVE_NAV_STATUSES = {1, 2, 3}
PREVIEW_WINDOW_NAME = "AprilTag S0 Search"
MANUAL_PREVIEW_CLOSE_REASONS = {
    "preview stop requested",
    "preview window was closed",
}


class SearchFailure(RuntimeError):
    """Expected terminal failure that should return S0 to the home pose."""


class OtherAxesRestored(RuntimeError):
    """Internal signal requesting that the interrupted S0 target be resumed."""


def is_manual_preview_close(reason: str) -> bool:
    return str(reason) in MANUAL_PREVIEW_CLOSE_REASONS


def clamp_int(value: float | int, low: int, high: int) -> int:
    return int(max(low, min(high, int(round(value)))))


def build_arm_payload(pwm: Iterable[int], duration_ms: int) -> str:
    values = list(pwm)
    if len(values) != SERVO_COUNT:
        raise ValueError(f"expected {SERVO_COUNT} servo values")
    duration = clamp_int(duration_ms, 20, 9999)
    return "{" + "".join(
        f"#{index:03d}P{clamp_int(value, 500, 2500):04d}T{duration:04d}!"
        for index, value in enumerate(values)
    ) + "}"


def build_s0_payload(s0_pwm: int, duration_ms: int) -> str:
    duration = clamp_int(duration_ms, 20, 9999)
    pwm = clamp_int(s0_pwm, 500, 2500)
    return f"#000P{pwm:04d}T{duration:04d}!"


def build_servo_targets_payload(
    targets: Sequence[tuple[int, int]],
    duration_ms: int,
) -> str:
    if not targets:
        raise ValueError("at least one servo target is required")
    duration = clamp_int(duration_ms, 20, 9999)
    parts: list[str] = []
    seen: set[int] = set()
    for index, pwm in targets:
        index = int(index)
        if index < 0 or index >= SERVO_COUNT:
            raise ValueError(f"servo index out of range: {index}")
        if index in seen:
            raise ValueError(f"duplicate servo index: {index}")
        seen.add(index)
        parts.append(f"#{index:03d}P{clamp_int(pwm, 500, 2500):04d}T{duration:04d}!")
    return parts[0] if len(parts) == 1 else "{" + "".join(parts) + "}"


def off_home_axes(
    values: Sequence[int],
    home_pwm: Sequence[int],
    tolerance_pwm: int,
) -> list[int]:
    if len(values) < SERVO_COUNT or len(home_pwm) < SERVO_COUNT:
        return list(range(1, SERVO_COUNT))
    tolerance = abs(int(tolerance_pwm))
    return [
        index
        for index in range(1, SERVO_COUNT)
        if abs(int(values[index]) - int(home_pwm[index])) > tolerance
    ]


def other_axes_home_targets(home_pwm: Sequence[int]) -> list[tuple[int, int]]:
    if len(home_pwm) < SERVO_COUNT:
        raise ValueError(f"home_pwm must contain at least {SERVO_COUNT} values")
    return [(index, int(home_pwm[index])) for index in range(1, SERVO_COUNT)]


def s0_pose(s0_pwm: int, home_pwm: Sequence[int]) -> list[int]:
    if len(home_pwm) != SERVO_COUNT:
        raise ValueError("home_pwm must contain six values")
    return [int(s0_pwm), *[int(value) for value in home_pwm[1:]]]


def parse_f103_state(payload: str) -> dict[str, Any]:
    data = json.loads(payload)
    if not isinstance(data, dict):
        raise ValueError("f103_state must be a JSON object")
    raw_state = str(data.get("last_state") or "")
    fields: dict[str, str] = {}
    if raw_state.startswith("@STATE:"):
        state_body = raw_state[len("@STATE:") :]
        if state_body.endswith("!"):
            state_body = state_body[:-1]
        for item in state_body.split(","):
            if "=" in item:
                key, value = item.split("=", 1)
                fields[key.strip()] = value.strip()
    servo_pwm = [None] * SERVO_COUNT
    for index, value in STATE_PWM_RE.findall(raw_state):
        servo_pwm[int(index)] = int(value)
    data["state_fields"] = fields
    data["servo_pwm"] = None if any(value is None for value in servo_pwm) else servo_pwm
    data["estop"] = int(data.get("estop", fields.get("ESTOP", 0)) or 0)
    data["mode"] = str(data.get("mode") or fields.get("MODE", ""))
    return data


def tag_center_x(tag: dict[str, Any]) -> float:
    corners = np.asarray(tag["corners"], dtype=np.float64).reshape(4, 2)
    return float(np.mean(corners[:, 0]))


def tag_size_px(tag: dict[str, Any]) -> float:
    corners = np.asarray(tag["corners"], dtype=np.float64).reshape(4, 2)
    sides = np.linalg.norm(np.roll(corners, -1, axis=0) - corners, axis=1)
    return float(np.mean(sides))


class TagConfirmation:
    def __init__(self, window_frames: int, required_frames: int, preferred_id: int):
        self.values: collections.deque[set[int]] = collections.deque(maxlen=max(1, window_frames))
        self.required_frames = max(1, required_frames)
        self.preferred_id = int(preferred_id)

    def observe(self, tags: Sequence[dict[str, Any]]) -> dict[str, Any] | None:
        valid = [tag for tag in tags if bool(tag.get("valid"))]
        by_id = {int(tag["id"]): tag for tag in valid}
        self.values.append(set(by_id))
        counts = collections.Counter(tag_id for ids in self.values for tag_id in ids)
        candidates = [tag_id for tag_id, count in counts.items() if count >= self.required_frames]
        candidates = [tag_id for tag_id in candidates if tag_id in by_id]
        if not candidates:
            return None
        candidates.sort(
            key=lambda tag_id: (
                tag_id != self.preferred_id,
                -counts[tag_id],
                -tag_size_px(by_id[tag_id]),
                float(by_id[tag_id].get("reprojection_error_px", float("inf"))),
                tag_id,
            )
        )
        return by_id[candidates[0]]


@dataclass(frozen=True)
class SearchSegment:
    target_pwm: int
    duration_ms: int


@dataclass(frozen=True)
class PositioningSegment:
    target_pwm: int
    duration_ms: int
    timeout_s: float


def build_min_positioning_segments(config: dict[str, Any]) -> list[PositioningSegment]:
    arm = config["arm"]
    configured = arm.get("move_to_min_waypoints")
    if not configured:
        return [
            PositioningSegment(
                int(arm["min_pwm"]),
                int(arm["move_to_min_duration_ms"]),
                float(arm["move_to_min_timeout_s"]),
            )
        ]
    segments = [
        PositioningSegment(
            int(item["target_pwm"]),
            int(item["duration_ms"]),
            float(item["timeout_s"]),
        )
        for item in configured
    ]
    minimum = int(arm["min_pwm"])
    maximum = int(arm["max_pwm"])
    if segments[-1].target_pwm != minimum:
        raise ValueError("move_to_min_waypoints must finish at arm.min_pwm")
    if any(not minimum <= item.target_pwm <= maximum for item in segments):
        raise ValueError("move_to_min_waypoints contains an out-of-range target")
    if any(item.duration_ms <= 0 or item.timeout_s <= item.duration_ms / 1000.0 for item in segments):
        raise ValueError("each move_to_min waypoint timeout must exceed its motion duration")
    if any(segments[index].target_pwm >= segments[index - 1].target_pwm for index in range(1, len(segments))):
        raise ValueError("move_to_min_waypoints must descend monotonically")
    return segments


def build_search_segments(config: dict[str, Any]) -> list[SearchSegment]:
    search = config["search"]
    segments = [
        SearchSegment(int(value), int(search["segment_duration_ms"]))
        for value in search["targets"]
    ]
    if not segments or segments[-1].target_pwm != int(config["arm"]["max_pwm"]):
        raise ValueError("search targets must finish at arm.max_pwm")
    if any(segment.target_pwm < int(config["arm"]["min_pwm"]) for segment in segments):
        raise ValueError("search target is below arm.min_pwm")
    if any(segment.target_pwm > int(config["arm"]["max_pwm"]) for segment in segments):
        raise ValueError("search target is above arm.max_pwm")
    estimated_duration_s = sum(segment.duration_ms for segment in segments) / 1000.0
    estimated_duration_s += len(segments) * float(search.get("step_settle_s", 0.0))
    estimated_duration_s += (
        len(segments)
        * max(1, int(search.get("stationary_frames_per_step", 3)))
        * max(0.0, float(search.get("detect_interval_s", 0.0)))
    )
    if estimated_duration_s > float(search["max_duration_s"]):
        raise ValueError("estimated stepped search duration exceeds search max_duration_s")
    return segments


def alignment_target(
    current_pwm: int,
    origin_pwm: int,
    error_px: float,
    config: dict[str, Any],
) -> int:
    alignment = config["alignment"]
    arm = config["arm"]
    tolerance = float(alignment["tolerance_px"])
    if abs(error_px) <= tolerance:
        return int(current_pwm)
    delta = int(round(float(alignment["pwm_per_px"]) * float(error_px)))
    minimum = max(1, int(alignment["min_step_pwm"]))
    maximum = max(minimum, int(alignment["max_step_pwm"]))
    if abs(delta) < minimum:
        delta = minimum if delta >= 0 else -minimum
    delta = clamp_int(delta, -maximum, maximum)
    bound = int(alignment["max_total_delta_pwm"])
    low = max(int(arm["min_pwm"]), int(origin_pwm) - bound)
    high = min(int(arm["max_pwm"]), int(origin_pwm) + bound)
    return clamp_int(int(current_pwm) + delta, low, high)


def alignment_consensus(observations: Sequence[bool], required_frames: int) -> bool:
    required = max(1, int(required_frames))
    return bool(observations) and bool(observations[-1]) and sum(bool(value) for value in observations) >= required


class DivergenceGuard:
    def __init__(self, increase_px: float, consecutive_limit: int):
        self.increase_px = float(increase_px)
        self.consecutive_limit = max(1, int(consecutive_limit))
        self.reference_error: float | None = None
        self.increases = 0

    def command_sent(self, error_px: float) -> None:
        self.reference_error = abs(float(error_px))

    def evaluate(self, error_px: float) -> bool:
        if self.reference_error is None:
            return False
        if abs(float(error_px)) > self.reference_error + self.increase_px:
            self.increases += 1
        else:
            self.increases = 0
        self.reference_error = None
        return self.increases >= self.consecutive_limit


class CameraPipeline:
    def __init__(self, config: dict[str, Any]):
        self.config = config
        paths = config["paths"]
        self.intrinsics_path = Path(paths["intrinsics"])
        expected_hash = str(paths.get("intrinsics_sha256", "")).upper()
        if expected_hash and sha256_file(self.intrinsics_path) != expected_hash:
            raise RuntimeError("camera intrinsics SHA256 mismatch")
        self.intrinsics = load_yaml(self.intrinsics_path)
        camera_matrix, dist_coeffs, new_matrix, image_size = intrinsic_arrays(self.intrinsics)
        self.image_size = tuple(int(value) for value in image_size)
        if str(self.intrinsics.get("model")) == "fisheye":
            self.maps = cv2.fisheye.initUndistortRectifyMap(
                camera_matrix, dist_coeffs, np.eye(3), new_matrix, self.image_size, cv2.CV_32FC1
            )
        else:
            self.maps = cv2.initUndistortRectifyMap(
                camera_matrix, dist_coeffs, None, new_matrix, self.image_size, cv2.CV_32FC1
            )
        self.detect_markers = create_tag_detector(config["apriltag"]["dictionary"])
        self.capture: cv2.VideoCapture | None = None

    def open(self) -> None:
        camera = self.config["camera"]
        device = str(camera["device"])
        width, height, fps = int(camera["width"]), int(camera["height"]), int(camera["fps"])
        capture = None
        if str(camera.get("backend", "gstreamer")).lower() == "gstreamer":
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
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            capture.set(cv2.CAP_PROP_FPS, fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            raise RuntimeError(f"cannot open camera: {device}")
        self.capture = capture

    def read(self) -> np.ndarray:
        if self.capture is None:
            raise RuntimeError("camera is not open")
        ok, frame = self.capture.read()
        if not ok or frame is None:
            raise RuntimeError("camera read failed")
        if bool(self.config["camera"].get("rotate_180", True)):
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        actual_size = (frame.shape[1], frame.shape[0])
        if actual_size != self.image_size:
            raise RuntimeError(f"camera/intrinsics size mismatch: {actual_size} != {self.image_size}")
        return cv2.remap(frame, self.maps[0], self.maps[1], cv2.INTER_LINEAR)

    def detect(self, frame: np.ndarray) -> dict[str, Any]:
        return detect_tags(frame, self.intrinsics, self.config["apriltag"], self.detect_markers)

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None


def create_ros_node(config: dict[str, Any]):
    try:
        import rclpy
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import Twist
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import SetBool
    except Exception as exc:  # pragma: no cover - board runtime dependency
        raise RuntimeError(f"ROS 2 dependencies unavailable: {exc}") from exc

    ros = config["ros"]
    arm_topic = str(ros["arm_topic"])
    cmd_topic = str(ros["cabinet_cmd_topic"])
    state_topic = str(ros["f103_state_topic"])
    nav_status_topic = str(ros["nav_status_topic"])

    class ArmSearchNode(Node):
        def __init__(self):
            super().__init__("arm_apriltag_s0_search")
            reliable = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            # Match the proven arm_cmd_cli publisher profile exactly.  The
            # bridge subscription is reliable/volatile, which is also what a
            # depth-only publisher resolves to on Foxy.
            self.arm_pub = self.create_publisher(String, arm_topic, 10)
            self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
            self.create_subscription(String, state_topic, self._on_state, reliable)
            self.create_subscription(GoalStatusArray, nav_status_topic, self._on_nav, 10)
            self.mode_client = self.create_client(SetBool, str(ros["cabinet_mode_service"]))
            self.last_state: dict[str, Any] = {}
            self.last_state_at = 0.0
            self.nav_active = False
            self.nav_status_at = 0.0
            self.lease_acquired = False

        def _on_state(self, message: String) -> None:
            try:
                self.last_state = parse_f103_state(message.data)
                self.last_state_at = time.monotonic()
            except Exception:
                return

        def _on_nav(self, message: GoalStatusArray) -> None:
            self.nav_active = any(int(item.status) in ACTIVE_NAV_STATUSES for item in message.status_list)
            self.nav_status_at = time.monotonic()

        def state_age(self) -> float:
            return float("inf") if self.last_state_at <= 0.0 else time.monotonic() - self.last_state_at

        def arm_publisher_count(self) -> int:
            return int(self.count_publishers(arm_topic))

        def arm_subscriber_count(self) -> int:
            return int(self.count_subscribers(arm_topic))

        def nav_status_publisher_count(self) -> int:
            return int(self.count_publishers(nav_status_topic))

        def arm_command_count(self) -> int | None:
            value = self.last_state.get("arm_cmd_count")
            try:
                return int(value)
            except (TypeError, ValueError):
                return None

        def publish_zero(self) -> None:
            self.cmd_pub.publish(Twist())

        def publish_arm_payload(self, payload: str) -> None:
            before = self.arm_command_count()
            message = String()
            message.data = payload
            self.arm_pub.publish(message)

            safety = config["safety"]
            hold_s = max(0.20, float(safety.get("arm_command_delivery_hold_s", 0.20)))
            timeout_s = max(hold_s, float(safety.get("arm_command_ack_timeout_s", 1.50)))
            deadline = time.monotonic() + timeout_s
            hold_deadline = time.monotonic() + hold_s
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                current = self.arm_command_count()
                if before is not None and current is not None and current > before:
                    while time.monotonic() < hold_deadline:
                        rclpy.spin_once(self, timeout_sec=0.02)
                    return
            raise SearchFailure(
                f"bridge did not accept arm command within {timeout_s:.2f}s: {payload[:80]}"
            )

        def publish_pose(self, pwm: Sequence[int], duration_ms: int) -> None:
            self.publish_arm_payload(build_arm_payload(pwm, duration_ms))

        def publish_s0(self, s0_pwm: int, duration_ms: int) -> None:
            self.publish_arm_payload(build_s0_payload(s0_pwm, duration_ms))

        def publish_servo_targets(
            self,
            targets: Sequence[tuple[int, int]],
            duration_ms: int,
        ) -> None:
            self.publish_arm_payload(build_servo_targets_payload(targets, duration_ms))

        def stop_s0(self) -> None:
            self.publish_arm_payload("#000PDST!")

        def set_cabinet_mode(self, enabled: bool, timeout_s: float = 3.0) -> bool:
            if not self.mode_client.wait_for_service(timeout_sec=timeout_s):
                return False
            request = SetBool.Request()
            request.data = bool(enabled)
            future = self.mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
            result = future.result() if future.done() else None
            if result is not None and bool(result.success):
                self.lease_acquired = bool(enabled)
                return True
            return False

    rclpy.init(args=None)
    return rclpy, ArmSearchNode()


class ArmAprilTagSearch:
    def __init__(self, config_path: Path, execute: bool, show: bool = False):
        self.config_path = config_path
        self.config = load_yaml(config_path)
        self.execute = bool(execute)
        self.show = bool(show)
        self.home_pwm = [int(value) for value in self.config["arm"]["home_pwm"]]
        if self.home_pwm != HOME_PWM:
            raise ValueError(f"home_pwm must be {HOME_PWM}")
        self.segments = build_search_segments(self.config)
        self.min_positioning_segments = build_min_positioning_segments(self.config)
        self.status_path = Path(self.config["output"]["status_file"])
        self.capture_dir = Path(self.config["output"]["capture_dir"])
        self.camera = CameraPipeline(self.config)
        self.rclpy: Any = None
        self.node: Any = None
        self.stop_requested = False
        self.latest_frame: np.ndarray | None = None
        self.latest_tag: dict[str, Any] | None = None
        self.state = "INIT"
        self.reason = ""
        self.search_started_at = 0.0
        self.search_elapsed_final: float | None = None
        self.search_detection_frames = 0
        self.search_valid_frames = 0
        self.arm_command_retries = 0
        self.other_axes_restore_count = 0
        self.other_axes_command_retries = 0
        self.other_axes_resume_count = 0
        self.other_axes_home_command_count = 0
        self.other_axes_home_refresh_count = 0
        self.last_other_axes_home_command_at = 0.0
        self.last_other_axes_home_context = ""
        self.last_restored_axes: list[int] = []
        self.last_zero_publish_at = 0.0
        self.preview_open = False
        self.preview_opened_at = 0.0
        self.preview_was_visible = False
        self.preview_tags: list[dict[str, Any]] = []
        self.last_preview_at = 0.0
        self.last_preview_detection_at = 0.0
        self.preview_close_reason = ""
        self.final_s0_pwm: int | None = None
        self.aligned = False
        self.aligned_tag_distance_mm: float | None = None
        self.aligned_tag_distance_spread_mm: float | None = None
        self.navigation_status_fallback = ""

    def request_stop(self, *_args: Any) -> None:
        self.stop_requested = True

    def write_status(self, state: str, **extra: Any) -> None:
        self.state = state
        payload: dict[str, Any] = {
            "state": state,
            "updated_at": time.time(),
            "execute": self.execute,
            "show": self.show,
            "preview_close_reason": self.preview_close_reason,
            "reason": self.reason,
            "search_elapsed_s": (
                None
                if self.search_started_at <= 0.0
                else round(
                    self.search_elapsed_final
                    if self.search_elapsed_final is not None
                    else time.monotonic() - self.search_started_at,
                    3,
                )
            ),
            "search_detection_frames": self.search_detection_frames,
            "search_valid_frames": self.search_valid_frames,
            "arm_command_retries": self.arm_command_retries,
            "other_axes_restore_count": self.other_axes_restore_count,
            "other_axes_command_retries": self.other_axes_command_retries,
            "other_axes_resume_count": self.other_axes_resume_count,
            "other_axes_home_command_count": self.other_axes_home_command_count,
            "other_axes_home_refresh_count": self.other_axes_home_refresh_count,
            "last_other_axes_home_context": self.last_other_axes_home_context,
            "last_other_axes_home_command_age_s": (
                None
                if self.last_other_axes_home_command_at <= 0.0
                else round(time.monotonic() - self.last_other_axes_home_command_at, 3)
            ),
            "last_restored_axes": self.last_restored_axes,
            "other_axes_pwm": (
                None
                if self.current_servo_pwm() is None
                else self.current_servo_pwm()[1:SERVO_COUNT]
            ),
            "s0_pwm": self.current_s0_pwm(),
            "tag_id": None if self.latest_tag is None else int(self.latest_tag["id"]),
            "tag_center_x_px": None if self.latest_tag is None else round(tag_center_x(self.latest_tag), 2),
            "tag_distance_mm": (
                None if self.aligned_tag_distance_mm is None else round(self.aligned_tag_distance_mm, 2)
            ),
            "tag_distance_spread_mm": (
                None
                if self.aligned_tag_distance_spread_mm is None
                else round(self.aligned_tag_distance_spread_mm, 2)
            ),
            "frame_center_x_px": None
            if self.latest_frame is None
            else round(self.latest_frame.shape[1] * 0.5, 2),
            "aligned": self.aligned,
            "navigation_status_fallback": self.navigation_status_fallback,
        }
        payload.update(extra)
        atomic_write_json(self.status_path, payload)

    def spin_once(self, timeout_s: float = 0.0) -> None:
        if self.rclpy is not None and self.node is not None and self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=timeout_s)
            if self.node.lease_acquired:
                now = time.monotonic()
                period_s = max(
                    0.05,
                    float(self.config["safety"].get("cabinet_zero_publish_period_s", 0.25)),
                )
                if now - self.last_zero_publish_at >= period_s:
                    self.node.publish_zero()
                    self.last_zero_publish_at = now
            self.update_preview()

    def open_preview(self) -> None:
        if not self.show:
            return
        if sys.platform.startswith("linux") and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        ):
            raise SearchFailure(
                "--show requires DISPLAY or WAYLAND_DISPLAY; use DISPLAY=:0 on the board desktop"
            )
        try:
            cv2.namedWindow(PREVIEW_WINDOW_NAME, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(PREVIEW_WINDOW_NAME, 960, 540)
        except cv2.error as exc:
            raise SearchFailure(f"cannot open preview window: {exc}") from exc
        self.preview_open = True
        self.preview_opened_at = time.monotonic()
        self.preview_was_visible = False

    def close_preview(self) -> None:
        if not self.preview_open:
            return
        try:
            cv2.destroyWindow(PREVIEW_WINDOW_NAME)
            cv2.waitKey(1)
        except cv2.error:
            pass
        self.preview_open = False
        self.preview_opened_at = 0.0
        self.preview_was_visible = False

    def update_preview(
        self,
        frame: np.ndarray | None = None,
        detection: dict[str, Any] | None = None,
        *,
        force: bool = False,
    ) -> None:
        if not self.preview_open:
            return
        now = time.monotonic()
        close_grace_s = max(
            0.5,
            float(self.config["camera"].get("preview_close_grace_s", 1.0)),
        )
        if (
            self.preview_was_visible
            and now - self.preview_opened_at >= close_grace_s
        ):
            try:
                if cv2.getWindowProperty(PREVIEW_WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    self.stop_requested = True
                    raise SearchFailure("preview window was closed")
            except cv2.error as exc:
                self.stop_requested = True
                raise SearchFailure("preview window was closed") from exc
        fps = max(1.0, float(self.config["camera"].get("fps", 15)))
        if frame is None and not force and now - self.last_preview_at < 1.0 / fps:
            return
        if frame is None:
            frame = self.camera.read()
        self.latest_frame = frame
        detect_period_s = max(0.05, float(self.config["camera"].get("preview_detect_period_s", 0.10)))
        if detection is None and (force or now - self.last_preview_detection_at >= detect_period_s):
            detection = self.camera.detect(frame)
        if detection is not None:
            self.preview_tags = list(detection.get("tags", []))
            self.last_preview_detection_at = now

        display = frame.copy()
        height, width = display.shape[:2]
        center_x = int(round(width * 0.5))
        cv2.line(display, (center_x, 0), (center_x, height - 1), (0, 255, 255), 2)
        for tag in self.preview_tags:
            corners = np.rint(np.asarray(tag["corners"])).astype(np.int32)
            valid = bool(tag.get("valid"))
            color = (0, 220, 0) if valid else (0, 165, 255)
            cv2.polylines(display, [corners], True, color, 2, cv2.LINE_AA)
            tag_x = int(round(tag_center_x(tag)))
            if valid:
                cv2.line(display, (tag_x, 0), (tag_x, height - 1), color, 1)
            cv2.putText(
                display,
                f"ID {int(tag['id'])}{'' if valid else ' invalid'}",
                tuple(corners[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                color,
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            display,
            f"{self.state}  S0={self.current_s0_pwm()}  Q/Esc: stop",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (20, 20, 230),
            2,
            cv2.LINE_AA,
        )
        display = self.decorate_preview(display)
        cv2.imshow(PREVIEW_WINDOW_NAME, display)
        key = cv2.waitKey(1) & 0xFF
        self.last_preview_at = now
        if key in (ord("q"), ord("Q"), 27):
            self.stop_requested = True
            raise SearchFailure("preview stop requested")
        try:
            visibility = cv2.getWindowProperty(PREVIEW_WINDOW_NAME, cv2.WND_PROP_VISIBLE)
            if visibility >= 1:
                self.preview_was_visible = True
            elif (
                self.preview_was_visible
                and now - self.preview_opened_at >= close_grace_s
            ):
                self.stop_requested = True
                raise SearchFailure("preview window was closed")
        except cv2.error as exc:
            if (
                self.preview_was_visible
                and now - self.preview_opened_at >= close_grace_s
            ):
                self.stop_requested = True
                raise SearchFailure("preview window was closed") from exc

    def decorate_preview(self, display: np.ndarray) -> np.ndarray:
        """Allow integrated tools to add overlays without changing S0 search behavior."""
        return display

    def current_s0_pwm(self) -> int | None:
        values = self.current_servo_pwm()
        return None if values is None else int(values[0])

    def current_servo_pwm(self) -> list[int] | None:
        if self.node is None:
            return None
        values = self.node.last_state.get("servo_pwm")
        if isinstance(values, list) and len(values) >= SERVO_COUNT:
            return [int(value) for value in values[:SERVO_COUNT]]
        return None

    def other_axes_needing_restore(self) -> list[int]:
        values = self.current_servo_pwm()
        if values is None:
            raise SearchFailure("F103 servo PWM feedback is unavailable")
        return off_home_axes(
            values,
            self.home_pwm,
            int(self.config["safety"].get("other_axes_tolerance_pwm", 12)),
        )

    def assert_safe_state(self, *, require_home_axes: bool) -> None:
        if self.stop_requested:
            raise SearchFailure("interrupted")
        if self.node is None:
            raise SearchFailure("ROS node is unavailable")
        max_state_age = float(self.config["safety"]["f103_state_timeout_s"])
        if self.node.state_age() > max_state_age:
            raise SearchFailure("F103 state is stale")
        state = self.node.last_state
        if int(state.get("estop", 0)):
            raise SearchFailure("F103 ESTOP is active")
        if self.node.arm_subscriber_count() <= 0:
            raise SearchFailure("/arm_cmd has no bridge subscriber")
        if self.execute and self.node.arm_publisher_count() != 1:
            raise SearchFailure("another /arm_cmd publisher is active")
        if self.node.nav_active:
            raise SearchFailure("navigation action is active")
        wheel_limit = abs(int(self.config["safety"]["wheel_stop_tolerance_mm_s"]))
        targets = state.get("target_wheels_mm_s", [])
        feedback = state.get("last_wheel_mm_s", [])
        if any(abs(int(value)) > wheel_limit for value in list(targets) + list(feedback)):
            raise SearchFailure("chassis is not stationary")
        if require_home_axes:
            if self.other_axes_needing_restore():
                raise SearchFailure("S1-S5 are not at the required home pose")

    def wait_state(self, timeout_s: float, *, require_home_axes: bool = False) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            if self.node is not None and self.node.state_age() <= float(self.config["safety"]["f103_state_timeout_s"]):
                self.assert_safe_state(require_home_axes=require_home_axes)
                return
        raise SearchFailure("timed out waiting for fresh F103 state")

    def wait_pose(
        self,
        expected: Sequence[int],
        timeout_s: float,
        *,
        monitor_other_axes: bool = False,
    ) -> None:
        deadline = time.monotonic() + timeout_s
        tolerance = abs(int(self.config["safety"]["servo_target_tolerance_pwm"]))
        values: list[int] | None = None
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            self.assert_safe_state(require_home_axes=False)
            if monitor_other_axes and self.restore_other_axes_if_needed("s0_motion"):
                raise OtherAxesRestored()
            if monitor_other_axes:
                self.refresh_other_axes_home_if_due("s0_motion")
            values = self.node.last_state.get("servo_pwm") if self.node is not None else None
            if isinstance(values, list) and len(values) == SERVO_COUNT:
                if all(abs(int(values[index]) - int(expected[index])) <= tolerance for index in range(SERVO_COUNT)):
                    return
        raise SearchFailure(
            f"servo motion did not reach the commanded pose; expected={list(expected)}, actual={values}"
        )

    def wait_s0_stopped(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        stable_required_s = max(
            0.10,
            float(self.config["safety"].get("other_axes_s0_stable_s", 0.20)),
        )
        stable_since = 0.0
        previous: int | None = None
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            self.assert_safe_state(require_home_axes=False)
            current = self.current_s0_pwm()
            if current is None:
                continue
            if previous is not None and abs(current - previous) <= 2:
                stable_since = time.monotonic() if stable_since <= 0.0 else stable_since
                if time.monotonic() - stable_since >= stable_required_s:
                    return True
            else:
                stable_since = 0.0
            previous = current
        return False

    def pause_s0_for_other_axes(self) -> None:
        timeout_s = float(self.config["safety"].get("other_axes_s0_stop_timeout_s", 1.0))
        for _attempt in range(2):
            self.stop_s0()
            if self.wait_s0_stopped(timeout_s):
                return
        raise SearchFailure("S0 did not stop before restoring S1-S5")

    def wait_other_axes_home(self, axes: Sequence[int], timeout_s: float) -> None:
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        tolerance = abs(int(self.config["safety"].get("other_axes_tolerance_pwm", 12)))
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            self.assert_safe_state(require_home_axes=False)
            values = self.current_servo_pwm()
            if values is not None and all(
                abs(values[index] - self.home_pwm[index]) <= tolerance
                for index in axes
            ):
                return
        raise SearchFailure(f"S1-S5 restore timed out for axes {list(axes)}")

    def restore_other_axes_if_needed(self, context: str) -> bool:
        if not self.execute:
            return False
        axes = self.other_axes_needing_restore()
        if not axes:
            return False
        max_events = max(
            1,
            int(self.config["safety"].get("other_axes_restore_max_events", 3)),
        )
        if self.other_axes_restore_count >= max_events:
            raise SearchFailure(
                f"S1-S5 exceeded {max_events} automatic restore events"
            )

        before = self.current_servo_pwm()
        self.pause_s0_for_other_axes()
        duration_ms = int(
            self.config["safety"].get("other_axes_restore_duration_ms", 2000)
        )
        timeout_s = float(
            self.config["safety"].get("other_axes_restore_timeout_s", 3.5)
        )
        retries = max(
            0,
            int(self.config["safety"].get("other_axes_restore_retry_count", 2)),
        )
        targets = [(index, self.home_pwm[index]) for index in axes]
        last_error: Exception | None = None
        self.other_axes_restore_count += 1
        self.last_restored_axes = list(axes)
        self.write_status(
            self.state,
            correction_state="RESTORE_OTHER_AXES",
            correction_context=str(context),
            correction_axes=list(axes),
            correction_before=None if before is None else before[1:SERVO_COUNT],
        )
        for attempt in range(retries + 1):
            if self.node is None:
                raise SearchFailure("ROS node is unavailable")
            self.node.publish_servo_targets(targets, duration_ms)
            try:
                self.wait_other_axes_home(axes, timeout_s)
                self.write_status(
                    self.state,
                    correction_state="OTHER_AXES_HOME",
                    correction_context=str(context),
                    correction_axes=list(axes),
                )
                return True
            except SearchFailure as exc:
                last_error = exc
                if attempt >= retries:
                    break
                self.other_axes_command_retries += 1
        raise SearchFailure(
            f"S1-S5 restore failed after {retries + 1} command attempt(s): {last_error}"
        )

    def send_other_axes_home(
        self,
        context: str,
        duration_ms: int,
        *,
        periodic: bool,
    ) -> None:
        if not self.execute:
            return
        if self.node is None:
            raise SearchFailure("ROS node is unavailable")
        self.node.publish_servo_targets(
            other_axes_home_targets(self.home_pwm),
            duration_ms,
        )
        self.other_axes_home_command_count += 1
        if periodic:
            self.other_axes_home_refresh_count += 1
        self.last_other_axes_home_command_at = time.monotonic()
        self.last_other_axes_home_context = str(context)
        self.write_status(
            self.state,
            correction_state=(
                "REFRESH_OTHER_AXES_HOME"
                if periodic
                else "FORCE_OTHER_AXES_HOME"
            ),
            correction_context=str(context),
        )

    def establish_other_axes_home(self) -> None:
        safety = self.config["safety"]
        duration_ms = int(safety.get("other_axes_initial_home_duration_ms", 2000))
        self.send_other_axes_home(
            "before_search",
            duration_ms,
            periodic=False,
        )
        hold_s = max(
            float(duration_ms) / 1000.0,
            float(safety.get("other_axes_initial_home_hold_s", 2.2)),
        )
        deadline = time.monotonic() + hold_s
        while time.monotonic() < deadline:
            self.spin_once(0.05)
            self.assert_safe_state(require_home_axes=False)
        self.assert_safe_state(require_home_axes=True)

    def refresh_other_axes_home_if_due(self, context: str) -> bool:
        if not self.execute or self.search_started_at <= 0.0:
            return False
        safety = self.config["safety"]
        max_count = max(
            0,
            int(safety.get("other_axes_home_refresh_max_count", 10)),
        )
        if self.other_axes_home_refresh_count >= max_count:
            return False
        interval_s = max(
            0.5,
            float(safety.get("other_axes_home_refresh_interval_s", 2.0)),
        )
        if (
            self.last_other_axes_home_command_at > 0.0
            and time.monotonic() - self.last_other_axes_home_command_at < interval_s
        ):
            return False
        self.send_other_axes_home(
            context,
            int(safety.get("other_axes_home_refresh_duration_ms", 800)),
            periodic=True,
        )
        return True

    def read_detection(self) -> dict[str, Any]:
        self.spin_once(0.0)
        self.restore_other_axes_if_needed("detection")
        self.refresh_other_axes_home_if_due("detection")
        self.assert_safe_state(require_home_axes=True)
        frame = self.camera.read()
        self.latest_frame = frame
        result = self.camera.detect(frame)
        self.update_preview(frame, result, force=True)
        return result

    def move_s0(self, target_pwm: int, duration_ms: int) -> None:
        if self.node is None:
            raise SearchFailure("ROS node is unavailable")
        self.node.publish_s0(target_pwm, duration_ms)

    def move_s0_and_wait(
        self,
        target_pwm: int,
        duration_ms: int,
        attempt_timeout_s: float,
        *,
        hard_deadline: float | None = None,
        retry_count: int | None = None,
        monitor_other_axes: bool = False,
    ) -> None:
        retries = (
            int(self.config["safety"].get("arm_command_retry_count", 1))
            if retry_count is None
            else int(retry_count)
        )
        retries = max(0, retries)
        expected = s0_pose(target_pwm, self.home_pwm)
        backoff_s = max(0.0, float(self.config["safety"].get("arm_command_retry_backoff_s", 0.25)))
        last_error: Exception | None = None
        attempt = 0
        while attempt <= retries:
            if hard_deadline is not None and time.monotonic() >= hard_deadline:
                break
            self.move_s0(target_pwm, duration_ms)
            timeout_s = max(0.05, float(attempt_timeout_s))
            if hard_deadline is not None:
                timeout_s = min(timeout_s, max(0.05, hard_deadline - time.monotonic()))
            try:
                self.wait_pose(
                    expected,
                    timeout_s,
                    monitor_other_axes=monitor_other_axes,
                )
                return
            except OtherAxesRestored:
                self.other_axes_resume_count += 1
                self.write_status(
                    self.state,
                    correction_state="RESUME_S0_TARGET",
                    resume_target_pwm=int(target_pwm),
                )
                continue
            except SearchFailure as exc:
                last_error = exc
                if attempt >= retries:
                    break
                attempt += 1
                self.arm_command_retries += 1
                self.write_status(
                    self.state,
                    retry_target_pwm=int(target_pwm),
                    retry_attempt=attempt,
                )
                retry_deadline = time.monotonic() + backoff_s
                while time.monotonic() < retry_deadline:
                    self.spin_once(0.02)
        if hard_deadline is not None and time.monotonic() >= hard_deadline:
            raise SearchFailure(f"S0 target {int(target_pwm)} missed the active deadline")
        raise SearchFailure(
            f"S0 target {int(target_pwm)} was not reached after {retries + 1} command attempt(s): {last_error}"
        )

    def stop_s0(self) -> None:
        if self.node is not None:
            self.node.stop_s0()

    def allow_missing_navigation_status(self) -> bool:
        return False

    def preflight(self) -> None:
        self.write_status("WAIT_ROS")
        self.wait_state(float(self.config["safety"]["ready_timeout_s"]))
        if self.node is not None and self.node.nav_status_publisher_count() > 0:
            deadline = time.monotonic() + float(self.config["safety"]["nav_status_timeout_s"])
            while self.node.nav_status_at <= 0.0 and time.monotonic() < deadline:
                self.spin_once(0.05)
            if self.node.nav_status_at <= 0.0 and not self.allow_missing_navigation_status():
                raise SearchFailure("navigation status is unavailable")
        self.assert_safe_state(require_home_axes=False)
        discovery_settle_s = max(
            0.20,
            float(self.config["safety"].get("arm_discovery_settle_s", 0.30)),
        )
        discovery_deadline = time.monotonic() + discovery_settle_s
        while time.monotonic() < discovery_deadline:
            self.spin_once(0.05)
        self.write_status(
            "READY",
            arm_publishers=self.node.arm_publisher_count() if self.node is not None else 0,
            arm_subscribers=self.node.arm_subscriber_count() if self.node is not None else 0,
            arm_cmd_count=self.node.arm_command_count() if self.node is not None else None,
        )

    def search(self) -> int:
        arm = self.config["arm"]
        search = self.config["search"]
        self.establish_other_axes_home()
        self.write_status("MOVE_TO_MIN")
        for step_index, segment in enumerate(self.min_positioning_segments, start=1):
            self.write_status(
                "MOVE_TO_MIN",
                min_position_step=step_index,
                min_position_step_count=len(self.min_positioning_segments),
                min_position_target_pwm=segment.target_pwm,
            )
            self.move_s0_and_wait(
                segment.target_pwm,
                segment.duration_ms,
                segment.timeout_s,
                monitor_other_axes=True,
            )
            self.write_status(
                "MOVE_TO_MIN",
                min_position_step=step_index,
                min_position_step_count=len(self.min_positioning_segments),
                min_position_target_pwm=segment.target_pwm,
                min_position_actual_pwm=self.current_s0_pwm(),
            )

        confirmation = TagConfirmation(
            int(search["window_frames"]),
            int(search["required_frames"]),
            int(self.config["apriltag"]["preferred_id"]),
        )
        self.search_started_at = time.monotonic()
        deadline = self.search_started_at + float(search["max_duration_s"])
        detect_interval = float(search["detect_interval_s"])
        self.write_status("SEARCH")

        for step_index, segment in enumerate(self.segments, start=1):
            if time.monotonic() >= deadline:
                break
            self.restore_other_axes_if_needed("before_search_segment")
            self.refresh_other_axes_home_if_due("before_search_segment")
            self.assert_safe_state(require_home_axes=True)
            remaining_s = max(0.0, deadline - time.monotonic())
            pose_timeout_s = min(
                remaining_s,
                float(segment.duration_ms) / 1000.0
                + float(search.get("step_motion_timeout_margin_s", 0.60)),
            )
            if pose_timeout_s <= 0.0:
                break
            self.move_s0_and_wait(
                segment.target_pwm,
                segment.duration_ms,
                pose_timeout_s,
                hard_deadline=deadline,
                monitor_other_axes=True,
            )

            settle_deadline = min(
                deadline,
                time.monotonic() + max(0.0, float(search.get("step_settle_s", 0.10))),
            )
            while time.monotonic() < settle_deadline:
                self.spin_once(0.02)
                self.restore_other_axes_if_needed("search_settle")
                self.refresh_other_axes_home_if_due("search_settle")
                self.assert_safe_state(require_home_axes=True)

            frames_this_step = max(1, int(search.get("stationary_frames_per_step", 3)))
            for frame_index in range(1, frames_this_step + 1):
                if time.monotonic() >= deadline:
                    break
                result = self.read_detection()
                self.search_detection_frames += 1
                valid_tags = [item for item in result.get("tags", []) if bool(item.get("valid"))]
                if valid_tags:
                    self.search_valid_frames += 1
                candidate = confirmation.observe(result.get("tags", []))
                self.write_status(
                    "SEARCH",
                    search_step=step_index,
                    search_step_count=len(self.segments),
                    search_target_pwm=segment.target_pwm,
                    search_frame=frame_index,
                    detected_ids=[int(item["id"]) for item in result.get("tags", [])],
                    valid_ids=[int(item["id"]) for item in valid_tags],
                )
                if candidate is not None:
                    self.latest_tag = candidate
                    self.stop_s0()
                    time.sleep(float(search["stop_settle_s"]))
                    found_pwm = self.current_s0_pwm()
                    if found_pwm is None:
                        raise SearchFailure("S0 feedback unavailable after tag lock")
                    self.search_elapsed_final = time.monotonic() - self.search_started_at
                    self.write_status("TAG_LOCKED", locked_tag_id=int(candidate["id"]))
                    return self.align(int(candidate["id"]), found_pwm)
                frame_deadline = min(deadline, time.monotonic() + detect_interval)
                while time.monotonic() < frame_deadline:
                    self.spin_once(0.01)
        self.search_elapsed_final = time.monotonic() - self.search_started_at
        raise SearchFailure("AprilTag not found before full search completed")

    def align(self, locked_tag_id: int, origin_pwm: int) -> int:
        alignment = self.config["alignment"]
        deadline = time.monotonic() + float(alignment["timeout_s"])
        lost_since = 0.0
        stable_frames = 0
        required_frames = max(1, int(alignment["stable_frames"]))
        window_frames = max(required_frames, int(alignment.get("stable_window_frames", required_frames)))
        stable_observations: collections.deque[bool] = collections.deque(maxlen=window_frames)
        next_detect_at = 0.0
        next_command_at = 0.0
        commanded_pwm = int(origin_pwm)
        guard = DivergenceGuard(
            float(alignment["divergence_increase_px"]),
            int(alignment["divergence_consecutive"]),
        )
        pending_evaluation = False
        stable_distances: collections.deque[float | None] = collections.deque(maxlen=window_frames)
        self.write_status("ALIGN", locked_tag_id=locked_tag_id, origin_s0_pwm=origin_pwm)
        while time.monotonic() < deadline:
            self.spin_once(0.0)
            self.restore_other_axes_if_needed("alignment")
            self.refresh_other_axes_home_if_due("alignment")
            self.assert_safe_state(require_home_axes=True)
            if time.monotonic() < next_detect_at:
                time.sleep(0.01)
                continue
            result = self.read_detection()
            next_detect_at = time.monotonic() + float(alignment["detect_interval_s"])
            tag = next(
                (
                    item
                    for item in result.get("tags", [])
                    if bool(item.get("valid")) and int(item.get("id", -1)) == locked_tag_id
                ),
                None,
            )
            now = time.monotonic()
            if tag is None:
                stable_observations.append(False)
                stable_distances.append(None)
                stable_frames = sum(stable_observations)
                lost_since = now if lost_since <= 0.0 else lost_since
                if now - lost_since > float(alignment["lost_timeout_s"]):
                    raise SearchFailure("locked AprilTag was lost during alignment")
                self.write_status(
                    "ALIGN",
                    stable_frames=stable_frames,
                    stable_window=list(stable_observations),
                )
                continue

            self.latest_tag = tag
            lost_since = 0.0
            error_px = tag_center_x(tag) - self.latest_frame.shape[1] * 0.5
            if pending_evaluation:
                if guard.evaluate(error_px):
                    raise SearchFailure("S0 alignment diverged")
                pending_evaluation = False
            if abs(error_px) <= float(alignment["tolerance_px"]):
                stable_observations.append(True)
                distance = tag.get("distance_mm")
                if distance is not None and math.isfinite(float(distance)) and float(distance) > 0.0:
                    stable_distances.append(float(distance))
                else:
                    stable_distances.append(None)
                stable_frames = sum(stable_observations)
                self.write_status(
                    "ALIGN",
                    error_px=round(error_px, 2),
                    stable_frames=stable_frames,
                    stable_window=list(stable_observations),
                )
                if alignment_consensus(stable_observations, required_frames):
                    self.stop_s0()
                    self.final_s0_pwm = self.current_s0_pwm() or commanded_pwm
                    values = [value for value in stable_distances if value is not None]
                    if values:
                        self.aligned_tag_distance_mm = float(np.median(values))
                        self.aligned_tag_distance_spread_mm = float(max(values) - min(values))
                    self.aligned = True
                    self.write_status(
                        "ALIGNED",
                        error_px=round(error_px, 2),
                        stable_frames=stable_frames,
                        stable_window=list(stable_observations),
                    )
                    return 0
                continue

            stable_frames = 0
            stable_observations.clear()
            stable_distances.clear()
            if now < next_command_at:
                continue
            target = alignment_target(commanded_pwm, origin_pwm, error_px, self.config)
            if target == commanded_pwm:
                raise SearchFailure("S0 alignment reached configured limit")
            guard.command_sent(error_px)
            pending_evaluation = True
            commanded_pwm = target
            self.move_s0_and_wait(
                target,
                int(alignment["command_duration_ms"]),
                float(alignment.get("command_timeout_s", 1.0)),
                hard_deadline=deadline,
                monitor_other_axes=True,
            )
            next_command_at = now + float(alignment["command_interval_s"])
            self.write_status("ALIGN", error_px=round(error_px, 2), command_s0_pwm=target)
        raise SearchFailure("S0 alignment timed out")

    def save_capture(self) -> None:
        if self.latest_frame is None:
            return
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        raw_path = self.capture_dir / f"{stamp}_raw.jpg"
        annotated_path = self.capture_dir / f"{stamp}_annotated.jpg"
        cv2.imwrite(str(raw_path), self.latest_frame)
        annotated = self.latest_frame.copy()
        height, width = annotated.shape[:2]
        center_x = int(round(width * 0.5))
        cv2.line(annotated, (center_x, 0), (center_x, height - 1), (0, 255, 255), 2)
        if self.latest_tag is not None:
            corners = np.rint(np.asarray(self.latest_tag["corners"])).astype(np.int32)
            cv2.polylines(annotated, [corners], True, (0, 255, 0), 2, cv2.LINE_AA)
            tag_x = int(round(tag_center_x(self.latest_tag)))
            cv2.line(annotated, (tag_x, 0), (tag_x, height - 1), (0, 220, 0), 2)
            cv2.putText(
                annotated,
                f"AprilTag {int(self.latest_tag['id'])}",
                tuple(corners[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            annotated,
            f"{self.state}: {self.reason or 'ok'}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (20, 20, 230),
            2,
            cv2.LINE_AA,
        )
        cv2.imwrite(str(annotated_path), annotated)
        self.write_status("ALIGNED" if self.aligned else self.state, raw_image=str(raw_path), annotated_image=str(annotated_path))

    def wait_for_manual_preview_close(self) -> None:
        if not self.show or not self.preview_open:
            return
        self.write_status("ALIGNED_WAIT_CLOSE", preview_waiting_for_manual_close=True)
        while True:
            if self.stop_requested:
                self.preview_close_reason = "stop signal received"
                self.stop_requested = False
                self.write_status(
                    "ALIGNED_CLOSED",
                    preview_waiting_for_manual_close=False,
                )
                return
            try:
                self.spin_once(0.05)
            except SearchFailure as exc:
                if not is_manual_preview_close(str(exc)):
                    raise
                self.preview_close_reason = str(exc)
                self.stop_requested = False
                self.write_status(
                    "ALIGNED_CLOSED",
                    preview_waiting_for_manual_close=False,
                )
                return
            if self.stop_requested:
                self.preview_close_reason = "stop signal received"
                self.stop_requested = False
                self.write_status(
                    "ALIGNED_CLOSED",
                    preview_waiting_for_manual_close=False,
                )
                return
            self.restore_other_axes_if_needed("aligned_preview")
            self.refresh_other_axes_home_if_due("aligned_preview")
            self.assert_safe_state(require_home_axes=True)

    def after_alignment(self) -> int:
        """Hook for a single-owner workflow that continues after S0 alignment."""
        return 0

    def return_home(self) -> None:
        if self.node is None:
            return
        self.move_s0_and_wait(
            int(self.home_pwm[0]),
            int(self.config["arm"]["return_home_duration_ms"]),
            float(self.config["arm"]["return_home_timeout_s"]),
            retry_count=max(2, int(self.config["safety"].get("arm_command_retry_count", 1))),
        )
        self.final_s0_pwm = int(self.home_pwm[0])

    def run(self) -> int:
        lock_handle = None
        motion_started = False
        result = 1
        try:
            import fcntl

            lock_path = Path(self.config["safety"]["lock_file"])
            lock_path.parent.mkdir(parents=True, exist_ok=True)
            lock_handle = lock_path.open("w", encoding="ascii")
            try:
                fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise SearchFailure("another S0 search process is already running") from exc

            self.rclpy, self.node = create_ros_node(self.config)
            self.camera.open()
            self.open_preview()
            for _ in range(int(self.config["camera"]["warmup_frames"])):
                self.latest_frame = self.camera.read()
                self.update_preview(self.latest_frame, force=True)
                self.spin_once(0.01)
            self.preflight()
            if not self.execute:
                self.write_status("READY", dry_run=True)
                return 0

            self.assert_safe_state(require_home_axes=False)
            if not self.node.set_cabinet_mode(True):
                raise SearchFailure("failed to acquire cabinet_mode")
            # Enabling the lease sends a forced chassis-zero command.  Give
            # the serial/F103 command path time to settle before the first
            # servo command; otherwise that first arm command can be lost.
            mode_settle_deadline = time.monotonic() + max(
                0.0,
                float(self.config["safety"].get("cabinet_mode_settle_s", 1.20)),
            )
            while time.monotonic() < mode_settle_deadline:
                self.spin_once(0.05)
                self.assert_safe_state(require_home_axes=False)
            self.assert_safe_state(require_home_axes=False)
            motion_started = True
            result = self.search()
            if result == 0 and self.aligned:
                result = self.after_alignment()
                if result == 0:
                    self.wait_for_manual_preview_close()
            return result
        except SearchFailure as exc:
            self.reason = str(exc)
            if self.search_started_at > 0.0 and self.search_elapsed_final is None:
                self.search_elapsed_final = time.monotonic() - self.search_started_at
            result = 2
            self.write_status("FAILED")
            return result
        except Exception as exc:
            self.reason = str(exc)
            if self.search_started_at > 0.0 and self.search_elapsed_final is None:
                self.search_elapsed_final = time.monotonic() - self.search_started_at
            result = 1
            self.write_status("FAILED")
            return result
        finally:
            try:
                if motion_started and not self.aligned:
                    try:
                        self.stop_s0()
                    except Exception as exc:
                        self.reason = f"{self.reason}; S0 stop confirmation failed: {exc}".strip("; ")
                    if self.node is not None and self.node.state_age() <= float(self.config["safety"]["f103_state_timeout_s"]):
                        try:
                            self.return_home()
                            self.final_s0_pwm = int(self.home_pwm[0])
                        except Exception as exc:
                            self.reason = f"{self.reason}; home confirmation failed: {exc}".strip("; ")
                self.save_capture()
            finally:
                if self.node is not None:
                    self.node.publish_zero()
                    if self.node.lease_acquired:
                        self.node.set_cabinet_mode(False, timeout_s=2.0)
                    self.node.destroy_node()
                if self.rclpy is not None and self.rclpy.ok():
                    self.rclpy.shutdown()
                self.close_preview()
                self.camera.close()
                if lock_handle is not None:
                    lock_handle.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Sweep S0 for an AprilTag, then center it horizontally.")
    parser.add_argument(
        "--config",
        default=str(ROOT_DIR / "config" / "arm_apriltag_s0_search.yaml"),
        help="YAML configuration path",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="allow arm motion; without this flag only environment checks run",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="show a live annotated camera window; Q/Esc requests a safe stop",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    runner = ArmAprilTagSearch(Path(args.config), bool(args.execute), bool(args.show))
    signal.signal(signal.SIGINT, runner.request_stop)
    signal.signal(signal.SIGTERM, runner.request_stop)
    return runner.run()


if __name__ == "__main__":
    raise SystemExit(main())

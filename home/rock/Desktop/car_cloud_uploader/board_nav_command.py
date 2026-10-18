#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_action_status_default,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


ROOT_DIR = Path(__file__).resolve().parent
RUNTIME_DIR = ROOT_DIR / "runtime" / "cloud_nav_command"
STATUS_PATH = RUNTIME_DIR / "status.json"
PID_PATH = RUNTIME_DIR / "cruise.pid"
LOG_PATH = RUNTIME_DIR / "cruise.log"
GOAL_UPDATE_PATH = RUNTIME_DIR / "goal_update.json"
GOAL_UPDATE_ACK_PATH = RUNTIME_DIR / "goal_update_ack.json"
CABINET_JOB_SCRIPT = ROOT_DIR / "cabinet_inspection_job.py"
CABINET_JOB_TERMINAL_STATES = {"completed", "partial", "failed", "canceled", "timeout"}
CHILD_ACTION_NAMES = (
    "/compute_path_to_pose",
    "/follow_path",
    "/wait",
)
PATH_CHILD_ACTION_NAMES = (
    "/compute_path_to_pose",
    "/follow_path",
)
ACTIVE_CHILD_STATUS_CODES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}
GOAL_PATH_START_TIMEOUT_SEC = 10.0
GOAL_RAW_COMMAND_TIMEOUT_SEC = 6.0
GOAL_PROGRESS_DISTANCE_M = 0.02
ACTION_STATUS_FRESH_TIMEOUT_SEC = 2.5
RETURN_POSE_FRESH_TIMEOUT_SEC = 1.0
GOAL_ARRIVAL_XY_TOLERANCE_M = 0.15
GOAL_ARRIVAL_YAW_TOLERANCE_RAD = 0.10
GOAL_ARRIVAL_CONFIRM_SAMPLES = 3
GOAL_ARRIVAL_CONFIRM_TIMEOUT_SEC = 2.0
GOAL_RESPONSE_TIMEOUT_SEC = 8.0
CONTROLLER_START_TIMEOUT_SEC = 8.0
CONTROLLER_LOST_TIMEOUT_SEC = 6.0
PATH_ACTIVE_MAX_AGE_SEC = 6.0
LIFECYCLE_RECOVERY_TIMEOUT_SEC = 8.0
MAP_WAIT_TIMEOUT_SEC = 5.0
FOOTPRINT_HALF_LENGTH_M = 0.14
FOOTPRINT_HALF_WIDTH_M = 0.129
OCCUPIED_CELL_THRESHOLD = 50
LIFECYCLE_NODE_NAMES = (
    "planner_server",
    "controller_server",
    "recoveries_server",
    "bt_navigator",
)


class NavigationCleanupError(RuntimeError):
    pass


class NavigationStallError(RuntimeError):
    def __init__(self, message: str, cleanup: dict[str, Any]) -> None:
        super().__init__(message)
        self.cleanup = cleanup


class NavigationControlError(NavigationStallError):
    pass


class NavigationRecoveryError(RuntimeError):
    pass


def navigation_control_fault(
    *,
    elapsed_sec: float,
    path_seen: bool,
    path_age_sec: float | None,
    controller_started: bool,
    raw_cmd_age_sec: float | None,
) -> str:
    if not path_seen or path_age_sec is None or path_age_sec > PATH_ACTIVE_MAX_AGE_SEC:
        return ""
    if not controller_started:
        return "controller_not_started" if elapsed_sec >= CONTROLLER_START_TIMEOUT_SEC else ""
    if raw_cmd_age_sec is not None and raw_cmd_age_sec >= CONTROLLER_LOST_TIMEOUT_SEC:
        return "controller_lost"
    return ""


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _world_to_map_cell(
    map_snapshot: dict[str, Any],
    x: float,
    y: float,
) -> tuple[int, int] | None:
    resolution = float(map_snapshot["resolution"])
    origin_yaw = float(map_snapshot.get("origin_yaw", 0.0))
    dx = float(x) - float(map_snapshot["origin_x"])
    dy = float(y) - float(map_snapshot["origin_y"])
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    map_x = (cos_yaw * dx + sin_yaw * dy) / resolution
    map_y = (-sin_yaw * dx + cos_yaw * dy) / resolution
    cell_x = math.floor(map_x)
    cell_y = math.floor(map_y)
    if (
        cell_x < 0
        or cell_y < 0
        or cell_x >= int(map_snapshot["width"])
        or cell_y >= int(map_snapshot["height"])
    ):
        return None
    return int(cell_x), int(cell_y)


def _map_cell_center(
    map_snapshot: dict[str, Any],
    cell_x: int,
    cell_y: int,
) -> tuple[float, float]:
    resolution = float(map_snapshot["resolution"])
    local_x = (float(cell_x) + 0.5) * resolution
    local_y = (float(cell_y) + 0.5) * resolution
    origin_yaw = float(map_snapshot.get("origin_yaw", 0.0))
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    return (
        float(map_snapshot["origin_x"]) + cos_yaw * local_x - sin_yaw * local_y,
        float(map_snapshot["origin_y"]) + sin_yaw * local_x + cos_yaw * local_y,
    )


def _map_cell_corners(
    map_snapshot: dict[str, Any],
    cell_x: int,
    cell_y: int,
) -> tuple[tuple[float, float], ...]:
    resolution = float(map_snapshot["resolution"])
    origin_x = float(map_snapshot["origin_x"])
    origin_y = float(map_snapshot["origin_y"])
    origin_yaw = float(map_snapshot.get("origin_yaw", 0.0))
    cos_yaw = math.cos(origin_yaw)
    sin_yaw = math.sin(origin_yaw)
    corners: list[tuple[float, float]] = []
    for local_x, local_y in (
        (float(cell_x) * resolution, float(cell_y) * resolution),
        ((float(cell_x) + 1.0) * resolution, float(cell_y) * resolution),
        (
            (float(cell_x) + 1.0) * resolution,
            (float(cell_y) + 1.0) * resolution,
        ),
        (float(cell_x) * resolution, (float(cell_y) + 1.0) * resolution),
    ):
        corners.append(
            (
                origin_x + cos_yaw * local_x - sin_yaw * local_y,
                origin_y + sin_yaw * local_x + cos_yaw * local_y,
            )
        )
    return tuple(corners)


def _convex_polygons_intersect(
    first: tuple[tuple[float, float], ...] | list[tuple[float, float]],
    second: tuple[tuple[float, float], ...] | list[tuple[float, float]],
) -> bool:
    for polygon in (first, second):
        for index, (start_x, start_y) in enumerate(polygon):
            end_x, end_y = polygon[(index + 1) % len(polygon)]
            axis_x = -(end_y - start_y)
            axis_y = end_x - start_x
            first_projection = [axis_x * px + axis_y * py for px, py in first]
            second_projection = [axis_x * px + axis_y * py for px, py in second]
            if max(first_projection) < min(second_projection) or max(
                second_projection
            ) < min(first_projection):
                return False
    return True


def validate_pose_on_map(
    map_snapshot: dict[str, Any],
    pose: dict[str, Any],
    *,
    label: str,
) -> dict[str, Any]:
    if not map_snapshot or not map_snapshot.get("data"):
        return {"valid": False, "label": label, "reason": "map_unavailable"}
    x = float(pose["x"])
    y = float(pose["y"])
    yaw = float(pose.get("yaw", 0.0))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    corners: list[tuple[float, float]] = []
    for local_x, local_y in (
        (FOOTPRINT_HALF_LENGTH_M, FOOTPRINT_HALF_WIDTH_M),
        (FOOTPRINT_HALF_LENGTH_M, -FOOTPRINT_HALF_WIDTH_M),
        (-FOOTPRINT_HALF_LENGTH_M, -FOOTPRINT_HALF_WIDTH_M),
        (-FOOTPRINT_HALF_LENGTH_M, FOOTPRINT_HALF_WIDTH_M),
    ):
        corners.append(
            (
                x + cos_yaw * local_x - sin_yaw * local_y,
                y + sin_yaw * local_x + cos_yaw * local_y,
            )
        )
    corner_cells = [_world_to_map_cell(map_snapshot, px, py) for px, py in corners]
    if any(cell is None for cell in corner_cells):
        return {
            "valid": False,
            "label": label,
            "reason": "footprint_outside_map",
            "pose": {"x": x, "y": y, "yaw": yaw},
        }

    cells = [cell for cell in corner_cells if cell is not None]
    min_x = max(0, min(cell[0] for cell in cells) - 1)
    max_x = min(int(map_snapshot["width"]) - 1, max(cell[0] for cell in cells) + 1)
    min_y = max(0, min(cell[1] for cell in cells) - 1)
    max_y = min(int(map_snapshot["height"]) - 1, max(cell[1] for cell in cells) + 1)
    data = map_snapshot["data"]
    width = int(map_snapshot["width"])
    for cell_y in range(min_y, max_y + 1):
        for cell_x in range(min_x, max_x + 1):
            cell_corners = _map_cell_corners(map_snapshot, cell_x, cell_y)
            if not _convex_polygons_intersect(corners, cell_corners):
                continue
            value = int(data[cell_y * width + cell_x])
            if value < 0 or value >= OCCUPIED_CELL_THRESHOLD:
                return {
                    "valid": False,
                    "label": label,
                    "reason": "unknown_cell" if value < 0 else "occupied_cell",
                    "cell": {"x": cell_x, "y": cell_y, "value": value},
                    "pose": {"x": x, "y": y, "yaw": yaw},
                }
    return {
        "valid": True,
        "label": label,
        "reason": "free",
        "pose": {"x": x, "y": y, "yaw": yaw},
    }


def navigation_stall_reason(
    *,
    elapsed_sec: float,
    goal_id: str,
    stabilizer: dict[str, Any],
    active_children: list[dict[str, Any]],
    child_statuses_fresh: bool,
) -> str:
    if elapsed_sec < GOAL_PATH_START_TIMEOUT_SEC or active_children:
        return ""
    if not child_statuses_fresh or not stabilizer:
        return ""
    try:
        status_age_sec = float(stabilizer.get("status_age_sec"))
        raw_cmd_age = float(stabilizer.get("raw_cmd_age"))
    except (TypeError, ValueError):
        return ""
    if status_age_sec > 1.5 or raw_cmd_age < GOAL_RAW_COMMAND_TIMEOUT_SEC:
        return ""
    stabilizer_goal_id = str(stabilizer.get("goal_id") or "")
    if stabilizer_goal_id and stabilizer_goal_id != goal_id:
        return ""
    if not bool(stabilizer.get("action_active")):
        return ""
    if not bool(stabilizer.get("awaiting_goal_path")):
        return ""
    if stabilizer.get("path_goal_id"):
        return ""
    return (
        "NavigateToPose is active but no child action or goal path started "
        f"within {GOAL_PATH_START_TIMEOUT_SEC:.0f}s"
    )


def map_pose_from_transform(
    transform,
    *,
    now_sec: float,
    max_age_sec: float = RETURN_POSE_FRESH_TIMEOUT_SEC,
) -> dict[str, float] | None:
    stamp = transform.header.stamp
    stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
    age_sec = float(now_sec) - stamp_sec
    if stamp_sec <= 0.0 or age_sec < -0.25 or age_sec > max(0.0, float(max_age_sec)):
        return None

    translation = transform.transform.translation
    rotation = transform.transform.rotation
    siny_cosp = 2.0 * (
        float(rotation.w) * float(rotation.z)
        + float(rotation.x) * float(rotation.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        float(rotation.y) ** 2 + float(rotation.z) ** 2
    )
    return {
        "x": float(translation.x),
        "y": float(translation.y),
        "yaw": math.atan2(siny_cosp, cosy_cosp),
        "updated_at": time.monotonic(),
    }


def goal_pose_errors(
    current_pose: dict[str, Any],
    goal_pose: dict[str, Any],
) -> dict[str, float]:
    dx = float(goal_pose["x"]) - float(current_pose["x"])
    dy = float(goal_pose["y"]) - float(current_pose["y"])
    yaw_delta = float(current_pose["yaw"]) - float(goal_pose.get("yaw", 0.0))
    yaw_error = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
    return {
        "distance_m": math.hypot(dx, dy),
        "yaw_error_rad": yaw_error,
    }


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def normalized_yaw_radians(raw: dict[str, Any]) -> float:
    if "yaw_deg" in raw:
        yaw = math.radians(float(raw.get("yaw_deg", 0.0)))
    else:
        yaw = float(raw.get("yaw", 0.0))
        # Older cloud commands sometimes placed degree values in `yaw`.
        if abs(yaw) > math.tau + 1e-6:
            yaw = math.radians(yaw)
    return math.atan2(math.sin(yaw), math.cos(yaw))


def safe_pose_dict(raw: Any) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise RuntimeError("pose must be an object")
    try:
        x = float(raw["x"])
        y = float(raw["y"])
        yaw = normalized_yaw_radians(raw)
    except Exception as exc:
        raise RuntimeError(f"invalid pose: {exc}") from exc
    pose: dict[str, Any] = {"x": x, "y": y, "yaw": yaw}
    label = str(raw.get("label") or "").strip()
    if label:
        pose["label"] = label
    kind = str(raw.get("kind") or "transit").strip().lower()
    if kind not in {"transit", "cabinet"}:
        raise RuntimeError(f"invalid navigation point kind: {kind}")
    pose["kind"] = kind
    if kind == "cabinet":
        try:
            apriltag_id = int(raw.get("apriltag_id"))
        except (TypeError, ValueError) as exc:
            raise RuntimeError("cabinet point requires apriltag_id") from exc
        if apriltag_id not in {10, 11, 12, 13}:
            raise RuntimeError("cabinet apriltag_id must be 10-13")
        pose["apriltag_id"] = apriltag_id
    return pose


def optional_pose_from_json(raw: str) -> dict[str, Any]:
    if not raw:
        return {}
    decoded = json.loads(raw)
    if decoded is None or decoded == {}:
        return {}
    return safe_pose_dict(decoded)


def safe_points(raw: Any) -> list[dict[str, Any]]:
    if not isinstance(raw, list):
        raise RuntimeError("points must be a list")
    points: list[dict[str, Any]] = []
    for index, item in enumerate(raw, start=1):
        pose = safe_pose_dict(item)
        pose["label"] = str(item.get("label") or f"P{index}")
        points.append(pose)
    if not points:
        raise RuntimeError("at least one point is required")
    return points


def safe_task_meta(raw: Any) -> dict[str, Any]:
    payload = raw if isinstance(raw, dict) else {}
    session_id = str(payload.get("session_id") or f"nav-{int(time.time())}-{os.getpid()}").strip()
    generation = str(payload.get("navigation_generation") or "").strip()
    if not generation:
        generation = f"{session_id}-{time.time_ns()}"
    localization_raw = payload.get("localization_initial_pose")
    localization_initial_pose = (
        safe_pose_dict(localization_raw)
        if isinstance(localization_raw, dict) and localization_raw
        else {}
    )
    return {
        "session_id": session_id,
        "navigation_generation": generation,
        "map_name": str(payload.get("map_name") or payload.get("map") or "").strip(),
        "map_source": str(payload.get("map_source") or "selected_map").strip(),
        "route_source": str(payload.get("route_source") or "temporary").strip(),
        "start_source": str(payload.get("start_source") or "current_pose").strip(),
        "localization_initial_pose": localization_initial_pose,
        "goal_refresh": {"enabled": False, "mode": "disabled"},
        "auto_cabinet_inspection": bool(payload.get("auto_cabinet_inspection")),
    }


def read_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp_path.replace(path)


def remove_file(path: Path) -> None:
    try:
        path.unlink()
    except OSError:
        pass


def acknowledge_goal_update(request: dict[str, Any], status: str, message: str) -> None:
    write_json(
        GOAL_UPDATE_ACK_PATH,
        {
            "request_id": str(request.get("request_id") or ""),
            "session_id": str(request.get("session_id") or ""),
            "status": status,
            "message": message,
            "updated_at": utc_now_iso(),
        },
    )


def consume_goal_update(session_id: str) -> dict[str, Any] | None:
    request = read_json(GOAL_UPDATE_PATH)
    if not request:
        return None
    remove_file(GOAL_UPDATE_PATH)
    request_session = str(request.get("session_id") or "").strip()
    if not request_session or request_session != str(session_id or "").strip():
        acknowledge_goal_update(request, "rejected", "navigation session changed")
        return None
    try:
        request["goal_pose"] = safe_pose_dict(request.get("goal_pose"))
    except Exception as exc:
        acknowledge_goal_update(request, "rejected", str(exc))
        return None
    acknowledge_goal_update(request, "accepted", "goal update accepted")
    return request


def merge_status(update: dict[str, Any]) -> dict[str, Any]:
    current = read_json(STATUS_PATH)
    current.update(update)
    current["updated_at"] = utc_now_iso()
    write_json(STATUS_PATH, current)
    return current


def clear_pid_file() -> None:
    try:
        PID_PATH.unlink()
    except OSError:
        pass


def read_pid() -> int | None:
    try:
        value = PID_PATH.read_text(encoding="utf-8").strip()
        return int(value) if value else None
    except Exception:
        return None


def pid_alive(pid: int | None) -> bool:
    if not pid:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def point_labels(points: list[dict[str, Any]]) -> list[str]:
    labels: list[str] = []
    for index, point in enumerate(points, start=1):
        label = str(point.get("label") or f"P{index}")
        labels.append(label)
    return labels


class NavCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("cloud_nav_command_bridge")
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.latest_map_pose: dict[str, float] | None = None
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._amcl_pose_callback,
            10,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )
        self.return_pose_source = ""
        self.last_arrival_confirmation: dict[str, Any] = {}
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.latest_map_snapshot: dict[str, Any] = {}
        self.latest_plan_at = -1.0
        self.latest_plan_pose_count = 0
        self.plan_message_count = 0
        self.latest_raw_cmd_at = -1.0
        self.latest_raw_cmd = {"linear_x": 0.0, "angular_z": 0.0}
        self.raw_cmd_message_count = 0
        self.latest_odom_at = -1.0
        self.latest_scan_at = -1.0
        self.lifecycle_reset_count = 0
        self.lifecycle_last_result: dict[str, Any] = {}
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self._map_callback,
            map_qos,
        )
        self.plan_sub = self.create_subscription(NavPath, "/plan", self._plan_callback, 10)
        self.raw_cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel_nav_raw",
            self._raw_cmd_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.child_action_statuses: dict[str, list[dict[str, Any]]] = {
            action_name: [] for action_name in CHILD_ACTION_NAMES
        }
        self.child_status_received_at: dict[str, float] = {
            action_name: -1.0 for action_name in CHILD_ACTION_NAMES
        }
        self.child_status_subscriptions: list[Any] = []
        self.child_cancel_clients = {}
        for action_name in CHILD_ACTION_NAMES:
            self.child_cancel_clients[action_name] = self.create_client(
                CancelGoal, f"{action_name}/_action/cancel_goal"
            )
        self.lifecycle_manager_client = self.create_client(
            ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes",
        )
        self.lifecycle_state_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in LIFECYCLE_NODE_NAMES
        }
        self.latest_stabilizer_status: dict[str, Any] = {}
        self.stabilizer_status_received_at = -1.0
        self.stabilizer_status_sub = self.create_subscription(
            String,
            "/nav_motion_stabilizer/status",
            self._stabilizer_status_callback,
            10,
        )

    @staticmethod
    def _quat_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
        yaw = float(yaw_rad)
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    @staticmethod
    def _yaw_from_quaternion(orientation) -> float:
        siny_cosp = 2.0 * (float(orientation.w) * float(orientation.z) + float(orientation.x) * float(orientation.y))
        cosy_cosp = 1.0 - 2.0 * (float(orientation.y) ** 2 + float(orientation.z) ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(float(angle)), math.cos(float(angle)))

    def _record_map_pose(self, x: float, y: float, yaw: float) -> None:
        self.latest_map_pose = {
            "x": float(x),
            "y": float(y),
            "yaw": self._normalize_angle(yaw),
            "updated_at": time.monotonic(),
        }

    def _amcl_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        self._record_map_pose(
            float(pose.position.x),
            float(pose.position.y),
            self._yaw_from_quaternion(pose.orientation),
        )

    def _map_callback(self, message: OccupancyGrid) -> None:
        origin = message.info.origin
        self.latest_map_snapshot = {
            "width": int(message.info.width),
            "height": int(message.info.height),
            "resolution": float(message.info.resolution),
            "origin_x": float(origin.position.x),
            "origin_y": float(origin.position.y),
            "origin_yaw": self._yaw_from_quaternion(origin.orientation),
            "data": tuple(int(value) for value in message.data),
            "received_at": time.monotonic(),
        }

    def _plan_callback(self, message: NavPath) -> None:
        self.latest_plan_at = time.monotonic()
        self.latest_plan_pose_count = len(message.poses)
        self.plan_message_count += 1

    def _raw_cmd_callback(self, message: Twist) -> None:
        self.latest_raw_cmd_at = time.monotonic()
        self.latest_raw_cmd = {
            "linear_x": float(message.linear.x),
            "angular_z": float(message.angular.z),
        }
        self.raw_cmd_message_count += 1

    def _odom_callback(self, _message: Odometry) -> None:
        self.latest_odom_at = time.monotonic()

    def _scan_callback(self, _message: LaserScan) -> None:
        self.latest_scan_at = time.monotonic()

    def fresh_tf_map_pose(
        self,
        max_age_sec: float = RETURN_POSE_FRESH_TIMEOUT_SEC,
    ) -> dict[str, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None
        return map_pose_from_transform(
            transform,
            now_sec=self.get_clock().now().nanoseconds / 1_000_000_000.0,
            max_age_sec=max_age_sec,
        )

    def publish_stop(self, executor: SingleThreadedExecutor) -> None:
        message = Twist()
        for _ in range(5):
            self.stop_pub.publish(message)
            executor.spin_once(timeout_sec=0.05)
            time.sleep(0.05)

    @staticmethod
    def _status_goal_id(status) -> str:
        return bytes(status.goal_info.goal_id.uuid).hex()

    def _child_status_callback(
        self, action_name: str, message: GoalStatusArray
    ) -> None:
        active: list[dict[str, Any]] = [
            {
                "goal_id": self._status_goal_id(status),
                "status": int(status.status),
            }
            for status in message.status_list
            if int(status.status) in ACTIVE_CHILD_STATUS_CODES
        ]
        latest_terminal = next(
            (
                {
                    "goal_id": self._status_goal_id(status),
                    "status": int(status.status),
                }
                for status in reversed(message.status_list)
                if int(status.status) not in ACTIVE_CHILD_STATUS_CODES
            ),
            None,
        )
        self.child_action_statuses[action_name] = active + (
            [latest_terminal] if latest_terminal is not None else []
        )
        self.child_status_received_at[action_name] = time.monotonic()

    def _enable_child_status_tracking(self) -> None:
        if self.child_status_subscriptions:
            return
        for action_name in CHILD_ACTION_NAMES:
            self.child_action_statuses[action_name] = []
            self.child_status_received_at[action_name] = -1.0
            self.child_status_subscriptions.append(
                self.create_subscription(
                    GoalStatusArray,
                    f"{action_name}/_action/status",
                    lambda message, name=action_name: self._child_status_callback(
                        name, message
                    ),
                    qos_profile_action_status_default,
                )
            )

    def _disable_child_status_tracking(self) -> None:
        subscriptions = self.child_status_subscriptions
        self.child_status_subscriptions = []
        for subscription in subscriptions:
            with contextlib.suppress(Exception):
                self.destroy_subscription(subscription)

    def _stabilizer_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        if isinstance(payload, dict):
            self.latest_stabilizer_status = payload
            self.stabilizer_status_received_at = time.monotonic()

    def stabilizer_snapshot(self) -> dict[str, Any]:
        snapshot = dict(self.latest_stabilizer_status)
        snapshot["status_age_sec"] = (
            None
            if self.stabilizer_status_received_at < 0.0
            else max(0.0, time.monotonic() - self.stabilizer_status_received_at)
        )
        return snapshot

    def wait_for_map(
        self,
        executor: SingleThreadedExecutor,
        timeout_sec: float = MAP_WAIT_TIMEOUT_SEC,
    ) -> bool:
        deadline = time.monotonic() + max(0.2, float(timeout_sec))
        while time.monotonic() < deadline:
            if self.latest_map_snapshot:
                return True
            executor.spin_once(timeout_sec=0.1)
        return bool(self.latest_map_snapshot)

    def validate_navigation_pose(
        self,
        pose: dict[str, Any],
        *,
        label: str,
    ) -> dict[str, Any]:
        return validate_pose_on_map(self.latest_map_snapshot, pose, label=label)

    def controller_snapshot(self, goal_accepted_at: float) -> dict[str, Any]:
        now = time.monotonic()
        path_seen = (
            self.latest_plan_at >= goal_accepted_at
            and self.latest_plan_pose_count > 0
        )
        controller_started = self.latest_raw_cmd_at >= goal_accepted_at
        path_age = None if self.latest_plan_at < 0.0 else max(0.0, now - self.latest_plan_at)
        raw_cmd_age = None if self.latest_raw_cmd_at < 0.0 else max(0.0, now - self.latest_raw_cmd_at)
        return {
            "controller_started": controller_started,
            "path_seen": path_seen,
            "path_age_sec": path_age,
            "raw_cmd_age_sec": raw_cmd_age,
            "latest_raw_cmd": dict(self.latest_raw_cmd),
            "plan_pose_count": self.latest_plan_pose_count,
        }

    def load_summary(self) -> dict[str, Any]:
        try:
            load_1m, load_5m, load_15m = os.getloadavg()
            system_load = {
                "load_1m": round(load_1m, 3),
                "load_5m": round(load_5m, 3),
                "load_15m": round(load_15m, 3),
            }
        except (AttributeError, OSError):
            system_load = {}
        return {
            **system_load,
            "process_cpu_sec": round(time.process_time(), 3),
            "plan_message_count": int(self.plan_message_count),
            "raw_cmd_message_count": int(self.raw_cmd_message_count),
            "child_status_tracking": bool(self.child_status_subscriptions),
        }

    def _spin_future_until(
        self,
        executor: SingleThreadedExecutor,
        future,
        deadline: float,
    ) -> bool:
        while not future.done() and time.monotonic() < deadline:
            self.stop_pub.publish(Twist())
            executor.spin_once(timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
        return bool(future.done())

    def _lifecycle_command(
        self,
        executor: SingleThreadedExecutor,
        command: int,
        deadline: float,
    ) -> tuple[bool, str]:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0 or not self.lifecycle_manager_client.wait_for_service(
            timeout_sec=min(0.5, remaining)
        ):
            return False, "lifecycle manager service unavailable"
        request = ManageLifecycleNodes.Request()
        request.command = int(command)
        future = self.lifecycle_manager_client.call_async(request)
        if not self._spin_future_until(executor, future, deadline):
            return False, "lifecycle command timed out"
        try:
            response = future.result()
        except Exception as exc:
            return False, f"lifecycle command failed: {exc}"
        if not bool(getattr(response, "success", False)):
            return False, "lifecycle manager rejected command"
        return True, "ok"

    def _lifecycle_states(
        self,
        executor: SingleThreadedExecutor,
        deadline: float,
    ) -> dict[str, str]:
        futures: dict[str, Any] = {}
        states: dict[str, str] = {}
        for name, client in self.lifecycle_state_clients.items():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0 or not client.wait_for_service(
                timeout_sec=min(0.2, remaining)
            ):
                states[name] = "service_unavailable"
                continue
            futures[name] = client.call_async(GetState.Request())
        for name, future in futures.items():
            if not self._spin_future_until(executor, future, deadline):
                states[name] = "timeout"
                continue
            try:
                states[name] = str(future.result().current_state.label or "unknown")
            except Exception as exc:
                states[name] = f"error:{exc}"
        return states

    def recover_navigation_lifecycle(
        self,
        executor: SingleThreadedExecutor,
        *,
        reason: str,
        timeout_sec: float = LIFECYCLE_RECOVERY_TIMEOUT_SEC,
    ) -> dict[str, Any]:
        started_at = time.monotonic()
        deadline = started_at + max(1.0, float(timeout_sec))
        self.lifecycle_reset_count += 1
        self.publish_stop(executor)
        merge_status(
            {
                "goal_active": False,
                "failure_stage": "lifecycle_recovery",
                "lifecycle_reset_count": self.lifecycle_reset_count,
                "lifecycle_recovery_reason": reason,
                "message": f"导航 action 树异常，正在执行 lifecycle 恢复：{reason}",
            }
        )
        reset_ok, reset_message = self._lifecycle_command(
            executor,
            int(getattr(ManageLifecycleNodes.Request, "RESET", 3)),
            deadline,
        )
        startup_ok = False
        startup_message = "not_started"
        if reset_ok:
            startup_ok, startup_message = self._lifecycle_command(
                executor,
                int(getattr(ManageLifecycleNodes.Request, "STARTUP", 0)),
                deadline,
            )

        states: dict[str, str] = {}
        ready = False
        readiness: dict[str, Any] = {}
        while startup_ok and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            now = time.monotonic()
            action_ready = self.action_client.wait_for_server(timeout_sec=0.05)
            child_interfaces_ready = all(
                client.wait_for_service(timeout_sec=0.01)
                for client in self.child_cancel_clients.values()
            )
            tf_ready = self.fresh_tf_map_pose(max_age_sec=1.5) is not None
            odom_age = None if self.latest_odom_at < 0.0 else now - self.latest_odom_at
            scan_age = None if self.latest_scan_at < 0.0 else now - self.latest_scan_at
            sensor_ready = (
                odom_age is not None
                and odom_age <= 1.5
                and scan_age is not None
                and scan_age <= 1.5
            )
            readiness = {
                "action_ready": bool(action_ready),
                "child_interfaces_ready": bool(child_interfaces_ready),
                "tf_ready": bool(tf_ready),
                "odom_age_sec": None if odom_age is None else round(odom_age, 3),
                "scan_age_sec": None if scan_age is None else round(scan_age, 3),
            }
            if action_ready and child_interfaces_ready and tf_ready and sensor_ready:
                states = self._lifecycle_states(executor, deadline)
                ready = all(states.get(name) == "active" for name in LIFECYCLE_NODE_NAMES)
                if ready:
                    break
            self.stop_pub.publish(Twist())

        result = {
            "success": bool(reset_ok and startup_ok and ready),
            "reason": reason,
            "reset": {"success": reset_ok, "message": reset_message},
            "startup": {"success": startup_ok, "message": startup_message},
            "lifecycle_states": states,
            "readiness": readiness,
            "elapsed_sec": round(time.monotonic() - started_at, 3),
            "reset_count": self.lifecycle_reset_count,
            "checked_at": utc_now_iso(),
        }
        self.lifecycle_last_result = dict(result)
        merge_status(
            {
                "lifecycle_recovery": result,
                "lifecycle_reset_count": self.lifecycle_reset_count,
                "lifecycle_recovery_elapsed_sec": result["elapsed_sec"],
                "message": (
                    "导航 lifecycle 恢复完成，准备重试当前点"
                    if result["success"]
                    else "导航 lifecycle 恢复失败，路线已安全停止"
                ),
            }
        )
        return result

    def active_child_goals(self) -> list[dict[str, Any]]:
        active: list[dict[str, Any]] = []
        for action_name, records in self.child_action_statuses.items():
            for record in records:
                if int(record.get("status", GoalStatus.STATUS_UNKNOWN)) in ACTIVE_CHILD_STATUS_CODES:
                    active.append({"action": action_name, **record})
        return active

    def child_statuses_fresh(
        self, timeout_sec: float = ACTION_STATUS_FRESH_TIMEOUT_SEC
    ) -> bool:
        now = time.monotonic()
        return all(
            self.child_status_received_at[action_name] >= 0.0
            and now - self.child_status_received_at[action_name] <= timeout_sec
            for action_name in PATH_CHILD_ACTION_NAMES
        )

    def cleanup_navigation_children(
        self,
        executor: SingleThreadedExecutor,
        timeout_sec: float = 3.0,
    ) -> dict[str, Any]:
        self._enable_child_status_tracking()
        started_at = time.monotonic()
        deadline = started_at + max(0.5, float(timeout_sec))
        zero = Twist()

        observation_deadline = min(deadline, time.monotonic() + 0.25)
        while time.monotonic() < observation_deadline:
            self.stop_pub.publish(zero)
            executor.spin_once(timeout_sec=0.05)
        active_before = self.active_child_goals()

        cancel_futures = {}
        service_available = {}
        for action_name, client in self.child_cancel_clients.items():
            available = client.wait_for_service(timeout_sec=0.05)
            service_available[action_name] = bool(available)
            if available:
                cancel_futures[action_name] = client.call_async(CancelGoal.Request())

        stable_empty_since = None
        empty_confirmed = False
        while time.monotonic() < deadline:
            self.stop_pub.publish(zero)
            executor.spin_once(timeout_sec=0.05)
            active_now = self.active_child_goals()
            all_responses_done = all(
                future.done() for future in cancel_futures.values()
            )
            if not active_now and all_responses_done:
                if stable_empty_since is None:
                    stable_empty_since = time.monotonic()
                elif time.monotonic() - stable_empty_since >= 0.20:
                    empty_confirmed = True
                    break
            else:
                stable_empty_since = None

        responses = {}
        for action_name, future in cancel_futures.items():
            if not future.done():
                responses[action_name] = {"completed": False}
                continue
            try:
                response = future.result()
                responses[action_name] = {
                    "completed": True,
                    "return_code": int(response.return_code),
                    "goals_canceling": [
                        bytes(info.goal_id.uuid).hex()
                        for info in response.goals_canceling
                    ],
                }
            except Exception as exc:
                responses[action_name] = {
                    "completed": True,
                    "error": str(exc),
                }

        active_after = self.active_child_goals()
        responses_done = all(future.done() for future in cancel_futures.values())
        services_ready = all(service_available.values())
        confirmed = not active_after and responses_done and empty_confirmed and services_ready
        result = {
            "confirmed": confirmed,
            "elapsed_sec": round(time.monotonic() - started_at, 3),
            "active_before": active_before,
            "active_after": active_after,
            "service_available": service_available,
            "cancel_responses": responses,
            "checked_at": utc_now_iso(),
        }
        if not confirmed:
            result["reason"] = (
                "active child navigation actions remain"
                if active_after
                else "child action cancellation service unavailable"
                if not services_ready
                else "child action cancellation confirmation timed out"
            )
        else:
            result["reason"] = "all child navigation actions are terminal"
        self._disable_child_status_tracking()
        return result

    def _cancel_goal(self, goal_handle, executor: SingleThreadedExecutor) -> bool:
        self.stop_pub.publish(Twist())
        cancel_future = goal_handle.cancel_goal_async()
        deadline = time.time() + 2.0
        while not cancel_future.done() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if not cancel_future.done():
            return False
        response = cancel_future.result()
        return bool(response and getattr(response, "goals_canceling", []))

    def cancel_goal_tree(
        self,
        goal_handle,
        executor: SingleThreadedExecutor,
    ) -> dict[str, Any]:
        self.stop_pub.publish(Twist())
        parent_cancel_confirmed = self._cancel_goal(goal_handle, executor)
        cleanup = self.cleanup_navigation_children(executor, timeout_sec=3.0)
        cleanup["parent_cancel_confirmed"] = parent_cancel_confirmed
        merge_status({"child_action_cleanup": cleanup})
        return cleanup

    def publish_initialpose(self, pose: dict[str, float], executor: SingleThreadedExecutor) -> None:
        subscriber_deadline = time.time() + 10.0
        while self.initialpose_pub.get_subscription_count() < 1 and time.time() < subscriber_deadline:
            executor.spin_once(timeout_sec=0.1)
        if self.initialpose_pub.get_subscription_count() < 1:
            raise RuntimeError("/initialpose has no subscribers")
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = float(pose["x"])
        message.pose.pose.position.y = float(pose["y"])
        qx, qy, qz, qw = self._quat_from_yaw(float(pose.get("yaw", 0.0)))
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.0685
        message.pose.covariance = covariance
        for _ in range(3):
            message.header.stamp = self.get_clock().now().to_msg()
            self.initialpose_pub.publish(message)
            executor.spin_once(timeout_sec=0.1)
            time.sleep(0.15)

    def send_goal(
        self,
        pose: dict[str, Any],
        executor: SingleThreadedExecutor,
        stop_flag,
        *,
        status_context: dict[str, Any],
        allow_replacement: bool = True,
    ) -> dict[str, Any]:
        session_id = str(status_context.get("session_id") or "")
        server_deadline = time.monotonic() + 15.0
        while not self.action_client.wait_for_server(timeout_sec=0.2):
            if stop_flag["stop"]:
                self.publish_stop(executor)
                raise KeyboardInterrupt()
            replacement = consume_goal_update(session_id) if allow_replacement else None
            if replacement is not None:
                return {"kind": "replacement", "request": replacement}
            if time.monotonic() >= server_deadline:
                raise RuntimeError("/navigate_to_pose action server not ready")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(pose["x"])
        goal_msg.pose.pose.position.y = float(pose["y"])
        qx, qy, qz, qw = self._quat_from_yaw(float(pose.get("yaw", 0.0)))
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        last_status_write = 0.0
        best_distance: float | None = None
        last_progress_at = utc_now_iso()

        def feedback_callback(feedback_message) -> None:
            nonlocal best_distance, last_progress_at, last_status_write
            feedback = getattr(feedback_message, "feedback", None)
            if feedback is None:
                return
            distance = None
            with contextlib.suppress(Exception):
                distance = float(feedback.distance_remaining)
            pose_msg = getattr(getattr(feedback, "current_pose", None), "pose", None)
            if pose_msg is not None:
                self._record_map_pose(
                    float(pose_msg.position.x),
                    float(pose_msg.position.y),
                    self._yaw_from_quaternion(pose_msg.orientation),
                )
            now_mono = time.monotonic()
            if now_mono - last_status_write < 1.0:
                return
            last_status_write = now_mono
            progress_update: dict[str, Any] = {}
            if distance is not None and math.isfinite(distance):
                if best_distance is None or best_distance - distance >= GOAL_PROGRESS_DISTANCE_M:
                    best_distance = distance
                    last_progress_at = utc_now_iso()
                    progress_update["last_progress_at"] = last_progress_at
            merge_status({
                **status_context,
                "state": "navigating",
                "task_phase": "navigating",
                "goal_active": True,
                "task_running": True,
                "distance_remaining": distance,
                "last_feedback_at": utc_now_iso(),
                "navigation_best_distance": best_distance,
                **progress_update,
                "message": str(status_context.get("message") or "正在导航"),
            })

        send_future = self.action_client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        response_deadline = time.monotonic() + GOAL_RESPONSE_TIMEOUT_SEC
        while not send_future.done():
            if stop_flag["stop"]:
                self.publish_stop(executor)
                raise KeyboardInterrupt()
            executor.spin_once(timeout_sec=0.1)
            if time.monotonic() >= response_deadline:
                self.publish_stop(executor)
                cleanup = self.cleanup_navigation_children(executor, timeout_sec=3.0)
                cleanup["stall_reason"] = "goal_response_timeout"
                raise NavigationControlError(
                    f"NavigateToPose goal response exceeded {GOAL_RESPONSE_TIMEOUT_SEC:.0f}s",
                    cleanup,
                )
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.publish_stop(executor)
            raise RuntimeError("navigation goal rejected")

        goal_accepted_at = time.monotonic()
        goal_id = bytes(goal_handle.goal_id.uuid).hex()
        merge_status(
            {
                "goal_id": goal_id,
                "controller_started": False,
                "path_age_sec": None,
                "navigation_load": self.load_summary(),
            }
        )
        result_future = goal_handle.get_result_async()
        last_monitor_status_at = 0.0
        while not result_future.done():
            if stop_flag["stop"]:
                self.cancel_goal_tree(goal_handle, executor)
                raise KeyboardInterrupt()
            executor.spin_once(timeout_sec=0.1)
            stabilizer = self.stabilizer_snapshot()
            controller = self.controller_snapshot(goal_accepted_at)
            fault = navigation_control_fault(
                elapsed_sec=time.monotonic() - goal_accepted_at,
                path_seen=bool(controller["path_seen"]),
                path_age_sec=controller["path_age_sec"],
                controller_started=bool(controller["controller_started"]),
                raw_cmd_age_sec=controller["raw_cmd_age_sec"],
            )
            if time.monotonic() - last_monitor_status_at >= 1.0:
                last_monitor_status_at = time.monotonic()
                merge_status(
                    {
                        "goal_id": goal_id,
                        "controller_started": bool(controller["controller_started"]),
                        "path_age_sec": controller["path_age_sec"],
                        "raw_cmd_age_sec": controller["raw_cmd_age_sec"],
                        "controller_monitor": controller,
                        "navigation_load": self.load_summary(),
                    }
                )
            if fault:
                cleanup = self.cancel_goal_tree(goal_handle, executor)
                cleanup["stall_reason"] = fault
                cleanup["stabilizer"] = stabilizer
                cleanup["controller"] = controller
                merge_status(
                    {
                        "failure_stage": fault,
                        "navigation_stall": cleanup,
                        "message": f"导航控制链异常，正在安全清理：{fault}",
                    }
                )
                raise NavigationControlError(fault, cleanup)
            replacement = consume_goal_update(session_id) if allow_replacement else None
            if replacement is not None:
                cleanup = self.cancel_goal_tree(goal_handle, executor)
                if not cleanup.get("parent_cancel_confirmed") or not cleanup.get(
                    "confirmed"
                ):
                    self.publish_stop(executor)
                    raise NavigationCleanupError(
                        "navigation goal tree cancellation was not confirmed"
                    )
                return {"kind": "replacement", "request": replacement}

        replacement = consume_goal_update(session_id) if allow_replacement else None
        if replacement is not None:
            return {"kind": "replacement", "request": replacement}
        result = result_future.result()
        status = int(result.status) if result is not None else GoalStatus.STATUS_UNKNOWN
        arrival_confirmation: dict[str, Any] = {}
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.publish_stop(executor)
            arrival_confirmation = confirm_goal_arrival(
                self,
                executor,
                stop_flag,
                pose,
                timeout_s=GOAL_ARRIVAL_CONFIRM_TIMEOUT_SEC,
            )
            self.last_arrival_confirmation = dict(arrival_confirmation)
            merge_status({"arrival_confirmation": arrival_confirmation})
            if not bool(arrival_confirmation.get("confirmed")):
                return {
                    "kind": "result",
                    "status": GoalStatus.STATUS_UNKNOWN,
                    "invalid_success": True,
                    "arrival_confirmation": arrival_confirmation,
                    "failure_message": (
                        "Nav2 reported success before physical arrival confirmation "
                        f"(distance={arrival_confirmation.get('distance_m')}, "
                        f"yaw_error={arrival_confirmation.get('yaw_error_rad')})"
                    ),
                }
        return {
            "kind": "result",
            "status": status,
            "arrival_confirmation": arrival_confirmation,
        }


def wait_with_spin(
    node: NavCommandNode,
    executor: SingleThreadedExecutor,
    stop_flag: dict[str, bool],
    duration_s: float,
) -> None:
    deadline = time.monotonic() + max(0.0, float(duration_s))
    while time.monotonic() < deadline:
        if stop_flag["stop"]:
            node.publish_stop(executor)
            raise KeyboardInterrupt()
        executor.spin_once(timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))


def confirm_goal_arrival(
    node: NavCommandNode,
    executor: SingleThreadedExecutor,
    stop_flag: dict[str, bool],
    goal_pose: dict[str, Any],
    *,
    timeout_s: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + max(0.2, float(timeout_s))
    consecutive = 0
    observations = 0
    last_result: dict[str, Any] = {
        "confirmed": False,
        "reason": "no_fresh_map_pose",
        "distance_m": None,
        "yaw_error_rad": None,
    }
    while time.monotonic() < deadline:
        if stop_flag["stop"]:
            node.publish_stop(executor)
            raise KeyboardInterrupt()
        executor.spin_once(timeout_sec=0.1)

        pose_source = "tf_map_base_link"
        current_pose = node.fresh_tf_map_pose(RETURN_POSE_FRESH_TIMEOUT_SEC)
        if current_pose is None:
            pose_source = "amcl"
            amcl_pose = node.latest_map_pose
            if (
                amcl_pose
                and time.monotonic() - float(amcl_pose.get("updated_at", 0.0))
                <= RETURN_POSE_FRESH_TIMEOUT_SEC
            ):
                current_pose = amcl_pose
        if current_pose is None:
            consecutive = 0
            continue

        observations += 1
        errors = goal_pose_errors(current_pose, goal_pose)
        within_tolerance = (
            errors["distance_m"] <= GOAL_ARRIVAL_XY_TOLERANCE_M
            and errors["yaw_error_rad"] <= GOAL_ARRIVAL_YAW_TOLERANCE_RAD
        )
        consecutive = consecutive + 1 if within_tolerance else 0
        last_result = {
            "confirmed": consecutive >= GOAL_ARRIVAL_CONFIRM_SAMPLES,
            "reason": "confirmed" if consecutive >= GOAL_ARRIVAL_CONFIRM_SAMPLES else "out_of_tolerance",
            "pose_source": pose_source,
            "pose": {
                "x": float(current_pose["x"]),
                "y": float(current_pose["y"]),
                "yaw": float(current_pose["yaw"]),
            },
            "goal": {
                "x": float(goal_pose["x"]),
                "y": float(goal_pose["y"]),
                "yaw": float(goal_pose.get("yaw", 0.0)),
            },
            "distance_m": round(errors["distance_m"], 4),
            "yaw_error_rad": round(errors["yaw_error_rad"], 4),
            "xy_tolerance_m": GOAL_ARRIVAL_XY_TOLERANCE_M,
            "yaw_tolerance_rad": GOAL_ARRIVAL_YAW_TOLERANCE_RAD,
            "consecutive_samples": consecutive,
            "required_samples": GOAL_ARRIVAL_CONFIRM_SAMPLES,
            "observations": observations,
        }
        if last_result["confirmed"]:
            return last_result
    last_result["confirmed"] = False
    last_result["observations"] = observations
    return last_result


def capture_fresh_return_pose(
    node: NavCommandNode,
    executor: SingleThreadedExecutor,
    stop_flag: dict[str, bool],
    timeout_s: float = 10.0,
) -> dict[str, Any]:
    deadline = time.monotonic() + max(1.0, float(timeout_s))
    while time.monotonic() < deadline:
        if stop_flag["stop"]:
            node.publish_stop(executor)
            raise KeyboardInterrupt()
        executor.spin_once(timeout_sec=0.1)
        pose = node.latest_map_pose
        pose_source = "amcl"
        if not pose or time.monotonic() - float(pose.get("updated_at", 0.0)) > RETURN_POSE_FRESH_TIMEOUT_SEC:
            pose = node.fresh_tf_map_pose(RETURN_POSE_FRESH_TIMEOUT_SEC)
            pose_source = "tf_map_base_link"
        if pose:
            node.return_pose_source = pose_source
            return {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "yaw": float(pose["yaw"]),
                "label": "S",
                "kind": "transit",
            }
    raise RuntimeError(
        "fresh map pose unavailable from AMCL and map->base_link TF; "
        "cabinet inspection route was not started"
    )


def run_job_command(arguments: list[str], timeout_s: float = 20.0) -> dict[str, Any]:
    completed = subprocess.run(
        [sys.executable, str(CABINET_JOB_SCRIPT), *arguments],
        cwd=str(ROOT_DIR),
        capture_output=True,
        text=True,
        timeout=timeout_s,
        check=False,
    )
    try:
        payload = json.loads(completed.stdout)
    except Exception as exc:
        raise RuntimeError(f"cabinet inspection job returned invalid JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise RuntimeError("cabinet inspection job returned a non-object result")
    if completed.returncode != 0:
        raise RuntimeError(str(payload.get("message") or completed.stderr or "cabinet inspection job failed"))
    return payload


def inspection_job_id(session_id: str, point_index: int, point: dict[str, Any], attempt: int) -> str:
    clean_session = re.sub(r"[^A-Za-z0-9_.-]+", "-", str(session_id or "nav")).strip("-.") or "nav"
    clean_label = re.sub(r"[^A-Za-z0-9_.-]+", "-", str(point.get("label") or f"C{point_index + 1}")).strip("-.") or "cabinet"
    return f"{clean_session}-wp{point_index + 1}-{clean_label}-scan{attempt}"[:120]


def start_inspection_job(
    inspection_id: str,
    session_id: str,
    point_index: int,
    point: dict[str, Any],
) -> dict[str, Any]:
    return run_job_command(
        [
            "start",
            "--inspection-id",
            inspection_id,
            "--expected-tag-id",
            str(int(point["apriltag_id"])),
            "--navigation-session-id",
            session_id,
            "--waypoint-index",
            str(point_index),
            "--waypoint-label",
            str(point.get("label") or f"C{point_index + 1}"),
            "--timeout-s",
            "900",
        ]
    )


def inspection_job_status(inspection_id: str) -> dict[str, Any]:
    return run_job_command(["status", "--inspection-id", inspection_id], timeout_s=5.0)


def stop_inspection_job(inspection_id: str) -> None:
    if not inspection_id:
        return
    with contextlib.suppress(Exception):
        run_job_command(["stop", "--inspection-id", inspection_id], timeout_s=20.0)


def append_navigation_attempt(record: dict[str, Any]) -> None:
    current = read_json(STATUS_PATH)
    history = current.get("navigation_attempt_history")
    if not isinstance(history, list):
        history = []
    history.append(record)
    merge_status({"navigation_attempt_history": history[-30:]})


def navigate_with_retries(
    node: NavCommandNode,
    executor: SingleThreadedExecutor,
    stop_flag: dict[str, bool],
    point: dict[str, Any],
    *,
    session_id: str,
    point_index: int,
    point_count: int,
    returning: bool = False,
) -> tuple[bool, int, int]:
    last_status = GoalStatus.STATUS_UNKNOWN
    label = str(point.get("label") or ("S" if returning else f"P{point_index + 1}"))
    validation = node.validate_navigation_pose(point, label=label)
    merge_status({"route_point_validation": validation})
    if not validation.get("valid"):
        node.last_arrival_confirmation = {
            "confirmed": False,
            "reason": str(validation.get("reason") or "invalid_map_pose"),
            "map_validation": validation,
        }
        append_navigation_attempt(
            {
                "label": label,
                "point_index": point_index,
                "returning": returning,
                "attempt": 0,
                "started_at": utc_now_iso(),
                "finished_at": utc_now_iso(),
                "duration_sec": 0.0,
                "status_code": GoalStatus.STATUS_UNKNOWN,
                "result": "rejected",
                "failure_message": f"map validation failed: {validation.get('reason')}",
                "map_validation": validation,
            }
        )
        return False, 0, GoalStatus.STATUS_UNKNOWN
    for attempt in range(1, 4):
        node.last_arrival_confirmation = {}
        attempt_started_at = utc_now_iso()
        attempt_started_mono = time.monotonic()
        phase = "returning" if returning else "navigating"
        merge_status(
            {
                "mode": phase,
                "state": phase,
                "task_phase": phase,
                "task_running": True,
                "goal_active": True,
                "current_index": point_index,
                "current_goal": point,
                "current_point_kind": "return" if returning else str(point.get("kind") or "transit"),
                "current_apriltag_id": point.get("apriltag_id"),
                "navigation_attempt": attempt,
                "arrival_confirmation": {},
                "return_state": "navigating" if returning else str(read_json(STATUS_PATH).get("return_state") or "pending"),
                "message": (
                    f"正在返航到起点 S，第 {attempt}/3 次"
                    if returning
                    else f"正在导航到 {label}（{point_index + 1}/{point_count}），第 {attempt}/3 次"
                ),
            }
        )
        failure_message = ""
        failure_cleanup: dict[str, Any] | None = None
        arrival_confirmation: dict[str, Any] = {}
        reported_success_rejected = False
        requires_lifecycle_recovery = False
        lifecycle_recovery: dict[str, Any] = {}
        try:
            outcome = node.send_goal(
                point,
                executor,
                stop_flag,
                status_context={
                    "session_id": session_id,
                    "current_index": point_index,
                    "current_goal": point,
                    "last_goal": point,
                    "navigation_attempt": attempt,
                    "message": f"正在导航到 {label}，第 {attempt}/3 次",
                },
                allow_replacement=False,
            )
            last_status = int(outcome.get("status", GoalStatus.STATUS_UNKNOWN))
            arrival_confirmation = dict(outcome.get("arrival_confirmation") or {})
            reported_success_rejected = bool(outcome.get("invalid_success"))
            failure_message = str(outcome.get("failure_message") or "")
        except NavigationControlError as exc:
            last_status = GoalStatus.STATUS_UNKNOWN
            failure_message = str(exc)
            failure_cleanup = exc.cleanup
            requires_lifecycle_recovery = True
        except NavigationStallError as exc:
            last_status = GoalStatus.STATUS_UNKNOWN
            failure_message = str(exc)
            failure_cleanup = exc.cleanup
        except KeyboardInterrupt:
            node.publish_stop(executor)
            cleanup = node.cleanup_navigation_children(executor, timeout_sec=3.0)
            append_navigation_attempt(
                {
                    "label": label,
                    "point_index": point_index,
                    "returning": returning,
                    "attempt": attempt,
                    "started_at": attempt_started_at,
                    "finished_at": utc_now_iso(),
                    "duration_sec": round(time.monotonic() - attempt_started_mono, 3),
                    "status_code": GoalStatus.STATUS_CANCELED,
                    "result": "canceled",
                    "cleanup": cleanup,
                    "stabilizer": node.stabilizer_snapshot(),
                }
            )
            raise
        except Exception as exc:
            last_status = GoalStatus.STATUS_UNKNOWN
            failure_message = str(exc)
        if last_status == GoalStatus.STATUS_SUCCEEDED:
            node.last_arrival_confirmation = dict(arrival_confirmation)
            merge_status({"arrival_confirmation": arrival_confirmation})
            if bool(arrival_confirmation.get("confirmed")):
                append_navigation_attempt(
                    {
                        "label": label,
                        "point_index": point_index,
                        "returning": returning,
                        "attempt": attempt,
                        "started_at": attempt_started_at,
                        "finished_at": utc_now_iso(),
                        "duration_sec": round(time.monotonic() - attempt_started_mono, 3),
                        "status_code": last_status,
                        "result": "succeeded",
                        "arrival_confirmation": arrival_confirmation,
                        "cleanup": {"required": False, "confirmed": True},
                        "stabilizer": node.stabilizer_snapshot(),
                    }
                )
                return True, attempt, last_status
            reported_success_rejected = True
            distance = arrival_confirmation.get("distance_m")
            yaw_error = arrival_confirmation.get("yaw_error_rad")
            if not failure_message:
                failure_message = (
                    "Nav2 reported success before physical arrival confirmation "
                    f"(distance={distance}, yaw_error={yaw_error})"
                )
        node.publish_stop(executor)
        cleanup = failure_cleanup or node.cleanup_navigation_children(
            executor, timeout_sec=3.0
        )
        append_navigation_attempt(
            {
                "label": label,
                "point_index": point_index,
                "returning": returning,
                "attempt": attempt,
                "started_at": attempt_started_at,
                "finished_at": utc_now_iso(),
                "duration_sec": round(time.monotonic() - attempt_started_mono, 3),
                "status_code": last_status,
                "result": "failed",
                "failure_message": failure_message,
                "reported_success_rejected": reported_success_rejected,
                "arrival_confirmation": arrival_confirmation,
                "cleanup": cleanup,
                "stabilizer": node.stabilizer_snapshot(),
            }
        )
        cleanup_confirmed = bool(cleanup.get("confirmed"))
        if failure_cleanup is not None and "parent_cancel_confirmed" in cleanup:
            cleanup_confirmed = cleanup_confirmed and bool(
                cleanup.get("parent_cancel_confirmed")
            )
        if requires_lifecycle_recovery or not cleanup_confirmed:
            recovery_reason = failure_message or str(cleanup.get("reason") or "action_cleanup_failed")
            lifecycle_recovery = node.recover_navigation_lifecycle(
                executor,
                reason=recovery_reason,
            )
            if lifecycle_recovery.get("success"):
                cleanup_confirmed = True
            else:
                cleanup_confirmed = False
        if not cleanup_confirmed:
            message = (
                f"{label} 导航失败后，Nav2 action 树未能恢复；"
                "已锁定停车并终止本次路线"
            )
            merge_status(
                {
                    "goal_active": False,
                    "failure_stage": "child_action_cleanup",
                    "child_action_cleanup": cleanup,
                    "lifecycle_recovery": lifecycle_recovery,
                    "message": message,
                    "last_error": message,
                }
            )
            raise NavigationCleanupError(message)
        merge_status(
            {
                "goal_active": False,
                "failure_stage": (
                    "arrival_confirmation"
                    if reported_success_rejected
                    else "navigate_to_pose"
                ),
                "child_action_cleanup": cleanup,
                "lifecycle_recovery": lifecycle_recovery,
                "message": (
                    f"{label} 第 {attempt}/3 次导航失败：{failure_message}"
                    if failure_message
                    else f"{label} 第 {attempt}/3 次导航失败，状态码 {last_status}"
                ),
            }
        )
        if attempt < 3:
            wait_with_spin(node, executor, stop_flag, 1.0)
    return False, 3, last_status


def wait_for_inspection_job(
    node: NavCommandNode,
    executor: SingleThreadedExecutor,
    stop_flag: dict[str, bool],
    inspection_id: str,
    point: dict[str, Any],
    scan_attempt: int,
) -> dict[str, Any]:
    deadline = time.monotonic() + 930.0
    last_status_write = 0.0
    while time.monotonic() < deadline:
        if stop_flag["stop"]:
            stop_inspection_job(inspection_id)
            node.publish_stop(executor)
            raise KeyboardInterrupt()
        executor.spin_once(timeout_sec=0.05)
        snapshot = inspection_job_status(inspection_id)
        state = str(snapshot.get("state") or "")
        now_mono = time.monotonic()
        if now_mono - last_status_write >= 1.0:
            last_status_write = now_mono
            merge_status(
                {
                    "mode": "scanning",
                    "state": "scanning",
                    "task_phase": "scanning",
                    "task_running": True,
                    "goal_active": False,
                    "current_point_kind": "cabinet",
                    "current_apriltag_id": int(point["apriltag_id"]),
                    "scan_attempt": scan_attempt,
                    "scan_command_id": inspection_id,
                    "active_inspection_id": inspection_id,
                    "auto_recognition_state": state or "starting",
                    "message": f"正在扫描 {point.get('label', 'C')} / T{point['apriltag_id']}，第 {scan_attempt}/2 次",
                }
            )
        if state in CABINET_JOB_TERMINAL_STATES:
            return snapshot
        time.sleep(0.2)
    stop_inspection_job(inspection_id)
    return {"state": "timeout", "result": {"status": "failed", "message": "inspection job wait timed out"}}


def run_cabinet_patrol(
    node: NavCommandNode,
    executor: SingleThreadedExecutor,
    stop_flag: dict[str, bool],
    points: list[dict[str, Any]],
    start_pose: dict[str, Any],
    task_meta: dict[str, Any],
) -> int:
    if start_pose:
        node.publish_initialpose(start_pose, executor)
        wait_with_spin(node, executor, stop_flag, 4.0)
        return_pose = {**start_pose, "label": "S", "kind": "transit"}
        node.return_pose_source = "explicit_start_pose"
    else:
        localization_initial_pose = task_meta.get("localization_initial_pose")
        if isinstance(localization_initial_pose, dict) and localization_initial_pose:
            merge_status(
                {
                    "mode": "localizing",
                    "state": "localizing",
                    "task_phase": "localizing",
                    "task_running": True,
                    "goal_active": False,
                    "localization_initial_pose": localization_initial_pose,
                    "message": "正在使用地图默认位姿初始化定位，等待地图位姿确认返航起点",
                }
            )
            node.latest_map_pose = None
            node.publish_initialpose(localization_initial_pose, executor)
        return_pose = capture_fresh_return_pose(node, executor, stop_flag, timeout_s=15.0)

    return_validation = node.validate_navigation_pose(return_pose, label="S")
    merge_status({"return_pose_validation": return_validation})
    if not return_validation.get("valid"):
        raise RuntimeError(
            "return pose S rejected by map validation: "
            f"{return_validation.get('reason')}"
        )

    session_id = task_meta["session_id"]
    point_results: list[dict[str, Any]] = []
    missing: list[str] = []
    completed = 0
    merge_status(
        {
            "mode": "planning",
            "state": "planning",
            "task_phase": "planning",
            "task_running": True,
            "goal_active": False,
            "return_pose_source": node.return_pose_source,
            "return_pose": return_pose,
            "points": points,
            "point_labels": point_labels(points),
            "current_index": -1,
            "loop_index": 1,
            "loop_count": 1,
            "completed_count": 0,
            "current_goal": points[0] if points else {},
            "goal_pose": points[-1] if points else {},
            "last_goal": {},
            "start_pose": return_pose,
            "localization_initial_pose": task_meta.get("localization_initial_pose", {}),
            "start_source": task_meta["start_source"],
            "session_id": session_id,
            "navigation_generation": task_meta["navigation_generation"],
            "map": task_meta["map_name"],
            "active_map": task_meta["map_name"],
            "map_name": task_meta["map_name"],
            "map_source": task_meta["map_source"],
            "route_source": task_meta["route_source"],
            "auto_cabinet_inspection": True,
            "auto_recognition_state": "pending",
            "current_point_kind": "",
            "current_apriltag_id": None,
            "navigation_attempt": 0,
            "navigation_attempt_history": [],
            "child_action_cleanup": {},
            "lifecycle_reset_count": node.lifecycle_reset_count,
            "lifecycle_recovery": {},
            "failure_stage": "",
            "scan_attempt": 0,
            "scan_command_id": "",
            "active_inspection_id": "",
            "point_results": [],
            "return_state": "pending",
            "missing_items": [],
            "message": "配电柜巡检路线已启动",
            "last_result": "",
            "last_error": "",
            "pid": os.getpid(),
        }
    )

    for point_index, point in enumerate(points):
        if stop_flag["stop"]:
            raise KeyboardInterrupt()
        reached, nav_attempts, nav_status = navigate_with_retries(
            node,
            executor,
            stop_flag,
            point,
            session_id=session_id,
            point_index=point_index,
            point_count=len(points),
        )
        record: dict[str, Any] = {
            "index": point_index,
            "label": str(point.get("label") or f"P{point_index + 1}"),
            "kind": str(point.get("kind") or "transit"),
            "apriltag_id": point.get("apriltag_id"),
            "navigation_attempts": nav_attempts,
            "navigation_status_code": nav_status,
            "navigation_status": "completed" if reached else "failed",
            "arrival_confirmation": dict(node.last_arrival_confirmation),
            "scan_attempts": 0,
            "scan_status": "not_required" if point.get("kind") != "cabinet" else "skipped",
        }
        if reached and not bool(record["arrival_confirmation"].get("confirmed")):
            reached = False
            record["navigation_status"] = "failed"
            record["navigation_guard_error"] = "arrival_confirmation_missing"
        if not reached:
            missing.append(f"{record['label']}:navigation_failed")
            point_results.append(record)
            merge_status(
                {
                    "goal_active": False,
                    "point_results": point_results,
                    "missing_items": missing,
                    "message": f"{record['label']} 三次导航均失败，已跳过",
                }
            )
            continue

        completed += 1
        if point.get("kind") == "cabinet":
            node.publish_stop(executor)
            scan_terminal = "failed"
            scan_result: dict[str, Any] = {}
            inspection_ids: list[str] = []
            for scan_attempt in range(1, 3):
                inspection_id = inspection_job_id(session_id, point_index, point, scan_attempt)
                inspection_ids.append(inspection_id)
                merge_status(
                    {
                        "mode": "scanning",
                        "state": "scanning",
                        "task_phase": "scanning",
                        "goal_active": False,
                        "current_index": point_index,
                        "current_goal": point,
                        "current_point_kind": "cabinet",
                        "current_apriltag_id": int(point["apriltag_id"]),
                        "scan_attempt": scan_attempt,
                        "scan_command_id": inspection_id,
                        "active_inspection_id": inspection_id,
                        "auto_recognition_state": "starting",
                        "message": f"已到达 {record['label']}，正在启动 T{point['apriltag_id']} 扫描",
                    }
                )
                try:
                    start_inspection_job(inspection_id, session_id, point_index, point)
                    snapshot = wait_for_inspection_job(node, executor, stop_flag, inspection_id, point, scan_attempt)
                    scan_terminal = str(snapshot.get("state") or "failed")
                    scan_result = snapshot.get("result") if isinstance(snapshot.get("result"), dict) else {}
                except KeyboardInterrupt:
                    stop_inspection_job(inspection_id)
                    raise
                except Exception as exc:
                    scan_terminal = "failed"
                    scan_result = {"status": "failed", "message": str(exc)}
                if scan_terminal in {"completed", "partial"}:
                    break
                if scan_attempt < 2:
                    node.publish_stop(executor)
                    wait_with_spin(node, executor, stop_flag, 1.0)
            record["scan_attempts"] = len(inspection_ids)
            record["inspection_ids"] = inspection_ids
            record["scan_status"] = scan_terminal
            record["scan_result"] = scan_result
            if scan_terminal != "completed":
                missing.append(f"{record['label']}:scan_{scan_terminal}")
        point_results.append(record)
        merge_status(
            {
                "goal_active": False,
                "completed_count": completed,
                "last_goal": point,
                "point_results": point_results,
                "missing_items": missing,
                "active_inspection_id": "",
                "auto_recognition_state": record["scan_status"],
                "message": f"{record['label']} 已处理，继续下一点",
            }
        )

    merge_status(
        {
            "mode": "returning",
            "state": "returning",
            "task_phase": "returning",
            "current_goal": return_pose,
            "return_state": "navigating",
            "message": "全部路线点已处理，正在返回起点 S",
        }
    )
    returned, return_attempts, return_status = navigate_with_retries(
        node,
        executor,
        stop_flag,
        return_pose,
        session_id=session_id,
        point_index=len(points),
        point_count=len(points),
        returning=True,
    )
    node.publish_stop(executor)
    if not returned:
        missing.append("S:return_failed")
    final_state = "failed" if not returned else ("partial" if missing else "completed")
    message = (
        "配电柜巡检完成并已返回起点"
        if final_state == "completed"
        else "配电柜巡检存在缺项，已返回起点"
        if final_state == "partial"
        else "配电柜巡检已处理路线点，但返航失败"
    )
    merge_status(
        {
            "mode": final_state,
            "state": final_state,
            "task_phase": "terminal",
            "task_running": False,
            "goal_active": False,
            "current_index": len(points) - 1,
            "completed_count": completed,
            "current_goal": {},
            "last_goal": return_pose if returned else (points[-1] if points else {}),
            "point_results": point_results,
            "return_state": "completed" if returned else "failed",
            "return_attempts": return_attempts,
            "return_status_code": return_status,
            "missing_items": missing,
            "active_inspection_id": "",
            "auto_recognition_state": "completed" if final_state == "completed" else final_state,
            "message": message,
            "last_result": message,
            "last_error": "" if returned else "return to start failed after three attempts",
            "pid": 0,
        }
    )
    print(json.dumps({"status": final_state, "point_results": point_results, "missing_items": missing}, ensure_ascii=False), flush=True)
    return 0 if returned else 1


def command_set_initial_pose(args: argparse.Namespace) -> int:
    pose = safe_pose_dict(json.loads(args.pose_json))
    rclpy.init(args=None)
    node = NavCommandNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        node.publish_initialpose(pose, executor)
        merge_status(
            {
                "mode": "idle",
                "task_running": False,
                "goal_active": False,
                "start_pose": pose,
                "message": "initial pose published",
                "last_result": "initial pose published",
                "last_error": "",
            }
        )
        print("initial pose published", flush=True)
        return 0
    finally:
        with contextlib.suppress(Exception):
            node.action_client.destroy()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


def build_patrol_command(points_json: str, loop_count: int, start_pose_json: str, task_json: str) -> list[str]:
    return [
        sys.executable,
        str(Path(__file__).resolve()),
        "run-cruise",
        "--points-json",
        points_json,
        "--loop-count",
        str(loop_count),
        "--start-pose-json",
        start_pose_json,
        "--task-json",
        task_json,
    ]


def command_start_cruise(args: argparse.Namespace) -> int:
    points = safe_points(json.loads(args.points_json))
    loop_count = max(1, int(args.loop_count))
    start_pose = optional_pose_from_json(args.start_pose_json)
    task_meta = safe_task_meta(json.loads(args.task_json or "{}"))

    existing_pid = read_pid()
    if pid_alive(existing_pid):
        raise RuntimeError(f"cruise already running: pid={existing_pid}")
    clear_pid_file()
    remove_file(GOAL_UPDATE_PATH)
    remove_file(GOAL_UPDATE_ACK_PATH)

    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    task_json = json.dumps(task_meta, ensure_ascii=False)
    command = build_patrol_command(args.points_json, loop_count, args.start_pose_json or "", task_json)
    write_json(
        STATUS_PATH,
        {
            "updated_at": utc_now_iso(),
            "mode": "starting",
            "state": "starting",
            "task_phase": "starting",
            "task_running": True,
            "goal_active": False,
            "session_id": task_meta["session_id"],
            "navigation_generation": task_meta["navigation_generation"],
            "map": task_meta["map_name"],
            "active_map": task_meta["map_name"],
            "map_name": task_meta["map_name"],
            "map_source": task_meta["map_source"],
            "route_source": task_meta["route_source"],
            "start_source": task_meta["start_source"],
            "points": points,
            "point_labels": point_labels(points),
            "start_pose": start_pose,
            "localization_initial_pose": task_meta.get("localization_initial_pose", {}),
            "current_index": -1,
            "loop_index": 0,
            "loop_count": loop_count,
            "completed_count": 0,
            "current_goal": points[0] if points else {},
            "goal_pose": points[-1] if points else {},
            "last_goal": {},
            "goal_id": "",
            "controller_started": False,
            "path_age_sec": None,
            "navigation_attempt": 0,
            "navigation_attempt_history": [],
            "child_action_cleanup": {},
            "lifecycle_reset_count": 0,
            "lifecycle_recovery": {},
            "failure_stage": "",
            "arrival_confirmation": {},
            "return_pose": {},
            "return_pose_source": "",
            "return_state": "pending",
            "return_attempts": 0,
            "point_results": [],
            "missing_items": [],
            "scan_attempt": 0,
            "scan_command_id": "",
            "active_inspection_id": "",
            "auto_cabinet_inspection": task_meta["auto_cabinet_inspection"],
            "auto_recognition_state": "pending" if task_meta["auto_cabinet_inspection"] else "disabled",
            "auto_recognition_command_id": "",
            "auto_recognition_error": "",
            "message": f"starting cruise with {len(points)} point(s)",
            "last_result": "",
            "last_error": "",
            "pid": 0,
        },
    )
    with LOG_PATH.open("a", encoding="utf-8") as log_handle:
        log_handle.write(f"\n[{utc_now_iso()}] start cruise request points={len(points)} loop_count={loop_count}\n")
        log_handle.flush()
        kwargs: dict[str, Any] = {
            "stdout": log_handle,
            "stderr": log_handle,
            "cwd": str(ROOT_DIR),
            "start_new_session": True,
        }
        process = subprocess.Popen(command, **kwargs)
    PID_PATH.write_text(str(process.pid), encoding="utf-8")
    merge_status(
        {
            "mode": "starting",
            "state": "starting",
            "task_phase": "starting",
            "task_running": True,
            "goal_active": False,
            "message": f"starting cruise with {len(points)} point(s)",
            "last_result": "",
            "last_error": "",
            "points": points,
            "point_labels": point_labels(points),
            "current_index": -1,
            "loop_index": 0,
            "loop_count": loop_count,
            "completed_count": 0,
            "current_goal": points[0] if points else {},
            "goal_pose": points[-1] if points else {},
            "last_goal": {},
            "start_pose": start_pose,
            "localization_initial_pose": task_meta.get("localization_initial_pose", {}),
            "start_source": task_meta["start_source"],
            "session_id": task_meta["session_id"],
            "navigation_generation": task_meta["navigation_generation"],
            "map": task_meta["map_name"],
            "active_map": task_meta["map_name"],
            "map_name": task_meta["map_name"],
            "map_source": task_meta["map_source"],
            "route_source": task_meta["route_source"],
            "goal_refresh": task_meta["goal_refresh"],
            "goal_refresh_count": 0,
            "goal_refresh_suppressed": False,
            "goal_refresh_suppress_distance_m": 0.0,
            "goal_refresh_suppress_delay_s": 0.0,
            "goal_refresh_final_send_pending": False,
            "final_heading_state": "",
            "final_heading_delay_s": 0.0,
            "final_heading_error_deg": None,
            "last_goal_refresh_at": "",
            "auto_cabinet_inspection": task_meta["auto_cabinet_inspection"],
            "auto_recognition_state": "pending" if task_meta["auto_cabinet_inspection"] else "disabled",
            "auto_recognition_command_id": "",
            "auto_recognition_error": "",
            "no_progress_since": "",
            "last_progress_at": "",
            "goal_revision": 1,
            "goal_replacement_count": 0,
            "previous_goal": {},
            "last_goal_replaced_at": "",
            "goal_update_state": "",
            "pid": process.pid,
        }
    )
    print(f"cruise started pid={process.pid}", flush=True)
    return 0


def standalone_navigation_cleanup() -> tuple[dict[str, Any], bool]:
    rclpy.init(args=None)
    node = NavCommandNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        cleanup = node.cleanup_navigation_children(executor, timeout_sec=3.0)
        stack_ready = node.action_client.wait_for_server(timeout_sec=1.0)
        return cleanup, bool(stack_ready)
    finally:
        with contextlib.suppress(Exception):
            node.action_client.destroy()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


def finish_cruise_stop(
    previous: dict[str, Any],
    previous_goal: dict[str, Any],
    *,
    forced: bool = False,
) -> int:
    try:
        cleanup, stack_ready = standalone_navigation_cleanup()
    except Exception as exc:
        cleanup = {
            "confirmed": False,
            "reason": f"standalone cleanup failed: {exc}",
            "checked_at": utc_now_iso(),
        }
        stack_ready = False
    if cleanup.get("confirmed"):
        if stack_ready:
            message = (
                "路线进程已强制停止，导航系统保持就绪"
                if forced
                else "路线已停止，导航系统保持就绪"
            )
        else:
            message = (
                "路线进程已强制停止，导航系统当前未运行"
                if forced
                else "路线已停止，导航系统当前未运行"
            )
        state = "canceled"
        last_error = ""
        exit_code = 0
    else:
        message = "路线已停止，但 Nav2 子任务清理未确认；底盘保持零速"
        state = "failed"
        last_error = str(cleanup.get("reason") or message)
        exit_code = 1
    merge_status(
        {
            "mode": state,
            "state": state,
            "task_phase": "terminal",
            "task_running": False,
            "goal_active": False,
            "message": message,
            "last_result": message if exit_code == 0 else "",
            "last_error": last_error,
            "failure_stage": "" if exit_code == 0 else "child_action_cleanup",
            "child_action_cleanup": cleanup,
            "navigation_stack_ready": stack_ready,
            "current_goal": {},
            "last_goal": previous_goal or previous.get("last_goal", {}),
            "pid": 0,
        }
    )
    print(json.dumps({"message": message, "cleanup": cleanup}, ensure_ascii=False), flush=True)
    return exit_code


def command_stop_cruise(_args: argparse.Namespace) -> int:
    remove_file(GOAL_UPDATE_PATH)
    previous = read_json(STATUS_PATH)
    stop_inspection_job(str(previous.get("active_inspection_id") or ""))
    previous_goal = previous.get("current_goal") if isinstance(previous.get("current_goal"), dict) else {}
    pid = read_pid()
    if not pid_alive(pid):
        clear_pid_file()
        return finish_cruise_stop(previous, previous_goal)

    merge_status({"mode": "stopping", "state": "stopping", "task_phase": "terminal", "message": f"正在停止导航 pid={pid}"})
    os.kill(pid, signal.SIGTERM)
    deadline = time.time() + 8.0
    while time.time() < deadline:
        if not pid_alive(pid):
            clear_pid_file()
            return finish_cruise_stop(previous, previous_goal)
        time.sleep(0.2)

    os.kill(pid, signal.SIGKILL)
    clear_pid_file()
    return finish_cruise_stop(previous, previous_goal, forced=True)


def command_replace_goal(args: argparse.Namespace) -> int:
    goal_pose = safe_pose_dict(json.loads(args.goal_json))
    status = read_json(STATUS_PATH)
    pid = read_pid()
    if not pid_alive(pid) or not bool(status.get("task_running")):
        raise RuntimeError("no active navigation task")
    if bool(status.get("auto_cabinet_inspection")):
        raise RuntimeError("cabinet inspection route does not allow live goal replacement")
    active_session = str(status.get("session_id") or "").strip()
    requested_session = str(args.session_id or "").strip()
    if not active_session or requested_session != active_session:
        raise RuntimeError("navigation session changed")

    request_id = str(args.request_id or f"goal-{int(time.time() * 1000)}").strip()
    request = {
        "request_id": request_id,
        "session_id": requested_session,
        "map_name": str(args.map_name or "").strip(),
        "goal_pose": goal_pose,
        "replace_remaining": True,
        "requested_at": utc_now_iso(),
    }
    remove_file(GOAL_UPDATE_ACK_PATH)
    write_json(GOAL_UPDATE_PATH, request)
    deadline = time.time() + 5.0
    while time.time() < deadline:
        ack = read_json(GOAL_UPDATE_ACK_PATH)
        if str(ack.get("request_id") or "") != request_id:
            time.sleep(0.05)
            continue
        if ack.get("status") == "accepted":
            print(json.dumps(ack, ensure_ascii=False), flush=True)
            return 0
        raise RuntimeError(str(ack.get("message") or "goal update rejected"))
    raise RuntimeError("navigation worker did not acknowledge goal update")


def command_run_cruise(args: argparse.Namespace) -> int:
    points = safe_points(json.loads(args.points_json))
    loop_count = max(1, int(args.loop_count))
    start_pose = optional_pose_from_json(args.start_pose_json)
    task_meta = safe_task_meta(json.loads(args.task_json or "{}"))
    stop_flag = {"stop": False}

    def handle_stop(_signum, _frame):
        stop_flag["stop"] = True

    signal.signal(signal.SIGTERM, handle_stop)
    signal.signal(signal.SIGINT, handle_stop)

    rclpy.init(args=None)
    node = NavCommandNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    PID_PATH.write_text(str(os.getpid()), encoding="utf-8")
    try:
        if not node.wait_for_map(executor, MAP_WAIT_TIMEOUT_SEC):
            raise RuntimeError("navigation map was not received within 5 seconds")
        initial_validation: dict[str, Any] = {}
        initial_candidate = start_pose or task_meta.get("localization_initial_pose")
        if isinstance(initial_candidate, dict) and initial_candidate:
            initial_validation = node.validate_navigation_pose(
                initial_candidate,
                label="S",
            )
            if not initial_validation.get("valid"):
                raise RuntimeError(
                    "start pose S rejected by map validation: "
                    f"{initial_validation.get('reason')}"
                )
        merge_status(
            {
                "navigation_generation": task_meta["navigation_generation"],
                "map_validation_ready": True,
                "start_pose_validation": initial_validation,
                "navigation_load": node.load_summary(),
            }
        )
        if task_meta["auto_cabinet_inspection"]:
            return run_cabinet_patrol(node, executor, stop_flag, points, start_pose, task_meta)
        if start_pose:
            node.publish_initialpose(start_pose, executor)
            settle_deadline = time.time() + 4.0
            while time.time() < settle_deadline:
                executor.spin_once(timeout_sec=0.1)
                if stop_flag["stop"]:
                    raise KeyboardInterrupt()
        total_points = len(points)
        completed = 0
        merge_status(
            {
                "mode": "planning",
                "state": "planning",
                "task_phase": "planning",
                "task_running": True,
                "goal_active": False,
                "points": points,
                "point_labels": point_labels(points),
                "current_index": -1,
                "loop_index": 0,
                "loop_count": loop_count,
                "completed_count": 0,
                "current_goal": points[0] if points else {},
                "goal_pose": points[-1] if points else {},
                "last_goal": {},
                "start_pose": start_pose,
                "start_source": task_meta["start_source"],
                "session_id": task_meta["session_id"],
                "navigation_generation": task_meta["navigation_generation"],
                "map": task_meta["map_name"],
                "active_map": task_meta["map_name"],
                "map_name": task_meta["map_name"],
                "map_source": task_meta["map_source"],
                "route_source": task_meta["route_source"],
                "goal_refresh": {"enabled": False, "mode": "disabled"},
                "goal_refresh_count": 0,
                "goal_refresh_suppressed": False,
                "goal_refresh_suppress_distance_m": 0.0,
                "goal_refresh_suppress_delay_s": 0.0,
                "goal_refresh_final_send_pending": False,
                "final_heading_state": "",
                "final_heading_delay_s": 0.0,
                "final_heading_error_deg": None,
                "last_goal_refresh_at": "",
                "auto_cabinet_inspection": False,
                "auto_recognition_state": "disabled",
                "auto_recognition_command_id": "",
                "auto_recognition_error": "",
                "no_progress_since": "",
                "last_progress_at": utc_now_iso(),
                "goal_revision": 1,
                "goal_replacement_count": 0,
                "previous_goal": {},
                "last_goal_replaced_at": "",
                "goal_update_state": "",
                "navigation_attempt_history": [],
                "child_action_cleanup": {},
                "lifecycle_reset_count": node.lifecycle_reset_count,
                "lifecycle_recovery": {},
                "failure_stage": "",
                "message": "目标已下发，等待 Nav2 规划路线",
                "last_error": "",
                "pid": os.getpid(),
            }
        )

        route_replaced = False
        for loop_index in range(loop_count):
            for point_index, point in enumerate(points):
                if stop_flag["stop"]:
                    raise KeyboardInterrupt()
                active_point = point
                active_index = point_index
                active_loop = loop_index + 1
                navigation_attempt = 0
                while True:
                    validation = node.validate_navigation_pose(
                        active_point,
                        label=str(active_point.get("label") or f"P{active_index + 1}"),
                    )
                    merge_status({"route_point_validation": validation})
                    if not validation.get("valid"):
                        raise RuntimeError(
                            f"navigation point {validation.get('label')} rejected: "
                            f"{validation.get('reason')}"
                        )
                    navigation_attempt += 1
                    merge_status(
                        {
                            "mode": "navigating",
                            "state": "navigating",
                            "task_phase": "navigating",
                            "task_running": True,
                            "goal_active": True,
                            "current_index": active_index,
                            "loop_index": active_loop,
                            "current_goal": active_point,
                            "completed_count": completed,
                            "navigation_attempt": navigation_attempt,
                            "no_progress_since": "",
                            "message": (
                                f"正在导航：目标 {active_index + 1}/{total_points}，"
                                f"循环 {active_loop}/{loop_count}，第 {navigation_attempt}/3 次"
                            ),
                        }
                    )
                    status_context = {
                        "session_id": task_meta["session_id"],
                        "current_index": active_index,
                        "loop_index": active_loop,
                        "current_goal": active_point,
                        "last_goal": active_point,
                        "navigation_attempt": navigation_attempt,
                        "goal_update_state": "accepted" if route_replaced else "",
                        "message": f"正在导航：目标 {active_index + 1}/{total_points}",
                    }
                    attempt_started_at = utc_now_iso()
                    attempt_started_mono = time.monotonic()
                    failure_message = ""
                    cleanup: dict[str, Any] = {}
                    lifecycle_required = False
                    try:
                        outcome = node.send_goal(
                            active_point,
                            executor,
                            stop_flag,
                            status_context=status_context,
                        )
                    except NavigationControlError as exc:
                        outcome = {"kind": "result", "status": GoalStatus.STATUS_UNKNOWN}
                        failure_message = str(exc)
                        cleanup = dict(exc.cleanup)
                        lifecycle_required = True
                    except NavigationCleanupError as exc:
                        outcome = {"kind": "result", "status": GoalStatus.STATUS_UNKNOWN}
                        failure_message = str(exc)
                        lifecycle_required = True
                    except KeyboardInterrupt:
                        raise
                    except Exception as exc:
                        outcome = {"kind": "result", "status": GoalStatus.STATUS_UNKNOWN}
                        failure_message = str(exc)

                    if outcome.get("kind") != "replacement":
                        status_code = int(outcome.get("status", GoalStatus.STATUS_UNKNOWN))
                        failure_message = str(outcome.get("failure_message") or failure_message)
                        if status_code == GoalStatus.STATUS_SUCCEEDED:
                            append_navigation_attempt(
                                {
                                    "label": str(active_point.get("label") or f"P{active_index + 1}"),
                                    "point_index": active_index,
                                    "returning": False,
                                    "attempt": navigation_attempt,
                                    "started_at": attempt_started_at,
                                    "finished_at": utc_now_iso(),
                                    "duration_sec": round(time.monotonic() - attempt_started_mono, 3),
                                    "status_code": status_code,
                                    "result": "succeeded",
                                    "arrival_confirmation": dict(outcome.get("arrival_confirmation") or {}),
                                    "cleanup": {"required": False, "confirmed": True},
                                    "stabilizer": node.stabilizer_snapshot(),
                                }
                            )
                            break

                        node.publish_stop(executor)
                        if not cleanup:
                            cleanup = node.cleanup_navigation_children(executor, timeout_sec=3.0)
                        recovery: dict[str, Any] = {}
                        if lifecycle_required or not cleanup.get("confirmed"):
                            recovery = node.recover_navigation_lifecycle(
                                executor,
                                reason=failure_message or str(cleanup.get("reason") or "action_cleanup_failed"),
                            )
                            if not recovery.get("success"):
                                raise NavigationRecoveryError(
                                    failure_message or "navigation lifecycle recovery failed"
                                )
                        append_navigation_attempt(
                            {
                                "label": str(active_point.get("label") or f"P{active_index + 1}"),
                                "point_index": active_index,
                                "returning": False,
                                "attempt": navigation_attempt,
                                "started_at": attempt_started_at,
                                "finished_at": utc_now_iso(),
                                "duration_sec": round(time.monotonic() - attempt_started_mono, 3),
                                "status_code": status_code,
                                "result": "failed",
                                "failure_message": failure_message,
                                "invalid_success": bool(outcome.get("invalid_success")),
                                "arrival_confirmation": dict(outcome.get("arrival_confirmation") or {}),
                                "cleanup": cleanup,
                                "lifecycle_recovery": recovery,
                                "stabilizer": node.stabilizer_snapshot(),
                            }
                        )
                        merge_status(
                            {
                                "goal_active": False,
                                "failure_stage": (
                                    "invalid_success"
                                    if outcome.get("invalid_success")
                                    else "navigate_to_pose"
                                ),
                                "child_action_cleanup": cleanup,
                                "lifecycle_recovery": recovery,
                                "message": (
                                    f"目标第 {navigation_attempt}/3 次失败："
                                    f"{failure_message or status_code}"
                                ),
                            }
                        )
                        if navigation_attempt >= 3:
                            raise RuntimeError(
                                failure_message or f"goal failed with status {status_code}"
                            )
                        wait_with_spin(node, executor, stop_flag, 1.0)
                        continue

                    request = outcome["request"]
                    previous_goal = active_point
                    active_point = request["goal_pose"]
                    active_index = 0
                    active_loop = 1
                    points = [active_point]
                    total_points = 1
                    loop_count = 1
                    completed = 0
                    route_replaced = True
                    navigation_attempt = 0
                    previous_status = read_json(STATUS_PATH)
                    goal_revision = int(previous_status.get("goal_revision", 0) or 0) + 1
                    replacement_count = int(previous_status.get("goal_replacement_count", 0) or 0) + 1
                    merge_status(
                        {
                            "mode": "planning",
                            "state": "planning",
                            "task_phase": "planning",
                            "task_running": True,
                            "goal_active": False,
                            "points": points,
                            "point_labels": point_labels(points),
                            "current_index": 0,
                            "loop_index": 1,
                            "loop_count": 1,
                            "completed_count": 0,
                            "previous_goal": previous_goal,
                            "current_goal": active_point,
                            "goal_pose": active_point,
                            "route_source": "live_retarget",
                            "goal_revision": goal_revision,
                            "goal_replacement_count": replacement_count,
                            "last_goal_replaced_at": utc_now_iso(),
                            "goal_update_state": "planning",
                            "goal_refresh_count": 0,
                            "goal_refresh_suppressed": False,
                            "goal_refresh_suppress_distance_m": 0.0,
                            "goal_refresh_suppress_delay_s": 0.0,
                            "goal_refresh_final_send_pending": False,
                            "final_heading_state": "",
                            "final_heading_delay_s": 0.0,
                            "final_heading_error_deg": None,
                            "last_goal_refresh_at": "",
                            "no_progress_since": "",
                            "last_progress_at": utc_now_iso(),
                            "message": "新终点已接受，正在等待 Nav2 重新规划",
                        }
                    )
                completed += 1
                merge_status(
                    {
                        "mode": "navigating",
                        "state": "navigating",
                        "task_phase": "navigating",
                        "task_running": True,
                        "goal_active": False,
                        "current_index": active_index,
                        "loop_index": active_loop,
                        "current_goal": points[active_index + 1] if active_index + 1 < total_points else {},
                        "last_goal": active_point,
                        "completed_count": completed,
                        "goal_update_state": "accepted" if route_replaced else "",
                        "message": f"目标 {active_index + 1}/{total_points} 已到达",
                    }
                )
                if route_replaced:
                    break
            if route_replaced:
                break

        merge_status(
            {
                "mode": "completed",
                "state": "completed",
                "task_phase": "terminal",
                "task_running": False,
                "goal_active": False,
                "goal_refresh_final_send_pending": False,
                "final_heading_state": "",
                "current_index": total_points - 1,
                "loop_index": loop_count,
                "completed_count": completed,
                "current_goal": {},
                "last_goal": points[-1] if points else {},
                "message": f"导航完成，共到达 {completed} 个目标",
                "last_result": f"导航完成，共到达 {completed} 个目标",
                "last_error": "",
                "auto_cabinet_inspection": False,
                "auto_recognition_state": "disabled",
                "auto_recognition_command_id": "",
                "auto_recognition_error": "",
                "pid": 0,
            }
        )
        print(f"cruise completed points={completed}", flush=True)
        return 0
    except KeyboardInterrupt:
        node.publish_stop(executor)
        cleanup = node.cleanup_navigation_children(executor, timeout_sec=3.0)
        merge_status(
            {
                "mode": "canceled",
                "state": "canceled",
                "task_phase": "terminal",
                "task_running": False,
                "goal_active": False,
                "goal_refresh_final_send_pending": False,
                "final_heading_state": "",
                "current_goal": {},
                "message": "路线已停止，导航系统保持就绪",
                "last_result": "路线已停止，导航系统保持就绪",
                "child_action_cleanup": cleanup,
                "pid": 0,
            }
        )
        print("cruise canceled", flush=True)
        return 130
    except Exception as exc:
        node.publish_stop(executor)
        cleanup = node.cleanup_navigation_children(executor, timeout_sec=3.0)
        if not cleanup.get("confirmed") and not isinstance(
            exc, NavigationCleanupError
        ):
            exc = NavigationCleanupError(
                f"{exc}; child action cleanup was not confirmed"
            )
        merge_status(
            {
                "mode": "failed",
                "state": "failed",
                "task_phase": "terminal",
                "task_running": False,
                "goal_active": False,
                "goal_refresh_final_send_pending": False,
                "final_heading_state": "",
                "current_goal": {},
                "message": str(exc),
                "last_error": str(exc),
                "child_action_cleanup": cleanup,
                "failure_stage": (
                    "child_action_cleanup"
                    if not cleanup.get("confirmed")
                    else "navigation_worker"
                ),
                "pid": 0,
            }
        )
        print(f"cruise failed: {exc}", flush=True)
        return 1
    finally:
        if stop_flag["stop"]:
            stop_inspection_job(str(read_json(STATUS_PATH).get("active_inspection_id") or ""))
        clear_pid_file()
        remove_file(GOAL_UPDATE_PATH)
        remove_file(GOAL_UPDATE_ACK_PATH)
        with contextlib.suppress(Exception):
            node.action_client.destroy()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Cloud navigation helper for initial pose and waypoint cruise.")
    sub = parser.add_subparsers(dest="command", required=True)

    parser_pose = sub.add_parser("set-initialpose")
    parser_pose.add_argument("--pose-json", required=True)
    parser_pose.set_defaults(func=command_set_initial_pose)

    parser_start = sub.add_parser("start-cruise")
    parser_start.add_argument("--points-json", required=True)
    parser_start.add_argument("--loop-count", default="1")
    parser_start.add_argument("--start-pose-json", default="")
    parser_start.add_argument("--task-json", default="{}")
    parser_start.set_defaults(func=command_start_cruise)

    parser_run = sub.add_parser("run-cruise")
    parser_run.add_argument("--points-json", required=True)
    parser_run.add_argument("--loop-count", default="1")
    parser_run.add_argument("--start-pose-json", default="")
    parser_run.add_argument("--task-json", default="{}")
    parser_run.set_defaults(func=command_run_cruise)

    parser_stop = sub.add_parser("stop-cruise")
    parser_stop.set_defaults(func=command_stop_cruise)

    parser_replace = sub.add_parser("replace-goal")
    parser_replace.add_argument("--goal-json", required=True)
    parser_replace.add_argument("--session-id", required=True)
    parser_replace.add_argument("--map-name", default="")
    parser_replace.add_argument("--request-id", default="")
    parser_replace.set_defaults(func=command_replace_goal)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())

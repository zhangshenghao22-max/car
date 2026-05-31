#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import contextlib
import io
import json
import math
import os
from pathlib import Path as FilePath
import subprocess
import threading
import time
from collections import deque
from typing import Any

import requests

try:
    from PIL import Image, ImageDraw
except Exception:
    Image = None
    ImageDraw = None

try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
except Exception:
    rclpy = None
    PoseStamped = None
    PoseWithCovarianceStamped = None
    Twist = None
    OccupancyGrid = None
    Odometry = None
    Path = None
    Context = None
    SingleThreadedExecutor = None
    Node = object
    LaserScan = None
    DurabilityPolicy = None
    HistoryPolicy = None
    QoSProfile = None
    ReliabilityPolicy = None
    qos_profile_sensor_data = None

try:
    from .teleop_shared import empty_teleop_state, sanitize_teleop_state
except Exception:
    from teleop_shared import empty_teleop_state, sanitize_teleop_state


def now_s() -> float:
    return time.time()


ROOT_DIR = FilePath(__file__).resolve().parent
MAP_DIR = ROOT_DIR / "lidar_maps"
NAV_COMMAND_STATUS_PATH = ROOT_DIR / "runtime" / "cloud_nav_command" / "status.json"


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def iso_from_timestamp(value: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(value))


def read_image_size(image_path: FilePath) -> tuple[int, int] | None:
    if not image_path.exists() or Image is None:
        return None
    try:
        with Image.open(image_path) as image:
            width, height = image.size
        if int(width) > 0 and int(height) > 0:
            return int(width), int(height)
    except Exception:
        return None
    return None


def parse_saved_map_meta(yaml_path: FilePath, image_path: FilePath | None) -> dict[str, Any]:
    meta: dict[str, Any] = {}
    try:
        text = yaml_path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        try:
            text = yaml_path.read_text(encoding="utf-8-sig")
        except Exception:
            return meta
    except Exception:
        return meta

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, raw_value = line.split(":", 1)
        key = key.strip()
        value = raw_value.strip()
        if key == "resolution":
            try:
                meta["resolution"] = float(value)
            except Exception:
                pass
        elif key == "origin":
            try:
                origin = ast.literal_eval(value)
            except Exception:
                origin = None
            if isinstance(origin, (list, tuple)) and len(origin) >= 2:
                try:
                    meta["origin_x"] = float(origin[0])
                    meta["origin_y"] = float(origin[1])
                except Exception:
                    pass

    if image_path is not None:
        image_size = read_image_size(image_path)
        if image_size is not None:
            meta["width"], meta["height"] = image_size

    required_keys = {"width", "height", "resolution", "origin_x", "origin_y"}
    if not required_keys.issubset(meta):
        return {}
    return meta


def list_saved_maps(map_dir: FilePath) -> list[dict[str, Any]]:
    if not map_dir.exists():
        return []

    items: list[dict[str, Any]] = []
    for yaml_path in sorted(map_dir.glob("*.yaml")):
        stem = yaml_path.stem
        pgm_path = yaml_path.with_suffix(".pgm")
        png_path = yaml_path.with_suffix(".png")
        image_path = pgm_path if pgm_path.exists() else png_path if png_path.exists() else None
        try:
            stat = yaml_path.stat()
        except OSError:
            continue
        preview_meta = parse_saved_map_meta(yaml_path, image_path)
        items.append(
            {
                "name": stem,
                "yaml": yaml_path.name,
                "has_image": bool(image_path and image_path.exists()),
                "image_path": str(image_path) if image_path and image_path.exists() else "",
                "preview_meta": preview_meta,
                "updated_at": iso_from_timestamp(stat.st_mtime),
                "size_bytes": int(stat.st_size),
            }
        )

    items.sort(key=lambda item: item.get("updated_at", ""), reverse=True)
    return items


def saved_map_preview_bytes(image_path: FilePath) -> bytes | None:
    if not image_path.exists():
        return None
    if Image is None:
        if image_path.suffix.lower() == ".png":
            return image_path.read_bytes()
        return None
    try:
        with Image.open(image_path) as image:
            if image.mode not in {"RGB", "RGBA"}:
                image = image.convert("RGB")
            buffer = io.BytesIO()
            image.save(buffer, format="PNG")
            return buffer.getvalue()
    except Exception:
        return None


def sync_saved_map_previews(
    *,
    session: requests.Session,
    server_url: str,
    headers: dict[str, str],
    cache: dict[str, dict[str, Any]],
    items: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    synced: list[dict[str, Any]] = []
    for item in items:
        entry = {key: value for key, value in item.items() if key != "image_path"}
        name = str(item.get("name") or "").strip()
        image_path_raw = str(item.get("image_path") or "").strip()
        if not name or not image_path_raw:
            synced.append(entry)
            continue

        image_path = FilePath(image_path_raw)
        try:
            stat = image_path.stat()
        except OSError:
            synced.append(entry)
            continue

        stamp = f"{stat.st_mtime_ns}:{stat.st_size}"
        cached = cache.get(name, {})
        if cached.get("stamp") != stamp:
            payload = saved_map_preview_bytes(image_path)
            if payload is not None:
                try:
                    response = session.post(
                        f"{server_url.rstrip('/')}/api/upload/saved-map-preview/{requests.utils.quote(name, safe='')}",
                        headers=headers,
                        files={"file": (f"{name}.png", payload, "image/png")},
                        timeout=20,
                    )
                    response.raise_for_status()
                    body = response.json()
                    preview = body.get("preview", {}) if isinstance(body, dict) else {}
                    if isinstance(preview, dict) and preview.get("url"):
                        cache[name] = {
                            "stamp": stamp,
                            "url": str(preview.get("url")),
                            "uploaded_at": str(preview.get("uploaded_at", "")),
                        }
                except Exception as exc:
                    print(f"[uploader] saved map preview upload failed for {name}: {exc}", flush=True)

        cached = cache.get(name, {})
        if cached.get("url"):
            entry["preview_url"] = cached.get("url", "")
            entry["preview_uploaded_at"] = cached.get("uploaded_at", "")
        synced.append(entry)
    return synced


def pose_to_dict(position, orientation) -> dict[str, float]:
    return {
        "x": float(position.x),
        "y": float(position.y),
        "yaw": quaternion_to_yaw(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ),
    }


def capture_mjpeg_frame(url: str, timeout: float = 8.0) -> bytes | None:
    with requests.get(url, stream=True, timeout=timeout) as response:
        response.raise_for_status()
        buffer = bytearray()
        start = b"\xff\xd8"
        end = b"\xff\xd9"
        for chunk in response.iter_content(chunk_size=4096):
            if not chunk:
                continue
            buffer.extend(chunk)
            begin = buffer.find(start)
            if begin < 0:
                continue
            finish = buffer.find(end, begin + 2)
            if finish < 0:
                continue
            return bytes(buffer[begin : finish + 2])
    return None


def load_json_file(path: FilePath) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        return payload if isinstance(payload, dict) else {}
    except Exception:
        return {}


def occupancy_grid_to_png(msg: OccupancyGrid) -> bytes | None:
    if Image is None:
        return None
    width = int(msg.info.width)
    height = int(msg.info.height)
    if width <= 0 or height <= 0 or len(msg.data) != width * height:
        return None

    pixels: list[int] = []
    for value in msg.data:
        if value < 0:
            pixels.append(205)
        elif value >= 65:
            pixels.append(0)
        else:
            pixels.append(max(0, min(255, 255 - int(value * 255 / 100))))

    image = Image.new("L", (width, height))
    image.putdata(pixels)
    image = image.transpose(Image.FLIP_TOP_BOTTOM).convert("RGB")
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


def laser_scan_to_png(msg: LaserScan, size: int = 720) -> bytes | None:
    if Image is None or ImageDraw is None:
        return None

    image = Image.new("RGB", (size, size), (248, 250, 252))
    draw = ImageDraw.Draw(image)
    center = size / 2.0

    valid_points: list[tuple[float, float]] = []
    angle = float(msg.angle_min)
    finite_ranges: list[float] = []
    for item in msg.ranges:
        rng = float(item)
        if math.isfinite(rng) and rng > 0.0:
            if msg.range_min > 0.0 and rng < float(msg.range_min):
                angle += float(msg.angle_increment)
                continue
            if msg.range_max > 0.0 and rng > float(msg.range_max):
                angle += float(msg.angle_increment)
                continue
            valid_points.append((angle, rng))
            finite_ranges.append(rng)
        angle += float(msg.angle_increment)

    for factor in (0.16, 0.30, 0.44):
        radius = center * factor
        draw.ellipse(
            (center - radius, center - radius, center + radius, center + radius),
            outline=(224, 231, 239),
            width=1,
        )

    draw.line((center, 0, center, size), fill=(229, 231, 235), width=1)
    draw.line((0, center, size, center), fill=(229, 231, 235), width=1)

    if valid_points:
        max_range = max(finite_ranges)
        if float(msg.range_max) > 0.0:
            max_range = min(max_range, float(msg.range_max))
        scale = (center * 0.86) / max(max_range, 0.001)
        for angle, rng in valid_points:
            px = center + math.cos(angle) * rng * scale
            py = center - math.sin(angle) * rng * scale
            draw.ellipse((px - 1.5, py - 1.5, px + 1.5, py + 1.5), fill=(31, 41, 55))

    draw.ellipse((center - 5, center - 5, center + 5, center + 5), fill=(22, 156, 107))

    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return buffer.getvalue()


class RosMirror(Node):
    def __init__(self, *, context: Context, teleop_topic: str):
        super().__init__("car_cloud_uploader", context=context)
        self.logs: deque[str] = deque(maxlen=80)
        self._log_lock = threading.Lock()
        self.map_meta: dict[str, Any] = {}
        self.map_frame: bytes | None = None
        self.scan_frame: bytes | None = None
        self.nav_path: dict[str, Any] = {"topic": "", "points": []}
        self.robot_pose: dict[str, Any] = {}
        self.goal_pose: dict[str, Any] = {}
        self.last_map_at = 0.0
        self.last_scan_at = 0.0
        self.last_path_at = 0.0
        self.last_pose_at = 0.0
        self.last_goal_at = 0.0
        self.robot_pose_source = ""
        self._last_summary = ""
        self._teleop_topic = str(teleop_topic or "/cmd_vel_cmd").strip() or "/cmd_vel_cmd"
        self._teleop_state: dict[str, Any] = empty_teleop_state()
        self._teleop_state_lock = threading.Lock()
        self._teleop_last_publish_at = 0.0
        self._teleop_zero_sent = True

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, "/plan", self._on_plan, reliable_qos)
        self.create_subscription(Path, "/global_plan", self._on_plan, reliable_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, reliable_qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, reliable_qos)
        self.create_subscription(PoseStamped, "/move_base_simple/goal", self._on_goal, reliable_qos)
        self._teleop_publisher = self.create_publisher(Twist, self._teleop_topic, reliable_qos)

        self.note("uploader connected to ROS topics")
        self.note(f"cloud teleop publisher ready on {self._teleop_topic}")

    def note(self, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        entry = f"[{stamp}] {message}"
        with self._log_lock:
            if not self.logs or self.logs[-1] != entry:
                self.logs.append(entry)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_meta = {
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
        }
        rendered = occupancy_grid_to_png(msg)
        if rendered:
            self.map_frame = rendered
        self.last_map_at = now_s()

    def _on_scan(self, msg: LaserScan) -> None:
        rendered = laser_scan_to_png(msg)
        if rendered:
            self.scan_frame = rendered
        self.last_scan_at = now_s()

    def _on_plan(self, msg: Path) -> None:
        topic_name = msg._topic_name if hasattr(msg, "_topic_name") else "/plan"
        self.nav_path = {
            "topic": topic_name,
            "points": [{"x": float(item.pose.position.x), "y": float(item.pose.position.y)} for item in msg.poses],
        }
        self.last_path_at = now_s()

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.robot_pose = pose_to_dict(msg.pose.pose.position, msg.pose.pose.orientation)
        self.robot_pose_source = "/amcl_pose"
        self.last_pose_at = now_s()

    def _on_odom(self, msg: Odometry) -> None:
        if self.last_pose_at and now_s() - self.last_pose_at < 1.5 and self.robot_pose_source == "/amcl_pose":
            return
        self.robot_pose = pose_to_dict(msg.pose.pose.position, msg.pose.pose.orientation)
        self.robot_pose_source = "/odom"
        self.last_pose_at = now_s()

    def _on_goal(self, msg: PoseStamped) -> None:
        self.goal_pose = pose_to_dict(msg.pose.position, msg.pose.orientation)
        self.last_goal_at = now_s()

    def _fresh(self, stamp: float, timeout_s: float) -> bool:
        return stamp > 0.0 and (now_s() - stamp) <= timeout_s

    def set_cloud_teleop(self, teleop_payload: dict[str, Any]) -> None:
        clean = sanitize_teleop_state(teleop_payload, board_id=str(teleop_payload.get("board_id") or "").strip())
        with self._teleop_state_lock:
            previous_status = str(self._teleop_state.get("status") or "")
            previous_controller = str(self._teleop_state.get("controller_id") or "")
            self._teleop_state = clean
        status = str(clean.get("status") or "idle")
        controller_id = str(clean.get("controller_id") or "")
        if status != previous_status or controller_id != previous_controller:
            owner = controller_id[:8] if controller_id else "-"
            self.note(f"teleop status={status} owner={owner}")

    def _publish_cloud_twist(self, *, linear_x: float, linear_y: float, angular_z: float) -> None:
        message = Twist()
        message.linear.x = float(linear_x)
        message.linear.y = float(linear_y)
        message.angular.z = float(angular_z)
        self._teleop_publisher.publish(message)
        self._teleop_last_publish_at = now_s()
        self._teleop_zero_sent = (
            abs(float(linear_x)) <= 1e-9
            and abs(float(linear_y)) <= 1e-9
            and abs(float(angular_z)) <= 1e-9
        )

    def pump_cloud_teleop(self) -> None:
        with self._teleop_state_lock:
            current = dict(self._teleop_state)

        enabled = bool(current.get("enabled"))
        twist = current.get("twist", {}) if isinstance(current.get("twist"), dict) else {}
        linear_x = float(twist.get("linear_x", 0.0) or 0.0)
        linear_y = float(twist.get("linear_y", 0.0) or 0.0)
        angular_z = float(twist.get("angular_z", 0.0) or 0.0)

        if enabled:
            self._publish_cloud_twist(linear_x=linear_x, linear_y=linear_y, angular_z=angular_z)
            return

        if not self._teleop_zero_sent:
            self._publish_cloud_twist(linear_x=0.0, linear_y=0.0, angular_z=0.0)

    def teleop_snapshot(self) -> dict[str, Any]:
        with self._teleop_state_lock:
            current = dict(self._teleop_state)
        return {
            "topic": self._teleop_topic,
            "enabled": bool(current.get("enabled")),
            "status": str(current.get("status") or "idle"),
            "controller_id": str(current.get("controller_id") or ""),
            "speed_level": int(current.get("speed_level", 0) or 0),
            "pressed_keys": list(current.get("pressed_keys", [])) if isinstance(current.get("pressed_keys"), list) else [],
            "twist": dict(current.get("twist", {})) if isinstance(current.get("twist"), dict) else {},
            "last_publish_at": iso_from_timestamp(self._teleop_last_publish_at) if self._teleop_last_publish_at > 0 else "",
            "subscriber_count": int(self.count_subscribers(self._teleop_topic)),
        }

    def cloud_snapshot(self) -> dict[str, Any]:
        map_fresh = self._fresh(self.last_map_at, 5.0)
        scan_fresh = self._fresh(self.last_scan_at, 5.0)
        path_fresh = self._fresh(self.last_path_at, 3.0) and bool(self.nav_path.get("points"))
        pose_fresh = self._fresh(self.last_pose_at, 3.0)
        goal_fresh = self._fresh(self.last_goal_at, 30.0)

        if path_fresh or goal_fresh:
            mode = "navigation"
        elif map_fresh or scan_fresh:
            mode = "mapping"
        else:
            mode = "idle"

        summary = f"mode={mode} map={int(map_fresh)} scan={int(scan_fresh)} path={int(path_fresh)} pose={int(pose_fresh)}"
        if summary != self._last_summary:
            self.note(summary)
            self._last_summary = summary

        current_goal = self.goal_pose if goal_fresh else {}
        with self._log_lock:
            logs = list(self.logs)
        return {
            "ros_status": {
                "mapping_live": bool(map_fresh or scan_fresh),
                "navigation_mode": mode,
                "scan_fresh": scan_fresh,
                "map_fresh": map_fresh,
                "odom_fresh": pose_fresh,
                "imu_filtered_live": False,
                "ekf_ready": pose_fresh,
                "last_ready_error": "",
            },
            "nav_status": {
                "nav_state": "goal_active" if path_fresh else ("goal_received" if goal_fresh else "idle"),
                "action_server_ready": bool(path_fresh or goal_fresh),
                "goal_active": path_fresh,
                "task_running": path_fresh,
                "blocked": False,
                "safety_interlock_reason": "",
                "goal_feedback": f"route points {len(self.nav_path.get('points', []))}" if path_fresh else "",
                "last_result": "",
                "last_error": "",
                "current_goal": current_goal,
            },
            "map_meta": dict(self.map_meta),
            "map_frame": self.map_frame if map_fresh else None,
            "scan_frame": self.scan_frame if scan_fresh else None,
            "nav_path": dict(self.nav_path),
            "robot_pose": dict(self.robot_pose) if pose_fresh else {},
            "goal_pose": dict(current_goal),
            "logs": logs,
        }


class CommandRunner:
    def __init__(self, command: dict[str, Any], ros_node: RosMirror):
        self.command = command
        self.ros_node = ros_node
        self.action = str(command.get("action") or "").strip()
        self.command_id = str(command.get("id") or "").strip()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name=f"cmd-{self.command_id}", daemon=True)
        self._done = False
        self._status = "running"
        self._result: dict[str, Any] = {"message": f"command running: {self.action}"}
        self._running_uploaded = False
        self._started_at = now_s()

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        status, result = execute_whitelisted_command(self.command, self.ros_node)
        with self._lock:
            self._status = status
            self._result = result
            self._done = True

    def mark_running_uploaded(self) -> None:
        with self._lock:
            self._running_uploaded = True

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "command_id": self.command_id,
                "action": self.action,
                "done": self._done,
                "status": self._status,
                "result": dict(self._result),
                "running_uploaded": self._running_uploaded,
                "started_at": self._started_at,
            }


def safe_get_json(session: requests.Session, url: str) -> dict[str, Any]:
    try:
        response = session.get(url, timeout=3)
        response.raise_for_status()
        payload = response.json()
        return payload if isinstance(payload, dict) else {}
    except Exception:
        return {}


def safe_capture_frame(url: str) -> bytes | None:
    try:
        return capture_mjpeg_frame(url)
    except Exception:
        return None


def merged_logs(*chunks: list[str]) -> list[str]:
    merged: list[str] = []
    for chunk in chunks:
        for item in chunk:
            text = str(item).strip()
            if text:
                merged.append(text)
    return merged[-80:]


def request_json(
    session: requests.Session,
    method: str,
    url: str,
    *,
    headers: dict[str, str] | None = None,
    json_payload: dict[str, Any] | None = None,
    timeout: float = 8.0,
) -> dict[str, Any]:
    response = session.request(method, url, headers=headers, json=json_payload, timeout=timeout)
    response.raise_for_status()
    payload = response.json()
    return payload if isinstance(payload, dict) else {}


def clip_text(text: str, *, limit: int = 2000) -> str:
    return text[-limit:] if len(text) > limit else text


def run_capture_recognition(params: dict[str, Any], ros_node: RosMirror) -> tuple[str, dict[str, Any]]:
    script = ROOT_DIR / "ros2_tools" / "camera_yolo_detect.py"
    if not script.exists():
        return "failed", {"message": f"YOLO script not found: {script}"}

    server_url = str(os.environ.get("CAR_CLOUD_SERVER_URL") or "").rstrip("/")
    upload_token = str(os.environ.get("CAR_CLOUD_UPLOAD_TOKEN") or "")
    board_id = str(os.environ.get("CAR_CLOUD_BOARD_ID") or "rk3588-f103-board")
    board_label = str(os.environ.get("CAR_CLOUD_BOARD_LABEL") or "RK3588 F103 Board")
    if not server_url:
        return "failed", {"message": "missing CAR_CLOUD_SERVER_URL"}
    if not upload_token:
        return "failed", {"message": "missing CAR_CLOUD_UPLOAD_TOKEN"}

    def param_float(name: str, default: float, low: float, high: float) -> float:
        try:
            value = float(params.get(name, default))
        except Exception:
            value = default
        return max(low, min(high, value))

    def param_int(name: str, default: int, low: int, high: int) -> int:
        try:
            value = int(params.get(name, default))
        except Exception:
            value = default
        return max(low, min(high, value))

    prefix = f"cloud_recognition_{time.strftime('%Y%m%d_%H%M%S')}"
    device = str(params.get("device") or "/dev/video0")
    conf = param_float("conf", 0.25, 0.01, 0.99)
    width = param_int("width", 640, 160, 3840)
    height = param_int("height", 480, 120, 2160)
    fps = param_int("fps", 15, 1, 60)

    cmd = [
        "python3",
        str(script),
        "--device",
        device,
        "--prefix",
        prefix,
        "--conf",
        f"{conf:.3f}",
        "--width",
        str(width),
        "--height",
        str(height),
        "--fps",
        str(fps),
        "--backend",
        "onnxruntime",
    ]
    ros_node.note("recognition capture start")
    try:
        completed = subprocess.run(
            cmd,
            cwd=str(ROOT_DIR),
            capture_output=True,
            text=True,
            timeout=90,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        ros_node.note("recognition capture timeout")
        return "failed", {
            "message": "recognition capture timed out",
            "stdout": clip_text(exc.stdout or ""),
            "stderr": clip_text(exc.stderr or ""),
        }
    except Exception as exc:
        ros_node.note("recognition capture exception")
        return "failed", {"message": str(exc)}

    if completed.returncode != 0:
        ros_node.note("recognition capture failed")
        return "failed", {
            "message": "recognition capture failed",
            "returncode": completed.returncode,
            "stdout": clip_text(completed.stdout or ""),
            "stderr": clip_text(completed.stderr or ""),
        }

    try:
        result = json.loads(completed.stdout)
    except Exception as exc:
        return "failed", {
            "message": f"recognition output is not JSON: {exc}",
            "stdout": clip_text(completed.stdout or ""),
            "stderr": clip_text(completed.stderr or ""),
        }
    if not isinstance(result, dict):
        return "failed", {"message": "recognition output JSON is not an object"}

    raw_path = FilePath(str(result.get("source_image") or ""))
    annotated_path = FilePath(str(result.get("annotated_image") or ""))
    if not raw_path.exists() or not annotated_path.exists():
        return "failed", {
            "message": "recognition image output missing",
            "source_image": str(raw_path),
            "annotated_image": str(annotated_path),
        }

    upload_payload = dict(result)
    upload_payload["board_id"] = board_id
    upload_payload["board_label"] = board_label
    try:
        with raw_path.open("rb") as raw_handle, annotated_path.open("rb") as annotated_handle:
            response = requests.post(
                f"{server_url}/api/upload/recognition",
                headers={"X-Upload-Token": upload_token},
                data={"payload": json.dumps(upload_payload, ensure_ascii=False)},
                files={
                    "raw_image": (raw_path.name, raw_handle, "image/jpeg"),
                    "annotated_image": (annotated_path.name, annotated_handle, "image/jpeg"),
                },
                timeout=30,
            )
        response.raise_for_status()
        uploaded = response.json()
    except Exception as exc:
        ros_node.note("recognition upload failed")
        return "failed", {
            "message": f"recognition upload failed: {exc}",
            "detections": result.get("detections", []),
            "source_image": str(raw_path),
            "annotated_image": str(annotated_path),
        }

    ros_node.note("recognition capture uploaded")
    return "completed", {
        "message": "recognition capture uploaded",
        "detections": result.get("detections", []),
        "detection_count": len(result.get("detections", []) if isinstance(result.get("detections"), list) else []),
        "inference_ms": result.get("inference_ms", ""),
        "backend": result.get("backend", ""),
        "recognition": uploaded.get("recognition", {}) if isinstance(uploaded, dict) else {},
    }


def execute_whitelisted_command(command: dict[str, Any], ros_node: RosMirror) -> tuple[str, dict[str, Any]]:
    action = str(command.get("action") or "").strip()
    params = command.get("params", {}) if isinstance(command.get("params"), dict) else {}

    env = os.environ.copy()
    env["CAR2_SKIP_UPLOADER_STOP"] = "1"
    nav_bridge = ROOT_DIR / "board_nav_command.py"

    if action == "start_mapping":
        cmd = ["bash", str(ROOT_DIR / "start_board_f103_mapping.sh")]
        timeout_s = 300
    elif action == "stop_mapping":
        cmd = ["bash", str(ROOT_DIR / "stop_board_f103_mapping.sh")]
        timeout_s = 120
    elif action == "save_map":
        map_name = str(params.get("name") or params.get("map_name") or params.get("map") or "").strip()
        cmd = ["bash", str(ROOT_DIR / "ros2_tools" / "save_ros2_map.sh")]
        if map_name:
            cmd.append(map_name)
        timeout_s = 180
    elif action == "start_navigation":
        map_name = str(params.get("map") or params.get("map_yaml") or params.get("map_name") or "").strip()
        if not map_name:
            return "failed", {"message": "missing navigation map"}
        cmd = ["bash", str(ROOT_DIR / "start_board_f103_navigation.sh"), map_name]
        timeout_s = 300
    elif action == "stop_navigation":
        cmd = ["bash", str(ROOT_DIR / "stop_board_f103_navigation.sh")]
        timeout_s = 120
    elif action == "set_initial_pose":
        pose = params.get("pose")
        if not isinstance(pose, dict):
            return "failed", {"message": "missing initial pose"}
        cmd = ["python3", str(nav_bridge), "set-initialpose", "--pose-json", json.dumps(pose, ensure_ascii=False)]
        timeout_s = 40
    elif action == "start_cruise":
        points = params.get("points")
        if not isinstance(points, list) or not points:
            return "failed", {"message": "missing cruise points"}
        loop_count = max(1, int(params.get("loop_count", 1) or 1))
        start_pose = params.get("start_pose", {})
        cmd = [
            "python3",
            str(nav_bridge),
            "start-cruise",
            "--points-json",
            json.dumps(points, ensure_ascii=False),
            "--loop-count",
            str(loop_count),
            "--start-pose-json",
            json.dumps(start_pose if isinstance(start_pose, dict) else {}, ensure_ascii=False),
        ]
        timeout_s = 40
    elif action == "stop_cruise":
        cmd = ["python3", str(nav_bridge), "stop-cruise"]
        timeout_s = 20
    elif action == "capture_recognition":
        return run_capture_recognition(params, ros_node)
    else:
        return "rejected", {"message": f"unsupported action: {action}"}

    ros_node.note(f"command start: {action}")
    try:
        if action in {"start_mapping", "stop_mapping", "start_navigation", "stop_navigation"} and nav_bridge.exists():
            subprocess.run(
                ["python3", str(nav_bridge), "stop-cruise"],
                cwd=str(ROOT_DIR),
                env=env,
                capture_output=True,
                text=True,
                timeout=20,
                check=False,
            )
        completed = subprocess.run(
            cmd,
            cwd=str(ROOT_DIR),
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        ros_node.note(f"command timeout: {action}")
        return "failed", {
            "message": f"command timed out: {action}",
            "stdout": clip_text(exc.stdout or ""),
            "stderr": clip_text(exc.stderr or ""),
        }
    except Exception as exc:
        ros_node.note(f"command exception: {action}")
        return "failed", {"message": str(exc)}

    if completed.returncode == 0:
        ros_node.note(f"command ok: {action}")
        return "completed", {
            "message": f"command completed: {action}",
            "stdout": clip_text(completed.stdout or ""),
            "stderr": clip_text(completed.stderr or ""),
        }

    ros_node.note(f"command failed: {action}")
    return "failed", {
        "message": f"command failed: {action}",
        "returncode": completed.returncode,
        "stdout": clip_text(completed.stdout or ""),
        "stderr": clip_text(completed.stderr or ""),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Upload current board mapping/navigation data to the cloud platform server.")
    parser.add_argument("--server-url", required=True, help="example: http://115.159.33.216:8765")
    parser.add_argument("--upload-token", required=True, help="must match server CAR_CLOUD_UPLOAD_TOKEN")
    parser.add_argument("--board-url", default="", help="optional local board web app URL, for example http://127.0.0.1:5000")
    parser.add_argument("--board-id", default="rk3588-f103-board", help="board identifier shown in the cloud")
    parser.add_argument("--board-label", default="RK3588 F103 Board", help="board label shown in the cloud")
    parser.add_argument("--state-interval", type=float, default=1.0, help="seconds between state uploads")
    parser.add_argument("--frame-interval", type=float, default=2.0, help="seconds between map/scan uploads")
    parser.add_argument("--command-interval", type=float, default=1.0, help="seconds between remote command polls")
    parser.add_argument("--teleop-interval", type=float, default=0.15, help="seconds between remote teleop polls")
    parser.add_argument("--teleop-topic", default="/cmd_vel_cmd", help="ROS topic used for cloud keyboard teleop")
    args = parser.parse_args()

    if rclpy is None:
        raise RuntimeError("rclpy is unavailable; source the ROS 2 environment before starting the uploader")

    os.environ["CAR_CLOUD_SERVER_URL"] = args.server_url.rstrip("/")
    os.environ["CAR_CLOUD_UPLOAD_TOKEN"] = args.upload_token
    os.environ["CAR_CLOUD_BOARD_ID"] = args.board_id
    os.environ["CAR_CLOUD_BOARD_LABEL"] = args.board_label

    session = requests.Session()
    headers = {"X-Upload-Token": args.upload_token}
    ros_context = Context()
    rclpy.init(args=None, context=ros_context)
    ros_node = RosMirror(context=ros_context, teleop_topic=args.teleop_topic)
    ros_executor = SingleThreadedExecutor(context=ros_context)
    ros_executor.add_node(ros_node)

    last_state_at = 0.0
    last_frame_at = 0.0
    last_command_at = 0.0
    last_teleop_at = 0.0
    last_teleop_success_at = 0.0
    active_command: CommandRunner | None = None
    pending_result_upload: dict[str, Any] | None = None
    saved_map_preview_cache: dict[str, dict[str, Any]] = {}

    try:
        while True:
            now = now_s()
            ros_executor.spin_once(timeout_sec=0.05)
            if now - last_teleop_at >= max(0.05, float(args.teleop_interval)):
                try:
                    teleop_payload = request_json(
                        session,
                        "GET",
                        f"{args.server_url.rstrip('/')}/api/board/teleop?board_id={args.board_id}",
                        headers=headers,
                        timeout=4,
                    )
                    teleop = teleop_payload.get("teleop", {})
                    ros_node.set_cloud_teleop(teleop if isinstance(teleop, dict) else {})
                    last_teleop_success_at = now
                except Exception as exc:
                    if last_teleop_success_at > 0 and (now - last_teleop_success_at) > 1.0:
                        ros_node.set_cloud_teleop({})
                    print(f"[uploader] teleop poll failed: {exc}", flush=True)
                last_teleop_at = now
            ros_node.pump_cloud_teleop()
            ros_snapshot = ros_node.cloud_snapshot()
            saved_maps = sync_saved_map_previews(
                session=session,
                server_url=args.server_url,
                headers=headers,
                cache=saved_map_preview_cache,
                items=list_saved_maps(MAP_DIR),
            )

            board_status = {"source": "ros_topics"}
            board_ros_status: dict[str, Any] = {}
            board_nav_status: dict[str, Any] = {}
            board_logs: list[str] = []

            if args.board_url:
                board_status = safe_get_json(session, f"{args.board_url}/api/status") or board_status
                board_ros_status = safe_get_json(session, f"{args.board_url}/api/ros/status")
                board_nav_status = safe_get_json(session, f"{args.board_url}/api/nav/status")
            board_logs = list(safe_get_json(session, f"{args.board_url}/api/logs").get("logs", []))

            cruise_status = load_json_file(NAV_COMMAND_STATUS_PATH)

            if now - last_state_at >= args.state_interval:
                ros_status = dict(ros_snapshot["ros_status"])
                ros_status.update({key: value for key, value in board_ros_status.items() if value not in (None, "", [], {})})

                nav_status = dict(ros_snapshot["nav_status"])
                nav_status.update({key: value for key, value in board_nav_status.items() if value not in (None, "", [], {})})
                if cruise_status:
                    cruise_mode = str(cruise_status.get("mode") or "").strip()
                    if cruise_mode:
                        nav_status["web_cruise_mode"] = cruise_mode
                    if "task_running" in cruise_status:
                        nav_status["task_running"] = bool(cruise_status.get("task_running"))
                    if "goal_active" in cruise_status:
                        nav_status["goal_active"] = bool(cruise_status.get("goal_active"))
                    if isinstance(cruise_status.get("current_goal"), dict) and cruise_status.get("current_goal"):
                        nav_status["current_goal"] = cruise_status.get("current_goal", {})
                    if cruise_status.get("last_result"):
                        nav_status["last_result"] = cruise_status.get("last_result", "")
                    if cruise_status.get("last_error"):
                        nav_status["last_error"] = cruise_status.get("last_error", "")
                    if bool(cruise_status.get("task_running")) and nav_status.get("nav_state") in {"", "idle", None}:
                        nav_status["nav_state"] = "task_running"
                    nav_status["active_task"] = {
                        "task_name": "web_cruise",
                        "mode": cruise_mode or "web_cruise",
                        "loop_count": int(cruise_status.get("loop_count", 1) or 1),
                        "current_loop": int(cruise_status.get("loop_index", 0) or 0),
                    } if bool(cruise_status.get("task_running")) else nav_status.get("active_task")

                goal_pose = ros_snapshot["goal_pose"]
                if not goal_pose and isinstance(nav_status.get("current_goal"), dict):
                    goal_pose = nav_status.get("current_goal", {})
                if not goal_pose and isinstance(cruise_status.get("current_goal"), dict):
                    goal_pose = cruise_status.get("current_goal", {})

                payload = {
                    "board_id": args.board_id,
                    "board_label": args.board_label,
                    "board": {
                        "saved_maps": saved_maps,
                        "cruise_status": cruise_status,
                    },
                    "status": board_status,
                    "ros_status": ros_status,
                    "nav_status": nav_status,
                    "telemetry": {
                        "teleop": ros_node.teleop_snapshot(),
                    },
                    "logs": merged_logs(ros_snapshot["logs"], board_logs),
                    "map_meta": ros_snapshot["map_meta"],
                    "nav_path": ros_snapshot["nav_path"],
                    "robot_pose": ros_snapshot["robot_pose"],
                    "goal_pose": goal_pose,
                }
                try:
                    response = session.post(
                        f"{args.server_url.rstrip('/')}/api/upload/state",
                        headers=headers,
                        json=payload,
                        timeout=8,
                    )
                    response.raise_for_status()
                    print(
                        f"[uploader] state ok mode={ros_status.get('navigation_mode')} "
                        f"map={int(bool(ros_status.get('map_fresh')))} "
                        f"scan={int(bool(ros_status.get('scan_fresh')))}",
                        flush=True,
                    )
                except Exception as exc:
                    print(f"[uploader] state upload failed: {exc}", flush=True)
                last_state_at = now

            if now - last_frame_at >= args.frame_interval:
                files: dict[str, tuple[str, bytes, str]] = {}
                map_frame = ros_snapshot["map_frame"]
                scan_frame = ros_snapshot["scan_frame"]

                if map_frame is None and args.board_url:
                    map_frame = safe_capture_frame(f"{args.board_url}/stream/lidar_map.mjpg")
                if scan_frame is None and args.board_url:
                    scan_frame = safe_capture_frame(f"{args.board_url}/stream/lidar_scan.mjpg")

                if map_frame:
                    files["map_frame"] = ("map.png", map_frame, "image/png")
                if scan_frame:
                    files["scan_frame"] = ("scan.png", scan_frame, "image/png")

                if files:
                    try:
                        response = session.post(
                            f"{args.server_url.rstrip('/')}/api/upload/snapshot",
                            headers=headers,
                            data={"payload": json.dumps({"board_id": args.board_id, "board_label": args.board_label})},
                            files=files,
                            timeout=12,
                        )
                        response.raise_for_status()
                        print(
                            f"[uploader] frame ok map={int('map_frame' in files)} scan={int('scan_frame' in files)}",
                            flush=True,
                        )
                    except Exception as exc:
                        print(f"[uploader] frame upload failed: {exc}", flush=True)
                last_frame_at = now

            if active_command is not None:
                snapshot = active_command.snapshot()
                if not snapshot["running_uploaded"]:
                    try:
                        request_json(
                            session,
                            "POST",
                            f"{args.server_url.rstrip('/')}/api/board/commands/{snapshot['command_id']}/result",
                            headers=headers,
                            json_payload={
                                "board_id": args.board_id,
                                "status": "running",
                                "result": {
                                    "message": f"command running: {snapshot['action']}",
                                },
                            },
                            timeout=20,
                        )
                        active_command.mark_running_uploaded()
                        print(f"[uploader] command {snapshot['action']} -> running", flush=True)
                    except Exception as exc:
                        print(f"[uploader] command running upload failed: {exc}", flush=True)

                if snapshot["done"] and pending_result_upload is None:
                    pending_result_upload = snapshot

            if pending_result_upload is not None:
                try:
                    request_json(
                        session,
                        "POST",
                        f"{args.server_url.rstrip('/')}/api/board/commands/{pending_result_upload['command_id']}/result",
                        headers=headers,
                        json_payload={
                            "board_id": args.board_id,
                            "status": pending_result_upload["status"],
                            "result": pending_result_upload["result"],
                        },
                        timeout=20,
                    )
                    print(
                        f"[uploader] command {pending_result_upload['action']} -> {pending_result_upload['status']}",
                        flush=True,
                    )
                    pending_result_upload = None
                    active_command = None
                except Exception as exc:
                    print(f"[uploader] command result upload failed: {exc}", flush=True)

            if active_command is None and now - last_command_at >= args.command_interval:
                try:
                    command_payload = request_json(
                        session,
                        "GET",
                        f"{args.server_url.rstrip('/')}/api/board/commands/next?board_id={args.board_id}",
                        headers=headers,
                        timeout=8,
                    )
                    command = command_payload.get("command")
                    if isinstance(command, dict) and command.get("id"):
                        action = str(command.get("action") or "")
                        ros_node.note(f"command claimed: {action}")
                        active_command = CommandRunner(command, ros_node)
                        active_command.start()
                except Exception as exc:
                    print(f"[uploader] command poll failed: {exc}", flush=True)
                last_command_at = now

            time.sleep(0.1)
    finally:
        with contextlib.suppress(Exception):
            ros_executor.remove_node(ros_node)
        with contextlib.suppress(Exception):
            ros_node.destroy_node()
        with contextlib.suppress(Exception):
            if ros_context.ok():
                rclpy.shutdown(context=ros_context)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

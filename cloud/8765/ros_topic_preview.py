from __future__ import annotations

import contextlib
import math
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

try:
    from map_msgs.msg import OccupancyGridUpdate
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.context import Context as RclpyContext
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener

    ROS_PREVIEW_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - depends on board runtime
    OccupancyGridUpdate = None
    rclpy = None
    OccupancyGrid = None
    RclpyContext = None
    SingleThreadedExecutor = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    DurabilityPolicy = None
    HistoryPolicy = None
    Time = None
    LaserScan = None
    Buffer = None
    TransformException = Exception
    TransformListener = None
    ROS_PREVIEW_IMPORT_ERROR = exc


POSE_HISTORY_LIMIT = 800
POSE_HISTORY_STEP_MM = 35.0


@dataclass
class RosPreviewSnapshot:
    available: bool
    running: bool
    scan_points: list[tuple[float, float, int]]
    map_image: np.ndarray
    pose_mm: tuple[float, float, float]
    pose_history_mm: list[tuple[float, float]]
    scan_frames: int
    map_frames: int
    last_scan_at: float
    last_map_at: float
    map_width_meters: float
    map_height_meters: float
    map_resolution: float
    last_error: str


class _PreviewNode(Node):
    def __init__(self, mirror: "RosPreviewMirror", *, context: RclpyContext):
        super().__init__("board_ros_preview", context=context)
        self._mirror = mirror

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.create_subscription(LaserScan, "/scan", self._on_scan, scan_qos)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        if OccupancyGridUpdate is not None:
            update_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(OccupancyGridUpdate, "/map_updates", self._on_map_update, update_qos)
        self._buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=20.0))
        self._listener = TransformListener(self._buffer, self, spin_thread=False)
        self.create_timer(0.25, self._update_pose)

    def _on_scan(self, msg: LaserScan):
        points: list[tuple[float, float, int]] = []
        angle = float(msg.angle_min)
        for distance in msg.ranges:
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max:
                angle_deg = math.degrees(angle) % 360.0
                points.append((angle_deg, float(distance) * 1000.0, 15))
            angle += float(msg.angle_increment)
        self._mirror.handle_scan(points)

    def _on_map(self, msg: OccupancyGrid):
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0:
            return

        data = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        image = np.full((height, width), 127, dtype=np.uint8)
        free_mask = data == 0
        occupied_mask = data >= 50
        partial_mask = (data > 0) & (data < 50)
        image[free_mask] = 255
        image[occupied_mask] = 0
        if np.any(partial_mask):
            scaled = np.clip(210 - (data[partial_mask] * 2), 110, 210)
            image[partial_mask] = scaled.astype(np.uint8)

        self._mirror.handle_map(
            image=image,
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            width_meters=float(msg.info.width) * float(msg.info.resolution),
            height_meters=float(msg.info.height) * float(msg.info.resolution),
        )

    def _on_map_update(self, msg: OccupancyGridUpdate):
        self._mirror.handle_map_update(
            x=int(msg.x),
            y=int(msg.y),
            width=int(msg.width),
            height=int(msg.height),
            data=list(msg.data),
        )

    def _update_pose(self):
        try:
            transform = self._buffer.lookup_transform("map", "base_link", Time())
        except TransformException:
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
        cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
        yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        self._mirror.handle_pose(float(translation.x), float(translation.y), float(yaw_deg))


class RosPreviewMirror:
    def __init__(self, log_callback=None):
        self.log_callback = log_callback
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._context: RclpyContext | None = None

        self._scan_points: list[tuple[float, float, int]] = []
        self._map_image = np.full((320, 320), 127, dtype=np.uint8)
        self._pose_mm = (0.0, 0.0, 0.0)
        self._pose_history_mm: list[tuple[float, float]] = []
        self._scan_frames = 0
        self._map_frames = 0
        self._last_scan_at = 0.0
        self._last_map_at = 0.0
        self._map_resolution = 0.05
        self._map_origin = (0.0, 0.0)
        self._map_width_meters = 16.0
        self._map_height_meters = 16.0
        self._last_error = ""

    @property
    def available(self) -> bool:
        return ROS_PREVIEW_IMPORT_ERROR is None

    def _log(self, message: str):
        if self.log_callback is not None:
            self.log_callback(message)

    def start(self):
        if not self.available:
            return
        if self._thread is not None and self._thread.is_alive():
            return
        with self._lock:
            self._last_error = ""
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None

    def reset(self):
        with self._lock:
            self._scan_points = []
            self._map_image = np.full((320, 320), 127, dtype=np.uint8)
            self._pose_mm = (0.0, 0.0, 0.0)
            self._pose_history_mm = []
            self._scan_frames = 0
            self._map_frames = 0
            self._last_scan_at = 0.0
            self._last_map_at = 0.0
            self._map_resolution = 0.05
            self._map_origin = (0.0, 0.0)
            self._map_width_meters = 16.0
            self._map_height_meters = 16.0
            self._last_error = ""

    def _worker(self):
        node = None
        executor = None
        context = None
        try:
            context = RclpyContext()
            rclpy.init(args=None, context=context)
            self._context = context
            node = _PreviewNode(self, context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self._log("ROS 预览订阅器已启动")
            while not self._stop_event.is_set():
                executor.spin_once(timeout_sec=0.2)
        except Exception as exc:  # pragma: no cover - board runtime only
            with self._lock:
                self._last_error = str(exc)
            self._log(f"ROS 预览订阅异常: {exc}")
        finally:
            if executor is not None:
                with contextlib.suppress(Exception):
                    executor.shutdown(timeout_sec=0.2)
            if executor is not None and node is not None:
                with contextlib.suppress(Exception):
                    executor.remove_node(node)
            if node is not None:
                with contextlib.suppress(Exception):
                    node.destroy_node()
            if context is not None:
                with contextlib.suppress(Exception):
                    if context.ok():
                        rclpy.shutdown(context=context)
            with self._lock:
                self._context = None

    def _append_pose_history(self, pose_mm: tuple[float, float, float]):
        current = (float(pose_mm[0]), float(pose_mm[1]))
        if not self._pose_history_mm:
            self._pose_history_mm.append(current)
            return
        previous = self._pose_history_mm[-1]
        if math.hypot(current[0] - previous[0], current[1] - previous[1]) >= POSE_HISTORY_STEP_MM:
            self._pose_history_mm.append(current)
        if len(self._pose_history_mm) > POSE_HISTORY_LIMIT:
            self._pose_history_mm = self._pose_history_mm[-POSE_HISTORY_LIMIT:]

    def handle_scan(self, points: list[tuple[float, float, int]]):
        with self._lock:
            self._scan_points = points
            self._scan_frames += 1
            self._last_scan_at = time.time()

    def handle_map(
        self,
        *,
        image: np.ndarray,
        resolution: float,
        origin_x: float,
        origin_y: float,
        width_meters: float,
        height_meters: float,
    ):
        with self._lock:
            self._map_image = image.copy()
            self._map_resolution = max(float(resolution), 1e-6)
            self._map_origin = (float(origin_x), float(origin_y))
            self._map_width_meters = max(float(width_meters), self._map_resolution)
            self._map_height_meters = max(float(height_meters), self._map_resolution)
            self._map_frames += 1
            self._last_map_at = time.time()

    def handle_pose(self, x_meters: float, y_meters: float, yaw_deg: float):
        with self._lock:
            origin_x, origin_y = self._map_origin
            pose_x_mm = max(0.0, (float(x_meters) - origin_x) * 1000.0)
            pose_y_mm = max(0.0, (float(y_meters) - origin_y) * 1000.0)
            self._pose_mm = (pose_x_mm, pose_y_mm, float(yaw_deg))
            self._append_pose_history(self._pose_mm)

    def handle_map_update(
        self,
        *,
        x: int,
        y: int,
        width: int,
        height: int,
        data: list[int],
    ):
        if width <= 0 or height <= 0:
            return
        patch_data = np.asarray(data, dtype=np.int16)
        if patch_data.size != width * height:
            return
        patch = patch_data.reshape((height, width))
        patch_image = np.full((height, width), 127, dtype=np.uint8)
        free_mask = patch == 0
        occupied_mask = patch >= 50
        partial_mask = (patch > 0) & (patch < 50)
        patch_image[free_mask] = 255
        patch_image[occupied_mask] = 0
        if np.any(partial_mask):
            scaled = np.clip(210 - (patch[partial_mask] * 2), 110, 210)
            patch_image[partial_mask] = scaled.astype(np.uint8)

        with self._lock:
            map_height, map_width = self._map_image.shape[:2]
            if map_width <= 0 or map_height <= 0:
                return
            left = max(0, int(x))
            top = max(0, int(y))
            right = min(map_width, left + width)
            bottom = min(map_height, top + height)
            if right <= left or bottom <= top:
                return
            patch_left = max(0, -int(x))
            patch_top = max(0, -int(y))
            patch_right = patch_left + (right - left)
            patch_bottom = patch_top + (bottom - top)
            self._map_image[top:bottom, left:right] = patch_image[patch_top:patch_bottom, patch_left:patch_right]
            self._map_frames += 1
            self._last_map_at = time.time()

    def snapshot(self) -> RosPreviewSnapshot:
        with self._lock:
            return RosPreviewSnapshot(
                available=self.available,
                running=bool(self._thread and self._thread.is_alive()),
                scan_points=list(self._scan_points),
                map_image=self._map_image.copy(),
                pose_mm=tuple(self._pose_mm),
                pose_history_mm=list(self._pose_history_mm),
                scan_frames=self._scan_frames,
                map_frames=self._map_frames,
                last_scan_at=self._last_scan_at,
                last_map_at=self._last_map_at,
                map_width_meters=self._map_width_meters,
                map_height_meters=self._map_height_meters,
                map_resolution=self._map_resolution,
                last_error=self._last_error,
            )

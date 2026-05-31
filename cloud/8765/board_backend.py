from __future__ import annotations

import collections
import contextlib
import errno
import glob
import json
import math
import os
import queue
import shlex
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import serial
from PIL import Image, ImageDraw, ImageFont
from serial.tools import list_ports

from avoidance_tab import DEFAULT_AVOIDANCE_THRESHOLD_MM, LidarAvoidanceBackend
from bluetooth_manager import (
    DEFAULT_BLE_NOTIFY_UUID,
    DEFAULT_BLE_SCAN_TIMEOUT,
    DEFAULT_BLE_WRITE_UUID,
    BleUartManager,
)
from lidar_slam_page import DEFAULT_LIDAR_PORT, LidarSlamBackend, LidarSlamWindow, MAP_EXPORT_DIR
from navigation_backend import NavigationSystem
from ros_topic_preview import ROS_PREVIEW_IMPORT_ERROR, RosPreviewMirror

try:
    import fcntl
except Exception:  # pragma: no cover - Windows fallback
    fcntl = None

try:
    import termios
except Exception:  # pragma: no cover - Windows fallback
    termios = None

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.context import Context as RclpyContext
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    ROS_ODOM_BRIDGE_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - depends on board runtime
    rclpy = None
    Odometry = None
    RclpyContext = None
    SingleThreadedExecutor = None
    Node = object
    QoSProfile = None
    ReliabilityPolicy = None
    HistoryPolicy = None
    ROS_ODOM_BRIDGE_IMPORT_ERROR = exc

try:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    ROS_ODOM_TF_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - depends on board runtime
    TransformStamped = None
    TransformBroadcaster = None
    StaticTransformBroadcaster = None
    ROS_ODOM_TF_IMPORT_ERROR = exc

try:
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient

    ROS_NAVIGATION_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - depends on board runtime
    GoalStatus = None
    PoseStamped = None
    NavigateToPose = None
    ActionClient = None
    ROS_NAVIGATION_IMPORT_ERROR = exc

BASE_DIR = Path(__file__).resolve().parent
YOLOV8_MODEL_PATH = BASE_DIR / "model" / "last" / "weights" / "best.pt"
YOLOV8_ONNX_PATH = BASE_DIR / "model" / "last" / "weights" / "best.onnx"
YOLOV8_LABELS_PATH = BASE_DIR / "model" / "last" / "weights" / "best.labels.json"
METER_PROJECT_ROOT = BASE_DIR / "model" / "Detect-and-read-meters-2" / "Detect-and-read-meters-2"
UPLOAD_MEDIA_DIR = BASE_DIR / "uploaded_media"
VISION_EXPORT_DIR = BASE_DIR / "vision_exports"
DEFAULT_SERIAL_BAUDRATE = 115200
DEFAULT_IMU_PORT = "/dev/myimu"
DEFAULT_IMU_BAUDRATE = 115200
DEFAULT_IMU_FRAME = "imu_link"
IMU_DRIVER_STATUS_STALE_AFTER_S = 1.2
IMU_DRIVER_STATUS_FILE_NAME = "imu_driver_status.json"
WIFI_RUNTIME_PROBE_LOG_NAME = "wifi_runtime_probe.log"
WIFI_RUNTIME_PROBE_STATE_FILE_NAME = "wifi_runtime_probe.state.json"
WIFI_RUNTIME_PROBE_INTERVAL_S = 5.0
CAR2_IMU_MODE_REQUIRED = "required"
DEFAULT_CAMERA_INDEX = 0
DEFAULT_CAMERA_FPS_LIMIT = 18.0
DEFAULT_PERIODIC_SNAPSHOT_INTERVAL_S = 5.0
DEFAULT_SERVO_HOME = [2300, 1500, 1500, 2200, 1500, 1500, 1500, 1500]
SERVO_MIN_PWM = 500
SERVO_MAX_PWM = 2500
SERVO_COUNT = 8
TRACKING_FIXED_JOINT3 = 2200
TRACKING_DEADZONE_X = 0.06
TRACKING_DEADZONE_Y = 0.06
TRACKING_GAIN_X = 260
TRACKING_GAIN_Y = 220
TRACKING_MAX_STEP_X = 55
TRACKING_MAX_STEP_Y = 45
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
VIDEO_EXTENSIONS = {".mp4", ".avi", ".mov", ".mkv", ".wmv", ".m4v"}
SUPPORTED_VISION_MEDIA_EXTENSIONS = set(IMAGE_EXTENSIONS)
MOTION_COMMANDS = {
    "forward": "$QJ!",
    "backward": "$HT!",
    "left": "$ZZ!",
    "right": "$YZ!",
    "stop": "$TZ!",
    "shift_left": "$ZPY!",
    "shift_right": "$YPY!",
}
LEGACY_STOP_COMMANDS = ("$TZ!", "$Car:0,0,0,0!", "$TZ!")
FORMAL_MODES = {"MANUAL", "MAPPING", "INSPECT", "ESTOP"}
FORMAL_ERROR_NAMES = {
    0: "NONE",
    1: "ESTOP_ACTIVE",
    2: "BAD_MODE",
    3: "KINEMATICS_FAIL",
}
CONTROL_ODOM_STALE_AFTER_S = 0.2
CONTROL_ODOM_RECOVERY_WAIT_S = 2.5
CONTROL_ODOM_PUBLISH_KEEPALIVE_S = 0.05
CONTROL_ODOM_TF_TIME_OFFSET_S = 0.10
LOCAL_ROS_LOCALHOST_ONLY = "0"
BASE_LINK_TO_LASER_X_METERS = 0.0
BASE_LINK_TO_LASER_Y_METERS = 0.0
BASE_LINK_TO_LASER_Z_METERS = 0.20
BASE_LINK_TO_IMU_X_METERS = 0.0
BASE_LINK_TO_IMU_Y_METERS = 0.0
BASE_LINK_TO_IMU_Z_METERS = 0.0
F103_MEC_WHEEL_BASE_METERS = 0.205
F103_MEC_AXLE_BASE_METERS = 0.225
DEFAULT_FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
]

UPLOAD_MEDIA_DIR.mkdir(exist_ok=True)
VISION_EXPORT_DIR.mkdir(exist_ok=True)


@dataclass
class Detection:
    x1: int
    y1: int
    x2: int
    y2: int
    label: str
    confidence: float
    source: str
    reading: str | None = None


class RuntimeLogger:
    def __init__(self, max_entries: int = 300):
        self._entries: collections.deque[str] = collections.deque(maxlen=max_entries)
        self._lock = threading.Lock()

    def log(self, message: str):
        stamp = time.strftime("%H:%M:%S")
        line = f"[{stamp}] {message}"
        with self._lock:
            self._entries.append(line)

    def tail(self, limit: int = 120) -> list[str]:
        with self._lock:
            return list(self._entries)[-limit:]


def discover_serial_ports() -> list[str]:
    ports: set[str] = set()
    if os.name == "nt":
        ports.update(port.device for port in list_ports.comports())
    patterns = (
        "/dev/car_ctrl_usb",
        "/dev/rplidar",
        "/dev/laser",
        "/dev/rt_shell",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
        "/dev/ttyCH341USB*",
        "/dev/ttyAMA*",
        "/dev/ttyS*",
    )
    for pattern in patterns:
        ports.update(glob.glob(pattern))

    if os.name != "nt":
        with contextlib.suppress(Exception):
            ports.update(port.device for port in list_ports.comports())

    preferred: list[str] = []
    others: list[str] = []
    for port in sorted(ports):
        if any(
            token in port
            for token in (
                "/dev/car_ctrl_usb",
                "/dev/rplidar",
                "/dev/laser",
                "/dev/rt_shell",
                "/dev/ttyUSB",
                "/dev/ttyACM",
                "/dev/ttyCH341USB",
            )
        ):
            preferred.append(port)
        else:
            others.append(port)
    return preferred + others


def _normalized_port_name(port: str) -> str:
    return str(port or "").strip()


def is_control_serial_port(port: str, active_control_port: str = "") -> bool:
    value = _normalized_port_name(port)
    active = _normalized_port_name(active_control_port)
    if not value:
        return False
    if value == active:
        return True
    return value == "/dev/car_ctrl_usb"


def discover_lidar_ports(active_control_port: str = "") -> list[str]:
    ports: list[str] = []
    for port in discover_serial_ports():
        value = _normalized_port_name(port)
        if not value or is_control_serial_port(value, active_control_port):
            continue
        if os.name != "nt" and not any(
            token in value for token in ("/dev/rplidar", "/dev/laser", "/dev/ttyUSB", "/dev/ttyACM", "/dev/ttyCH341USB")
        ):
            continue
        ports.append(value)

    deduped = list(dict.fromkeys(ports))

    def _priority(port: str) -> tuple[int, str]:
        if port == "/dev/rplidar":
            return (0, port)
        if port == "/dev/laser":
            return (1, port)
        if any(token in port for token in ("/dev/ttyUSB", "/dev/ttyACM", "/dev/ttyCH341USB")):
            return (2, port)
        return (3, port)

    return sorted(deduped, key=_priority)


class SerialMotionController:
    def __init__(self, logger: RuntimeLogger):
        self.logger = logger
        self.serial_conn: serial.Serial | None = None
        self.serial_lock = threading.Lock()
        self.port = ""
        self.baudrate = DEFAULT_SERIAL_BAUDRATE
        self.servo_values = DEFAULT_SERVO_HOME.copy()
        self.ble_manager = BleUartManager(rx_callback=self._handle_ble_rx)
        self.connection_mode: str | None = None
        self.ble_target = ""
        self.ble_write_uuid = DEFAULT_BLE_WRITE_UUID
        self.ble_notify_uuid = DEFAULT_BLE_NOTIFY_UUID
        self.ble_use_response = False
        self.cached_ble_devices: list[dict[str, Any]] = []
        self.serial_reader_thread: threading.Thread | None = None
        self.serial_reader_stop = threading.Event()
        self.serial_rx_buffer = ""
        self.formal_protocol_detected = False
        self.formal_report_enabled = False
        self.formal_last_rx_line = ""
        self.formal_last_rx_at = 0.0
        self.formal_last_state_at = 0.0
        self.formal_last_pong_at = 0.0
        self.formal_last_error_text = ""
        self.formal_mode = "UNKNOWN"
        self.formal_estop = False
        self.formal_sequence = 0
        self.formal_ai_mode = 255
        self.formal_mv_mode = 0
        self.formal_error_code = 0
        self.formal_state: dict[str, Any] = {}
        self.formal_wheel_speeds = {key: 0 for key in ("WA", "WB", "WC", "WD")}
        self.formal_target_speeds = {key: 0 for key in ("TA", "TB", "TC", "TD")}
        self.formal_odom_requested = False
        self.formal_odom_reporting = False
        self.formal_last_odom_at = 0.0
        self.formal_odom_sequence = 0
        self.formal_odom_dt_ms = 20
        self.formal_encoder_ticks = {key: 0 for key in ("EA", "EB", "EC", "ED")}
        self.formal_odom_wheel_speeds = {key: 0 for key in ("WA", "WB", "WC", "WD")}
        self.formal_odom_state: dict[str, Any] = {}
        self.formal_odom_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.formal_odom_twist = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.formal_odom_source = ""
        self.formal_odom_bridge_ready = ROS_ODOM_BRIDGE_IMPORT_ERROR is None
        # The deployed board firmware exposes formal telemetry but still uses legacy motion commands.
        self.control_profile = "legacy_motion_formal_telemetry"
        self.formal_input_supported = False
        self.legacy_last_reply = ""
        self.legacy_last_reply_at = 0.0
        self.last_tx_command = ""
        self.last_tx_at = 0.0
        self.last_motion_action = "stop"
        self.last_motion_command = ""
        self.last_motion_at = 0.0
        self.serial_exclusive_ok = False
        self.serial_exclusive_error = ""
        self.control_port_owner = "--"
        self.startup_probe_mode = "idle"
        self.serial_reader_last_error = ""
        self.odom_recovery_attempted = False
        self.odom_recovery_last_error = ""

    def _open_serial_connection(self, port: str, baudrate: int) -> serial.Serial:
        serial_kwargs = {
            "port": port,
            "baudrate": int(baudrate),
            "timeout": 0.05,
        }
        if os.name != "nt":
            serial_kwargs["exclusive"] = True
        try:
            conn = serial.Serial(**serial_kwargs)
        except TypeError:
            serial_kwargs.pop("exclusive", None)
            conn = serial.Serial(**serial_kwargs)
            self._claim_posix_serial_exclusive(conn)
            return conn
        self._claim_posix_serial_exclusive(conn)
        return conn

    def _claim_posix_serial_exclusive(self, conn: serial.Serial):
        if os.name == "nt" or fcntl is None or termios is None:
            return
        ioctl_flag = getattr(termios, "TIOCEXCL", None)
        if ioctl_flag is None:
            return
        try:
            fcntl.ioctl(conn.fileno(), ioctl_flag, 0)
        except TypeError:
            fcntl.ioctl(conn.fileno(), ioctl_flag)

    def _serial_open_conflict(self, exc: Exception) -> bool:
        errnos = {
            getattr(exc, "errno", None),
            getattr(getattr(exc, "__cause__", None), "errno", None),
            getattr(getattr(exc, "__context__", None), "errno", None),
        }
        if any(code in (errno.EBUSY, errno.EACCES, errno.EPERM, errno.EAGAIN) for code in errnos):
            return True
        lowered = str(exc).lower()
        return any(
            token in lowered
            for token in (
                "resource busy",
                "device or resource busy",
                "could not exclusively lock port",
                "permission denied",
                "access is denied",
            )
        )

    def _handle_ble_rx(self, text: str):
        self._ingest_rx_text(text, source="ble")

    def list_ports(self) -> list[str]:
        return discover_serial_ports()

    def list_ble_devices(self) -> list[dict[str, Any]]:
        return [dict(item) for item in self.cached_ble_devices]

    def ble_available(self) -> bool:
        return self.ble_manager.available

    def ble_import_error(self) -> str:
        return "" if self.ble_manager.import_error is None else str(self.ble_manager.import_error)

    def scan_ble_devices(self, timeout: float = DEFAULT_BLE_SCAN_TIMEOUT) -> list[dict[str, Any]]:
        if not self.ble_manager.available:
            raise RuntimeError(f"蓝牙不可用: {self.ble_import_error()}")
        devices = self.ble_manager.scan(timeout=float(timeout))
        self.cached_ble_devices = [
            {
                "label": item.label,
                "name": item.name or "",
                "address": item.address,
                "rssi": item.rssi,
            }
            for item in devices
        ]
        labels = [item["label"] for item in self.cached_ble_devices]
        self.logger.log(f"已扫描无线蓝牙设备: {labels if labels else '未发现设备'}")
        return self.list_ble_devices()

    def connect(self, port: str, baudrate: int = DEFAULT_SERIAL_BAUDRATE) -> tuple[bool, str]:
        if not port:
            return False, "串口为空"
        self.disconnect(silent=True)
        try:
            self.serial_conn = self._open_serial_connection(port, int(baudrate))
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            self.port = port
            self.baudrate = int(baudrate)
            self.connection_mode = "serial"
            self.ble_target = ""
            self._reset_formal_protocol_state()
            self._start_serial_reader()
            self.serial_exclusive_ok = True
            self.serial_exclusive_error = ""
            self.control_port_owner = "self"
            self.startup_probe_mode = "connect_only"
            self.serial_reader_last_error = ""
            self.reset_odom_recovery_state()
            self.logger.log(f"串口已连接: {port} @ {baudrate}")
            return True, f"已连接 {port}"
        except Exception as exc:
            self.serial_conn = None
            self.port = ""
            self.connection_mode = None
            self.serial_exclusive_ok = False
            self.control_port_owner = "--"
            self.startup_probe_mode = "idle"
            self.serial_exclusive_error = str(exc)
            self.serial_reader_last_error = ""
            self.reset_odom_recovery_state()
            if self._serial_open_conflict(exc):
                return False, f"控制串口忙/被占用: {port} ({exc})"
            return False, f"连接失败: {exc}"

    def connect_ble(
        self,
        *,
        address: str,
        label: str = "",
        write_uuid: str = DEFAULT_BLE_WRITE_UUID,
        notify_uuid: str | None = DEFAULT_BLE_NOTIFY_UUID,
        response: bool = False,
    ) -> tuple[bool, str]:
        if not self.ble_manager.available:
            return False, f"蓝牙不可用: {self.ble_import_error()}"
        if not address:
            return False, "蓝牙地址为空"

        self.disconnect(silent=True)
        try:
            ok, message = self.ble_manager.connect(
                address=address,
                write_uuid=write_uuid.strip() or DEFAULT_BLE_WRITE_UUID,
                notify_uuid=(notify_uuid or "").strip() or None,
            )
        except Exception as exc:
            ok, message = False, str(exc)

        if ok:
            self.connection_mode = "ble"
            self.ble_target = label or message or address
            self.ble_write_uuid = write_uuid.strip() or DEFAULT_BLE_WRITE_UUID
            self.ble_notify_uuid = (notify_uuid or "").strip()
            self.ble_use_response = bool(response)
            self.logger.log(f"蓝牙已连接: {self.ble_target}")
            return True, f"已连接 {self.ble_target}"

        self.connection_mode = None
        self.ble_target = ""
        return False, f"蓝牙连接失败: {message}"

    def disconnect(self, *, silent: bool = False):
        self._stop_serial_reader()
        if self.serial_conn is not None:
            with contextlib.suppress(Exception):
                if self.serial_conn.is_open:
                    self.serial_conn.close()
        self.serial_conn = None
        self.port = ""
        self.serial_exclusive_ok = False
        self.serial_exclusive_error = ""
        self.control_port_owner = "--"
        self.startup_probe_mode = "idle"
        self.serial_reader_last_error = ""
        self.reset_odom_recovery_state()
        self.disconnect_ble(silent=True)
        self.connection_mode = None
        self.ble_target = ""
        if not silent:
            self.logger.log("通信已断开")

    def disconnect_ble(self, *, silent: bool = False):
        with contextlib.suppress(Exception):
            self.ble_manager.disconnect()
        if self.connection_mode == "ble":
            self.connection_mode = None
        self.ble_target = ""
        if not silent:
            self.logger.log("蓝牙已断开")

    def is_connected(self) -> bool:
        return bool((self.serial_conn is not None and self.serial_conn.is_open) or self.ble_manager.is_connected())

    def is_ble_connected(self) -> bool:
        return self.ble_manager.is_connected()

    def current_target(self) -> str:
        if self.serial_conn is not None and self.serial_conn.is_open:
            return f"{self.port} @ {self.baudrate}"
        if self.ble_manager.is_connected():
            return self.ble_target or self.ble_manager.current_target() or "蓝牙已连接"
        return "未连接"

    def _reset_formal_protocol_state(self):
        self.serial_rx_buffer = ""
        self.formal_protocol_detected = False
        self.formal_report_enabled = False
        self.formal_last_rx_line = ""
        self.formal_last_rx_at = 0.0
        self.formal_last_state_at = 0.0
        self.formal_last_pong_at = 0.0
        self.formal_last_error_text = ""
        self.formal_mode = "UNKNOWN"
        self.formal_estop = False
        self.formal_sequence = 0
        self.formal_ai_mode = 255
        self.formal_mv_mode = 0
        self.formal_error_code = 0
        self.formal_state = {}
        self.formal_wheel_speeds = {key: 0 for key in ("WA", "WB", "WC", "WD")}
        self.formal_target_speeds = {key: 0 for key in ("TA", "TB", "TC", "TD")}
        self.formal_odom_requested = False
        self.formal_odom_reporting = False
        self.formal_last_odom_at = 0.0
        self.formal_odom_sequence = 0
        self.formal_odom_dt_ms = 20
        self.formal_encoder_ticks = {key: 0 for key in ("EA", "EB", "EC", "ED")}
        self.formal_odom_wheel_speeds = {key: 0 for key in ("WA", "WB", "WC", "WD")}
        self.formal_odom_state = {}
        self.formal_odom_pose = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.formal_odom_twist = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.formal_odom_source = ""
        self.formal_odom_bridge_ready = ROS_ODOM_BRIDGE_IMPORT_ERROR is None
        self.legacy_last_reply = ""
        self.legacy_last_reply_at = 0.0

    def _start_serial_reader(self):
        self._stop_serial_reader()
        self.serial_reader_stop = threading.Event()
        self.serial_reader_last_error = ""
        self.serial_reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self.serial_reader_thread.start()

    def _stop_serial_reader(self):
        if self.serial_reader_thread is None:
            return
        self.serial_reader_stop.set()
        self.serial_reader_thread.join(timeout=0.5)
        self.serial_reader_thread = None

    def _serial_reader_loop(self):
        while not self.serial_reader_stop.is_set():
            conn = self.serial_conn
            if conn is None:
                return
            try:
                waiting = getattr(conn, "in_waiting", 0)
                payload = conn.read(waiting or 1)
            except Exception as exc:
                self.serial_reader_last_error = str(exc)
                self.logger.log(f"串口接收异常: {exc}")
                return
            if payload:
                self._ingest_rx_text(payload.decode("utf-8", errors="ignore"), source="serial")

    def run_minimal_startup_probe(self) -> tuple[bool, str]:
        self.startup_probe_mode = "minimal_once"
        # The deployed F103 build requires the report stream to be enabled before odom starts.
        for command in ("$PING!", "$STATUS!", "$REPORT:ON!", "$ODOM:ON!"):
            ok, message = self.send_raw(command)
            if not ok:
                return False, message
            time.sleep(0.05)
        return True, "最小初始化序列已发送"

    def reset_odom_recovery_state(self):
        self.odom_recovery_attempted = False
        self.odom_recovery_last_error = ""

    def serial_reader_alive(self) -> bool:
        thread = self.serial_reader_thread
        return bool(thread is not None and thread.is_alive())

    def _serial_last_rx_age_ms(self) -> int | None:
        if self.formal_last_rx_at <= 0.0:
            return None
        return int(max(0.0, time.time() - self.formal_last_rx_at) * 1000.0)

    def odom_stale(self) -> bool:
        if not self.is_connected():
            return False
        if not self.formal_odom_requested:
            return False
        if self.formal_last_odom_at <= 0.0:
            return True
        return (time.time() - self.formal_last_odom_at) >= CONTROL_ODOM_STALE_AFTER_S

    def _wait_for_fresh_odom(self, *, after_ts: float, timeout_s: float) -> bool:
        deadline = time.time() + max(0.5, float(timeout_s))
        while time.time() < deadline:
            if self.formal_last_odom_at > after_ts and not self.odom_stale():
                return True
            time.sleep(0.05)
        return False

    def _reopen_serial_connection(self) -> tuple[bool, str]:
        if self.connection_mode != "serial":
            return False, "当前不是串口连接模式"
        port = self.port
        baudrate = self.baudrate
        if not port:
            return False, "控制串口为空"
        try:
            self._stop_serial_reader()
            if self.serial_conn is not None:
                with contextlib.suppress(Exception):
                    if self.serial_conn.is_open:
                        self.serial_conn.close()
            self.serial_conn = self._open_serial_connection(port, int(baudrate))
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            self.connection_mode = "serial"
            self.ble_target = ""
            self._reset_formal_protocol_state()
            self._start_serial_reader()
            self.serial_exclusive_ok = True
            self.serial_exclusive_error = ""
            self.control_port_owner = "self"
            self.startup_probe_mode = "recovery_reopen"
            self.logger.log(f"控制串口已重连: {port} @ {baudrate}")
            return True, f"控制串口已重连: {port}"
        except Exception as exc:
            self.serial_conn = None
            self.serial_exclusive_ok = False
            self.serial_exclusive_error = str(exc)
            self.control_port_owner = "--"
            self.serial_reader_last_error = str(exc)
            return False, f"控制串口重连失败: {exc}"

    def recover_stale_odom_once(self, timeout_s: float = CONTROL_ODOM_RECOVERY_WAIT_S) -> tuple[bool, str]:
        if self.odom_recovery_attempted:
            message = self.odom_recovery_last_error or "odom recovery already attempted"
            return False, message
        self.odom_recovery_attempted = True
        self.odom_recovery_last_error = ""

        if self.connection_mode != "serial" or self.serial_conn is None or not self.serial_conn.is_open:
            self.odom_recovery_last_error = "control serial is not connected"
            return False, self.odom_recovery_last_error

        recovery_started_at = time.time()
        reader_alive = self.serial_reader_alive()
        if not reader_alive:
            ok, message = self._reopen_serial_connection()
            if not ok:
                self.odom_recovery_last_error = message
                self.logger.log(f"odom recovery failed: {message}")
                return False, message

        self.startup_probe_mode = "recovery_minimal_once"
        ok, message = self.run_minimal_startup_probe()
        if not ok:
            self.odom_recovery_last_error = f"最小恢复探测失败: {message}"
            self.logger.log(f"odom recovery failed: {self.odom_recovery_last_error}")
            return False, self.odom_recovery_last_error

        if self._wait_for_fresh_odom(after_ts=recovery_started_at, timeout_s=timeout_s):
            success_message = "odom recovery succeeded"
            self.odom_recovery_last_error = ""
            self.logger.log(f"{success_message}: {message}")
            return True, success_message

        self.odom_recovery_last_error = "timed out waiting for fresh @ODOM after one recovery"
        self.logger.log(f"odom recovery failed: {self.odom_recovery_last_error}")
        return False, self.odom_recovery_last_error

    def _ingest_rx_text(self, text: str, *, source: str):
        chunk = (text or "").replace("\r", "\n")
        if not chunk:
            return
        self.serial_rx_buffer += chunk
        while True:
            newline_pos = self.serial_rx_buffer.find("\n")
            bang_pos = self.serial_rx_buffer.find("!")
            if newline_pos == -1 and bang_pos == -1:
                break
            if newline_pos != -1 and (bang_pos == -1 or newline_pos < bang_pos):
                line = self.serial_rx_buffer[:newline_pos]
                self.serial_rx_buffer = self.serial_rx_buffer[newline_pos + 1 :]
            else:
                line = self.serial_rx_buffer[: bang_pos + 1]
                self.serial_rx_buffer = self.serial_rx_buffer[bang_pos + 1 :]
            self._consume_rx_line(line.strip(), source=source)

        if len(self.serial_rx_buffer) > 4096:
            self.serial_rx_buffer = self.serial_rx_buffer[-1024:]

    def _consume_rx_line(self, line: str, *, source: str):
        line = str(line or "").strip()
        if not line:
            return
        marker_pos = line.find("@")
        if marker_pos > 0:
            prefix = line[:marker_pos].strip()
            suffix = line[marker_pos:].strip()
            if prefix:
                self._consume_rx_line(prefix, source=source)
            if suffix:
                self._consume_rx_line(suffix, source=source)
            return
        now = time.time()
        self.formal_last_rx_line = line
        self.formal_last_rx_at = now
        handled = self._parse_formal_line(line, now=now)
        if handled == "state":
            if self.formal_sequence <= 3 or self.formal_sequence % 20 == 0:
                self.logger.log(
                    f"状态上报: seq={self.formal_sequence} mode={self.formal_mode} estop={int(self.formal_estop)}"
                )
            return
        if handled == "odom":
            if self.formal_odom_sequence <= 3 or self.formal_odom_sequence % 100 == 0:
                self.logger.log(
                    "里程计上报: "
                    f"seq={self.formal_odom_sequence} "
                    f"ticks={dict(self.formal_encoder_ticks)} "
                    f"wheel_mm_s={dict(self.formal_odom_wheel_speeds)}"
                )
            return
        if self._remember_legacy_reply(line, now=now):
            prefix = "串口接收" if source == "serial" else "蓝牙接收"
            preview = line if len(line) <= 180 else f"{line[:177]}..."
            self.logger.log(f"{prefix}: {preview}")
            return
        prefix = "串口接收" if source == "serial" else "蓝牙接收"
        preview = line if len(line) <= 180 else f"{line[:177]}..."
        self.logger.log(f"{prefix}: {preview}")

    def _remember_legacy_reply(self, line: str, *, now: float) -> bool:
        text = str(line or "").strip()
        if not text:
            return False
        lowered = text.lower()
        if text == "AAA" or lowered == "hello word!" or lowered == "cmdok":
            self.legacy_last_reply = text
            self.legacy_last_reply_at = now
            return True
        return False

    @staticmethod
    def _safe_int(value: Any, default: int = 0) -> int:
        try:
            return int(str(value).strip())
        except Exception:
            return default

    @staticmethod
    def _parse_csv_equals(payload: str) -> dict[str, str]:
        result: dict[str, str] = {}
        for item in payload.split(","):
            if "=" not in item:
                continue
            key, value = item.split("=", 1)
            result[key.strip()] = value.strip()
        return result

    @staticmethod
    def _parse_csv_colon(payload: str) -> dict[str, str]:
        result: dict[str, str] = {}
        for item in payload.split(","):
            if ":" not in item:
                continue
            key, value = item.split(":", 1)
            result[key.strip()] = value.strip()
        return result

    def _parse_formal_line(self, line: str, *, now: float) -> str:
        if line == "@PONG!":
            self.formal_protocol_detected = True
            self.formal_last_pong_at = now
            return "pong"

        if line.startswith("@REPORT:OFF"):
            self.formal_protocol_detected = True
            self.formal_report_enabled = False
            return "report"

        if line.startswith("@ODOM:ON"):
            self.formal_protocol_detected = True
            self.formal_odom_requested = True
            self.formal_odom_reporting = True
            return "odom_ack"

        if line.startswith("@ODOM:OFF"):
            self.formal_protocol_detected = True
            self.formal_odom_requested = False
            self.formal_odom_reporting = False
            return "odom_ack"

        if line.startswith("@STATE:"):
            payload = line[len("@STATE:") :].rstrip("!")
            state = self._parse_csv_equals(payload)
            if not state:
                return "unknown"
            self.formal_protocol_detected = True
            self.formal_report_enabled = True
            self.formal_last_state_at = now
            self.formal_state = dict(state)
            self.formal_sequence = self._safe_int(state.get("SEQ"), self.formal_sequence)
            self.formal_mode = state.get("MODE", self.formal_mode).upper()
            self.formal_estop = bool(self._safe_int(state.get("ESTOP"), int(self.formal_estop)))
            self.formal_ai_mode = self._safe_int(state.get("AI"), self.formal_ai_mode)
            self.formal_mv_mode = self._safe_int(state.get("MV"), self.formal_mv_mode)
            self.formal_error_code = self._safe_int(state.get("ERR"), self.formal_error_code)
            for key in self.formal_wheel_speeds:
                self.formal_wheel_speeds[key] = self._safe_int(state.get(key), self.formal_wheel_speeds[key])
            for key in self.formal_target_speeds:
                self.formal_target_speeds[key] = self._safe_int(state.get(key), self.formal_target_speeds[key])
            for index in range(SERVO_COUNT):
                servo_key = f"S{index}"
                if servo_key in state:
                    self.servo_values[index] = self._safe_int(state.get(servo_key), self.servo_values[index])
            return "state"

        if line.startswith("@ODOM:"):
            payload = line[len("@ODOM:") :].rstrip("!")
            state = self._parse_csv_equals(payload)
            if not state:
                return "unknown"
            self.formal_protocol_detected = True
            self.formal_odom_requested = True
            self.formal_odom_reporting = True
            self.formal_last_odom_at = now
            self.formal_odom_state = dict(state)
            self.formal_odom_sequence = self._safe_int(state.get("SEQ"), self.formal_odom_sequence)
            self.formal_odom_dt_ms = max(1, self._safe_int(state.get("DT_MS"), self.formal_odom_dt_ms))
            for key in self.formal_encoder_ticks:
                self.formal_encoder_ticks[key] = self._safe_int(state.get(key), self.formal_encoder_ticks[key])
            for key in self.formal_odom_wheel_speeds:
                self.formal_odom_wheel_speeds[key] = self._safe_int(state.get(key), self.formal_odom_wheel_speeds[key])
            return "odom"

        if line.startswith("@MODE:"):
            payload = line[len("@MODE:") :].rstrip("!")
            mode_text, _, rest = payload.partition(",")
            mode_fields = self._parse_csv_colon(rest)
            self.formal_protocol_detected = True
            self.formal_mode = mode_text.strip().upper() or self.formal_mode
            if "ESTOP" in mode_fields:
                self.formal_estop = bool(self._safe_int(mode_fields["ESTOP"], int(self.formal_estop)))
            if self.formal_mode == "ESTOP":
                self.formal_estop = True
            return "mode"

        if line.startswith("@ERR:"):
            payload = line[len("@ERR:") :].rstrip("!")
            error_text, _, rest = payload.partition(",")
            fields = self._parse_csv_colon(rest)
            self.formal_protocol_detected = True
            self.formal_last_error_text = error_text.strip().upper() or self.formal_last_error_text
            if "MODE" in fields:
                self.formal_mode = fields["MODE"].strip().upper() or self.formal_mode
            if self.formal_last_error_text == "ESTOP" or self.formal_mode == "ESTOP":
                self.formal_estop = True
            return "error"

        return "unknown"

    def formal_error_name(self) -> str:
        if self.formal_last_error_text:
            return self.formal_last_error_text
        return FORMAL_ERROR_NAMES.get(self.formal_error_code, f"ERR_{self.formal_error_code}")

    def formal_status_text(self) -> str:
        if not self.is_connected():
            return "未连接"
        if not self.formal_protocol_detected:
            return "已连接，等待正式协议响应"
        if not self.formal_input_supported:
            if self.legacy_command_ready():
                return f"legacy 控制在线，模式 {self.formal_mode}"
            return "状态流在线，legacy 下行未确认"
        if self.odom_only_link_detected():
            return "仅里程计单向在线，控制下行未确认"
        if self.formal_estop:
            return "急停中"
        if self.formal_last_state_at <= 0:
            return f"协议在线，模式 {self.formal_mode}"
        age_ms = int(max(0.0, time.time() - self.formal_last_state_at) * 1000)
        if age_ms > 3000:
            return f"状态流超时 {age_ms} ms"
        return f"模式 {self.formal_mode}，状态流正常"

    def legacy_command_ready(self) -> bool:
        if not self.is_connected():
            return False
        if self.legacy_last_reply_at > 0.0:
            return True
        # The deployed F103 profile keeps legacy motion commands but reports readiness through
        # formal telemetry instead of the older AAA/cmdok text replies.
        if self.control_profile == "legacy_motion_formal_telemetry":
            now = time.time()
            for timestamp in (self.formal_last_state_at, self.formal_last_odom_at, self.formal_last_pong_at):
                if timestamp > 0.0 and now - timestamp < 3.0:
                    return True
        return False

    def control_ack_ready(self) -> bool:
        if not self.is_connected():
            return False
        if not self.formal_input_supported:
            return self.legacy_command_ready()
        if self.formal_sequence > 0:
            return True
        now = time.time()
        for timestamp in (self.formal_last_state_at, self.formal_last_pong_at):
            if timestamp > 0 and now - timestamp < 3.0:
                return True
        return False

    def odom_only_link_detected(self) -> bool:
        if not self.is_connected():
            return False
        if not self.formal_protocol_detected:
            return False
        if self.control_ack_ready():
            return False
        if not self.formal_odom_reporting:
            return False
        return self.formal_odom_sequence > 0

    def send_raw(self, command: str) -> tuple[bool, str]:
        command = (command or "").strip()
        if not command:
            return False, "命令为空"
        if self.serial_conn is not None and self.serial_conn.is_open:
            try:
                with self.serial_lock:
                    assert self.serial_conn is not None
                    self.serial_conn.write(command.encode("ascii"))
                    self.serial_conn.flush()
                if command == "$REPORT:ON!":
                    self.formal_report_enabled = True
                elif command == "$REPORT:OFF!":
                    self.formal_report_enabled = False
                elif command == "$ODOM:ON!":
                    self.formal_odom_requested = True
                elif command == "$ODOM:OFF!":
                    self.formal_odom_requested = False
                    self.formal_odom_reporting = False
                self.last_tx_command = command
                self.last_tx_at = time.time()
                self.logger.log(f"串口发送命令: {command}")
                return True, "ok"
            except Exception as exc:
                return False, f"串口发送失败: {exc}"
        if self.ble_manager.is_connected():
            try:
                ok, message = self.ble_manager.send_text(command, response=self.ble_use_response)
            except Exception as exc:
                ok, message = False, str(exc)
            if ok:
                if command == "$REPORT:ON!":
                    self.formal_report_enabled = True
                elif command == "$REPORT:OFF!":
                    self.formal_report_enabled = False
                elif command == "$ODOM:ON!":
                    self.formal_odom_requested = True
                elif command == "$ODOM:OFF!":
                    self.formal_odom_requested = False
                    self.formal_odom_reporting = False
                self.last_tx_command = command
                self.last_tx_at = time.time()
                self.logger.log(f"蓝牙发送命令: {command}")
                return True, "ok"
            return False, f"蓝牙发送失败: {message}"
        return False, "通信未连接"

    def handshake(self) -> tuple[bool, str]:
        self.startup_probe_mode = "manual_handshake"
        commands = ("$PING!", "$STATUS!", "$REPORT:ON!", "$ODOM:ON!", "$DRS!")
        for command in commands:
            ok, message = self.send_raw(command)
            if not ok:
                return False, message
            time.sleep(0.05)
        return True, "正式协议握手命令已发送"

    def request_status(self) -> tuple[bool, str]:
        return self.send_raw("$STATUS!")

    def set_report_stream(self, enabled: bool) -> tuple[bool, str]:
        if not self.formal_input_supported:
            return False, "当前板端固件不支持 formal 上报开关命令"
        ok, message = self.send_raw("$REPORT:ON!" if enabled else "$REPORT:OFF!")
        if ok:
            self.formal_report_enabled = bool(enabled)
        return ok, message

    def set_odom_stream(self, enabled: bool) -> tuple[bool, str]:
        ok, message = self.send_raw("$ODOM:ON!" if enabled else "$ODOM:OFF!")
        if ok:
            self.formal_odom_requested = bool(enabled)
            if not enabled:
                self.formal_odom_reporting = False
        return ok, message

    def set_mode(self, mode: str) -> tuple[bool, str]:
        normalized = str(mode or "").strip().upper()
        if normalized not in FORMAL_MODES:
            return False, f"不支持的模式: {mode}"
        if not self.formal_input_supported:
            return False, "当前板端固件不支持 formal 模式切换命令"
        return self.send_raw(f"$MODE:{normalized}!")

    def emergency_stop(self) -> tuple[bool, str]:
        return self._send_stop_sequence(include_formal_estop=self.formal_input_supported)

    def clear_emergency_stop(self) -> tuple[bool, str]:
        if not self.formal_input_supported:
            return self._send_stop_sequence(include_formal_estop=False, clear_only=True)
        ok, message = self.send_raw("$ESTOP:CLR!")
        if ok:
            self.formal_estop = False
        return ok, message

    def send_motion(self, action: str) -> tuple[bool, str]:
        if action == "stop":
            ok, message = self._send_stop_sequence(include_formal_estop=False)
            if ok:
                self.last_motion_action = "stop"
                self.last_motion_command = LEGACY_STOP_COMMANDS[-1]
                self.last_motion_at = time.time()
            return ok, message
        command = MOTION_COMMANDS.get(action, "")
        if not command:
            return False, f"未知动作: {action}"
        ok, message = self.send_raw(command)
        if ok:
            self.last_motion_action = action
            self.last_motion_command = command
            self.last_motion_at = time.time()
        return ok, message

    def _send_stop_sequence(self, *, include_formal_estop: bool, clear_only: bool = False) -> tuple[bool, str]:
        commands: list[str] = []
        if include_formal_estop:
            commands.append("$ESTOP:CLR!" if clear_only else "$ESTOP!")
        commands.extend(LEGACY_STOP_COMMANDS)
        for _ in range(2):
            for command in commands:
                ok, message = self.send_raw(command)
                if not ok:
                    return False, message
                time.sleep(0.03)
        if include_formal_estop:
            action = "急停清除" if clear_only else "急停/停车"
            return True, f"{action}序列已发送"
        return True, "legacy 停车序列已发送"

    def send_servo_targets(self, targets: dict[int, int], duration: int = 120) -> tuple[bool, str]:
        parts: list[str] = []
        normalized: dict[int, int] = {}
        for index, value in sorted(targets.items()):
            if index < 0 or index >= SERVO_COUNT:
                continue
            clamped = max(SERVO_MIN_PWM, min(SERVO_MAX_PWM, int(value)))
            parts.append(f"#{index:03d}P{clamped:04d}T{max(20, int(duration)):04d}!")
            normalized[index] = clamped
        if not parts:
            return False, "没有有效关节命令"
        ok, message = self.send_raw("".join(parts))
        if ok:
            for index, value in normalized.items():
                self.servo_values[index] = value
        return ok, message

    def home(self) -> tuple[bool, str]:
        targets = {index: value for index, value in enumerate(DEFAULT_SERVO_HOME)}
        return self.send_servo_targets(targets, duration=600)

    def reset_arm(self) -> tuple[bool, str]:
        ok, message = self.send_raw("$DJR!")
        if ok:
            self.servo_values = DEFAULT_SERVO_HOME.copy()
        return ok, message

    def stop_all_servos(self) -> tuple[bool, str]:
        return self.send_raw("$DST!")

    def odom_sample(self) -> dict[str, Any]:
        return {
            "connected": self.is_connected(),
            "requested": self.formal_odom_requested,
            "reporting": self.formal_odom_reporting,
            "sequence": self.formal_odom_sequence,
            "dt_ms": self.formal_odom_dt_ms,
            "last_at": self.formal_last_odom_at,
            "encoder_ticks": dict(self.formal_encoder_ticks),
            "wheel_speeds_mm_s": dict(self.formal_odom_wheel_speeds),
        }

    def update_odom_bridge_feedback(
        self,
        *,
        pose_x: float,
        pose_y: float,
        theta: float,
        vx: float,
        vy: float,
        wz: float,
        source: str,
        bridge_ready: bool,
    ):
        self.formal_odom_pose = {
            "x": float(pose_x),
            "y": float(pose_y),
            "theta": float(theta),
        }
        self.formal_odom_twist = {
            "vx": float(vx),
            "vy": float(vy),
            "wz": float(wz),
        }
        self.formal_odom_source = str(source or "")
        self.formal_odom_bridge_ready = bool(bridge_ready)

    def status(self) -> dict[str, Any]:
        state_age_ms = None
        if self.formal_last_state_at > 0:
            state_age_ms = int(max(0.0, time.time() - self.formal_last_state_at) * 1000)
        odom_age_ms = None
        if self.formal_last_odom_at > 0:
            odom_age_ms = int(max(0.0, time.time() - self.formal_last_odom_at) * 1000)
        odom_reporting = bool(
            self.formal_odom_reporting
            and odom_age_ms is not None
            and odom_age_ms < int(CONTROL_ODOM_STALE_AFTER_S * 1000.0)
        )
        tx_age_ms = None
        if self.last_tx_at > 0:
            tx_age_ms = int(max(0.0, time.time() - self.last_tx_at) * 1000)
        motion_age_ms = None
        if self.last_motion_at > 0:
            motion_age_ms = int(max(0.0, time.time() - self.last_motion_at) * 1000)
        return {
            "connected": self.is_connected(),
            "target": self.current_target(),
            "mode": self.connection_mode or "disconnected",
            "exclusive_open": self.serial_exclusive_ok,
            "exclusive_error": self.serial_exclusive_error,
            "port_owner": self.control_port_owner if self.connection_mode == "serial" and self.is_connected() else "--",
            "startup_probe_mode": self.startup_probe_mode,
            "serial_reader_alive": self.serial_reader_alive(),
            "serial_reader_last_error": self.serial_reader_last_error,
            "serial_last_rx_age_ms": self._serial_last_rx_age_ms(),
            "last_tx_command": self.last_tx_command,
            "last_tx_age_ms": tx_age_ms,
            "last_motion_action": self.last_motion_action,
            "last_motion_command": self.last_motion_command,
            "last_motion_age_ms": motion_age_ms,
            "odom_stale": self.odom_stale(),
            "odom_recovery_attempted": self.odom_recovery_attempted,
            "odom_recovery_last_error": self.odom_recovery_last_error,
            "servo_values": list(self.servo_values),
            "ports": self.list_ports(),
            "serial_port": self.port,
            "serial_baudrate": self.baudrate,
            "ble_supported": self.ble_available(),
            "ble_connected": self.is_ble_connected(),
            "ble_target": self.ble_target or self.ble_manager.current_target(),
            "ble_devices": self.list_ble_devices(),
            "ble_import_error": self.ble_import_error(),
            "ble_settings": {
                "write_uuid": self.ble_write_uuid,
                "notify_uuid": self.ble_notify_uuid,
                "response": self.ble_use_response,
            },
            "formal": {
                "detected": self.formal_protocol_detected,
                "reporting": self.formal_report_enabled,
                "mode": self.formal_mode,
                "estop": self.formal_estop,
                "sequence": self.formal_sequence,
                "ai_mode": self.formal_ai_mode,
                "mv_mode": self.formal_mv_mode,
                "error_code": self.formal_error_code,
                "error_name": self.formal_error_name(),
                "last_error_text": self.formal_last_error_text,
                "last_rx_line": self.formal_last_rx_line,
                "last_rx_at": self.formal_last_rx_at,
                "last_state_at": self.formal_last_state_at,
                "last_pong_at": self.formal_last_pong_at,
                "state_age_ms": state_age_ms,
                "wheel_speeds_mm_s": dict(self.formal_wheel_speeds),
                "target_speeds_mm_s": dict(self.formal_target_speeds),
                "raw_state": dict(self.formal_state),
                "control_profile": self.control_profile,
                "formal_input_supported": self.formal_input_supported,
                "legacy_command_ready": self.legacy_command_ready(),
                "legacy_last_reply": self.legacy_last_reply,
                "legacy_last_reply_at": self.legacy_last_reply_at,
                "status_text": self.formal_status_text(),
                "control_ack_ready": self.control_ack_ready(),
                "odom_only_link": self.odom_only_link_detected(),
                "control_warning": (
                    "板端当前为 legacy 控制 + formal 遥测组合，$MODE/$ESTOP/$REPORT 输入命令不生效"
                    if not self.formal_input_supported
                    else "当前只收到 @ODOM，未收到 @STATE/@MODE/@PONG，控制下行可能不通"
                )
                + (
                    "；legacy 下行尚未用探针确认"
                    if not self.control_ack_ready()
                    else ""
                ),
                "odom_reporting": odom_reporting,
                "odom_requested": self.formal_odom_requested,
                "odom_last_at": self.formal_last_odom_at,
                "odom_age_ms": odom_age_ms,
                "odom_sequence": self.formal_odom_sequence,
                "odom_dt_ms": self.formal_odom_dt_ms,
                "encoder_ticks": dict(self.formal_encoder_ticks),
                "odom_wheel_speeds_mm_s": dict(self.formal_odom_wheel_speeds),
                "odom_pose": dict(self.formal_odom_pose),
                "odom_twist": dict(self.formal_odom_twist),
                "odom_source": self.formal_odom_source or "f103_serial",
                "odom_bridge_ready": self.formal_odom_bridge_ready,
                "odom_raw_state": dict(self.formal_odom_state),
            },
        }


class SerialOdomBridge:
    def __init__(self, logger: RuntimeLogger, controller: SerialMotionController):
        self.logger = logger
        self.controller = controller
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._theta = 0.0
        self._last_sequence = 0
        self._last_publish_at = 0.0
        self._bridge_ready = False
        self._tf_warning_logged = False
        self._start()

    def _start(self):
        if ROS_ODOM_BRIDGE_IMPORT_ERROR is not None:
            self.logger.log(f"串口 odom bridge 不可用: {ROS_ODOM_BRIDGE_IMPORT_ERROR}")
            self.controller.update_odom_bridge_feedback(
                pose_x=0.0,
                pose_y=0.0,
                theta=0.0,
                vx=0.0,
                vy=0.0,
                wz=0.0,
                source="",
                bridge_ready=False,
            )
            return
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _reset_pose_if_needed(self, sequence: int):
        if sequence > 0 and self._last_sequence > 0 and sequence < self._last_sequence:
            self._pose_x = 0.0
            self._pose_y = 0.0
            self._theta = 0.0

    @staticmethod
    def _compute_twist(wheel_speeds_mm_s: dict[str, int]) -> tuple[float, float, float]:
        v_a = float(wheel_speeds_mm_s.get("WA", 0)) / 1000.0
        v_b = float(wheel_speeds_mm_s.get("WB", 0)) / 1000.0
        v_c = float(wheel_speeds_mm_s.get("WC", 0)) / 1000.0
        v_d = float(wheel_speeds_mm_s.get("WD", 0)) / 1000.0
        base = F103_MEC_WHEEL_BASE_METERS + F103_MEC_AXLE_BASE_METERS
        vx = (v_a + v_b + v_c + v_d) / 4.0
        vy = (v_a - v_b - v_c + v_d) / 4.0
        wz = 0.0 if base <= 1e-6 else (v_a - v_b + v_c - v_d) / (2.0 * base)
        return vx, vy, wz

    @staticmethod
    def _yaw_to_quaternion(theta: float) -> tuple[float, float]:
        half = theta * 0.5
        return math.sin(half), math.cos(half)

    @staticmethod
    def _offset_stamp(stamp, offset_s: float):
        if stamp is None or abs(float(offset_s or 0.0)) < 1e-6:
            return stamp
        try:
            total_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            total_ns += int(float(offset_s) * 1_000_000_000)
            if total_ns < 0:
                total_ns = 0
            shifted = stamp.__class__()
            shifted.sec = total_ns // 1_000_000_000
            shifted.nanosec = total_ns % 1_000_000_000
            return shifted
        except Exception:
            return stamp

    @staticmethod
    def _build_transform_message(
        stamp,
        *,
        parent_frame: str,
        child_frame: str,
        x: float,
        y: float,
        z: float,
        qz: float,
        qw: float,
    ):
        if TransformStamped is None:
            return None
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = parent_frame
        message.child_frame_id = child_frame
        message.transform.translation.x = float(x)
        message.transform.translation.y = float(y)
        message.transform.translation.z = float(z)
        message.transform.rotation.x = 0.0
        message.transform.rotation.y = 0.0
        message.transform.rotation.z = float(qz)
        message.transform.rotation.w = float(qw)
        return message

    def _publish_odom(self, publisher, stamp, *, vx: float, vy: float, wz: float):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = float(self._pose_x)
        message.pose.pose.position.y = float(self._pose_y)
        qz, qw = self._yaw_to_quaternion(self._theta)
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = float(vx)
        message.twist.twist.linear.y = float(vy)
        message.twist.twist.angular.z = float(wz)
        message.pose.covariance[0] = 0.03
        message.pose.covariance[7] = 0.03
        message.pose.covariance[35] = 0.08
        message.twist.covariance[0] = 0.02
        message.twist.covariance[7] = 0.02
        message.twist.covariance[35] = 0.05
        publisher.publish(message)

    def _worker(self):
        os.environ["ROS_LOCALHOST_ONLY"] = LOCAL_ROS_LOCALHOST_ONLY
        os.environ.pop("LD_PRELOAD", None)
        context = RclpyContext()
        try:
            rclpy.init(args=None, context=context)
            node = Node("f103_serial_odom_bridge", context=context)
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            publisher = node.create_publisher(Odometry, "/odom_raw", qos)
            tf_broadcaster = None
            static_tf_broadcaster = None
            if StaticTransformBroadcaster is not None and TransformStamped is not None:
                static_tf_broadcaster = StaticTransformBroadcaster(node)
                laser_tf = self._build_transform_message(
                    node.get_clock().now().to_msg(),
                    parent_frame="base_link",
                    child_frame="laser",
                    x=BASE_LINK_TO_LASER_X_METERS,
                    y=BASE_LINK_TO_LASER_Y_METERS,
                    z=BASE_LINK_TO_LASER_Z_METERS,
                    qz=0.0,
                    qw=1.0,
                )
                imu_tf = self._build_transform_message(
                    node.get_clock().now().to_msg(),
                    parent_frame="base_link",
                    child_frame=DEFAULT_IMU_FRAME,
                    x=BASE_LINK_TO_IMU_X_METERS,
                    y=BASE_LINK_TO_IMU_Y_METERS,
                    z=BASE_LINK_TO_IMU_Z_METERS,
                    qz=0.0,
                    qw=1.0,
                )
                static_messages = [message for message in (laser_tf, imu_tf) if message is not None]
                if static_messages:
                    # /tf_static is transient-local with a shallow history; publish all static
                    # transforms in one sample so late-joining nodes receive the full set.
                    static_tf_broadcaster.sendTransform(static_messages)
            elif ROS_ODOM_TF_IMPORT_ERROR is not None and not self._tf_warning_logged:
                self.logger.log(f"串口静态 laser/imu TF 广播不可用: {ROS_ODOM_TF_IMPORT_ERROR}")
                self._tf_warning_logged = True
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self._bridge_ready = True
            self.logger.log("串口 raw odom bridge 已启动: 发布 /odom_raw 和静态 laser/imu TF")
            stale_logged = False
            while not self._stop_event.is_set():
                executor.spin_once(timeout_sec=0.01)
                sample = self.controller.odom_sample()
                now = time.time()
                odom_fresh = bool(
                    sample.get("connected")
                    and sample.get("requested")
                    and sample.get("reporting")
                    and sample.get("last_at")
                    and (now - float(sample.get("last_at") or 0.0) <= CONTROL_ODOM_STALE_AFTER_S)
                )
                if not odom_fresh:
                    self.controller.update_odom_bridge_feedback(
                        pose_x=self._pose_x,
                        pose_y=self._pose_y,
                        theta=self._theta,
                        vx=0.0,
                        vy=0.0,
                        wz=0.0,
                        source="f103_serial_raw",
                        bridge_ready=True,
                    )
                    if not stale_logged and sample.get("requested"):
                        self.logger.log("串口 odom bridge 等待新的 @ODOM 数据")
                        stale_logged = True
                    time.sleep(0.01)
                    continue

                sequence = int(sample.get("sequence") or 0)
                sequence_changed = sequence != self._last_sequence
                if (
                    not sequence_changed
                    and self._last_publish_at > 0.0
                    and now - self._last_publish_at < CONTROL_ODOM_PUBLISH_KEEPALIVE_S
                ):
                    time.sleep(0.002)
                    continue

                stale_logged = False
                vx, vy, wz = self._compute_twist(sample.get("wheel_speeds_mm_s") or {})
                if sequence_changed:
                    self._reset_pose_if_needed(sequence)
                    dt = max(0.001, float(sample.get("dt_ms") or 20) / 1000.0)
                    self._pose_x += (vx * math.cos(self._theta) - vy * math.sin(self._theta)) * dt
                    self._pose_y += (vx * math.sin(self._theta) + vy * math.cos(self._theta)) * dt
                    self._theta += wz * dt
                    self._last_sequence = sequence
                stamp = node.get_clock().now().to_msg()
                self._publish_odom(publisher, stamp, vx=vx, vy=vy, wz=wz)
                self._last_publish_at = now
                self.controller.update_odom_bridge_feedback(
                    pose_x=self._pose_x,
                    pose_y=self._pose_y,
                    theta=self._theta,
                    vx=vx,
                    vy=vy,
                    wz=wz,
                    source="f103_serial_raw",
                    bridge_ready=True,
                )
        except Exception as exc:  # pragma: no cover - board runtime only
            self._bridge_ready = False
            self.controller.update_odom_bridge_feedback(
                pose_x=self._pose_x,
                pose_y=self._pose_y,
                theta=self._theta,
                vx=0.0,
                vy=0.0,
                wz=0.0,
                source="",
                bridge_ready=False,
            )
            self.logger.log(f"串口 odom bridge 异常: {exc}")
        finally:
            with contextlib.suppress(Exception):
                if "executor" in locals():
                    executor.shutdown()
            with contextlib.suppress(Exception):
                if "node" in locals():
                    node.destroy_node()
            with contextlib.suppress(Exception):
                if context.ok():
                    context.shutdown()

    def close(self):
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._thread = None


class VisionSystem:
    def __init__(self, logger: RuntimeLogger, controller: SerialMotionController):
        self.logger = logger
        self.controller = controller
        self.capture: cv2.VideoCapture | None = None
        self.capture_thread: threading.Thread | None = None
        self.running = False
        self.frame_lock = threading.Lock()
        self.inference_lock = threading.Lock()
        self.latest_frame = self._blank_frame("等待启动摄像头")
        self.last_frame_at = 0.0
        self.camera_index = DEFAULT_CAMERA_INDEX
        self.frame_size = "0x0"
        self.current_fps = 0.0
        self._fps_tick = time.time()
        self._fps_counter = 0

        self.yolo_model = None
        self.yolo_backend = ""
        self.yolo_names: dict[int, str] = {}
        self.meter_pipeline = None
        self.yolo_enabled = True
        self.meter_enabled = True
        self.tracking_enabled = False
        self.yolo_confidence = 0.45
        self.meter_confidence = 0.55
        self.yolo_interval = 0.15
        self.meter_interval = 0.8
        self.tracking_interval = 0.18
        self.last_yolo_infer_at = 0.0
        self.last_meter_infer_at = 0.0
        self.last_tracking_at = 0.0
        self.last_yolo_detections: list[Detection] = []
        self.last_meter_detections: list[Detection] = []
        self.tracking_target: Detection | None = None
        self.tracking_error = (0.0, 0.0)
        self.snapshot_interval_s = DEFAULT_PERIODIC_SNAPSHOT_INTERVAL_S
        self.periodic_capture_enabled = True
        self.snapshot_busy = False
        self.last_snapshot_started_at = 0.0
        self.last_snapshot_completed_at = 0.0
        self.last_snapshot_error = ""
        self.latest_result: dict[str, Any] | None = None
        self.model_error = ""
        self.model_warnings: list[str] = []
        self.yolo_model_error = ""
        self.meter_model_error = ""
        self.device_name = "cpu"

    @staticmethod
    def _blank_frame(text: str) -> np.ndarray:
        image = np.full((720, 960, 3), 245, dtype=np.uint8)
        cv2.putText(image, text, (60, 360), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (70, 70, 70), 2, cv2.LINE_AA)
        return image

    def list_cameras(self, max_index: int = 6) -> list[int]:
        available: list[int] = []
        for index in range(max_index):
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                ok, _ = cap.read()
                if ok:
                    available.append(index)
            cap.release()
        return available

    def _refresh_model_state(self):
        warnings: list[str] = []
        if self.yolo_enabled and self.yolo_model is None and self.yolo_model_error:
            warnings.append(f"YOLO 不可用: {self.yolo_model_error}")
        if self.meter_enabled and self.meter_pipeline is None and self.meter_model_error:
            warnings.append(f"仪表模型不可用: {self.meter_model_error}")
        self.model_warnings = warnings
        self.model_error = " | ".join(warnings)

    def _load_models(self):
        if not self.yolo_enabled and not self.meter_enabled:
            self.model_warnings = []
            self.model_error = ""
            raise RuntimeError("请先开启 YOLO 检测或仪表识别")

        if self.yolo_enabled and self.yolo_model is None:
            try:
                if YOLOV8_ONNX_PATH.exists():
                    try:
                        self.yolo_model = cv2.dnn.readNetFromONNX(str(YOLOV8_ONNX_PATH))
                        self.yolo_backend = "opencv-dnn"
                        self.device_name = "cpu"
                    except Exception as cv_exc:
                        import onnxruntime as ort

                        self.yolo_model = ort.InferenceSession(
                            str(YOLOV8_ONNX_PATH),
                            providers=["CPUExecutionProvider"],
                        )
                        self.yolo_backend = "onnxruntime"
                        self.device_name = "cpu"
                        self.logger.log(f"OpenCV DNN 加载 ONNX 失败，已切换 onnxruntime: {cv_exc}")
                    if YOLOV8_LABELS_PATH.exists():
                        raw = json.loads(YOLOV8_LABELS_PATH.read_text(encoding="utf-8"))
                        self.yolo_names = {int(key): str(value) for key, value in raw.items()}
                    self.yolo_model_error = ""
                    self.logger.log(f"YOLOv8 ONNX 已加载: {YOLOV8_ONNX_PATH.name}")
                else:
                    from ultralytics import YOLO
                    import torch

                    self.device_name = "cuda" if torch.cuda.is_available() else "cpu"
                    if not YOLOV8_MODEL_PATH.exists():
                        raise FileNotFoundError(f"未找到 YOLOv8 模型: {YOLOV8_MODEL_PATH}")
                    self.yolo_model = YOLO(str(YOLOV8_MODEL_PATH))
                    self.yolo_names = {int(k): str(v) for k, v in self.yolo_model.names.items()}
                    self.yolo_backend = "ultralytics"
                    self.yolo_model_error = ""
                    self.logger.log(f"YOLOv8 PyTorch 已加载: {YOLOV8_MODEL_PATH.name}")
            except Exception as exc:
                self.yolo_model = None
                self.yolo_backend = ""
                self.yolo_model_error = str(exc)
                self.logger.log(f"YOLOv8 加载失败: {exc}")

        if self.meter_enabled and self.meter_pipeline is None:
            try:
                if not METER_PROJECT_ROOT.exists():
                    raise FileNotFoundError(f"未找到仪表读数模型目录: {METER_PROJECT_ROOT}")
                from robot_control_page import MeterPipeline

                self.meter_pipeline = MeterPipeline(METER_PROJECT_ROOT)
                self.meter_model_error = ""
                self.logger.log("仪表识别模型已加载")
            except Exception as exc:
                self.meter_pipeline = None
                self.meter_model_error = str(exc)
                self.logger.log(f"仪表识别模型加载失败: {exc}")

        self._refresh_model_state()

        enabled_models: list[tuple[str, bool, str]] = []
        if self.yolo_enabled:
            enabled_models.append(("YOLO", self.yolo_model is not None, self.yolo_model_error))
        if self.meter_enabled:
            enabled_models.append(("仪表模型", self.meter_pipeline is not None, self.meter_model_error))
        if enabled_models and not any(ok for _, ok, _ in enabled_models):
            details = " | ".join(f"{name}不可用: {error or '未加载'}" for name, _, error in enabled_models)
            raise RuntimeError(details)

    def load_models(self) -> tuple[bool, str]:
        try:
            self._load_models()
        except Exception as exc:
            self.model_error = str(exc)
            self.logger.log(f"模型加载失败: {exc}")
            return False, str(exc)
        if self.model_warnings:
            message = f"模型已部分加载: {' | '.join(self.model_warnings)}"
            self.logger.log(message)
            return True, message
        self.model_error = ""
        return True, "模型已加载"

    @staticmethod
    def _normalized_result_stem(source_path: Path) -> str:
        safe = "".join(ch if ch.isalnum() else "_" for ch in source_path.stem).strip("_")
        safe = safe[:60] or "upload"
        return f"{time.strftime('%Y%m%d_%H%M%S')}_{safe}"

    @staticmethod
    def _format_result_time(timestamp: float) -> str:
        if not timestamp:
            return ""
        return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp))

    def _remember_result(self, result: dict[str, Any], *, capture_mode: str, captured_at: float | None = None) -> dict[str, Any]:
        payload = dict(result)
        stamped_at = time.time() if captured_at is None else float(captured_at)
        payload["capture_mode"] = capture_mode
        payload["captured_at"] = round(stamped_at, 3)
        payload["captured_at_text"] = self._format_result_time(stamped_at)
        self.latest_result = dict(payload)
        return payload

    def _infer_uploaded_frame(self, frame: np.ndarray) -> tuple[list[Detection], list[Detection]]:
        yolo_detections: list[Detection] = []
        meter_detections: list[Detection] = []
        with self.inference_lock:
            if self.yolo_enabled and self.yolo_model is not None:
                yolo_detections = self.detect_with_yolo(frame)
            if self.meter_enabled and self.meter_pipeline is not None:
                meter_detections = self.meter_pipeline.detect(frame, self.meter_confidence)
        return yolo_detections, meter_detections

    @staticmethod
    def draw_media_overlay(frame: np.ndarray, lines: list[str]):
        y = 28
        for line in lines:
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 4)
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (240, 240, 240), 1)
            y += 28

    def _annotate_uploaded_frame(
        self,
        frame: np.ndarray,
        yolo_detections: list[Detection],
        meter_detections: list[Detection],
        lines: list[str] | None = None,
    ) -> np.ndarray:
        annotated = frame.copy()
        self.draw_detections(annotated, yolo_detections)
        self.draw_detections(annotated, meter_detections)
        if lines:
            self.draw_media_overlay(annotated, lines)
        return annotated

    def _save_annotated_image_result(
        self,
        *,
        source_path: Path,
        frame: np.ndarray,
        yolo_detections: list[Detection],
        meter_detections: list[Detection],
        message: str,
    ) -> dict[str, Any]:
        annotated = self._annotate_uploaded_frame(frame, yolo_detections, meter_detections)
        result_path = VISION_EXPORT_DIR / f"{self._normalized_result_stem(source_path)}.jpg"
        if not cv2.imwrite(str(result_path), annotated):
            raise RuntimeError("识别结果图片保存失败")
        return {
            "result_name": result_path.name,
            "result_url": f"/api/vision/result/{result_path.name}",
            "processed_frames": 1,
            "total_frames": 1,
            "message": message,
        }

    @staticmethod
    def _create_video_writer(stem: str, fps: float, frame_size: tuple[int, int]) -> tuple[cv2.VideoWriter, Path]:
        normalized_fps = float(fps) if fps and fps > 0 else 15.0
        for suffix, codec in ((".mp4", "mp4v"), (".avi", "XVID"), (".avi", "MJPG")):
            output_path = VISION_EXPORT_DIR / f"{stem}{suffix}"
            writer = cv2.VideoWriter(
                str(output_path),
                cv2.VideoWriter_fourcc(*codec),
                normalized_fps,
                frame_size,
            )
            if writer.isOpened():
                return writer, output_path
            writer.release()
        raise RuntimeError("无法创建识别结果视频文件")

    def _analyze_uploaded_image(self, source_path: Path) -> dict[str, Any]:
        frame = cv2.imread(str(source_path))
        if frame is None:
            raise RuntimeError("上传图片读取失败")

        yolo_detections, meter_detections = self._infer_uploaded_frame(frame)
        return self._save_annotated_image_result(
            source_path=source_path,
            frame=frame,
            yolo_detections=yolo_detections,
            meter_detections=meter_detections,
            message="图片识别完成",
        )

    def _analyze_uploaded_video(self, source_path: Path) -> dict[str, Any]:
        capture = cv2.VideoCapture(str(source_path))
        if not capture.isOpened():
            raise RuntimeError("上传视频打开失败")

        writer: cv2.VideoWriter | None = None
        result_path: Path | None = None
        processed_frames = 0
        total_yolo = 0
        total_meter = 0
        total_frames = max(int(capture.get(cv2.CAP_PROP_FRAME_COUNT)) or 0, 0)
        fps = capture.get(cv2.CAP_PROP_FPS)
        if not fps or fps <= 0 or fps > 120:
            fps = 15.0
        stem = self._normalized_result_stem(source_path)

        try:
            while True:
                ok, frame = capture.read()
                if not ok or frame is None:
                    break
                if writer is None:
                    writer, result_path = self._create_video_writer(stem, fps, (frame.shape[1], frame.shape[0]))

                processed_frames += 1
                yolo_detections, meter_detections = self._infer_uploaded_frame(frame)
                total_yolo += len(yolo_detections)
                total_meter += len(meter_detections)
                annotated = self._annotate_uploaded_frame(frame, yolo_detections, meter_detections)
                writer.write(annotated)
        finally:
            capture.release()
            if writer is not None:
                writer.release()

        if processed_frames <= 0 or result_path is None:
            raise RuntimeError("上传视频中没有可处理的帧")

        return {
            "result_name": result_path.name,
            "result_url": f"/api/vision/result/{result_path.name}",
            "processed_frames": processed_frames,
            "total_frames": total_frames or processed_frames,
            "fps": round(float(fps), 2),
            "message": "视频识别完成",
        }

    def analyze_uploaded_media(self, source_path: Path) -> dict[str, Any]:
        suffix = source_path.suffix.lower()
        if suffix not in SUPPORTED_VISION_MEDIA_EXTENSIONS:
            raise ValueError("暂仅支持上传图片文件")
        if not self.yolo_enabled and not self.meter_enabled:
            raise RuntimeError("请先开启 YOLO 检测或仪表识别")

        try:
            self._load_models()
        except Exception as exc:
            self.model_error = str(exc)
            self.logger.log(f"上传识别模型加载失败: {exc}")
            raise

        self.logger.log(f"开始处理上传文件: {source_path.name}")
        if suffix in IMAGE_EXTENSIONS:
            result = self._remember_result(self._analyze_uploaded_image(source_path), capture_mode="manual_upload")
        else:
            result = self._analyze_uploaded_video(source_path)
        self.logger.log(f"上传文件识别完成: {source_path.name} -> {result['result_name']}")
        return result

    def _schedule_periodic_snapshot(self, frame: np.ndarray, now: float):
        if not self.running or not self.periodic_capture_enabled:
            return
        if not (self.yolo_enabled or self.meter_enabled):
            return
        if self.snapshot_busy or now - self.last_snapshot_started_at < self.snapshot_interval_s:
            return
        self.snapshot_busy = True
        self.last_snapshot_started_at = now
        snapshot = frame.copy()
        threading.Thread(
            target=self._run_periodic_snapshot,
            args=(snapshot, now),
            daemon=True,
        ).start()

    def _run_periodic_snapshot(self, frame: np.ndarray, captured_at: float):
        source_path = UPLOAD_MEDIA_DIR / f"camera_snapshot_{time.strftime('%Y%m%d_%H%M%S', time.localtime(captured_at))}.jpg"
        try:
            if not cv2.imwrite(str(source_path), frame):
                raise RuntimeError("抓拍原图保存失败")
            yolo_detections, meter_detections = self._infer_uploaded_frame(frame)
            result = self._save_annotated_image_result(
                source_path=source_path,
                frame=frame,
                yolo_detections=yolo_detections,
                meter_detections=meter_detections,
                message="抓拍识别完成",
            )
            self._remember_result(result, capture_mode="camera_snapshot", captured_at=captured_at)
            self.last_snapshot_completed_at = time.time()
            self.last_snapshot_error = ""
            self.logger.log(f"自动抓拍识别完成: {result['result_name']}")
        except Exception as exc:
            self.last_snapshot_error = str(exc)
            self.logger.log(f"自动抓拍识别失败: {exc}")
        finally:
            self.snapshot_busy = False

    @staticmethod
    def _serialize_detection(detection: Detection) -> dict[str, Any]:
        payload = asdict(detection)
        payload["center_x"] = round((detection.x1 + detection.x2) / 2.0, 1)
        payload["center_y"] = round((detection.y1 + detection.y2) / 2.0, 1)
        payload["width"] = max(0, detection.x2 - detection.x1)
        payload["height"] = max(0, detection.y2 - detection.y1)
        return payload

    def _recent_detections(self, limit: int = 8) -> list[dict[str, Any]]:
        merged = [
            *[self._serialize_detection(item) for item in self.last_meter_detections],
            *[self._serialize_detection(item) for item in self.last_yolo_detections],
        ]
        merged.sort(
            key=lambda item: (
                1 if item.get("source") == "meter" else 0,
                item.get("width", 0) * item.get("height", 0),
                item.get("confidence", 0.0),
            ),
            reverse=True,
        )
        return merged[:limit]

    def set_flags(
        self,
        *,
        yolo_enabled: bool | None = None,
        meter_enabled: bool | None = None,
        tracking_enabled: bool | None = None,
        yolo_confidence: float | None = None,
        meter_confidence: float | None = None,
    ):
        if yolo_enabled is not None:
            self.yolo_enabled = bool(yolo_enabled)
        if meter_enabled is not None:
            self.meter_enabled = bool(meter_enabled)
        if tracking_enabled is not None:
            self.tracking_enabled = bool(tracking_enabled)
            self.logger.log(f"自动追踪{'开启' if self.tracking_enabled else '关闭'}")
        if yolo_confidence is not None:
            self.yolo_confidence = max(0.1, min(0.95, float(yolo_confidence)))
        if meter_confidence is not None:
            self.meter_confidence = max(0.1, min(0.95, float(meter_confidence)))

    def start(self, camera_index: int = DEFAULT_CAMERA_INDEX) -> tuple[bool, str]:
        if self.running:
            return True, "摄像头已在运行"
        try:
            self._load_models()
        except Exception as exc:
            self.model_error = str(exc)
            self.logger.log(f"模型加载失败: {exc}")
            return False, str(exc)
        self.running = True
        self.camera_index = int(camera_index)
        self.capture_thread = threading.Thread(target=self._worker, daemon=True)
        self.capture_thread.start()
        self.logger.log(f"已请求启动摄像头: index={self.camera_index}")
        if self.model_warnings:
            return True, f"摄像头启动中（{'; '.join(self.model_warnings)}）"
        return True, "摄像头启动中"

    def stop(self):
        self.running = False
        capture = self.capture
        self.capture = None
        if capture is not None:
            with contextlib.suppress(Exception):
                capture.release()
        self.snapshot_busy = False
        self.logger.log("摄像头已停止")

    def _open_camera(self, camera_index: int) -> cv2.VideoCapture:
        capture = cv2.VideoCapture(camera_index)
        if capture.isOpened():
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _worker(self):
        capture = self._open_camera(self.camera_index)
        self.capture = capture
        if not capture.isOpened():
            self.model_error = f"摄像头 {self.camera_index} 打开失败"
            self.running = False
            self.logger.log(self.model_error)
            return
        self.logger.log(f"摄像头已打开: {self.camera_index}")

        while self.running:
            ok, frame = capture.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue

            now = time.time()
            self.frame_size = f"{frame.shape[1]}x{frame.shape[0]}"
            if self.yolo_enabled and self.yolo_model is not None and now - self.last_yolo_infer_at >= self.yolo_interval:
                try:
                    with self.inference_lock:
                        self.last_yolo_detections = self.detect_with_yolo(frame)
                    self.last_yolo_infer_at = now
                except Exception as exc:
                    self.logger.log(f"YOLO 推理失败: {exc}")

            if self.meter_enabled and self.meter_pipeline is not None and now - self.last_meter_infer_at >= self.meter_interval:
                try:
                    with self.inference_lock:
                        self.last_meter_detections = self.meter_pipeline.detect(frame, self.meter_confidence)
                    self.last_meter_infer_at = now
                except Exception as exc:
                    self.logger.log(f"仪表识别失败: {exc}")

            self.run_auto_tracking(frame, now)
            self._schedule_periodic_snapshot(frame, now)

            annotated = frame.copy()
            self.draw_detections(annotated, self.last_yolo_detections)
            self.draw_detections(annotated, self.last_meter_detections)
            self.draw_tracking_overlay(annotated)
            self.draw_overlay(annotated)

            with self.frame_lock:
                self.latest_frame = annotated
                self.last_frame_at = now

            self._fps_counter += 1
            if now - self._fps_tick >= 1.0:
                self.current_fps = self._fps_counter / max(now - self._fps_tick, 1e-6)
                self._fps_tick = now
                self._fps_counter = 0

            sleep_s = max(0.0, (1.0 / DEFAULT_CAMERA_FPS_LIMIT) - 0.005)
            if sleep_s > 0:
                time.sleep(sleep_s)

        capture.release()
        self.capture = None
        self.running = False
        self.logger.log("实时画面线程已退出")

    def detect_with_yolo(self, frame: np.ndarray) -> list[Detection]:
        if self.yolo_model is None:
            return []
        if self.yolo_backend == "opencv-dnn":
            return self._detect_with_onnx(frame)
        if self.yolo_backend == "onnxruntime":
            return self._detect_with_onnxruntime(frame)

        import torch

        with torch.no_grad():
            result = self.yolo_model.predict(
                source=frame,
                conf=self.yolo_confidence,
                imgsz=640,
                verbose=False,
                device=self.device_name,
            )[0]
        detections: list[Detection] = []
        names = result.names if hasattr(result, "names") else self.yolo_names
        for box in result.boxes:
            xyxy = box.xyxy[0].detach().cpu().numpy().astype(int).tolist()
            cls_id = int(box.cls[0].item())
            detections.append(
                Detection(
                    x1=xyxy[0],
                    y1=xyxy[1],
                    x2=xyxy[2],
                    y2=xyxy[3],
                    label=str(names.get(cls_id, cls_id)),
                    confidence=float(box.conf[0].item()),
                    source="yolo",
                )
            )
        return detections

    def _postprocess_yolo_outputs(self, frame: np.ndarray, outputs: np.ndarray) -> list[Detection]:
        if outputs.ndim == 3:
            outputs = outputs[0]
        if outputs.shape[0] < outputs.shape[1]:
            outputs = outputs.transpose(1, 0)

        boxes: list[list[int]] = []
        confidences: list[float] = []
        class_ids: list[int] = []
        x_factor = frame.shape[1] / 640.0
        y_factor = frame.shape[0] / 640.0

        for row in outputs:
            class_scores = row[4:]
            class_id = int(np.argmax(class_scores))
            confidence = float(class_scores[class_id])
            if confidence < self.yolo_confidence:
                continue
            cx, cy, w, h = row[:4]
            left = int((cx - w / 2.0) * x_factor)
            top = int((cy - h / 2.0) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)
            boxes.append([left, top, width, height])
            confidences.append(confidence)
            class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.yolo_confidence, 0.45)
        detections: list[Detection] = []
        if len(indices) == 0:
            return detections
        for raw_index in indices:
            index = int(raw_index if np.isscalar(raw_index) else raw_index[0])
            left, top, width, height = boxes[index]
            x1 = max(0, left)
            y1 = max(0, top)
            x2 = min(frame.shape[1] - 1, left + width)
            y2 = min(frame.shape[0] - 1, top + height)
            cls_id = class_ids[index]
            detections.append(
                Detection(
                    x1=x1,
                    y1=y1,
                    x2=x2,
                    y2=y2,
                    label=str(self.yolo_names.get(cls_id, cls_id)),
                    confidence=float(confidences[index]),
                    source="yolo",
                )
            )
        return detections

    def _detect_with_onnx(self, frame: np.ndarray) -> list[Detection]:
        assert self.yolo_model is not None
        image = cv2.resize(frame, (640, 640), interpolation=cv2.INTER_LINEAR)
        blob = cv2.dnn.blobFromImage(image, scalefactor=1.0 / 255.0, size=(640, 640), swapRB=True, crop=False)
        self.yolo_model.setInput(blob)
        outputs = self.yolo_model.forward()
        return self._postprocess_yolo_outputs(frame, outputs)

    def _detect_with_onnxruntime(self, frame: np.ndarray) -> list[Detection]:
        assert self.yolo_model is not None
        image = cv2.resize(frame, (640, 640), interpolation=cv2.INTER_LINEAR)
        blob = cv2.dnn.blobFromImage(image, scalefactor=1.0 / 255.0, size=(640, 640), swapRB=True, crop=False)
        input_name = self.yolo_model.get_inputs()[0].name
        outputs = self.yolo_model.run(None, {input_name: blob.astype(np.float32)})[0]
        return self._postprocess_yolo_outputs(frame, outputs)

    def select_tracking_target(self) -> Detection | None:
        candidates: list[Detection] = []
        meter_candidates = [item for item in self.last_meter_detections if item.label == "meter"]
        if meter_candidates:
            candidates.extend(meter_candidates)
        else:
            candidates.extend(self.last_meter_detections)
        candidates.extend(self.last_yolo_detections)
        if not candidates:
            return None
        return max(candidates, key=lambda d: ((1 if d.source == "meter" else 0), (d.x2 - d.x1) * (d.y2 - d.y1), d.confidence))

    def run_auto_tracking(self, frame: np.ndarray, now: float):
        if not self.tracking_enabled:
            self.tracking_target = None
            self.tracking_error = (0.0, 0.0)
            return
        target = self.select_tracking_target()
        self.tracking_target = target
        if target is None:
            self.tracking_error = (0.0, 0.0)
            return
        height, width = frame.shape[:2]
        target_x = (target.x1 + target.x2) / 2.0
        target_y = (target.y1 + target.y2) / 2.0
        error_x = (target_x - width / 2.0) / max(width, 1)
        error_y = (target_y - height / 2.0) / max(height, 1)
        self.tracking_error = (error_x, error_y)
        if now - self.last_tracking_at < self.tracking_interval:
            return
        if not self.controller.is_connected():
            return
        targets: dict[int, int] = {}
        if abs(error_x) > TRACKING_DEADZONE_X:
            delta_x = int(np.clip(-error_x * TRACKING_GAIN_X, -TRACKING_MAX_STEP_X, TRACKING_MAX_STEP_X))
            targets[0] = self.controller.servo_values[0] + delta_x
        if abs(error_y) > TRACKING_DEADZONE_Y:
            delta_y = int(np.clip(error_y * TRACKING_GAIN_Y, -TRACKING_MAX_STEP_Y, TRACKING_MAX_STEP_Y))
            targets[1] = self.controller.servo_values[1] + delta_y
        if self.controller.servo_values[3] != TRACKING_FIXED_JOINT3 or targets:
            targets[3] = TRACKING_FIXED_JOINT3
        if not targets:
            return
        ok, _ = self.controller.send_servo_targets(targets, duration=120)
        if ok:
            self.last_tracking_at = now

    @staticmethod
    def draw_detections(frame: np.ndarray, detections: list[Detection]):
        font_scale = max(0.72, min(1.05, frame.shape[1] / 1180.0))
        thickness = 2 if font_scale < 0.9 else 3
        for detection in detections:
            color = (64, 220, 64) if detection.source == "yolo" else (0, 165, 255)
            prefix = "YOLO" if detection.source == "yolo" else "Meter"
            cv2.rectangle(frame, (detection.x1, detection.y1), (detection.x2, detection.y2), color, thickness)
            label = f"{prefix}:{detection.label} {detection.confidence:.2f}"
            if detection.reading:
                label += f" | {detection.reading}"
            (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
            text_x = max(0, detection.x1)
            text_y = detection.y1 - 10
            if text_y <= text_height:
                text_y = min(frame.shape[0] - 6, detection.y1 + text_height + 14)
            cv2.rectangle(frame, (text_x, text_y - text_height - 8), (text_x + text_width + 8, text_y + 4), color, -1)
            cv2.putText(
                frame,
                label,
                (text_x + 4, text_y - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (20, 20, 20),
                thickness,
            )

    def draw_tracking_overlay(self, frame: np.ndarray):
        height, width = frame.shape[:2]
        center = (width // 2, height // 2)
        center_color = (0, 220, 255) if self.tracking_enabled else (120, 120, 120)
        cv2.drawMarker(frame, center, center_color, markerType=cv2.MARKER_CROSS, markerSize=28, thickness=2, line_type=cv2.LINE_AA)
        cv2.circle(frame, center, 18, center_color, 1, lineType=cv2.LINE_AA)
        if self.tracking_target is None:
            return
        target_center = (int((self.tracking_target.x1 + self.tracking_target.x2) / 2), int((self.tracking_target.y1 + self.tracking_target.y2) / 2))
        cv2.circle(frame, target_center, 7, (0, 255, 255), -1, lineType=cv2.LINE_AA)
        cv2.line(frame, center, target_center, (0, 255, 255), 2, lineType=cv2.LINE_AA)
        dx, dy = self.tracking_error
        label = f"Track {self.tracking_target.label} dx={dx:+.3f} dy={dy:+.3f}"
        cv2.putText(frame, label, (12, max(32, height - 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 3)
        cv2.putText(frame, label, (12, max(32, height - 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 1)

    def draw_overlay(self, frame: np.ndarray):
        tracking_state = "ON" if self.tracking_enabled else "OFF"
        lines = [
            f"FPS: {self.current_fps:.1f}",
            f"Resolution: {self.frame_size}",
            f"Control: {self.controller.current_target()}",
            f"YOLO boxes: {len(self.last_yolo_detections)}",
            f"Meter boxes: {len(self.last_meter_detections)}",
            f"Tracking: {tracking_state}",
        ]
        y = 28
        for line in lines:
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 4)
            cv2.putText(frame, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (240, 240, 240), 1)
            y += 28

    def frame_jpeg(self) -> bytes:
        with self.frame_lock:
            frame = self.latest_frame.copy()
        ok, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return b""
        return buffer.tobytes()

    def status(self) -> dict[str, Any]:
        tracking_error = [round(float(value), 4) for value in self.tracking_error]
        return {
            "running": self.running,
            "camera_index": self.camera_index,
            "frame_size": self.frame_size,
            "fps": round(self.current_fps, 2),
            "yolo_backend": self.yolo_backend,
            "yolo_enabled": self.yolo_enabled,
            "meter_enabled": self.meter_enabled,
            "tracking_enabled": self.tracking_enabled,
            "yolo_confidence": round(self.yolo_confidence, 2),
            "meter_confidence": round(self.meter_confidence, 2),
            "yolo_count": len(self.last_yolo_detections),
            "meter_count": len(self.last_meter_detections),
            "snapshot_interval_s": round(float(self.snapshot_interval_s), 1),
            "periodic_capture_enabled": self.periodic_capture_enabled,
            "snapshot_busy": self.snapshot_busy,
            "last_snapshot_started_at": round(self.last_snapshot_started_at, 3) if self.last_snapshot_started_at else 0.0,
            "last_snapshot_completed_at": round(self.last_snapshot_completed_at, 3) if self.last_snapshot_completed_at else 0.0,
            "last_snapshot_completed_text": self._format_result_time(self.last_snapshot_completed_at),
            "last_snapshot_error": self.last_snapshot_error,
            "latest_result": dict(self.latest_result) if isinstance(self.latest_result, dict) else None,
            "model_error": self.model_error,
            "model_warnings": list(self.model_warnings),
            "yolo_model_error": self.yolo_model_error,
            "meter_model_error": self.meter_model_error,
            "device_name": self.device_name,
            "yolo_model_loaded": self.yolo_model is not None,
            "meter_model_loaded": self.meter_pipeline is not None,
            "active_detectors": [
                name
                for name, ready in (
                    ("YOLO", self.yolo_enabled and self.yolo_model is not None),
                    ("仪表模型", self.meter_enabled and self.meter_pipeline is not None),
                )
                if ready
            ],
            "recent_detections": self._recent_detections(),
            "tracking_target": asdict(self.tracking_target) if self.tracking_target else None,
            "tracking_error": tracking_error,
            "tracking_joint_policy": "自动跟踪仅调整关节 0/1，关节 3 固定 2200，其余关节保持当前值。",
            "last_frame_age": None if not self.last_frame_at else round(time.time() - self.last_frame_at, 2),
        }


class LidarSystem:
    def __init__(self, logger: RuntimeLogger):
        self.logger = logger
        self.backend = LidarSlamBackend(log_callback=self.logger.log)
        self.scan_backend = LidarAvoidanceBackend(log_callback=self.logger.log, motion_callback=None)
        self.port = DEFAULT_LIDAR_PORT
        self.scan_only_mode = False

    def list_ports(self) -> list[str]:
        return discover_serial_ports()

    def start(self, port: str) -> tuple[bool, str]:
        self.port = port or DEFAULT_LIDAR_PORT
        self.scan_only_mode = False
        self.scan_backend.stop()
        ok = self.backend.start(self.port)
        state = self.backend.snapshot()
        if ok:
            return True, state.status

        reason = state.last_error or state.status
        if "pybreezyslam" in reason.lower() or "breezyslam" in state.status.lower():
            scan_ok = self.scan_backend.start(self.port)
            if scan_ok:
                self.scan_only_mode = True
                self.logger.log("检测到 pybreezyslam 缺失，已切换为雷达扫点模式")
                return True, "已启动雷达扫点模式（当前板子缺少 pybreezyslam，暂不提供本地图像建图）"
        return False, reason

    def stop(self):
        self.backend.stop()
        self.scan_backend.stop()
        self.scan_only_mode = False

    def reset(self):
        if self.scan_only_mode:
            self.logger.log("扫点模式下没有可重置的地图")
            return
        self.backend.reset_map()

    def save(self) -> dict[str, str]:
        if self.scan_only_mode:
            raise RuntimeError("当前是扫点模式，缺少 pybreezyslam，暂时不能保存建图结果")
        target = MAP_EXPORT_DIR / time.strftime("board_map_%Y%m%d_%H%M%S")
        saved = self.backend.save_map(target)
        return {key: str(value) for key, value in saved.items()}

    def scan_jpeg(self) -> bytes:
        if self.scan_only_mode:
            state = self.scan_backend.snapshot()
            image = LidarSlamWindow.render_scan_image(state.scan_points, size=(960, 720))
        else:
            state = self.backend.snapshot()
            image = LidarSlamWindow.render_scan_image(state.scan_points, size=(960, 720))
        ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        return buffer.tobytes() if ok else b""

    def map_jpeg(self) -> bytes:
        if self.scan_only_mode:
            return draw_text_panel(
                [
                    "当前正在运行: 雷达扫点模式",
                    f"串口: {self.port}",
                    "原因: 板子缺少 pybreezyslam 编译扩展",
                    "当前可正常看实时扫点，但暂时不能生成地图图像。",
                    "后续补齐依赖后，这一页会恢复真正建图。",
                ]
            )
        state = self.backend.snapshot()
        image = LidarSlamWindow.render_map_image(None, state.map_image, state.pose_mm, state.pose_history_mm, size=(960, 720), map_size_meters=state.map_size_meters)
        ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        return buffer.tobytes() if ok else b""

    def status(self) -> dict[str, Any]:
        if self.scan_only_mode:
            state = self.scan_backend.snapshot()
            return {
                "running": state.running,
                "port": state.port,
                "status": "雷达扫点模式运行中",
                "mode": "scan_only",
                "scan_count": state.scan_count,
                "device_info": state.device_info,
                "health": state.health,
                "pose_mm": (0.0, 0.0, 0.0),
                "map_size_pixels": 0,
                "map_size_meters": 0.0,
                "last_error": "缺少 pybreezyslam，暂时无法在板子本机做当前这套建图",
            }
        state = self.backend.snapshot()
        return {
            "running": state.running,
            "port": state.port,
            "status": state.status,
            "mode": "slam",
            "scan_count": state.scan_count,
            "device_info": state.device_info,
            "health": state.health,
            "pose_mm": state.pose_mm,
            "map_size_pixels": state.map_size_pixels,
            "map_size_meters": state.map_size_meters,
            "last_error": state.last_error,
        }


class AvoidanceSystem:
    def __init__(self, logger: RuntimeLogger, controller: SerialMotionController):
        self.logger = logger
        self.controller = controller
        self.backend = LidarAvoidanceBackend(log_callback=self.logger.log, motion_callback=self._motion_callback)
        self.port = DEFAULT_LIDAR_PORT
        self._interlock_callback = None

    def set_interlock_callback(self, callback):
        self._interlock_callback = callback

    def _motion_callback(self, command: str, action: str):
        if self._interlock_callback is not None:
            try:
                if self._interlock_callback(command, action):
                    return
            except Exception as exc:
                self.logger.log(f"避障联锁回调异常: {exc}")
        self.controller.send_raw(command)

    def list_ports(self) -> list[str]:
        return discover_lidar_ports(self.controller.port)

    def start_lidar(self, port: str) -> tuple[bool, str]:
        available_ports = self.list_ports()
        self.port = port or (available_ports[0] if available_ports else DEFAULT_LIDAR_PORT)
        if is_control_serial_port(self.port, self.controller.port):
            return False, f"所选串口 {self.port} 是控制串口，不能用于雷达避障"
        ok = self.backend.start(self.port)
        return ok, self.backend.status

    def stop_lidar(self):
        self.backend.stop()

    def start(self) -> tuple[bool, str]:
        return self.backend.start_avoidance()

    def stop(self) -> tuple[bool, str]:
        return self.backend.stop_avoidance()

    def set_threshold(self, threshold_mm: int):
        self.backend.set_threshold_mm(threshold_mm)
        self.logger.log(f"避障阈值已设置为 {threshold_mm} mm")

    def scan_jpeg(self) -> bytes:
        state = self.backend.snapshot()
        image = LidarSlamWindow.render_scan_image(state.scan_points, size=(960, 720))
        ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        return buffer.tobytes() if ok else b""

    def status(self) -> dict[str, Any]:
        state = self.backend.snapshot()
        return {
            "running": state.running,
            "status": state.status,
            "port": state.port,
            "scan_count": state.scan_count,
            "front_mm": state.front_mm,
            "front_left_mm": state.front_left_mm,
            "front_right_mm": state.front_right_mm,
            "left_mm": state.left_mm,
            "right_mm": state.right_mm,
            "rear_mm": state.rear_mm,
            "threshold_mm": state.threshold_mm,
            "avoidance_enabled": state.avoidance_enabled,
            "current_action": state.current_action,
            "device_info": state.device_info,
            "health": state.health,
            "last_error": state.last_error,
        }


class MapAnnotationStore:
    def __init__(self, logger: RuntimeLogger):
        self.logger = logger
        self.maps_dir = MAP_EXPORT_DIR

    def list_maps(self) -> list[dict[str, str]]:
        entries: list[dict[str, str]] = []
        for yaml_path in sorted(self.maps_dir.glob("*.yaml")):
            image_name = ""
            with contextlib.suppress(Exception):
                for line in yaml_path.read_text(encoding="utf-8").splitlines():
                    if line.startswith("image:"):
                        image_name = line.split(":", 1)[1].strip()
                        break
            entries.append({"name": yaml_path.stem, "yaml": yaml_path.name, "image": image_name})
        return entries

    def _annotation_path(self, map_name: str) -> Path:
        return self.maps_dir / f"{Path(map_name).stem}.annotations.json"

    def _map_yaml_path(self, map_name: str) -> Path:
        return self.maps_dir / f"{Path(map_name).stem}.yaml"

    def map_metadata(self, map_name: str) -> dict[str, Any]:
        stem = Path(map_name).stem
        yaml_path = self._map_yaml_path(stem)
        metadata: dict[str, Any] = {
            "name": stem,
            "yaml_file": yaml_path.name if yaml_path.exists() else "",
            "image_file": "",
            "resolution": 0.05,
            "origin": [0.0, 0.0, 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
            "width_px": 0,
            "height_px": 0,
        }
        if yaml_path.exists():
            with contextlib.suppress(Exception):
                for line in yaml_path.read_text(encoding="utf-8", errors="replace").splitlines():
                    if ":" not in line:
                        continue
                    key, value = line.split(":", 1)
                    key = key.strip()
                    value = value.strip()
                    if key == "image":
                        metadata["image_file"] = Path(value).name
                    elif key == "resolution":
                        metadata["resolution"] = float(value)
                    elif key == "origin":
                        origin = json.loads(value)
                        if isinstance(origin, list) and len(origin) >= 3:
                            metadata["origin"] = [float(origin[0]), float(origin[1]), float(origin[2])]
                    elif key == "negate":
                        metadata["negate"] = int(float(value))
                    elif key == "occupied_thresh":
                        metadata["occupied_thresh"] = float(value)
                    elif key == "free_thresh":
                        metadata["free_thresh"] = float(value)

        image_path = self.map_image_path(stem)
        if image_path is not None and image_path.exists():
            metadata["image_file"] = image_path.name
            with contextlib.suppress(Exception):
                with Image.open(image_path) as image:
                    metadata["width_px"], metadata["height_px"] = image.size
        return metadata

    def load_annotations(self, map_name: str) -> dict[str, Any]:
        path = self._annotation_path(map_name)
        payload: dict[str, Any]
        if not path.exists():
            payload = {"points": []}
        else:
            payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            payload = {"points": []}
        payload["map_name"] = Path(map_name).stem
        payload["points"] = list(payload.get("points", []))
        payload["map"] = self.map_metadata(map_name)
        return payload

    def save_annotations(self, map_name: str, payload: dict[str, Any]) -> str:
        path = self._annotation_path(map_name)
        stored = dict(payload or {})
        stored.pop("map", None)
        stored["map_name"] = Path(map_name).stem
        stored["points"] = list(stored.get("points", []))
        path.write_text(json.dumps(stored, ensure_ascii=False, indent=2), encoding="utf-8")
        self.logger.log(f"已保存地图标注: {path.name}")
        return str(path)

    def map_image_path(self, map_name: str) -> Path | None:
        stem = Path(map_name).stem
        for suffix in (".png", ".pgm"):
            candidate = self.maps_dir / f"{stem}{suffix}"
            if candidate.exists():
                return candidate
        yaml_path = self.maps_dir / f"{stem}.yaml"
        if yaml_path.exists():
            with contextlib.suppress(Exception):
                for line in yaml_path.read_text(encoding="utf-8").splitlines():
                    if line.startswith("image:"):
                        rel = line.split(":", 1)[1].strip()
                        candidate = yaml_path.parent / rel
                        if candidate.exists():
                            return candidate
        return None


class RosSystem:
    def __init__(self, logger: RuntimeLogger, controller: "SerialMotionController | None" = None):
        self.logger = logger
        self.controller = controller
        self.workspace = Path.home() / "Desktop" / "car2.0_board_deploy"
        self.ros_tools_dir = self.workspace / "ros2_tools"
        self.maps_dir = self.workspace / "lidar_maps"
        self.ros_configs_dir = self.workspace / "ros_configs"
        self.ros_probe_script = self.ros_tools_dir / "ros_probe.py"
        self.wifi_probe_script = self.ros_tools_dir / "wifi_runtime_probe.py"
        self.rviz_config = self.ros_configs_dir / "lidar_slam.rviz"
        self.ros_log_dir = self.workspace / "logs" / "ros2_mapping"
        self.imu_driver_status_path = self.ros_log_dir / IMU_DRIVER_STATUS_FILE_NAME
        self.wifi_probe_log_path = Path(
            os.environ.get("CAR2_WIFI_PROBE_LOG", str(self.ros_log_dir / WIFI_RUNTIME_PROBE_LOG_NAME))
        )
        self.wifi_probe_state_path = Path(
            os.environ.get("CAR2_WIFI_PROBE_STATE", str(self.ros_log_dir / WIFI_RUNTIME_PROBE_STATE_FILE_NAME))
        )
        self.ros_setup = Path("/opt/ros/foxy/setup.bash")
        self.rock_ws = Path.home() / "Desktop" / "rock_ws" / "ros_ws"
        self.last_lidar_port = self._default_lidar_port()
        self._status_cache: dict[str, Any] = {}
        self._status_cached_at = 0.0
        self._cache_ttl_s = 2.0
        self._stream_window_s = 2.5
        self._mapping_session_started_at = 0.0
        self._localization_session_started_at = 0.0
        self._localization_map_name = ""
        self._last_ready_error = ""
        self._rviz_default_render_mode = "compatibility"
        self._rviz_mode_file = self.ros_log_dir / "rviz2.mode.json"
        self._rviz_launch_requested = False
        self._rviz_launch_source = ""
        self._rviz_launch_target = ""
        self._rviz_launch_error = ""
        self._rviz_launch_warning = ""
        self._graph_cache: dict[str, set[str]] = {"topics": set(), "nodes": set()}
        self._graph_cached_at = 0.0
        self._graph_cache_ttl_s = 8.0
        self._graph_cli_timeout_s = 3.0
        self._graph_refresh_wait_s = (self._graph_cli_timeout_s * 2.0) + 0.8
        self._graph_empty_refreshes = 0
        self._graph_refresh_lock = threading.Lock()
        self._wifi_probe_lock = threading.Lock()
        self._wifi_probe_stop_event = threading.Event()
        self._wifi_probe_thread: threading.Thread | None = None
        self._wifi_probe_session = ""
        self._wifi_probe_last_summary: dict[str, Any] = {}
        self._wifi_probe_last_error = ""
        self._wifi_probe_last_sample_at = 0.0
        self._wifi_probe_interval_s = WIFI_RUNTIME_PROBE_INTERVAL_S
        self._scan_probe_lock = threading.Lock()
        self._scan_probe_last_ok_at = 0.0
        self._scan_probe_last_attempt_at = 0.0
        self._scan_probe_last_error = ""
        self._scan_probe_inflight = False
        self._scan_probe_timeout_s = 1.5
        self._scan_probe_interval_s = 1.0
        self._imu_raw_probe_lock = threading.Lock()
        self._imu_raw_probe_last_ok_at = 0.0
        self._imu_raw_probe_last_attempt_at = 0.0
        self._imu_raw_probe_last_error = ""
        self._imu_raw_probe_inflight = False
        self._imu_raw_probe_timeout_s = 1.5
        self._imu_raw_probe_interval_s = 1.0
        self._imu_filtered_probe_lock = threading.Lock()
        self._imu_filtered_probe_last_ok_at = 0.0
        self._imu_filtered_probe_last_attempt_at = 0.0
        self._imu_filtered_probe_last_error = ""
        self._imu_filtered_probe_inflight = False
        self._imu_filtered_probe_timeout_s = 1.5
        self._imu_filtered_probe_interval_s = 1.0
        self._odom_probe_lock = threading.Lock()
        self._odom_probe_last_ok_at = 0.0
        self._odom_probe_last_attempt_at = 0.0
        self._odom_probe_last_error = ""
        self._odom_probe_inflight = False
        self._odom_probe_timeout_s = 1.5
        self._odom_probe_interval_s = 1.0
        self._map_probe_lock = threading.Lock()
        self._map_probe_last_ok_at = 0.0
        self._map_probe_last_attempt_at = 0.0
        self._map_probe_last_error = ""
        self._map_probe_inflight = False
        self._map_probe_timeout_s = 1.8
        self._map_probe_interval_s = 1.0
        self._map_to_odom_tf_lock = threading.Lock()
        self._map_to_odom_tf_last_ok_at = 0.0
        self._map_to_odom_tf_last_attempt_at = 0.0
        self._map_to_odom_tf_last_error = ""
        self._map_to_odom_tf_inflight = False
        self._map_to_odom_tf_timeout_s = 1.5
        self._map_to_odom_tf_interval_s = 1.0
        self.preview = RosPreviewMirror(log_callback=self.logger.log)
        self.preview.start()
        self._refresh_ros_graph_cache_async()

    @staticmethod
    def _exclusive_local_mode() -> bool:
        return os.environ.get("CAR2_EXCLUSIVE_GUARD_OK", "").strip() == "1"

    def _default_lidar_port(self) -> str:
        ports = self.list_lidar_ports()
        if ports:
            return ports[0]
        return "/dev/rplidar"

    @staticmethod
    def _imu_mode() -> str:
        return os.environ.get("CAR2_IMU_MODE", CAR2_IMU_MODE_REQUIRED).strip() or CAR2_IMU_MODE_REQUIRED

    @staticmethod
    def _imu_port() -> str:
        return os.environ.get("CAR2_IMU_PORT", DEFAULT_IMU_PORT).strip() or DEFAULT_IMU_PORT

    @staticmethod
    def _imu_baudrate() -> int:
        with contextlib.suppress(Exception):
            return int(os.environ.get("CAR2_IMU_BAUDRATE", str(DEFAULT_IMU_BAUDRATE)).strip())
        return DEFAULT_IMU_BAUDRATE

    @staticmethod
    def _imu_frame() -> str:
        return os.environ.get("CAR2_IMU_FRAME", DEFAULT_IMU_FRAME).strip() or DEFAULT_IMU_FRAME

    def _read_imu_driver_status(self) -> dict[str, Any]:
        status_path = self.imu_driver_status_path
        if not status_path.exists():
            return {}
        with contextlib.suppress(Exception):
            payload = json.loads(status_path.read_text(encoding="utf-8", errors="replace"))
            if isinstance(payload, dict):
                return payload
        return {}

    def list_ports(self) -> list[str]:
        return discover_serial_ports()

    def list_lidar_ports(self) -> list[str]:
        ports = discover_lidar_ports(self._active_control_port())
        last_lidar_port = getattr(self, "last_lidar_port", "")
        if (
            last_lidar_port
            and last_lidar_port not in ports
            and not self._is_control_port(last_lidar_port)
            and self._port_exists(last_lidar_port)
        ):
            ports.insert(0, last_lidar_port)
        return ports

    def _active_control_port(self) -> str:
        return "" if self.controller is None else self.controller.port

    def _is_control_port(self, port: str) -> bool:
        return is_control_serial_port(port, self._active_control_port())

    def _resolve_lidar_port(self, port: str) -> str:
        selected_port = (port or self.last_lidar_port or self._default_lidar_port()).strip()
        if not selected_port:
            selected_port = self._default_lidar_port()
        if self._is_control_port(selected_port):
            raise RuntimeError(f"所选串口 {selected_port} 是控制串口，不允许用于雷达建图，请改选 /dev/rplidar")
        if selected_port.startswith("/dev/") and not self._port_exists(selected_port):
            raise RuntimeError(f"未检测到雷达串口: {selected_port}")
        return selected_port

    def _reset_preview_session(self):
        self.preview.reset()
        self._reset_imu_probe_state()
        self._reset_odom_probe_state()
        self._reset_scan_probe_state()
        self._reset_map_probe_state()
        self._reset_map_to_odom_tf_probe_state()
        self._status_cached_at = 0.0

    def _reset_imu_probe_state(self):
        with self._imu_raw_probe_lock:
            self._imu_raw_probe_last_ok_at = 0.0
            self._imu_raw_probe_last_attempt_at = 0.0
            self._imu_raw_probe_last_error = ""
            self._imu_raw_probe_inflight = False
        with self._imu_filtered_probe_lock:
            self._imu_filtered_probe_last_ok_at = 0.0
            self._imu_filtered_probe_last_attempt_at = 0.0
            self._imu_filtered_probe_last_error = ""
            self._imu_filtered_probe_inflight = False

    def _reset_odom_probe_state(self):
        with self._odom_probe_lock:
            self._odom_probe_last_ok_at = 0.0
            self._odom_probe_last_attempt_at = 0.0
            self._odom_probe_last_error = ""
            self._odom_probe_inflight = False

    def _reset_scan_probe_state(self):
        with self._scan_probe_lock:
            self._scan_probe_last_ok_at = 0.0
            self._scan_probe_last_attempt_at = 0.0
            self._scan_probe_last_error = ""
            self._scan_probe_inflight = False

    def _reset_map_probe_state(self):
        with self._map_probe_lock:
            self._map_probe_last_ok_at = 0.0
            self._map_probe_last_attempt_at = 0.0
            self._map_probe_last_error = ""
            self._map_probe_inflight = False

    def _reset_map_to_odom_tf_probe_state(self):
        with self._map_to_odom_tf_lock:
            self._map_to_odom_tf_last_ok_at = 0.0
            self._map_to_odom_tf_last_attempt_at = 0.0
            self._map_to_odom_tf_last_error = ""
            self._map_to_odom_tf_inflight = False

    def _clear_ready_error(self):
        self._last_ready_error = ""

    def _set_ready_error(self, message: str):
        self._last_ready_error = str(message or "").strip()

    def error_code_for(self, message: str) -> str:
        return self._ready_error_code(message)

    def _pid_file(self, name: str) -> Path:
        return self.ros_log_dir / f"{name}.pid"

    @staticmethod
    def _pid_running(pid: int) -> bool:
        try:
            os.kill(pid, 0)
            return True
        except Exception:
            return False

    def _pid_file_running(self, name: str) -> bool:
        pid_file = self._pid_file(name)
        if not pid_file.exists():
            return False
        with contextlib.suppress(Exception):
            pid = int(pid_file.read_text(encoding="utf-8", errors="replace").strip())
            running = self._pid_running(pid)
            if not running:
                with contextlib.suppress(Exception):
                    pid_file.unlink()
            return running
        with contextlib.suppress(Exception):
            pid_file.unlink()
        return False

    def _log_has_any(self, name: str, patterns: tuple[str, ...], lines: int = 120) -> bool:
        if not patterns:
            return False
        tail = "\n".join(self._log_tail(name, lines=lines)).lower()
        return any(pattern.lower() in tail for pattern in patterns)

    def _managed_process_running(self, name: str, keyword: str, *, fatal_patterns: tuple[str, ...] = ()) -> bool:
        if self._log_has_any(name, fatal_patterns):
            return False
        return self._pid_file_running(name) or self._process_running(keyword)

    def _refresh_ros_graph_cache(self):
        topics = self._ros2_topic_list()
        nodes = self._ros2_node_list()
        now = time.time()
        has_graph_data = bool(topics or nodes)
        if has_graph_data:
            self._graph_cache = {
                "topics": set(topics),
                "nodes": set(nodes),
            }
            self._graph_empty_refreshes = 0
            self._graph_cached_at = now
            return
        self._graph_empty_refreshes += 1
        if self._graph_empty_refreshes >= 2 or not (
            self._graph_cache.get("topics") or self._graph_cache.get("nodes")
        ):
            self._graph_cache = {
                "topics": set(),
                "nodes": set(),
            }
        self._graph_cached_at = now

    def _refresh_ros_graph_cache_async(self):
        if not self._graph_refresh_lock.acquire(blocking=False):
            return

        def _worker():
            try:
                self._refresh_ros_graph_cache()
            finally:
                self._graph_refresh_lock.release()

        threading.Thread(target=_worker, daemon=True).start()

    def _ros_cli_graph_snapshot(self) -> tuple[set[str], set[str]]:
        cache_empty = not self._graph_cache.get("topics") and not self._graph_cache.get("nodes")
        if cache_empty or time.time() - self._graph_cached_at > self._graph_cache_ttl_s:
            if self._graph_refresh_lock.acquire(blocking=False):
                try:
                    self._refresh_ros_graph_cache()
                finally:
                    self._graph_refresh_lock.release()
            elif cache_empty:
                deadline = time.time() + self._graph_refresh_wait_s
                while time.time() < deadline and not (
                    self._graph_cache.get("topics") or self._graph_cache.get("nodes")
                ):
                    time.sleep(0.05)
                if (
                    not (self._graph_cache.get("topics") or self._graph_cache.get("nodes"))
                    and self._graph_refresh_lock.acquire(blocking=False)
                ):
                    try:
                        self._refresh_ros_graph_cache()
                    finally:
                        self._graph_refresh_lock.release()
        return (
            set(self._graph_cache.get("topics", set())),
            set(self._graph_cache.get("nodes", set())),
        )

    def _log_tail(self, name: str, lines: int = 40) -> list[str]:
        log_file = self.ros_log_dir / f"{name}.log"
        if not log_file.exists():
            return []
        with contextlib.suppress(Exception):
            return log_file.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:]
        return []

    def _rviz_log_paths(self) -> list[Path]:
        candidates = list(self.ros_log_dir.glob("rviz2_manual_*.log"))
        return sorted(
            candidates,
            key=lambda path: path.stat().st_mtime if path.exists() else 0.0,
            reverse=True,
        )

    def _read_rviz_mode_info(self) -> dict[str, Any]:
        if not self._rviz_mode_file.exists():
            return {}
        with contextlib.suppress(Exception):
            payload = json.loads(self._rviz_mode_file.read_text(encoding="utf-8", errors="replace"))
            if isinstance(payload, dict):
                return payload
        return {}

    def _write_rviz_mode_info(
        self,
        *,
        render_mode: str,
        display: str,
        xauthority: str,
        xdg_runtime_dir: str,
        log_file: Path,
    ):
        payload = {
            "render_mode": render_mode,
            "display": display,
            "xauthority": xauthority,
            "xdg_runtime_dir": xdg_runtime_dir,
            "log_file": str(log_file),
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        self._rviz_mode_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    def _rviz_log_tail(self, lines: int = 80) -> list[str]:
        mode_info = self._read_rviz_mode_info()
        candidates: list[Path] = []
        log_hint = str(mode_info.get("log_file") or "").strip()
        if log_hint:
            candidates.append(Path(log_hint))
        for path in self._rviz_log_paths():
            if path not in candidates:
                candidates.append(path)
        for log_file in candidates:
            if not log_file.exists():
                continue
            with contextlib.suppress(Exception):
                return log_file.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:]
        return []

    def _ensure_rviz_runtime_dir(self) -> str:
        uid = getattr(os, "getuid", lambda: 1000)()
        runtime_dir = Path(f"/run/user/{uid}")
        if not runtime_dir.exists():
            runtime_dir = Path("/tmp") / f"runtime-{Path.home().name}"
        runtime_dir.mkdir(parents=True, exist_ok=True)
        with contextlib.suppress(OSError):
            os.chmod(runtime_dir, 0o700)
        return str(runtime_dir)

    def _fallback_graphical_env(self) -> dict[str, str]:
        home = Path.home()
        uid = getattr(os, "getuid", lambda: 1000)()
        runtime_dir = Path(f"/run/user/{uid}")
        if not runtime_dir.exists():
            runtime_dir = Path("/run/user/1000")
        if not runtime_dir.exists():
            runtime_dir = Path(self._ensure_rviz_runtime_dir())
        xauthority = home / ".Xauthority"
        return {
            "XAUTHORITY": str(xauthority),
            "XDG_RUNTIME_DIR": str(runtime_dir),
            "DBUS_SESSION_BUS_ADDRESS": f"unix:path={runtime_dir / 'bus'}",
        }

    @staticmethod
    def _current_process_graphical_env() -> dict[str, str]:
        keys = (
            "DISPLAY",
            "XAUTHORITY",
            "XDG_RUNTIME_DIR",
            "DBUS_SESSION_BUS_ADDRESS",
            "WAYLAND_DISPLAY",
            "XDG_SESSION_TYPE",
            "XDG_CURRENT_DESKTOP",
            "DESKTOP_SESSION",
        )
        env = {key: str(os.environ.get(key) or "").strip() for key in keys}
        if not env.get("DISPLAY") and not env.get("WAYLAND_DISPLAY"):
            return {}
        env = {key: value for key, value in env.items() if value}
        if "XAUTHORITY" not in env:
            xauthority = Path.home() / ".Xauthority"
            if xauthority.exists():
                env["XAUTHORITY"] = str(xauthority)
        return env

    def _graphical_session_env(self) -> dict[str, str]:
        command = (
            "for sid in $(loginctl list-sessions --no-legend 2>/dev/null | awk '{print $1}'); do "
            "loginctl show-session \"$sid\" -p Name -p User -p Remote -p State -p Type -p Class -p Leader; "
            "echo __SESSION_END__; "
            "done"
        )
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", command],
                cwd=self.workspace,
                text=True,
                capture_output=True,
                timeout=12,
                env=self._clean_script_env(),
                check=False,
            )
        except Exception:
            return {}

        current_user = Path.home().name
        for block in (segment.strip() for segment in result.stdout.split("__SESSION_END__")):
            if not block:
                continue
            fields: dict[str, str] = {}
            for line in block.splitlines():
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                fields[key.strip()] = value.strip()
            if fields.get("Name") != current_user:
                continue
            if fields.get("State") != "active":
                continue
            if fields.get("Class") != "user" or fields.get("Type") not in {"x11", "wayland"}:
                continue
            leader = fields.get("Leader", "").strip()
            if not leader:
                continue
            environ_path = Path("/proc") / leader / "environ"
            with contextlib.suppress(Exception):
                payload = environ_path.read_bytes().split(b"\0")
                session_env: dict[str, str] = {}
                for entry in payload:
                    if not entry or b"=" not in entry:
                        continue
                    key, value = entry.split(b"=", 1)
                    session_env[key.decode("utf-8", errors="replace")] = value.decode("utf-8", errors="replace")
                if session_env.get("DISPLAY") or session_env.get("WAYLAND_DISPLAY"):
                    return session_env
        fallback_command = "systemctl --user show-environment 2>/dev/null"
        fallback_env = self._clean_script_env()
        for key in ("XDG_RUNTIME_DIR", "DBUS_SESSION_BUS_ADDRESS"):
            value = str(os.environ.get(key) or "").strip()
            if value:
                fallback_env[key] = value
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", fallback_command],
                cwd=self.workspace,
                text=True,
                capture_output=True,
                timeout=8,
                env=fallback_env,
                check=False,
            )
        except Exception:
            return {}
        session_env = {}
        for line in result.stdout.splitlines():
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            if key not in {
                "DISPLAY",
                "XAUTHORITY",
                "XDG_RUNTIME_DIR",
                "DBUS_SESSION_BUS_ADDRESS",
                "WAYLAND_DISPLAY",
                "XDG_SESSION_TYPE",
                "XDG_CURRENT_DESKTOP",
                "DESKTOP_SESSION",
            }:
                continue
            value = value.strip()
            if value:
                session_env[key] = value
        if session_env.get("DISPLAY") or session_env.get("WAYLAND_DISPLAY"):
            return session_env
        return {}

    @staticmethod
    def _display_socket_exists(display_name: str) -> bool:
        name = str(display_name or "").strip()
        if not name:
            return False
        display_index = name.lstrip(":").split(".", 1)[0]
        if not display_index:
            return False
        return (Path("/tmp/.X11-unix") / f"X{display_index}").exists()

    def _graphical_candidate_status(self, source_env: dict[str, str], *, source: str, display: str = "") -> dict[str, Any]:
        merged = dict(self._fallback_graphical_env())
        for key, value in (source_env or {}).items():
            text = str(value or "").strip()
            if text:
                merged[key] = text
        if display:
            merged["DISPLAY"] = str(display).strip()
        merged["DISPLAY"] = str(merged.get("DISPLAY") or "").strip()
        merged["WAYLAND_DISPLAY"] = str(merged.get("WAYLAND_DISPLAY") or "").strip()
        merged["XAUTHORITY"] = str(merged.get("XAUTHORITY") or "").strip() or str(Path.home() / ".Xauthority")
        merged["XDG_RUNTIME_DIR"] = str(merged.get("XDG_RUNTIME_DIR") or "").strip() or self._ensure_rviz_runtime_dir()
        if not merged.get("DBUS_SESSION_BUS_ADDRESS"):
            merged["DBUS_SESSION_BUS_ADDRESS"] = f"unix:path={Path(merged['XDG_RUNTIME_DIR']) / 'bus'}"

        display_name = merged["DISPLAY"]
        wayland_display = merged["WAYLAND_DISPLAY"]
        xauthority_path = Path(merged["XAUTHORITY"]) if merged["XAUTHORITY"] else None
        xauthority_exists = bool(xauthority_path and xauthority_path.exists())
        runtime_dir_exists = Path(merged["XDG_RUNTIME_DIR"]).exists()
        display_exists = self._display_socket_exists(display_name)

        ready = False
        last_error = ""
        if not display_name and not wayland_display:
            last_error = "当前会话没有图形环境"
        elif not display_name and wayland_display:
            last_error = f"当前图形会话只有 Wayland ({wayland_display})，缺少 DISPLAY"
        elif display_name and not (xauthority_exists or display_exists):
            last_error = f"找到显示 {display_name}，但 XAUTHORITY / DISPLAY socket 不完整"
        else:
            ready = True

        return {
            "env": merged,
            "ready": ready,
            "display": display_name,
            "xauthority": merged["XAUTHORITY"],
            "xdg_runtime_dir": merged["XDG_RUNTIME_DIR"],
            "source": source,
            "last_error": last_error,
        }

    def _reset_rviz_launch_state(self) -> None:
        self._rviz_launch_requested = False
        self._rviz_launch_source = ""
        self._rviz_launch_target = ""
        self._rviz_launch_error = ""
        self._rviz_launch_warning = ""
        self._status_cached_at = 0.0

    def _set_rviz_launch_state(
        self,
        *,
        requested: bool,
        source: str = "",
        target: str = "",
        error: str = "",
        warning: str = "",
    ) -> None:
        self._rviz_launch_requested = bool(requested)
        self._rviz_launch_source = str(source or "").strip()
        self._rviz_launch_target = str(target or "").strip()
        self._rviz_launch_error = str(error or "").strip()
        self._rviz_launch_warning = str(warning or "").strip()
        self._status_cached_at = 0.0

    def _desktop_session_status(self, display: str = "") -> dict[str, Any]:
        current_env = self._current_process_graphical_env()
        session_env = self._graphical_session_env()
        current_status = (
            self._graphical_candidate_status(current_env, source="current_process", display=display)
            if current_env
            else {}
        )
        if current_status and current_status["ready"]:
            return current_status
        if current_status and current_status.get("display"):
            return current_status

        session_status = (
            self._graphical_candidate_status(session_env, source="session", display=display)
            if session_env
            else {}
        )
        if session_status and session_status["ready"]:
            return session_status
        if current_status:
            return current_status
        if session_status:
            return session_status
        status = self._graphical_candidate_status({}, source="none", display=display)
        status["ready"] = False
        status["last_error"] = "当前会话没有图形环境，且未检测到活动桌面会话"
        return status

    @staticmethod
    def _ready_error_code(message: str) -> str:
        text = str(message or "").strip().lower()
        if not text:
            return ""
        if "2d pose estimate" in text or "初始位姿" in text:
            return "initial_pose_required"
        if "控制串口" in text:
            return "control_port_selected"
        if "tf 未建立" in text or "laser 数据" in text or "laser 帧" in text:
            return "tf_missing"
        if "雷达有数据，但 slam_toolbox 未持续出图" in text:
            return "slam_no_map"
        if "/scan" in text and "未收到" in text:
            return "scan_not_fresh"
        if "/map" in text and "未收到" in text:
            return "map_not_fresh"
        if "未检测到雷达串口" in text or "lidar port not found" in text:
            return "lidar_port_missing"
        if "链路不完整" in text:
            return "mapping_chain_incomplete"
        if "rviz" in text and "display" in text:
            return "rviz_display_unavailable"
        return "mapping_not_ready"

    def _rviz_diagnostics(self) -> dict[str, Any]:
        live = self._pid_file_running("rviz2")
        mode_info = self._read_rviz_mode_info()
        render_mode = str(mode_info.get("render_mode") or "").strip()
        if not render_mode:
            render_mode = "unknown" if live else self._rviz_default_render_mode

        if not live:
            return {
                "live": False,
                "render_mode": render_mode,
                "error_code": "",
                "last_error": "",
            }

        error_code = ""
        last_error = ""
        for line in reversed(self._rviz_log_tail(lines=120)):
            lowered = line.lower()
            if "pluginlibfactory" in lowered and "rviz_common/time" in lowered:
                error_code = "config_plugin"
                last_error = "RViz 配置含不兼容的 Time 面板"
                break
            if "glsl link result" in lowered or "indexed_8bit_image" in lowered:
                error_code = "render_gl"
                last_error = "RViz 地图渲染异常，当前图形栈与硬件渲染不兼容"
                break
            if "message filter dropping message" in lowered and "frame 'laser'" in lowered and "unknown" in lowered:
                error_code = "tf_wait"
                last_error = "RViz 已打开，但仍在等待 laser TF/首帧"
                break
        return {
            "live": live,
            "render_mode": render_mode,
            "error_code": error_code,
            "last_error": last_error,
        }

    def _log_ready_error_hint(self) -> str:
        for line in reversed(self._log_tail("sllidar_driver", lines=120)):
            lowered = line.lower()
            if "sl_result_operation_timeout" in lowered or "operation time out" in lowered:
                return "雷达驱动启动失败: sllidar 串口通信超时，请检查雷达供电、串口和波特率"
            if "exit code 255" in lowered and "sllidar" in lowered:
                return "雷达驱动已退出: sllidar_node 启动失败"
        for line in reversed(self._log_tail("slam_toolbox", lines=80)):
            lowered = line.lower()
            if "message filter dropping message" in lowered and "frame 'laser'" in lowered and "unknown" in lowered:
                return "TF 未建立/雷达帧未接入，slam_toolbox 正在丢弃 laser 数据"
        for line in reversed(self._log_tail("nav2_bringup", lines=120)):
            lowered = line.lower()
            if "yaml_filename" in lowered and "empty" in lowered:
                return "定位导航启动失败: map_server 未拿到有效地图文件"
            if "amcl" in lowered and "error" in lowered:
                return "AMCL 启动异常，请检查地图文件、里程计和 laser TF"
            if "bt_navigator" in lowered and "error" in lowered:
                return "Nav2 行为树导航器启动异常"
        return ""

    def _session_frame_seen(self, frames: int, last_at: float, session_started_at: float | None = None) -> bool:
        started_at = self._mapping_session_started_at if session_started_at is None else float(session_started_at)
        return bool(started_at and frames > 0 and last_at and last_at >= started_at)

    def _session_frame_fresh(
        self,
        frames: int,
        last_at: float,
        *,
        now: float | None = None,
        session_started_at: float | None = None,
    ) -> bool:
        current = time.time() if now is None else float(now)
        return self._session_frame_seen(frames, last_at, session_started_at) and current - last_at < self._stream_window_s

    def _script(self, name: str) -> Path:
        return self.ros_tools_dir / name

    def _map_yaml_path(self, map_name: str) -> Path:
        return self.maps_dir / f"{Path(map_name).stem}.yaml"

    def _map_image_path_from_yaml(self, yaml_path: Path) -> Path | None:
        if not yaml_path.exists():
            return None
        with contextlib.suppress(Exception):
            for line in yaml_path.read_text(encoding="utf-8", errors="replace").splitlines():
                if not line.startswith("image:"):
                    continue
                relative = line.split(":", 1)[1].strip()
                if not relative:
                    break
                candidate = (yaml_path.parent / relative).resolve()
                maps_root = self.maps_dir.resolve()
                if candidate.exists() and (candidate == maps_root or maps_root in candidate.parents):
                    return candidate
                break
        for suffix in (".pgm", ".png"):
            candidate = yaml_path.with_suffix(suffix)
            if candidate.exists():
                return candidate
        return None

    def map_image_path(self, map_name: str) -> Path | None:
        return self._map_image_path_from_yaml(self._map_yaml_path(map_name))

    def map_artifact_path(self, map_name: str, artifact: str) -> Path | None:
        yaml_path = self._map_yaml_path(map_name)
        if artifact == "yaml":
            return yaml_path if yaml_path.exists() else None
        if artifact == "image":
            return self._map_image_path_from_yaml(yaml_path)
        return None

    def list_saved_maps(self, limit: int = 8) -> list[dict[str, Any]]:
        entries: list[dict[str, Any]] = []
        yaml_paths = sorted(
            self.maps_dir.glob("*.yaml"),
            key=lambda path: path.stat().st_mtime if path.exists() else 0.0,
            reverse=True,
        )
        for yaml_path in yaml_paths:
            image_path = self._map_image_path_from_yaml(yaml_path)
            total_bytes = yaml_path.stat().st_size
            if image_path is not None and image_path.exists():
                total_bytes += image_path.stat().st_size
            entries.append(
                {
                    "name": yaml_path.stem,
                    "yaml_file": yaml_path.name,
                    "image_file": image_path.name if image_path else "",
                    "has_image": image_path is not None and image_path.exists(),
                    "updated_at": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(yaml_path.stat().st_mtime)),
                    "updated_ts": int(yaml_path.stat().st_mtime),
                    "size_text": f"{total_bytes / 1024.0:.1f} KB",
                }
            )
            if limit and len(entries) >= limit:
                break
        return entries

    def saved_maps_count(self) -> int:
        return len(list(self.maps_dir.glob("*.yaml")))

    @staticmethod
    def _port_exists(port: str) -> bool:
        return bool(port) and Path(port).exists()

    @staticmethod
    def _clean_script_env() -> dict[str, str]:
        env: dict[str, str] = {}
        for key in ("HOME", "USER", "LOGNAME", "SHELL", "PATH", "LANG", "LC_ALL", "LC_CTYPE", "TERM"):
            value = os.environ.get(key, "").strip()
            if value:
                env[key] = value
        env.setdefault("HOME", str(Path.home()))
        env.setdefault("USER", Path.home().name)
        env.setdefault("LOGNAME", env["USER"])
        env.setdefault("SHELL", "/bin/bash")
        env.setdefault("PATH", "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin")
        env["PYTHONPATH"] = ""
        env["ROS_LOCALHOST_ONLY"] = LOCAL_ROS_LOCALHOST_ONLY
        env["CAR2_IMU_MODE"] = os.environ.get("CAR2_IMU_MODE", CAR2_IMU_MODE_REQUIRED)
        env["CAR2_IMU_PORT"] = os.environ.get("CAR2_IMU_PORT", DEFAULT_IMU_PORT)
        env["CAR2_IMU_BAUDRATE"] = os.environ.get("CAR2_IMU_BAUDRATE", str(DEFAULT_IMU_BAUDRATE))
        env["CAR2_IMU_FRAME"] = os.environ.get("CAR2_IMU_FRAME", DEFAULT_IMU_FRAME)
        return env

    def _run_script(self, script_name: str, *args: str, timeout: int = 120) -> subprocess.CompletedProcess[str]:
        script = self._script(script_name)
        if not script.exists():
            raise FileNotFoundError(f"未找到脚本: {script}")
        command = ["/bin/bash", str(script), *args]
        return subprocess.run(
            command,
            cwd=self.workspace,
            text=True,
            capture_output=True,
            timeout=timeout,
            env=self._clean_script_env(),
            check=False,
        )

    def _wifi_probe_enabled(self) -> bool:
        return self.wifi_probe_script.exists()

    def _update_wifi_probe_summary(self, summary: dict[str, Any] | None, error: str = "") -> dict[str, Any]:
        now = time.time()
        with self._wifi_probe_lock:
            if summary:
                self._wifi_probe_last_summary = dict(summary)
                self._wifi_probe_last_sample_at = now
                self._wifi_probe_last_error = error.strip()
            elif error:
                self._wifi_probe_last_error = error.strip()
        self._status_cached_at = 0.0
        return dict(summary or {})

    def _wifi_probe_snapshot(
        self,
        label: str,
        *,
        phase: str = "",
        note: str = "",
        log_errors: bool = True,
        timeout: int = 24,
    ) -> dict[str, Any]:
        if not self._wifi_probe_enabled():
            return self._update_wifi_probe_summary({}, error=f"未找到 Wi-Fi probe 脚本: {self.wifi_probe_script}")
        self.ros_log_dir.mkdir(parents=True, exist_ok=True)
        command = [
            sys.executable or "python3",
            str(self.wifi_probe_script),
            "--log-file",
            str(self.wifi_probe_log_path),
            "--state-file",
            str(self.wifi_probe_state_path),
            "--label",
            str(label),
        ]
        if phase:
            command.extend(["--phase", str(phase)])
        if note:
            command.extend(["--note", str(note)])
        try:
            result = subprocess.run(
                command,
                cwd=self.workspace,
                text=True,
                capture_output=True,
                timeout=timeout,
                env=self._clean_script_env(),
                check=False,
            )
        except Exception as exc:
            error = str(exc)
            if log_errors:
                self.logger.log(f"Wi-Fi probe 快照失败[{label}]: {error}")
            return self._update_wifi_probe_summary({}, error=error)

        summary: dict[str, Any] = {}
        payload_text = (result.stdout or "").strip()
        if payload_text:
            with contextlib.suppress(Exception):
                decoded = json.loads(payload_text.splitlines()[-1])
                if isinstance(decoded, dict):
                    summary = decoded
        if result.returncode == 0:
            return self._update_wifi_probe_summary(summary, error="")

        error = (result.stderr or payload_text or f"wifi probe exited {result.returncode}").strip()
        if log_errors:
            self.logger.log(f"Wi-Fi probe 快照失败[{label}]: {error}")
        return self._update_wifi_probe_summary(summary, error=error)

    def _wifi_probe_worker(self) -> None:
        while not self._wifi_probe_stop_event.wait(self._wifi_probe_interval_s):
            session_name = ""
            with self._wifi_probe_lock:
                session_name = self._wifi_probe_session or "idle"
            self._wifi_probe_snapshot(
                f"{session_name}_steady",
                phase="steady_state",
                note="periodic_session_sample",
                log_errors=False,
            )

    def _start_wifi_probe(self, session_name: str) -> None:
        if not self._wifi_probe_enabled():
            self._update_wifi_probe_summary({}, error=f"未找到 Wi-Fi probe 脚本: {self.wifi_probe_script}")
            return
        self.ros_log_dir.mkdir(parents=True, exist_ok=True)
        with self._wifi_probe_lock:
            self._wifi_probe_session = str(session_name or "").strip() or "idle"
            thread = self._wifi_probe_thread
            if thread is not None and thread.is_alive():
                self._status_cached_at = 0.0
                return
            self._wifi_probe_stop_event = threading.Event()
            self._wifi_probe_thread = threading.Thread(
                target=self._wifi_probe_worker,
                name="wifi-runtime-probe",
                daemon=True,
            )
            self._wifi_probe_thread.start()
        self._status_cached_at = 0.0

    def _stop_wifi_probe(self, reason: str = "") -> None:
        session_name = ""
        thread: threading.Thread | None = None
        with self._wifi_probe_lock:
            session_name = str(self._wifi_probe_session or "").strip()
            thread = self._wifi_probe_thread
            self._wifi_probe_stop_event.set()
            self._wifi_probe_thread = None
            self._wifi_probe_session = ""
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        if session_name and self._wifi_probe_enabled():
            self._wifi_probe_snapshot(
                f"{session_name}_session_stopped",
                phase="session_stop",
                note=reason or f"{session_name} stop requested",
                log_errors=False,
            )
        self._status_cached_at = 0.0

    @staticmethod
    def _command_output_text(result: subprocess.CompletedProcess[str]) -> str:
        parts = [part.strip() for part in (result.stdout, result.stderr) if part and part.strip()]
        return "\n".join(parts)

    def _ensure_ok(self, action: str, result: subprocess.CompletedProcess[str]):
        if result.returncode == 0:
            output = self._command_output_text(result)
            if output:
                self.logger.log(f"{action}: {output.splitlines()[-1]}")
            return
        message = self._command_output_text(result) or f"{action}失败，退出码 {result.returncode}"
        raise RuntimeError(message)

    def _probe_odom_topic_once(self, topic_name: str = "/odom", timeout_s: float | None = None) -> tuple[bool, str]:
        if not self.ros_setup.exists():
            return False, f"未找到 ROS2 环境: {self.ros_setup}"
        if not self.ros_probe_script.exists():
            return False, f"未找到 ROS probe 脚本: {self.ros_probe_script}"
        probe_timeout = max(0.5, float(timeout_s or self._odom_probe_timeout_s))
        command = (
            f"export ROS_LOCALHOST_ONLY={LOCAL_ROS_LOCALHOST_ONLY}; "
            "unset LD_PRELOAD; "
            "source /opt/ros/foxy/setup.bash >/dev/null 2>&1; "
            f"[ -f '{self.rock_ws / 'install' / 'setup.bash'}' ] && source '{self.rock_ws / 'install' / 'setup.bash'}' >/dev/null 2>&1 || true; "
            f"python3 '{self.ros_probe_script}' odom --topic {shlex.quote(topic_name)} --timeout {probe_timeout:.2f}"
        )
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", command],
                text=True,
                capture_output=True,
                timeout=max(2, int(math.ceil(probe_timeout)) + 2),
                env=self._clean_script_env(),
                check=False,
            )
        except Exception as exc:
            return False, str(exc)
        if result.returncode == 0:
            return True, (result.stdout or result.stderr or f"{topic_name} message received").strip()
        error_text = (result.stderr or result.stdout or "").strip() or f"odom probe failed ({result.returncode})"
        return False, error_text

    def _probe_imu_topic_once(self, topic_name: str = "/imu/data", timeout_s: float | None = None) -> tuple[bool, str]:
        if not self.ros_setup.exists():
            return False, f"未找到 ROS2 环境: {self.ros_setup}"
        if not self.ros_probe_script.exists():
            return False, f"未找到 ROS probe 脚本: {self.ros_probe_script}"
        probe_timeout = max(0.5, float(timeout_s or self._imu_filtered_probe_timeout_s))
        command = (
            f"export ROS_LOCALHOST_ONLY={LOCAL_ROS_LOCALHOST_ONLY}; "
            "unset LD_PRELOAD; "
            "source /opt/ros/foxy/setup.bash >/dev/null 2>&1; "
            f"[ -f '{self.rock_ws / 'install' / 'setup.bash'}' ] && source '{self.rock_ws / 'install' / 'setup.bash'}' >/dev/null 2>&1 || true; "
            f"python3 '{self.ros_probe_script}' imu --topic {shlex.quote(topic_name)} --timeout {probe_timeout:.2f}"
        )
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", command],
                text=True,
                capture_output=True,
                timeout=max(2, int(math.ceil(probe_timeout)) + 2),
                env=self._clean_script_env(),
                check=False,
            )
        except Exception as exc:
            return False, str(exc)
        if result.returncode == 0:
            return True, (result.stdout or result.stderr or f"{topic_name} message received").strip()
        error_text = (result.stderr or result.stdout or "").strip() or f"imu probe failed ({result.returncode})"
        return False, error_text

    def _imu_raw_probe_fresh(self, *, now: float | None = None) -> bool:
        current = time.time() if now is None else float(now)
        with self._imu_raw_probe_lock:
            last_ok_at = self._imu_raw_probe_last_ok_at
        return bool(last_ok_at and current - last_ok_at < self._stream_window_s)

    def _imu_filtered_probe_fresh(self, *, now: float | None = None) -> bool:
        current = time.time() if now is None else float(now)
        with self._imu_filtered_probe_lock:
            last_ok_at = self._imu_filtered_probe_last_ok_at
        return bool(last_ok_at and current - last_ok_at < self._stream_window_s)

    def _refresh_imu_raw_probe_async(self, *, force: bool = False):
        if not self.ros_probe_script.exists():
            return
        now = time.time()
        with self._imu_raw_probe_lock:
            if self._imu_raw_probe_inflight:
                return
            if not force and now - self._imu_raw_probe_last_attempt_at < self._imu_raw_probe_interval_s:
                return
            self._imu_raw_probe_inflight = True
            self._imu_raw_probe_last_attempt_at = now

        def _worker():
            ok, message = self._probe_imu_topic_once("/imu/data_raw", self._imu_raw_probe_timeout_s)
            finished_at = time.time()
            with self._imu_raw_probe_lock:
                if ok:
                    self._imu_raw_probe_last_ok_at = finished_at
                    self._imu_raw_probe_last_error = ""
                else:
                    self._imu_raw_probe_last_error = message
                self._imu_raw_probe_inflight = False

        threading.Thread(target=_worker, daemon=True).start()

    def _refresh_imu_filtered_probe_async(self, *, force: bool = False):
        if not self.ros_probe_script.exists():
            return
        now = time.time()
        with self._imu_filtered_probe_lock:
            if self._imu_filtered_probe_inflight:
                return
            if not force and now - self._imu_filtered_probe_last_attempt_at < self._imu_filtered_probe_interval_s:
                return
            self._imu_filtered_probe_inflight = True
            self._imu_filtered_probe_last_attempt_at = now

        def _worker():
            ok, message = self._probe_imu_topic_once("/imu/data", self._imu_filtered_probe_timeout_s)
            finished_at = time.time()
            with self._imu_filtered_probe_lock:
                if ok:
                    self._imu_filtered_probe_last_ok_at = finished_at
                    self._imu_filtered_probe_last_error = ""
                else:
                    self._imu_filtered_probe_last_error = message
                self._imu_filtered_probe_inflight = False

        threading.Thread(target=_worker, daemon=True).start()

    def _odom_probe_fresh(self, *, now: float | None = None) -> bool:
        current = time.time() if now is None else float(now)
        with self._odom_probe_lock:
            last_ok_at = self._odom_probe_last_ok_at
        return bool(last_ok_at and current - last_ok_at < self._stream_window_s)

    def _odom_probe_age_ms(self, *, now: float | None = None) -> int | None:
        current = time.time() if now is None else float(now)
        with self._odom_probe_lock:
            last_ok_at = self._odom_probe_last_ok_at
        if not last_ok_at:
            return None
        return max(0, int((current - last_ok_at) * 1000.0))

    def _refresh_odom_probe_async(self, *, force: bool = False):
        if not self.ros_probe_script.exists():
            return
        now = time.time()
        with self._odom_probe_lock:
            if self._odom_probe_inflight:
                return
            if not force and now - self._odom_probe_last_attempt_at < self._odom_probe_interval_s:
                return
            self._odom_probe_inflight = True
            self._odom_probe_last_attempt_at = now

        def _worker():
            ok, message = self._probe_odom_topic_once("/odom")
            finished_at = time.time()
            with self._odom_probe_lock:
                if ok:
                    self._odom_probe_last_ok_at = finished_at
                    self._odom_probe_last_error = ""
                else:
                    self._odom_probe_last_error = message
                self._odom_probe_inflight = False

        threading.Thread(target=_worker, daemon=True).start()

    def _probe_scan_topic_once(self, timeout_s: float | None = None) -> tuple[bool, str]:
        if not self.ros_setup.exists():
            return False, f"未找到 ROS2 环境: {self.ros_setup}"
        if not self.ros_probe_script.exists():
            return False, f"未找到 ROS probe 脚本: {self.ros_probe_script}"
        probe_timeout = max(0.5, float(timeout_s or self._scan_probe_timeout_s))
        command = (
            f"export ROS_LOCALHOST_ONLY={LOCAL_ROS_LOCALHOST_ONLY}; "
            "unset LD_PRELOAD; "
            "source /opt/ros/foxy/setup.bash >/dev/null 2>&1; "
            f"[ -f '{self.rock_ws / 'install' / 'setup.bash'}' ] && source '{self.rock_ws / 'install' / 'setup.bash'}' >/dev/null 2>&1 || true; "
            f"python3 '{self.ros_probe_script}' scan --timeout {probe_timeout:.2f}"
        )
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", command],
                text=True,
                capture_output=True,
                timeout=max(2, int(math.ceil(probe_timeout)) + 2),
                env=self._clean_script_env(),
                check=False,
            )
        except Exception as exc:
            return False, str(exc)
        if result.returncode == 0:
            return True, (result.stdout or result.stderr or "/scan message received").strip()
        error_text = (result.stderr or result.stdout or "").strip() or f"scan probe failed ({result.returncode})"
        return False, error_text

    def _scan_probe_fresh(self, *, now: float | None = None) -> bool:
        current = time.time() if now is None else float(now)
        with self._scan_probe_lock:
            last_ok_at = self._scan_probe_last_ok_at
        return bool(last_ok_at and current - last_ok_at < self._stream_window_s)

    def _scan_probe_age_ms(self, *, now: float | None = None) -> int | None:
        current = time.time() if now is None else float(now)
        with self._scan_probe_lock:
            last_ok_at = self._scan_probe_last_ok_at
        if not last_ok_at:
            return None
        return max(0, int((current - last_ok_at) * 1000.0))

    def _refresh_scan_probe_async(self, *, force: bool = False):
        if not self.ros_probe_script.exists():
            return
        now = time.time()
        with self._scan_probe_lock:
            if self._scan_probe_inflight:
                return
            if not force and now - self._scan_probe_last_attempt_at < self._scan_probe_interval_s:
                return
            self._scan_probe_inflight = True
            self._scan_probe_last_attempt_at = now

        def _worker():
            ok, message = self._probe_scan_topic_once()
            finished_at = time.time()
            with self._scan_probe_lock:
                if ok:
                    self._scan_probe_last_ok_at = finished_at
                    self._scan_probe_last_error = ""
                else:
                    self._scan_probe_last_error = message
                self._scan_probe_inflight = False

        threading.Thread(target=_worker, daemon=True).start()

    def _probe_map_topic_once(self, timeout_s: float | None = None) -> tuple[bool, str]:
        if not self.ros_setup.exists():
            return False, f"未找到 ROS2 环境: {self.ros_setup}"
        probe_timeout = max(0.5, float(timeout_s or self._map_probe_timeout_s))
        python_code = """
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapWaiter(Node):
    def __init__(self, *, context: Context):
        super().__init__("car2_map_probe", context=context)
        self.received = False
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_msg, qos)

    def _on_msg(self, _message: OccupancyGrid):
        self.received = True


context = Context()
context.init(args=None)
node = MapWaiter(context=context)
executor = SingleThreadedExecutor(context=context)
executor.add_node(node)
deadline = time.time() + {timeout_s}
try:
    while context.ok() and time.time() < deadline and not node.received:
        remaining = max(0.05, min(0.2, deadline - time.time()))
        executor.spin_once(timeout_sec=remaining)
finally:
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown(context=context)

if node.received:
    print("/map message received")
    raise SystemExit(0)
print("timeout waiting for /map message", file=sys.stderr)
raise SystemExit(1)
""".strip().format(timeout_s=f"{probe_timeout:.2f}")
        command = (
            f"export ROS_LOCALHOST_ONLY={LOCAL_ROS_LOCALHOST_ONLY}; "
            "unset LD_PRELOAD; "
            "source /opt/ros/foxy/setup.bash >/dev/null 2>&1; "
            f"[ -f '{self.rock_ws / 'install' / 'setup.bash'}' ] && source '{self.rock_ws / 'install' / 'setup.bash'}' >/dev/null 2>&1 || true; "
            f"python3 -c {shlex.quote(python_code)}"
        )
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", command],
                text=True,
                capture_output=True,
                timeout=max(2, int(math.ceil(probe_timeout)) + 2),
                env=self._clean_script_env(),
                check=False,
            )
        except Exception as exc:
            return False, str(exc)
        if result.returncode == 0:
            return True, (result.stdout or result.stderr or "/map message received").strip()
        error_text = (result.stderr or result.stdout or "").strip() or f"map probe failed ({result.returncode})"
        return False, error_text

    def _probe_tf_once(self, target_frame: str, source_frame: str, timeout_s: float) -> tuple[bool, str]:
        if not self.ros_setup.exists():
            return False, f"未找到 ROS2 环境: {self.ros_setup}"
        if not self.ros_probe_script.exists():
            return False, f"未找到 ROS probe 脚本: {self.ros_probe_script}"
        probe_timeout = max(0.5, float(timeout_s))
        command = (
            f"export ROS_LOCALHOST_ONLY={LOCAL_ROS_LOCALHOST_ONLY}; "
            "unset LD_PRELOAD; "
            "source /opt/ros/foxy/setup.bash >/dev/null 2>&1; "
            f"[ -f '{self.rock_ws / 'install' / 'setup.bash'}' ] && source '{self.rock_ws / 'install' / 'setup.bash'}' >/dev/null 2>&1 || true; "
            f"python3 '{self.ros_probe_script}' tf --target {shlex.quote(target_frame)} --source {shlex.quote(source_frame)} --timeout {probe_timeout:.2f}"
        )
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", command],
                text=True,
                capture_output=True,
                timeout=max(2, int(math.ceil(probe_timeout)) + 2),
                env=self._clean_script_env(),
                check=False,
            )
        except Exception as exc:
            return False, str(exc)
        if result.returncode == 0:
            return True, (result.stdout or result.stderr or f"tf ready: {target_frame} -> {source_frame}").strip()
        error_text = (result.stderr or result.stdout or "").strip() or f"tf probe failed ({result.returncode})"
        return False, error_text

    def _session_probe_fresh(
        self,
        last_ok_at: float,
        *,
        now: float | None = None,
        session_started_at: float | None = None,
        sticky: bool = False,
    ) -> bool:
        started_at = float(session_started_at or 0.0)
        if not last_ok_at or not started_at or last_ok_at < started_at:
            return False
        if sticky:
            return True
        current = time.time() if now is None else float(now)
        return current - last_ok_at < self._stream_window_s

    def _map_probe_fresh(
        self,
        *,
        now: float | None = None,
        session_started_at: float | None = None,
        sticky: bool = False,
    ) -> bool:
        with self._map_probe_lock:
            last_ok_at = self._map_probe_last_ok_at
        return self._session_probe_fresh(
            last_ok_at,
            now=now,
            session_started_at=session_started_at,
            sticky=sticky,
        )

    def _map_probe_age_ms(self, *, now: float | None = None) -> int | None:
        current = time.time() if now is None else float(now)
        with self._map_probe_lock:
            last_ok_at = self._map_probe_last_ok_at
        if not last_ok_at:
            return None
        return max(0, int((current - last_ok_at) * 1000.0))

    def _refresh_map_probe_async(self, *, force: bool = False):
        now = time.time()
        with self._map_probe_lock:
            if self._map_probe_inflight:
                return
            if not force and now - self._map_probe_last_attempt_at < self._map_probe_interval_s:
                return
            self._map_probe_inflight = True
            self._map_probe_last_attempt_at = now

        def _worker():
            ok, message = self._probe_map_topic_once()
            finished_at = time.time()
            with self._map_probe_lock:
                if ok:
                    self._map_probe_last_ok_at = finished_at
                    self._map_probe_last_error = ""
                else:
                    self._map_probe_last_error = message
                self._map_probe_inflight = False

        threading.Thread(target=_worker, daemon=True).start()

    def _map_to_odom_tf_fresh(self, *, now: float | None = None) -> bool:
        with self._map_to_odom_tf_lock:
            last_ok_at = self._map_to_odom_tf_last_ok_at
        return self._session_probe_fresh(
            last_ok_at,
            now=now,
            session_started_at=self._localization_session_started_at,
        )

    def _map_to_odom_tf_age_ms(self, *, now: float | None = None) -> int | None:
        current = time.time() if now is None else float(now)
        with self._map_to_odom_tf_lock:
            last_ok_at = self._map_to_odom_tf_last_ok_at
        if not last_ok_at:
            return None
        return max(0, int((current - last_ok_at) * 1000.0))

    def _refresh_map_to_odom_tf_probe_async(self, *, force: bool = False):
        now = time.time()
        with self._map_to_odom_tf_lock:
            if self._map_to_odom_tf_inflight:
                return
            if not force and now - self._map_to_odom_tf_last_attempt_at < self._map_to_odom_tf_interval_s:
                return
            self._map_to_odom_tf_inflight = True
            self._map_to_odom_tf_last_attempt_at = now

        def _worker():
            ok, message = self._probe_tf_once("map", "odom", self._map_to_odom_tf_timeout_s)
            finished_at = time.time()
            with self._map_to_odom_tf_lock:
                if ok:
                    self._map_to_odom_tf_last_ok_at = finished_at
                    self._map_to_odom_tf_last_error = ""
                else:
                    self._map_to_odom_tf_last_error = message
                self._map_to_odom_tf_inflight = False

        threading.Thread(target=_worker, daemon=True).start()

    def _mapping_ready(self, ros_status: dict[str, Any]) -> bool:
        return all(
            (
                ros_status.get("imu_driver_ready"),
                ros_status.get("imu_filtered_live"),
                ros_status.get("imu_tf_live"),
                ros_status.get("ekf_ready"),
                ros_status.get("driver_live"),
                ros_status.get("slam_live"),
                ros_status.get("laser_tf_live"),
                ros_status.get("odom_tf_live"),
                ros_status.get("scan_fresh"),
                ros_status.get("map_fresh"),
            )
        )

    def _nav_stack_live(self, ros_status: dict[str, Any]) -> bool:
        if "nav_stack_live" in ros_status:
            return bool(ros_status.get("nav_stack_live"))
        return all(
            (
                ros_status.get("imu_driver_ready"),
                ros_status.get("imu_filtered_live"),
                ros_status.get("imu_tf_live"),
                ros_status.get("ekf_ready"),
                ros_status.get("driver_live"),
                ros_status.get("laser_tf_live"),
                ros_status.get("odom_tf_live"),
                ros_status.get("map_server_live"),
                ros_status.get("amcl_live"),
                ros_status.get("nav2_live"),
                ros_status.get("scan_fresh"),
            )
        )

    def _rviz_map_ready(self, ros_status: dict[str, Any]) -> bool:
        if "rviz_map_ready" in ros_status:
            return bool(ros_status.get("rviz_map_ready"))
        return bool(
            ros_status.get("rviz_live")
            and ros_status.get("map_fresh")
            and ros_status.get("scan_fresh")
            and ros_status.get("laser_tf_live")
            and ros_status.get("odom_tf_live")
            and str(ros_status.get("rviz_error_code") or "").strip().lower() not in {"config_plugin", "render_gl"}
        )

    def _localized_ready(self, ros_status: dict[str, Any]) -> bool:
        if "localized_ready" in ros_status:
            return bool(ros_status.get("localized_ready"))
        return bool(self._nav_stack_live(ros_status) and ros_status.get("map_to_odom_tf_live"))

    def _localization_ready(self, ros_status: dict[str, Any]) -> bool:
        return self._localized_ready(ros_status)

    def _derive_ready_error(self, ros_status: dict[str, Any]) -> str:
        log_hint = self._log_ready_error_hint()
        if log_hint:
            return log_hint
        if self._is_control_port(ros_status.get("last_lidar_port", "")):
            return f"当前所选串口 {ros_status.get('last_lidar_port', '--')} 实际上是控制串口"
        if not ros_status.get("lidar_port_present", True):
            return f"未检测到雷达串口: {ros_status.get('last_lidar_port', '--')}"
        missing: list[str] = []
        if self._imu_mode() == CAR2_IMU_MODE_REQUIRED:
            if not ros_status.get("imu_port_present", True):
                missing.append(f"未检测到 IMU 串口: {ros_status.get('imu_port', self._imu_port())}")
            elif not ros_status.get("imu_driver_ready"):
                missing.append(f"IMU 串口存在，但 IMU driver 未就绪: {ros_status.get('imu_port', self._imu_port())}")
            if not ros_status.get("imu_filtered_live"):
                missing.append("/imu/data 未就绪")
            if not ros_status.get("imu_tf_live"):
                missing.append("base_link -> imu_link TF 未就绪")
            if not ros_status.get("ekf_ready"):
                missing.append("EKF /odom 未就绪")
        if not ros_status.get("driver_live"):
            missing.append("sllidar_driver 未运行")
        if not ros_status.get("odom_tf_live"):
            missing.append("odom -> base_link TF 未就绪")
        if not ros_status.get("laser_tf_live"):
            missing.append("base_link -> laser TF 未就绪")
        if not ros_status.get("slam_live"):
            missing.append("slam_toolbox 未运行")
        if missing:
            return "建图链路不完整: " + "，".join(missing)
        if ros_status.get("scan_fresh") and not ros_status.get("map_fresh"):
            return "雷达有数据，但 slam_toolbox 未持续出图"
        if not ros_status.get("scan_fresh"):
            return "当前建图会话仍未收到新的 /scan 帧"
        if not ros_status.get("map_fresh"):
            return "当前建图会话仍未收到新的 /map 帧"
        return ""

    def _wait_ready(self, session_started_at: float, timeout_s: float = 30.0) -> dict[str, Any]:
        self._mapping_session_started_at = float(session_started_at)
        deadline = time.time() + timeout_s
        latest = self.status()
        while time.time() < deadline:
            self._status_cached_at = 0.0
            latest = self.status()
            if self._mapping_ready(latest):
                self._clear_ready_error()
                return latest
            time.sleep(1.0)
        self._set_ready_error(self._derive_ready_error(latest))
        return latest

    def _derive_localization_error(self, ros_status: dict[str, Any]) -> str:
        if self._is_control_port(ros_status.get("last_lidar_port", "")):
            return f"当前所选串口 {ros_status.get('last_lidar_port', '--')} 实际上是控制串口"
        if not ros_status.get("lidar_port_present", True):
            return f"未检测到雷达串口: {ros_status.get('last_lidar_port', '--')}"
        if not ros_status.get("localization_map"):
            return "当前未指定定位地图"
        if self._nav_stack_live(ros_status) and ros_status.get("rviz_map_ready") and not ros_status.get("localized_ready"):
            return "导航栈与地图显示已就绪，等待在 RViz 中使用 2D Pose Estimate 设置初始位姿"
        missing: list[str] = []
        if self._imu_mode() == CAR2_IMU_MODE_REQUIRED:
            if not ros_status.get("imu_port_present", True):
                missing.append(f"未检测到 IMU 串口: {ros_status.get('imu_port', self._imu_port())}")
            elif not ros_status.get("imu_driver_ready"):
                missing.append(f"IMU 串口存在，但 IMU driver 未就绪: {ros_status.get('imu_port', self._imu_port())}")
            if not ros_status.get("imu_filtered_live"):
                missing.append("/imu/data 未就绪")
            if not ros_status.get("imu_tf_live"):
                missing.append("base_link -> imu_link TF 未就绪")
            if not ros_status.get("ekf_ready"):
                missing.append("EKF /odom 未就绪")
        if not ros_status.get("driver_live"):
            missing.append("sllidar_driver 未运行")
        if not ros_status.get("laser_tf_live"):
            missing.append("base_link -> laser TF 未就绪")
        if not ros_status.get("odom_tf_live"):
            missing.append("odom -> base_link TF 未就绪")
        if not ros_status.get("map_server_live"):
            missing.append("map_server 未运行")
        if not ros_status.get("amcl_live"):
            missing.append("AMCL 未运行")
        if not ros_status.get("nav2_live"):
            missing.append("Nav2 未运行")
        if missing:
            return "定位导航链路不完整: " + "，".join(missing)
        if not ros_status.get("map_fresh"):
            preview_error = str(ros_status.get("preview_error") or "").strip()
            map_probe_error = str(ros_status.get("map_probe_error") or "").strip()
            if preview_error and map_probe_error:
                return f"当前定位会话 /map 预览异常，且探针未通过: {map_probe_error}"
            if preview_error:
                return f"当前定位会话 /map 预览异常: {preview_error}"
            if ros_status.get("map_topic_ready") and map_probe_error:
                return f"当前定位会话 /map 探针未通过: {map_probe_error}"
            return "当前定位会话仍未收到可用于显示的 /map 数据"
        if not ros_status.get("rviz_map_ready"):
            return "地图或定位显示链尚未就绪，请检查 /map、/scan 和 TF"
        if not ros_status.get("scan_fresh"):
            preview_error = str(ros_status.get("preview_error") or "").strip()
            scan_probe_error = str(ros_status.get("scan_probe_error") or "").strip()
            if preview_error and scan_probe_error:
                return f"当前定位会话 /scan 预览异常，且探针未通过: {scan_probe_error}"
            if preview_error:
                return f"当前定位会话 /scan 预览异常: {preview_error}"
            if ros_status.get("scan_topic_ready") and scan_probe_error:
                return f"当前定位会话 /scan 探针未通过: {scan_probe_error}"
            return "当前定位会话仍未收到新的 /scan 帧"
        if not ros_status.get("map_to_odom_tf_live"):
            return "导航栈与地图显示已就绪，等待在 RViz 中使用 2D Pose Estimate 设置初始位姿"
        return ""

    def _wait_localization_ready(self, session_started_at: float, timeout_s: float = 35.0) -> dict[str, Any]:
        self._localization_session_started_at = float(session_started_at)
        deadline = time.time() + timeout_s
        latest = self.status()
        while time.time() < deadline:
            self._status_cached_at = 0.0
            latest = self.status()
            if self._localization_ready(latest):
                self._clear_ready_error()
                return latest
            time.sleep(1.0)
        self._set_ready_error(self._derive_localization_error(latest))
        return latest

    def _wait_localization_start_ready(self, session_started_at: float, timeout_s: float = 12.0) -> dict[str, Any]:
        self._localization_session_started_at = float(session_started_at)
        deadline = time.time() + timeout_s
        latest = self.status()
        while time.time() < deadline:
            self._status_cached_at = 0.0
            latest = self.status()
            if self._nav_stack_live(latest) and self._rviz_map_ready(latest):
                return latest
            time.sleep(0.5)
        return latest

    def _start_result_message(self, selected_port: str, *, restarted: bool = False) -> str:
        action = "ROS2 建图已重启" if restarted else "ROS2 建图已启动"
        return f"{action}，雷达串口 {selected_port}"

    def _start_failure_message(self, selected_port: str, ros_status: dict[str, Any], *, restarted: bool = False) -> str:
        action = "重启" if restarted else "启动"
        reason = self._last_ready_error or self._derive_ready_error(ros_status) or "链路尚未就绪"
        details = [
            f"imu_driver={ros_status.get('imu_driver_ready')}",
            f"imu_filtered={ros_status.get('imu_filtered_live')}",
            f"imu_tf={ros_status.get('imu_tf_live')}",
            f"ekf={ros_status.get('ekf_ready')}",
            f"driver={ros_status.get('driver_live')}",
            f"slam={ros_status.get('slam_live')}",
            f"laser_tf={ros_status.get('laser_tf_live')}",
            f"odom_tf={ros_status.get('odom_tf_live')}",
            f"scan_fresh={ros_status.get('scan_fresh')}",
            f"map_fresh={ros_status.get('map_fresh')}",
            f"port={selected_port}",
        ]
        return f"ROS2 建图{action}未完全就绪: {reason} ({', '.join(details)})"

    def _start_or_reset_mapping(self, port: str, *, restarted: bool) -> dict[str, Any]:
        selected_port = self._resolve_lidar_port(port)
        self._reset_rviz_launch_state()
        self.preview.start()
        if self._exclusive_local_mode():
            self.logger.log("终端独占本地模式：跳过重复的建图/导航 stop 预清理")
        else:
            self.stop_localization(silent=True)
            self.stop_mapping(silent=True)
            time.sleep(1.5)
        self._stop_wifi_probe("mapping session reset")
        self._start_wifi_probe("mapping")
        self._wifi_probe_snapshot(
            "mapping_start_requested",
            phase="mapping_start",
            note=f"lidar={selected_port} restarted={int(bool(restarted))}",
        )
        self._wifi_probe_snapshot(
            "mapping_preclean_complete",
            phase="post_preclean",
            note=f"exclusive_local={int(bool(self._exclusive_local_mode()))}",
        )
        self._reset_preview_session()
        self._clear_ready_error()
        session_started_at = time.time()
        self._mapping_session_started_at = session_started_at
        self._localization_session_started_at = 0.0
        self._localization_map_name = ""
        self.last_lidar_port = selected_port
        result = self._run_script("start_ros2_mapping_stack.sh", selected_port, timeout=180)
        try:
            self._ensure_ok("启动ROS2建图", result)
        except Exception as exc:
            self._set_ready_error(str(exc))
            self._status_cached_at = 0.0
            raise
        self._status_cached_at = 0.0
        ros_status = self._wait_ready(session_started_at=session_started_at)
        if not self._mapping_ready(ros_status):
            raise RuntimeError(self._start_failure_message(selected_port, ros_status, restarted=restarted))
        self._wifi_probe_snapshot(
            "mapping_stack_ready",
            phase="mapping_ready",
            note=f"lidar={selected_port}",
        )

        rviz_started = False
        rviz_reused = False
        rviz_render_mode = ""
        warning = ""
        try:
            self.logger.log("尝试打开 RViz")
            self._wifi_probe_snapshot("rviz_open_requested", phase="rviz_requested", note="mapping session")
            rviz_result = self.open_rviz(include_status=not self._exclusive_local_mode())
            rviz_started = True
            rviz_reused = bool(rviz_result.get("reused"))
            rviz_render_mode = str(rviz_result.get("render_mode") or "").strip()
            warning = str(rviz_result.get("warning") or "").strip()
            self._wifi_probe_snapshot(
                "rviz_open_succeeded",
                phase="rviz_opened",
                note=f"mapping reused={int(rviz_reused)} mode={rviz_render_mode or '--'}",
            )
        except Exception as exc:
            warning = f"建图已启动，但 RViz 未打开: {exc}"
            self.logger.log(warning)
            self._wifi_probe_snapshot(
                "rviz_open_failed",
                phase="rviz_failed",
                note=f"mapping error={exc}",
            )
        self._status_cached_at = 0.0
        response_status = self.status() if not self._exclusive_local_mode() else {}
        return {
            "message": self._start_result_message(selected_port, restarted=restarted),
            "status": response_status,
            "rviz_started": rviz_started,
            "rviz_reused": rviz_reused,
            "rviz_render_mode": rviz_render_mode,
            "warning": warning,
        }

    def start_mapping(self, port: str) -> dict[str, Any]:
        return self._start_or_reset_mapping(port, restarted=False)

    def stop_mapping(self, silent: bool = False) -> str:
        self._wifi_probe_snapshot("mapping_stop_requested", phase="mapping_stop", note="stop_mapping invoked", log_errors=False)
        self._mapping_session_started_at = 0.0
        self._clear_ready_error()
        self._reset_preview_session()
        result = self._run_script("stop_ros2_mapping_stack.sh", timeout=90)
        self._stop_wifi_probe("mapping session stopped")
        if silent and result.returncode != 0:
            self._reset_preview_session()
            return self._command_output_text(result)
        self._ensure_ok("停止ROS2建图", result)
        self._reset_preview_session()
        self._status_cached_at = 0.0
        return self._command_output_text(result) or "ROS2 建图已停止"

    def reset_mapping(self, port: str) -> dict[str, Any]:
        return self._start_or_reset_mapping(port, restarted=True)

    def start_localization(self, map_name: str, port: str = "") -> dict[str, Any]:
        normalized_map = Path(map_name).stem
        if not normalized_map:
            raise RuntimeError("必须指定定位地图")
        selected_port = self._resolve_lidar_port(port)
        self._reset_rviz_launch_state()
        yaml_path = self._map_yaml_path(normalized_map)
        if not yaml_path.exists():
            raise RuntimeError(f"未找到地图文件: {yaml_path}")
        self.preview.start()
        if self._exclusive_local_mode():
            self.logger.log("终端独占本地模式：跳过重复的定位/建图 stop 预清理")
        else:
            self.stop_mapping(silent=True)
            self.stop_localization(silent=True)
            time.sleep(1.5)
        self._stop_wifi_probe("navigation session reset")
        self._start_wifi_probe("navigation")
        self._wifi_probe_snapshot(
            "navigation_start_requested",
            phase="navigation_start",
            note=f"map={normalized_map} lidar={selected_port}",
        )
        self._wifi_probe_snapshot(
            "navigation_preclean_complete",
            phase="post_preclean",
            note=f"map={normalized_map} exclusive_local={int(bool(self._exclusive_local_mode()))}",
        )
        self._reset_preview_session()
        self._clear_ready_error()
        session_started_at = time.time()
        self._localization_session_started_at = session_started_at
        self._mapping_session_started_at = 0.0
        self._localization_map_name = normalized_map
        self.last_lidar_port = selected_port
        self.logger.log(f"启动定位导航: map={normalized_map} lidar={selected_port}")
        result = self._run_script("start_ros2_navigation_stack.sh", normalized_map, selected_port, timeout=180)
        try:
            self._ensure_ok("启动定位导航", result)
        except Exception as exc:
            self._set_ready_error(str(exc))
            self._status_cached_at = 0.0
            raise
        self.logger.log("定位导航启动脚本已返回，开始刷新运行态")
        self._status_cached_at = 0.0
        self._wifi_probe_snapshot(
            "navigation_stack_ready",
            phase="navigation_ready",
            note=f"map={normalized_map} lidar={selected_port}",
        )
        warning = ""
        rviz_started = False
        rviz_reused = False
        rviz_render_mode = ""
        try:
            self.logger.log("尝试打开 RViz")
            self._wifi_probe_snapshot("rviz_open_requested", phase="rviz_requested", note="navigation session")
            rviz_result = self.open_rviz(include_status=not self._exclusive_local_mode())
            rviz_started = True
            rviz_reused = bool(rviz_result.get("reused"))
            rviz_render_mode = str(rviz_result.get("render_mode") or "").strip()
            rviz_warning = str(rviz_result.get("warning") or "").strip()
            if rviz_warning:
                warning = f"{warning}; {rviz_warning}" if warning else rviz_warning
            self._wifi_probe_snapshot(
                "rviz_open_succeeded",
                phase="rviz_opened",
                note=f"navigation reused={int(rviz_reused)} mode={rviz_render_mode or '--'}",
            )
        except Exception as exc:
            rviz_warning = f"定位导航已启动，但 RViz 未打开: {exc}"
            warning = f"{warning}; {rviz_warning}" if warning else rviz_warning
            self.logger.log(rviz_warning)
            self._wifi_probe_snapshot(
                "rviz_open_failed",
                phase="rviz_failed",
                note=f"navigation error={exc}",
            )
        self._status_cached_at = 0.0
        response_status: dict[str, Any] = {}
        initial_pose_hint = "导航栈与地图显示已就绪，请在 RViz 中使用 2D Pose Estimate 设置初始位姿"
        if self._exclusive_local_mode():
            self.logger.log("终端独占本地模式：跳过 start_localization 内部运行态快照，由 teleop 接管桥接和导航就绪判定")
            warning = f"{warning}; {initial_pose_hint}" if warning else initial_pose_hint
        else:
            time.sleep(0.8)
            response_status = self._wait_localization_start_ready(session_started_at, timeout_s=10.0)
            if self._nav_stack_live(response_status) and self._rviz_map_ready(response_status):
                if self._localized_ready(response_status):
                    self.logger.log("定位导航运行态已通过快速检查")
                else:
                    warning = f"{warning}; {initial_pose_hint}" if warning else initial_pose_hint
                    self.logger.log(initial_pose_hint)
            else:
                reason = self._derive_localization_error(response_status) or "定位导航主栈已启动，但地图显示仍未完全就绪"
                warning = f"{warning}; {reason}" if warning else reason
                self.logger.log(warning)
        message = f"定位导航栈已启动，地图 {normalized_map}，雷达串口 {selected_port}"
        if warning and "2D Pose Estimate" in warning:
            message = f"{message}；请在 RViz 中使用 2D Pose Estimate 设置初始位姿"
        return {
            "message": message,
            "status": response_status,
            "warning": warning,
            "rviz_started": rviz_started,
            "rviz_reused": rviz_reused,
            "rviz_render_mode": rviz_render_mode,
        }

    def stop_localization(self, silent: bool = False) -> str:
        self._wifi_probe_snapshot(
            "navigation_stop_requested",
            phase="navigation_stop",
            note="stop_localization invoked",
            log_errors=False,
        )
        self._localization_session_started_at = 0.0
        self._localization_map_name = ""
        self._clear_ready_error()
        result = self._run_script("stop_ros2_navigation_stack.sh", timeout=90)
        self._stop_wifi_probe("navigation session stopped")
        if silent and result.returncode != 0:
            self._status_cached_at = 0.0
            return self._command_output_text(result)
        self._ensure_ok("停止定位导航", result)
        self._status_cached_at = 0.0
        return self._command_output_text(result) or "定位导航栈已停止"

    def set_navigation_tolerance(self, arrival_tolerance: float) -> dict[str, float]:
        tolerance = max(0.05, float(arrival_tolerance))
        yaw_tolerance = max(0.2, min(1.2, tolerance * 1.6))
        workspace_setup = self.rock_ws / "install" / "setup.bash"
        commands = [
            f"export ROS_LOCALHOST_ONLY={LOCAL_ROS_LOCALHOST_ONLY}",
            "unset LD_PRELOAD",
            "source /opt/ros/foxy/setup.bash >/dev/null 2>&1",
            f"[ -f '{workspace_setup}' ] && source '{workspace_setup}' >/dev/null 2>&1 || true",
            f"ros2 param set /controller_server FollowPath.xy_goal_tolerance {tolerance:.3f}",
            f"ros2 param set /controller_server FollowPath.yaw_goal_tolerance {yaw_tolerance:.3f}",
        ]
        result = subprocess.run(
            ["/bin/bash", "-lc", "; ".join(commands)],
            cwd=self.workspace,
            text=True,
            capture_output=True,
            timeout=20,
            env=self._clean_script_env(),
            check=False,
        )
        self._ensure_ok("更新导航到点容差", result)
        return {"arrival_tolerance": round(tolerance, 3), "yaw_tolerance": round(yaw_tolerance, 3)}

    def save_map(self, name: str = "") -> dict[str, str]:
        map_name = (name or time.strftime("ros2_map_%Y%m%d_%H%M%S")).strip()
        result = self._run_script("save_ros2_map.sh", map_name, timeout=180)
        self._ensure_ok("保存ROS2地图", result)
        base = self.maps_dir / map_name
        files = {
            "base": str(base),
            "yaml": str(base.with_suffix(".yaml")),
            "image": str(base.with_suffix(".pgm")),
            "name": map_name,
        }
        self.logger.log(f"ROS2 地图已保存: {files['yaml']}")
        return files

    def open_rviz(self, display: str = "", *, include_status: bool = True) -> dict[str, Any]:
        desired_mode = self._rviz_default_render_mode
        self._set_rviz_launch_state(requested=True)
        existing_diag = self._rviz_diagnostics()
        if existing_diag["live"]:
            self._status_cached_at = 0.0
            warning = ""
            if existing_diag["render_mode"] != desired_mode:
                warning = "RViz 已在运行，但当前不是兼容模式；如地图仍不显示，请先关闭 RViz 后再重开"
            existing_target = str(self._read_rviz_mode_info().get("display") or "").strip()
            self._set_rviz_launch_state(
                requested=True,
                source="reused",
                target=f"DISPLAY={existing_target}" if existing_target else "",
                warning=warning,
            )
            return {
                "message": "RViz2 已在运行",
                "render_mode": existing_diag["render_mode"],
                "warning": warning,
                "reused": True,
                "status": self.status() if include_status else {},
            }
        if not self.ros_setup.exists():
            error = f"未找到 ROS2 环境: {self.ros_setup}"
            self._set_rviz_launch_state(requested=True, error=error)
            raise RuntimeError(error)
        if not self.rviz_config.exists():
            error = f"未找到 RViz 配置: {self.rviz_config}"
            self._set_rviz_launch_state(requested=True, error=error)
            raise RuntimeError(error)

        self.ros_log_dir.mkdir(parents=True, exist_ok=True)
        desktop_status = self._desktop_session_status(display)
        session_env = desktop_status["env"]
        display_hint = desktop_status["display"]
        xauthority = desktop_status["xauthority"]
        xdg_runtime_dir = desktop_status["xdg_runtime_dir"]
        launch_target = f"DISPLAY={display_hint}" if display_hint else ""
        launch_warning = ""
        if desktop_status["source"] == "session" and display_hint:
            launch_warning = f"当前终端图形环境不可用，RViz 已回退到活动桌面会话 {display_hint}"
        self._set_rviz_launch_state(
            requested=True,
            source=desktop_status["source"],
            target=launch_target,
            warning=launch_warning,
        )
        if not display_hint:
            error = desktop_status["last_error"] or "当前会话没有图形环境，RViz 未启动"
            self._set_rviz_launch_state(
                requested=True,
                source=desktop_status["source"],
                target=launch_target,
                error=error,
                warning=launch_warning,
            )
            raise RuntimeError(error)
        if xauthority and not Path(xauthority).exists() and not desktop_status["ready"]:
            error = f"找到显示 {display_hint}，但 XAUTHORITY / runtime 环境不完整 ({xauthority})"
            self._set_rviz_launch_state(
                requested=True,
                source=desktop_status["source"],
                target=launch_target,
                error=error,
                warning=launch_warning,
            )
            raise RuntimeError(error)
        if not desktop_status["ready"]:
            error = desktop_status["last_error"] or f"无法连接 DISPLAY {display_hint}"
            self._set_rviz_launch_state(
                requested=True,
                source=desktop_status["source"],
                target=launch_target,
                error=error,
                warning=launch_warning,
            )
            raise RuntimeError(error)

        log_file = self.ros_log_dir / f"rviz2_manual_{time.strftime('%Y%m%d_%H%M%S')}.log"
        pid_file = self.ros_log_dir / "rviz2.pid"
        workspace_setup = self.rock_ws / "install" / "setup.bash"
        command_parts = [
            "source /opt/ros/foxy/setup.bash >/dev/null 2>&1",
            f"[ -f '{workspace_setup}' ] && source '{workspace_setup}' >/dev/null 2>&1 || true",
            f"exec rviz2 -d '{self.rviz_config}'",
        ]
        env = self._clean_script_env()
        env["HOME"] = str(Path.home())
        env["DISPLAY"] = display_hint
        env["XDG_RUNTIME_DIR"] = xdg_runtime_dir
        env["LIBGL_ALWAYS_SOFTWARE"] = "1"
        env["QT_XCB_FORCE_SOFTWARE_OPENGL"] = "1"
        env["QT_OPENGL"] = "software"
        env["QT_QPA_PLATFORM"] = "xcb"
        if xauthority:
            env["XAUTHORITY"] = xauthority
        for key in ("DBUS_SESSION_BUS_ADDRESS", "WAYLAND_DISPLAY", "XDG_SESSION_TYPE", "XDG_CURRENT_DESKTOP", "DESKTOP_SESSION"):
            value = session_env.get(key, "").strip()
            if value:
                env[key] = value
        self.logger.log(
            f"RViz 启动目标: source={desktop_status['source']} "
            f"display={display_hint} xauthority={xauthority or '--'}"
        )

        with log_file.open("w", encoding="utf-8", errors="replace") as handle:
            process = subprocess.Popen(
                ["/bin/bash", "-lc", "; ".join(command_parts)],
                cwd=self.workspace,
                stdout=handle,
                stderr=subprocess.STDOUT,
                env=env,
            )

        for _ in range(6):
            time.sleep(1.0)
            if process.poll() is not None:
                break
        if process.poll() is not None:
            details = ""
            with contextlib.suppress(Exception):
                tail = log_file.read_text(encoding="utf-8", errors="replace").splitlines()[-8:]
                details = " | ".join(line.strip() for line in tail if line.strip())
            if not details:
                details = f"请确认板端图形环境可用，当前 DISPLAY={display_hint}"
            error = f"RViz2 启动失败: RViz 进程启动后立刻退出: {details}"
            self._set_rviz_launch_state(
                requested=True,
                source=desktop_status["source"],
                target=launch_target,
                error=error,
                warning=launch_warning,
            )
            raise RuntimeError(error)

        pid_file.write_text(str(process.pid), encoding="utf-8")
        self._write_rviz_mode_info(
            render_mode=desired_mode,
            display=display_hint,
            xauthority=xauthority,
            xdg_runtime_dir=xdg_runtime_dir,
            log_file=log_file,
        )
        self._set_rviz_launch_state(
            requested=True,
            source=desktop_status["source"],
            target=launch_target,
            warning=launch_warning,
        )
        self.logger.log(f"RViz2 已启动: DISPLAY={display_hint} 模式={desired_mode}")
        self._status_cached_at = 0.0
        return {
            "message": f"RViz2 已启动，显示 {display_hint}",
            "render_mode": desired_mode,
            "warning": launch_warning,
            "reused": False,
            "status": self.status() if include_status else {},
        }

    @staticmethod
    def _age_ms(timestamp: float) -> int | None:
        if not timestamp:
            return None
        return max(0, int((time.time() - timestamp) * 1000.0))

    def scan_jpeg(self) -> bytes:
        self.preview.start()
        preview = self.preview.snapshot()
        if not preview.available:
            return draw_text_panel(
                [
                    "ROS 实时扫点预览不可用",
                    f"原因: {ROS_PREVIEW_IMPORT_ERROR}",
                    "请确认板端服务已 source ROS 环境后再启动。",
                ]
            )
        if not preview.scan_frames or not preview.scan_points:
            return draw_text_panel(
                [
                    "正在等待 /scan 实时数据",
                    f"当前扫帧数: {preview.scan_frames}",
                    f"最近扫帧年龄: {self._age_ms(preview.last_scan_at) or '--'} ms",
                ]
            )

        max_distance = min(
            12000.0,
            max(6000.0, max((point[1] for point in preview.scan_points), default=6000.0) * 1.05),
        )
        image = LidarSlamWindow.render_scan_image(
            preview.scan_points,
            size=(1420, 900),
            max_distance_mm=max_distance,
        )
        ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        return buffer.tobytes() if ok else b""

    def map_jpeg(self) -> bytes:
        self.preview.start()
        preview = self.preview.snapshot()
        if not preview.available:
            return draw_text_panel(
                [
                    "ROS 地图预览不可用",
                    f"原因: {ROS_PREVIEW_IMPORT_ERROR}",
                    "请确认板端服务已 source ROS 环境后再启动。",
                ]
            )
        if not preview.map_frames:
            return draw_text_panel(
                [
                    "正在等待 /map 实时数据",
                    f"当前地图帧数: {preview.map_frames}",
                    f"最近地图帧年龄: {self._age_ms(preview.last_map_at) or '--'} ms",
                ]
            )

        image = LidarSlamWindow.render_map_image(
            preview.map_image,
            preview.pose_mm,
            preview.pose_history_mm,
            size=(1420, 900),
            map_size_meters=(preview.map_width_meters, preview.map_height_meters),
        )
        overlay_rows = [
            f"Map frames: {preview.map_frames}",
            f"Last map age: {self._age_ms(preview.last_map_at) or 0} ms",
            f"Pose: {preview.pose_mm[0] / 1000.0:.2f}, {preview.pose_mm[1] / 1000.0:.2f} m",
        ]
        overlay_x = 28
        overlay_y = image.shape[0] - 72
        cv2.rectangle(
            image,
            (overlay_x - 12, overlay_y - 26),
            (overlay_x + 340, overlay_y + 54),
            (246, 248, 250),
            -1,
        )
        cv2.rectangle(
            image,
            (overlay_x - 12, overlay_y - 26),
            (overlay_x + 340, overlay_y + 54),
            (208, 214, 220),
            1,
        )
        for index, row in enumerate(overlay_rows):
            cv2.putText(
                image,
                row,
                (overlay_x, overlay_y + index * 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (48, 62, 78),
                2,
                lineType=cv2.LINE_AA,
            )
        ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        return buffer.tobytes() if ok else b""

    def status(self) -> dict[str, Any]:
        now = time.time()
        if self._status_cache and now - self._status_cached_at < self._cache_ttl_s:
            return dict(self._status_cache)

        saved_maps = self.list_saved_maps(limit=1)
        latest_saved_map = saved_maps[0] if saved_maps else None
        lidar_ports = self.list_lidar_ports()
        topics, nodes = self._ros_cli_graph_snapshot()
        scan_topic_ready = "/scan" in topics
        map_topic_ready = "/map" in topics
        tf_topic_ready = "/tf" in topics or "/tf_static" in topics
        amcl_pose_topic_ready = "/amcl_pose" in topics
        navigate_action_ready = any(topic.startswith("/navigate_to_pose/_action/") for topic in topics)
        imu_raw_topic_ready = "/imu/data_raw" in topics
        imu_filtered_topic_ready = "/imu/data" in topics
        odom_raw_topic_ready = "/odom_raw" in topics
        final_odom_topic_ready = "/odom" in topics
        port_present = self._port_exists(self.last_lidar_port)
        imu_port = self._imu_port()
        imu_baudrate = self._imu_baudrate()
        imu_frame = self._imu_frame()
        imu_mode = self._imu_mode()
        imu_port_present = self._port_exists(imu_port)
        imu_driver_status = self._read_imu_driver_status()
        imu_driver_last_error = str(imu_driver_status.get("last_error") or "").strip()
        imu_driver_version = str(imu_driver_status.get("version") or "").strip()
        imu_last_age_ms = imu_driver_status.get("last_publish_age_ms")
        if not isinstance(imu_last_age_ms, (int, float)):
            with contextlib.suppress(Exception):
                last_publish_at = float(imu_driver_status.get("last_publish_at") or 0.0)
                if last_publish_at > 0.0:
                    imu_last_age_ms = max(0, int((now - last_publish_at) * 1000.0))
        if not isinstance(imu_last_age_ms, (int, float)):
            imu_last_age_ms = None
        else:
            imu_last_age_ms = int(imu_last_age_ms)
        imu_driver_status_fresh = bool(
            imu_driver_status.get("ready")
            and imu_last_age_ms is not None
            and imu_last_age_ms < int(IMU_DRIVER_STATUS_STALE_AFTER_S * 1000.0)
        )
        preview = self.preview.snapshot()
        rviz_diag = self._rviz_diagnostics()
        desktop_status = self._desktop_session_status()
        controller_status = self.controller.status() if self.controller is not None else {}
        control_formal = controller_status.get("formal", {})
        control_connected = bool(controller_status.get("connected"))
        control_ack_ready = bool(control_formal.get("control_ack_ready"))
        raw_odom_bridge_ready = bool(control_formal.get("odom_bridge_ready"))
        raw_odom_live = bool(control_formal.get("odom_reporting"))
        raw_embedded_tf_live = bool(
            control_connected and raw_odom_bridge_ready and (raw_odom_live or odom_raw_topic_ready or tf_topic_ready)
        )
        mapping_session_live = self._mapping_session_started_at > 0
        localization_session_live = self._localization_session_started_at > 0
        imu_driver_node_live = any("car2_imu_driver" in node for node in nodes)
        imu_filter_node_live = any("imu_filter" in node for node in nodes)
        ekf_node_live = any("ekf" in node for node in nodes)
        driver_node_live = any("sllidar" in node for node in nodes)
        slam_node_live = any("slam_toolbox" in node for node in nodes)
        laser_tf_node_live = any("static_transform_publisher" in node for node in nodes)
        odom_tf_node_live = any("ekf" in node or "odometry_to_tf" in node for node in nodes)
        map_server_node_live = any("map_server" in node for node in nodes)
        amcl_node_live = any("amcl" in node for node in nodes)
        nav2_node_live = any(
            any(name in node for name in ("planner_server", "controller_server", "bt_navigator", "recoveries_server"))
            for node in nodes
        )
        imu_driver_running = self._managed_process_running("imu_driver", "car2_imu_driver.py")
        imu_filter_running = self._managed_process_running("imu_filter", "imu_filter_madgwick_node")
        ekf_running = self._managed_process_running("ekf_localization", "ekf_node")
        lidar_driver_running = self._managed_process_running(
            "sllidar_driver",
            "sllidar_node",
            fatal_patterns=("SL_RESULT_OPERATION_TIMEOUT", "operation time out", "exit code 255"),
        )
        slam_toolbox_running = self._managed_process_running("slam_toolbox", "async_slam_toolbox_node")
        laser_tf_running = self._managed_process_running("laser_tf", "base_link laser")
        imu_tf_running = self._managed_process_running("imu_tf", f"base_link {imu_frame}")
        odom_tf_running = self._managed_process_running("odom_to_tf", "odom base_link")
        nav2_bringup_running = self._managed_process_running("nav2_bringup", "nav2_bringup")
        rviz_running = self._pid_file_running("rviz2")
        imu_driver_live = bool(imu_driver_running or imu_driver_node_live)
        imu_filter_live = bool(imu_filter_running or imu_filter_node_live)
        ekf_live = bool(ekf_running or ekf_node_live)
        imu_driver_ready = bool(
            imu_port_present
            and (
                imu_driver_status_fresh
                or (imu_driver_live and imu_raw_topic_ready)
            )
        )
        driver_live = bool(lidar_driver_running or driver_node_live)
        slam_live = bool(slam_toolbox_running or slam_node_live)
        laser_tf_live = bool(laser_tf_running or laser_tf_node_live or raw_embedded_tf_live)
        imu_tf_live = bool(raw_embedded_tf_live or imu_tf_running)
        map_server_live = bool(map_server_node_live or (nav2_bringup_running and (map_topic_ready or localization_session_live)))
        amcl_live = bool(amcl_node_live or (nav2_bringup_running and (amcl_pose_topic_ready or localization_session_live)))
        nav2_live = bool(navigate_action_ready or nav2_node_live or (nav2_bringup_running and localization_session_live))
        localization_live = bool(map_server_live or amcl_live or nav2_live or localization_session_live)
        navigation_mode = (
            "mapping"
            if (slam_live or mapping_session_live)
            else ("localization" if localization_live else "idle")
        )
        imu_session_live = bool(mapping_session_live or localization_session_live)
        if imu_session_live and (imu_driver_ready or imu_raw_topic_ready or imu_driver_live):
            self._refresh_imu_raw_probe_async(force=not imu_raw_topic_ready)
        if imu_session_live and (imu_filter_live or imu_filtered_topic_ready or ekf_live):
            self._refresh_imu_filtered_probe_async(force=not imu_filtered_topic_ready)
        with self._imu_raw_probe_lock:
            imu_raw_probe_error = self._imu_raw_probe_last_error
            imu_raw_probe_inflight = self._imu_raw_probe_inflight
        with self._imu_filtered_probe_lock:
            imu_filtered_probe_error = self._imu_filtered_probe_last_error
            imu_filtered_probe_inflight = self._imu_filtered_probe_inflight
        imu_raw_probe_fresh = bool(imu_session_live and self._imu_raw_probe_fresh(now=now))
        imu_filtered_probe_fresh = bool(imu_session_live and self._imu_filtered_probe_fresh(now=now))
        imu_raw_topic_ready = bool(imu_raw_topic_ready or imu_raw_probe_fresh)
        imu_filtered_topic_ready = bool(imu_filtered_topic_ready or imu_filtered_probe_fresh)
        imu_raw_live = bool(
            imu_port_present
            and imu_raw_topic_ready
            and (imu_driver_ready or imu_driver_status_fresh or imu_driver_live)
        )
        imu_filtered_live = bool(
            imu_filtered_topic_ready
            and (imu_filter_live or imu_filtered_probe_fresh or ekf_live)
        )
        if (mapping_session_live or localization_session_live) and (final_odom_topic_ready or ekf_live):
            self._refresh_odom_probe_async(force=not final_odom_topic_ready)
        with self._odom_probe_lock:
            odom_probe_error = self._odom_probe_last_error
            odom_probe_inflight = self._odom_probe_inflight
        ekf_odom_fresh = bool(
            (mapping_session_live or localization_session_live)
            and self._odom_probe_fresh(now=now)
        )
        final_odom_topic_ready = bool(final_odom_topic_ready or ekf_odom_fresh)
        final_odom_visible = bool(final_odom_topic_ready)
        ekf_ready = bool(final_odom_visible and (ekf_live or ekf_odom_fresh))
        ekf_last_age_ms = self._odom_probe_age_ms(now=now)
        odom_tf_live = bool(
            (odom_tf_running or odom_tf_node_live or ekf_live or ekf_odom_fresh)
            and final_odom_visible
            and (tf_topic_ready or ekf_odom_fresh)
        )
        preview_unhealthy = bool((not preview.available) or (not preview.running) or preview.last_error)
        active_scan_session_started_at = 0.0
        if navigation_mode == "mapping":
            active_scan_session_started_at = self._mapping_session_started_at
        elif navigation_mode == "localization":
            active_scan_session_started_at = self._localization_session_started_at
        preview_scan_fresh = self._session_frame_fresh(
            preview.scan_frames,
            preview.last_scan_at,
            now=now,
            session_started_at=active_scan_session_started_at,
        )
        if active_scan_session_started_at and (scan_topic_ready or driver_live) and (not preview_scan_fresh or preview_unhealthy):
            self._refresh_scan_probe_async(force=preview_unhealthy)
        with self._scan_probe_lock:
            scan_probe_error = self._scan_probe_last_error
            scan_probe_inflight = self._scan_probe_inflight
        scan_probe_fresh = bool(
            active_scan_session_started_at
            and (scan_topic_ready or driver_live)
            and self._scan_probe_fresh(now=now)
        )
        scan_fresh = bool(preview_scan_fresh or scan_probe_fresh)
        scan_topic_ready = bool(scan_topic_ready or scan_probe_fresh)
        scan_stream_live = scan_fresh
        preview_map_fresh = self._session_frame_fresh(
            preview.map_frames,
            preview.last_map_at,
            now=now,
            session_started_at=self._mapping_session_started_at,
        )
        preview_map_ready = self._session_frame_seen(
            preview.map_frames,
            preview.last_map_at,
            session_started_at=self._localization_session_started_at,
        )
        if mapping_session_live and (map_topic_ready or slam_live) and (not preview_map_fresh or preview_unhealthy):
            self._refresh_map_probe_async(force=preview_unhealthy)
        if localization_session_live and (map_topic_ready or map_server_live) and (not preview_map_ready or preview_unhealthy):
            self._refresh_map_probe_async(force=preview_unhealthy)
        with self._map_probe_lock:
            map_probe_error = self._map_probe_last_error
            map_probe_inflight = self._map_probe_inflight
        map_probe_mapping_fresh = bool(
            mapping_session_live
            and self._map_probe_fresh(now=now, session_started_at=self._mapping_session_started_at)
        )
        map_probe_localization_ready = bool(
            localization_session_live
            and self._map_probe_fresh(
                now=now,
                session_started_at=self._localization_session_started_at,
                sticky=True,
            )
        )
        with self._map_to_odom_tf_lock:
            map_to_odom_tf_error = self._map_to_odom_tf_last_error
            map_to_odom_tf_inflight = self._map_to_odom_tf_inflight
        if navigation_mode == "mapping":
            map_fresh = bool(preview_map_fresh or map_probe_mapping_fresh)
            map_probe_fresh = map_probe_mapping_fresh
            preview_map_mode_ready = preview_map_fresh
        elif navigation_mode == "localization":
            map_fresh = bool(preview_map_ready or map_probe_localization_ready)
            map_probe_fresh = map_probe_localization_ready
            preview_map_mode_ready = preview_map_ready
        else:
            map_fresh = bool(
                preview_map_fresh
                or preview_map_ready
                or map_probe_mapping_fresh
                or map_probe_localization_ready
            )
            map_probe_fresh = bool(map_probe_mapping_fresh or map_probe_localization_ready)
            preview_map_mode_ready = bool(preview_map_fresh or preview_map_ready)
        map_topic_ready = bool(map_topic_ready or map_probe_fresh)
        map_stream_live = bool(map_fresh or map_server_live or slam_live or map_topic_ready)
        if map_stream_live and rviz_diag["error_code"] == "tf_wait":
            rviz_diag["error_code"] = ""
            rviz_diag["last_error"] = ""
        rviz_map_ready = bool(
            map_topic_ready
            and map_fresh
            and scan_fresh
            and laser_tf_live
            and imu_tf_live
            and odom_tf_live
            and rviz_diag["error_code"] not in {"config_plugin", "render_gl"}
        )
        nav_stack_live = self._nav_stack_live(
            {
                "imu_driver_ready": imu_driver_ready,
                "imu_filtered_live": imu_filtered_live,
                "imu_tf_live": imu_tf_live,
                "ekf_ready": ekf_ready,
                "driver_live": driver_live,
                "laser_tf_live": laser_tf_live,
                "odom_tf_live": odom_tf_live,
                "map_server_live": map_server_live,
                "amcl_live": amcl_live,
                "nav2_live": nav2_live,
                "scan_fresh": scan_fresh,
            }
        )
        if localization_session_live and nav_stack_live and not self._map_to_odom_tf_fresh(now=now):
            self._refresh_map_to_odom_tf_probe_async()
        map_to_odom_tf_live = bool(localization_session_live and self._map_to_odom_tf_fresh(now=now))
        navigation_drive_ready = bool(nav2_live and control_connected and raw_odom_bridge_ready and ekf_ready)
        localized_ready = bool(nav_stack_live and amcl_pose_topic_ready and map_to_odom_tf_live)
        status_context = {
            "last_lidar_port": self.last_lidar_port,
            "lidar_port_present": port_present,
            "localization_map": self._localization_map_name,
            "imu_driver_ready": imu_driver_ready,
            "imu_filtered_live": imu_filtered_live,
            "imu_tf_live": imu_tf_live,
            "ekf_ready": ekf_ready,
            "driver_live": driver_live,
            "slam_live": slam_live,
            "laser_tf_live": laser_tf_live,
            "odom_tf_live": odom_tf_live,
            "map_server_live": map_server_live,
            "amcl_live": amcl_live,
            "nav2_live": nav2_live,
            "nav_stack_live": nav_stack_live,
            "scan_fresh": scan_fresh,
            "map_fresh": map_fresh,
            "rviz_live": rviz_running,
            "rviz_error_code": rviz_diag["error_code"],
            "rviz_map_ready": rviz_map_ready,
            "localized_ready": localized_ready,
            "map_to_odom_tf_live": map_to_odom_tf_live,
            "scan_topic_ready": scan_topic_ready,
            "map_topic_ready": map_topic_ready,
            "scan_probe_error": scan_probe_error,
            "map_probe_error": map_probe_error,
            "preview_error": preview.last_error or "",
        }
        last_ready_error = self._last_ready_error
        if navigation_mode == "mapping" and self._mapping_ready(status_context):
            last_ready_error = ""
        elif navigation_mode == "localization":
            if localized_ready:
                last_ready_error = ""
            else:
                localization_hint = self._derive_localization_error(status_context)
                last_ready_error = self._log_ready_error_hint() or localization_hint or self._last_ready_error
        elif (
            imu_driver_running
            or imu_filter_running
            or ekf_running
            or driver_live
            or slam_live
            or laser_tf_running
            or odom_tf_running
            or nav2_bringup_running
            or mapping_session_live
            or localization_session_live
        ):
            last_ready_error = self._log_ready_error_hint() or self._last_ready_error
        rviz_map_render_ok = bool(
            rviz_running
            and rviz_map_ready
            and rviz_diag["error_code"] not in {"config_plugin", "render_gl"}
        )
        with self._wifi_probe_lock:
            wifi_probe_running = bool(self._wifi_probe_thread and self._wifi_probe_thread.is_alive())
            wifi_probe_summary = dict(self._wifi_probe_last_summary)
            wifi_probe_error = self._wifi_probe_last_error
            wifi_probe_last_sample_at = self._wifi_probe_last_sample_at
        error_code = self._ready_error_code(last_ready_error)
        if not error_code and rviz_diag["error_code"]:
            error_code = rviz_diag["error_code"]
        status = {
            "roscore": False,
            "rviz": rviz_running,
            "rviz_live": rviz_running,
            "rviz_render_mode": rviz_diag["render_mode"],
            "rviz_error_code": rviz_diag["error_code"],
            "rviz_last_error": rviz_diag["last_error"],
            "rviz_map_render_ok": rviz_map_render_ok,
            "rviz_display": desktop_status["display"],
            "rviz_launch_requested": self._rviz_launch_requested,
            "rviz_launch_source": self._rviz_launch_source,
            "rviz_launch_target": self._rviz_launch_target,
            "rviz_launch_error": self._rviz_launch_error,
            "rviz_launch_warning": self._rviz_launch_warning,
            "mapping": driver_live and slam_live,
            "navigation_mode": navigation_mode,
            "localization_map": self._localization_map_name,
            "nav_session_started_at": round(self._localization_session_started_at, 3) if localization_session_live else 0.0,
            "mapping_backend": "slam_toolbox" if slam_live else "idle",
            "gmapping": False,
            "hector": False,
            "scan_bridge": False,
            "imu_mode": imu_mode,
            "imu_port": imu_port,
            "imu_baudrate": imu_baudrate,
            "imu_frame": imu_frame,
            "imu_status_file": str(self.imu_driver_status_path),
            "imu_port_present": imu_port_present,
            "imu_driver_live": imu_driver_live,
            "imu_driver_ready": imu_driver_ready,
            "imu_driver_last_error": imu_driver_last_error,
            "imu_driver_version": imu_driver_version,
            "imu_raw_topic_ready": imu_raw_topic_ready,
            "imu_raw_live": imu_raw_live,
            "imu_raw_probe_error": imu_raw_probe_error,
            "imu_raw_probe_inflight": imu_raw_probe_inflight,
            "imu_filtered_topic_ready": imu_filtered_topic_ready,
            "imu_filtered_live": imu_filtered_live,
            "imu_filtered_probe_error": imu_filtered_probe_error,
            "imu_filtered_probe_inflight": imu_filtered_probe_inflight,
            "imu_last_age_ms": imu_last_age_ms,
            "ekf_live": ekf_live,
            "ekf_ready": ekf_ready,
            "ekf_last_age_ms": ekf_last_age_ms,
            "ekf_error": odom_probe_error,
            "odom_source": "ekf",
            "odom_raw_source": control_formal.get("odom_source") or "f103_serial_raw",
            "odom_raw_topic_ready": odom_raw_topic_ready,
            "odom_raw_live": raw_odom_live,
            "driver_live": driver_live,
            "slam_live": slam_live,
            "laser_tf_live": laser_tf_live,
            "imu_tf_live": imu_tf_live,
            "odom_tf_live": odom_tf_live,
            "odom_bridge_ready": raw_odom_bridge_ready,
            "odom_raw_bridge_ready": raw_odom_bridge_ready,
            "embedded_tf_live": raw_embedded_tf_live,
            "raw_embedded_tf_live": raw_embedded_tf_live,
            "map_server_live": map_server_live,
            "amcl_live": amcl_live,
            "nav2_live": nav2_live,
            "nav_stack_live": nav_stack_live,
            "rviz_map_ready": rviz_map_ready,
            "localized_ready": localized_ready,
            "map_to_odom_tf_live": map_to_odom_tf_live,
            "initial_pose_required": bool(nav_stack_live and rviz_map_ready and not localized_ready),
            "navigation_drive_ready": navigation_drive_ready,
            "control_port_connected": control_connected,
            "control_ack_ready": control_ack_ready,
            "ros_localhost_only": LOCAL_ROS_LOCALHOST_ONLY,
            "scan_fresh": scan_fresh,
            "map_fresh": map_fresh,
            "error_code": error_code,
            "last_ready_error": last_ready_error,
            "lidar_ports": lidar_ports,
            "lidar_driver": lidar_driver_running,
            "slam_toolbox": slam_toolbox_running,
            "nav2_bringup": nav2_bringup_running,
            "imu_driver_running": imu_driver_running,
            "imu_filter_running": imu_filter_running,
            "ekf_running": ekf_running,
            "scan_topic_ready": scan_topic_ready,
            "map_topic_ready": map_topic_ready,
            "tf_topic_ready": tf_topic_ready,
            "amcl_pose_topic_ready": amcl_pose_topic_ready,
            "navigate_action_ready": navigate_action_ready,
            "driver_node_live": driver_node_live,
            "slam_node_live": slam_node_live,
            "laser_tf_node_live": laser_tf_node_live,
            "odom_tf_node_live": odom_tf_node_live,
            "imu_driver_node_live": imu_driver_node_live,
            "imu_filter_node_live": imu_filter_node_live,
            "ekf_node_live": ekf_node_live,
            "map_server_node_live": map_server_node_live,
            "amcl_node_live": amcl_node_live,
            "nav2_node_live": nav2_node_live,
            "preview_available": preview.available,
            "preview_running": preview.running,
            "preview_error": preview.last_error or ("" if preview.available else str(ROS_PREVIEW_IMPORT_ERROR)),
            "preview_scan_fresh": preview_scan_fresh,
            "preview_map_fresh": preview_map_fresh,
            "preview_map_ready": preview_map_ready,
            "preview_map_mode_ready": preview_map_mode_ready,
            "scan_frames_received": preview.scan_frames,
            "map_frames_received": preview.map_frames,
            "odom_probe_fresh": ekf_odom_fresh,
            "odom_probe_ok": ekf_odom_fresh,
            "odom_probe_age_ms": ekf_last_age_ms,
            "odom_probe_error": odom_probe_error,
            "odom_probe_inflight": odom_probe_inflight,
            "scan_probe_fresh": scan_probe_fresh,
            "scan_probe_ok": scan_probe_fresh,
            "scan_probe_age_ms": self._scan_probe_age_ms(now=now),
            "scan_probe_error": scan_probe_error,
            "scan_probe_inflight": scan_probe_inflight,
            "map_probe_fresh": map_probe_fresh,
            "map_probe_ok": map_probe_fresh,
            "map_probe_age_ms": self._map_probe_age_ms(now=now),
            "map_probe_error": map_probe_error,
            "map_probe_inflight": map_probe_inflight,
            "map_to_odom_tf_age_ms": self._map_to_odom_tf_age_ms(now=now),
            "map_to_odom_tf_error": map_to_odom_tf_error,
            "map_to_odom_tf_inflight": map_to_odom_tf_inflight,
            "scan_stream_live": scan_stream_live,
            "map_stream_live": map_stream_live,
            "last_scan_age_ms": self._age_ms(preview.last_scan_at),
            "last_map_age_ms": self._age_ms(preview.last_map_at),
            "map_width_m": round(preview.map_width_meters, 2),
            "map_height_m": round(preview.map_height_meters, 2),
            "map_resolution_cm": round(preview.map_resolution * 100.0, 2),
            "pose_m": [
                round(preview.pose_mm[0] / 1000.0, 2),
                round(preview.pose_mm[1] / 1000.0, 2),
                round(preview.pose_mm[2], 1),
            ],
            "last_lidar_port": self.last_lidar_port,
            "lidar_port_present": port_present,
            "scripts_ready": all(
                self._script(name).exists()
                for name in (
                    "start_ros2_mapping_stack.sh",
                    "stop_ros2_mapping_stack.sh",
                    "save_ros2_map.sh",
                    "start_ros2_navigation_stack.sh",
                    "stop_ros2_navigation_stack.sh",
                )
            ),
            "maps_dir": str(self.maps_dir),
            "saved_maps_count": self.saved_maps_count(),
            "latest_saved_map": latest_saved_map["name"] if latest_saved_map else "",
            "latest_saved_at": latest_saved_map["updated_at"] if latest_saved_map else "",
            "ros_setup_exists": self.ros_setup.exists(),
            "ros_workspace_exists": self.rock_ws.exists(),
            "desktop_session_ready": desktop_status["ready"],
            "desktop_session_source": desktop_status["source"],
            "desktop_session_error": desktop_status["last_error"],
            "wifi_probe_enabled": self._wifi_probe_enabled(),
            "wifi_probe_running": wifi_probe_running,
            "wifi_probe_log_path": str(self.wifi_probe_log_path),
            "wifi_probe_state_path": str(self.wifi_probe_state_path),
            "wifi_probe_last_label": str(wifi_probe_summary.get("label") or ""),
            "wifi_probe_last_phase": str(wifi_probe_summary.get("phase") or ""),
            "wifi_probe_last_error": wifi_probe_error,
            "wifi_probe_last_age_ms": self._age_ms(wifi_probe_last_sample_at),
            "wifi_probe_ap_local_state": str(wifi_probe_summary.get("ap_local_state") or ""),
            "wifi_probe_rock_active_device": str(wifi_probe_summary.get("rock_active_device") or ""),
            "wifi_probe_ap_iface": str(wifi_probe_summary.get("ap_iface") or ""),
            "wifi_probe_ap_ssid": str(wifi_probe_summary.get("ap_ssid") or ""),
            "wifi_probe_hotspot_ip_iface": str(wifi_probe_summary.get("hotspot_ip_iface") or ""),
            "wifi_probe_hotspot_ip_value": str(wifi_probe_summary.get("hotspot_ip_value") or ""),
            "note": "???????? teleop /odom_raw + IMU driver/filter + EKF /odom?WASD ? /cmd_vel legacy ????????",
        }
        self._status_cache = dict(status)
        self._status_cached_at = now
        return status

    def _ros2_topic_list(self) -> set[str]:
        if not self.ros_setup.exists():
            return set()
        command = (
            f"export ROS_LOCALHOST_ONLY={LOCAL_ROS_LOCALHOST_ONLY}; "
            "source /opt/ros/foxy/setup.bash >/dev/null 2>&1; "
            f"[ -f '{self.rock_ws / 'install' / 'setup.bash'}' ] && source '{self.rock_ws / 'install' / 'setup.bash'}' >/dev/null 2>&1 || true; "
            "ros2 topic list --no-daemon 2>/dev/null || true"
        )
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", command],
                text=True,
                capture_output=True,
                timeout=self._graph_cli_timeout_s,
                env=self._clean_script_env(),
                check=False,
            )
            return {line.strip() for line in result.stdout.splitlines() if line.strip()}
        except Exception:
            return set()

    def _ros2_node_list(self) -> set[str]:
        if not self.ros_setup.exists():
            return set()
        command = (
            f"export ROS_LOCALHOST_ONLY={LOCAL_ROS_LOCALHOST_ONLY}; "
            "source /opt/ros/foxy/setup.bash >/dev/null 2>&1; "
            f"[ -f '{self.rock_ws / 'install' / 'setup.bash'}' ] && source '{self.rock_ws / 'install' / 'setup.bash'}' >/dev/null 2>&1 || true; "
            "ros2 node list --no-daemon 2>/dev/null || true"
        )
        try:
            result = subprocess.run(
                ["/bin/bash", "-lc", command],
                text=True,
                capture_output=True,
                timeout=self._graph_cli_timeout_s,
                env=self._clean_script_env(),
                check=False,
            )
            return {line.strip() for line in result.stdout.splitlines() if line.strip()}
        except Exception:
            return set()

    @staticmethod
    def _process_running(keyword: str) -> bool:
        try:
            output = subprocess.check_output(["ps", "-eo", "args="], text=True)
            return any(keyword in line for line in output.splitlines())
        except Exception:
            return False


class RtBridgeSystem:
    def __init__(self, logger: RuntimeLogger):
        self.logger = logger
        self.microros_ws = Path.home() / "Desktop" / "rock_ws" / "microros_ws"
        self.deploy_root = Path.home() / "Desktop" / "car2.0_board_deploy"
        self.runtime_dir = self.deploy_root / "runtime"
        self.agent_disable_flag = self.runtime_dir / "disable_micro_ros_agent.flag"
        self.agent_service = "micro-ros-agent.service"
        self.agent_port = 8888
        self.agent_transport = "udp4"
        self.agent_log = self.deploy_root / "logs" / "micro_ros_agent_stdout.log"
        self._status_cache: dict[str, Any] = {}
        self._status_cached_at = 0.0
        self._cache_ttl_s = 2.0

    @staticmethod
    def _run_command(command: list[str], timeout: int = 20) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        env["ROS_LOCALHOST_ONLY"] = LOCAL_ROS_LOCALHOST_ONLY
        env.pop("LD_PRELOAD", None)
        try:
            return subprocess.run(
                command,
                text=True,
                capture_output=True,
                timeout=timeout,
                env=env,
                check=False,
            )
        except FileNotFoundError as exc:
            return subprocess.CompletedProcess(command, 127, "", str(exc))

    def _run_bash(self, script: str, timeout: int = 20) -> subprocess.CompletedProcess[str]:
        return self._run_command(["/bin/bash", "-lc", script], timeout=timeout)

    def _systemctl_state(self, action: str) -> str:
        result = self._run_command(["systemctl", action, self.agent_service], timeout=10)
        text = (result.stdout or result.stderr or "").strip()
        return text or "unknown"

    def _agent_listening(self) -> bool:
        result = self._run_command(["/bin/bash", "-lc", f"ss -lunp | grep ':{self.agent_port} ' || true"], timeout=10)
        return bool((result.stdout or "").strip())

    def _interface_ipv4(self, iface: str) -> str:
        result = self._run_bash(f"ip -4 -brief addr show {iface} 2>/dev/null | awk '{{print $3}}' | cut -d/ -f1", timeout=10)
        return (result.stdout or "").strip()

    def _first_interface_ipv4(self, ifaces: list[str]) -> str:
        for iface in ifaces:
            value = self._interface_ipv4(iface)
            if value:
                return value
        return ""

    def _ros2_runtime(self) -> tuple[list[str], list[str]]:
        ros_setup = Path("/opt/ros/foxy/setup.bash")
        ws_setup = self.microros_ws / "install" / "setup.bash"
        if not ros_setup.exists():
            return [], []
        script_lines = [
            f"export ROS_LOCALHOST_ONLY={LOCAL_ROS_LOCALHOST_ONLY}",
            f"source {ros_setup} >/dev/null 2>&1",
            f"[ -f {ws_setup} ] && source {ws_setup} >/dev/null 2>&1 || true",
            "ros2 node list --no-daemon 2>/dev/null || true",
            "echo __TOPIC_SPLIT__",
            "ros2 topic list --no-daemon 2>/dev/null || true",
        ]
        result = self._run_bash("; ".join(script_lines), timeout=25)
        output = result.stdout or ""
        nodes_text, _, topics_text = output.partition("__TOPIC_SPLIT__")
        nodes = [line.strip() for line in nodes_text.splitlines() if line.strip()]
        topics = [line.strip() for line in topics_text.splitlines() if line.strip()]
        return nodes, topics

    def _tail_agent_log(self, lines: int = 8) -> list[str]:
        if not self.agent_log.exists():
            return []
        with contextlib.suppress(Exception):
            return self.agent_log.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:]
        return []

    def _agent_disabled(self) -> bool:
        return self.agent_disable_flag.exists()

    def status(self) -> dict[str, Any]:
        now = time.time()
        if self._status_cache and now - self._status_cached_at < self._cache_ttl_s:
            return dict(self._status_cache)

        agent_disabled = self._agent_disabled()
        service_enabled = self._systemctl_state("is-enabled")
        service_active = self._systemctl_state("is-active")
        agent_listening = self._agent_listening()
        nodes, topics = self._ros2_runtime()
        ethernet_ip = self._first_interface_ipv4(["enP4p65s0", "eth0"])
        ivshmem_ip = self._first_interface_ipv4(["enp255s5"])
        wifi_ip = self._first_interface_ipv4(["wlP2p33s0", "wlan0", "wlan1"])
        note = (
            "当前正式 /odom 由 teleop /odom_raw + IMU + EKF 生成，RT micro-ROS agent 已按配置停用，避免 /odom 冲突。"
            if agent_disabled
            else "当前第2步链路为 Linux 上运行 micro_ros_agent，RT 侧优先通过内部 ivshmem 地址执行 microros_chassis udp <板端内部IP> 8888 接入。"
        )
        status = {
            "agent_service": self.agent_service,
            "agent_disabled": agent_disabled,
            "agent_enabled": service_enabled == "enabled",
            "agent_active": False if agent_disabled else service_active == "active",
            "agent_transport": self.agent_transport,
            "agent_port": self.agent_port,
            "agent_listening": False if agent_disabled else agent_listening,
            "agent_target_ip": ivshmem_ip or ethernet_ip or "",
            "ethernet_ip": ethernet_ip or "",
            "ivshmem_ip": ivshmem_ip or "",
            "wifi_ip": wifi_ip or "",
            "microros_ws_exists": self.microros_ws.exists(),
            "rt_node_online": False if agent_disabled else any("rt_chassis" in node for node in nodes),
            "cmd_vel_ready": False if agent_disabled else "/cmd_vel" in topics,
            "odom_ready": False if agent_disabled else "/odom" in topics,
            "ros_nodes": nodes,
            "ros_topics": topics,
            "agent_log_tail": self._tail_agent_log(),
            "bridge_ready": False if agent_disabled else service_active == "active" and agent_listening,
            "note": note,
            "preferred_odom_source": "ekf",
        }
        self._status_cache = dict(status)
        self._status_cached_at = now
        return status


class BoardPlatform:
    def __init__(self):
        self.logger = RuntimeLogger()
        self.controller = SerialMotionController(self.logger)
        self.odom_bridge = SerialOdomBridge(self.logger, self.controller)
        self.vision = VisionSystem(self.logger, self.controller)
        self.lidar = LidarSystem(self.logger)
        self.avoidance = AvoidanceSystem(self.logger, self.controller)
        self.maps = MapAnnotationStore(self.logger)
        self.ros = RosSystem(self.logger, self.controller)
        self.nav = NavigationSystem(self.logger, self.ros, self.maps, self.controller, self.avoidance)
        self.rt_bridge = RtBridgeSystem(self.logger)
        self._snapshot_lock = threading.Lock()
        self._snapshot_stop = threading.Event()
        self._snapshot_refresh_interval_s = 2.0
        self._snapshot_stale_after_s = 8.0
        self._ros_snapshot: dict[str, Any] = self._decorate_snapshot({}, updated_ts=0.0)
        self._rt_bridge_snapshot: dict[str, Any] = self._decorate_snapshot({}, updated_ts=0.0)
        self._snapshot_thread = threading.Thread(target=self._snapshot_worker, daemon=True)
        self._snapshot_thread.start()

    def _decorate_snapshot(self, payload: dict[str, Any], *, updated_ts: float) -> dict[str, Any]:
        snapshot = dict(payload or {})
        now = time.time()
        snapshot["updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(updated_ts)) if updated_ts else ""
        snapshot["updated_ts"] = round(float(updated_ts), 3) if updated_ts else 0.0
        snapshot["age_ms"] = None if not updated_ts else max(0, int((now - updated_ts) * 1000.0))
        snapshot["stale"] = (not updated_ts) or (now - updated_ts > self._snapshot_stale_after_s)
        return snapshot

    def _set_ros_snapshot(self, payload: dict[str, Any], *, updated_ts: float | None = None) -> dict[str, Any]:
        snapshot = self._decorate_snapshot(payload, updated_ts=time.time() if updated_ts is None else float(updated_ts))
        with self._snapshot_lock:
            self._ros_snapshot = snapshot
        return dict(snapshot)

    def _set_rt_bridge_snapshot(self, payload: dict[str, Any], *, updated_ts: float | None = None) -> dict[str, Any]:
        snapshot = self._decorate_snapshot(payload, updated_ts=time.time() if updated_ts is None else float(updated_ts))
        with self._snapshot_lock:
            self._rt_bridge_snapshot = snapshot
        return dict(snapshot)

    def _snapshot_copy(self, snapshot: dict[str, Any]) -> dict[str, Any]:
        updated_ts = float(snapshot.get("updated_ts") or 0.0)
        return self._decorate_snapshot(snapshot, updated_ts=updated_ts)

    def refresh_ros_status(self, force: bool = False) -> dict[str, Any]:
        with self._snapshot_lock:
            current = dict(self._ros_snapshot)
        if current.get("updated_ts") and not force:
            age_ms = current.get("age_ms")
            if isinstance(age_ms, (int, float)) and age_ms < self._snapshot_refresh_interval_s * 1000.0:
                return self._snapshot_copy(current)
        try:
            return self._set_ros_snapshot(self.ros.status())
        except Exception as exc:
            fallback = dict(current)
            fallback["last_ready_error"] = str(exc)
            fallback["error_code"] = fallback.get("error_code") or self.ros.error_code_for(str(exc)) or "ros_status_failed"
            fallback["stale"] = True
            return self._set_ros_snapshot(fallback, updated_ts=float(current.get("updated_ts") or 0.0))

    def refresh_rt_bridge_status(self, force: bool = False) -> dict[str, Any]:
        with self._snapshot_lock:
            current = dict(self._rt_bridge_snapshot)
        if current.get("updated_ts") and not force:
            age_ms = current.get("age_ms")
            if isinstance(age_ms, (int, float)) and age_ms < self._snapshot_refresh_interval_s * 1000.0:
                return self._snapshot_copy(current)
        try:
            return self._set_rt_bridge_snapshot(self.rt_bridge.status())
        except Exception as exc:
            fallback = dict(current)
            fallback["bridge_ready"] = False
            fallback["note"] = str(exc)
            fallback["stale"] = True
            return self._set_rt_bridge_snapshot(fallback, updated_ts=float(current.get("updated_ts") or 0.0))

    def ros_status(self) -> dict[str, Any]:
        with self._snapshot_lock:
            snapshot = dict(self._ros_snapshot)
        if not snapshot.get("updated_ts"):
            return self.refresh_ros_status(force=True)
        return self._snapshot_copy(snapshot)

    def store_ros_status(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._set_ros_snapshot(payload)

    def rt_bridge_status(self) -> dict[str, Any]:
        with self._snapshot_lock:
            snapshot = dict(self._rt_bridge_snapshot)
        if not snapshot.get("updated_ts"):
            return self.refresh_rt_bridge_status(force=True)
        return self._snapshot_copy(snapshot)

    def _snapshot_worker(self):
        while not self._snapshot_stop.is_set():
            self.refresh_ros_status(force=True)
            if self._snapshot_stop.wait(0.2):
                break
            self.refresh_rt_bridge_status(force=True)
            self._snapshot_stop.wait(self._snapshot_refresh_interval_s)

    def all_status(self) -> dict[str, Any]:
        control_status = self.controller.status()
        ros_status = self.ros_status()
        rt_bridge_status = self.rt_bridge_status()
        nav_status = self.nav.status(force=True)
        formal = control_status.get("formal", {})
        serial_ack_ready = bool(formal.get("control_ack_ready"))
        motion_backend = "serial"
        if not serial_ack_ready and rt_bridge_status.get("cmd_vel_ready"):
            motion_backend = "ros_cmd_vel"
        return {
            "control": control_status,
            "vision": self.vision.status(),
            "lidar": self.lidar.status(),
            "avoidance": self.avoidance.status(),
            "ros": ros_status,
            "nav": nav_status,
            "rt_bridge": rt_bridge_status,
            "motion_backend": motion_backend,
            "serial_ack_ready": serial_ack_ready,
            "odom_source": ros_status.get("odom_source") or formal.get("odom_source") or "ekf",
            "nav_state": nav_status.get("nav_state", "idle"),
            "active_task": nav_status.get("active_task"),
            "safety_interlock_reason": nav_status.get("safety_interlock_reason", ""),
            "logs": self.logger.tail(120),
        }


def draw_text_panel(lines: list[str], size: tuple[int, int] = (960, 720)) -> bytes:
    width, height = size
    image = Image.new("RGB", size, (246, 248, 250))
    draw = ImageDraw.Draw(image)
    font = None
    for candidate in DEFAULT_FONT_CANDIDATES:
        path = Path(candidate)
        if path.exists():
            font = ImageFont.truetype(str(path), 26)
            break
    if font is None:
        font = ImageFont.load_default()
    y = 28
    for line in lines:
        draw.text((28, y), line, fill=(40, 52, 64), font=font)
        y += 36
    array = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
    ok, buffer = cv2.imencode(".jpg", array, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    return buffer.tobytes() if ok else b""

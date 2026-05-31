from __future__ import annotations

import math
import os
import queue
import shutil
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import tkinter as tk
from PIL import Image
from serial.tools import list_ports
from tkinter import filedialog, ttk
from tkinter.scrolledtext import ScrolledText

try:
    from PIL import ImageTk
    IMAGETK_IMPORT_ERROR = None
except Exception as exc:
    ImageTk = None
    IMAGETK_IMPORT_ERROR = exc

try:
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import RPLidarA1
    BREEZYSLAM_IMPORT_ERROR = None
except Exception as exc:
    RMHC_SLAM = None
    RPLidarA1 = None
    BREEZYSLAM_IMPORT_ERROR = exc

try:
    from rplidar import RPLidar
    RPLIDAR_IMPORT_ERROR = None
except Exception as exc:
    RPLidar = None
    RPLIDAR_IMPORT_ERROR = exc


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_LIDAR_PORT = "COM16" if os.name == "nt" else "/dev/rplidar"
DEFAULT_LIDAR_BAUDRATE = 115200
DEFAULT_MAP_SIZE_PIXELS = 800
DEFAULT_MAP_SIZE_METERS = 16
DEFAULT_SCAN_VIEW_SIZE = 680
DEFAULT_MAP_VIEW_SIZE = 680
DEFAULT_SCAN_VIEW_DIMS = (900, 680)
DEFAULT_MAP_VIEW_DIMS = (900, 680)
DEFAULT_SCAN_RANGE_MM = 6000
DEFAULT_REFRESH_MS = 100
MAX_VALID_DISTANCE_MM = 12000
POSE_HISTORY_LIMIT = 800
POSE_HISTORY_STEP_MM = 35.0
MAP_EXPORT_DIR = BASE_DIR / "lidar_maps"
MAP_EXPORT_DIR.mkdir(exist_ok=True)


@dataclass
class LidarSlamState:
    running: bool
    status: str
    port: str
    device_info: dict | None
    health: tuple[str, int] | None
    pose_mm: tuple[float, float, float]
    pose_history_mm: list[tuple[float, float]]
    scan_points: list[tuple[float, float, int]]
    map_image: np.ndarray
    scan_count: int
    map_size_pixels: int
    map_size_meters: float
    last_error: str | None = None


class LidarSlamBackend:
    def __init__(
        self,
        *,
        log_callback=None,
        map_size_pixels: int = DEFAULT_MAP_SIZE_PIXELS,
        map_size_meters: float = DEFAULT_MAP_SIZE_METERS,
    ):
        self.log_callback = log_callback
        self.map_size_pixels = int(map_size_pixels)
        self.map_size_meters = float(map_size_meters)
        self.map_resolution = self.map_size_meters / max(self.map_size_pixels, 1)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reset_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._lidar = None
        self._map_bytes = bytearray(self.map_size_pixels * self.map_size_pixels)
        self._slam = None
        self.running = False
        self.status = "\u672a\u542f\u52a8"
        self.device_info: dict | None = None
        self.health: tuple[str, int] | None = None
        self.scan_count = 0
        self.last_error: str | None = None
        self.port = DEFAULT_LIDAR_PORT
        self.pose_mm = self._center_pose()
        self.pose_history_mm: list[tuple[float, float]] = [(self.pose_mm[0], self.pose_mm[1])]
        self.latest_scan: list[tuple[float, float, int]] = []
        self.latest_map = np.full((self.map_size_pixels, self.map_size_pixels), 127, dtype=np.uint8)

    def _center_pose(self) -> tuple[float, float, float]:
        center_mm = self.map_size_meters * 1000.0 / 2.0
        return (center_mm, center_mm, 0.0)

    def _log(self, message: str):
        if self.log_callback is not None:
            self.log_callback(f"[{time.strftime('%H:%M:%S')}] {message}")

    def _set_status(self, message: str, *, error: str | None = None):
        with self._lock:
            self.status = message
            self.last_error = error

    def _create_slam(self):
        if RMHC_SLAM is None or RPLidarA1 is None:
            raise RuntimeError(f"BreezySLAM \u5bfc\u5165\u5931\u8d25: {BREEZYSLAM_IMPORT_ERROR}")
        self._slam = RMHC_SLAM(RPLidarA1(), self.map_size_pixels, self.map_size_meters)
        self._map_bytes = bytearray(self.map_size_pixels * self.map_size_pixels)
        with self._lock:
            self.pose_mm = self._center_pose()
            self.pose_history_mm = [(self.pose_mm[0], self.pose_mm[1])]
            self.latest_scan = []
            self.latest_map = np.full((self.map_size_pixels, self.map_size_pixels), 127, dtype=np.uint8)
            self.scan_count = 0
            self.last_error = None

    def _append_pose_history(self, pose_mm: tuple[float, float, float]):
        current = (float(pose_mm[0]), float(pose_mm[1]))
        if not self.pose_history_mm:
            self.pose_history_mm.append(current)
            return
        previous = self.pose_history_mm[-1]
        if math.hypot(current[0] - previous[0], current[1] - previous[1]) >= POSE_HISTORY_STEP_MM:
            self.pose_history_mm.append(current)
        if len(self.pose_history_mm) > POSE_HISTORY_LIMIT:
            self.pose_history_mm = self.pose_history_mm[-POSE_HISTORY_LIMIT:]

    def start(self, port: str = DEFAULT_LIDAR_PORT) -> bool:
        if self.running:
            return True
        if RPLidar is None:
            self._set_status("RPLidar \u4f9d\u8d56\u7f3a\u5931", error=str(RPLIDAR_IMPORT_ERROR))
            self._log(f"RPLidar \u5bfc\u5165\u5931\u8d25: {RPLIDAR_IMPORT_ERROR}")
            return False
        if RMHC_SLAM is None or RPLidarA1 is None:
            self._set_status("BreezySLAM \u4f9d\u8d56\u7f3a\u5931", error=str(BREEZYSLAM_IMPORT_ERROR))
            self._log(f"BreezySLAM \u5bfc\u5165\u5931\u8d25: {BREEZYSLAM_IMPORT_ERROR}")
            return False

        self.port = port or DEFAULT_LIDAR_PORT
        self._stop_event.clear()
        self._reset_event.clear()
        self.running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        self._set_status(f"\u6b63\u5728\u8fde\u63a5\u96f7\u8fbe {self.port} ...")
        return True

    def stop(self):
        self._stop_event.set()
        lidar = self._lidar
        if lidar is not None:
            for action in (getattr(lidar, "stop", None), getattr(lidar, "stop_motor", None), getattr(lidar, "disconnect", None)):
                if action is None:
                    continue
                try:
                    action()
                except Exception:
                    pass

        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.5)
        self.running = False
        self._set_status("\u96f7\u8fbe\u5df2\u505c\u6b62")

    def reset_map(self):
        self._reset_event.set()
        with self._lock:
            self.pose_mm = self._center_pose()
            self.pose_history_mm = [(self.pose_mm[0], self.pose_mm[1])]
            self.latest_map = np.full((self.map_size_pixels, self.map_size_pixels), 127, dtype=np.uint8)
            self.scan_count = 0
        self._log("\u5df2\u8bf7\u6c42\u91cd\u7f6e SLAM \u5730\u56fe")

    def snapshot(self) -> LidarSlamState:
        with self._lock:
            if isinstance(self.device_info, dict):
                device_info = dict(self.device_info)
            elif self.device_info is None:
                device_info = None
            else:
                device_info = {"raw": str(self.device_info)}
            return LidarSlamState(
                running=self.running,
                status=self.status,
                port=self.port,
                device_info=device_info,
                health=tuple(self.health) if self.health else None,
                pose_mm=tuple(self.pose_mm),
                pose_history_mm=list(self.pose_history_mm),
                scan_points=list(self.latest_scan),
                map_image=self.latest_map.copy(),
                scan_count=self.scan_count,
                map_size_pixels=self.map_size_pixels,
                map_size_meters=self.map_size_meters,
                last_error=self.last_error,
            )

    def save_map(self, base_path: Path) -> dict[str, Path]:
        base_path = Path(base_path)
        base_path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            raw_map = self.latest_map.copy()
            pose_mm = tuple(self.pose_mm)
            scan_count = int(self.scan_count)

        display_map = np.flipud(raw_map)
        ros_map = np.full_like(display_map, 205)
        ros_map[display_map >= 180] = 254
        ros_map[display_map <= 100] = 0

        png_path = base_path.with_suffix(".png")
        pgm_path = base_path.with_suffix(".pgm")
        yaml_path = base_path.with_suffix(".yaml")
        pose_path = base_path.with_suffix(".txt")

        cv2.imwrite(str(png_path), display_map)
        cv2.imwrite(str(pgm_path), ros_map)

        origin = -self.map_size_meters / 2.0
        yaml_text = (
            f"image: {pgm_path.name}\n"
            f"resolution: {self.map_resolution:.6f}\n"
            f"origin: [{origin:.6f}, {origin:.6f}, 0.000000]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
            "mode: trinary\n"
        )
        yaml_path.write_text(yaml_text, encoding="utf-8")
        pose_path.write_text(
            (
                f"scan_count={scan_count}\n"
                f"pose_mm_x={pose_mm[0]:.3f}\n"
                f"pose_mm_y={pose_mm[1]:.3f}\n"
                f"pose_deg={pose_mm[2]:.3f}\n"
            ),
            encoding="utf-8",
        )

        self._log(f"\u5730\u56fe\u5df2\u4fdd\u5b58: {pgm_path.name}, {yaml_path.name}")
        return {"png": png_path, "pgm": pgm_path, "yaml": yaml_path, "pose": pose_path}

    def _worker(self):
        lidar = None
        try:
            self._create_slam()
            lidar = RPLidar(self.port, baudrate=DEFAULT_LIDAR_BAUDRATE, timeout=3)
            self._lidar = lidar
            info = lidar.get_info()
            health = lidar.get_health()
            with self._lock:
                self.device_info = info
                self.health = health
            self._set_status(f"\u96f7\u8fbe\u8fd0\u884c\u4e2d: {self.port}")
            self._log(f"RPLidar \u5df2\u8fde\u63a5: {info}")
            self._log(f"RPLidar \u5065\u5eb7\u72b6\u6001: {health}")

            try:
                lidar.start_motor()
            except Exception:
                pass

            for scan in lidar.iter_scans(max_buf_meas=1000):
                if self._stop_event.is_set():
                    break

                if self._reset_event.is_set():
                    self._create_slam()
                    self._reset_event.clear()
                    self._log("SLAM \u5730\u56fe\u5df2\u91cd\u7f6e")

                points: list[tuple[float, float, int]] = []
                for quality, angle, distance in scan:
                    if distance <= 0 or distance > MAX_VALID_DISTANCE_MM:
                        continue
                    points.append((float(angle), float(distance), int(quality)))

                if len(points) < 12:
                    continue

                points.sort(key=lambda item: item[0])
                distances = [item[1] for item in points]
                angles = [item[0] for item in points]
                self._slam.update(distances, scan_angles_degrees=angles)
                self._slam.getmap(self._map_bytes)
                pose = self._slam.getpos()
                map_image = np.frombuffer(self._map_bytes, dtype=np.uint8).reshape(
                    (self.map_size_pixels, self.map_size_pixels)
                ).copy()

                with self._lock:
                    self.pose_mm = (float(pose[0]), float(pose[1]), float(pose[2]))
                    self._append_pose_history(self.pose_mm)
                    self.latest_scan = points
                    self.latest_map = map_image
                    self.scan_count += 1
                    if self.scan_count % 8 == 0:
                        self.status = f"\u96f7\u8fbe\u8fd0\u884c\u4e2d: {self.port} | \u5df2\u7d2f\u8ba1 {self.scan_count} \u5708"

        except Exception as exc:
            if not self._stop_event.is_set():
                self._set_status("\u96f7\u8fbe/SLAM \u8fd0\u884c\u5931\u8d25", error=str(exc))
                self._log(f"\u96f7\u8fbe/SLAM \u5f02\u5e38: {exc}")
        finally:
            self.running = False
            self._lidar = None
            if lidar is not None:
                for action in (getattr(lidar, "stop", None), getattr(lidar, "stop_motor", None), getattr(lidar, "disconnect", None)):
                    if action is None:
                        continue
                    try:
                        action()
                    except Exception:
                        pass
            if self._stop_event.is_set():
                self._set_status("\u96f7\u8fbe\u5df2\u505c\u6b62")


class LidarSlamWindow:
    def __init__(self, master: tk.Misc):
        self.window = tk.Toplevel(master)
        self.window.title("\u6fc0\u5149\u96f7\u8fbe\u4e0e SLAM \u5efa\u56fe")
        self.window.geometry("1560x980")
        self.window.minsize(1280, 820)

        self.style = ttk.Style(self.window)
        self.style.configure("TLabel", font=("Microsoft YaHei UI", 10))
        self.style.configure("TButton", font=("Microsoft YaHei UI", 10))
        self.style.configure("TLabelframe.Label", font=("Microsoft YaHei UI", 11, "bold"))
        self.style.configure("Header.TLabel", font=("Microsoft YaHei UI", 10, "bold"))
        self.style.configure("Mono.TLabel", font=("Consolas", 10))

        self.log_queue: queue.Queue[str] = queue.Queue()
        self.backend = LidarSlamBackend(log_callback=self.log_queue.put)

        self.port_var = tk.StringVar(value=DEFAULT_LIDAR_PORT)
        self.status_var = tk.StringVar(value="\u672a\u542f\u52a8")
        self.device_var = tk.StringVar(value="\u8bbe\u5907\u4fe1\u606f: \u672a\u8fde\u63a5")
        self.health_var = tk.StringVar(value="\u5065\u5eb7\u72b6\u6001: \u672a\u8fde\u63a5")
        self.pose_var = tk.StringVar(value="\u4f4d\u59ff: x=0.00m  y=0.00m  yaw=0.0deg")
        self.scan_var = tk.StringVar(value="\u626b\u70b9: 0")
        self.map_var = tk.StringVar(value="\u5730\u56fe: 16.0m x 16.0m @ 2.0cm/px")
        self.ros_var = tk.StringVar(value=self.detect_ros_status())

        self.available_ports: list[str] = []
        self.scan_photo = None
        self.map_photo = None

        self.build_ui()
        self.refresh_ports()
        self.render_placeholders()
        self.window.after(DEFAULT_REFRESH_MS, self.refresh_loop)
        self.window.after(120, self.flush_log_queue)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self._log_local("\u96f7\u8fbe\u9875\u9762\u5df2\u6253\u5f00")

    @staticmethod
    def detect_ros_status() -> str:
        ros2 = shutil.which("ros2")
        roscore = shutil.which("roscore")
        if ros2:
            return f"ROS \u73af\u5883: ros2 -> {ros2}"
        if roscore:
            return f"ROS \u73af\u5883: roscore -> {roscore}"
        return "\u672c\u673a\u672a\u68c0\u6d4b\u5230 ROS\uff1b\u5f53\u524d\u9875\u9762\u7528\u4e32\u53e3\u76f4\u8fde + BreezySLAM\uff0c\u5e76\u5bfc\u51fa ROS \u517c\u5bb9\u5730\u56fe"

    def _log_local(self, message: str):
        self.log_queue.put(f"[{time.strftime('%H:%M:%S')}] {message}")

    def build_ui(self):
        self.window.columnconfigure(0, weight=1)
        self.window.rowconfigure(1, weight=1)

        top = ttk.LabelFrame(self.window, text="\u8bbe\u5907\u4e0e\u5efa\u56fe\u63a7\u5236", padding=12)
        top.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 6))
        for col in range(6):
            top.columnconfigure(col, weight=1 if col in (1, 3, 5) else 0)

        ttk.Label(top, text="\u96f7\u8fbe\u4e32\u53e3", style="Header.TLabel").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, state="readonly", width=14)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(8, 12))
        ttk.Button(top, text="\u5237\u65b0\u4e32\u53e3", command=self.refresh_ports).grid(row=0, column=2, sticky="ew")
        ttk.Button(top, text="\u542f\u52a8\u96f7\u8fbe / \u5efa\u56fe", command=self.start_backend).grid(row=0, column=3, sticky="ew", padx=(12, 6))
        ttk.Button(top, text="\u505c\u6b62", command=self.stop_backend).grid(row=0, column=4, sticky="ew", padx=6)
        ttk.Button(top, text="\u91cd\u7f6e\u5730\u56fe", command=self.reset_map).grid(row=0, column=5, sticky="ew", padx=(6, 0))

        ttk.Label(top, textvariable=self.status_var, style="Header.TLabel").grid(
            row=1, column=0, columnspan=3, sticky="w", pady=(10, 0)
        )
        ttk.Button(top, text="\u4fdd\u5b58\u5730\u56fe", command=self.save_map).grid(
            row=1, column=3, sticky="ew", padx=(12, 6), pady=(10, 0)
        )
        ttk.Label(top, textvariable=self.ros_var, wraplength=620, justify="right").grid(
            row=1, column=4, columnspan=2, sticky="e", pady=(10, 0)
        )

        ttk.Label(top, textvariable=self.device_var, wraplength=720, justify="left").grid(
            row=2, column=0, columnspan=3, sticky="w", pady=(8, 0)
        )
        ttk.Label(top, textvariable=self.health_var, wraplength=520, justify="left").grid(
            row=2, column=3, columnspan=3, sticky="w", pady=(8, 0)
        )
        ttk.Label(top, textvariable=self.pose_var, style="Mono.TLabel").grid(
            row=3, column=0, columnspan=3, sticky="w", pady=(8, 0)
        )
        ttk.Label(top, textvariable=self.scan_var).grid(row=3, column=3, sticky="w", pady=(8, 0))
        ttk.Label(top, textvariable=self.map_var).grid(row=3, column=4, columnspan=2, sticky="w", pady=(8, 0))

        main = ttk.Frame(self.window, padding=(10, 0, 10, 10))
        main.grid(row=1, column=0, sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)
        main.rowconfigure(1, weight=0)

        scan_frame = ttk.LabelFrame(main, text="\u5b9e\u65f6\u96f7\u8fbe\u626b\u70b9", padding=10)
        scan_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        scan_frame.columnconfigure(0, weight=1)
        scan_frame.rowconfigure(0, weight=1)
        self.scan_label = ttk.Label(scan_frame, anchor="center")
        self.scan_label.grid(row=0, column=0, sticky="nsew")
        ttk.Label(
            scan_frame,
            text="\u8bf4\u660e\uff1a\u4e0a\u65b9\u4ee3\u8868\u8f66\u5934\u671d\u5411\uff1b\u5706\u73af\u6bcf\u683c\u7ea6 1 \u7c73\uff1b\u70b9\u989c\u8272\u4ece\u7eff\u5230\u6a59\u8868\u793a\u8ddd\u79bb\u4ece\u8fd1\u5230\u8fdc\uff0c\u8d8a\u4eae\u8868\u793a\u8d28\u91cf\u8d8a\u9ad8\u3002",
            wraplength=640,
            justify="left",
        ).grid(row=1, column=0, sticky="w", pady=(8, 0))

        map_frame = ttk.LabelFrame(main, text="SLAM \u5efa\u56fe\u53ef\u89c6\u5316", padding=10)
        map_frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        map_frame.columnconfigure(0, weight=1)
        map_frame.rowconfigure(0, weight=1)
        self.map_label = ttk.Label(map_frame, anchor="center")
        self.map_label.grid(row=0, column=0, sticky="nsew")
        ttk.Label(
            map_frame,
            text="\u8bf4\u660e\uff1a\u6d45\u8272=\u7a7a\u95f2\u533a\u57df\uff0c\u7070\u8272=\u672a\u77e5\u533a\u57df\uff0c\u6df1\u8272=\u969c\u788d\u7269\uff0c\u6a59\u7ebf=\u79fb\u52a8\u8f68\u8ff9\uff0c\u7ea2\u7bad\u5934=\u673a\u5668\u4eba\u5f53\u524d\u4f4d\u7f6e\u4e0e\u671d\u5411\u3002",
            wraplength=640,
            justify="left",
        ).grid(row=1, column=0, sticky="w", pady=(8, 0))

        log_frame = ttk.LabelFrame(main, text="\u8fd0\u884c\u65e5\u5fd7", padding=10)
        log_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(10, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = ScrolledText(log_frame, height=9, wrap="word", state="disabled", font=("Microsoft YaHei UI", 10))
        self.log_text.grid(row=0, column=0, sticky="nsew")

    def render_placeholders(self):
        scan = self.render_scan_image([], size=DEFAULT_SCAN_VIEW_DIMS)
        mapping = self.render_map_image(
            np.full((DEFAULT_MAP_SIZE_PIXELS, DEFAULT_MAP_SIZE_PIXELS), 127, dtype=np.uint8),
            (DEFAULT_MAP_SIZE_METERS * 500.0, DEFAULT_MAP_SIZE_METERS * 500.0, 0.0),
            [],
            size=DEFAULT_MAP_VIEW_DIMS,
            map_size_meters=DEFAULT_MAP_SIZE_METERS,
        )
        self.update_image_label(self.scan_label, scan, "scan")
        self.update_image_label(self.map_label, mapping, "map")

    def refresh_ports(self):
        ports = [port.device for port in list_ports.comports()]
        self.available_ports = ports
        self.port_combo["values"] = ports
        if DEFAULT_LIDAR_PORT in ports:
            self.port_var.set(DEFAULT_LIDAR_PORT)
        elif ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        port_message = ports if ports else "\u672a\u53d1\u73b0\u53ef\u7528\u4e32\u53e3"
        self._log_local(f"\u5df2\u626b\u63cf\u96f7\u8fbe\u4e32\u53e3: {port_message}")

    def start_backend(self):
        port = self.port_var.get().strip()
        if not port:
            self.status_var.set("\u8bf7\u5148\u9009\u62e9\u96f7\u8fbe\u4e32\u53e3")
            return
        if self.backend.start(port):
            self.status_var.set(f"\u6b63\u5728\u542f\u52a8\u96f7\u8fbe\u4e0e\u5efa\u56fe: {port}")
            self._log_local(f"\u5df2\u8bf7\u6c42\u542f\u52a8\u96f7\u8fbe\u4e0e\u5efa\u56fe: {port}")
        else:
            state = self.backend.snapshot()
            self.status_var.set(state.status)

    def stop_backend(self):
        self.backend.stop()
        self.status_var.set("\u96f7\u8fbe\u5df2\u505c\u6b62")
        self._log_local("\u5df2\u505c\u6b62\u96f7\u8fbe\u4e0e\u5efa\u56fe")

    def reset_map(self):
        self.backend.reset_map()
        self.status_var.set("\u5df2\u91cd\u7f6e\u5730\u56fe\uff0c\u7b49\u5f85\u65b0\u7684\u626b\u63cf\u6570\u636e")

    def save_map(self):
        timestamp = time.strftime("map_%Y%m%d_%H%M%S")
        target = filedialog.asksaveasfilename(
            parent=self.window,
            title="\u4fdd\u5b58 ROS \u517c\u5bb9\u5730\u56fe",
            initialdir=str(MAP_EXPORT_DIR),
            initialfile=timestamp,
            defaultextension=".pgm",
            filetypes=[("ROS map", "*.pgm"), ("PNG image", "*.png"), ("All files", "*.*")],
        )
        if not target:
            return
        saved = self.backend.save_map(Path(target).with_suffix(""))
        self.status_var.set(f"\u5730\u56fe\u5df2\u4fdd\u5b58: {saved['pgm'].name}")

    def refresh_loop(self):
        if not self.is_alive():
            return

        state = self.backend.snapshot()
        self.status_var.set(state.status if not state.last_error else f"{state.status} | {state.last_error}")
        device_text = state.device_info if state.device_info else "\u672a\u8fde\u63a5"
        health_text = state.health if state.health else "\u672a\u8fde\u63a5"
        self.device_var.set(f"\u8bbe\u5907\u4fe1\u606f: {device_text}")
        self.health_var.set(f"\u5065\u5eb7\u72b6\u6001: {health_text}")
        self.pose_var.set(
            f"\u4f4d\u59ff: x={state.pose_mm[0] / 1000.0:.2f}m  y={state.pose_mm[1] / 1000.0:.2f}m  yaw={state.pose_mm[2]:.1f}deg"
        )

        distances = [point[1] for point in state.scan_points]
        qualities = [point[2] for point in state.scan_points]
        if distances:
            nearest = min(distances) / 1000.0
            farthest = max(distances) / 1000.0
            avg_quality = float(np.mean(qualities)) if qualities else 0.0
            self.scan_var.set(
                f"\u5f53\u524d\u626b\u70b9: {len(state.scan_points)} | \u7d2f\u8ba1\u626b\u63cf: {state.scan_count} \u5708 | \u6700\u8fd1\u76ee\u6807: {nearest:.2f}m | \u6700\u8fdc: {farthest:.2f}m | \u5e73\u5747\u8d28\u91cf: {avg_quality:.1f}"
            )
        else:
            self.scan_var.set(f"\u5f53\u524d\u626b\u70b9: 0 | \u7d2f\u8ba1\u626b\u63cf: {state.scan_count} \u5708")

        resolution_cm = state.map_size_meters * 100.0 / max(state.map_size_pixels, 1)
        path_length = max(len(state.pose_history_mm) - 1, 0)
        self.map_var.set(
            f"\u5730\u56fe\u8303\u56f4: {state.map_size_meters:.1f}m x {state.map_size_meters:.1f}m | \u5206\u8fa8\u7387: {resolution_cm:.2f}cm/px | \u8f68\u8ff9\u70b9: {path_length}"
        )

        try:
            scan_image = self.render_scan_image(
                state.scan_points,
                size=self._label_dimensions(self.scan_label, DEFAULT_SCAN_VIEW_DIMS),
            )
            map_image = self.render_map_image(
                state.map_image,
                state.pose_mm,
                state.pose_history_mm,
                size=self._label_dimensions(self.map_label, DEFAULT_MAP_VIEW_DIMS),
                map_size_meters=state.map_size_meters,
            )
            self.update_image_label(self.scan_label, scan_image, "scan")
            self.update_image_label(self.map_label, map_image, "map")
        except Exception as exc:
            self._log_local(f"界面渲染异常: {exc}")
        self.window.after(DEFAULT_REFRESH_MS, self.refresh_loop)

    def flush_log_queue(self):
        if not self.is_alive():
            return
        updated = False
        self.log_text.configure(state="normal")
        while not self.log_queue.empty():
            self.log_text.insert("end", self.log_queue.get_nowait() + "\n")
            updated = True
        if updated:
            self.log_text.see("end")
        self.log_text.configure(state="disabled")
        self.window.after(120, self.flush_log_queue)

    @staticmethod
    def _label_dimensions(label: ttk.Label, fallback: int | tuple[int, int]) -> tuple[int, int]:
        if isinstance(fallback, int):
            fallback_width = fallback
            fallback_height = fallback
        else:
            fallback_width, fallback_height = fallback

        current_width = label.winfo_width()
        current_height = label.winfo_height()
        width = current_width if current_width > 24 else fallback_width
        height = current_height if current_height > 24 else fallback_height
        return (
            int(max(260, min(width, 1500))),
            int(max(150, min(height, 1000))),
        )

    @staticmethod
    def _resolve_size(size: int | tuple[int, int], default_square: int) -> tuple[int, int]:
        if isinstance(size, int):
            edge = int(size)
            return max(260, edge), max(180, edge)
        if isinstance(size, tuple) and len(size) == 2:
            return max(260, int(size[0])), max(150, int(size[1]))
        return default_square, default_square

    @staticmethod
    def _distance_in_sector(
        scan_points: list[tuple[float, float, int]],
        start_angle: float,
        end_angle: float,
    ) -> str:
        distances: list[float] = []
        for angle_deg, distance_mm, _quality in scan_points:
            angle = angle_deg % 360.0
            if start_angle <= end_angle:
                in_sector = start_angle <= angle <= end_angle
            else:
                in_sector = angle >= start_angle or angle <= end_angle
            if in_sector:
                distances.append(distance_mm)
        if not distances:
            return "--"
        return f"{min(distances) / 1000.0:.2f} m"

    @staticmethod
    def _build_map_canvas(map_image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        flipped = np.flipud(map_image)
        canvas = np.zeros((flipped.shape[0], flipped.shape[1], 3), dtype=np.uint8)

        free_mask = flipped >= 180
        occupied_mask = flipped <= 100
        unknown_mask = ~(free_mask | occupied_mask)

        canvas[free_mask] = (248, 248, 244)
        canvas[unknown_mask] = (198, 203, 210)
        canvas[occupied_mask] = (36, 48, 62)
        known_mask = free_mask | occupied_mask
        return canvas, known_mask

    def update_image_label(self, label: ttk.Label, image_array: np.ndarray, kind: str):
        if ImageTk is None:
            return
        rgb = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        photo = ImageTk.PhotoImage(image=image)
        label.configure(image=photo)
        if kind == "scan":
            self.scan_photo = photo
        else:
            self.map_photo = photo

    @staticmethod
    def render_scan_image(
        scan_points: list[tuple[float, float, int]],
        *,
        size: int | tuple[int, int] = DEFAULT_SCAN_VIEW_SIZE,
        max_distance_mm: float = DEFAULT_SCAN_RANGE_MM,
    ) -> np.ndarray:
        width, height = LidarSlamWindow._resolve_size(size, DEFAULT_SCAN_VIEW_SIZE)
        canvas = np.full((height, width, 3), (246, 248, 250), dtype=np.uint8)
        cv2.rectangle(canvas, (0, 0), (width - 1, height - 1), (218, 223, 228), 1)

        margin = 14 if min(width, height) < 420 else 22
        sidebar_width = max(180 if width < 760 else 230, int(width * 0.27))
        sidebar_width = min(sidebar_width, max(160, width - margin * 2 - 140))
        panel_left = max(margin + 120, width - sidebar_width - margin)
        cv2.rectangle(canvas, (panel_left, margin), (width - margin, height - margin), (238, 241, 244), -1)
        cv2.rectangle(canvas, (panel_left, margin), (width - margin, height - margin), (214, 220, 226), 1)

        plot_left = margin
        plot_top = margin
        plot_right = panel_left - 12
        plot_bottom = height - margin
        plot_size = min(max(80, plot_right - plot_left), max(80, plot_bottom - plot_top))
        plot_left += max(0, (plot_right - plot_left - plot_size) // 2)
        plot_top += max(0, (plot_bottom - plot_top - plot_size) // 2)
        plot_right = plot_left + plot_size
        plot_bottom = plot_top + plot_size

        cv2.rectangle(canvas, (plot_left, plot_top), (plot_right, plot_bottom), (252, 253, 254), -1)
        cv2.rectangle(canvas, (plot_left, plot_top), (plot_right, plot_bottom), (214, 220, 226), 1)

        center_x = plot_left + plot_size // 2
        center_y = plot_top + plot_size // 2
        radius = int(plot_size * 0.43)
        scale = radius / max(max_distance_mm, 1.0)
        meters = max(1, int(max_distance_mm // 1000))

        for meter in range(-meters, meters + 1):
            offset = int(meter * 1000.0 * scale)
            line_color = (228, 232, 237) if meter != 0 else (160, 169, 178)
            cv2.line(canvas, (center_x + offset, plot_top + 12), (center_x + offset, plot_bottom - 12), line_color, 1, lineType=cv2.LINE_AA)
            cv2.line(canvas, (plot_left + 12, center_y + offset), (plot_right - 12, center_y + offset), line_color, 1, lineType=cv2.LINE_AA)
            if meter > 0 and abs(offset) <= radius:
                cv2.putText(canvas, f"{meter}m", (center_x + offset + 4, plot_top + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (88, 102, 118), 1)

        cv2.circle(canvas, (center_x, center_y), radius, (184, 194, 205), 1, lineType=cv2.LINE_AA)
        cv2.putText(canvas, 'FRONT', (center_x - 36, plot_top + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (56, 72, 88), 2)
        cv2.putText(canvas, 'REAR', (center_x - 30, plot_bottom - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (56, 72, 88), 2)
        cv2.putText(canvas, 'L', (plot_left + 10, center_y + 7), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (56, 72, 88), 2)
        cv2.putText(canvas, 'R', (plot_right - 20, center_y + 7), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (56, 72, 88), 2)

        nearest_point = None
        transformed_points: list[tuple[int, int, float, int]] = []
        for angle_deg, distance_mm, quality in scan_points:
            clipped_distance = min(distance_mm, max_distance_mm)
            radians = math.radians(angle_deg)
            x_mm = math.sin(radians) * clipped_distance
            y_mm = math.cos(radians) * clipped_distance
            px = int(center_x + x_mm * scale)
            py = int(center_y - y_mm * scale)
            if px < plot_left or px > plot_right or py < plot_top or py > plot_bottom:
                continue
            transformed_points.append((px, py, clipped_distance, quality))
            if nearest_point is None or distance_mm < nearest_point[1]:
                nearest_point = (angle_deg, distance_mm, px, py)

        for px, py, distance_mm, quality in transformed_points:
            distance_ratio = distance_mm / max(max_distance_mm, 1.0)
            quality_ratio = min(max(quality / 15.0, 0.0), 1.0)
            color = (
                int(46 + 92 * distance_ratio),
                int(150 + 80 * quality_ratio),
                int(255 - 110 * distance_ratio),
            )
            radius_px = 2 if quality < 10 else 3
            cv2.circle(canvas, (px, py), radius_px, color, -1, lineType=cv2.LINE_AA)

        robot = np.array([[center_x, center_y - 18], [center_x - 15, center_y + 14], [center_x + 15, center_y + 14]], dtype=np.int32)
        cv2.fillConvexPoly(canvas, robot, (41, 55, 73), lineType=cv2.LINE_AA)
        cv2.circle(canvas, (center_x, center_y), 4, (0, 149, 255), -1, lineType=cv2.LINE_AA)

        summary_y = margin + 34
        cv2.putText(canvas, 'Top-Down Scan View', (panel_left + 14, summary_y), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (45, 59, 73), 2)

        info_rows = [
            ('Points', str(len(scan_points))),
            ('Nearest', '--' if nearest_point is None else f"{nearest_point[1] / 1000.0:.2f} m"),
            ('Front', LidarSlamWindow._distance_in_sector(scan_points, 340.0, 20.0)),
            ('Left', LidarSlamWindow._distance_in_sector(scan_points, 20.0, 100.0)),
            ('Right', LidarSlamWindow._distance_in_sector(scan_points, 260.0, 340.0)),
            ('Rear', LidarSlamWindow._distance_in_sector(scan_points, 160.0, 200.0)),
        ]

        row_y = summary_y + 28
        for label, value in info_rows:
            cv2.rectangle(canvas, (panel_left + 14, row_y - 18), (width - margin - 14, row_y + 18), (250, 251, 252), -1)
            cv2.rectangle(canvas, (panel_left + 14, row_y - 18), (width - margin - 14, row_y + 18), (222, 227, 232), 1)
            cv2.putText(canvas, label, (panel_left + 24, row_y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (88, 101, 115), 1)
            cv2.putText(canvas, value, (panel_left + 120, row_y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (35, 48, 62), 2)
            row_y += 48

        legend_y = min(height - 94, row_y + 8)
        cv2.putText(canvas, 'Legend', (panel_left + 14, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (45, 59, 73), 2)
        cv2.circle(canvas, (panel_left + 28, legend_y + 28), 6, (42, 214, 220), -1, lineType=cv2.LINE_AA)
        cv2.putText(canvas, 'Near / strong return', (panel_left + 44, legend_y + 33), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (74, 88, 102), 1)
        cv2.circle(canvas, (panel_left + 28, legend_y + 56), 6, (128, 174, 200), -1, lineType=cv2.LINE_AA)
        cv2.putText(canvas, 'Far / weak return', (panel_left + 44, legend_y + 61), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (74, 88, 102), 1)
        cv2.putText(canvas, 'Robot stays at the center of the plot.', (panel_left + 14, legend_y + 88), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (92, 105, 118), 1)

        if nearest_point is not None:
            _nearest_angle, nearest_distance_mm, px, py = nearest_point
            cv2.line(canvas, (center_x, center_y), (px, py), (0, 166, 255), 2, lineType=cv2.LINE_AA)
            cv2.circle(canvas, (px, py), 8, (0, 166, 255), 2, lineType=cv2.LINE_AA)
            cv2.putText(canvas, f"Nearest obstacle: {nearest_distance_mm / 1000.0:.2f} m", (plot_left + 14, plot_bottom - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (41, 55, 73), 2)
        else:
            cv2.putText(canvas, 'Waiting for lidar data...', (plot_left + plot_size // 2 - 112, center_y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (118, 128, 138), 2)

        return canvas

    def render_map_image(
        map_image: np.ndarray,
        pose_mm: tuple[float, float, float],
        pose_history_mm: list[tuple[float, float]],
        *,
        size: int | tuple[int, int] = DEFAULT_MAP_VIEW_SIZE,
        map_size_meters: float | tuple[float, float] = DEFAULT_MAP_SIZE_METERS,
    ) -> np.ndarray:
        width, height = LidarSlamWindow._resolve_size(size, DEFAULT_MAP_VIEW_SIZE)
        map_height_px = max(int(map_image.shape[0]), 1)
        map_width_px = max(int(map_image.shape[1]), 1)
        if isinstance(map_size_meters, (tuple, list)):
            map_width_meters = max(float(map_size_meters[0]), 0.1)
            map_height_meters = max(float(map_size_meters[1]), 0.1)
        else:
            map_width_meters = max(float(map_size_meters), 0.1)
            map_height_meters = map_width_meters
        mm_per_pixel_x = (map_width_meters * 1000.0) / map_width_px
        mm_per_pixel_y = (map_height_meters * 1000.0) / map_height_px

        full_canvas, known_mask = LidarSlamWindow._build_map_canvas(map_image)

        history_points_px: list[tuple[int, int]] = []
        for x_mm, y_mm in pose_history_mm:
            px = int(np.clip(x_mm / mm_per_pixel_x, 0, map_width_px - 1))
            py = int(np.clip(map_height_px - 1 - (y_mm / mm_per_pixel_y), 0, map_height_px - 1))
            history_points_px.append((px, py))

        pose_px = int(np.clip(pose_mm[0] / mm_per_pixel_x, 0, map_width_px - 1))
        pose_py = int(np.clip(map_height_px - 1 - (pose_mm[1] / mm_per_pixel_y), 0, map_height_px - 1))

        ys, xs = np.where(known_mask)
        all_xs = xs.tolist()
        all_ys = ys.tolist()
        all_xs.extend(point[0] for point in history_points_px)
        all_ys.extend(point[1] for point in history_points_px)
        all_xs.append(pose_px)
        all_ys.append(pose_py)

        if all_xs and all_ys:
            min_x = max(0, min(all_xs))
            max_x = min(map_width_px - 1, max(all_xs))
            min_y = max(0, min(all_ys))
            max_y = min(map_height_px - 1, max(all_ys))
        else:
            min_x = map_width_px // 4
            max_x = map_width_px * 3 // 4
            min_y = map_height_px // 4
            max_y = map_height_px * 3 // 4

        margin_px = max(28, max(map_width_px, map_height_px) // 10)
        span = max(max_x - min_x + 1, max_y - min_y + 1, min(map_width_px, map_height_px) // 5)
        span = min(span + margin_px * 2, map_width_px, map_height_px)
        center_x = (min_x + max_x) // 2
        center_y = (min_y + max_y) // 2
        left = int(np.clip(center_x - span // 2, 0, max(0, map_width_px - span)))
        top = int(np.clip(center_y - span // 2, 0, max(0, map_height_px - span)))
        right = left + span
        bottom = top + span

        crop = full_canvas[top:bottom, left:right].copy()

        margin = 14 if min(width, height) < 420 else 22
        sidebar_width = max(180 if width < 760 else 250, int(width * 0.26))
        sidebar_width = min(sidebar_width, max(160, width - margin * 2 - 140))
        view_size = min(
            max(96, width - sidebar_width - margin * 3),
            max(96, height - margin * 2),
        )
        view_x = margin
        view_y = max(margin, (height - view_size) // 2)

        global_view = cv2.resize(full_canvas, (view_size, view_size), interpolation=cv2.INTER_NEAREST)
        global_scale_x = view_size / map_width_px
        global_scale_y = view_size / map_height_px
        theta = math.radians(pose_mm[2])

        if len(history_points_px) >= 2:
            global_path = np.array(
                [(int(px * global_scale_x), int(py * global_scale_y)) for px, py in history_points_px],
                dtype=np.int32,
            )
            cv2.polylines(global_view, [global_path], False, (0, 170, 255), max(2, view_size // 240), lineType=cv2.LINE_AA)

        known_left = int(min_x * global_scale_x)
        known_top = int(min_y * global_scale_y)
        known_right = int(max_x * global_scale_x)
        known_bottom = int(max_y * global_scale_y)
        cv2.rectangle(global_view, (known_left, known_top), (known_right, known_bottom), (0, 166, 255), 2)

        global_robot = (int(pose_px * global_scale_x), int(pose_py * global_scale_y))
        global_head = (
            int(global_robot[0] + math.sin(theta) * max(14, view_size // 18)),
            int(global_robot[1] - math.cos(theta) * max(14, view_size // 18)),
        )
        cv2.circle(global_view, global_robot, max(4, view_size // 100), (36, 82, 255), -1, lineType=cv2.LINE_AA)
        cv2.arrowedLine(global_view, global_robot, global_head, (36, 82, 255), max(2, view_size // 260), line_type=cv2.LINE_AA, tipLength=0.32)

        cropped_view = cv2.resize(crop, (view_size, view_size), interpolation=cv2.INTER_NEAREST)

        mm_per_pixel = max((mm_per_pixel_x + mm_per_pixel_y) / 2.0, 1.0)
        scaled_grid = int(round((1000.0 / mm_per_pixel) * (view_size / span)))
        if scaled_grid >= 26:
            for index in range(0, view_size, scaled_grid):
                cv2.line(cropped_view, (index, 0), (index, view_size - 1), (218, 224, 229), 1, lineType=cv2.LINE_AA)
                cv2.line(cropped_view, (0, index), (view_size - 1, index), (218, 224, 229), 1, lineType=cv2.LINE_AA)

        if len(history_points_px) >= 2:
            scaled_path = []
            for px, py in history_points_px:
                sx = int((px - left) * view_size / span)
                sy = int((py - top) * view_size / span)
                scaled_path.append((sx, sy))
            cv2.polylines(cropped_view, [np.array(scaled_path, dtype=np.int32)], False, (0, 170, 255), max(2, view_size // 220), lineType=cv2.LINE_AA)

        robot_x = int((pose_px - left) * view_size / span)
        robot_y = int((pose_py - top) * view_size / span)
        arrow_length = max(22, view_size // 14)
        head = (int(robot_x + math.sin(theta) * arrow_length), int(robot_y - math.cos(theta) * arrow_length))
        cv2.circle(cropped_view, (robot_x, robot_y), max(5, view_size // 95), (36, 82, 255), -1, lineType=cv2.LINE_AA)
        cv2.arrowedLine(cropped_view, (robot_x, robot_y), head, (36, 82, 255), max(2, view_size // 240), line_type=cv2.LINE_AA, tipLength=0.32)

        canvas = np.full((height, width, 3), (246, 248, 250), dtype=np.uint8)
        cv2.rectangle(canvas, (0, 0), (width - 1, height - 1), (218, 223, 228), 1)
        canvas[view_y:view_y + view_size, view_x:view_x + view_size] = global_view
        cv2.rectangle(canvas, (view_x, view_y), (view_x + view_size, view_y + view_size), (214, 220, 226), 1)

        sidebar_left = view_x + view_size + margin
        cv2.rectangle(canvas, (sidebar_left, margin), (width - margin, height - margin), (238, 241, 244), -1)
        cv2.rectangle(canvas, (sidebar_left, margin), (width - margin, height - margin), (214, 220, 226), 1)

        overview_size = min(
            max(72, width - sidebar_left - margin * 2),
            max(72, height - margin * 2 - 40),
            max(80, int(height * 0.34)),
        )
        overview_x = sidebar_left + 16
        overview_y = margin + 34
        canvas[overview_y:overview_y + overview_size, overview_x:overview_x + overview_size] = cv2.resize(cropped_view, (overview_size, overview_size), interpolation=cv2.INTER_NEAREST)
        cv2.rectangle(canvas, (overview_x, overview_y), (overview_x + overview_size, overview_y + overview_size), (214, 220, 226), 1)

        cv2.putText(canvas, 'Global Map View', (view_x + 12, view_y + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (45, 59, 73), 2)
        cv2.putText(canvas, 'Local Tracking View', (overview_x, overview_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (45, 59, 73), 2)

        known_ratio = float(np.count_nonzero(known_mask)) / float(map_height_px * map_width_px)
        resolution_cm_x = map_width_meters * 100.0 / max(map_width_px, 1)
        resolution_cm_y = map_height_meters * 100.0 / max(map_height_px, 1)
        resolution_text = (
            f"{resolution_cm_x:.2f} cm/px"
            if abs(resolution_cm_x - resolution_cm_y) < 0.01
            else f"{resolution_cm_x:.2f} x {resolution_cm_y:.2f} cm/px"
        )
        info_rows = [
            ('Map size', f"{map_width_meters:.1f} x {map_height_meters:.1f} m"),
            ('Resolution', resolution_text),
            ('Known area', f"{known_ratio * 100.0:.1f} %"),
            ('Known span', f"{(max_x - min_x + 1) * resolution_cm_x / 100.0:.2f} x {(max_y - min_y + 1) * resolution_cm_y / 100.0:.2f} m"),
            ('Path points', str(max(len(pose_history_mm) - 1, 0))),
            ('Pose', f"{pose_mm[0] / 1000.0:.2f}, {pose_mm[1] / 1000.0:.2f} m"),
            ('Yaw', f"{pose_mm[2]:.1f} deg"),
        ]

        row_y = overview_y + overview_size + 34
        for label, value in info_rows:
            cv2.rectangle(canvas, (sidebar_left + 14, row_y - 18), (width - margin - 14, row_y + 18), (250, 251, 252), -1)
            cv2.rectangle(canvas, (sidebar_left + 14, row_y - 18), (width - margin - 14, row_y + 18), (222, 227, 232), 1)
            cv2.putText(canvas, label, (sidebar_left + 24, row_y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (88, 101, 115), 1)
            cv2.putText(canvas, value, (sidebar_left + 132, row_y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (35, 48, 62), 2)
            row_y += 44

        legend_y = min(height - 92, row_y + 8)
        cv2.putText(canvas, 'Legend', (sidebar_left + 14, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (45, 59, 73), 2)
        legends = [
            ((248, 248, 244), 'Free space'),
            ((198, 203, 210), 'Unknown'),
            ((36, 48, 62), 'Obstacle'),
            ((0, 170, 255), 'Path'),
            ((36, 82, 255), 'Current pose'),
        ]
        for index, (color, label) in enumerate(legends):
            y = legend_y + 26 + index * 22
            cv2.rectangle(canvas, (sidebar_left + 18, y - 12), (sidebar_left + 34, y + 4), color, -1)
            cv2.rectangle(canvas, (sidebar_left + 18, y - 12), (sidebar_left + 34, y + 4), (176, 182, 188), 1)
            cv2.putText(canvas, label, (sidebar_left + 44, y), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (74, 88, 102), 1)

        return canvas

    def is_alive(self) -> bool:
        try:
            return bool(self.window.winfo_exists())
        except tk.TclError:
            return False

    def focus(self):
        if self.is_alive():
            self.window.deiconify()
            self.window.lift()
            self.window.focus_force()

    def close(self):
        self.backend.stop()
        if self.is_alive():
            self.window.destroy()

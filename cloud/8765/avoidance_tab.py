from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass

import cv2
import tkinter as tk
from PIL import Image
from serial.tools import list_ports
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

try:
    from PIL import ImageTk
    IMAGETK_IMPORT_ERROR = None
except Exception as exc:
    ImageTk = None
    IMAGETK_IMPORT_ERROR = exc

from lidar_slam_page import DEFAULT_LIDAR_PORT, DEFAULT_REFRESH_MS, DEFAULT_SCAN_VIEW_DIMS, LidarSlamWindow

try:
    from rplidar import RPLidar

    RPLIDAR_IMPORT_ERROR = None
except Exception as exc:
    RPLidar = None
    RPLIDAR_IMPORT_ERROR = exc


DEFAULT_LIDAR_BAUDRATE = 115200
DEFAULT_AVOIDANCE_THRESHOLD_MM = 50
MIN_THRESHOLD_MM = 10
MAX_THRESHOLD_MM = 1000
MAX_VALID_DISTANCE_MM = 12000

COMMAND_FORWARD = "$QJ!"
COMMAND_BACKWARD = "$HT!"
COMMAND_LEFT = "$ZZ!"
COMMAND_RIGHT = "$YZ!"
COMMAND_STOP = "$TZ!"


@dataclass
class AvoidanceState:
    running: bool
    status: str
    port: str
    scan_points: list[tuple[float, float, int]]
    scan_count: int
    front_mm: float | None
    front_left_mm: float | None
    front_right_mm: float | None
    left_mm: float | None
    right_mm: float | None
    rear_mm: float | None
    threshold_mm: int
    device_info: dict | None
    health: tuple[str, int] | None
    avoidance_enabled: bool
    current_action: str
    last_error: str | None = None


class LidarAvoidanceBackend:
    def __init__(self, *, log_callback=None, motion_callback=None):
        self.log_callback = log_callback
        self.motion_callback = motion_callback
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._control_thread: threading.Thread | None = None
        self._lidar = None

        self.running = False
        self.port = DEFAULT_LIDAR_PORT
        self.status = "未启动"
        self.last_error: str | None = None
        self.scan_points: list[tuple[float, float, int]] = []
        self.scan_count = 0
        self.device_info: dict | None = None
        self.health: tuple[str, int] | None = None
        self.front_mm: float | None = None
        self.front_left_mm: float | None = None
        self.front_right_mm: float | None = None
        self.left_mm: float | None = None
        self.right_mm: float | None = None
        self.rear_mm: float | None = None
        self.threshold_mm = DEFAULT_AVOIDANCE_THRESHOLD_MM
        self.avoidance_enabled = False
        self.current_action = "待命"
        self._last_motion_command: str | None = None
        self._last_scan_at = 0.0

    def _log(self, message: str):
        if self.log_callback is not None:
            self.log_callback(f"[{time.strftime('%H:%M:%S')}] {message}")

    def _set_status(self, message: str, *, error: str | None = None):
        with self._lock:
            self.status = message
            self.last_error = error

    def set_threshold_mm(self, value_mm: int):
        clamped = max(MIN_THRESHOLD_MM, min(MAX_THRESHOLD_MM, int(value_mm)))
        with self._lock:
            self.threshold_mm = clamped

    @staticmethod
    def _distance_in_sector(
        scan_points: list[tuple[float, float, int]],
        start_angle: float,
        end_angle: float,
    ) -> float | None:
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
            return None
        return float(min(distances))

    def _update_sector_metrics(self, scan_points: list[tuple[float, float, int]]):
        self.front_mm = self._distance_in_sector(scan_points, 345.0, 15.0)
        self.front_left_mm = self._distance_in_sector(scan_points, 15.0, 75.0)
        self.left_mm = self._distance_in_sector(scan_points, 75.0, 135.0)
        self.rear_mm = self._distance_in_sector(scan_points, 165.0, 195.0)
        self.right_mm = self._distance_in_sector(scan_points, 225.0, 285.0)
        self.front_right_mm = self._distance_in_sector(scan_points, 285.0, 345.0)

    def start(self, port: str = DEFAULT_LIDAR_PORT) -> bool:
        if self.running:
            return True
        if RPLidar is None:
            self._set_status("RPLidar 依赖缺失", error=str(RPLIDAR_IMPORT_ERROR))
            self._log(f"RPLidar 导入失败: {RPLIDAR_IMPORT_ERROR}")
            return False

        self.port = port or DEFAULT_LIDAR_PORT
        self._stop_event.clear()
        self.running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        self._ensure_control_thread()
        self._set_status(f"正在连接雷达 {self.port} ...")
        return True

    def _ensure_control_thread(self):
        if self._control_thread is not None and self._control_thread.is_alive():
            return
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

    def stop(self):
        self.avoidance_enabled = False
        self.current_action = "待命"
        self._stop_motion(force=True)
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
        self._set_status("雷达已停止")

    def start_avoidance(self) -> tuple[bool, str]:
        if not self.running:
            return False, "请先启动雷达"
        self.avoidance_enabled = True
        self.current_action = "避障已启动"
        self._ensure_control_thread()
        self._log("避障测试已启动")
        return True, "避障测试运行中"

    def stop_avoidance(self) -> tuple[bool, str]:
        self.avoidance_enabled = False
        self.current_action = "避障已停止"
        self._stop_motion(force=True)
        self._log("避障测试已停止")
        return True, "避障测试已停止"

    def _worker(self):
        lidar = None
        try:
            lidar = RPLidar(self.port, baudrate=DEFAULT_LIDAR_BAUDRATE, timeout=3)
            self._lidar = lidar
            info = lidar.get_info()
            health = lidar.get_health()
            with self._lock:
                self.device_info = info
                self.health = health
            self._set_status(f"雷达运行中: {self.port}")
            self._log(f"RPLidar 已连接: {info}")
            self._log(f"RPLidar 健康状态: {health}")

            try:
                lidar.start_motor()
            except Exception:
                pass

            for scan in lidar.iter_scans(max_buf_meas=1000):
                if self._stop_event.is_set():
                    break

                points: list[tuple[float, float, int]] = []
                for quality, angle, distance in scan:
                    if distance <= 0 or distance > MAX_VALID_DISTANCE_MM:
                        continue
                    points.append((float(angle), float(distance), int(quality)))

                if len(points) < 5:
                    continue

                points.sort(key=lambda item: item[0])
                with self._lock:
                    self.scan_points = points
                    self.scan_count += 1
                    self._last_scan_at = time.time()
                    self._update_sector_metrics(points)
                    if self.scan_count % 10 == 0:
                        self.status = f"雷达运行中: {self.port} | 已累计 {self.scan_count} 圈"
        except Exception as exc:
            if not self._stop_event.is_set():
                self._set_status("雷达运行失败", error=str(exc))
                self._log(f"雷达异常: {exc}")
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

    def _control_loop(self):
        while not self._stop_event.is_set():
            if not self.avoidance_enabled:
                time.sleep(0.08)
                continue

            with self._lock:
                front = self.front_mm
                front_left = self.front_left_mm
                front_right = self.front_right_mm
                left = self.left_mm
                right = self.right_mm
                rear = self.rear_mm
                last_scan_at = self._last_scan_at
                threshold_mm = self.threshold_mm

            if time.time() - last_scan_at > 1.0:
                self.current_action = "等待雷达数据"
                self._stop_motion()
                time.sleep(0.1)
                continue

            command, action = self._decide_motion(
                front=front,
                front_left=front_left,
                front_right=front_right,
                left=left,
                right=right,
                rear=rear,
                threshold_mm=threshold_mm,
            )
            self.current_action = action
            self._issue_motion(command)
            time.sleep(0.1)

    def _decide_motion(
        self,
        *,
        front: float | None,
        front_left: float | None,
        front_right: float | None,
        left: float | None,
        right: float | None,
        rear: float | None,
        threshold_mm: int,
    ) -> tuple[str, str]:
        def min_value(*values: float | None) -> float | None:
            valid = [value for value in values if value is not None]
            return min(valid) if valid else None

        front_block = min_value(front, front_left, front_right)
        left_space = min_value(left, front_left)
        right_space = min_value(right, front_right)
        threshold_cm = threshold_mm / 10.0

        if front_block is None:
            return COMMAND_STOP, "前方没有有效数据，停车等待"
        if front_block > threshold_mm:
            return COMMAND_FORWARD, f"前方安全，大于 {threshold_cm:.1f} cm，继续前进"

        left_score = left_space if left_space is not None else -1.0
        right_score = right_space if right_space is not None else -1.0
        if left_score > threshold_mm or right_score > threshold_mm:
            if left_score >= right_score:
                return COMMAND_LEFT, f"前方小于 {threshold_cm:.1f} cm，向左避障"
            return COMMAND_RIGHT, f"前方小于 {threshold_cm:.1f} cm，向右避障"

        rear_score = rear if rear is not None else -1.0
        if rear_score > threshold_mm:
            return COMMAND_BACKWARD, f"左右都不通，后退脱困（阈值 {threshold_cm:.1f} cm）"

        return COMMAND_STOP, f"四周都小于 {threshold_cm:.1f} cm，紧急停车"

    def _issue_motion(self, command: str):
        if command == self._last_motion_command:
            return
        self._last_motion_command = command
        if self.motion_callback is not None:
            self.motion_callback(command, self.current_action)

    def _stop_motion(self, *, force: bool = False):
        if not force and self._last_motion_command == COMMAND_STOP:
            return
        self._last_motion_command = COMMAND_STOP
        if self.motion_callback is not None:
            self.motion_callback(COMMAND_STOP, "避障停止，发送停车")

    def snapshot(self) -> AvoidanceState:
        with self._lock:
            if isinstance(self.device_info, dict):
                device_info = dict(self.device_info)
            elif self.device_info is None:
                device_info = None
            else:
                device_info = {"raw": str(self.device_info)}
            return AvoidanceState(
                running=self.running,
                status=self.status,
                port=self.port,
                scan_points=list(self.scan_points),
                scan_count=self.scan_count,
                front_mm=self.front_mm,
                front_left_mm=self.front_left_mm,
                front_right_mm=self.front_right_mm,
                left_mm=self.left_mm,
                right_mm=self.right_mm,
                rear_mm=self.rear_mm,
                threshold_mm=self.threshold_mm,
                device_info=device_info,
                health=tuple(self.health) if self.health else None,
                avoidance_enabled=self.avoidance_enabled,
                current_action=self.current_action,
                last_error=self.last_error,
            )


class ObstacleAvoidanceTab(ttk.Frame):
    def __init__(self, master: tk.Misc, controller):
        super().__init__(master)
        self.controller = controller
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.backend = LidarAvoidanceBackend(
            log_callback=self.log_queue.put,
            motion_callback=self.enqueue_motion_command,
        )

        self.port_var = tk.StringVar(value=DEFAULT_LIDAR_PORT)
        self.status_var = tk.StringVar(value="未启动")
        self.serial_hint_var = controller.serial_status_var
        self.action_var = tk.StringVar(value="当前动作: 待命")
        self.scan_info_var = tk.StringVar(value="扫点: 0")
        self.front_var = tk.StringVar(value="前方: --")
        self.left_var = tk.StringVar(value="左侧: --")
        self.right_var = tk.StringVar(value="右侧: --")
        self.rear_var = tk.StringVar(value="后方: --")
        self.device_var = tk.StringVar(value="设备信息: 未连接")
        self.health_var = tk.StringVar(value="健康状态: 未连接")
        self.threshold_cm_var = tk.IntVar(value=DEFAULT_AVOIDANCE_THRESHOLD_MM // 10)
        self.threshold_text_var = tk.StringVar()

        self.available_ports: list[str] = []
        self.port_combo: ttk.Combobox | None = None
        self.scan_photo = None
        self._threshold_syncing = False

        self.build_ui()
        self.refresh_ports()
        self._sync_threshold_from_ui()
        self.render_placeholder()
        self.after(DEFAULT_REFRESH_MS, self.refresh_loop)
        self.after(120, self.flush_log_queue)

    def build_ui(self):
        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=2)
        self.rowconfigure(0, weight=1)

        left = ttk.Frame(self, padding=(10, 10, 5, 10))
        left.grid(row=0, column=0, sticky="nsew")
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=1)

        ttk.Label(left, text="实时避障雷达图", font=("Microsoft YaHei UI", 14, "bold")).grid(row=0, column=0, sticky="w")
        self.scan_label = ttk.Label(left, anchor="center")
        self.scan_label.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        ttk.Label(
            left,
            text="最简单避障逻辑：前方小于阈值就转向，左右都不通则后退，仍不通则停车。",
            wraplength=760,
            justify="left",
        ).grid(row=2, column=0, sticky="w", pady=(8, 0))

        right = ttk.Frame(self, padding=(5, 10, 10, 10))
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(3, weight=1)

        control_frame = ttk.LabelFrame(right, text="设备与测试控制", padding=10)
        control_frame.grid(row=0, column=0, sticky="ew")
        for col in range(3):
            control_frame.columnconfigure(col, weight=1 if col == 1 else 0)

        ttk.Label(control_frame, text="雷达串口").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(8, 8))
        ttk.Button(control_frame, text="刷新串口", command=self.refresh_ports).grid(row=0, column=2, sticky="ew")

        ttk.Label(control_frame, text="避障阈值 (cm)").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Spinbox(
            control_frame,
            from_=1,
            to=100,
            increment=1,
            textvariable=self.threshold_cm_var,
            width=8,
            command=self._sync_threshold_from_ui,
        ).grid(row=1, column=1, sticky="w", padx=(8, 8), pady=(10, 0))
        ttk.Button(control_frame, text="应用阈值", command=self._sync_threshold_from_ui).grid(row=1, column=2, sticky="ew", pady=(10, 0))

        button_bar = ttk.Frame(control_frame)
        button_bar.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        for col in range(2):
            button_bar.columnconfigure(col, weight=1)
        ttk.Button(button_bar, text="启动雷达", command=self.start_lidar).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(button_bar, text="停止雷达", command=self.stop_lidar).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        button_bar2 = ttk.Frame(control_frame)
        button_bar2.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        for col in range(2):
            button_bar2.columnconfigure(col, weight=1)
        ttk.Button(button_bar2, text="启动避障测试", command=self.start_avoidance).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(button_bar2, text="停止避障测试", command=self.stop_avoidance).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        ttk.Label(control_frame, textvariable=self.threshold_text_var).grid(row=4, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Label(control_frame, textvariable=self.status_var, wraplength=420, justify="left").grid(row=5, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Label(control_frame, textvariable=self.action_var, wraplength=420, justify="left").grid(row=6, column=0, columnspan=3, sticky="w", pady=(4, 0))
        ttk.Label(control_frame, textvariable=self.serial_hint_var, wraplength=420, justify="left").grid(row=7, column=0, columnspan=3, sticky="w", pady=(4, 0))
        ttk.Label(
            control_frame,
            text="提示：此页会直接占用雷达串口。开始避障测试前，请先停止“雷达与 ROS SLAM”页中的雷达。",
            wraplength=420,
            justify="left",
        ).grid(row=8, column=0, columnspan=3, sticky="w", pady=(8, 0))

        data_frame = ttk.LabelFrame(right, text="扇区距离", padding=10)
        data_frame.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        for col in range(2):
            data_frame.columnconfigure(col, weight=1)
        ttk.Label(data_frame, textvariable=self.front_var).grid(row=0, column=0, sticky="w")
        ttk.Label(data_frame, textvariable=self.left_var).grid(row=0, column=1, sticky="w")
        ttk.Label(data_frame, textvariable=self.right_var).grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Label(data_frame, textvariable=self.rear_var).grid(row=1, column=1, sticky="w", pady=(6, 0))
        ttk.Label(data_frame, textvariable=self.scan_info_var).grid(row=2, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Label(data_frame, textvariable=self.device_var, wraplength=420, justify="left").grid(row=3, column=0, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Label(data_frame, textvariable=self.health_var, wraplength=420, justify="left").grid(row=4, column=0, columnspan=2, sticky="w", pady=(4, 0))

        log_frame = ttk.LabelFrame(right, text="避障测试日志", padding=10)
        log_frame.grid(row=3, column=0, sticky="nsew", pady=(8, 0))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log_text = ScrolledText(log_frame, height=12, wrap="word", state="disabled", font=("Microsoft YaHei UI", 10))
        self.log_text.grid(row=0, column=0, sticky="nsew")

        self.threshold_cm_var.trace_add("write", lambda *_: self._sync_threshold_from_ui())

    def _sync_threshold_from_ui(self):
        if self._threshold_syncing:
            return
        self._threshold_syncing = True
        try:
            try:
                threshold_cm = int(self.threshold_cm_var.get())
            except Exception:
                threshold_cm = DEFAULT_AVOIDANCE_THRESHOLD_MM // 10
            threshold_cm = max(1, min(100, threshold_cm))
            if self.threshold_cm_var.get() != threshold_cm:
                self.threshold_cm_var.set(threshold_cm)
            self.backend.set_threshold_mm(threshold_cm * 10)
            self.threshold_text_var.set(f"当前避障阈值: {threshold_cm} cm")
            self.log_queue.put(f"[{time.strftime('%H:%M:%S')}] 已设置避障阈值: {threshold_cm} cm")
        finally:
            self._threshold_syncing = False

    @staticmethod
    def _format_distance(value_mm: float | None) -> str:
        if value_mm is None:
            return "--"
        return f"{value_mm / 10.0:.1f} cm"

    def refresh_ports(self):
        ports = [port.device for port in list_ports.comports()]
        self.available_ports = ports
        if self.port_combo is not None:
            self.port_combo["values"] = ports
        if DEFAULT_LIDAR_PORT in ports:
            self.port_var.set(DEFAULT_LIDAR_PORT)
        elif ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        port_text = ports if ports else "未发现可用串口"
        self.log_queue.put(f"[{time.strftime('%H:%M:%S')}] 已扫描雷达串口: {port_text}")

    def render_placeholder(self):
        image = LidarSlamWindow.render_scan_image([], size=DEFAULT_SCAN_VIEW_DIMS)
        self.update_image_label(image)

    def start_lidar(self):
        port = self.port_var.get().strip()
        if not port:
            self.status_var.set("请先选择雷达串口")
            return
        if self.backend.start(port):
            self.status_var.set(f"正在启动避障雷达: {port}")
        else:
            state = self.backend.snapshot()
            self.status_var.set(state.status if not state.last_error else f"{state.status} | {state.last_error}")

    def stop_lidar(self):
        self.backend.stop()
        self.status_var.set("雷达已停止")

    def start_avoidance(self):
        link_ready = getattr(self.controller, "has_control_link", None)
        if callable(link_ready):
            ready = bool(link_ready())
        else:
            serial_conn = getattr(self.controller, "serial_conn", None)
            ready = bool(serial_conn is not None and serial_conn.is_open)
        if not ready:
            self.status_var.set("请先在主控制页连接小车通信链路，再启动避障测试")
            return
        self._sync_threshold_from_ui()
        ok, message = self.backend.start_avoidance()
        self.status_var.set(message)
        if ok:
            self.controller.root.after(0, lambda: self.controller.log("已启动激光雷达避障测试"))

    def stop_avoidance(self):
        _ok, message = self.backend.stop_avoidance()
        self.status_var.set(message)
        self.controller.root.after(0, lambda: self.controller.log("已停止激光雷达避障测试"))

    def enqueue_motion_command(self, command: str, action: str):
        def _send():
            ok = self.controller.send_serial_command(command, show_log=False)
            if ok:
                self.controller.status_var.set(f"避障测试: {action}")
                self.controller.log(f"避障动作: {action} -> {command}")
            else:
                self.status_var.set("小车串口发送失败，请检查主控制页串口连接")

        self.controller.root.after(0, _send)

    def refresh_loop(self):
        if not self.winfo_exists():
            return

        state = self.backend.snapshot()
        status = state.status if not state.last_error else f"{state.status} | {state.last_error}"
        self.status_var.set(status)
        self.action_var.set(f"当前动作: {state.current_action}")
        self.scan_info_var.set(f"扫点: {len(state.scan_points)} | 累计扫描: {state.scan_count} 圈")
        self.front_var.set(f"前方: {self._format_distance(state.front_mm)}")
        self.left_var.set(f"左侧: {self._format_distance(state.left_mm)}")
        self.right_var.set(f"右侧: {self._format_distance(state.right_mm)}")
        self.rear_var.set(f"后方: {self._format_distance(state.rear_mm)}")
        self.device_var.set(f"设备信息: {state.device_info if state.device_info else '未连接'}")
        self.health_var.set(f"健康状态: {state.health if state.health else '未连接'}")
        self.threshold_text_var.set(f"当前避障阈值: {state.threshold_mm / 10.0:.1f} cm")

        try:
            image = LidarSlamWindow.render_scan_image(
                state.scan_points,
                size=LidarSlamWindow._label_dimensions(self.scan_label, DEFAULT_SCAN_VIEW_DIMS),
            )
            self.update_image_label(image)
        except Exception as exc:
            self.log_queue.put(f"[{time.strftime('%H:%M:%S')}] 避障页渲染异常: {exc}")

        self.after(DEFAULT_REFRESH_MS, self.refresh_loop)

    def flush_log_queue(self):
        if not self.winfo_exists():
            return
        updated = False
        self.log_text.configure(state="normal")
        while not self.log_queue.empty():
            self.log_text.insert("end", self.log_queue.get_nowait() + "\n")
            updated = True
        if updated:
            self.log_text.see("end")
        self.log_text.configure(state="disabled")
        self.after(120, self.flush_log_queue)

    def update_image_label(self, image_array):
        if ImageTk is None:
            return
        rgb = cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(rgb)
        photo = ImageTk.PhotoImage(image=image)
        self.scan_label.configure(image=photo)
        self.scan_photo = photo

    def close(self):
        self.backend.stop()

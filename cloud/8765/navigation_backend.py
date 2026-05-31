from __future__ import annotations

import contextlib
import json
import math
import queue
import threading
import time
from pathlib import Path
from typing import Any

from avoidance_tab import DEFAULT_AVOIDANCE_THRESHOLD_MM

_UNSET = object()

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.context import Context as RclpyContext
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    ROS_NAVIGATION_IMPORT_ERROR = None
except Exception as exc:  # pragma: no cover - depends on board runtime
    rclpy = None
    GoalStatus = None
    PoseStamped = None
    NavigateToPose = None
    ActionClient = None
    RclpyContext = None
    SingleThreadedExecutor = None
    Node = object
    ROS_NAVIGATION_IMPORT_ERROR = exc


class NavigationTaskStore:
    def __init__(self, logger, maps_dir: Path):
        self.logger = logger
        self.maps_dir = maps_dir

    def _task_path(self, map_name: str) -> Path:
        return self.maps_dir / f"{Path(map_name).stem}.tasks.json"

    @staticmethod
    def _normalize_task(payload: dict[str, Any]) -> dict[str, Any]:
        task_name = str(payload.get("task_name") or "").strip()
        if not task_name:
            raise RuntimeError("任务名称不能为空")

        waypoint_ids = [str(item).strip() for item in payload.get("waypoint_ids", []) if str(item).strip()]
        if not waypoint_ids:
            raise RuntimeError(f"任务 {task_name} 没有任何点位")

        loop_count = max(1, int(payload.get("loop_count", 1)))
        arrival_tolerance = max(0.05, float(payload.get("arrival_tolerance", 0.25)))
        avoidance_threshold_mm = max(10, int(payload.get("avoidance_threshold_mm", DEFAULT_AVOIDANCE_THRESHOLD_MM)))
        mode = str(payload.get("mode") or "inspect").strip() or "inspect"
        return {
            "task_name": task_name,
            "waypoint_ids": waypoint_ids,
            "loop_count": loop_count,
            "arrival_tolerance": round(arrival_tolerance, 3),
            "avoidance_threshold_mm": avoidance_threshold_mm,
            "mode": mode,
        }

    def normalize_bundle(self, map_name: str, payload: dict[str, Any]) -> dict[str, Any]:
        return {
            "map_name": Path(map_name).stem,
            "tasks": [self._normalize_task(item) for item in payload.get("tasks", [])],
        }

    def load(self, map_name: str) -> dict[str, Any]:
        path = self._task_path(map_name)
        if not path.exists():
            return {"map_name": Path(map_name).stem, "tasks": []}
        payload = json.loads(path.read_text(encoding="utf-8"))
        return self.normalize_bundle(map_name, payload)

    def save(self, map_name: str, payload: dict[str, Any]) -> dict[str, Any]:
        normalized = self.normalize_bundle(map_name, payload)
        path = self._task_path(map_name)
        path.write_text(json.dumps(normalized, ensure_ascii=False, indent=2), encoding="utf-8")
        self.logger.log(f"已保存导航任务: {path.name}")
        return normalized


class NavActionBridge:
    def __init__(self, logger):
        self.logger = logger
        self._available = all(
            item is not None
            for item in (
                rclpy,
                RclpyContext,
                Node,
                ActionClient,
                NavigateToPose,
                PoseStamped,
                GoalStatus,
                SingleThreadedExecutor,
            )
        )
        self._stop_event = threading.Event()
        self._command_queue: queue.Queue[dict[str, Any]] = queue.Queue()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._server_ready = False
        self._last_server_check = 0.0
        self._last_server_ready_at = 0.0
        self._goal_handle = None
        self._goal_label = ""
        self._feedback_text = ""
        self._last_error = ""
        self._last_result = ""
        self._result_sequence = 0
        self._context = None
        self._executor = None
        self._node = None
        self._client = None
        self._worker_alive = False
        if self._available:
            self._thread = threading.Thread(target=self._worker, daemon=True)
            self._thread.start()

    @staticmethod
    def _goal_status_label(status_code: int) -> str:
        mapping = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }
        return mapping.get(int(status_code), f"STATUS_{int(status_code)}")

    def _build_goal(self, *, x: float, y: float, yaw_deg: float):
        yaw_rad = math.radians(float(yaw_deg))
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def _submit(self, kind: str, payload: dict[str, Any] | None = None, timeout: float = 8.0) -> tuple[bool, str]:
        if not self._available:
            return False, f"导航动作接口不可用: {ROS_NAVIGATION_IMPORT_ERROR}"
        request = {
            "kind": kind,
            "payload": payload or {},
            "event": threading.Event(),
            "ok": False,
            "message": "导航动作请求超时",
        }
        self._command_queue.put(request)
        if not request["event"].wait(timeout):
            return False, str(request["message"])
        return bool(request["ok"]), str(request["message"])

    def send_goal(self, *, x: float, y: float, yaw_deg: float, label: str) -> tuple[bool, str]:
        return self._submit("send_goal", {"x": x, "y": y, "yaw_deg": yaw_deg, "label": label}, timeout=20.0)

    def cancel(self) -> tuple[bool, str]:
        return self._submit("cancel", timeout=6.0)

    def _probe_server_ready(self, timeout_s: float = 0.0) -> bool:
        if self._client is None:
            with self._lock:
                self._server_ready = False
            return False
        deadline = time.time() + max(0.0, float(timeout_s))
        ready = False
        while not self._stop_event.is_set():
            with contextlib.suppress(Exception):
                ready = bool(self._client.wait_for_server(timeout_sec=0.0))
            if ready or time.time() >= deadline:
                break
            if self._executor is not None:
                with contextlib.suppress(Exception):
                    self._executor.spin_once(timeout_sec=0.05)
            else:
                time.sleep(0.05)
        now = time.time()
        with self._lock:
            self._server_ready = bool(ready)
            if ready:
                self._last_server_ready_at = now
        return bool(ready)

    def _server_recently_ready(self, window_s: float = 15.0) -> bool:
        with self._lock:
            last_ready_at = float(self._last_server_ready_at or 0.0)
        if last_ready_at <= 0.0:
            return False
        return (time.time() - last_ready_at) <= max(0.0, float(window_s))

    def _update_server_ready(self):
        now = time.time()
        if now - self._last_server_check < 0.5 or self._client is None:
            return
        self._last_server_check = now
        self._probe_server_ready(timeout_s=0.0)

    def _worker(self):
        try:
            with self._lock:
                self._worker_alive = True
            self._context = RclpyContext()
            rclpy.init(args=None, context=self._context)
            self._node = Node("board_nav_action_bridge", context=self._context)
            self._client = ActionClient(self._node, NavigateToPose, "/navigate_to_pose")
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            while not self._stop_event.is_set():
                try:
                    request = self._command_queue.get_nowait()
                except queue.Empty:
                    request = None
                if request is not None:
                    self._handle_request(request)
                self._update_server_ready()
                self._executor.spin_once(timeout_sec=0.05)
        except Exception as exc:  # pragma: no cover - runtime specific
            with self._lock:
                self._last_error = str(exc)
        finally:
            node = self._node
            executor = self._executor
            self._client = None
            self._node = None
            self._executor = None
            with self._lock:
                self._worker_alive = False
            if executor is not None and node is not None:
                with contextlib.suppress(Exception):
                    executor.remove_node(node)
            if node is not None:
                with contextlib.suppress(Exception):
                    node.destroy_node()
            if self._context is not None:
                with contextlib.suppress(Exception):
                    if self._context.ok():
                        rclpy.shutdown(context=self._context)
                self._context = None

    def _handle_request(self, request: dict[str, Any]):
        if request["kind"] == "cancel":
            self._handle_cancel(request)
            return

        if self._node is None or self._client is None:
            request["message"] = "导航动作客户端尚未初始化"
            request["event"].set()
            return
        if self._goal_handle is not None:
            request["message"] = "当前已有导航目标正在执行"
            request["event"].set()
            return
        ready_now = self._probe_server_ready(timeout_s=10.0)
        ready_recently = self._server_recently_ready(window_s=20.0)
        if not ready_now and not ready_recently:
            request["message"] = "等待 /navigate_to_pose 动作服务超时"
            request["event"].set()
            return

        payload = request["payload"]
        goal_label = str(payload.get("label") or "导航目标").strip() or "导航目标"
        goal = self._build_goal(
            x=float(payload.get("x", 0.0)),
            y=float(payload.get("y", 0.0)),
            yaw_deg=float(payload.get("yaw_deg", 0.0)),
        )
        future = self._client.send_goal_async(goal, feedback_callback=self._handle_feedback)
        future.add_done_callback(
            lambda fut, req=request, label=goal_label: self._handle_goal_response(fut, req, label)
        )

    def _handle_request(self, request: dict[str, Any]):
        if request["kind"] == "cancel":
            self._handle_cancel(request)
            return

        if self._node is None or self._client is None:
            request["message"] = "navigation action client is not initialized"
            request["event"].set()
            return
        if self._goal_handle is not None:
            request["message"] = "a navigation goal is already active"
            request["event"].set()
            return

        ready_now = self._probe_server_ready(timeout_s=10.0)
        ready_recently = self._server_recently_ready(window_s=20.0)
        if not ready_now and not ready_recently:
            request["message"] = "waited too long for /navigate_to_pose action server"
            request["event"].set()
            return
        if not ready_now and ready_recently:
            self.logger.log(
                "navigate_to_pose probe timed out during goal dispatch; using recently-ready cache"
            )

        payload = request["payload"]
        goal_label = str(payload.get("label") or "navigation goal").strip() or "navigation goal"
        goal = self._build_goal(
            x=float(payload.get("x", 0.0)),
            y=float(payload.get("y", 0.0)),
            yaw_deg=float(payload.get("yaw_deg", 0.0)),
        )
        try:
            future = self._client.send_goal_async(goal, feedback_callback=self._handle_feedback)
        except Exception as exc:  # pragma: no cover - runtime specific
            with self._lock:
                self._last_error = str(exc)
            request["message"] = f"failed to send navigation goal: {exc}"
            request["event"].set()
            return
        future.add_done_callback(
            lambda fut, req=request, label=goal_label: self._handle_goal_response(fut, req, label)
        )

    def _handle_goal_response(self, future, request: dict[str, Any], goal_label: str):
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - runtime specific
            with self._lock:
                self._last_error = str(exc)
            request["message"] = f"发送导航目标失败: {exc}"
            request["event"].set()
            return

        if goal_handle is None or not goal_handle.accepted:
            request["message"] = "导航目标未被 Nav2 接收"
            request["event"].set()
            return

        with self._lock:
            self._goal_handle = goal_handle
            self._goal_label = goal_label
            self._feedback_text = "目标已接收，等待执行"
            self._last_error = ""
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_goal_result)
        request["ok"] = True
        request["message"] = f"Nav2 已接收目标: {goal_label}"
        request["event"].set()

    def _handle_goal_response(self, future, request: dict[str, Any], goal_label: str):
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - runtime specific
            with self._lock:
                self._last_error = str(exc)
            request["message"] = f"failed to send navigation goal: {exc}"
            request["event"].set()
            return

        if goal_handle is None or not goal_handle.accepted:
            request["message"] = "navigation goal was rejected by Nav2"
            request["event"].set()
            return

        with self._lock:
            self._goal_handle = goal_handle
            self._goal_label = goal_label
            self._feedback_text = "goal accepted; waiting for execution"
            self._last_error = ""
            self._server_ready = True
            self._last_server_ready_at = time.time()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._handle_goal_result)
        request["ok"] = True
        request["message"] = f"Nav2 accepted goal: {goal_label}"
        request["event"].set()

    def _handle_feedback(self, feedback_msg):
        feedback = getattr(feedback_msg, "feedback", None)
        if feedback is None:
            return
        parts: list[str] = []
        distance_remaining = getattr(feedback, "distance_remaining", None)
        if distance_remaining is not None:
            parts.append(f"remaining {float(distance_remaining):.2f} m")
        eta = getattr(feedback, "estimated_time_remaining", None)
        eta_sec = int(getattr(eta, "sec", 0)) if eta is not None else 0
        if eta_sec > 0:
            parts.append(f"eta {eta_sec} s")
        recoveries = getattr(feedback, "number_of_recoveries", None)
        if recoveries is not None:
            parts.append(f"recoveries {int(recoveries)}")
        with self._lock:
            self._feedback_text = " | ".join(parts) if parts else "导航执行中"

    def _handle_goal_result(self, future):
        status_label = "UNKNOWN"
        message = "导航结束"
        try:
            result_wrapper = future.result()
            status_label = self._goal_status_label(result_wrapper.status)
            result = result_wrapper.result
            error_code = int(getattr(result, "error_code", 0))
            error_msg = str(getattr(result, "error_msg", "") or "").strip()
            message = status_label
            if error_code:
                message = f"{status_label} ({error_code})"
            if error_msg:
                message = f"{message}: {error_msg}"
        except Exception as exc:  # pragma: no cover - runtime specific
            message = f"读取导航结果失败: {exc}"
            with self._lock:
                self._last_error = str(exc)

        with self._lock:
            self._goal_handle = None
            self._goal_label = ""
            self._feedback_text = ""
            self._last_result = message
            if status_label in {"SUCCEEDED", "CANCELED"}:
                self._last_error = ""
            elif not self._last_error:
                self._last_error = message
            self._result_sequence += 1

    def _handle_cancel(self, request: dict[str, Any]):
        if self._goal_handle is None:
            request["ok"] = True
            request["message"] = "当前没有正在执行的导航目标"
            request["event"].set()
            return
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda fut, req=request: self._handle_cancel_done(fut, req))

    def _handle_cancel_done(self, future, request: dict[str, Any]):
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - runtime specific
            request["message"] = f"取消导航目标失败: {exc}"
            request["event"].set()
            return
        request["ok"] = True
        request["message"] = (
            "已请求取消导航目标" if getattr(response, "goals_canceling", []) else "取消请求已发送"
        )
        request["event"].set()

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "available": self._available,
                "worker_alive": self._worker_alive,
                "node_ready": self._node is not None,
                "client_ready": self._client is not None,
                "ready": self._server_ready,
                "goal_active": self._goal_handle is not None,
                "goal_label": self._goal_label,
                "feedback_text": self._feedback_text,
                "last_error": self._last_error,
                "last_result": self._last_result,
                "result_sequence": self._result_sequence,
            }


    def status(self) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            last_server_ready_at = float(self._last_server_ready_at or 0.0)
            server_ready_age_ms = None
            if last_server_ready_at > 0.0:
                server_ready_age_ms = int(max(0.0, now - last_server_ready_at) * 1000)
            server_recently_ready = bool(last_server_ready_at > 0.0 and (now - last_server_ready_at) <= 20.0)
            return {
                "available": self._available,
                "worker_alive": self._worker_alive,
                "node_ready": self._node is not None,
                "client_ready": self._client is not None,
                "ready": bool(self._server_ready or server_recently_ready),
                "server_ready": self._server_ready,
                "server_recently_ready": server_recently_ready,
                "server_ready_age_ms": server_ready_age_ms,
                "goal_active": self._goal_handle is not None,
                "goal_label": self._goal_label,
                "feedback_text": self._feedback_text,
                "last_error": self._last_error,
                "last_result": self._last_result,
                "result_sequence": self._result_sequence,
            }


class NavigationSystem:
    def __init__(self, logger, ros, maps, controller, avoidance):
        self.logger = logger
        self.ros = ros
        self.maps = maps
        self.controller = controller
        self.avoidance = avoidance
        self.task_store = NavigationTaskStore(logger, maps.maps_dir)
        self.action = NavActionBridge(logger)
        self._lock = threading.Lock()
        self._task_thread: threading.Thread | None = None
        self._task_stop = threading.Event()
        self._monitor_stop = threading.Event()
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        self._last_action_result_sequence = 0
        self._localization_map = ""
        self._nav_state = "idle"
        self._current_goal: dict[str, Any] | None = None
        self._active_task: dict[str, Any] | None = None
        self._remaining_waypoint_ids: list[str] = []
        self._completed_waypoint_ids: list[str] = []
        self._safety_interlock_reason = ""
        self._last_error = ""
        self._last_result = ""
        self._active_threshold_mm = DEFAULT_AVOIDANCE_THRESHOLD_MM
        self.avoidance.set_interlock_callback(self._handle_avoidance_motion)

    @staticmethod
    def _distance_in_sector(
        scan_points: list[tuple[float, float, int]],
        start_angle: float,
        end_angle: float,
    ) -> float | None:
        distances: list[float] = []
        for angle_deg, distance_mm, _quality in scan_points:
            angle = float(angle_deg) % 360.0
            if start_angle <= end_angle:
                in_sector = start_angle <= angle <= end_angle
            else:
                in_sector = angle >= start_angle or angle <= end_angle
            if in_sector:
                distances.append(float(distance_mm))
        return min(distances) if distances else None

    @staticmethod
    def _task_remaining_ids(
        ordered_waypoint_ids: list[str],
        *,
        current_index: int,
        current_loop: int,
        total_loops: int,
    ) -> list[str]:
        remaining = list(ordered_waypoint_ids[current_index + 1 :])
        for _ in range(max(0, total_loops - current_loop - 1)):
            remaining.extend(ordered_waypoint_ids)
        return remaining

    def _set_runtime(
        self,
        *,
        nav_state: str | None = None,
        current_goal: dict[str, Any] | None | object = _UNSET,
        active_task: dict[str, Any] | None | object = _UNSET,
        remaining_waypoint_ids: list[str] | None = None,
        completed_waypoint_ids: list[str] | None = None,
        safety_reason: str | None = None,
        last_error: str | None = None,
        last_result: str | None = None,
        localization_map: str | None = None,
        threshold_mm: int | None = None,
    ):
        with self._lock:
            if nav_state is not None:
                self._nav_state = nav_state
            if current_goal is not _UNSET:
                self._current_goal = None if current_goal is None else dict(current_goal)
            if active_task is not _UNSET:
                self._active_task = None if active_task is None else dict(active_task)
            if remaining_waypoint_ids is not None:
                self._remaining_waypoint_ids = list(remaining_waypoint_ids)
            if completed_waypoint_ids is not None:
                self._completed_waypoint_ids = list(completed_waypoint_ids)
            if safety_reason is not None:
                self._safety_interlock_reason = str(safety_reason)
            if last_error is not None:
                self._last_error = str(last_error)
            if last_result is not None:
                self._last_result = str(last_result)
            if localization_map is not None:
                self._localization_map = str(localization_map)
            if threshold_mm is not None:
                self._active_threshold_mm = int(threshold_mm)

    def _waypoint_lookup(self, map_name: str) -> dict[str, dict[str, Any]]:
        payload = self.maps.load_annotations(map_name)
        points = payload.get("points", []) if isinstance(payload, dict) else []
        lookup: dict[str, dict[str, Any]] = {}
        for item in points:
            point_id = str(item.get("id") or "").strip()
            if point_id:
                lookup[point_id] = dict(item)
        return lookup

    def _task_points(self, map_name: str) -> list[dict[str, Any]]:
        return list(self._waypoint_lookup(map_name).values())

    def load_tasks(self, map_name: str) -> dict[str, Any]:
        payload = self.task_store.load(map_name)
        payload["points"] = self._task_points(map_name)
        return payload

    def save_tasks(self, map_name: str, payload: dict[str, Any]) -> dict[str, Any]:
        normalized = self.task_store.normalize_bundle(map_name, payload)
        lookup = self._waypoint_lookup(map_name)
        allowed_types = {"pose", "inspect"}
        for task in normalized["tasks"]:
            for waypoint_id in task["waypoint_ids"]:
                point = lookup.get(waypoint_id)
                if point is None:
                    raise RuntimeError(f"任务 {task['task_name']} 引用了不存在的点位: {waypoint_id}")
                point_type = str(point.get("type") or "pose").strip() or "pose"
                if point_type not in allowed_types:
                    raise RuntimeError(f"点位 {point.get('name') or waypoint_id} 类型 {point_type} 不允许用于导航")
        self.task_store.save(map_name, normalized)
        normalized["points"] = list(lookup.values())
        return normalized

    def _resolve_goal(self, payload: dict[str, Any], map_name: str) -> dict[str, Any]:
        waypoint_id = str(payload.get("waypoint_id") or "").strip()
        if waypoint_id:
            point = self._waypoint_lookup(map_name).get(waypoint_id)
            if point is None:
                raise RuntimeError(f"未找到点位: {waypoint_id}")
            return {
                "waypoint_id": waypoint_id,
                "label": str(point.get("name") or waypoint_id),
                "x": float(point.get("x", 0.0)),
                "y": float(point.get("y", 0.0)),
                "yaw": float(point.get("yaw", 0.0)),
                "type": str(point.get("type") or "pose"),
                "note": str(point.get("note") or ""),
            }
        return {
            "waypoint_id": "",
            "label": str(payload.get("label") or "自定义目标").strip() or "自定义目标",
            "x": float(payload.get("x", 0.0)),
            "y": float(payload.get("y", 0.0)),
            "yaw": float(payload.get("yaw", 0.0)),
            "type": "pose",
            "note": "",
        }

    def _goal_running(self) -> bool:
        return self._nav_state in {"goal_running", "task_running"}

    def _handle_avoidance_motion(self, command: str, action: str) -> bool:
        if not self._goal_running():
            return False
        if str(command).strip() == "$TZ!":
            return True
        self._trigger_interlock(f"避障联锁触发: {action}")
        return True

    def _scan_interlock_reason(self, threshold_mm: int) -> str:
        preview = self.ros.preview.snapshot()
        if not preview.available or not preview.scan_points:
            return ""
        front = self._distance_in_sector(preview.scan_points, 345.0, 15.0)
        front_left = self._distance_in_sector(preview.scan_points, 15.0, 75.0)
        front_right = self._distance_in_sector(preview.scan_points, 285.0, 345.0)
        values = [value for value in (front, front_left, front_right) if value is not None]
        if not values:
            return ""
        nearest = min(values)
        if nearest > float(threshold_mm):
            return ""
        return f"前向安全联锁触发: {int(nearest)} mm <= {int(threshold_mm)} mm"

    def _trigger_interlock(self, reason: str):
        if not reason:
            return
        if self._nav_state == "blocked" and self._safety_interlock_reason == reason:
            return
        self.logger.log(reason)
        self.action.cancel()
        with contextlib.suppress(Exception):
            self.controller.send_motion("stop")
        self._task_stop.set()
        self._set_runtime(nav_state="blocked", safety_reason=reason, last_error=reason, last_result=reason)

    def _monitor_loop(self):
        while not self._monitor_stop.is_set():
            try:
                action_status = self.action.status()
                result_sequence = int(action_status.get("result_sequence") or 0)
                if self._goal_running():
                    reason = self._scan_interlock_reason(self._active_threshold_mm)
                    if reason:
                        self._trigger_interlock(reason)
                task_running = self._task_thread is not None and self._task_thread.is_alive()
                if (
                    not task_running
                    and self._nav_state == "goal_running"
                    and result_sequence != self._last_action_result_sequence
                ):
                    self._last_action_result_sequence = result_sequence
                    last_result = str(action_status.get("last_result") or "")
                    if "SUCCEEDED" in last_result:
                        self._set_runtime(
                            nav_state="goal_succeeded",
                            current_goal=None,
                            last_error="",
                            last_result=last_result,
                            safety_reason="",
                        )
                    elif "CANCELED" in last_result:
                        self._set_runtime(
                            nav_state="idle",
                            current_goal=None,
                            active_task=None,
                            last_error="",
                            last_result=last_result,
                            safety_reason="",
                        )
                    elif last_result:
                        self._set_runtime(
                            nav_state="goal_failed",
                            current_goal=None,
                            last_error=last_result,
                            last_result=last_result,
                        )
            except Exception as exc:  # pragma: no cover - defensive
                self._set_runtime(last_error=str(exc))
            self._monitor_stop.wait(0.2)

    def _ensure_localization(self, map_name: str, port: str = ""):
        normalized_map = Path(map_name).stem
        ros_status = self.ros.status()
        if (
            ros_status.get("navigation_mode") == "localization"
            and Path(str(ros_status.get("localization_map") or "")).stem == normalized_map
            and ros_status.get("map_server_live")
            and ros_status.get("amcl_live")
            and ros_status.get("nav2_live")
        ):
            self._set_runtime(localization_map=normalized_map)
            return
        self.start_localization(normalized_map, port)

    def start_localization(self, map_name: str, port: str = "") -> dict[str, Any]:
        normalized_map = Path(map_name).stem
        if not normalized_map:
            raise RuntimeError("必须指定地图名称")
        self.stop_task(silent=True)
        self.action.cancel()
        result = self.ros.start_localization(normalized_map, port)
        self._set_runtime(
            nav_state="localized",
            current_goal=None,
            active_task=None,
            remaining_waypoint_ids=[],
            completed_waypoint_ids=[],
            safety_reason="",
            last_error="",
            last_result=result.get("message", ""),
            localization_map=normalized_map,
        )
        return {
            "message": result.get("message", "定位导航栈已启动"),
            "status": self.status(force=True),
            "warning": result.get("warning", ""),
        }

    def stop_localization(self, *, silent: bool = False) -> dict[str, Any]:
        self.stop_task(silent=True)
        self.action.cancel()
        message = self.ros.stop_localization(silent=silent)
        self._set_runtime(
            nav_state="idle",
            current_goal=None,
            active_task=None,
            remaining_waypoint_ids=[],
            completed_waypoint_ids=[],
            safety_reason="",
            last_error="",
            last_result=message,
            localization_map="",
        )
        return {"message": message, "status": self.status(force=True)}

    def start_goal(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_name = Path(str(payload.get("map_name") or self._localization_map or "")).stem
        if not map_name:
            raise RuntimeError("单点导航必须指定 map_name")
        self.stop_task(silent=True)
        self._ensure_localization(map_name, str(payload.get("port") or ""))

        arrival_tolerance = float(payload.get("arrival_tolerance", 0.25))
        with contextlib.suppress(Exception):
            self.ros.set_navigation_tolerance(arrival_tolerance)

        goal = self._resolve_goal(payload, map_name)
        threshold_mm = int(payload.get("avoidance_threshold_mm", DEFAULT_AVOIDANCE_THRESHOLD_MM))
        before_sequence = int(self.action.status().get("result_sequence") or 0)
        ok, message = self.action.send_goal(x=goal["x"], y=goal["y"], yaw_deg=goal["yaw"], label=goal["label"])
        if not ok:
            raise RuntimeError(message)
        self._last_action_result_sequence = before_sequence
        self._set_runtime(
            nav_state="goal_running",
            current_goal=goal,
            active_task=None,
            remaining_waypoint_ids=[goal["waypoint_id"]] if goal["waypoint_id"] else [],
            completed_waypoint_ids=[],
            safety_reason="",
            last_error="",
            last_result=message,
            localization_map=map_name,
            threshold_mm=threshold_mm,
        )
        return {"message": message, "status": self.status(force=True)}

    def cancel_goal(self) -> dict[str, Any]:
        self.stop_task(silent=True)
        ok, message = self.action.cancel()
        with contextlib.suppress(Exception):
            self.controller.send_motion("stop")
        if ok:
            self._set_runtime(
                nav_state="idle",
                current_goal=None,
                active_task=None,
                remaining_waypoint_ids=[],
                safety_reason="",
                last_error="",
                last_result=message,
            )
        else:
            self._set_runtime(last_error=message, last_result=message)
        return {"message": message, "status": self.status(force=True)}

    def _run_task(self, map_name: str, task: dict[str, Any]):
        ordered_waypoint_ids = list(task["waypoint_ids"])
        lookup = self._waypoint_lookup(map_name)
        ordered_waypoints = [lookup[waypoint_id] for waypoint_id in ordered_waypoint_ids if waypoint_id in lookup]
        completed: list[str] = []
        total_loops = max(1, int(task.get("loop_count", 1)))
        try:
            if not ordered_waypoints:
                raise RuntimeError(f"任务 {task['task_name']} 没有可执行点位")

            with contextlib.suppress(Exception):
                self.ros.set_navigation_tolerance(float(task.get("arrival_tolerance", 0.25)))

            for loop_index in range(total_loops):
                for point_index, point in enumerate(ordered_waypoints):
                    if self._task_stop.is_set():
                        self._set_runtime(
                            nav_state="idle",
                            current_goal=None,
                            active_task=None,
                            remaining_waypoint_ids=[],
                            safety_reason="",
                            last_result="任务已停止",
                        )
                        return

                    goal = {
                        "waypoint_id": str(point.get("id") or ""),
                        "label": str(point.get("name") or point.get("id") or f"point_{point_index + 1}"),
                        "x": float(point.get("x", 0.0)),
                        "y": float(point.get("y", 0.0)),
                        "yaw": float(point.get("yaw", 0.0)),
                        "type": str(point.get("type") or "pose"),
                        "note": str(point.get("note") or ""),
                    }
                    remaining = self._task_remaining_ids(
                        ordered_waypoint_ids,
                        current_index=point_index,
                        current_loop=loop_index,
                        total_loops=total_loops,
                    )
                    self._set_runtime(
                        nav_state="task_running",
                        current_goal=goal,
                        active_task={
                            "map_name": map_name,
                            "task_name": task["task_name"],
                            "mode": task["mode"],
                            "loop_count": total_loops,
                            "current_loop": loop_index + 1,
                            "arrival_tolerance": float(task.get("arrival_tolerance", 0.25)),
                            "avoidance_threshold_mm": int(task["avoidance_threshold_mm"]),
                        },
                        remaining_waypoint_ids=remaining,
                        completed_waypoint_ids=completed,
                        threshold_mm=int(task["avoidance_threshold_mm"]),
                        safety_reason="",
                        last_error="",
                        last_result=f"前往点位: {goal['label']}",
                    )

                    before_sequence = int(self.action.status().get("result_sequence") or 0)
                    ok, message = self.action.send_goal(
                        x=goal["x"],
                        y=goal["y"],
                        yaw_deg=goal["yaw"],
                        label=goal["label"],
                    )
                    if not ok:
                        raise RuntimeError(message)

                    while not self._task_stop.is_set():
                        time.sleep(0.2)
                        action_status = self.action.status()
                        sequence = int(action_status.get("result_sequence") or 0)
                        if sequence == before_sequence:
                            continue
                        self._last_action_result_sequence = sequence
                        last_result = str(action_status.get("last_result") or "")
                        if "SUCCEEDED" in last_result:
                            completed.append(goal["waypoint_id"] or goal["label"])
                            break
                        if self._nav_state == "blocked":
                            self._set_runtime(
                                active_task=None,
                                current_goal=None,
                                remaining_waypoint_ids=[],
                                completed_waypoint_ids=completed,
                            )
                            return
                        raise RuntimeError(last_result or f"点位 {goal['label']} 导航失败")

                    if self._task_stop.is_set():
                        self.action.cancel()
                        with contextlib.suppress(Exception):
                            self.controller.send_motion("stop")
                        self._set_runtime(
                            nav_state="idle",
                            current_goal=None,
                            active_task=None,
                            remaining_waypoint_ids=[],
                            safety_reason="",
                            last_result="任务已停止",
                        )
                        return

            self._set_runtime(
                nav_state="task_completed",
                current_goal=None,
                active_task=None,
                remaining_waypoint_ids=[],
                completed_waypoint_ids=completed,
                safety_reason="",
                last_error="",
                last_result=f"任务 {task['task_name']} 已完成",
            )
        except Exception as exc:
            self._set_runtime(
                nav_state="blocked" if self._nav_state == "blocked" else "task_failed",
                current_goal=None,
                active_task=None,
                remaining_waypoint_ids=[],
                completed_waypoint_ids=completed,
                last_error=str(exc),
                last_result=str(exc),
            )
        finally:
            self._task_thread = None

    def start_task(self, payload: dict[str, Any]) -> dict[str, Any]:
        map_name = Path(str(payload.get("map_name") or "")).stem
        task_name = str(payload.get("task_name") or "").strip()
        if not map_name or not task_name:
            raise RuntimeError("启动任务必须提供 map_name 和 task_name")
        if self._task_thread is not None and self._task_thread.is_alive():
            raise RuntimeError("当前已有任务在运行")

        tasks_payload = self.task_store.load(map_name)
        task = next((item for item in tasks_payload["tasks"] if item["task_name"] == task_name), None)
        if task is None:
            raise RuntimeError(f"未找到任务: {task_name}")

        self._ensure_localization(map_name, str(payload.get("port") or ""))
        self._task_stop.clear()
        self._task_thread = threading.Thread(target=self._run_task, args=(map_name, task), daemon=True)
        self._task_thread.start()
        self._set_runtime(
            nav_state="task_running",
            active_task={
                "map_name": map_name,
                "task_name": task_name,
                "mode": task["mode"],
                "loop_count": task["loop_count"],
                "current_loop": 1,
                "arrival_tolerance": float(task.get("arrival_tolerance", 0.25)),
                "avoidance_threshold_mm": int(task["avoidance_threshold_mm"]),
            },
            remaining_waypoint_ids=list(task["waypoint_ids"]),
            completed_waypoint_ids=[],
            threshold_mm=int(task["avoidance_threshold_mm"]),
            safety_reason="",
            last_error="",
            last_result=f"任务 {task_name} 已启动",
            localization_map=map_name,
        )
        return {"message": f"任务 {task_name} 已启动", "status": self.status(force=True)}

    def stop_task(self, *, silent: bool = False) -> dict[str, Any]:
        self._task_stop.set()
        thread = self._task_thread
        if thread is not None and thread.is_alive():
            self.action.cancel()
            thread.join(timeout=2.0)
        self._task_thread = None
        with contextlib.suppress(Exception):
            self.controller.send_motion("stop")
        if not silent:
            self._set_runtime(
                nav_state="idle",
                active_task=None,
                current_goal=None,
                remaining_waypoint_ids=[],
                completed_waypoint_ids=[],
                safety_reason="",
                last_error="",
                last_result="任务已停止",
            )
        return {"message": "任务已停止", "status": self.status(force=True)}

    def status(self, force: bool = False) -> dict[str, Any]:
        _ = force
        ros_status = self.ros.status()
        action_status = self.action.status()
        with self._lock:
            nav_state = self._nav_state
            current_goal = dict(self._current_goal) if isinstance(self._current_goal, dict) else None
            active_task = dict(self._active_task) if isinstance(self._active_task, dict) else None
            remaining_waypoint_ids = list(self._remaining_waypoint_ids)
            completed_waypoint_ids = list(self._completed_waypoint_ids)
            safety_reason = self._safety_interlock_reason
            last_error = self._last_error
            last_result = self._last_result
            localization_map = self._localization_map or str(ros_status.get("localization_map") or "")
            threshold_mm = self._active_threshold_mm
        return {
            "available": action_status.get("available", False),
            "navigation_mode": ros_status.get("navigation_mode", "idle"),
            "localization_active": ros_status.get("navigation_mode") == "localization",
            "localization_map": localization_map,
            "action_server_ready": action_status.get("ready", False),
            "goal_active": action_status.get("goal_active", False),
            "goal_feedback": action_status.get("feedback_text", ""),
            "nav_state": nav_state,
            "current_goal": current_goal,
            "active_task": active_task,
            "task_running": self._task_thread is not None and self._task_thread.is_alive(),
            "remaining_waypoint_ids": remaining_waypoint_ids,
            "completed_waypoint_ids": completed_waypoint_ids,
            "safety_interlock_reason": safety_reason,
            "blocked": nav_state == "blocked",
            "last_error": last_error or action_status.get("last_error", ""),
            "last_result": last_result or action_status.get("last_result", ""),
            "map_server_live": ros_status.get("map_server_live", False),
            "amcl_live": ros_status.get("amcl_live", False),
            "nav2_live": ros_status.get("nav2_live", False),
            "threshold_mm": threshold_mm,
        }

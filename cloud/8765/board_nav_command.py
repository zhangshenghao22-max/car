#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


ROOT_DIR = Path(__file__).resolve().parent
RUNTIME_DIR = ROOT_DIR / "runtime" / "cloud_nav_command"
STATUS_PATH = RUNTIME_DIR / "status.json"
PID_PATH = RUNTIME_DIR / "cruise.pid"
LOG_PATH = RUNTIME_DIR / "cruise.log"


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def safe_pose_dict(raw: Any) -> dict[str, float]:
    if not isinstance(raw, dict):
        raise RuntimeError("pose must be an object")
    try:
        x = float(raw["x"])
        y = float(raw["y"])
        yaw = float(raw.get("yaw", 0.0))
    except Exception as exc:
        raise RuntimeError(f"invalid pose: {exc}") from exc
    return {"x": x, "y": y, "yaw": yaw}


def safe_points(raw: Any) -> list[dict[str, float]]:
    if not isinstance(raw, list):
        raise RuntimeError("points must be a list")
    points: list[dict[str, float]] = []
    for index, item in enumerate(raw, start=1):
        pose = safe_pose_dict(item)
        pose["label"] = str(item.get("label") or f"P{index}")
        points.append(pose)
    if not points:
        raise RuntimeError("at least one point is required")
    return points


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
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    @staticmethod
    def _quat_from_yaw(yaw_deg: float) -> tuple[float, float, float, float]:
        yaw_rad = math.radians(float(yaw_deg))
        return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))

    def publish_initialpose(self, pose: dict[str, float], executor: SingleThreadedExecutor) -> None:
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

    def send_goal(self, pose: dict[str, float], executor: SingleThreadedExecutor, stop_flag) -> int:
        if not self.action_client.wait_for_server(timeout_sec=15.0):
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

        send_future = self.action_client.send_goal_async(goal_msg)
        while not send_future.done():
            if stop_flag["stop"]:
                raise KeyboardInterrupt()
            executor.spin_once(timeout_sec=0.1)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("navigation goal rejected")

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if stop_flag["stop"]:
                cancel_future = goal_handle.cancel_goal_async()
                deadline = time.time() + 5.0
                while not cancel_future.done() and time.time() < deadline:
                    executor.spin_once(timeout_sec=0.1)
                raise KeyboardInterrupt()
            executor.spin_once(timeout_sec=0.1)

        result = result_future.result()
        status_code = int(result.status) if result is not None else GoalStatus.STATUS_UNKNOWN
        return status_code


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


def build_patrol_command(points_json: str, loop_count: int, start_pose_json: str) -> list[str]:
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
    ]


def command_start_cruise(args: argparse.Namespace) -> int:
    points = safe_points(json.loads(args.points_json))
    loop_count = max(1, int(args.loop_count))
    start_pose = safe_pose_dict(json.loads(args.start_pose_json)) if args.start_pose_json else {}

    existing_pid = read_pid()
    if pid_alive(existing_pid):
        raise RuntimeError(f"cruise already running: pid={existing_pid}")
    clear_pid_file()

    RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
    command = build_patrol_command(args.points_json, loop_count, args.start_pose_json or "")
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
            "mode": "cruise_starting",
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
            "start_pose": start_pose,
            "pid": process.pid,
        }
    )
    print(f"cruise started pid={process.pid}", flush=True)
    return 0


def command_stop_cruise(_args: argparse.Namespace) -> int:
    pid = read_pid()
    if not pid_alive(pid):
        clear_pid_file()
        merge_status(
            {
                "mode": "idle",
                "task_running": False,
                "goal_active": False,
                "message": "no active cruise",
                "pid": 0,
            }
        )
        print("no active cruise", flush=True)
        return 0

    merge_status({"mode": "stopping", "message": f"stopping cruise pid={pid}"})
    os.kill(pid, signal.SIGTERM)
    deadline = time.time() + 8.0
    while time.time() < deadline:
        if not pid_alive(pid):
            clear_pid_file()
            merge_status(
                {
                    "mode": "idle",
                    "task_running": False,
                    "goal_active": False,
                    "message": "cruise stopped",
                    "last_result": "cruise stopped",
                    "pid": 0,
                }
            )
            print("cruise stopped", flush=True)
            return 0
        time.sleep(0.2)

    os.kill(pid, signal.SIGKILL)
    clear_pid_file()
    merge_status(
        {
            "mode": "idle",
            "task_running": False,
            "goal_active": False,
            "message": "cruise killed",
            "last_error": "",
            "pid": 0,
        }
    )
    print("cruise killed", flush=True)
    return 0


def command_run_cruise(args: argparse.Namespace) -> int:
    points = safe_points(json.loads(args.points_json))
    loop_count = max(1, int(args.loop_count))
    start_pose = safe_pose_dict(json.loads(args.start_pose_json)) if args.start_pose_json else {}
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
                "mode": "cruise_running",
                "task_running": True,
                "goal_active": False,
                "points": points,
                "point_labels": point_labels(points),
                "current_index": -1,
                "loop_index": 0,
                "loop_count": loop_count,
                "completed_count": 0,
                "current_goal": points[0] if points else {},
                "start_pose": start_pose,
                "message": "cruise running",
                "last_error": "",
                "pid": os.getpid(),
            }
        )

        for loop_index in range(loop_count):
            for point_index, point in enumerate(points):
                if stop_flag["stop"]:
                    raise KeyboardInterrupt()
                merge_status(
                    {
                        "mode": "cruise_running",
                        "task_running": True,
                        "goal_active": True,
                        "current_index": point_index,
                        "loop_index": loop_index + 1,
                        "current_goal": point,
                        "completed_count": completed,
                        "message": f"running point {point_index + 1}/{total_points} loop {loop_index + 1}/{loop_count}",
                    }
                )
                status_code = node.send_goal(point, executor, stop_flag)
                if status_code != GoalStatus.STATUS_SUCCEEDED:
                    raise RuntimeError(f"goal failed with status {status_code}")
                completed += 1
                merge_status(
                    {
                        "mode": "cruise_running",
                        "task_running": True,
                        "goal_active": False,
                        "current_index": point_index,
                        "loop_index": loop_index + 1,
                        "current_goal": point,
                        "completed_count": completed,
                        "message": f"point {point_index + 1}/{total_points} done",
                    }
                )

        merge_status(
            {
                "mode": "completed",
                "task_running": False,
                "goal_active": False,
                "current_index": total_points - 1,
                "loop_index": loop_count,
                "completed_count": completed,
                "message": f"cruise completed with {completed} point(s)",
                "last_result": f"cruise completed with {completed} point(s)",
                "last_error": "",
                "pid": 0,
            }
        )
        print(f"cruise completed points={completed}", flush=True)
        return 0
    except KeyboardInterrupt:
        merge_status(
            {
                "mode": "canceled",
                "task_running": False,
                "goal_active": False,
                "message": "cruise canceled",
                "last_result": "cruise canceled",
                "pid": 0,
            }
        )
        print("cruise canceled", flush=True)
        return 130
    except Exception as exc:
        merge_status(
            {
                "mode": "failed",
                "task_running": False,
                "goal_active": False,
                "message": str(exc),
                "last_error": str(exc),
                "pid": 0,
            }
        )
        print(f"cruise failed: {exc}", flush=True)
        return 1
    finally:
        clear_pid_file()
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
    parser_start.set_defaults(func=command_start_cruise)

    parser_run = sub.add_parser("run-cruise")
    parser_run.add_argument("--points-json", required=True)
    parser_run.add_argument("--loop-count", default="1")
    parser_run.add_argument("--start-pose-json", default="")
    parser_run.set_defaults(func=command_run_cruise)

    parser_stop = sub.add_parser("stop-cruise")
    parser_stop.set_defaults(func=command_stop_cruise)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())

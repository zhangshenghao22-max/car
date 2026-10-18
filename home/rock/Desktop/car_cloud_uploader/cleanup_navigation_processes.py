#!/usr/bin/env python3
"""Stop orphaned processes belonging to the board navigation launch."""

from __future__ import annotations

import os
import signal
import time


MATCHES = (
    "rt_robot_nav2_complete.launch.py",
    "/opt/ros/foxy/lib/nav2_",
    "/opt/ros/foxy/lib/robot_localization/ekf_node",
    "/rplidar_ros/rplidar_node",
    "/imu_ros2_device/ybimu_driver",
    "/imu_filter_madgwick/imu_filter_madgwick_node",
    "/tf2_ros/static_transform_publisher",
    "/robot_state_publisher/robot_state_publisher",
    "/joint_state_publisher/joint_state_publisher",
    "/rt_robot_nav2/imu_z_calibrator.py",
    "/rt_robot_nav2/costmap_scan_range_filter.py",
    "/rt_robot_nav2/nav_motion_stabilizer.py",
    "/rt_robot_nav2/maze_turn_manager.py",
    "/rt_robot_nav2/nav_plan_diagnostics.py",
    "/rt_robot_nav2/auto_initialpose.py",
    "/rt_robot_nav2/goal_stop_publisher.py",
)


def command_line(pid: int) -> str:
    try:
        data = open(f"/proc/{pid}/cmdline", "rb").read()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return ""
    return data.replace(b"\0", b" ").decode("utf-8", "replace")


def parent_pid(pid: int) -> int:
    try:
        fields = open(f"/proc/{pid}/stat", "r", encoding="ascii").read().split()
        return int(fields[3])
    except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError, IndexError):
        return 0


def protected_processes() -> set[int]:
    protected = {os.getpid()}
    pid = os.getppid()
    while pid > 1 and pid not in protected:
        protected.add(pid)
        pid = parent_pid(pid)
    return protected


def matching_processes() -> list[tuple[int, str]]:
    protected = protected_processes()
    found = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid in protected:
            continue
        cmd = command_line(pid)
        if cmd and any(marker in cmd for marker in MATCHES):
            found.append((pid, cmd))
    return found


def signal_processes(processes: list[tuple[int, str]], sig: signal.Signals) -> None:
    for pid, _ in processes:
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


def remaining(processes: list[tuple[int, str]]) -> list[tuple[int, str]]:
    return [(pid, cmd) for pid, cmd in processes if os.path.exists(f"/proc/{pid}")]


def main() -> int:
    processes = matching_processes()
    if not processes:
        print("navigation process cleanup: no stale processes")
        return 0

    for pid, cmd in processes:
        print(f"navigation process cleanup: stopping pid={pid} cmd={cmd[:160]}")

    signal_processes(processes, signal.SIGINT)
    time.sleep(2.0)
    processes = remaining(processes)
    if processes:
        signal_processes(processes, signal.SIGTERM)
        time.sleep(1.5)
    processes = remaining(processes)
    if processes:
        signal_processes(processes, signal.SIGKILL)
        time.sleep(0.5)

    processes = remaining(processes)
    if processes:
        print("navigation process cleanup: failed to stop all processes")
        return 1
    print("navigation process cleanup: complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

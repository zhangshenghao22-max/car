#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import os
import select
import sys
import threading
import time

try:
    import termios
    import tty
except Exception:  # pragma: no cover - non-Linux fallback
    termios = None
    tty = None

try:
    import msvcrt
except Exception:  # pragma: no cover - Linux runtime path
    msvcrt = None

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


HELP_LINES = [
    "========================================",
    "Keyboard Teleop Control",
    "========================================",
    "w/s : Forward/Backward (board sign corrected: w sends negative X, s sends positive X)",
    "a/d : Left/Right strafe (board sign corrected: a sends negative Y, d sends positive Y)",
    "q/e : Increase/Decrease angular Z velocity (rotate left/right)",
    "x   : Stop all movement",
    "r   : Reset all velocities to zero",
    "i   : Print current velocities",
    "h   : Print this help message",
    "Ctrl+C : Exit",
    "========================================",
]


class KeyReader:
    def __enter__(self):
        self._fd = None
        self._old_termios = None
        if os.name == "nt" and msvcrt is not None:
            return self
        if termios is None or tty is None:
            raise RuntimeError("raw terminal input is not available in this environment")
        if not sys.stdin.isatty():
            raise RuntimeError("stdin is not a TTY")
        self._fd = sys.stdin.fileno()
        self._old_termios = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._fd is not None and self._old_termios is not None:
            with contextlib.suppress(Exception):
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_termios)

    def read_key(self, timeout_s: float = 0.05) -> str:
        if os.name == "nt" and msvcrt is not None:
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                if msvcrt.kbhit():
                    return msvcrt.getwch()
                time.sleep(0.01)
            return ""
        if self._fd is None:
            return ""
        ready, _, _ = select.select([self._fd], [], [], timeout_s)
        if not ready:
            return ""
        return os.read(self._fd, 8).decode("utf-8", errors="ignore")


class KeyboardTeleop(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__(args.node_name)
        qos_reliability = self._resolve_qos_reliability(args.qos_reliability, args.cmd_vel_topic)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=args.qos_depth,
            reliability=ReliabilityPolicy.RELIABLE
            if qos_reliability == "reliable"
            else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(Twist, args.cmd_vel_topic, qos)
        self._cmd_vel_topic = args.cmd_vel_topic
        self._linear_step = float(args.linear_velocity_step)
        self._angular_step = float(args.angular_velocity_step)
        self._max_linear_x = abs(float(args.max_linear_x))
        self._max_linear_y = abs(float(args.max_linear_y))
        self._max_angular_z = abs(float(args.max_angular_z))
        self._publish_mode = str(args.publish_mode)
        self._hold_rate_hz = max(1.0, float(args.hold_rate_hz))
        self._zero_on_exit = bool(args.zero_on_exit)
        self._current_linear_x = 0.0
        self._current_linear_y = 0.0
        self._current_angular_z = 0.0
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._state_dirty = False
        self._last_zero_sent = False
        self._hold_thread = None
        if self._publish_mode == "hold":
            self._hold_thread = threading.Thread(target=self._hold_publish_loop, daemon=True)
            self._hold_thread.start()
        self.get_logger().info("Keyboard teleop node started")
        self.get_logger().info(f"Publishing to topic: {self._cmd_vel_topic}")
        self.get_logger().info(
            f"Mode={self._publish_mode}, qos={qos_reliability}, "
            f"step(linear)={self._linear_step:.2f}, step(angular)={self._angular_step:.2f}"
        )

    def _resolve_qos_reliability(self, requested: str, topic_name: str) -> str:
        requested = str(requested or "auto").strip().lower()
        if requested in ("reliable", "best_effort"):
            return requested
        saw_reliable = False
        deadline = time.time() + 1.2
        while time.time() < deadline:
            with contextlib.suppress(Exception):
                rclpy.spin_once(self, timeout_sec=0.05)
            infos = []
            with contextlib.suppress(Exception):
                infos = list(self.get_subscriptions_info_by_topic(topic_name))
            for info in infos:
                reliability = getattr(getattr(info, "qos_profile", None), "reliability", None)
                if reliability == ReliabilityPolicy.BEST_EFFORT:
                    return "best_effort"
                if reliability == ReliabilityPolicy.RELIABLE:
                    saw_reliable = True
            if infos:
                break
            time.sleep(0.05)
        if saw_reliable:
            return "reliable"
        return "reliable"

    def shutdown(self):
        if self._zero_on_exit:
            with self._lock:
                self._current_linear_x = 0.0
                self._current_linear_y = 0.0
                self._current_angular_z = 0.0
                self._mark_dirty_locked()
                self._publish_locked("exit")
        self._stop_event.set()
        if self._hold_thread is not None and self._hold_thread.is_alive():
            self._hold_thread.join(timeout=1.0)

    def print_help(self):
        print("")
        for line in HELP_LINES:
            print(line)
        print("")

    def print_velocities(self):
        with self._lock:
            linear_x = self._current_linear_x
            linear_y = self._current_linear_y
            angular_z = self._current_angular_z
        self.get_logger().info(
            f"Current velocities - Linear X: {linear_x:.2f}, "
            f"Linear Y: {linear_y:.2f}, Angular Z: {angular_z:.2f}"
        )

    def _mark_dirty_locked(self):
        self._state_dirty = True
        if self._is_zero_locked():
            self._last_zero_sent = False

    def _is_zero_locked(self) -> bool:
        return (
            abs(self._current_linear_x) <= 1e-9
            and abs(self._current_linear_y) <= 1e-9
            and abs(self._current_angular_z) <= 1e-9
        )

    def _build_message_locked(self) -> Twist:
        message = Twist()
        message.linear.x = self._current_linear_x
        message.linear.y = self._current_linear_y
        message.angular.z = self._current_angular_z
        return message

    def _publish_locked(self, reason: str):
        message = self._build_message_locked()
        self._publisher.publish(message)
        self._state_dirty = False
        self._last_zero_sent = self._is_zero_locked()
        subscriber_count = self.count_subscribers(self._cmd_vel_topic)
        if subscriber_count <= 0:
            self.get_logger().warning(
                f"Published to {self._cmd_vel_topic} but no subscriber is visible yet ({reason})"
            )
        else:
            self.get_logger().debug(
                f"Published to {self._cmd_vel_topic} ({reason}), subscribers={subscriber_count}"
            )

    def _hold_publish_loop(self):
        interval = 1.0 / self._hold_rate_hz
        while not self._stop_event.is_set():
            with self._lock:
                should_publish = self._state_dirty or not self._is_zero_locked()
                should_publish = should_publish or (self._is_zero_locked() and not self._last_zero_sent)
                if should_publish:
                    self._publish_locked("hold_loop")
            time.sleep(interval)

    def _publish_immediately_if_needed(self, reason: str):
        if self._publish_mode != "once":
            return
        with self._lock:
            self._publish_locked(reason)

    def _set_velocity(self, *, linear_x: float | None = None, linear_y: float | None = None, angular_z: float | None = None):
        with self._lock:
            if linear_x is not None:
                self._current_linear_x = linear_x
            if linear_y is not None:
                self._current_linear_y = linear_y
            if angular_z is not None:
                self._current_angular_z = angular_z
            self._mark_dirty_locked()

    def stop_motion(self, reason: str = "stop"):
        self._set_velocity(linear_x=0.0, linear_y=0.0, angular_z=0.0)
        self._publish_immediately_if_needed(reason)

    def process_key(self, key: str):
        if not key:
            return
        key = key[0]
        if key in ("h", "H"):
            self.print_help()
            return
        if key in ("i", "I"):
            self.print_velocities()
            return

        with self._lock:
            if key in ("w", "W"):
                self._current_linear_x = max(self._current_linear_x - self._linear_step, -self._max_linear_x)
            elif key in ("s", "S"):
                self._current_linear_x = min(self._current_linear_x + self._linear_step, self._max_linear_x)
            elif key in ("a", "A"):
                self._current_linear_y = max(self._current_linear_y - self._linear_step, -self._max_linear_y)
            elif key in ("d", "D"):
                self._current_linear_y = min(self._current_linear_y + self._linear_step, self._max_linear_y)
            elif key in ("q", "Q"):
                self._current_angular_z = min(self._current_angular_z + self._angular_step, self._max_angular_z)
            elif key in ("e", "E"):
                self._current_angular_z = max(self._current_angular_z - self._angular_step, -self._max_angular_z)
            elif key in ("x", "X", "r", "R"):
                self._current_linear_x = 0.0
                self._current_linear_y = 0.0
                self._current_angular_z = 0.0
            else:
                return
            self._mark_dirty_locked()
            linear_x = self._current_linear_x
            linear_y = self._current_linear_y
            angular_z = self._current_angular_z

        self.get_logger().info(
            f"Linear X: {linear_x:.2f}, Linear Y: {linear_y:.2f}, Angular Z: {angular_z:.2f}"
        )
        self._publish_immediately_if_needed(f"key={key}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Board keyboard teleop compatibility script for the current project."
    )
    parser.add_argument("--node-name", default="keyboard_teleop")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--linear-velocity-step", type=float, default=0.1)
    parser.add_argument("--angular-velocity-step", type=float, default=0.1)
    parser.add_argument("--max-linear-x", type=float, default=0.9)
    parser.add_argument("--max-linear-y", type=float, default=0.9)
    parser.add_argument("--max-angular-z", type=float, default=0.9)
    parser.add_argument(
        "--publish-mode",
        choices=("hold", "once"),
        default="hold",
        help="hold: continuously republish current Twist for the current project chain; once: emulate the board native script.",
    )
    parser.add_argument(
        "--hold-rate-hz",
        type=float,
        default=10.0,
        help="publish rate used only when --publish-mode=hold",
    )
    parser.add_argument(
        "--qos-reliability",
        choices=("auto", "reliable", "best_effort"),
        default="auto",
    )
    parser.add_argument("--qos-depth", type=int, default=5)
    parser.add_argument("--zero-on-exit", dest="zero_on_exit", action="store_true", default=True)
    parser.add_argument("--no-zero-on-exit", dest="zero_on_exit", action="store_false")
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    rclpy.init(args=None)
    node = KeyboardTeleop(args)
    node.print_help()
    try:
        with KeyReader() as key_reader:
            while rclpy.ok():
                key = key_reader.read_key(timeout_s=0.05)
                if key:
                    node.process_key(key)
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        with contextlib.suppress(Exception):
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

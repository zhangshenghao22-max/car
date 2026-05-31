#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import os
import select
import sys
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
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String as RosString


SERVO_MIN_PWM = 500
SERVO_MAX_PWM = 2500
DEFAULT_SERVO_COUNT = 6
DEFAULT_SERVO_IDS = [0, 1, 2, 3, 4, 5]
DEFAULT_HOME = [2300, 1450, 1500, 2200, 1500, 1700]
DEFAULT_STEP_PWM = 50
DEFAULT_COARSE_STEP_PWM = 200
DEFAULT_DURATION_MS = 400
DEFAULT_DURATION_STEP_MS = 50
DEFAULT_ACTIVE_SLOT = 2

HELP_LINES = [
    "========================================",
    "F103 ROS2 Keyboard Arm Teleop",
    "========================================",
    "1-6 : Select active joint slot",
    "j/l : Active joint -/+ fine PWM step",
    "u/o : Active joint -/+ coarse PWM step",
    "n/m : Motion duration -/+ 50 ms",
    "r   : Reset active joint to home PWM",
    "g   : Send all current joint targets together",
    "H   : Send all joints to home pose",
    "x   : Stop active joint with #IDPDST!",
    "i   : Print current joint state",
    "h   : Print this help message",
    "q   : Exit",
    "Ctrl+C : Exit",
    "",
    "Use this script with f103-usb-chassis.service running in ROS_DOMAIN_ID=17.",
    "Control path: keyboard -> /arm_cmd -> f103_usb_ros2_bridge.py -> F103 -> arm bus/PWM",
    "========================================",
]


def clamp_pwm(value: int, min_pwm: int, max_pwm: int) -> int:
    return max(min_pwm, min(max_pwm, int(value)))


def clamp_duration_ms(value: int) -> int:
    return max(20, min(5000, int(value)))


def parse_home_values(home_text: str, servo_count: int) -> list[int]:
    parts = [part.strip() for part in str(home_text).split(",") if part.strip()]
    values = [clamp_pwm(int(part), SERVO_MIN_PWM, SERVO_MAX_PWM) for part in parts]
    if len(values) < servo_count:
        values.extend([1500] * (servo_count - len(values)))
    return values[:servo_count]


def parse_servo_ids(servo_ids_text: str, servo_count: int) -> list[int]:
    parts = [part.strip() for part in str(servo_ids_text).split(",") if part.strip()]
    if not parts:
        parts = [str(index) for index in range(servo_count)]
    values = [int(part) for part in parts]
    if len(values) < servo_count:
        start = values[-1] + 1 if values else 0
        values.extend(range(start, start + (servo_count - len(values))))
    return values[:servo_count]


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


class KeyboardArmTeleop(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__(args.node_name)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=args.qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(RosString, args.arm_topic, qos)
        self._arm_topic = str(args.arm_topic)
        self._servo_count = max(1, int(args.servo_count))
        self._servo_ids = parse_servo_ids(args.servo_ids, self._servo_count)
        self._home = parse_home_values(args.home, self._servo_count)
        self._servo_values = list(self._home)
        requested_active_slot = max(1, int(args.active_slot))
        self._active_joint = min(self._servo_count - 1, requested_active_slot - 1)
        self._step_pwm = max(1, int(args.step_pwm))
        self._coarse_step_pwm = max(self._step_pwm, int(args.coarse_step_pwm))
        self._duration_ms = clamp_duration_ms(args.duration_ms)
        self._duration_step_ms = max(1, int(args.duration_step_ms))
        self._min_pwm = int(args.min_pwm)
        self._max_pwm = int(args.max_pwm)
        self._stop_on_exit = bool(args.stop_on_exit)
        self._last_payload = ""
        self._last_subscriber_count = 0
        self.get_logger().info(f"Keyboard arm teleop node started on {self._arm_topic}")
        self.get_logger().info(
            "Control path: Linux /arm_cmd -> f103_usb_ros2_bridge -> F103 arm command parser"
        )
        self.get_logger().info(
            "ROS environment: ROS_LOCALHOST_ONLY=%s ROS_DOMAIN_ID=%s"
            % (os.environ.get("ROS_LOCALHOST_ONLY", ""), os.environ.get("ROS_DOMAIN_ID", ""))
        )
        self.get_logger().info(
            "Active slot defaults to slot %d -> hardware #%03d"
            % (self._active_joint + 1, self._servo_ids[self._active_joint])
        )

    def wait_for_subscriber(self, timeout_s: float) -> int:
        deadline = time.time() + max(0.0, float(timeout_s))
        count = 0
        while time.time() < deadline:
            with contextlib.suppress(Exception):
                rclpy.spin_once(self, timeout_sec=0.05)
            count = self.count_subscribers(self._arm_topic)
            if count > 0:
                self._last_subscriber_count = count
                return count
            time.sleep(0.05)
        self._last_subscriber_count = count
        return count

    def print_help(self):
        print("")
        for line in HELP_LINES:
            print(line)
        print("")
        self.print_state()

    def print_state(self):
        joint_dump = " ".join(
            f"[{index + 1}/#{self._servo_ids[index]:03d}:{value}]"
            if index != self._active_joint
            else f"*[{index + 1}/#{self._servo_ids[index]:03d}:{value}]*"
            for index, value in enumerate(self._servo_values)
        )
        self.get_logger().info(
            f"active_joint={self._active_joint + 1}, hardware_id=#{self._servo_ids[self._active_joint]:03d}, "
            f"duration_ms={self._duration_ms}, "
            f"subscribers={self.count_subscribers(self._arm_topic)}, joints={joint_dump}"
        )

    def _publish_payload(self, payload: str, reason: str) -> None:
        message = RosString()
        message.data = payload
        self._publisher.publish(message)
        self._last_payload = payload
        subscriber_count = self.count_subscribers(self._arm_topic)
        self._last_subscriber_count = subscriber_count
        if subscriber_count <= 0:
            self.get_logger().warning(
                f"Published arm command but no subscriber is visible on {self._arm_topic} ({reason})"
            )
        else:
            self.get_logger().info(
                f"Published {reason}: {payload} (subscribers={subscriber_count})"
            )

    def _build_servo_payload(self, targets: list[tuple[int, int]]) -> str:
        duration_ms = clamp_duration_ms(self._duration_ms)
        parts = [
            f"#{index:03d}P{clamp_pwm(value, self._min_pwm, self._max_pwm):04d}T{duration_ms:04d}!"
            for index, value in targets
        ]
        if len(parts) == 1:
            return parts[0]
        return "{" + "".join(parts) + "}"

    def _send_active_joint(self, reason: str) -> None:
        slot_index = self._active_joint
        hardware_id = self._servo_ids[slot_index]
        value = self._servo_values[slot_index]
        payload_reason = f"{reason}_servo_{hardware_id:03d}"
        self._publish_payload(self._build_servo_payload([(hardware_id, value)]), payload_reason)

    def send_all_current(self, reason: str = "all_joints") -> None:
        targets = [(hardware_id, value) for hardware_id, value in zip(self._servo_ids, self._servo_values)]
        self._publish_payload(self._build_servo_payload(targets), reason)

    def send_home(self) -> None:
        self._servo_values = list(self._home)
        self.send_all_current("home_pose")

    def send_stop(self, reason: str = "stop") -> None:
        active_id = self._servo_ids[self._active_joint]
        self._publish_payload(f"#{active_id:03d}PDST!", reason)

    def _adjust_active_joint(self, delta_pwm: int) -> None:
        index = self._active_joint
        updated = clamp_pwm(self._servo_values[index] + delta_pwm, self._min_pwm, self._max_pwm)
        self._servo_values[index] = updated
        self._send_active_joint(f"joint_{index + 1}")
        self.print_state()

    def process_key(self, key: str) -> bool:
        if not key:
            return True
        key = key[0]
        if key in ("q", "Q"):
            return False
        if key in ("h",):
            self.print_help()
            return True
        if key in ("i", "I"):
            self.print_state()
            return True
        if key.isdigit():
            index = int(key) - 1
            if 0 <= index < self._servo_count:
                self._active_joint = index
                self.print_state()
            return True

        if key in ("j", "J"):
            delta = -self._coarse_step_pwm if key == "J" else -self._step_pwm
            self._adjust_active_joint(delta)
            return True
        if key in ("l", "L"):
            delta = self._coarse_step_pwm if key == "L" else self._step_pwm
            self._adjust_active_joint(delta)
            return True
        if key in ("u", "U"):
            self._adjust_active_joint(-self._coarse_step_pwm)
            return True
        if key in ("o", "O"):
            self._adjust_active_joint(self._coarse_step_pwm)
            return True
        if key in ("n", "N"):
            self._duration_ms = clamp_duration_ms(self._duration_ms - self._duration_step_ms)
            self.print_state()
            return True
        if key in ("m", "M"):
            self._duration_ms = clamp_duration_ms(self._duration_ms + self._duration_step_ms)
            self.print_state()
            return True
        if key in ("r", "R"):
            self._servo_values[self._active_joint] = self._home[self._active_joint]
            self._send_active_joint(f"reset_joint_{self._active_joint + 1}")
            self.print_state()
            return True
        if key in ("g", "G"):
            self.send_all_current("sync_all_joints")
            self.print_state()
            return True
        if key == "H":
            self.send_home()
            self.print_state()
            return True
        if key in ("x", "X"):
            self.send_stop("manual_stop")
            return True
        return True

    def shutdown(self):
        if self._stop_on_exit:
            with contextlib.suppress(Exception):
                self.send_stop("exit")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Board keyboard arm teleop for Linux -> ROS2 -> F103 via /arm_cmd."
    )
    parser.add_argument("--node-name", default="keyboard_arm_teleop")
    parser.add_argument("--arm-topic", default="/arm_cmd")
    parser.add_argument("--servo-count", type=int, default=DEFAULT_SERVO_COUNT)
    parser.add_argument(
        "--servo-ids",
        default=",".join(str(value) for value in DEFAULT_SERVO_IDS),
        help="comma-separated hardware servo ids used by each visible slot; default is 0-5",
    )
    parser.add_argument(
        "--home",
        default=",".join(str(value) for value in DEFAULT_HOME),
        help="comma-separated home PWM list; defaults to the six-axis arm home pose",
    )
    parser.add_argument(
        "--active-slot",
        type=int,
        default=DEFAULT_ACTIVE_SLOT,
        help="1-based visible slot selected on startup; default selects hardware #001",
    )
    parser.add_argument("--step-pwm", type=int, default=DEFAULT_STEP_PWM)
    parser.add_argument("--coarse-step-pwm", type=int, default=DEFAULT_COARSE_STEP_PWM)
    parser.add_argument("--duration-ms", type=int, default=DEFAULT_DURATION_MS)
    parser.add_argument("--duration-step-ms", type=int, default=DEFAULT_DURATION_STEP_MS)
    parser.add_argument("--min-pwm", type=int, default=SERVO_MIN_PWM)
    parser.add_argument("--max-pwm", type=int, default=SERVO_MAX_PWM)
    parser.add_argument("--wait-subscriber", type=float, default=3.0)
    parser.add_argument("--qos-depth", type=int, default=10)
    parser.add_argument("--stop-on-exit", dest="stop_on_exit", action="store_true", default=True)
    parser.add_argument("--no-stop-on-exit", dest="stop_on_exit", action="store_false")
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    rclpy.init(args=None)
    node = KeyboardArmTeleop(args)
    subscriber_count = node.wait_for_subscriber(args.wait_subscriber)
    if subscriber_count <= 0:
        node.get_logger().warning(
            f"No subscriber detected on {args.arm_topic}. "
            "Start the F103 USB mapping/navigation bridge before sending commands."
        )
    node.print_help()
    try:
        with KeyReader() as key_reader:
            keep_running = True
            while rclpy.ok() and keep_running:
                key = key_reader.read_key(timeout_s=0.05)
                if key:
                    keep_running = node.process_key(key)
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

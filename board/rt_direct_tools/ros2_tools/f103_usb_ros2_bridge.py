#!/usr/bin/env python3
from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    import serial
except Exception as exc:  # pragma: no cover - runtime dependency check
    raise SystemExit(f"pyserial is required: {exc}")


DEFAULT_PORT = "/dev/f103"
DEFAULT_BAUDRATE = 115200
WHEEL_BASE_M = 0.205
AXLE_BASE_M = 0.225

ODOM_RE = re.compile(
    r"@ODOM:SEQ=(?P<seq>\d+),DT_MS=(?P<dt>\d+),"
    r"EA=(?P<ea>-?\d+),EB=(?P<eb>-?\d+),EC=(?P<ec>-?\d+),ED=(?P<ed>-?\d+),"
    r"WA=(?P<wa>-?\d+),WB=(?P<wb>-?\d+),WC=(?P<wc>-?\d+),WD=(?P<wd>-?\d+)!"
)
TWIST_OK_RE = re.compile(r"@TWIST:SEQ=(?P<seq>\d+),OK!")
TWIST_ERR_RE = re.compile(r"@TWIST:SEQ=(?P<seq>\d+),ERR=(?P<err>[^,]+),MODE=(?P<mode>[^!]+)!")


@dataclass
class OdomFrame:
    seq: int
    dt_s: float
    encoder_ticks: tuple[int, int, int, int]
    wheel_mm_s: tuple[int, int, int, int]


class F103UsbRos2Bridge(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__(args.node_name)
        self.args = args
        self.args.port = self._resolve_serial_port(args.port)
        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.seq = int(time.time()) % 900000

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_seq: Optional[int] = None
        self.last_cmd_time = 0.0
        self.last_rx_time = 0.0
        self.last_state = ""
        self.last_twist_ack = ""
        self.rx_count = 0
        self.odom_count = 0
        self.state_count = 0
        self.twist_ok_count = 0
        self.twist_err_count = 0
        self.unknown_count = 0
        self.arm_cmd_count = 0
        self.arm_cmd_reject_count = 0
        self.last_arm_cmd = ""
        self.last_arm_cmd_time = 0.0

        self.target_vx = 0
        self.target_vy = 0
        self.target_wz = 0
        self.last_sent = (None, None, None)
        self.last_zero_sent_at = 0.0

        self.odom_pub = self.create_publisher(Odometry, args.odom_topic, 20)
        self.state_pub = self.create_publisher(String, args.state_topic, 20)

        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Twist, args.cmd_topic, self._on_twist, cmd_qos)
        if args.extra_cmd_topic and args.extra_cmd_topic != args.cmd_topic:
            self.create_subscription(Twist, args.extra_cmd_topic, self._on_twist, cmd_qos)
        if args.arm_topic and not args.disable_arm_topic:
            arm_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(String, args.arm_topic, self._on_arm_cmd, arm_qos)

        self.serial = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            timeout=0.05,
            write_timeout=0.5,
        )
        with self.serial_lock:
            if not args.keep_input_buffer:
                self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

        self.reader_thread = threading.Thread(target=self._reader_loop, name="f103-usb-rx", daemon=True)
        self.reader_thread.start()

        self.command_timer = self.create_timer(1.0 / max(1.0, args.command_rate_hz), self._command_timer)
        self.status_timer = self.create_timer(max(0.5, args.status_period), self._publish_status)
        self.handshake_timer = self.create_timer(0.2, self._handshake_timer)
        self.handshake_done = False

        self.get_logger().info(
            f"F103 USB bridge opened {args.port} @ {args.baudrate}; "
            f"subscribe {args.cmd_topic}"
            + (f" and {args.extra_cmd_topic}" if args.extra_cmd_topic else "")
            + (f"; arm {args.arm_topic}" if args.arm_topic and not args.disable_arm_topic else "")
            + f"; publish {args.odom_topic}"
        )

    def destroy_node(self):
        self.stop_event.set()
        self._send_twist_mm_s(0, 0, 0, force=True)
        try:
            if self.reader_thread.is_alive():
                self.reader_thread.join(timeout=0.5)
        except Exception:
            pass
        try:
            self.serial.close()
        except Exception:
            pass
        super().destroy_node()

    def _send_handshake(self):
        for command in ("$PING!", "$STATUS!", "$REPORT:ON!", "$ODOM:ON!", "$MODE:NAVIGATION!", "$REPORT:ON!", "$ODOM:ON!"):
            self._write_command(command)
            time.sleep(max(0.03, self.args.handshake_interval))
        self._send_twist_mm_s(0, 0, 0, force=True)

    def _handshake_timer(self):
        if self.handshake_done:
            return
        self.handshake_done = True
        self.get_logger().info("sending F103 handshake")
        self._send_handshake()
        self.get_logger().info("F103 handshake sent")

    def _write_command(self, command: str):
        payload = command.strip()
        if not payload:
            return
        if not payload.endswith("\r\n"):
            payload += "\r\n"
        data = payload.encode("ascii", errors="ignore")
        with self.serial_lock:
            self.serial.write(data)
            if self.args.flush_serial_writes:
                self.serial.flush()

    def _send_twist_mm_s(self, vx: int, vy: int, wz: int, *, force: bool = False):
        now = time.monotonic()
        if not force:
            if (vx, vy, wz) == self.last_sent:
                if (vx, vy, wz) != (0, 0, 0):
                    pass
                elif now - self.last_zero_sent_at < self.args.zero_repeat_period:
                    return
        self.seq = (self.seq % 999999) + 1
        self._write_command(f"$TWIST:SEQ={self.seq},VX={vx},VY={vy},WZ={wz}!")
        self.last_sent = (vx, vy, wz)
        if (vx, vy, wz) == (0, 0, 0):
            self.last_zero_sent_at = now

    def _on_twist(self, msg: Twist):
        vx = self._clamp(
            round(msg.linear.x * 1000.0 * self.args.cmd_linear_scale * self.args.cmd_vx_sign),
            -self.args.max_linear_mm_s,
            self.args.max_linear_mm_s,
        )
        vy = self._clamp(
            round(msg.linear.y * 1000.0 * self.args.cmd_linear_scale * self.args.cmd_vy_sign),
            -self.args.max_linear_mm_s,
            self.args.max_linear_mm_s,
        )
        wz = self._clamp(
            round(msg.angular.z * 1000.0 * self.args.cmd_angular_scale * self.args.cmd_wz_sign),
            -self.args.max_angular_mrad_s,
            self.args.max_angular_mrad_s,
        )
        self.target_vx = vx
        self.target_vy = vy
        self.target_wz = wz
        self.last_cmd_time = time.monotonic()
        self._send_twist_mm_s(vx, vy, wz, force=True)

    def _on_arm_cmd(self, msg: String):
        now = time.monotonic()
        if now - self.last_arm_cmd_time < max(0.0, self.args.arm_min_interval):
            return
        payload = self._sanitize_arm_payload(str(msg.data))
        if not payload:
            self.arm_cmd_reject_count += 1
            self.get_logger().warning("rejected empty or invalid /arm_cmd payload")
            return
        self._write_command(payload)
        self.arm_cmd_count += 1
        self.last_arm_cmd = payload[:160]
        self.last_arm_cmd_time = now

    @staticmethod
    def _sanitize_arm_payload(payload: str) -> str:
        payload = str(payload or "").strip()
        if not payload:
            return ""
        if len(payload) > 512:
            return ""
        if any(ord(ch) < 32 for ch in payload):
            return ""
        if payload[0] not in ("#", "$", "{"):
            return ""
        if not payload.endswith("!") and not payload.endswith("}"):
            return ""
        if not all(32 <= ord(ch) < 127 for ch in payload):
            return ""
        return payload

    def _command_timer(self):
        now = time.monotonic()
        if self.last_cmd_time <= 0.0 or now - self.last_cmd_time > self.args.cmd_timeout:
            self.target_vx = 0
            self.target_vy = 0
            self.target_wz = 0
        self._send_twist_mm_s(self.target_vx, self.target_vy, self.target_wz)

    def _reader_loop(self):
        buffer = b""
        while not self.stop_event.is_set():
            try:
                chunk = self.serial.read(256)
            except Exception as exc:
                self.get_logger().error(f"serial read failed: {exc}")
                time.sleep(0.2)
                continue
            if not chunk:
                continue
            buffer += chunk
            while b"!" in buffer:
                bang_index = buffer.find(b"!")
                frame_bytes = buffer[: bang_index + 1]
                buffer = buffer[bang_index + 1 :]
                text = frame_bytes.decode("ascii", errors="ignore").strip()
                at_index = text.rfind("@")
                if at_index >= 0:
                    self._handle_frame(text[at_index:])
                elif text:
                    self.unknown_count += 1
            if len(buffer) > 2048:
                buffer = buffer[-256:]

    def _handle_frame(self, line: str):
        self.rx_count += 1
        self.last_rx_time = time.monotonic()

        match = ODOM_RE.fullmatch(line)
        if match:
            frame = OdomFrame(
                seq=int(match.group("seq")),
                dt_s=self._safe_dt(int(match.group("dt")) / 1000.0),
                encoder_ticks=(
                    int(match.group("ea")),
                    int(match.group("eb")),
                    int(match.group("ec")),
                    int(match.group("ed")),
                ),
                wheel_mm_s=(
                    int(match.group("wa")),
                    int(match.group("wb")),
                    int(match.group("wc")),
                    int(match.group("wd")),
                ),
            )
            self._handle_odom(frame)
            return

        if line.startswith("@STATE:") or line.startswith("@MODE:") or line == "@PONG!":
            self.last_state = line
            self.state_count += 1
            self._publish_status()
            return

        if TWIST_OK_RE.fullmatch(line):
            self.last_twist_ack = line
            self.twist_ok_count += 1
            return

        if TWIST_ERR_RE.fullmatch(line):
            self.last_twist_ack = line
            self.twist_err_count += 1
            self.get_logger().warning(f"F103 rejected twist: {line}")
            return

        self.unknown_count += 1
        self.get_logger().debug(f"ignored F103 frame: {line}")

    def _handle_odom(self, frame: OdomFrame):
        if self.last_odom_seq is not None and frame.seq < self.last_odom_seq:
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
        self.last_odom_seq = frame.seq
        if self.odom_count == 0:
            self.get_logger().info(f"first F103 @ODOM received: seq={frame.seq}")

        vx, vy, wz = self._compute_body_twist(frame.wheel_mm_s)
        dt = frame.dt_s

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt
        self.yaw = self._normalize_angle(self.yaw + wz * dt)
        self.odom_count += 1

        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.args.odom_frame
        message.child_frame_id = self.args.base_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.position.z = 0.0
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = vx
        message.twist.twist.linear.y = vy
        message.twist.twist.angular.z = wz
        message.pose.covariance = self._pose_covariance()
        message.twist.covariance = self._twist_covariance()
        self.odom_pub.publish(message)
        if self.odom_count == 1:
            self.get_logger().info(f"first {self.args.odom_topic} published from F103 @ODOM")

    def _compute_body_twist(self, wheel_mm_s: tuple[int, int, int, int]) -> tuple[float, float, float]:
        v_a = wheel_mm_s[0] / 1000.0
        v_b = -wheel_mm_s[1] / 1000.0
        v_c = wheel_mm_s[2] / 1000.0
        v_d = -wheel_mm_s[3] / 1000.0
        base = max(1e-6, self.args.wheel_base_m + self.args.axle_base_m)

        vx = (v_a + v_b + v_c + v_d) / 4.0
        vy = (v_a - v_b - v_c + v_d) / 4.0
        wz = (v_a - v_b + v_c - v_d) / (2.0 * base)

        vx *= self.args.odom_linear_scale * self.args.odom_vx_sign
        vy *= self.args.odom_linear_scale * self.args.odom_vy_sign
        wz *= self.args.odom_angular_scale * self.args.odom_wz_sign
        return vx, vy, wz

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps(
            {
                "port": self.args.port,
                "rx_count": self.rx_count,
                "odom_count": self.odom_count,
                "state_count": self.state_count,
                "twist_ok_count": self.twist_ok_count,
                "twist_err_count": self.twist_err_count,
                "unknown_count": self.unknown_count,
                "arm_cmd_count": self.arm_cmd_count,
                "arm_cmd_reject_count": self.arm_cmd_reject_count,
                "last_arm_cmd": self.last_arm_cmd,
                "last_arm_cmd_age_s": None
                if self.last_arm_cmd_time <= 0.0
                else round(time.monotonic() - self.last_arm_cmd_time, 3),
                "last_state": self.last_state,
                "last_twist_ack": self.last_twist_ack,
                "last_odom_seq": self.last_odom_seq,
                "last_rx_age_s": None if self.last_rx_time <= 0.0 else round(time.monotonic() - self.last_rx_time, 3),
                "target_vx_mm_s": self.target_vx,
                "target_vy_mm_s": self.target_vy,
                "target_wz_mrad_s": self.target_wz,
                "pose_x": round(self.x, 4),
                "pose_y": round(self.y, 4),
                "yaw": round(self.yaw, 4),
            },
            ensure_ascii=True,
        )
        self.state_pub.publish(msg)

    @staticmethod
    def _pose_covariance() -> list[float]:
        cov = [0.0] * 36
        cov[0] = 0.04
        cov[7] = 0.04
        cov[14] = 999.0
        cov[21] = 999.0
        cov[28] = 999.0
        cov[35] = 0.08
        return cov

    @staticmethod
    def _twist_covariance() -> list[float]:
        cov = [0.0] * 36
        cov[0] = 0.05
        cov[7] = 0.05
        cov[14] = 999.0
        cov[21] = 999.0
        cov[28] = 999.0
        cov[35] = 0.08
        return cov

    @staticmethod
    def _safe_dt(dt_s: float) -> float:
        if dt_s < 0.001 or dt_s > 0.2:
            return 0.02
        return dt_s

    @staticmethod
    def _normalize_angle(value: float) -> float:
        while value > math.pi:
            value -= 2.0 * math.pi
        while value < -math.pi:
            value += 2.0 * math.pi
        return value

    @staticmethod
    def _clamp(value: int, low: int, high: int) -> int:
        return max(low, min(high, int(value)))

    @staticmethod
    def _resolve_serial_port(port: str) -> str:
        if port and os.path.exists(port):
            return port
        detected = F103UsbRos2Bridge._detect_f103_port()
        if detected:
            return detected
        raise SystemExit(
            f"F103 serial port not found: {port}. "
            "Connect the F103 USB serial or pass --port explicitly. "
            "The auto detector excludes /dev/rplidar and /dev/myimu."
        )

    @staticmethod
    def _detect_f103_port() -> str:
        excluded: set[str] = set()
        for link in ("/dev/laser", "/dev/imu", "/dev/rplidar", "/dev/myimu"):
            if os.path.exists(link):
                try:
                    excluded.add(os.path.realpath(link))
                except Exception:
                    pass

        candidates_by_real: dict[str, str] = {}
        for pattern in ("/dev/f103", "/dev/f103_usb", "/dev/serial/by-path/*", "/dev/ttyUSB*", "/dev/ttyACM*"):
            for candidate in glob.glob(pattern):
                try:
                    real = os.path.realpath(candidate)
                except Exception:
                    continue
                if real in excluded:
                    continue
                candidates_by_real.setdefault(real, candidate)
        candidates = list(candidates_by_real.values())
        return candidates[0] if len(candidates) == 1 else ""


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Linux USB serial bridge from ROS2 Twist to F103; publishes internal F103 odom on /odom_raw."
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--node-name", default="f103_usb_ros2_bridge")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--extra-cmd-topic", default="/cmd_vel_cmd")
    parser.add_argument("--arm-topic", default="/arm_cmd")
    parser.add_argument("--disable-arm-topic", action="store_true")
    parser.add_argument("--arm-min-interval", type=float, default=0.02)
    parser.add_argument("--odom-topic", default="/odom_raw")
    parser.add_argument("--state-topic", default="/f103_state")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--command-rate-hz", type=float, default=10.0)
    parser.add_argument("--cmd-timeout", type=float, default=0.45)
    parser.add_argument("--zero-repeat-period", type=float, default=0.5)
    parser.add_argument("--status-period", type=float, default=1.0)
    parser.add_argument("--handshake-interval", type=float, default=0.08)
    parser.add_argument("--keep-input-buffer", action="store_true", default=True)
    parser.add_argument("--clear-input-buffer", dest="keep_input_buffer", action="store_false")
    parser.add_argument("--flush-serial-writes", action="store_true", default=False)
    parser.add_argument("--max-linear-mm-s", type=int, default=900)
    parser.add_argument("--max-angular-mrad-s", type=int, default=900)
    parser.add_argument("--wheel-base-m", type=float, default=WHEEL_BASE_M)
    parser.add_argument("--axle-base-m", type=float, default=AXLE_BASE_M)
    parser.add_argument("--cmd-linear-scale", type=float, default=1.0)
    parser.add_argument("--cmd-angular-scale", type=float, default=1.0)
    parser.add_argument("--cmd-vx-sign", type=float, default=1.0)
    parser.add_argument("--cmd-vy-sign", type=float, default=1.0)
    parser.add_argument("--cmd-wz-sign", type=float, default=1.0)
    parser.add_argument("--odom-linear-scale", type=float, default=1.0)
    parser.add_argument("--odom-angular-scale", type=float, default=1.0)
    parser.add_argument("--odom-vx-sign", type=float, default=1.0)
    parser.add_argument("--odom-vy-sign", type=float, default=1.0)
    parser.add_argument("--odom-wz-sign", type=float, default=1.0)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    rclpy.init()
    node = None
    try:
        node = F103UsbRos2Bridge(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
import string
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

try:
    import serial
except Exception as exc:  # pragma: no cover - board runtime dependency
    raise SystemExit(f"pyserial unavailable: {exc}") from exc


DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B61032904-if00"
DEFAULT_BAUDRATE = 115200
DEFAULT_CMD_TOPIC = "/bus_servo_cmd"
DEFAULT_STATE_TOPIC = "/bus_servo_state"
MAX_PAYLOAD_LEN = 512
DANGEROUS_PATTERNS = (
    r"PID\d{3}",
    r"PBD\d",
    r"PSCK",
    r"PCSD",
    r"PCSM\d?",
    r"PSMI",
    r"PSMX",
    r"PCLE0?",
)


def is_printable_ascii(payload: str) -> bool:
    return all(ch in string.printable and ch not in "\x0b\x0c" for ch in payload)


def safe_display(data: bytes | str) -> str:
    if isinstance(data, str):
        data = data.encode("utf-8", errors="replace")
    text = data.decode("ascii", errors="backslashreplace")
    return "".join(ch if ch in string.printable and ch not in "\x0b\x0c" else f"\\x{ord(ch):02x}" for ch in text)


def contains_dangerous_config(payload: str) -> bool:
    upper = payload.upper()
    return any(re.search(pattern, upper) for pattern in DANGEROUS_PATTERNS)


def sanitize_bus_servo_payload(payload: str, *, allow_config: bool = False) -> tuple[str, str]:
    payload = str(payload or "").strip()
    if not payload:
        return "", "empty payload"
    if len(payload) > MAX_PAYLOAD_LEN:
        return "", f"payload too long: {len(payload)} > {MAX_PAYLOAD_LEN}"
    if not is_printable_ascii(payload):
        return "", "payload contains non-printable characters"
    if payload[0] == "{":
        if not payload.endswith("}"):
            return "", "multi-servo payload must end with }"
        inner = payload[1:-1]
        if not inner or "#" not in inner:
            return "", "multi-servo payload has no servo command"
        if not re.fullmatch(r"(#[0-9]{3}P[0-9]{4}T[0-9]{4}!)+", inner):
            return "", "multi-servo payload must be {#000P1500T1000!...}"
    elif payload[0] == "#":
        if not payload.endswith("!"):
            return "", "single command must end with !"
        if not re.fullmatch(r"#[0-9]{3}P[A-Za-z0-9]+!", payload):
            return "", "single command must look like #000P...!"
    else:
        return "", "payload must start with # or {"
    if contains_dangerous_config(payload) and not allow_config:
        return "", "configuration command rejected; restart bridge with --allow-config to permit it"
    return payload, ""


class BusServoBridge(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("bus_servo_ros2_bridge")
        self.args = args
        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.command_count = 0
        self.reject_count = 0
        self.rx_count = 0
        self.error_count = 0
        self.last_tx = ""
        self.last_rx = ""
        self.last_tx_time = 0.0
        self.last_rx_time = 0.0
        self.last_error = ""

        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(String, args.cmd_topic, self._on_command, cmd_qos)
        self.state_pub = self.create_publisher(String, args.state_topic, state_qos)

        self.serial = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            timeout=args.read_timeout,
            write_timeout=args.write_timeout,
        )
        with self.serial_lock:
            if not args.keep_input_buffer:
                self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

        self.reader_thread = threading.Thread(target=self._reader_loop, name="bus-servo-rx", daemon=True)
        self.reader_thread.start()
        self.state_timer = self.create_timer(max(0.2, args.state_period), self._publish_state)
        self.get_logger().info(
            f"bus servo bridge opened {args.port} @ {args.baudrate}; "
            f"subscribe {args.cmd_topic}; publish {args.state_topic}; "
            f"allow_config={args.allow_config}"
        )

    def destroy_node(self):
        self.stop_event.set()
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

    def _on_command(self, msg: String) -> None:
        payload, reason = sanitize_bus_servo_payload(str(msg.data), allow_config=self.args.allow_config)
        if not payload:
            self.reject_count += 1
            self.last_error = reason
            self.get_logger().warning(f"rejected /bus_servo_cmd: {reason}")
            return
        self._write_payload(payload)

    def _write_payload(self, payload: str) -> None:
        data = payload.encode("ascii")
        if self.args.append_crlf:
            data += b"\r\n"
        try:
            with self.serial_lock:
                self.serial.write(data)
                if self.args.flush_serial_writes:
                    self.serial.flush()
            self.command_count += 1
            self.last_tx = payload
            self.last_tx_time = time.monotonic()
            self.get_logger().info(f"TX {payload}")
        except Exception as exc:
            self.error_count += 1
            self.last_error = str(exc)
            self.get_logger().error(f"serial write failed: {exc}")

    def _reader_loop(self) -> None:
        buffer = b""
        while not self.stop_event.is_set():
            try:
                data = self.serial.read(256)
            except Exception as exc:
                self.error_count += 1
                self.last_error = str(exc)
                time.sleep(0.2)
                continue
            if not data:
                continue
            buffer += data
            if len(buffer) > 1024:
                buffer = buffer[-1024:]
            while b"\n" in buffer or b"\r" in buffer:
                split_positions = [pos for pos in (buffer.find(b"\n"), buffer.find(b"\r")) if pos >= 0]
                pos = min(split_positions)
                line = buffer[:pos].strip()
                buffer = buffer[pos + 1 :]
                if line:
                    self._record_rx(line)
            if buffer and time.monotonic() - self.last_rx_time > self.args.rx_idle_flush:
                line = buffer.strip()
                buffer = b""
                if line:
                    self._record_rx(line)

    def _record_rx(self, line: bytes) -> None:
        text = safe_display(line)
        self.rx_count += 1
        self.last_rx = text
        self.last_rx_time = time.monotonic()
        self.get_logger().info(f"RX {text}")

    def _publish_state(self) -> None:
        now = time.monotonic()
        state = {
            "port": self.args.port,
            "baudrate": self.args.baudrate,
            "cmd_topic": self.args.cmd_topic,
            "state_topic": self.args.state_topic,
            "append_crlf": self.args.append_crlf,
            "serial_open": bool(self.serial and self.serial.is_open),
            "command_count": self.command_count,
            "reject_count": self.reject_count,
            "rx_count": self.rx_count,
            "error_count": self.error_count,
            "last_tx": self.last_tx,
            "last_rx": self.last_rx,
            "last_tx_age_s": None if self.last_tx_time <= 0 else round(now - self.last_tx_time, 3),
            "last_rx_age_s": None if self.last_rx_time <= 0 else round(now - self.last_rx_time, 3),
            "last_error": self.last_error,
            "allow_config": self.args.allow_config,
        }
        msg = String()
        msg.data = json.dumps(state, ensure_ascii=False, sort_keys=True)
        self.state_pub.publish(msg)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS2 bridge for ASCII bus servo commands")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument("--cmd-topic", default=DEFAULT_CMD_TOPIC)
    parser.add_argument("--state-topic", default=DEFAULT_STATE_TOPIC)
    parser.add_argument("--allow-config", action="store_true", help="allow ID/baudrate/startup/factory-reset config commands")
    parser.add_argument("--no-append-crlf", dest="append_crlf", action="store_false")
    parser.add_argument("--keep-input-buffer", action="store_true")
    parser.add_argument("--no-flush-serial-writes", dest="flush_serial_writes", action="store_false")
    parser.add_argument("--read-timeout", type=float, default=0.05)
    parser.add_argument("--write-timeout", type=float, default=0.5)
    parser.add_argument("--state-period", type=float, default=1.0)
    parser.add_argument("--rx-idle-flush", type=float, default=0.1)
    parser.set_defaults(append_crlf=True, flush_serial_writes=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    node = None
    try:
        node = BusServoBridge(args)
        rclpy.spin(node)
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

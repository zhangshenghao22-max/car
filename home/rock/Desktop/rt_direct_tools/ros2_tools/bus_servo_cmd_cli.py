#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
import sys
import time
from typing import Iterable

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from std_msgs.msg import String as RosString


TOPIC_NAME = "/bus_servo_cmd"
SERVO_MIN_PWM = 500
SERVO_MAX_PWM = 2500
SERVO_MIN_ID = 0
SERVO_MAX_ID = 254
BROADCAST_ID = 255
DEFAULT_DURATION_MS = 1000
DEFAULT_HOME = [2300, 1500, 1500, 2200, 1500, 1500]


def clamp_pwm(value: int) -> int:
    return max(SERVO_MIN_PWM, min(SERVO_MAX_PWM, int(value)))


def clamp_duration(value: int) -> int:
    return max(0, min(9999, int(value)))


def check_servo_id(value: int) -> int:
    value = int(value)
    if value < SERVO_MIN_ID or value > SERVO_MAX_ID:
        raise ValueError(f"servo id out of range: {value}")
    return value


def check_query_id(value: int) -> int:
    value = int(value)
    if value < SERVO_MIN_ID or value > BROADCAST_ID:
        raise ValueError(f"query id out of range: {value}")
    return value


def build_servo_payload(pairs: Iterable[tuple[int, int]], duration_ms: int) -> str:
    parts: list[str] = []
    duration_ms = clamp_duration(duration_ms)
    for index, pwm in pairs:
        index = check_servo_id(index)
        parts.append(f"#{index:03d}P{clamp_pwm(pwm):04d}T{duration_ms:04d}!")
    if not parts:
        raise ValueError("no valid servo targets provided")
    if len(parts) == 1:
        return parts[0]
    return "{" + "".join(parts) + "}"


def build_simple_payload(servo_id: int, suffix: str, *, allow_broadcast: bool = False) -> str:
    if allow_broadcast:
        servo_id = check_query_id(servo_id)
    else:
        servo_id = check_servo_id(servo_id)
    suffix = str(suffix).strip().upper()
    if not suffix or not re.fullmatch(r"[A-Z0-9]+", suffix):
        raise ValueError(f"invalid command suffix: {suffix!r}")
    return f"#{servo_id:03d}P{suffix}!"


def parse_set_values(values: list[str]) -> list[tuple[int, int]]:
    pairs: list[tuple[int, int]] = []
    for item in values:
        if "=" not in item:
            raise ValueError(f"invalid --set value: {item}")
        left, right = item.split("=", 1)
        pairs.append((int(left.strip()), int(right.strip())))
    return pairs


def sanitize_raw_payload(payload: str) -> str:
    payload = str(payload or "").strip()
    if not payload:
        raise ValueError("raw payload is empty")
    if len(payload) > 512:
        raise ValueError("raw payload is too long")
    if payload[0] == "{":
        if not payload.endswith("}"):
            raise ValueError("multi-servo raw payload must end with }")
    elif payload[0] == "#":
        if not payload.endswith("!"):
            raise ValueError("single raw payload must end with !")
    else:
        raise ValueError("raw payload must start with # or {")
    if any(ord(ch) < 32 or ord(ch) > 126 for ch in payload):
        raise ValueError("raw payload must be printable ASCII")
    return payload


def publish_payload(payload: str, wait_subscriber_s: float, topic_name: str) -> int:
    context = Context()
    context.init(args=None)
    node = Node("bus_servo_cmd_cli", context=context)
    publisher = node.create_publisher(RosString, topic_name, 10)
    try:
        deadline = time.time() + max(0.0, wait_subscriber_s)
        sub_count = 0
        while time.time() < deadline:
            sub_count = len(node.get_subscriptions_info_by_topic(topic_name))
            if sub_count > 0:
                break
            time.sleep(0.1)
        if sub_count <= 0:
            print(f"no subscribers on {topic_name}", file=sys.stderr)
            return 1
        msg = RosString()
        msg.data = payload
        publisher.publish(msg)
        time.sleep(0.2)
        print(f"published {topic_name}: {payload}")
        print(f"subscriber_count={sub_count}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def publish_sequence(payloads: list[str], wait_subscriber_s: float, topic_name: str, interval_s: float) -> int:
    context = Context()
    context.init(args=None)
    node = Node("bus_servo_cmd_cli", context=context)
    publisher = node.create_publisher(RosString, topic_name, 10)
    try:
        deadline = time.time() + max(0.0, wait_subscriber_s)
        sub_count = 0
        while time.time() < deadline:
            sub_count = len(node.get_subscriptions_info_by_topic(topic_name))
            if sub_count > 0:
                break
            time.sleep(0.1)
        if sub_count <= 0:
            print(f"no subscribers on {topic_name}", file=sys.stderr)
            return 1
        for payload in payloads:
            msg = RosString()
            msg.data = payload
            publisher.publish(msg)
            print(f"published {topic_name}: {payload}")
            time.sleep(max(0.0, interval_s))
        print(f"subscriber_count={sub_count}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish ASCII bus servo commands over ROS2")
    parser.add_argument("--topic", default=TOPIC_NAME)
    parser.add_argument("--wait-subscriber", type=float, default=3.0)
    subparsers = parser.add_subparsers(dest="command", required=True)

    raw_parser = subparsers.add_parser("raw", help="publish a raw bus servo command")
    raw_parser.add_argument("payload")

    servo_parser = subparsers.add_parser("servo", help="send one or more servo PWM targets")
    servo_parser.add_argument("--set", action="append", required=True, help="target in ID=PWM form, for example 1=1500")
    servo_parser.add_argument("--duration", type=int, default=DEFAULT_DURATION_MS)

    home_parser = subparsers.add_parser("home", help="send the default six-axis home pose")
    home_parser.add_argument("--duration", type=int, default=2000)

    stop_parser = subparsers.add_parser("stop", help="stop one servo at current position")
    stop_parser.add_argument("--id", type=int, default=1)

    pause_parser = subparsers.add_parser("pause", help="pause one servo")
    pause_parser.add_argument("--id", type=int, default=1)

    resume_parser = subparsers.add_parser("resume", help="resume one paused servo")
    resume_parser.add_argument("--id", type=int, default=1)

    torque_release_parser = subparsers.add_parser("torque-release", help="release servo torque")
    torque_release_parser.add_argument("--id", type=int, default=1)

    torque_restore_parser = subparsers.add_parser("torque-restore", help="restore servo torque")
    torque_restore_parser.add_argument("--id", type=int, default=1)

    read_id_parser = subparsers.add_parser("read-id", help="read servo id")
    read_id_parser.add_argument("--id", type=int, default=255)

    version_parser = subparsers.add_parser("version", help="read servo version")
    version_parser.add_argument("--id", type=int, default=255)

    position_parser = subparsers.add_parser("position", help="read servo position")
    position_parser.add_argument("--id", type=int, default=255)

    mode_parser = subparsers.add_parser("mode", help="read servo mode")
    mode_parser.add_argument("--id", type=int, default=255)

    voltage_parser = subparsers.add_parser("voltage", help="read servo temperature and voltage")
    voltage_parser.add_argument("--id", type=int, default=255)

    safe_seq_parser = subparsers.add_parser("safe-seq", help="small safe motion sequence for one servo")
    safe_seq_parser.add_argument("--id", type=int, default=1)
    safe_seq_parser.add_argument("--center", type=int, default=1500)
    safe_seq_parser.add_argument("--delta", type=int, default=50)
    safe_seq_parser.add_argument("--duration", type=int, default=1000)
    safe_seq_parser.add_argument("--interval", type=float, default=1.2)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "raw":
            payload = sanitize_raw_payload(args.payload)
        elif args.command == "servo":
            payload = build_servo_payload(parse_set_values(args.set), args.duration)
        elif args.command == "home":
            payload = build_servo_payload(list(enumerate(DEFAULT_HOME)), args.duration)
        elif args.command == "stop":
            payload = build_simple_payload(args.id, "DST")
        elif args.command == "pause":
            payload = build_simple_payload(args.id, "DPT")
        elif args.command == "resume":
            payload = build_simple_payload(args.id, "DCT")
        elif args.command == "torque-release":
            payload = build_simple_payload(args.id, "ULK")
        elif args.command == "torque-restore":
            payload = build_simple_payload(args.id, "ULR")
        elif args.command == "read-id":
            payload = build_simple_payload(args.id, "ID", allow_broadcast=True)
        elif args.command == "version":
            payload = build_simple_payload(args.id, "VER", allow_broadcast=True)
        elif args.command == "position":
            payload = build_simple_payload(args.id, "RAD", allow_broadcast=True)
        elif args.command == "mode":
            payload = build_simple_payload(args.id, "MOD", allow_broadcast=True)
        elif args.command == "voltage":
            payload = build_simple_payload(args.id, "RTV", allow_broadcast=True)
        elif args.command == "safe-seq":
            center = clamp_pwm(args.center)
            delta = max(1, abs(int(args.delta)))
            low = clamp_pwm(center - delta)
            high = clamp_pwm(center + delta)
            payloads = [
                build_servo_payload([(check_servo_id(args.id), center)], args.duration),
                build_servo_payload([(check_servo_id(args.id), low)], args.duration),
                build_servo_payload([(check_servo_id(args.id), high)], args.duration),
                build_servo_payload([(check_servo_id(args.id), center)], args.duration),
                build_simple_payload(args.id, "DST"),
            ]
            return publish_sequence(payloads, args.wait_subscriber, args.topic, args.interval)
        else:
            print(f"unknown command: {args.command}", file=sys.stderr)
            return 2
    except Exception as exc:
        print(f"failed to build payload: {exc}", file=sys.stderr)
        return 2

    return publish_payload(payload, args.wait_subscriber, args.topic)


if __name__ == "__main__":
    raise SystemExit(main())

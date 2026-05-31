#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable

try:
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from std_msgs.msg import String as RosString
except Exception as exc:  # pragma: no cover - depends on board runtime
    print(f"arm_cmd_cli unavailable: {exc}", file=sys.stderr)
    raise SystemExit(2)


TOPIC_NAME = "/arm_cmd"
SERVO_COUNT = 6
SERVO_MIN_PWM = 500
SERVO_MAX_PWM = 2500
DEFAULT_DURATION_MS = 400
DEFAULT_HOME = [2300, 1450, 1500, 2200, 1500, 1700]
ARM_STOP_ALL = "#255PDST!"
GLOBAL_STOP = "$DST!"
GLOBAL_RESET = "$DJR!"


def clamp_pwm(value: int) -> int:
    return max(SERVO_MIN_PWM, min(SERVO_MAX_PWM, int(value)))


def clamp_duration(value: int) -> int:
    return max(20, min(5000, int(value)))


def build_servo_payload(pairs: Iterable[tuple[int, int]], duration_ms: int) -> str:
    parts: list[str] = []
    duration_ms = clamp_duration(duration_ms)
    for index, pwm in pairs:
        if index < 0 or index >= SERVO_COUNT:
            raise ValueError(f"servo index out of range: {index}")
        parts.append(f"#{index:03d}P{clamp_pwm(pwm):04d}T{duration_ms:04d}!")
    if not parts:
        raise ValueError("no valid servo targets provided")
    if len(parts) == 1:
        return parts[0]
    return "{" + "".join(parts) + "}"


def parse_set_values(values: list[str]) -> list[tuple[int, int]]:
    pairs: list[tuple[int, int]] = []
    for item in values:
        if "=" not in item:
            raise ValueError(f"invalid --set value: {item}")
        left, right = item.split("=", 1)
        pairs.append((int(left.strip()), int(right.strip())))
    return pairs


def build_servo_stop_payload(index: int) -> str:
    if index == 255:
        return ARM_STOP_ALL
    if index < 0 or index >= SERVO_COUNT:
        raise ValueError(f"servo index out of range: {index}")
    return f"#{index:03d}PDST!"


def build_kms_payload(x: float, y: float, z: float, duration_ms: int) -> str:
    return f"$KMS:{float(x):.1f},{float(y):.1f},{float(z):.1f},{clamp_duration(duration_ms)}!"


def validate_raw_payload(payload: str) -> str:
    payload = str(payload or "").strip()
    if not payload:
        raise ValueError("raw payload is empty")
    if len(payload) > 512:
        raise ValueError("raw payload is too long")
    if any(ord(ch) < 32 or ord(ch) >= 127 for ch in payload):
        raise ValueError("raw payload must be printable ASCII")
    if payload[0] not in ("#", "$", "{"):
        raise ValueError("raw payload must start with '#', '$', or '{'")
    if not payload.endswith("!") and not payload.endswith("}"):
        raise ValueError("raw payload must end with '!' or '}'")
    return payload


def wait_for_subscribers(node: Node, topic_name: str, timeout_s: float) -> int:
    deadline = time.time() + max(0.0, float(timeout_s))
    count = 0
    while time.time() < deadline:
        count = len(node.get_subscriptions_info_by_topic(topic_name))
        if count > 0:
            return count
        time.sleep(0.1)
    return count


def publish_payload(payload: str, topic_name: str, wait_subscriber_s: float) -> int:
    context = Context()
    context.init(args=None)
    node = Node("arm_cmd_cli", context=context)
    publisher = node.create_publisher(RosString, topic_name, 10)
    try:
        sub_count = wait_for_subscribers(node, topic_name, wait_subscriber_s)
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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Publish arm commands to the F103 mapping/navigation chain")
    parser.add_argument("--topic", default=TOPIC_NAME, help="ROS2 String topic used by the F103 bridge")
    parser.add_argument("--wait-subscriber", type=float, default=3.0, help="seconds to wait for topic subscribers")
    subparsers = parser.add_subparsers(dest="command", required=True)

    raw_parser = subparsers.add_parser("raw", help="publish a raw F103 arm command string")
    raw_parser.add_argument("payload", help="raw payload, for example '#000P1500T0500!' or '{#000P1500T1000!#001P1500T1000!}'")

    home_parser = subparsers.add_parser("home", help="move all servos to the default home pose")
    home_parser.add_argument("--duration", type=int, default=2000, help="motion duration in ms")

    servo_parser = subparsers.add_parser("servo", help="send one or more servo PWM targets")
    servo_parser.add_argument("--set", action="append", required=True, help="servo target in INDEX=PWM form")
    servo_parser.add_argument("--duration", type=int, default=DEFAULT_DURATION_MS, help="motion duration in ms")

    stop_servo_parser = subparsers.add_parser("stop-servo", help="stop one servo only, without stopping the chassis")
    stop_servo_parser.add_argument("--id", type=int, required=True, help="servo index, 0-5")

    subparsers.add_parser("stop-all-servos", help="stop all arm servos with #255PDST!, without stopping the chassis")
    subparsers.add_parser("stop", help="alias of stop-all-servos; does not stop the chassis")

    kms_parser = subparsers.add_parser("kms", help="send F103 kinematics move command: $KMS:x,y,z,time!")
    kms_parser.add_argument("x", type=float, help="target X in mm")
    kms_parser.add_argument("y", type=float, help="target Y in mm")
    kms_parser.add_argument("z", type=float, help="target Z in mm")
    kms_parser.add_argument("--duration", type=int, default=1000, help="motion duration in ms")

    reset_parser = subparsers.add_parser("reset", help="safe alias of home; move six arm servos only")
    reset_parser.add_argument("--duration", type=int, default=2000, help="motion duration in ms")
    subparsers.add_parser("global-reset", help="publish $DJR!: F103 global arm reset plus chassis stop")
    subparsers.add_parser("global-stop", help="publish $DST!: emergency stop for arm and chassis")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command == "raw":
            payload = validate_raw_payload(args.payload)
        elif args.command in ("home", "reset"):
            payload = build_servo_payload(list(enumerate(DEFAULT_HOME)), args.duration)
        elif args.command == "servo":
            payload = build_servo_payload(parse_set_values(args.set), args.duration)
        elif args.command == "stop-servo":
            payload = build_servo_stop_payload(args.id)
        elif args.command in ("stop", "stop-all-servos"):
            payload = ARM_STOP_ALL
        elif args.command == "kms":
            payload = build_kms_payload(args.x, args.y, args.z, args.duration)
        elif args.command == "global-reset":
            payload = GLOBAL_RESET
        elif args.command == "global-stop":
            payload = GLOBAL_STOP
        else:
            print(f"unknown command: {args.command}", file=sys.stderr)
            return 2
    except Exception as exc:
        print(f"failed to build arm payload: {exc}", file=sys.stderr)
        return 2

    return publish_payload(payload, args.topic, args.wait_subscriber)


if __name__ == "__main__":
    raise SystemExit(main())

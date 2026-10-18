#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapCapture(Node):
    def __init__(self, topic: str):
        super().__init__("cloud_map_save_fallback")
        self.map_msg: OccupancyGrid | None = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, topic, self._on_map, qos)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg


def yaml_quote(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def save_pgm_and_yaml(msg: OccupancyGrid, base_path: Path) -> None:
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = float(msg.info.resolution)
    if width <= 0 or height <= 0 or resolution <= 0:
        raise RuntimeError(f"invalid map metadata: width={width} height={height} resolution={resolution}")
    data = list(msg.data)
    expected = width * height
    if len(data) != expected:
        raise RuntimeError(f"invalid map data size: got {len(data)}, expected {expected}")

    pgm_path = base_path.with_suffix(".pgm")
    yaml_path = base_path.with_suffix(".yaml")
    base_path.parent.mkdir(parents=True, exist_ok=True)

    # ROS map files store row 0 at the top of the image, while OccupancyGrid
    # data starts at the map origin. Flip vertically to match map_saver_cli.
    with pgm_path.open("wb") as handle:
        handle.write(f"P5\n# CREATOR: cloud fallback map saver\n{width} {height}\n255\n".encode("ascii"))
        pixels = bytearray()
        for y in range(height - 1, -1, -1):
            row_offset = y * width
            for x in range(width):
                value = int(data[row_offset + x])
                if value < 0:
                    pixels.append(205)
                elif value >= 65:
                    pixels.append(0)
                elif value <= 25:
                    pixels.append(254)
                else:
                    pixels.append(205)
        handle.write(pixels)

    origin = msg.info.origin
    yaw = quaternion_to_yaw(
        float(origin.orientation.x),
        float(origin.orientation.y),
        float(origin.orientation.z),
        float(origin.orientation.w),
    )
    yaml_text = "\n".join(
        [
            f"image: {yaml_quote(pgm_path.name)}",
            "mode: trinary",
            f"resolution: {resolution:.12g}",
            "origin: "
            f"[{float(origin.position.x):.12g}, {float(origin.position.y):.12g}, {yaw:.12g}]",
            "negate: 0",
            "occupied_thresh: 0.65",
            "free_thresh: 0.25",
            "",
        ]
    )
    yaml_path.write_text(yaml_text, encoding="utf-8")


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wait_for_map(topic: str, timeout_s: float) -> OccupancyGrid:
    rclpy.init()
    node = MapCapture(topic)
    deadline = time.monotonic() + max(0.5, timeout_s)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.map_msg is not None:
                return node.map_msg
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise TimeoutError(f"no {topic} message within {timeout_s:.1f}s")


def main() -> int:
    parser = argparse.ArgumentParser(description="Fallback ROS2 OccupancyGrid map saver")
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--base-path", required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    base_path = Path(args.base_path).expanduser().resolve()
    try:
        msg = wait_for_map(args.topic, args.timeout)
        save_pgm_and_yaml(msg, base_path)
    except Exception as exc:
        print(f"fallback map save failed: {exc}", file=sys.stderr)
        return 1

    print(f"fallback map saved: {base_path.with_suffix('.yaml')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

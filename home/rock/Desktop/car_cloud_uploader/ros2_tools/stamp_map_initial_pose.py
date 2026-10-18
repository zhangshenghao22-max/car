#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time
from typing import Iterable

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

try:
    import yaml
except ImportError:  # pragma: no cover - board image should include PyYAML
    yaml = None


class PoseLookup(Node):
    def __init__(self) -> None:
        super().__init__("cloud_stamp_map_initial_pose")
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaml_path_from_arg(value: str) -> Path:
    path = Path(value).expanduser()
    if path.suffix.lower() == ".yaml":
        return path.resolve()
    return path.with_suffix(".yaml").resolve()


def wait_for_pose(
    map_frame: str,
    base_frames: Iterable[str],
    timeout_s: float,
) -> tuple[dict[str, float], str] | None:
    deadline = time.monotonic() + max(0.1, timeout_s)
    rclpy.init()
    node = PoseLookup()
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            for base_frame in base_frames:
                clean_base = str(base_frame or "").strip()
                if not clean_base:
                    continue
                try:
                    transform = node.buffer.lookup_transform(
                        map_frame,
                        clean_base,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.05),
                    )
                except TransformException:
                    continue

                translation = transform.transform.translation
                rotation = transform.transform.rotation
                pose = {
                    "x": float(translation.x),
                    "y": float(translation.y),
                    "yaw": yaw_from_quaternion(
                        float(rotation.x),
                        float(rotation.y),
                        float(rotation.z),
                        float(rotation.w),
                    ),
                }
                return pose, f"tf:{map_frame}->{clean_base}"
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return None


def write_initial_pose(yaml_path: Path, pose: dict[str, float]) -> None:
    if yaml is None:
        raise RuntimeError("PyYAML is not available")
    if not yaml_path.is_file():
        raise FileNotFoundError(f"map yaml does not exist: {yaml_path}")

    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"map yaml root must be a mapping: {yaml_path}")

    x = float(pose["x"])
    y = float(pose["y"])
    yaw = float(pose["yaw"])
    data["rt_robot_initial_pose"] = {
        "x": x,
        "y": y,
        "yaw": yaw,
    }
    data["initial_pose"] = {
        "x": x,
        "y": y,
        "z": 0.0,
        "yaw": yaw,
    }
    yaml_path.write_text(
        yaml.safe_dump(data, default_flow_style=False, sort_keys=False),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Write navigation initial pose into a ROS map yaml")
    parser.add_argument("map_path", help="Map yaml path or base path without .yaml")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_footprint")
    parser.add_argument("--fallback-base-frame", default="base_link")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--no-zero-fallback", action="store_true")
    args = parser.parse_args()

    yaml_path = yaml_path_from_arg(args.map_path)
    lookup = wait_for_pose(
        args.map_frame,
        (args.base_frame, args.fallback_base_frame),
        args.timeout,
    )
    if lookup is None:
        if args.no_zero_fallback:
            print(
                f"initial pose not saved: no TF {args.map_frame}->{args.base_frame} "
                f"or {args.map_frame}->{args.fallback_base_frame} within {args.timeout:.1f}s",
                file=sys.stderr,
            )
            return 1
        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        source = "fallback:zero"
    else:
        pose, source = lookup

    try:
        write_initial_pose(yaml_path, pose)
    except Exception as exc:
        print(f"initial pose write failed: {exc}", file=sys.stderr)
        return 1

    print(
        "initial pose saved: "
        f"{yaml_path} source={source} "
        f"x={pose['x']:.6f} y={pose['y']:.6f} yaw={pose['yaw']:.6f}"
    )
    if source == "fallback:zero":
        print("initial pose fallback to zero")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

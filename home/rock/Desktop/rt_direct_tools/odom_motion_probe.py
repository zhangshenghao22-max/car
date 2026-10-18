#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass, field
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String


def yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def angle_delta(a: float, b: float) -> float:
    value = b - a
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


@dataclass
class OdomSample:
    t: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float


@dataclass
class ProbeState:
    odom: list[OdomSample] = field(default_factory=list)
    imu_yaw: list[tuple[float, float]] = field(default_factory=list)
    scan_front: list[tuple[float, float]] = field(default_factory=list)
    f103_state: list[dict[str, Any]] = field(default_factory=list)


class OdomMotionProbe(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("odom_motion_probe")
        self.args = args
        self.state = ProbeState()
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, reliable_qos)
        self.create_subscription(Imu, args.imu_topic, self._on_imu, best_effort_qos)
        self.create_subscription(LaserScan, args.scan_topic, self._on_scan, best_effort_qos)
        self.create_subscription(String, args.f103_state_topic, self._on_f103_state, reliable_qos)

    def _on_odom(self, msg: Odometry):
        now = time.monotonic()
        pose = msg.pose.pose
        twist = msg.twist.twist
        self.state.odom.append(
            OdomSample(
                t=now,
                x=float(pose.position.x),
                y=float(pose.position.y),
                yaw=yaw_from_quat(float(pose.orientation.z), float(pose.orientation.w)),
                vx=float(twist.linear.x),
                vy=float(twist.linear.y),
                wz=float(twist.angular.z),
            )
        )

    def _on_imu(self, msg: Imu):
        now = time.monotonic()
        self.state.imu_yaw.append((now, yaw_from_quat(float(msg.orientation.z), float(msg.orientation.w))))

    def _on_scan(self, msg: LaserScan):
        if not msg.ranges:
            return
        now = time.monotonic()
        half_angle = math.radians(self.args.front_angle_deg) * 0.5
        values: list[float] = []
        for i, value in enumerate(msg.ranges):
            if not math.isfinite(value):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= half_angle and value >= self.args.scan_min_valid_m:
                values.append(float(value))
        if values:
            self.state.scan_front.append((now, min(values)))

    def _on_f103_state(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            payload = {"raw": msg.data}
        payload["_t"] = time.monotonic()
        self.state.f103_state.append(payload)

    def publish_cmd(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)


def spin_for(node: OdomMotionProbe, seconds: float, vx: float, vy: float, wz: float, rate_hz: float):
    deadline = time.monotonic() + seconds
    next_pub = 0.0
    period = 1.0 / max(1.0, rate_hz)
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_pub:
            node.publish_cmd(vx, vy, wz)
            next_pub = now + period
        rclpy.spin_once(node, timeout_sec=0.02)


def summarize(samples: list[OdomSample], t0: float, t1: float) -> dict[str, Any]:
    window = [s for s in samples if t0 <= s.t <= t1]
    if len(window) < 2:
        return {"count": len(window)}
    first = window[0]
    last = window[-1]
    dt = max(1e-6, last.t - first.t)
    dx = last.x - first.x
    dy = last.y - first.y
    dyaw = angle_delta(first.yaw, last.yaw)
    return {
        "count": len(window),
        "hz": round((len(window) - 1) / dt, 2),
        "dt_s": round(dt, 3),
        "dx_m": round(dx, 4),
        "dy_m": round(dy, 4),
        "dist_m": round(math.hypot(dx, dy), 4),
        "dyaw_rad": round(dyaw, 4),
        "avg_twist_vx": round(sum(s.vx for s in window) / len(window), 4),
        "avg_twist_vy": round(sum(s.vy for s in window) / len(window), 4),
        "avg_twist_wz": round(sum(s.wz for s in window) / len(window), 4),
        "first": {"x": round(first.x, 4), "y": round(first.y, 4), "yaw": round(first.yaw, 4)},
        "last": {"x": round(last.x, 4), "y": round(last.y, 4), "yaw": round(last.yaw, 4)},
    }


def summarize_pairs(values: list[tuple[float, float]], t0: float, t1: float, *, angular: bool = False) -> dict[str, Any]:
    window = [v for v in values if t0 <= v[0] <= t1]
    if len(window) < 2:
        return {"count": len(window)}
    dt = max(1e-6, window[-1][0] - window[0][0])
    if angular:
        delta = angle_delta(window[0][1], window[-1][1])
        return {
            "count": len(window),
            "hz": round((len(window) - 1) / dt, 2),
            "delta_rad": round(delta, 4),
            "first": round(window[0][1], 4),
            "last": round(window[-1][1], 4),
        }
    vals = [v[1] for v in window]
    return {
        "count": len(window),
        "hz": round((len(window) - 1) / dt, 2),
        "min": round(min(vals), 4),
        "max": round(max(vals), 4),
        "first": round(vals[0], 4),
        "last": round(vals[-1], 4),
    }


def summarize_f103(states: list[dict[str, Any]], t0: float, t1: float) -> dict[str, Any]:
    window = [s for s in states if t0 <= float(s.get("_t", 0.0)) <= t1]
    if not window:
        return {"count": 0}
    last = dict(window[-1])
    last.pop("_t", None)
    wheel_rows = [s.get("last_wheel_mm_s") for s in window if isinstance(s.get("last_wheel_mm_s"), list)]
    if not wheel_rows:
        return {"count": len(window), "last": last}
    cols = list(zip(*wheel_rows))
    avg = [round(sum(col) / len(col), 1) for col in cols]
    return {"count": len(window), "avg_last_wheel_mm_s": avg, "last": last}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--odom-topic", default="/odom_raw")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--f103-state-topic", default="/f103_state")
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--baseline", type=float, default=1.5)
    parser.add_argument("--after", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--front-angle-deg", type=float, default=50.0)
    parser.add_argument("--scan-min-valid-m", type=float, default=0.08)
    parser.add_argument("--front-stop-m", type=float, default=0.25)
    args = parser.parse_args()

    rclpy.init()
    node = OdomMotionProbe(args)
    try:
        start = time.monotonic()
        spin_for(node, args.baseline, 0.0, 0.0, 0.0, args.rate)
        move_start = time.monotonic()
        if args.vx > 0 and node.state.scan_front and node.state.scan_front[-1][1] < args.front_stop_m:
            blocked = True
            spin_for(node, 0.3, 0.0, 0.0, 0.0, args.rate)
        else:
            blocked = False
            spin_for(node, args.duration, args.vx, args.vy, args.wz, args.rate)
        move_end = time.monotonic()
        spin_for(node, args.after, 0.0, 0.0, 0.0, args.rate)
        stop_end = time.monotonic()
        for _ in range(5):
            node.publish_cmd(0.0, 0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)

        result = {
            "cmd": {"vx": args.vx, "vy": args.vy, "wz": args.wz, "duration_s": args.duration},
            "blocked_by_front_scan": blocked,
            "total": summarize(node.state.odom, start, stop_end),
            "baseline_odom": summarize(node.state.odom, start, move_start),
            "move_odom": summarize(node.state.odom, move_start, move_end),
            "after_odom": summarize(node.state.odom, move_end, stop_end),
            "move_imu_yaw": summarize_pairs(node.state.imu_yaw, move_start, move_end, angular=True),
            "move_front_scan": summarize_pairs(node.state.scan_front, move_start, move_end),
            "move_f103_state": summarize_f103(node.state.f103_state, move_start, move_end),
            "sample_counts": {
                "odom": len(node.state.odom),
                "imu": len(node.state.imu_yaw),
                "scan": len(node.state.scan_front),
                "f103_state": len(node.state.f103_state),
            },
        }
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        node.publish_cmd(0.0, 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

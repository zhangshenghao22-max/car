#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time


def probe_topic_endpoints(topic_name: str, endpoint_kind: str, timeout_s: float) -> int:
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
    except Exception as exc:
        print(f"topic probe unavailable: {exc}", file=sys.stderr)
        return 2

    context = Context()
    context.init(args=None)
    node = Node("car2_topic_graph_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.time() + max(0.2, float(timeout_s))
    count = 0
    try:
        while context.ok() and time.time() < deadline:
            remaining = max(0.05, min(0.2, deadline - time.time()))
            executor.spin_once(timeout_sec=remaining)
            if endpoint_kind == "subscriptions":
                count = len(node.get_subscriptions_info_by_topic(topic_name))
            else:
                count = len(node.get_publishers_info_by_topic(topic_name))
            if count > 0:
                break
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)

    if count > 0:
        print(f"{endpoint_kind}={count}")
        return 0
    print(f"timeout waiting for {endpoint_kind} on {topic_name}", file=sys.stderr)
    return 1


def probe_navigate_action(action_name: str, timeout_s: float) -> int:
    try:
        import rclpy
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
    except Exception as exc:
        print(f"action probe unavailable: {exc}", file=sys.stderr)
        return 2

    context = Context()
    context.init(args=None)
    node = Node("car2_nav_action_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    client = ActionClient(node, NavigateToPose, action_name)
    deadline = time.time() + max(0.2, float(timeout_s))
    ready = False
    try:
        while context.ok() and time.time() < deadline:
            remaining = max(0.05, min(0.2, deadline - time.time()))
            executor.spin_once(timeout_sec=0.0)
            if client.wait_for_server(timeout_sec=remaining):
                ready = True
                break
    finally:
        client.destroy()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)

    if ready:
        print(f"action server ready: {action_name}")
        return 0
    print(f"timeout waiting for action server {action_name}", file=sys.stderr)
    return 1


def probe_odom(timeout_s: float, topic_name: str = "/odom") -> int:
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
    except Exception as exc:
        print(f"odom probe unavailable: {exc}", file=sys.stderr)
        return 2

    class OdomWaiter(Node):
        def __init__(self, *, context: Context):
            super().__init__("car2_odom_probe", context=context)
            self.received = False
            self.create_subscription(Odometry, topic_name, self._on_msg, 10)

        def _on_msg(self, _message: Odometry):
            self.received = True

    context = Context()
    context.init(args=None)
    node = OdomWaiter(context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.time() + max(0.2, float(timeout_s))
    try:
        while context.ok() and time.time() < deadline and not node.received:
            remaining = max(0.05, min(0.2, deadline - time.time()))
            executor.spin_once(timeout_sec=remaining)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)

    if node.received:
        print(f"{topic_name} message received")
        return 0
    print(f"timeout waiting for {topic_name} message", file=sys.stderr)
    return 1


def probe_twist(timeout_s: float, topic_name: str = "/cmd_vel", reliability: str = "reliable") -> int:
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
    except Exception as exc:
        print(f"twist probe unavailable: {exc}", file=sys.stderr)
        return 2

    class TwistWaiter(Node):
        def __init__(self, *, context: Context):
            super().__init__("car2_twist_probe", context=context)
            self.received = False
            qos = QoSProfile(depth=20)
            if reliability == "best_effort":
                qos.reliability = ReliabilityPolicy.BEST_EFFORT
            else:
                qos.reliability = ReliabilityPolicy.RELIABLE
            self.create_subscription(Twist, topic_name, self._on_msg, qos)

        def _on_msg(self, _message: Twist):
            self.received = True

    context = Context()
    context.init(args=None)
    node = TwistWaiter(context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.time() + max(0.2, float(timeout_s))
    try:
        while context.ok() and time.time() < deadline and not node.received:
            remaining = max(0.05, min(0.2, deadline - time.time()))
            executor.spin_once(timeout_sec=remaining)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)

    if node.received:
        print(f"{topic_name} Twist message received")
        return 0
    print(f"timeout waiting for {topic_name} Twist message", file=sys.stderr)
    return 1


def probe_imu(timeout_s: float, topic_name: str = "/imu/data") -> int:
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu
    except Exception as exc:
        print(f"imu probe unavailable: {exc}", file=sys.stderr)
        return 2

    class ImuWaiter(Node):
        def __init__(self, *, context: Context):
            super().__init__("car2_imu_probe", context=context)
            self.received = False
            self.create_subscription(Imu, topic_name, self._on_msg, qos_profile_sensor_data)

        def _on_msg(self, _message: Imu):
            self.received = True

    context = Context()
    context.init(args=None)
    node = ImuWaiter(context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.time() + max(0.2, float(timeout_s))
    try:
        while context.ok() and time.time() < deadline and not node.received:
            remaining = max(0.05, min(0.2, deadline - time.time()))
            executor.spin_once(timeout_sec=remaining)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)

    if node.received:
        print(f"{topic_name} message received")
        return 0
    print(f"timeout waiting for {topic_name} message", file=sys.stderr)
    return 1


def probe_scan(timeout_s: float) -> int:
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
    except Exception as exc:
        print(f"scan probe unavailable: {exc}", file=sys.stderr)
        return 2

    class ScanWaiter(Node):
        def __init__(self, *, context: Context):
            super().__init__("car2_scan_probe", context=context)
            self.received = False
            self.create_subscription(LaserScan, "/scan", self._on_msg, qos_profile_sensor_data)

        def _on_msg(self, _message: LaserScan):
            self.received = True

    context = Context()
    context.init(args=None)
    node = ScanWaiter(context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.time() + max(0.2, float(timeout_s))
    try:
        while context.ok() and time.time() < deadline and not node.received:
            remaining = max(0.05, min(0.2, deadline - time.time()))
            executor.spin_once(timeout_sec=remaining)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)

    if node.received:
        print("/scan message received")
        return 0
    print("timeout waiting for /scan message", file=sys.stderr)
    return 1


def probe_map(timeout_s: float) -> int:
    try:
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    except Exception as exc:
        print(f"map probe unavailable: {exc}", file=sys.stderr)
        return 2

    class MapWaiter(Node):
        def __init__(self, *, context: Context):
            super().__init__("car2_map_probe", context=context)
            self.received = False
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(OccupancyGrid, "/map", self._on_msg, qos)

        def _on_msg(self, _message: OccupancyGrid):
            self.received = True

    context = Context()
    context.init(args=None)
    node = MapWaiter(context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.time() + max(0.2, float(timeout_s))
    try:
        while context.ok() and time.time() < deadline and not node.received:
            remaining = max(0.05, min(0.2, deadline - time.time()))
            executor.spin_once(timeout_sec=remaining)
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)

    if node.received:
        print("/map message received")
        return 0
    print("timeout waiting for /map message", file=sys.stderr)
    return 1


def probe_tf(target_frame: str, source_frame: str, timeout_s: float) -> int:
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformListener
    except Exception as exc:
        print(f"tf probe unavailable: {exc}", file=sys.stderr)
        return 2

    context = Context()
    context.init(args=None)
    node = Node("car2_tf_probe", context=context)
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    deadline = time.time() + max(0.2, float(timeout_s))
    ready = False
    try:
        while context.ok() and time.time() < deadline:
            remaining = max(0.05, min(0.2, deadline - time.time()))
            executor.spin_once(timeout_sec=remaining)
            try:
                buffer.lookup_transform(target_frame, source_frame, Time())
                ready = True
                break
            except Exception:
                pass
    finally:
        del listener
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=context)

    if ready:
        print(f"tf ready: {target_frame} -> {source_frame}")
        return 0
    print(f"timeout waiting for TF {target_frame} -> {source_frame}", file=sys.stderr)
    return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ROS2 runtime probe for /odom and TF readiness")
    subparsers = parser.add_subparsers(dest="command", required=True)

    odom_parser = subparsers.add_parser("odom", help="wait for a real nav_msgs/Odometry message")
    odom_parser.add_argument("--topic", default="/odom", help="odometry topic to subscribe")
    odom_parser.add_argument("--timeout", type=float, default=3.0, help="probe timeout in seconds")

    twist_parser = subparsers.add_parser("twist", help="wait for a real geometry_msgs/Twist message")
    twist_parser.add_argument("--topic", default="/cmd_vel", help="Twist topic to subscribe")
    twist_parser.add_argument(
        "--qos-reliability",
        choices=("reliable", "best_effort"),
        default="reliable",
        help="subscription reliability",
    )
    twist_parser.add_argument("--timeout", type=float, default=3.0, help="probe timeout in seconds")

    imu_parser = subparsers.add_parser("imu", help="wait for a real sensor_msgs/Imu message")
    imu_parser.add_argument("--topic", default="/imu/data", help="imu topic to subscribe")
    imu_parser.add_argument("--timeout", type=float, default=3.0, help="probe timeout in seconds")

    scan_parser = subparsers.add_parser("scan", help="wait for a real /scan message")
    scan_parser.add_argument("--timeout", type=float, default=3.0, help="probe timeout in seconds")

    map_parser = subparsers.add_parser("map", help="wait for a real /map message")
    map_parser.add_argument("--timeout", type=float, default=3.0, help="probe timeout in seconds")

    topic_parser = subparsers.add_parser("topic-endpoints", help="wait for topic publishers or subscriptions")
    topic_parser.add_argument("--topic", required=True, help="topic name to inspect")
    topic_parser.add_argument(
        "--kind",
        choices=("subscriptions", "publishers"),
        default="subscriptions",
        help="graph endpoint type to wait for",
    )
    topic_parser.add_argument("--timeout", type=float, default=3.0, help="probe timeout in seconds")

    action_parser = subparsers.add_parser("navigate-action", help="wait for a NavigateToPose action server")
    action_parser.add_argument("--name", default="/navigate_to_pose", help="action name to inspect")
    action_parser.add_argument("--timeout", type=float, default=3.0, help="probe timeout in seconds")

    tf_parser = subparsers.add_parser("tf", help="wait for a TF transform")
    tf_parser.add_argument("--target", required=True, help="target frame")
    tf_parser.add_argument("--source", required=True, help="source frame")
    tf_parser.add_argument("--timeout", type=float, default=3.0, help="probe timeout in seconds")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.command == "odom":
        return probe_odom(args.timeout, topic_name=args.topic)
    if args.command == "twist":
        return probe_twist(args.timeout, topic_name=args.topic, reliability=args.qos_reliability)
    if args.command == "imu":
        return probe_imu(args.timeout, topic_name=args.topic)
    if args.command == "scan":
        return probe_scan(args.timeout)
    if args.command == "map":
        return probe_map(args.timeout)
    if args.command == "topic-endpoints":
        return probe_topic_endpoints(args.topic, args.kind, args.timeout)
    if args.command == "navigate-action":
        return probe_navigate_action(args.name, args.timeout)
    if args.command == "tf":
        return probe_tf(args.target, args.source, args.timeout)
    print(f"unknown command: {args.command}", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

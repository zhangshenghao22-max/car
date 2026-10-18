#!/usr/bin/env python3
"""
Read initial pose from map yaml and publish it once to /initialpose.
Simplified version following mature project practices.
"""

import math
import os
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

try:
    import yaml
except ImportError:
    yaml = None


class AutoInitialPoseNode(Node):
    """
    Read an initial pose from a map yaml and publish it once to /initialpose.
    
    Expected yaml structure (one of):
      rt_robot_initial_pose:
        x: 1.0
        y: 2.0
        yaw: 0.0
      # or, backward compatible:
      initial_pose:
        x: 1.0
        y: 2.0
        yaw: 0.0
    """

    def __init__(self) -> None:
        super().__init__('auto_initialpose')

        self.declare_parameter('map_yaml', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_delay_sec', 2.0)
        self.declare_parameter('publish_repeats', 5)
        self.declare_parameter('repeat_interval_sec', 0.2)
        self.declare_parameter('tf_wait_timeout_sec', 10.0)

        self._map_yaml_path: str = self.get_parameter('map_yaml').get_parameter_value().string_value
        self._frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value
        self._delay: float = self.get_parameter('publish_delay_sec').get_parameter_value().double_value
        self._publish_repeats: int = max(
            1, self.get_parameter('publish_repeats').get_parameter_value().integer_value
        )
        self._repeat_interval: float = max(
            0.05, self.get_parameter('repeat_interval_sec').get_parameter_value().double_value
        )
        self._max_wait_time: float = max(
            self._delay, self.get_parameter('tf_wait_timeout_sec').get_parameter_value().double_value
        )
        self._published = False

        if not self._map_yaml_path:
            self.get_logger().warn('map_yaml parameter is empty, exiting.')
            self._shutdown_later()
            return

        if not os.path.isfile(self._map_yaml_path):
            self.get_logger().warn(f'map_yaml file does not exist: {self._map_yaml_path}')
            self._shutdown_later()
            return

        if yaml is None:
            self.get_logger().error('PyYAML is not available, cannot read initial pose.')
            self._shutdown_later()
            return

        self._initial_pose = self._load_initial_pose(self._map_yaml_path)
        if self._initial_pose is None:
            self.get_logger().info(
                f'No initial pose found in map yaml: {self._map_yaml_path} '
                f'(expected rt_robot_initial_pose or initial_pose). Exiting.'
            )
            self._shutdown_later()
            return

        initialpose_qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', initialpose_qos)
        
        # Subscribe to map with transient_local QoS to match map_server
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self._map_received = False
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            map_qos
        )
        self._map_client = self.create_client(GetMap, '/map_server/map')
        self._map_service_checked = False
        self._map_service_ready = False
        
        self.get_logger().info(
            f'Will publish initial pose from {self._map_yaml_path} '
            f'after map is ready and {self._delay:.1f}s delay: '
            f"x={self._initial_pose['x']:.3f}, y={self._initial_pose['y']:.3f}, "
            f"yaw={self._initial_pose['yaw']:.3f}, repeats={self._publish_repeats}, "
            f"interval={self._repeat_interval:.2f}s, max_wait={self._max_wait_time:.1f}s"
        )
        
        self._check_timer = self.create_timer(0.5, self._check_and_publish)
        self._start_time = self.get_clock().now()

    def _load_initial_pose(self, path: str) -> Optional[dict]:
        """Load initial pose from yaml file"""
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(f'Failed to read yaml file {path}: {exc}')
            return None

        pose = data.get('rt_robot_initial_pose') or data.get('initial_pose')
        if not isinstance(pose, dict):
            return None

        try:
            x = float(pose.get('x', 0.0))
            y = float(pose.get('y', 0.0))
            yaw = float(pose.get('yaw', 0.0))
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(f'Invalid initial pose format in {path}: {exc}')
            return None

        return {'x': x, 'y': y, 'yaw': yaw}

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """Callback when map is received"""
        if not self._map_received:
            self._map_received = True
            self.get_logger().info(
                f'Map received: {msg.info.width}x{msg.info.height}, '
                f'resolution: {msg.info.resolution:.3f}m/pixel'
            )

    def _check_and_publish(self) -> None:
        """Check if map is ready and delay has passed, then publish initial pose"""
        if self._published:
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

        if not self._map_service_checked and elapsed >= self._delay:
            self._map_service_checked = True
            if self._map_client.wait_for_service(timeout_sec=0.1):
                future = self._map_client.call_async(GetMap.Request())
                future.add_done_callback(self._map_service_callback)
            else:
                self.get_logger().warn('/map_server/map service is not available yet')

        if elapsed > self._max_wait_time:
            if not self._map_received and not self._map_service_ready:
                self.get_logger().error(
                    f'Timeout: no /map topic or /map_server/map service result after '
                    f'{self._max_wait_time:.1f}s. Publishing initial pose anyway.'
                )
            self._publish_once()
            self._check_timer.cancel()
            return
        
        if elapsed >= self._delay and (self._map_received or self._map_service_ready):
            self._publish_once()
            self._check_timer.cancel()

    def _map_service_callback(self, future) -> None:
        """Mark map ready when map_server service returns a valid map."""
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Failed to call /map_server/map: {exc}')
            return

        if response is None:
            self.get_logger().warn('/map_server/map returned no response')
            return

        info = response.map.info
        if info.width > 0 and info.height > 0:
            self._map_service_ready = True
            self.get_logger().info(
                f'Map service ready: {info.width}x{info.height}, '
                f'resolution: {info.resolution:.3f}m/pixel'
            )

    def _publish_once(self) -> None:
        """Publish initial pose once"""
        if self._published:
            return
        self._published = True

        if self._initial_pose is None:
            self._shutdown_later()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._frame_id

        # Stamp zero asks AMCL to use the latest available TF. A recent odom
        # stamp can race with AMCL's TF buffer during startup and trigger
        # extrapolation warnings.
        msg.header.stamp = rclpy.time.Time().to_msg()
        self.get_logger().info('Using zero timestamp for initial pose')
        msg.pose.pose.position.x = self._initial_pose['x']
        msg.pose.pose.position.y = self._initial_pose['y']
        msg.pose.pose.position.z = 0.0

        qz, qw = math.sin(self._initial_pose['yaw'] / 2.0), math.cos(self._initial_pose['yaw'] / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        # Default covariance for initial pose
        msg.pose.covariance[0] = 0.25   # x
        msg.pose.covariance[7] = 0.25   # y
        msg.pose.covariance[35] = (math.radians(5.0) ** 2)  # yaw

        deadline = self.get_clock().now().nanoseconds / 1e9 + 3.0
        while self._pub.get_subscription_count() < 1 and self.get_clock().now().nanoseconds / 1e9 < deadline:
            time.sleep(0.05)

        subscriptions = self._pub.get_subscription_count()
        self.get_logger().info(f'/initialpose subscriber count before publish: {subscriptions}')

        for _ in range(self._publish_repeats):
            self._pub.publish(msg)
            time.sleep(self._repeat_interval)
        self.get_logger().info(
            f'Published initial pose to /initialpose with BEST_EFFORT QoS '
            f'{self._publish_repeats} times'
        )
        self._shutdown_later()

    def _shutdown_later(self) -> None:
        """Shutdown node after a short delay"""
        self.create_timer(0.5, lambda: rclpy.shutdown())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoInitialPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

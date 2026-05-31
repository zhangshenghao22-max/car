#!/usr/bin/env python3
"""
Read initial pose from map yaml and publish it once to /initialpose.
Simplified version following mature project practices.
"""

import math
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

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

        # TF buffer for querying transform timestamps
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._map_yaml_path: str = self.get_parameter('map_yaml').get_parameter_value().string_value
        self._frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value
        self._delay: float = self.get_parameter('publish_delay_sec').get_parameter_value().double_value

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

        self._pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
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
        
        self.get_logger().info(
            f'Will publish initial pose from {self._map_yaml_path} '
            f'after map is ready and {self._delay:.1f}s delay: '
            f"x={self._initial_pose['x']:.3f}, y={self._initial_pose['y']:.3f}, "
            f"yaw={self._initial_pose['yaw']:.3f}"
        )
        
        self._check_timer = self.create_timer(0.5, self._check_and_publish)
        self._start_time = self.get_clock().now()
        self._max_wait_time = 10.0  # Maximum time to wait for map

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
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        
        # Check timeout
        if elapsed > self._max_wait_time:
            if not self._map_received:
                self.get_logger().error(
                    f'Timeout: Did not receive map after {self._max_wait_time}s. '
                    f'Publishing initial pose anyway.'
                )
            self._publish_once()
            self._check_timer.cancel()
            return
        
        # Check if map is received and delay has passed
        if self._map_received and elapsed >= self._delay:
            self._publish_once()
            self._check_timer.cancel()

    def _publish_once(self) -> None:
        """Publish initial pose once"""
        if self._initial_pose is None:
            self._shutdown_later()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._frame_id

        # Use TF timestamp to ensure synchronization with AMCL
        # Try to get the latest map->base_footprint transform timestamp
        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame_id,
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            # Use the transform's timestamp to ensure synchronization
            msg.header.stamp = transform.header.stamp
            self.get_logger().info(
                f'Using TF timestamp for initial pose: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
            )
        except Exception as e:
            # Fallback to current time if TF lookup fails
            self.get_logger().warn(
                f'TF lookup failed: {e}. Using current clock time as fallback.'
            )
            msg.header.stamp = self.get_clock().now().to_msg()
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

        self._pub.publish(msg)
        self.get_logger().info('Published initial pose to /initialpose')
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

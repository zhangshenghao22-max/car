#!/usr/bin/env python3

import math
import os
import sys
import fcntl
import time
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

try:
    import yaml
except ImportError:
    yaml = None

from tf2_ros import Buffer, TransformException, TransformListener
from geometry_msgs.msg import TransformStamped


class SaveInitialPoseToMap(Node):
    """
    After a map is saved, read the current robot pose in the map frame
    and write it into the map yaml as `rt_robot_initial_pose`.

    This node is intended to be started once (e.g., from save_map.launch.py
    after map_saver_cli finishes) and will shut itself down after writing.
    """

    def __init__(self) -> None:
        # Flag to signal that work is complete and node should exit
        self._should_exit = False
        super().__init__('save_initialpose_to_map')

        self.declare_parameter('map_base_path', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('timeout_sec', 5.0)

        base_path = self.get_parameter('map_base_path').get_parameter_value().string_value
        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._timeout = self.get_parameter('timeout_sec').get_parameter_value().double_value

        if not base_path:
            self.get_logger().warn('map_base_path parameter is empty, will not write initial pose.')
            self._shutdown_now()
            return

        # map_saver_cli uses -f <base_path> and appends .yaml / .pgm
        if base_path.endswith('.yaml'):
            self._yaml_path = base_path
        else:
            self._yaml_path = base_path + '.yaml'

        if yaml is None:
            self.get_logger().warn('PyYAML not available, cannot write initial pose to map yaml.')
            self._shutdown_now()
            return

        # TF buffer/listener
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            f'Will wait for map file {self._yaml_path} and then read transform '
            f'{self._map_frame} -> {self._base_frame}'
        )

        # File waiting state: 'waiting' -> 'ready' -> 'done'/'failed'
        self._file_state = 'waiting'
        self._file_wait_retry = 0
        self._file_wait_interval = 0.2  # Check every 200ms
        self._file_max_wait = 3.0  # Wait up to 3 seconds for file (map_saver needs time to flush)
        self._file_max_retries = int(self._file_max_wait / self._file_wait_interval)

        # Start a timer to wait for file, then wait for TF and then process
        # Use retry mechanism to wait for file and TF buffer to populate
        self._retry_count = 0
        self._timer_interval = 0.5  # Check every 0.5 seconds
        self._max_retries = int(self._timeout / self._timer_interval)  # Number of retries based on timeout
        self._timer = self.create_timer(self._file_wait_interval, self._on_timer)

    def _on_timer(self) -> None:
        # Phase 1: Wait for map file to be created by map_saver_cli
        if self._file_state == 'waiting':
            self._file_wait_retry += 1

            if os.path.isfile(self._yaml_path):
                self._file_state = 'ready'
                self.get_logger().info(
                    f'Map file detected: {self._yaml_path} '
                    f'(after {self._file_wait_retry * self._file_wait_interval:.1f}s)'
                )
                # Change timer interval to TF check interval
                self._timer.cancel()
                self._timer = self.create_timer(self._timer_interval, self._on_timer)
                return

            if self._file_wait_retry > self._file_max_retries:
                self.get_logger().error(
                    f'Map yaml file not found after {self._file_max_wait}s: {self._yaml_path} '
                    f'(map_saver may have failed). Initial pose not written.'
                )
                self._timer.cancel()
                self._shutdown_now()
                return

            # Log every 1 second (5 * 200ms)
            if self._file_wait_retry % 5 == 0:
                self.get_logger().info(
                    f'Waiting for map file {self._yaml_path}... '
                    f'({self._file_wait_retry * self._file_wait_interval:.1f}s/{self._file_max_wait}s)'
                )
            return

        # Phase 2: File exists, now wait for TF transform
        self._retry_count += 1

        # Check if we've exceeded max retries
        if self._retry_count > self._max_retries:
            self.get_logger().warn(
                f'Could not get transform {self._map_frame} -> {self._base_frame} after '
                f'{self._retry_count} attempts. Initial pose not written.'
            )
            self._timer.cancel()
            self._shutdown_now()
            return

        try:
            # First, check if frames exist in TF tree
            # Try to get any transform involving map frame to verify it exists
            try:
                # Check if map frame exists by trying to get its transform to odom
                _ = self._tf_buffer.lookup_transform(
                    self._map_frame,
                    'odom',
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
            except TransformException:
                # If map->odom doesn't exist, map frame might not be available yet
                if self._retry_count % 4 == 0:  # Log every 2 seconds (4 * 0.5s)
                    self.get_logger().info(
                        f'Waiting for {self._map_frame} frame to become available... '
                        f'(attempt {self._retry_count}/{self._max_retries})'
                    )
                return  # Retry on next timer tick

            # Now try to get the actual transform we need
            transform: TransformStamped = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )

            # Success! We got the transform
            self.get_logger().info(
                f'Successfully obtained transform {self._map_frame} -> {self._base_frame} '
                f'after {self._retry_count} attempts'
            )
            self._timer.cancel()
            self._write_initial_pose(transform)
            self._shutdown_now()

        except TransformException as ex:
            # Transform not available yet, retry
            if self._retry_count % 4 == 0:  # Log every 2 seconds
                self.get_logger().info(
                    f'Waiting for transform {self._map_frame} -> {self._base_frame}... '
                    f'(attempt {self._retry_count}/{self._max_retries}): {ex}'
                )
            # Continue to next timer tick for retry
            return

    def _write_initial_pose(self, transform: TransformStamped) -> None:
        # Extract x, y, yaw from transform
        x = float(transform.transform.translation.x)
        y = float(transform.transform.translation.y)

        q = transform.transform.rotation
        # yaw from quaternion - use correct 3D quaternion to yaw formula
        # This handles cases where quaternion may have small x/y components
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(
            f'Captured initial pose in {self._map_frame}: '
            f'x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
        )

        # Load existing yaml with file lock to prevent race conditions
        if not os.path.isfile(self._yaml_path):
            self.get_logger().warn(f'Map yaml file does not exist: {self._yaml_path}')
            return

        # Use file locking to ensure atomic read-modify-write operation
        # This prevents race conditions when multiple processes try to write to the same file
        max_lock_retries = 10
        lock_retry_delay = 0.1  # 100ms
        
        for attempt in range(max_lock_retries):
            try:
                # Open file in read-write mode for locking
                with open(self._yaml_path, 'r+') as f:
                    # Try to acquire exclusive lock (non-blocking)
                    try:
                        fcntl.flock(f.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError:
                        # File is locked by another process, wait and retry
                        if attempt < max_lock_retries - 1:
                            self.get_logger().debug(
                                f'File {self._yaml_path} is locked, retrying in {lock_retry_delay}s... '
                                f'(attempt {attempt + 1}/{max_lock_retries})'
                            )
                            time.sleep(lock_retry_delay)
                            continue
                        else:
                            self.get_logger().error(
                                f'Failed to acquire lock on {self._yaml_path} after {max_lock_retries} attempts. '
                                f'Another process may be writing to the file.'
                            )
                            return
                    
                    # Lock acquired, read and modify
                    try:
                        data = yaml.safe_load(f) or {}
                    except Exception as exc:
                        fcntl.flock(f.fileno(), fcntl.LOCK_UN)
                        self.get_logger().error(f'Failed to parse map yaml {self._yaml_path}: {exc}')
                        return
                    
                    # Write / overwrite initial_pose in parameter format (following mature project practices)
                    # Format: initial_pose.x, initial_pose.y, initial_pose.z, initial_pose.yaw
                    if 'initial_pose' not in data:
                        data['initial_pose'] = {}
                    data['initial_pose']['x'] = float(x)
                    data['initial_pose']['y'] = float(y)
                    data['initial_pose']['z'] = 0.0  # z is always 0.0 for 2D navigation
                    data['initial_pose']['yaw'] = float(yaw)
                    
                    # Write back to file (truncate first)
                    f.seek(0)
                    f.truncate()
                    yaml.safe_dump(data, f, default_flow_style=False)
                    f.flush()  # Ensure data is written to disk
                    os.fsync(f.fileno())  # Force write to disk
                    
                    # Lock is automatically released when file is closed
                    
                self.get_logger().info(
                    f'Wrote rt_robot_initial_pose to {self._yaml_path}: '
                    f"x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}"
                )
                return  # Success, exit retry loop
                
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Failed to write map yaml {self._yaml_path}: {exc}')
                if attempt < max_lock_retries - 1:
                    time.sleep(lock_retry_delay)
                    continue
                else:
                    return  # Give up after max retries

    def _shutdown_now(self) -> None:
        """
        Signal that work is complete and shutdown the node.
        Setting _should_exit flag will cause rclpy.spin() to exit.
        """
        try:
            # Cancel any pending timers
            if hasattr(self, '_timer') and self._timer is not None:
                self._timer.cancel()
        except Exception:
            pass

        # Set flag to signal main loop to exit
        self._should_exit = True

        # Shutdown ROS context - this will cause rclpy.spin() to return
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SaveInitialPoseToMap()

    try:
        # Use spin_once in a loop so we can check the exit flag
        while rclpy.ok() and not node._should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected exception: {e}')
    finally:
        # Cleanup
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()



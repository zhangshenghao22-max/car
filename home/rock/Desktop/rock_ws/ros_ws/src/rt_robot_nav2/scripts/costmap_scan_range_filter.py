#!/usr/bin/env python3
"""Filter near self-echoes for Nav2 costmaps without changing raw /scan."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class CostmapScanRangeFilter(Node):
    def __init__(self):
        super().__init__("costmap_scan_range_filter")
        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_costmap")
        self.declare_parameter("min_obstacle_range", 0.25)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.min_obstacle_range = float(self.get_parameter("min_obstacle_range").value)

        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)
        self.subscription = self.create_subscription(LaserScan, self.input_topic, self._on_scan, 10)
        self.filtered_count = 0
        self.get_logger().info(
            f"costmap scan filter: {self.input_topic} -> {self.output_topic}, "
            f"min_obstacle_range={self.min_obstacle_range:.3f}m"
        )

    def _on_scan(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.intensities = msg.intensities

        ranges = []
        filtered = 0
        for value in msg.ranges:
            if math.isfinite(value) and value < self.min_obstacle_range:
                ranges.append(float("inf"))
                filtered += 1
            else:
                ranges.append(value)
        out.ranges = ranges
        self.filtered_count += filtered
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CostmapScanRangeFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

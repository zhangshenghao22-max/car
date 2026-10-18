#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuZCalibrator(Node):
    def __init__(self):
        super().__init__('imu_z_calibrator')

        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_calibrated')
        self.declare_parameter('imu_z_scale', 1.05502)
        self.declare_parameter('imu_z_bias', 0.0)
        self.declare_parameter('imu_yaw_rate_variance', 0.0025)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.imu_z_scale = float(self.get_parameter('imu_z_scale').value)
        self.imu_z_bias = float(self.get_parameter('imu_z_bias').value)
        self.imu_yaw_rate_variance = float(
            self.get_parameter('imu_yaw_rate_variance').value
        )

        self.publisher = self.create_publisher(
            Imu, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Imu, input_topic, self._on_imu, qos_profile_sensor_data
        )

        self.get_logger().info(
            'IMU Z calibrator: %s -> %s, z=(raw-%.6f)*%.6f, var=%.6f'
            % (
                input_topic,
                output_topic,
                self.imu_z_bias,
                self.imu_z_scale,
                self.imu_yaw_rate_variance,
            )
        )

    def _on_imu(self, msg):
        calibrated = copy.deepcopy(msg)
        calibrated.angular_velocity.z = (
            msg.angular_velocity.z - self.imu_z_bias
        ) * self.imu_z_scale
        calibrated.angular_velocity_covariance[8] = self.imu_yaw_rate_variance
        self.publisher.publish(calibrated)


def main(args=None):
    rclpy.init(args=args)
    node = ImuZCalibrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

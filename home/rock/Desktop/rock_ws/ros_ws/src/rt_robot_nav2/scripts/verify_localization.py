#!/usr/bin/env python3
"""
验证AMCL定位是否正确的脚本
检查：
1. AMCL粒子云是否收敛
2. 当前位姿与初始位姿的差异
3. TF变换是否正常
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
import tf2_ros
from tf2_ros import TransformException
import math
import time


class LocalizationVerifier(Node):
    def __init__(self):
        super().__init__('localization_verifier')
        
        # 订阅AMCL位姿和粒子云
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        self.particle_sub = self.create_subscription(
            ParticleCloud,
            '/particlecloud',
            self.particle_callback,
            10
        )
        
        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 状态变量
        self.initial_pose = None
        self.current_pose = None
        self.particle_count = 0
        self.particle_variance = 0.0
        self.last_check_time = time.time()
        self.check_interval = 2.0  # 每2秒检查一次
        
        # 创建定时器
        self.create_timer(1.0, self.check_localization_status)
        
        self.get_logger().info('Localization verifier started. Checking AMCL status...')
    
    def pose_callback(self, msg):
        """接收AMCL位姿"""
        self.current_pose = msg
        
        # 如果是第一次收到位姿，记录为初始位姿
        if self.initial_pose is None:
            self.initial_pose = msg
            self.get_logger().info(
                f'Initial pose received: x={msg.pose.pose.position.x:.3f}, '
                f'y={msg.pose.pose.position.y:.3f}, '
                f'yaw={self._quaternion_to_yaw(msg.pose.pose.orientation):.3f}'
            )
    
    def particle_callback(self, msg):
        """接收粒子云数据"""
        self.particle_count = len(msg.particles)
        
        if self.particle_count > 0:
            # 计算粒子云的位置方差（收敛度指标）
            positions = [(p.pose.position.x, p.pose.position.y) for p in msg.particles]
            if len(positions) > 1:
                mean_x = sum(p[0] for p in positions) / len(positions)
                mean_y = sum(p[1] for p in positions) / len(positions)
                variance = sum(
                    (p[0] - mean_x)**2 + (p[1] - mean_y)**2 
                    for p in positions
                ) / len(positions)
                self.particle_variance = variance
    
    def _quaternion_to_yaw(self, quaternion):
        """四元数转yaw角"""
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def _calculate_pose_difference(self, pose1, pose2):
        """计算两个位姿之间的差异"""
        if pose1 is None or pose2 is None:
            return None
        
        dx = pose2.pose.pose.position.x - pose1.pose.pose.position.x
        dy = pose2.pose.pose.position.y - pose1.pose.pose.position.y
        
        yaw1 = self._quaternion_to_yaw(pose1.pose.pose.orientation)
        yaw2 = self._quaternion_to_yaw(pose2.pose.pose.orientation)
        dyaw = yaw2 - yaw1
        
        # 归一化角度差到[-pi, pi]
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
        
        distance = math.sqrt(dx**2 + dy**2)
        
        return {
            'distance': distance,
            'dx': dx,
            'dy': dy,
            'dyaw': dyaw
        }
    
    def _check_tf_transform(self):
        """检查TF变换是否正常"""
        try:
            # 检查 map -> base_footprint 变换
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )
            
            # 检查 odom -> base_footprint 变换
            transform_odom = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time()
            )
            
            return True, transform, transform_odom
        except TransformException as ex:
            self.get_logger().warn(f'TF transform error: {ex}')
            return False, None, None
    
    def check_localization_status(self):
        """定期检查定位状态"""
        current_time = time.time()
        if current_time - self.last_check_time < self.check_interval:
            return
        
        self.last_check_time = current_time
        
        # 检查1: AMCL位姿是否收到
        if self.current_pose is None:
            self.get_logger().warn('⚠️  AMCL pose not received yet. Waiting...')
            return
        
        # 检查2: 粒子云状态
        if self.particle_count == 0:
            self.get_logger().warn('⚠️  No particles received. AMCL may not be initialized.')
        else:
            # 粒子云收敛度检查（方差越小，收敛越好）
            if self.particle_variance < 0.1:
                self.get_logger().info(
                    f'✅ Particle cloud converged: {self.particle_count} particles, '
                    f'variance={self.particle_variance:.4f}'
                )
            elif self.particle_variance < 1.0:
                self.get_logger().warn(
                    f'⚠️  Particle cloud partially converged: {self.particle_count} particles, '
                    f'variance={self.particle_variance:.4f}'
                )
            else:
                self.get_logger().warn(
                    f'❌ Particle cloud not converged: {self.particle_count} particles, '
                    f'variance={self.particle_variance:.4f}'
                )
        
        # 检查3: 位姿协方差（协方差越小，定位越准确）
        cov = self.current_pose.pose.covariance
        pos_cov = math.sqrt(cov[0] + cov[7])  # x和y的协方差
        yaw_cov = math.sqrt(cov[35])  # yaw的协方差
        
        if pos_cov < 0.1 and yaw_cov < 0.1:
            self.get_logger().info(
                f'✅ Pose covariance good: pos={pos_cov:.4f}m, yaw={math.degrees(yaw_cov):.2f}°'
            )
        elif pos_cov < 0.5 and yaw_cov < 0.5:
            self.get_logger().warn(
                f'⚠️  Pose covariance moderate: pos={pos_cov:.4f}m, yaw={math.degrees(yaw_cov):.2f}°'
            )
        else:
            self.get_logger().warn(
                f'❌ Pose covariance high: pos={pos_cov:.4f}m, yaw={math.degrees(yaw_cov):.2f}°'
            )
        
        # 检查4: 与初始位姿的差异
        if self.initial_pose is not None:
            diff = self._calculate_pose_difference(self.initial_pose, self.current_pose)
            if diff:
                if diff['distance'] < 0.1 and abs(diff['dyaw']) < 0.1:
                    self.get_logger().info(
                        f'✅ Pose stable: moved {diff["distance"]:.3f}m, '
                        f'rotated {math.degrees(diff["dyaw"]):.2f}° from initial pose'
                    )
                else:
                    self.get_logger().warn(
                        f'⚠️  Pose changed: moved {diff["distance"]:.3f}m, '
                        f'rotated {math.degrees(diff["dyaw"]):.2f}° from initial pose'
                    )
        
        # 检查5: TF变换
        tf_ok, transform, transform_odom = self._check_tf_transform()
        if tf_ok:
            self.get_logger().info(
                f'✅ TF transforms OK: map->base_footprint, odom->base_footprint'
            )
        else:
            self.get_logger().error('❌ TF transforms missing or invalid!')
        
        # 综合评估
        self._print_summary()
    
    def _print_summary(self):
        """打印定位状态摘要"""
        if self.current_pose is None:
            return
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Localization Status Summary:')
        self.get_logger().info(f'  Current pose: x={self.current_pose.pose.pose.position.x:.3f}, '
                              f'y={self.current_pose.pose.pose.position.y:.3f}')
        self.get_logger().info(f'  Particles: {self.particle_count}, Variance: {self.particle_variance:.4f}')
        
        if self.current_pose:
            cov = self.current_pose.pose.covariance
            pos_cov = math.sqrt(cov[0] + cov[7])
            yaw_cov = math.sqrt(cov[35])
            self.get_logger().info(f'  Position covariance: {pos_cov:.4f}m')
            self.get_logger().info(f'  Yaw covariance: {math.degrees(yaw_cov):.2f}°')
        
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    verifier = LocalizationVerifier()
    
    try:
        rclpy.spin(verifier)
    except KeyboardInterrupt:
        pass
    finally:
        verifier.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


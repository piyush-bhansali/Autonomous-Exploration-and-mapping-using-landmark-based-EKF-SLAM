#!/usr/bin/env python3
"""
Pose Comparison Test Node

Compares:
1. Gazebo ground truth pose (from /model/{robot}/pose Gazebo topic via custom bridge)
2. Raw odometry from DiffDrive plugin (/tb3_1/odom)
3. EKF estimated pose (/tb3_1/ekf_pose)

Outputs detailed comparison to diagnose EKF trajectory mismatch.

NOTE: To get true ground truth, you need to bridge Gazebo's /model/tb3_1/pose topic.
For now, this compares odometry (from DiffDrive) vs EKF (sensor fusion).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from collections import deque
import time


def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw angle"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


class PoseComparisonTest(Node):
    """Test node to compare ground truth vs odometry vs EKF"""

    def __init__(self):
        super().__init__('pose_comparison_test')

        # Parameters
        self.declare_parameter('robot_name', 'tb3_1')
        self.robot_name = self.get_parameter('robot_name').value

        # Storage for latest poses
        self.odometry = None
        self.ekf_pose = None

        # Statistics
        self.start_time = time.time()
        self.comparison_count = 0
        self.error_history = deque(maxlen=100)  # Last 100 samples

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )

        self.ekf_sub = self.create_subscription(
            PoseStamped,
            f'/{self.robot_name}/ekf_pose',
            self.ekf_callback,
            10
        )

        # Timer for periodic comparison
        self.create_timer(1.0, self.comparison_callback)

        self.get_logger().info(f'Pose Comparison Test started for {self.robot_name}')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info(f'  - /{self.robot_name}/odom (raw DiffDrive odometry)')
        self.get_logger().info(f'  - /{self.robot_name}/ekf_pose (EKF fused estimate)')
        self.get_logger().info('')
        self.get_logger().info('This test compares raw odometry from Gazebo DiffDrive plugin')
        self.get_logger().info('against EKF-fused estimate to verify odometry accuracy.')

    def odom_callback(self, msg):
        """Store odometry pose"""
        self.odometry = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'qx': msg.pose.pose.orientation.x,
            'qy': msg.pose.pose.orientation.y,
            'qz': msg.pose.pose.orientation.z,
            'qw': msg.pose.pose.orientation.w,
            'theta': quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ),
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'vtheta': msg.twist.twist.angular.z,
            'timestamp': self.get_clock().now()
        }

    def ekf_callback(self, msg):
        """Store EKF pose"""
        self.ekf_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'qx': msg.pose.orientation.x,
            'qy': msg.pose.orientation.y,
            'qz': msg.pose.orientation.z,
            'qw': msg.pose.orientation.w,
            'theta': quaternion_to_yaw(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ),
            'timestamp': self.get_clock().now()
        }

    def comparison_callback(self):
        """Periodic comparison of odometry vs EKF"""
        if self.odometry is None:
            self.get_logger().warn('No odometry data yet')
            return

        if self.ekf_pose is None:
            self.get_logger().warn('No EKF pose data yet')
            return

        self.comparison_count += 1
        elapsed = time.time() - self.start_time

        # EKF vs ODOMETRY difference
        error_x = self.ekf_pose['x'] - self.odometry['x']
        error_y = self.ekf_pose['y'] - self.odometry['y']
        error_pos = np.sqrt(error_x**2 + error_y**2)
        error_theta = self.ekf_pose['theta'] - self.odometry['theta']
        error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))

        # Store errors for statistics
        self.error_history.append(error_pos)

        # Compute statistics
        mean_error = np.mean(self.error_history) if len(self.error_history) > 0 else 0

        # Log detailed comparison
        self.get_logger().info('=' * 80)
        self.get_logger().info(f'ODOMETRY vs EKF COMPARISON #{self.comparison_count} (t={elapsed:.1f}s)')
        self.get_logger().info('-' * 80)

        # Odometry (from Gazebo DiffDrive)
        self.get_logger().info(
            f'ODOMETRY (DiffDrive): x={self.odometry["x"]:7.3f}, y={self.odometry["y"]:7.3f}, '
            f'θ={np.degrees(self.odometry["theta"]):7.2f}° | '
            f'v={np.sqrt(self.odometry["vx"]**2 + self.odometry["vy"]**2):5.2f} m/s'
        )

        # EKF (fused estimate from IMU + Odom + ICP)
        self.get_logger().info(
            f'EKF (Fused):          x={self.ekf_pose["x"]:7.3f}, y={self.ekf_pose["y"]:7.3f}, '
            f'θ={np.degrees(self.ekf_pose["theta"]):7.2f}°'
        )

        self.get_logger().info('-' * 80)

        # Difference
        self.get_logger().info(
            f'DIFFERENCE (EKF - Odom): Δpos={error_pos*1000:6.1f}mm, '
            f'Δx={error_x*1000:6.1f}mm, Δy={error_y*1000:6.1f}mm, '
            f'Δθ={np.degrees(error_theta):6.2f}°'
        )

        self.get_logger().info('-' * 80)

        # Statistics
        self.get_logger().info(
            f'MEAN DIFFERENCE (last 100): {mean_error*1000:.1f}mm'
        )

        # Quality assessment
        if error_pos > 3.0:  # 3m difference
            self.get_logger().error('❌ MASSIVE DIFFERENCE! Odometry is severely wrong (~10x error)')
            self.get_logger().error('    This indicates Gazebo DiffDrive plugin odometry is broken')
        elif error_pos > 1.0:  # 1m difference
            self.get_logger().warn('⚠️  LARGE DIFFERENCE! Odometry has significant systematic error')
        elif error_pos > 0.2:  # 20cm difference
            self.get_logger().warn('⚠️  Moderate difference - ICP corrections are compensating for drift')
        elif error_pos > 0.05:  # 5cm difference
            self.get_logger().info('✓  Small difference - EKF applying minor corrections')
        else:
            self.get_logger().info('✅ EXCELLENT - Odometry and EKF closely aligned')

        self.get_logger().info('=' * 80)


def main(args=None):
    rclpy.init(args=args)
    node = PoseComparisonTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

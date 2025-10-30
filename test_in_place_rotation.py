#!/usr/bin/env python3
"""
Test In-Place Rotation

Verifies that the robot rotates in-place (zero turning radius) with both
wheels moving in opposite directions.

This test:
1. Records starting position
2. Commands pure rotation (angular.z only, no linear.x)
3. Monitors position during rotation
4. Verifies robot stayed within a small radius (< 0.05m is perfect in-place)

Usage:
    python3 test_in_place_rotation.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time


class InPlaceRotationTest(Node):
    def __init__(self):
        super().__init__('in_place_rotation_test')

        self.robot_name = 'tb3_1'

        # Publisher and subscriber
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )

        # State
        self.current_position = None
        self.current_yaw = None
        self.start_position = None
        self.start_yaw = None
        self.position_history = []

        self.get_logger().info('In-Place Rotation Test initialized')

    def odom_callback(self, msg):
        """Track position from odometry"""
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        # Extract yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def run_test(self):
        """Run the in-place rotation test"""

        # Wait for odometry
        self.get_logger().info('Waiting for odometry...')
        while self.current_position is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('IN-PLACE ROTATION TEST')
        self.get_logger().info('='*60)

        # Record starting position
        self.start_position = self.current_position.copy()
        self.start_yaw = self.current_yaw

        self.get_logger().info(f'Starting position: ({self.start_position[0]:.3f}, {self.start_position[1]:.3f})')
        self.get_logger().info(f'Starting yaw: {np.degrees(self.start_yaw):.1f}°')
        self.get_logger().info('\nStarting in-place rotation in 2 seconds...')
        time.sleep(2)

        # Command pure rotation (360 degrees)
        twist = Twist()
        twist.linear.x = 0.0  # NO linear motion
        twist.angular.z = 0.5  # Pure rotation at 0.5 rad/s

        self.get_logger().info('\n🔄 Rotating 360° in place...')
        self.get_logger().info('   Command: linear.x=0.0, angular.z=0.5 rad/s')

        start_time = time.time()
        duration = 2 * np.pi / 0.5  # Time to rotate 360° at 0.5 rad/s

        rate = self.create_rate(20)  # 20 Hz monitoring

        # Rotate for the calculated duration
        while (time.time() - start_time) < duration and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.0)

            # Record position history
            if self.current_position is not None:
                self.position_history.append(self.current_position.copy())

            rate.sleep()

        # Stop
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)

        # Analyze results
        self.analyze_results()

    def analyze_results(self):
        """Analyze whether rotation was truly in-place"""

        if len(self.position_history) == 0:
            self.get_logger().error('❌ No position data recorded!')
            return

        # Calculate maximum deviation from start position
        positions = np.array(self.position_history)
        deviations = np.linalg.norm(positions - self.start_position, axis=1)

        max_deviation = deviations.max()
        mean_deviation = deviations.mean()

        # Calculate final position
        final_position = self.current_position
        final_drift = np.linalg.norm(final_position - self.start_position)

        # Calculate angle rotated
        angle_rotated = self.current_yaw - self.start_yaw
        # Normalize to [0, 2π]
        while angle_rotated < 0:
            angle_rotated += 2 * np.pi
        while angle_rotated > 2 * np.pi:
            angle_rotated -= 2 * np.pi

        # Results
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST RESULTS')
        self.get_logger().info('='*60)

        self.get_logger().info(f'\nPosition Analysis:')
        self.get_logger().info(f'  Start position:     ({self.start_position[0]:.3f}, {self.start_position[1]:.3f})')
        self.get_logger().info(f'  Final position:     ({final_position[0]:.3f}, {final_position[1]:.3f})')
        self.get_logger().info(f'  Final drift:        {final_drift*1000:.1f} mm')
        self.get_logger().info(f'  Max deviation:      {max_deviation*1000:.1f} mm')
        self.get_logger().info(f'  Mean deviation:     {mean_deviation*1000:.1f} mm')

        self.get_logger().info(f'\nRotation Analysis:')
        self.get_logger().info(f'  Start yaw:          {np.degrees(self.start_yaw):.1f}°')
        self.get_logger().info(f'  Final yaw:          {np.degrees(self.current_yaw):.1f}°')
        self.get_logger().info(f'  Angle rotated:      {np.degrees(angle_rotated):.1f}° (target: 360°)')

        # Pass/Fail criteria
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('EVALUATION')
        self.get_logger().info('='*60)

        # Perfect in-place: < 50mm deviation
        # Good: < 100mm deviation
        # Poor: > 100mm deviation

        if max_deviation < 0.05:
            self.get_logger().info('✅ EXCELLENT: Perfect in-place rotation!')
            self.get_logger().info('   Both wheels rotating in opposite directions correctly.')
        elif max_deviation < 0.10:
            self.get_logger().info('✅ GOOD: Near in-place rotation')
            self.get_logger().info('   Minor deviation acceptable for differential drive.')
        elif max_deviation < 0.20:
            self.get_logger().info('⚠️  FAIR: Some lateral movement during rotation')
            self.get_logger().info('   Robot may be pivoting around one wheel instead of center.')
        else:
            self.get_logger().info('❌ POOR: Significant lateral movement')
            self.get_logger().info('   Robot is NOT rotating in-place!')
            self.get_logger().info('   One wheel may be stationary (pivoting behavior).')

        # Rotation accuracy
        rotation_error = abs(360 - np.degrees(angle_rotated))
        if rotation_error < 10:
            self.get_logger().info(f'✅ Rotation accurate: {rotation_error:.1f}° error')
        else:
            self.get_logger().info(f'⚠️  Rotation error: {rotation_error:.1f}° (target: 360°)')

        self.get_logger().info('='*60 + '\n')

        # Interpretation guide
        self.get_logger().info('Interpretation Guide:')
        self.get_logger().info('  < 50mm deviation:  Perfect in-place rotation ✅')
        self.get_logger().info('  < 100mm deviation: Good differential drive behavior ✅')
        self.get_logger().info('  > 200mm deviation: Robot pivoting around one wheel ❌')
        self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)

    node = InPlaceRotationTest()

    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

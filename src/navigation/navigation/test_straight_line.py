#!/usr/bin/env python3
"""
Test node to move robot in a straight line for 7 meters.

This is useful for:
1. Testing odometry accuracy
2. Verifying scan-to-map ICP corrections
3. Checking map quality over distance
4. Debugging coordinate transformations

Usage:
    ros2 run navigation test_straight_line --ros-args -r __ns:=/tb3_1
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time


class StraightLineTest(Node):

    def __init__(self):
        super().__init__('straight_line_test')

        # Parameters
        self.declare_parameter('target_distance', 8.0)  # meters
        self.declare_parameter('linear_velocity', 0.2)  # m/s
        self.declare_parameter('rate_hz', 10.0)  # control loop rate

        self.target_distance = self.get_parameter('target_distance').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.rate_hz = self.get_parameter('rate_hz').value

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # State variables
        self.current_pose = None
        self.start_pose = None
        self.distance_traveled = 0.0
        self.test_started = False
        self.test_completed = False

        # Control timer
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

        self.get_logger().info('=== Straight Line Test Node Started ===')
        self.get_logger().info(f'Target distance: {self.target_distance:.2f}m')
        self.get_logger().info(f'Linear velocity: {self.linear_velocity:.2f}m/s')
        self.get_logger().info('Waiting for odometry...')

    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'timestamp': self.get_clock().now()
        }

        # Initialize start pose on first odometry message
        if self.start_pose is None:
            self.start_pose = self.current_pose.copy()
            self.test_started = True
            self.get_logger().info(f'✓ Start position: ({self.start_pose["x"]:.3f}, {self.start_pose["y"]:.3f})')
            self.get_logger().info(f'→ Moving forward {self.target_distance:.2f}m...')

    def control_loop(self):
        """Main control loop"""
        if not self.test_started or self.test_completed:
            return

        if self.current_pose is None:
            return

        # Calculate distance traveled
        dx = self.current_pose['x'] - self.start_pose['x']
        dy = self.current_pose['y'] - self.start_pose['y']
        self.distance_traveled = np.sqrt(dx**2 + dy**2)

        # Create velocity command
        twist = Twist()

        if self.distance_traveled < self.target_distance:
            # Keep moving forward
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0.0

            # Log progress every 1 meter
            if int(self.distance_traveled) != int(self.distance_traveled - 0.1):
                self.get_logger().info(
                    f'Distance: {self.distance_traveled:.2f}m / {self.target_distance:.2f}m '
                    f'({100*self.distance_traveled/self.target_distance:.1f}%)'
                )
        else:
            # Stop!
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if not self.test_completed:
                self.test_completed = True
                self.get_logger().info('=' * 50)
                self.get_logger().info('✓ TEST COMPLETED!')
                self.get_logger().info(f'  Start:    ({self.start_pose["x"]:.3f}, {self.start_pose["y"]:.3f})')
                self.get_logger().info(f'  End:      ({self.current_pose["x"]:.3f}, {self.current_pose["y"]:.3f})')
                self.get_logger().info(f'  Distance: {self.distance_traveled:.3f}m')
                self.get_logger().info(f'  Target:   {self.target_distance:.3f}m')
                self.get_logger().info(f'  Error:    {abs(self.distance_traveled - self.target_distance):.3f}m')
                self.get_logger().info('=' * 50)
                self.get_logger().info('Robot stopped. Press Ctrl+C to exit.')

        # Publish velocity command
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure robot stops
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_vel_pub.publish(twist)

        node.get_logger().info('Node shutting down...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

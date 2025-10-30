#!/usr/bin/env python3
"""
Simple Straight Line Motion Node

Makes the robot move in a straight line for testing mapping without exploration.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time


class SimpleStraightMotion(Node):
    """
    Simple node to make robot move in a straight line

    Subscribes to:
        - /robot_name/odom (Odometry): Robot odometry

    Publishes:
        - /robot_name/cmd_vel (Twist): Velocity commands
    """

    def __init__(self):
        super().__init__('simple_straight_motion')

        # Parameters
        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('linear_speed', 0.2)  # m/s
        self.declare_parameter('duration', 20.0)  # seconds to move
        self.declare_parameter('direction_x', 1.0)  # X component of direction
        self.declare_parameter('direction_y', 0.0)  # Y component of direction

        self.robot_name = self.get_parameter('robot_name').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.duration = self.get_parameter('duration').value
        self.direction_x = self.get_parameter('direction_x').value
        self.direction_y = self.get_parameter('direction_y').value

        # Normalize direction vector
        direction = np.array([self.direction_x, self.direction_y])
        direction_norm = np.linalg.norm(direction)
        if direction_norm > 0:
            direction = direction / direction_norm
            self.direction_x = direction[0]
            self.direction_y = direction[1]

        # State
        self.current_position = None
        self.current_yaw = None
        self.start_position = None
        self.start_time = None
        self.motion_started = False

        # Publishers and subscribers
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

        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info(f'Simple Straight Motion Node initialized for {self.robot_name}')
        self.get_logger().info(f'Speed: {self.linear_speed} m/s, Duration: {self.duration}s')
        self.get_logger().info(f'Direction: ({self.direction_x:.2f}, {self.direction_y:.2f})')

    def odom_callback(self, msg: Odometry):
        """Update current position from odometry"""
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Convert quaternion to yaw
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Record start position
        if self.start_position is None:
            self.start_position = self.current_position.copy()
            self.start_time = time.time()
            self.get_logger().info(f'Starting position: {self.start_position}')

    def control_loop(self):
        """Main control loop - move in straight line"""
        if self.current_position is None or self.current_yaw is None:
            return

        # Check if we should stop
        elapsed = time.time() - self.start_time
        if elapsed > self.duration:
            # Stop the robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

            if self.motion_started:
                distance_traveled = np.linalg.norm(self.current_position - self.start_position)
                self.get_logger().info(f'Motion complete! Distance traveled: {distance_traveled:.2f}m')
                self.motion_started = False

            return

        self.motion_started = True

        # Compute desired heading (angle towards direction vector)
        desired_yaw = np.arctan2(self.direction_y, self.direction_x)

        # Compute heading error
        heading_error = desired_yaw - self.current_yaw

        # Normalize to [-pi, pi]
        while heading_error > np.pi:
            heading_error -= 2 * np.pi
        while heading_error < -np.pi:
            heading_error += 2 * np.pi

        # Create velocity command
        cmd = Twist()

        # If heading error is large, rotate in place
        if abs(heading_error) > 0.3:  # ~17 degrees
            cmd.linear.x = 0.0
            cmd.angular.z = np.clip(heading_error * 1.5, -1.0, 1.0)
        else:
            # Move forward while correcting heading
            cmd.linear.x = self.linear_speed
            cmd.angular.z = np.clip(heading_error * 1.0, -0.5, 0.5)

        self.cmd_vel_pub.publish(cmd)

        # Log progress every 2 seconds
        if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.1:
            distance = np.linalg.norm(self.current_position - self.start_position)
            self.get_logger().info(
                f'Time: {elapsed:.1f}s, Distance: {distance:.2f}m, '
                f'Heading error: {np.degrees(heading_error):.1f}°'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SimpleStraightMotion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.cmd_vel_pub.publish(cmd)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

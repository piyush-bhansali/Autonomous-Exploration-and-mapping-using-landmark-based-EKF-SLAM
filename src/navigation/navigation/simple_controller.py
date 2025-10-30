#!/usr/bin/env python3
"""
Simple random walk controller - bypasses all complexity
Just makes the robot move to build the map
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time


class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        self.declare_parameter('robot_name', 'tb3_1')
        self.robot_name = self.get_parameter('robot_name').value

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )

        # Publish velocities
        self.cmd_pub = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10
        )

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)

        self.current_pos = None
        self.last_turn_time = time.time()
        self.turn_interval = 5.0  # Turn every 5 seconds

        self.get_logger().info(f'Simple controller started for {self.robot_name}')

    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

    def control_loop(self):
        if self.current_pos is None:
            return

        cmd = Twist()

        # Simple behavior: move forward, turn occasionally
        elapsed = time.time() - self.last_turn_time

        if elapsed > self.turn_interval:
            # Turn
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn

            if elapsed > self.turn_interval + 2.0:  # Turn for 2 seconds
                self.last_turn_time = time.time()
        else:
            # Move forward
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Vel: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}',
                              throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

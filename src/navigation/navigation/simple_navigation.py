#!/usr/bin/env python3
"""
Simple Autonomous Navigation Node

Clean, straightforward frontier-based exploration using:
- RRT* for path planning
- Pure Pursuit for path following
- Frontier detection at map boundary

State machine: DETECT_FRONTIERS → PLAN_PATH → EXECUTE_PATH → repeat
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
from enum import Enum

from navigation.simple_frontiers import SimpleFrontierDetector
from navigation.simple_rrt import SimpleRRTStar
from navigation.simple_controller import SimplePurePursuit


class State(Enum):
    """Navigation states"""
    WAIT_FOR_MAP = 0
    DETECT_FRONTIERS = 1
    PLAN_PATH = 2
    EXECUTE_PATH = 3
    DONE = 4


class SimpleNavigationNode(Node):
    """Main navigation node"""

    def __init__(self):
        super().__init__('simple_navigation')

        # Parameters
        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('robot_radius', 0.22)
        self.robot_name = self.get_parameter('robot_name').value
        self.robot_radius = self.get_parameter('robot_radius').value

        # State
        self.state = State.WAIT_FOR_MAP
        self.robot_pos = None
        self.robot_yaw = None
        self.map_points = None
        self.current_path = None
        self.current_waypoint_index = 0
        self.explored_positions = []  # Track visited frontiers

        # Components (created when map received)
        self.frontier_detector = SimpleFrontierDetector(self.robot_radius)
        self.path_planner = None
        self.controller = SimplePurePursuit(
            lookahead_distance=1.2,
            linear_velocity=0.2,
            max_angular_velocity=0.8
        )

        # QoS for map
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            PointCloud2,
            f'/{self.robot_name}/global_map',
            self.map_callback,
            map_qos
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.frontier_pub = self.create_publisher(PointCloud2, f'/{self.robot_name}/frontiers', 10)

        # Control timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'Simple Navigation started for {self.robot_name}')

    def map_callback(self, msg: PointCloud2):
        """Receive map from SLAM"""
        # Parse PointCloud2
        points = self._parse_pointcloud2(msg)

        if len(points) < 100:
            return  # Need minimum map size

        self.map_points = points

        # Create/update path planner
        self.path_planner = SimpleRRTStar(
            self.map_points,
            robot_radius=self.robot_radius,
            step_size=0.8,
            goal_bias=0.5
        )

        # Transition from WAIT_FOR_MAP if needed
        if self.state == State.WAIT_FOR_MAP:
            self.state = State.DETECT_FRONTIERS
            self.get_logger().info(f'Map received ({len(points)} points) - starting exploration')

    def odom_callback(self, msg: Odometry):
        """Update robot pose"""
        self.robot_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        # Extract yaw from quaternion
        qw = msg.pose.pose.orientation.w
        qz = msg.pose.pose.orientation.z
        self.robot_yaw = 2 * np.arctan2(qz, qw)

    def control_loop(self):
        """Main control loop"""
        # Check prerequisites
        if self.robot_pos is None or self.robot_yaw is None:
            return

        # State machine
        if self.state == State.WAIT_FOR_MAP:
            self._handle_wait_for_map()

        elif self.state == State.DETECT_FRONTIERS:
            self._handle_detect_frontiers()

        elif self.state == State.PLAN_PATH:
            self._handle_plan_path()

        elif self.state == State.EXECUTE_PATH:
            self._handle_execute_path()

        elif self.state == State.DONE:
            self._handle_done()

    def _handle_wait_for_map(self):
        """Waiting for map from SLAM"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _handle_detect_frontiers(self):
        """Detect frontiers and select best one"""
        if self.map_points is None:
            return

        self.get_logger().info(f'Detecting frontiers (map: {len(self.map_points)} points)')

        # Detect frontiers
        frontiers = self.frontier_detector.detect(self.map_points, self.robot_pos)

        if len(frontiers) == 0:
            self.get_logger().warn('No frontiers found - exploration complete?')
            self.state = State.DONE
            return

        # Visualize frontiers
        self._publish_frontier_viz(frontiers)

        # Filter out explored frontiers
        unexplored = [
            f for f in frontiers
            if all(np.linalg.norm(f.position - exp_pos) > 1.0
                   for exp_pos in self.explored_positions)
        ]

        if len(unexplored) == 0:
            self.get_logger().warn('All frontiers explored')
            self.state = State.DONE
            return

        # Select best frontier
        best_frontier = unexplored[0]
        self.current_goal = best_frontier.position

        self.get_logger().info(
            f'Selected frontier at [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}] '
            f'(score={best_frontier.score:.3f})'
        )

        # Transition to planning
        self.state = State.PLAN_PATH

    def _handle_plan_path(self):
        """Plan path to current goal"""
        if self.path_planner is None:
            return

        self.get_logger().info(f'Planning path to [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}]')

        # Plan with RRT*
        path = self.path_planner.plan(self.robot_pos, self.current_goal)

        if path is None:
            self.get_logger().warn('Planning FAILED - trying next frontier')
            self.explored_positions.append(self.current_goal)
            self.state = State.DETECT_FRONTIERS
            return

        self.get_logger().info(f'Path planned: {len(path)} waypoints')

        # Start path execution
        self.current_path = path
        self.current_waypoint_index = 0
        self.state = State.EXECUTE_PATH

    def _handle_execute_path(self):
        """Execute path using pure pursuit"""
        # Check if path complete
        if self.current_waypoint_index >= len(self.current_path):
            self.get_logger().info('Path complete - reached frontier')
            self.explored_positions.append(self.current_goal)
            self.state = State.DETECT_FRONTIERS
            return

        # Check if reached current waypoint
        waypoint = self.current_path[self.current_waypoint_index]
        dist_to_waypoint = np.linalg.norm(self.robot_pos - waypoint)

        if dist_to_waypoint < 0.3:  # Waypoint tolerance
            self.current_waypoint_index += 1
            return

        # Compute control
        v, w = self.controller.compute_control(
            self.robot_pos,
            self.robot_yaw,
            self.current_path,
            self.current_waypoint_index
        )

        # Publish velocity
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def _handle_done(self):
        """Exploration complete"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Exploration complete', once=True)

    def _publish_frontier_viz(self, frontiers):
        """Publish frontiers for RViz visualization"""
        if len(frontiers) == 0:
            return

        # Create colored point cloud
        points = []
        colors = [
            0xFF0000,  # Red (best)
            0xFF8000,  # Orange
            0xFFFF00,  # Yellow
            0x00FF00,  # Green
            0x0000FF,  # Blue
        ]

        for i, frontier in enumerate(frontiers[:10]):  # Top 10
            color = colors[i % len(colors)]
            x, y = frontier.position
            points.append((x, y, 0.5, color))  # Elevated for visibility

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        msg.point_step = 16
        msg.row_step = msg.point_step * len(points)

        # Pack data
        buffer = []
        for x, y, z, rgb in points:
            buffer.append(struct.pack('fffI', x, y, z, rgb))

        msg.data = b''.join(buffer)

        self.frontier_pub.publish(msg)

    def _parse_pointcloud2(self, msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 to numpy array"""
        points = []
        for i in range(msg.width):
            offset = i * msg.point_step
            x = struct.unpack_from('f', msg.data, offset + 0)[0]
            y = struct.unpack_from('f', msg.data, offset + 4)[0]
            z = struct.unpack_from('f', msg.data, offset + 8)[0]
            points.append([x, y, z])

        return np.array(points)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

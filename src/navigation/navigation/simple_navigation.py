#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import struct
from enum import Enum

from navigation.simple_frontiers import SimpleFrontierDetector
from navigation.simple_rrt import SimpleRRTStar
from navigation.smoothed_pure_pursuit import SmoothedPurePursuit


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
        self.current_goal = None
        self.all_frontiers = []  # Store all detected frontiers for continuous visualization

        # Components (created when map received)
        self.frontier_detector = SimpleFrontierDetector(self.robot_radius)
        self.path_planner = None
        self.controller = SmoothedPurePursuit(
            lookahead_distance=1.2,
            linear_velocity=0.2,
            max_angular_velocity=0.8,
            angular_smoothing_factor=0.3,
            goal_tolerance=0.3
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
        self.frontier_markers_pub = self.create_publisher(MarkerArray, f'/{self.robot_name}/frontier_markers', 10)
        self.path_pub = self.create_publisher(Path, f'/{self.robot_name}/planned_path', 10)
        self.goal_marker_pub = self.create_publisher(Marker, f'/{self.robot_name}/current_goal', 10)

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

        # ALWAYS publish visualization (every loop!)
        self._publish_all_visualizations()

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

        # Store for continuous visualization
        self.all_frontiers = frontiers

        if len(frontiers) == 0:
            self.get_logger().warn('No frontiers found - exploration complete?')
            self.state = State.DONE
            return

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

        # RE-DETECT FRONTIERS: Update frontier list as map grows during execution
        # This ensures we're always navigating to the best available frontier
        if self.map_points is not None:
            frontiers = self.frontier_detector.detect(self.map_points, self.robot_pos)
            self.all_frontiers = frontiers  # Update for visualization

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

    def _publish_all_visualizations(self):
        """Publish ALL visualizations continuously for debugging"""
        # 1. Publish frontier markers
        self._publish_frontier_markers()

        # 2. Publish current goal marker
        self._publish_goal_marker()

        # 3. Publish planned path
        self._publish_path()

    def _publish_frontier_markers(self):
        """Publish large sphere markers for all frontiers"""
        marker_array = MarkerArray()

        # Clear old markers first
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        if len(self.all_frontiers) == 0:
            self.frontier_markers_pub.publish(marker_array)
            return

        # Publish each frontier as a large sphere
        for i, frontier in enumerate(self.all_frontiers[:20]):  # Show top 20
            marker = Marker()
            marker.header.frame_id = 'odom'  # Gazebo publishes odom without namespace
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = float(frontier.position[0])
            marker.pose.position.y = float(frontier.position[1])
            marker.pose.position.z = 0.5  # Elevated for visibility
            marker.pose.orientation.w = 1.0

            # Size - LARGE spheres (0.5m diameter)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5

            # Color based on rank (best = red, others = yellow/orange)
            if i == 0:
                # BEST frontier - BRIGHT RED
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            elif i < 5:
                # Top 5 - Orange
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 0.8
            else:
                # Others - Yellow
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.6

            marker.lifetime.sec = 0  # Persist until deleted

            # Add score as text above sphere
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'frontier_scores'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = marker.pose.position.x
            text_marker.pose.position.y = marker.pose.position.y
            text_marker.pose.position.z = 1.0  # Above sphere
            text_marker.scale.z = 0.3  # Text height
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f'#{i+1}: {frontier.score:.2f}'

            marker_array.markers.append(marker)
            marker_array.markers.append(text_marker)

        self.frontier_markers_pub.publish(marker_array)

    def _publish_goal_marker(self):
        """Publish BRIGHT GREEN marker for current target goal"""
        if self.current_goal is None:
            return

        marker = Marker()
        marker.header.frame_id = 'odom'  # Gazebo publishes odom without namespace
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'current_goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = float(self.current_goal[0])
        marker.pose.position.y = float(self.current_goal[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # Size - VERY LARGE cylinder (1m diameter, 2m tall)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 2.0

        # BRIGHT GREEN - unmissable!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        marker.lifetime.sec = 0

        self.goal_marker_pub.publish(marker)

    def _publish_path(self):
        """Publish planned path as line strip"""
        if self.current_path is None or len(self.current_path) == 0:
            return

        path_msg = Path()
        path_msg.header.frame_id = 'odom'  # Gazebo publishes odom without namespace
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for waypoint in self.current_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(waypoint[0])
            pose.pose.position.y = float(waypoint[1])
            pose.pose.position.z = 0.2
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

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

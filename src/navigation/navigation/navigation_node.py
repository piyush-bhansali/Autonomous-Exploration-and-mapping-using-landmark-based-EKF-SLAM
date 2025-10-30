#!/usr/bin/env python3
"""
ROS2 Navigation Node for Frontier Exploration

Integrates frontier detection, RRT* planning, and robot control.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import time
import logging
import sys

from navigation.frontier_explorer import FrontierExplorer, ExplorationState
from navigation.frontier_detector import Frontier

# Configure Python logging to output to stdout
logging.basicConfig(
    level=logging.INFO,
    format='[%(name)s] [%(levelname)s]: %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)]
)


class NavigationNode(Node):
    """
    ROS2 node for autonomous frontier exploration

    Subscribes to:
        - /robot_name/global_map (PointCloud2): Global point cloud map
        - /robot_name/odom (PoseStamped): Robot odometry

    Publishes:
        - /robot_name/cmd_vel (Twist): Velocity commands
        - /robot_name/exploration_status (String): Exploration status
        - /robot_name/frontiers (MarkerArray): Frontier visualization
        - /robot_name/planned_path (MarkerArray): Path visualization
    """

    def __init__(self):
        super().__init__('navigation_node')

        # Parameters
        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('max_exploration_time', 3600.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_exploration_time = self.get_parameter('max_exploration_time').value

        # Initialize explorer
        self.explorer = FrontierExplorer(
            robot_radius=self.robot_radius,
            max_exploration_time=self.max_exploration_time
        )

        # State
        self.current_position = None
        self.current_heading = 0.0  # Current robot heading (yaw) in radians
        self.current_map = None
        self.last_map_update_time = 0.0
        self.map_update_interval = 2.0  # Update map every 2 seconds

        # Subscribers
        self.map_sub = self.create_subscription(
            PointCloud2,
            f'/{self.robot_name}/global_map',
            self.map_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            f'/{self.robot_name}/exploration_status',
            10
        )

        self.frontiers_pub = self.create_publisher(
            MarkerArray,
            f'/{self.robot_name}/frontiers',
            10
        )

        self.path_pub = self.create_publisher(
            MarkerArray,
            f'/{self.robot_name}/planned_path',
            10
        )

        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop
        )

        self.get_logger().info(f'Navigation node started for {self.robot_name}')

    def map_callback(self, msg: PointCloud2):
        """Receive and process global map updates"""
        try:
            # Throttle map updates to avoid excessive reprocessing
            current_time = time.time()
            if current_time - self.last_map_update_time < self.map_update_interval:
                return  # Skip this update

            # Convert PointCloud2 to numpy array
            points_list = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points_list.append([point[0], point[1], point[2]])

            if len(points_list) < 10:
                return

            points = np.array(points_list)

            # Create Open3D point cloud
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            self.current_map = pcd
            self.last_map_update_time = current_time

            # Update explorer map
            self.explorer.update_map(pcd)

            # Validate collision checker was created
            if self.explorer.collision_checker is None:
                self.get_logger().error('CRITICAL: Collision checker not initialized after map update!')
            else:
                self.get_logger().info(f'Map updated: {len(points)} points, Collision checker: OK', throttle_duration_sec=5.0)

        except Exception as e:
            self.get_logger().error(f'Error processing map: {e}')

    def odom_callback(self, msg: Odometry):
        """Receive robot odometry"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_position = np.array([x, y])

        # Extract current heading from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # Convert quaternion to yaw (heading)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.current_heading = np.arctan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """Main control loop - executed at control frequency"""
        if self.current_position is None:
            self.get_logger().warn('Waiting for odometry...', throttle_duration_sec=2.0)
            return

        if self.current_map is None:
            self.get_logger().warn('Waiting for map...', throttle_duration_sec=2.0)
            return

        # Execute exploration step
        self.get_logger().info(f'Position: {self.current_position}, Map points: {len(self.current_map.points)}', throttle_duration_sec=10.0)

        # Log explorer state (reduced frequency)
        status = self.explorer.get_status()
        self.get_logger().info(f'Explorer state: {status.state.name}', throttle_duration_sec=5.0)

        exploration_active, target_velocity = self.explorer.step(self.current_position)

        # Additional debug info (only for important events)
        if target_velocity is None and exploration_active:
            status = self.explorer.get_status()
            self.get_logger().warn(f'No velocity command! State: {status.state.name}, Frontier: {status.current_frontier is not None}, Path: {status.current_path is not None}', throttle_duration_sec=5.0)

        # Publish velocity command
        cmd = Twist()
        if target_velocity is not None:
            # Convert 2D velocity (vx, vy) to differential drive (linear.x, angular.z)
            vx = float(target_velocity[0])
            vy = float(target_velocity[1])

            # Compute desired heading from velocity vector
            desired_heading = np.arctan2(vy, vx)

            # Calculate heading error (angle difference)
            heading_error = desired_heading - self.current_heading

            # Normalize heading error to [-pi, pi]
            while heading_error > np.pi:
                heading_error -= 2.0 * np.pi
            while heading_error < -np.pi:
                heading_error += 2.0 * np.pi

            # Compute desired speed
            desired_speed = np.sqrt(vx**2 + vy**2)

            # Differential drive control logic:
            # - Only stop for very large errors (nearly backwards)
            # - Otherwise keep moving while turning for smoother navigation
            very_large_error = 1.5  # radians (~85 degrees)

            if abs(heading_error) > very_large_error:
                # Very large heading error (nearly backwards): stop and turn
                cmd.linear.x = 0.0
                cmd.angular.z = np.clip(heading_error * 1.5, -2.0, 2.0)
            else:
                # Keep moving forward while correcting heading
                # Scale speed based on heading error (slow down for large errors)
                speed_scale = 1.0 - abs(heading_error) / very_large_error
                speed_scale = max(speed_scale, 0.3)  # Maintain at least 30% speed

                cmd.linear.x = min(0.22, desired_speed * speed_scale)  # Max 0.22 m/s for TurtleBot3
                cmd.angular.z = np.clip(heading_error * 2.5, -1.5, 1.5)  # Smooth correction
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

        # Publish status
        status = self.explorer.get_status()
        status_msg = String()
        status_msg.data = status.state.name
        self.status_pub.publish(status_msg)

        # Visualizations (at lower rate)
        if int(time.time() * 2) % 5 == 0:  # Every 2.5 seconds
            self.publish_visualizations()

        # Log statistics
        if not exploration_active:
            stats = self.explorer.get_statistics()
            self.get_logger().info(f'Exploration finished: {stats}')

    def publish_visualizations(self):
        """Publish frontier and path visualizations"""
        status = self.explorer.get_status()

        # Publish frontiers
        if status.current_frontier is not None:
            self.publish_frontier_markers([status.current_frontier])

        # Publish path
        if status.current_path is not None:
            self.publish_path_markers(status.current_path, status.path_index)

    def publish_frontier_markers(self, frontiers: list):
        """Publish frontier visualization markers"""
        marker_array = MarkerArray()

        for i, frontier in enumerate(frontiers):
            # Frontier centroid marker
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(frontier.centroid[0])
            marker.pose.position.y = float(frontier.centroid[1])
            marker.pose.position.z = 0.2

            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

            # Frontier text label
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'frontier_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = float(frontier.centroid[0])
            text_marker.pose.position.y = float(frontier.centroid[1])
            text_marker.pose.position.z = 0.5

            text_marker.text = f'F{i}\nScore: {frontier.score:.2f}'
            text_marker.scale.z = 0.2

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            marker_array.markers.append(text_marker)

        self.frontiers_pub.publish(marker_array)

    def publish_path_markers(self, path: list, current_index: int):
        """Publish path visualization markers"""
        marker_array = MarkerArray()

        # Path line
        line_marker = Marker()
        line_marker.header.frame_id = 'map'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        line_marker.scale.x = 0.05

        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8

        for waypoint in path:
            p = Point()
            p.x = float(waypoint[0])
            p.y = float(waypoint[1])
            p.z = 0.05
            line_marker.points.append(p)

        marker_array.markers.append(line_marker)

        # Current waypoint marker
        if current_index < len(path):
            waypoint_marker = Marker()
            waypoint_marker.header = line_marker.header
            waypoint_marker.ns = 'current_waypoint'
            waypoint_marker.id = 1
            waypoint_marker.type = Marker.SPHERE
            waypoint_marker.action = Marker.ADD

            waypoint_marker.pose.position.x = float(path[current_index][0])
            waypoint_marker.pose.position.y = float(path[current_index][1])
            waypoint_marker.pose.position.z = 0.1

            waypoint_marker.scale.x = 0.2
            waypoint_marker.scale.y = 0.2
            waypoint_marker.scale.z = 0.2

            waypoint_marker.color.r = 1.0
            waypoint_marker.color.g = 1.0
            waypoint_marker.color.b = 0.0
            waypoint_marker.color.a = 1.0

            marker_array.markers.append(waypoint_marker)

        self.path_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

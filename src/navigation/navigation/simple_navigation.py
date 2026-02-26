#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException
import numpy as np
from enum import Enum
import threading
from scipy.spatial import KDTree

from navigation.convex_frontier_detector import ConvexFrontierDetector
from map_generation.transform_utils import quaternion_to_yaw
from navigation.rrt_star import RRTStar
from navigation.pure_pursuit_controller import PurePursuit
from autonomous_exploration.qos_profiles import (
    MAP_QOS,
    SCAN_QOS,
    CMD_VEL_QOS,
    PATH_QOS,
    VISUALIZATION_QOS
)
from navigation import navigation_utils as nav_utils


class State(Enum):
    """Navigation states"""
    WAIT_FOR_MAP = 0
    DETECT_FRONTIERS = 1
    PLAN_PATH = 2
    EXECUTE_PATH = 3
    DONE = 4


class SimpleNavigationNode(Node):
    
    def __init__(self):
        super().__init__('simple_navigation')

        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('enable_reactive_avoidance', True)
        self.declare_parameter('scan_emergency_distance', 0.4)
        self.declare_parameter('scan_angular_range', 60.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.robot_radius = 0.22
        self.enable_reactive_avoidance =                self.get_parameter('enable_reactive_avoidance').value
        self.scan_emergency_distance = self.get_parameter('scan_emergency_distance').value
        self.scan_angular_range = np.radians(self.get_parameter('scan_angular_range').value)
        self.path_deviation_threshold = 1.0

        self.state = State.WAIT_FOR_MAP
        self.robot_pos = None
        self.robot_yaw = None
        self.map_points = None
        self.map_header = None  # Store map header for frontier marker timestamps
        self.current_path = None
        self.current_waypoint_index = 0

        self.current_goal = None
        self.all_frontiers = []
        self.no_frontiers_count = 0

        self.stuck_detection_enabled = True
        self.stuck_check_window = 5.0
        self.stuck_distance_threshold = 0.1
        self.last_stuck_check_time = None
        self.last_stuck_check_position = None
        self.stuck_timeout = 30.0
        self.execute_path_start_time = None

        self.map_lock = threading.Lock()

        self._obstacle_kdtree = None
        self._obstacle_kdtree_hash = None

        self.scan_data = None
        self.scan_lock = threading.Lock()

        self.min_frontier_distance = 1.0
        self.frontier_detector = ConvexFrontierDetector(
            robot_radius=self.robot_radius
        )
        self.controller = PurePursuit(
            max_linear_velocity=0.20,
            max_angular_velocity=1.0
        )

        # TF2 for localization in map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.map_sub = self.create_subscription(
            PointCloud2,
            f'/{self.robot_name}/global_map',
            self.map_callback,
            MAP_QOS  # Using centralized QoS profile
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            SCAN_QOS  # Using centralized QoS profile
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', CMD_VEL_QOS)
        self.frontier_markers_pub = self.create_publisher(MarkerArray, f'/{self.robot_name}/frontier_markers', VISUALIZATION_QOS)
        self.path_pub = self.create_publisher(Path, f'/{self.robot_name}/planned_path', PATH_QOS)
        self.hull_markers_pub = self.create_publisher(MarkerArray, f'/{self.robot_name}/hull_boundary', VISUALIZATION_QOS)

        # Control timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'Simple Navigation started for {self.robot_name}')

    def map_callback(self, msg: PointCloud2):

        points = nav_utils.parse_pointcloud2(msg)

        if len(points) < 100:
            return

        with self.map_lock:
            self.map_points = points
            self.map_header = msg.header  

            map_hash = hash(self.map_points.tobytes())
            if self._obstacle_kdtree_hash != map_hash:
                points_2d = self.map_points[:, :2]
                self._obstacle_kdtree = KDTree(points_2d)
                self._obstacle_kdtree_hash = map_hash

        if self.robot_pos is not None and self.robot_yaw is not None:
            self.all_frontiers = self.frontier_detector.detect(
                self.map_points,
                self.robot_pos,
                self.robot_yaw,
                obstacle_kdtree=self._obstacle_kdtree
            )

            if len(self.all_frontiers) > 0:
                self.no_frontiers_count = 0
            else:
                self.no_frontiers_count += 1

        if self.state == State.WAIT_FOR_MAP:
            self.state = State.DETECT_FRONTIERS
            self.get_logger().info(f'Map received ({len(points)} points) - starting exploration')

    def _update_robot_pose_from_tf(self):
       
        try:
            
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',                              # Target frame (global)
                f'{self.robot_name}/base_footprint', # Source frame (robot)
                now,                                 # Get latest
                timeout=Duration(seconds=0.1)        # Wait up to 100ms
            )

            self.robot_pos = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y
            ])

            quat = transform.transform.rotation
            self.robot_yaw = quaternion_to_yaw(
                quat.x, quat.y, quat.z, quat.w
            )

            return True

        except (LookupException, ExtrapolationException, ConnectivityException) as e:
            
            self.get_logger().warn(
                f'TF lookup failed (map → {self.robot_name}/base_footprint): {e}',
                throttle_duration_sec=5.0
            )
            return False

    def scan_callback(self, msg: LaserScan):
        
        with self.scan_lock:
            self.scan_data = msg

    def control_loop(self):
        
        if not self._update_robot_pose_from_tf():
            
            return

        self._publish_all_visualizations()

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
       
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _handle_detect_frontiers(self):
        
        frontiers = self._get_frontiers()

        if len(frontiers) == 0:
            
            if self.no_frontiers_count >= 3:
                self.get_logger().warn(f'[STATE: DETECT_FRONTIERS] No frontiers found for {self.no_frontiers_count} consecutive map updates - exploration complete')
                self.state = State.DONE
                return
            else:
                return

        distant_frontiers = []
        for f in frontiers:
            dist = np.linalg.norm(f.position - self.robot_pos)
            if dist >= self.min_frontier_distance:
                distant_frontiers.append(f)

        if len(distant_frontiers) == 0:
            self.get_logger().warn(
                f'[STATE: DETECT_FRONTIERS] All {len(frontiers)} frontiers < {self.min_frontier_distance}m. '
                f'Using closest available frontier.'
            )
            distant_frontiers = frontiers

        best_frontier = distant_frontiers[0]
        self.current_goal = best_frontier.position
        
        self.state = State.PLAN_PATH

    def _handle_plan_path(self):
        
        if self.current_goal is None:
            self.get_logger().error('[STATE: PLAN_PATH] No current goal set!')
            self.state = State.DETECT_FRONTIERS
            return

        path_planner = RRTStar(
            self.map_points,
            robot_radius=self.robot_radius,
            obstacle_kdtree=self._obstacle_kdtree 
        )

        path = path_planner.plan(self.robot_pos, self.current_goal, logger=self.get_logger())

        if path is None:
            self.get_logger().warn('[STATE: PLAN_PATH] Planning FAILED - will re-detect frontiers')

            self.current_goal = None
            self.state = State.DETECT_FRONTIERS
            return

        self.get_logger().info(f'[STATE: PLAN_PATH] Success! {len(path)} waypoints, distance: {nav_utils.calculate_path_length(path):.2f}m')

        # Start path execution
        self.current_path = path

        self.current_waypoint_index = nav_utils.find_nearest_waypoint_index(path, self.robot_pos)

        self.state = State.EXECUTE_PATH
        self.execute_path_start_time = self.get_clock().now()
        self.last_stuck_check_time = self.get_clock().now()
        self.last_stuck_check_position = self.robot_pos.copy()

    def _handle_execute_path(self):
        
        if self.stuck_detection_enabled and self._is_stuck():
            self.get_logger().warn('[STATE: EXECUTE_PATH] Robot is STUCK! Aborting current path and replanning...')

            cmd = Twist()
            self.cmd_pub.publish(cmd)

            self.current_path = None
            self.current_goal = None

            self.state = State.DETECT_FRONTIERS
            return

        is_deviated, deviation_distance = nav_utils.check_path_deviation(
            self.robot_pos,
            self.current_path,
            self.current_waypoint_index,
            self.path_deviation_threshold
        )
        if is_deviated:
            self.get_logger().warn(
                f'[STATE: EXECUTE_PATH] PATH DEVIATION DETECTED! Distance from path: {deviation_distance:.2f}m '
                f'(threshold: {self.path_deviation_threshold:.2f}m). Replanning...'
            )

            cmd = Twist()
            self.cmd_pub.publish(cmd)

            self.current_path = None
            self.current_waypoint_index = 0

            self.state = State.PLAN_PATH
            return

        if self.current_waypoint_index >= len(self.current_path):
            self.get_logger().info(f'[STATE: EXECUTE_PATH] ✓ Path complete! Reached frontier at [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}]')

            cmd = Twist()
            self.cmd_pub.publish(cmd)

            self.current_path = None
            self.current_goal = None

            self.state = State.DETECT_FRONTIERS
            return

        frontiers = self._get_frontiers() 

        if len(frontiers) > 0:
            
            best_frontier = frontiers[0]  
            
            goal_distance = np.linalg.norm(best_frontier.position - self.current_goal)

            if goal_distance > 0.5: 
                dist_to_old_goal = np.linalg.norm(self.current_goal - self.robot_pos)
                dist_to_new_goal = np.linalg.norm(best_frontier.position - self.robot_pos)

                self.get_logger().warn(f'[STATE: EXECUTE_PATH] Goal changed - replanning!')
                self.get_logger().warn(f'  Total frontiers: {len(frontiers)}')

                cmd = Twist()
                self.cmd_pub.publish(cmd)

                self.current_path = None
                self.current_waypoint_index = 0

                self.current_goal = best_frontier.position.copy()

                self.state = State.PLAN_PATH
                return

        waypoint = self.current_path[self.current_waypoint_index]
        dist_to_waypoint = np.linalg.norm(self.robot_pos - waypoint)

        if dist_to_waypoint < 0.25:  
            self.current_waypoint_index += 1

            if self.current_waypoint_index > 0 and self.current_waypoint_index % 10 == 0:
               
                self.current_path = self.current_path[self.current_waypoint_index:]
                
                self.current_waypoint_index = 0

            return

        v, w = self.controller.compute_control(
            self.robot_pos,
            self.robot_yaw,
            self.current_path,
            self.current_waypoint_index
        )

        with self.scan_lock:
            scan = self.scan_data

        if self.enable_reactive_avoidance:
            obstacle_detected, min_distance, avoidance_direction = nav_utils.check_scan_for_obstacles(
                scan,
                self.scan_emergency_distance,
                self.scan_angular_range
            )
        else:
            obstacle_detected, min_distance, avoidance_direction = False, float('inf'), 0.0

   
        if obstacle_detected and min_distance < self.scan_emergency_distance:
            self.get_logger().warn(
                f'[OBSTACLE DETECTED] Obstacle at {min_distance:.2f}m < {self.scan_emergency_distance:.2f}m threshold! '
                f'Slowing to 20% speed and replanning...'
            )

            v *= 0.2
            w *= 0.2

            self.current_path = None
            self.current_waypoint_index = 0

            self.state = State.PLAN_PATH

            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)
            return

        if len(self.current_path) > 30 and self.current_waypoint_index % 20 == 0:
            progress = (self.current_waypoint_index / len(self.current_path)) * 100
            dist_to_goal = np.linalg.norm(self.robot_pos - self.current_path[-1])
            self.get_logger().info(
                f'[STATE: EXECUTE_PATH] Waypoint {self.current_waypoint_index}/{len(self.current_path)} '
                f'({progress:.1f}%), dist to goal: {dist_to_goal:.2f}m'
            )

        # Publish velocity
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def _handle_done(self):
       
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Exploration complete', once=True)

    def _get_frontiers(self) -> list:
        
        return self.all_frontiers

    def _is_stuck(self) -> bool:
        
        current_time = self.get_clock().now()

        # Global timeout check
        if self.execute_path_start_time is not None:
            time_in_state = (current_time - self.execute_path_start_time).nanoseconds / 1e9
            if time_in_state > self.stuck_timeout:
                self.get_logger().warn(
                    f'[STUCK DETECTION] Global timeout: {time_in_state:.1f}s > {self.stuck_timeout}s in EXECUTE_PATH'
                )
                return True

        # Initialize check
        if self.last_stuck_check_time is None or self.last_stuck_check_position is None:
            self.last_stuck_check_time = current_time
            self.last_stuck_check_position = self.robot_pos.copy()
            return False

        # Periodic movement check
        time_since_check = (current_time - self.last_stuck_check_time).nanoseconds / 1e9

        is_stuck, distance_moved = nav_utils.check_if_stuck(
            self.robot_pos,
            self.last_stuck_check_position,
            time_since_check,
            self.stuck_check_window,
            self.stuck_distance_threshold
        )

        if is_stuck:
            self.get_logger().warn(
                f'[STUCK DETECTION] Insufficient movement: {distance_moved:.3f}m < {self.stuck_distance_threshold}m '
                f'in {time_since_check:.1f}s'
            )
            return True

        # Update check window if enough time has passed
        if time_since_check >= self.stuck_check_window:
            self.last_stuck_check_time = current_time
            self.last_stuck_check_position = self.robot_pos.copy()

        return False

    def _publish_all_visualizations(self):
        
        self._publish_frontier_markers()

        self._publish_path()

        self._publish_hull_boundary()

    def _publish_frontier_markers(self):
        
        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        if len(self.all_frontiers) == 0:
            self.frontier_markers_pub.publish(marker_array)
            return

        if self.map_header is not None:
            header_frame = self.map_header.frame_id
            header_stamp = self.map_header.stamp
        else:
            header_frame = f'{self.robot_name}/odom'
            header_stamp = self.get_clock().now().to_msg()

        for i, frontier in enumerate(self.all_frontiers[:20]):  # Show top 20
            marker = Marker()
            marker.header.frame_id = header_frame
            marker.header.stamp = header_stamp
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(frontier.position[0])
            marker.pose.position.y = float(frontier.position[1])
            marker.pose.position.z = 0.5  # Elevated for visibility
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5

            if i == 0:
                
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            elif i < 5:
                
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 0.8
            else:
                
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.6

            marker.lifetime.sec = 0  

            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'frontier_scores'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = marker.pose.position.x
            text_marker.pose.position.y = marker.pose.position.y
            text_marker.pose.position.z = 1.0  
            text_marker.scale.z = 0.3  
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f'#{i+1}: {frontier.score:.2f}'

            marker_array.markers.append(marker)
            marker_array.markers.append(text_marker)

        self.frontier_markers_pub.publish(marker_array)

    def _publish_path(self):

        if self.current_path is None or len(self.current_path) == 0:
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'
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

    def _publish_hull_boundary(self):
        
        hull_data = self.frontier_detector.get_hull_visualization_data()

        if not hull_data['has_data']:
            return

        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        if self.map_header is not None:
            header_frame = self.map_header.frame_id
            header_stamp = self.map_header.stamp
        else:
            header_frame = f'{self.robot_name}/odom'
            header_stamp = self.get_clock().now().to_msg()

        if 'offset_boundary' in hull_data and len(hull_data['offset_boundary']) > 0:
            offset_marker = Marker()
            offset_marker.header.frame_id = header_frame
            offset_marker.header.stamp = header_stamp
            offset_marker.ns = 'offset_hull'
            offset_marker.id = 1
            offset_marker.type = Marker.LINE_STRIP
            offset_marker.action = Marker.ADD
            offset_marker.scale.x = 0.07 
            offset_marker.color.r = 0.0
            offset_marker.color.g = 0.5
            offset_marker.color.b = 1.0
            offset_marker.color.a = 1.0
            offset_marker.lifetime.sec = 0

            for point in hull_data['offset_boundary']:
                p = Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = 0.0
                offset_marker.points.append(p)

            marker_array.markers.append(offset_marker)

        self.hull_markers_pub.publish(marker_array)


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

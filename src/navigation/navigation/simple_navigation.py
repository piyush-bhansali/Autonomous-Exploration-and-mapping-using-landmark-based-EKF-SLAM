#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from enum import Enum
import threading

from navigation.simple_frontiers import SimpleFrontierDetector
from map_generation.utils import quaternion_to_yaw
from navigation.rrt_star import RRTStar
from navigation.smoothed_pure_pursuit import SmoothedPurePursuit
from multi_robot_mapping.qos_profiles import MAP_QOS, SCAN_QOS
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
        self.declare_parameter('scan_danger_distance', 0.5)
        self.declare_parameter('scan_emergency_distance', 0.3)
        self.declare_parameter('scan_angular_range', 60.0)
        self.declare_parameter('path_deviation_check_interval', 4.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.robot_radius = 0.22  
        self.enable_reactive_avoidance = self.get_parameter('enable_reactive_avoidance').value
        self.scan_danger_distance = self.get_parameter('scan_danger_distance').value
        self.scan_emergency_distance = self.get_parameter('scan_emergency_distance').value
        self.scan_angular_range = np.radians(self.get_parameter('scan_angular_range').value)
        self.path_deviation_threshold = 0.5  
        self.path_deviation_check_interval = self.get_parameter('path_deviation_check_interval').value

        self.state = State.WAIT_FOR_MAP
        self.previous_state = None 
        self.robot_pos = None
        self.robot_yaw = None
        self.map_points = None
        self.current_path = None
        self.current_waypoint_index = 0

        self.current_goal = None
        self.all_frontiers = []  

        self.stuck_detection_enabled = True
        self.stuck_check_window = 5.0  
        self.stuck_distance_threshold = 0.1  
        self.last_stuck_check_time = None
        self.last_stuck_check_position = None
        self.stuck_timeout = 30.0  
        self.execute_path_start_time = None

        self.map_lock = threading.Lock()

        self.scan_data = None
        self.scan_lock = threading.Lock()
        self.last_obstacle_warning_time = None
        self.obstacle_warning_cooldown = 2.0

        self.last_deviation_check_time = None
        self.replan_count = 0

        # Dynamic replanning thresholds
        self.replan_score_threshold = 0.50  
        self.replan_distance_threshold = 4.0  
        self.min_frontier_distance = 2.0 
        self.frontier_detector = SimpleFrontierDetector(self.robot_radius)
        self.controller = SmoothedPurePursuit(
            lookahead_distance=0.8,  # Reduced from 1.2m for tighter turns
            max_linear_velocity=0.18,
            min_linear_velocity=0.08,
            max_angular_velocity=0.8,  # Increased from 0.5 for sharper turns
            angular_smoothing_factor=0.4,  # Reduced from 0.6 for more responsive turning
            goal_tolerance=0.3,
            velocity_gain=0.7
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            PointCloud2,
            f'/{self.robot_name}/global_map',
            self.map_callback,
            MAP_QOS  # Using centralized QoS profile
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )

        # Scan subscriber for reactive obstacle avoidance
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            SCAN_QOS  # Using centralized QoS profile
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.frontier_markers_pub = self.create_publisher(MarkerArray, f'/{self.robot_name}/frontier_markers', 10)
        self.path_pub = self.create_publisher(Path, f'/{self.robot_name}/planned_path', 10)

        # Control timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'Simple Navigation started for {self.robot_name}')

    def map_callback(self, msg: PointCloud2):
        
        points = nav_utils.parse_pointcloud2(msg)

        if len(points) < 100:
            return

        with self.map_lock:
            self.map_points = points

        if self.robot_pos is not None and self.robot_yaw is not None:
            self.all_frontiers = self.frontier_detector.detect(
                self.map_points,
                self.robot_pos,
                self.robot_yaw
            )

        if self.state == State.WAIT_FOR_MAP:
            self.state = State.DETECT_FRONTIERS
            self.get_logger().info(f'Map received ({len(points)} points) - starting exploration')

    def odom_callback(self, msg: Odometry):
        """Update robot pose"""
        self.robot_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_yaw = quaternion_to_yaw(qx, qy, qz, qw)

    def scan_callback(self, msg: LaserScan):
        """Store laser scan data for reactive obstacle avoidance."""
        with self.scan_lock:
            self.scan_data = msg

    def control_loop(self):
        """Main control loop running at 10 Hz - executes state machine."""
        if self.robot_pos is None or self.robot_yaw is None:
            return

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
       
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _handle_detect_frontiers(self):
        """Select best frontier from already-detected frontiers."""
        # Frontiers are already detected in map_callback (event-driven)
        frontiers = self._get_frontiers()

        if len(frontiers) == 0:
            self.get_logger().warn('[STATE: DETECT_FRONTIERS] No frontiers found - exploration complete')
            self.state = State.DONE
            return

        self.get_logger().info(f'[STATE: DETECT_FRONTIERS] Found {len(frontiers)} frontiers')

        distant_frontiers = []
        for f in frontiers:
            dist = np.linalg.norm(f.position - self.robot_pos)
            if dist >= self.min_frontier_distance:
                distant_frontiers.append(f)
            else:
                self.get_logger().debug(
                    f'  Frontier at [{f.position[0]:.2f}, {f.position[1]:.2f}] too close '
                    f'(dist={dist:.2f}m < min={self.min_frontier_distance}m)'
                )

        if len(distant_frontiers) == 0:
            self.get_logger().warn(
                f'[STATE: DETECT_FRONTIERS] All {len(frontiers)} frontiers < {self.min_frontier_distance}m. '
                f'Using closest available frontier.'
            )
            distant_frontiers = frontiers

        self.get_logger().info(
            f'[STATE: DETECT_FRONTIERS] {len(distant_frontiers)} frontiers meet distance requirement '
            f'(≥{self.min_frontier_distance}m)'
        )

        best_frontier = distant_frontiers[0]
        self.current_goal = best_frontier.position
        dist_to_selected = np.linalg.norm(self.current_goal - self.robot_pos)

        self.get_logger().info(
            f'[STATE: DETECT_FRONTIERS] Selected frontier at [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}] '
            f'(dist={dist_to_selected:.2f}m, score={best_frontier.score:.3f})'
        )

        self.previous_state = self.state
        self.state = State.PLAN_PATH
        self.get_logger().info('[STATE: DETECT_FRONTIERS → PLAN_PATH]')

    def _handle_plan_path(self):
        """Plan path to current goal"""
        if self.current_goal is None:
            self.get_logger().error('[STATE: PLAN_PATH] No current goal set!')
            self.state = State.DETECT_FRONTIERS
            return

        self.get_logger().info(f'[STATE: PLAN_PATH] Planning from [{self.robot_pos[0]:.2f}, {self.robot_pos[1]:.2f}] to [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}]')

        with self.scan_lock:
            scan = self.scan_data

        combined_obstacles = nav_utils.get_combined_obstacle_map(
            self.map_points,
            scan,
            self.robot_pos,
            self.robot_yaw
        )

        path_planner_with_fresh_obstacles = RRTStar(
            combined_obstacles,
            robot_radius=self.robot_radius,
            step_size=0.2,
            goal_bias=0.5,
            max_iterations=1500
        )

        path = path_planner_with_fresh_obstacles.plan(self.robot_pos, self.current_goal)

        num_static = len(self.map_points) if self.map_points is not None else 0
        num_scan = len(combined_obstacles) - num_static
        self.get_logger().info(f'  RRT* using {len(combined_obstacles)} obstacles ({num_static} static + {num_scan} from scan)')

        if path is None:
            self.get_logger().warn('[STATE: PLAN_PATH] Planning FAILED - will re-detect frontiers')

            self.current_goal = None
            self.state = State.DETECT_FRONTIERS
            return

        self.get_logger().info(f'[STATE: PLAN_PATH] Success! {len(path)} waypoints, distance: {nav_utils.calculate_path_length(path):.2f}m')

        # Start path execution
        self.current_path = path

        self.current_waypoint_index = nav_utils.find_nearest_waypoint_index(path, self.robot_pos)

        if self.previous_state == State.EXECUTE_PATH:
            # Replanning during execution
            self.get_logger().info(
                f'[STATE: PLAN_PATH] Replanning: Starting from waypoint {self.current_waypoint_index}/{len(path)} '
                f'(nearest to current robot position)'
            )
        else:
            # New path (but robot may have moved during planning)
            self.get_logger().info(
                f'[STATE: PLAN_PATH] New path: Starting from waypoint {self.current_waypoint_index}/{len(path)} '
                f'(nearest to current robot position, compensates for planning latency)'
            )

        self.state = State.EXECUTE_PATH

        self.execute_path_start_time = self.get_clock().now()
        self.last_stuck_check_time = self.get_clock().now()
        self.last_stuck_check_position = self.robot_pos.copy()

        self.get_logger().info('[STATE: PLAN_PATH → EXECUTE_PATH] Starting path execution')

    def _handle_execute_path(self):
        
        if self.stuck_detection_enabled and self._is_stuck():
            self.get_logger().warn('[STATE: EXECUTE_PATH] Robot is STUCK! Aborting current path and replanning...')

            # Stop the robot
            cmd = Twist()
            self.cmd_pub.publish(cmd)

            # Clear current path and goal
            self.current_path = None
            self.current_goal = None

            # Transition back to frontier detection
            self.state = State.DETECT_FRONTIERS
            self.get_logger().info('[STATE: EXECUTE_PATH → DETECT_FRONTIERS] Recovering from stuck condition...')
            return

        # PATH DEVIATION CHECK: Replan if robot deviated too far from path
        is_deviated, deviation_distance = nav_utils.check_path_deviation(
            self.robot_pos,
            self.current_path,
            self.current_waypoint_index,
            self.path_deviation_threshold
        )
        if is_deviated:
            self.replan_count += 1
            self.get_logger().warn(
                f'[STATE: EXECUTE_PATH] PATH DEVIATION DETECTED! Distance from path: {deviation_distance:.2f}m '
                f'(threshold: {self.path_deviation_threshold:.2f}m). Replanning... (replan #{self.replan_count})'
            )

            # Stop the robot briefly
            cmd = Twist()
            self.cmd_pub.publish(cmd)

            # Keep current goal but clear path to force replanning
            self.current_path = None
            self.current_waypoint_index = 0

            # Transition to PLAN_PATH to create new path to same goal
            self.previous_state = self.state
            self.state = State.PLAN_PATH
            self.get_logger().info('[STATE: EXECUTE_PATH → PLAN_PATH] Replanning due to path deviation')
            return

        # Check if path complete
        if self.current_waypoint_index >= len(self.current_path):
            self.get_logger().info(f'[STATE: EXECUTE_PATH] ✓ Path complete! Reached frontier at [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}]')

            cmd = Twist()
            self.cmd_pub.publish(cmd)

            # Clear current path and goal
            self.current_path = None
            self.current_goal = None

            # Transition to detect new frontiers
            self.previous_state = self.state
            self.state = State.DETECT_FRONTIERS
            self.get_logger().info('[STATE: EXECUTE_PATH → DETECT_FRONTIERS] Searching for next frontier...')
            return

        frontiers = self._get_frontiers()  # Get event-driven frontiers (updated in map_callback)

        if len(frontiers) > 0:
            # DYNAMIC REPLANNING: Check if best frontier has changed
            best_frontier = frontiers[0]  # Frontiers already sorted by score

            # Check if it's different from current goal
            goal_distance = np.linalg.norm(best_frontier.position - self.current_goal)

            if goal_distance > 0.5:  # Different frontier (not current goal)
                # Goal changed - replan to new best frontier
                dist_to_old_goal = np.linalg.norm(self.current_goal - self.robot_pos)
                dist_to_new_goal = np.linalg.norm(best_frontier.position - self.robot_pos)

                self.get_logger().warn(f'[STATE: EXECUTE_PATH] Goal changed - replanning!')
                self.get_logger().warn(f'  Old goal: [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}] (dist={dist_to_old_goal:.2f}m)')
                self.get_logger().warn(f'  New goal: [{best_frontier.position[0]:.2f}, {best_frontier.position[1]:.2f}] (dist={dist_to_new_goal:.2f}m, score={best_frontier.score:.3f})')
                self.get_logger().warn(f'  Total frontiers: {len(frontiers)}')

                # Stop the robot
                cmd = Twist()
                self.cmd_pub.publish(cmd)

                # Clear current path
                self.current_path = None
                self.current_waypoint_index = 0

                # Set new goal
                self.current_goal = best_frontier.position.copy()

                # Transition to PLAN_PATH
                self.previous_state = self.state
                self.state = State.PLAN_PATH
                self.get_logger().info('[STATE: EXECUTE_PATH → PLAN_PATH] Replanning to new frontier')
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
                self.scan_danger_distance,
                self.scan_emergency_distance,
                self.scan_angular_range
            )
        else:
            obstacle_detected, min_distance, avoidance_direction = False, float('inf'), 0.0

        if obstacle_detected:
            current_time = self.get_clock().now()

            # Log warning (with cooldown to avoid spam)
            should_warn = (self.last_obstacle_warning_time is None or
                          (current_time - self.last_obstacle_warning_time).nanoseconds / 1e9 > self.obstacle_warning_cooldown)

            if should_warn:
                self.get_logger().warn(
                    f'[REACTIVE AVOIDANCE] Obstacle detected at {min_distance:.2f}m! '
                    f'Avoidance: {"LEFT" if avoidance_direction < 0 else "RIGHT" if avoidance_direction > 0 else "STOP"}'
                )
                self.last_obstacle_warning_time = current_time

            # Emergency stop if very close - force replanning
            if min_distance < self.scan_emergency_distance:
                self.get_logger().error(
                    f'[EMERGENCY STOP] Obstacle at {min_distance:.2f}m < emergency threshold {self.scan_emergency_distance:.2f}m!'
                )

                # STOP immediately
                cmd = Twist()
                self.cmd_pub.publish(cmd)

                # Force replanning by clearing current goal and path
                self.current_path = None
                self.current_goal = None
                self.current_waypoint_index = 0

                # Transition to detect new frontiers
                self.state = State.DETECT_FRONTIERS
                self.get_logger().warn('[EMERGENCY] Clearing blocked goal and searching for alternative frontiers!')
                return
            # Slow down and steer away if in danger zone
            else:
                # Reduce speed proportionally to distance
                speed_factor = (min_distance - self.scan_emergency_distance) / (self.scan_danger_distance - self.scan_emergency_distance)
                speed_factor = np.clip(speed_factor, 0.3, 1.0)  # At least 30% speed
                v = v * speed_factor

                # Add corrective steering (blend with planned steering)
                correction_gain = 0.25  # How much to steer away (0-1), reduced from 0.4 for gentler avoidance
                w = w + correction_gain * avoidance_direction * self.controller.max_w

        # Log progress periodically
        if self.current_waypoint_index % 10 == 0:
            progress = (self.current_waypoint_index / len(self.current_path)) * 100
            dist_to_goal = np.linalg.norm(self.robot_pos - self.current_path[-1])
            obstacle_status = f'AVOIDING (d={min_distance:.2f}m)' if obstacle_detected else 'CLEAR'
            self.get_logger().info(
                f'[STATE: EXECUTE_PATH] Waypoint {self.current_waypoint_index}/{len(self.current_path)} '
                f'({progress:.1f}%), dist to goal: {dist_to_goal:.2f}m, v={v:.2f}, w={w:.2f}, scan={obstacle_status}'
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

    def _get_frontiers(self) -> list:
        """
        Get current frontiers (event-driven approach).
        Frontiers are detected in map_callback when map updates, so just return stored value.
        """
        return self.all_frontiers

    def _is_stuck(self) -> bool:
        """Check if robot is stuck (not making progress)."""
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
        """Publish ALL visualizations continuously for debugging"""
        # 1. Publish frontier markers
        self._publish_frontier_markers()

        # 2. Publish planned path
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
            marker.header.frame_id = f'{self.robot_name}/odom'
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

    def _publish_path(self):
        """Publish planned path as line strip"""
        if self.current_path is None or len(self.current_path) == 0:
            return

        path_msg = Path()
        path_msg.header.frame_id = f'{self.robot_name}/odom'
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

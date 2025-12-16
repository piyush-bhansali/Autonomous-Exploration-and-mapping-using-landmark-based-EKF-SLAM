#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import struct
from enum import Enum
import threading

from navigation.simple_frontiers import SimpleFrontierDetector
from map_generation.utils import quaternion_to_yaw
from navigation.rrt_star import RRTStar
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
        # NOTE: use_sim_time is automatically declared by ROS 2, don't declare it again!
        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('enable_reactive_avoidance', True)
        self.declare_parameter('scan_danger_distance', 0.5)
        self.declare_parameter('scan_emergency_distance', 0.3)
        self.declare_parameter('scan_angular_range', 60.0)
        self.declare_parameter('enable_path_deviation_check', True)
        self.declare_parameter('path_deviation_threshold', 1.5)
        self.declare_parameter('path_deviation_check_interval', 4.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.enable_reactive_avoidance = self.get_parameter('enable_reactive_avoidance').value
        self.scan_danger_distance = self.get_parameter('scan_danger_distance').value
        self.scan_emergency_distance = self.get_parameter('scan_emergency_distance').value
        self.scan_angular_range = np.radians(self.get_parameter('scan_angular_range').value)
        self.enable_path_deviation_check = self.get_parameter('enable_path_deviation_check').value
        self.path_deviation_threshold = self.get_parameter('path_deviation_threshold').value
        self.path_deviation_check_interval = self.get_parameter('path_deviation_check_interval').value

        # State
        self.state = State.WAIT_FOR_MAP
        self.previous_state = None  # Track previous state for replan logic
        self.robot_pos = None
        self.robot_yaw = None
        self.map_points = None
        self.current_path = None
        self.current_waypoint_index = 0
        # Track unreachable frontiers with context (position-dependent)
        # Each entry: {'frontier_pos': [x,y], 'robot_pos': [x,y], 'has_alternatives': bool}
        self.unreachable_frontiers = []
        self.current_goal = None
        self.all_frontiers = []  # Store all detected frontiers for continuous visualization

        # BUG FIX #14: Stuck detection
        self.stuck_detection_enabled = True
        self.stuck_check_window = 5.0  # seconds to check for movement
        self.stuck_distance_threshold = 0.1  # meters - must move at least this far
        self.last_stuck_check_time = None
        self.last_stuck_check_position = None
        self.stuck_timeout = 30.0  # seconds - max time in EXECUTE_PATH state
        self.execute_path_start_time = None

        # BUG FIX #15: Thread safety for map updates
        self.map_lock = threading.Lock()

        # Scan-based obstacle avoidance
        self.scan_data = None
        self.scan_lock = threading.Lock()
        self.last_obstacle_warning_time = None
        self.obstacle_warning_cooldown = 2.0  # seconds between warnings

        # Frontier detection caching (avoid redundant expensive calls)
        self.cached_frontiers = []
        self.frontier_cache_time = None
        self.frontier_cache_duration = 0.5  # seconds - cache valid for 0.5s

        # Path deviation detection
        self.last_deviation_check_time = None
        self.replan_count = 0

        # Dynamic replanning thresholds
        self.replan_score_threshold = 0.50  # Replan if new frontier score improves by >50% (increased from 30% to prevent oscillation)
        self.replan_distance_threshold = 4.0  # Replan if new frontier is >4m closer

        # Minimum frontier distance to prevent selecting frontiers too close (especially after corridor exit)
        self.min_frontier_distance = 2.0  # Force frontiers to be at least 2.0m away

        # Components
        self.frontier_detector = SimpleFrontierDetector(self.robot_radius)
        self.controller = SmoothedPurePursuit(
            lookahead_distance=1.2,  # Constant higher lookahead for smoother paths
            max_linear_velocity=0.18,
            min_linear_velocity=0.08,
            max_angular_velocity=0.5,
            angular_smoothing_factor=0.6,
            goal_tolerance=0.3,
            velocity_gain=0.7
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

        # Scan subscriber for reactive obstacle avoidance
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Sensor data typically uses BEST_EFFORT
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            scan_qos
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.frontier_markers_pub = self.create_publisher(MarkerArray, f'/{self.robot_name}/frontier_markers', 10)
        self.path_pub = self.create_publisher(Path, f'/{self.robot_name}/planned_path', 10)

        # Control timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'Simple Navigation started for {self.robot_name}')

    def map_callback(self, msg: PointCloud2):
        """Receive map from SLAM"""
        # Parse PointCloud2
        points = self._parse_pointcloud2(msg)

        if len(points) < 100:
            return  # Need minimum map size

        # BUG FIX #15: Use lock to prevent race condition with control loop
        with self.map_lock:
            self.map_points = points

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

        # Extract yaw from quaternion using utility function
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_yaw = quaternion_to_yaw(qx, qy, qz, qw)

    def scan_callback(self, msg: LaserScan):
        """Update scan data for reactive obstacle avoidance and RRT* obstacles"""
        with self.scan_lock:
            self.scan_data = msg

        # Note: Scan conversion to points happens lazily in _get_combined_obstacle_map()
        # to avoid wasting CPU on scans that never get used for planning

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
        # Get frontiers (uses cache if valid)
        frontiers = self._get_frontiers(force_refresh=True)  # Force refresh in DETECT_FRONTIERS state

        if len(frontiers) == 0 and self.map_points is not None:
            self.get_logger().info(f'[STATE: DETECT_FRONTIERS] Map: {len(self.map_points)} points, Unreachable records: {len(self.unreachable_frontiers)}')

        # Store for continuous visualization
        self.all_frontiers = frontiers

        if len(frontiers) == 0:
            self.get_logger().warn('[STATE: DETECT_FRONTIERS] No frontiers found - exploration complete')
            self.state = State.DONE
            return

        self.get_logger().info(f'[STATE: DETECT_FRONTIERS] Found {len(frontiers)} total frontiers')

        # Filter out unreachable frontiers (position-dependent filtering)
        reachable = self._filter_unreachable_frontiers(frontiers, self.robot_pos)

        if len(reachable) == 0:
            self.get_logger().warn(
                f'[STATE: DETECT_FRONTIERS] All {len(frontiers)} frontiers unreachable from current position '
                f'({len(self.unreachable_frontiers)} unreachable records)'
            )
            for i, ur in enumerate(self.unreachable_frontiers):
                self.get_logger().warn(
                    f'  Unreachable #{i+1}: frontier=[{ur["frontier_pos"][0]:.2f}, {ur["frontier_pos"][1]:.2f}], '
                    f'from_position=[{ur["robot_pos"][0]:.2f}, {ur["robot_pos"][1]:.2f}], '
                    f'had_alternatives={ur["has_alternatives"]}'
                )
            self.state = State.DONE
            return

        self.get_logger().info(
            f'[STATE: DETECT_FRONTIERS] {len(reachable)} reachable frontiers available '
            f'(filtered {len(frontiers) - len(reachable)} unreachable)'
        )

        # Filter frontiers that are too close (minimum distance requirement)
        distant_frontiers = []
        for f in reachable:
            dist = np.linalg.norm(f.position - self.robot_pos)
            if dist >= self.min_frontier_distance:
                distant_frontiers.append(f)
            else:
                self.get_logger().debug(
                    f'  Frontier at [{f.position[0]:.2f}, {f.position[1]:.2f}] too close '
                    f'(dist={dist:.2f}m < min={self.min_frontier_distance}m)'
                )

        # If no distant frontiers, use closest available (fallback)
        if len(distant_frontiers) == 0:
            self.get_logger().warn(
                f'[STATE: DETECT_FRONTIERS] All {len(reachable)} frontiers < {self.min_frontier_distance}m. '
                f'Using closest available frontier.'
            )
            distant_frontiers = reachable

        self.get_logger().info(
            f'[STATE: DETECT_FRONTIERS] {len(distant_frontiers)} frontiers meet distance requirement '
            f'(≥{self.min_frontier_distance}m)'
        )

        # Select best frontier from distant ones
        best_frontier = distant_frontiers[0]
        self.current_goal = best_frontier.position
        dist_to_selected = np.linalg.norm(self.current_goal - self.robot_pos)

        self.get_logger().info(
            f'[STATE: DETECT_FRONTIERS] Selected frontier at [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}] '
            f'(dist={dist_to_selected:.2f}m, score={best_frontier.score:.3f})'
        )

        # Transition to planning
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

        # Get combined obstacle map (static map + recent LiDAR scan)
        combined_obstacles = self._get_combined_obstacle_map()

        # Update planner with fresh obstacles (includes recent LiDAR)
        path_planner_with_fresh_obstacles = RRTStar(
            combined_obstacles,
            robot_radius=self.robot_radius,
            step_size=0.4,
            goal_bias=0.5
        )

        # Plan with RRT*
        path = path_planner_with_fresh_obstacles.plan(self.robot_pos, self.current_goal)

        num_static = len(self.map_points) if self.map_points is not None else 0
        num_scan = len(combined_obstacles) - num_static
        self.get_logger().info(f'  RRT* using {len(combined_obstacles)} obstacles ({num_static} static + {num_scan} from scan)')

        if path is None:
            self.get_logger().warn('[STATE: PLAN_PATH] Planning FAILED - marking frontier as unreachable from current position')

            # Mark as unreachable using helper method
            self._mark_frontier_unreachable(self.current_goal, reason="path planning failed")

            self.current_goal = None
            self.state = State.DETECT_FRONTIERS
            return

        self.get_logger().info(f'[STATE: PLAN_PATH] Success! {len(path)} waypoints, distance: {self._path_length(path):.2f}m')

        # Start path execution
        self.current_path = path

        # BUG FIX #16: Resume from nearest waypoint when replanning (prevents U-turns)
        if self.previous_state == State.EXECUTE_PATH:
            # Replanning during execution - resume from nearest forward waypoint
            self.current_waypoint_index = self._find_nearest_waypoint_index(path)
            self.get_logger().info(
                f'[STATE: PLAN_PATH] Replanning: Resuming from waypoint {self.current_waypoint_index}/{len(path)} '
                f'(nearest to robot position)'
            )
        else:
            # New path - start from beginning
            self.current_waypoint_index = 0
            self.get_logger().info('[STATE: PLAN_PATH] New path: Starting from waypoint 0')

        self.state = State.EXECUTE_PATH

        # Initialize stuck detection timers
        self.execute_path_start_time = self.get_clock().now()
        self.last_stuck_check_time = self.get_clock().now()
        self.last_stuck_check_position = self.robot_pos.copy()

        self.get_logger().info('[STATE: PLAN_PATH → EXECUTE_PATH] Starting path execution')

    def _path_length(self, path):
        """Calculate total path length"""
        total = 0.0
        for i in range(len(path) - 1):
            total += np.linalg.norm(path[i+1] - path[i])
        return total

    def _distance(self, pos1: np.ndarray, pos2: np.ndarray) -> float:
        """
        Calculate Euclidean distance between two 2D positions.

        Helper method to improve code readability and reduce repetitive calculations.

        Args:
            pos1: First position [x, y]
            pos2: Second position [x, y]

        Returns:
            Distance in meters
        """
        return np.linalg.norm(pos1 - pos2)

    def _find_nearest_waypoint_index(self, path: np.ndarray) -> int:
        """
        Find the nearest waypoint ahead of the robot for path resumption.

        Used when replanning to avoid U-turns - robot continues from nearest
        forward waypoint instead of returning to path start.

        Args:
            path: Array of waypoints [N x 2]

        Returns:
            Index of nearest forward waypoint (0 if none found)
        """
        if path is None or len(path) == 0:
            return 0

        # Calculate distances from robot to all waypoints
        distances = np.linalg.norm(path - self.robot_pos, axis=1)

        # Find closest waypoint
        nearest_idx = np.argmin(distances)

        # Check if we should look ahead (skip waypoints behind us)
        # If nearest waypoint is very close (<0.3m), try next waypoint
        if nearest_idx < len(path) - 1 and distances[nearest_idx] < 0.3:
            # Check if next waypoint is also close
            if distances[nearest_idx + 1] < 1.0:
                return nearest_idx + 1

        return nearest_idx

    def _handle_execute_path(self):
        """Execute path using pure pursuit"""
        # BUG FIX #14: Check for stuck condition
        if self.stuck_detection_enabled and self._is_stuck():
            self.get_logger().warn('[STATE: EXECUTE_PATH] Robot is STUCK! Aborting current path and replanning...')

            # Stop the robot
            cmd = Twist()
            self.cmd_pub.publish(cmd)

            # Mark current goal as unreachable from current position (avoid re-selecting same unreachable goal)
            if self.current_goal is not None:
                self._mark_frontier_unreachable(self.current_goal, reason="stuck")

            # Clear current path and goal
            self.current_path = None
            self.current_goal = None

            # Transition back to frontier detection
            self.state = State.DETECT_FRONTIERS
            self.get_logger().info('[STATE: EXECUTE_PATH → DETECT_FRONTIERS] Recovering from stuck condition...')
            return

        # PATH DEVIATION CHECK: Replan if robot deviated too far from path
        is_deviated, deviation_distance = self._check_path_deviation()
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

            # Successfully reached frontier - no need to mark as unreachable
            # Gap filtering will naturally prevent re-selecting explored areas

            # Stop the robot
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

        # RE-DETECT FRONTIERS: Update frontier list as map grows during execution
        # This ensures we're always navigating to the best available frontier
        frontiers = self._get_frontiers()  # Use cached frontiers (0.5s cache)
        self.all_frontiers = frontiers  # Update for visualization

        if len(frontiers) > 0:

            # DYNAMIC REPLANNING: Check if we should switch to a better frontier
            should_replan, new_goal, reason = self._should_replan_to_new_frontier(frontiers, self.current_goal)

            if should_replan:
                self.get_logger().warn(f'[STATE: EXECUTE_PATH] REPLANNING! {reason}')
                self.get_logger().warn(f'  Old goal: [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}]')
                self.get_logger().warn(f'  New goal: [{new_goal[0]:.2f}, {new_goal[1]:.2f}]')

                # Stop the robot
                cmd = Twist()
                self.cmd_pub.publish(cmd)

                # Clear current path
                self.current_path = None
                self.current_waypoint_index = 0

                # Set new goal
                self.current_goal = new_goal.copy()

                # Transition to PLAN_PATH to create new path
                self.previous_state = self.state
                self.state = State.PLAN_PATH
                self.get_logger().info('[STATE: EXECUTE_PATH → PLAN_PATH] Replanning to better frontier')
                return

        # Check if reached current waypoint
        waypoint = self.current_path[self.current_waypoint_index]
        dist_to_waypoint = np.linalg.norm(self.robot_pos - waypoint)

        if dist_to_waypoint < 0.25:  # Waypoint tolerance (increased for better handling of sharp turns)
            self.current_waypoint_index += 1
            return

        # Compute control from path planner
        v, w = self.controller.compute_control(
            self.robot_pos,
            self.robot_yaw,
            self.current_path,
            self.current_waypoint_index
        )

        # REACTIVE OBSTACLE AVOIDANCE: Check scan for immediate obstacles
        obstacle_detected, min_distance, avoidance_direction = self._check_scan_for_obstacles()

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

    def _mark_frontier_unreachable(self, frontier_pos: np.ndarray, reason: str = ""):
        """
        Mark a frontier as unreachable from current robot position.

        This helper consolidates duplicate code from path planning failures
        and stuck detection.

        Args:
            frontier_pos: Position of unreachable frontier [x, y]
            reason: Human-readable reason for marking unreachable
        """
        # Check if there are alternative frontiers within 2m of robot
        current_frontiers = self._get_frontiers()  # Use cached frontiers

        has_alternatives = False
        if len(current_frontiers) > 0:
            nearby_alternatives = [
                f for f in current_frontiers
                if np.linalg.norm(f.position - self.robot_pos) < 2.0
                and np.linalg.norm(f.position - frontier_pos) > 0.5  # Not the failed goal itself
            ]
            has_alternatives = len(nearby_alternatives) > 0

        # Add to unreachable frontiers with context
        self.unreachable_frontiers.append({
            'frontier_pos': frontier_pos.copy(),
            'robot_pos': self.robot_pos.copy(),
            'has_alternatives': has_alternatives
        })

        self.get_logger().info(
            f'  Marked unreachable ({reason}): frontier=[{frontier_pos[0]:.2f}, {frontier_pos[1]:.2f}], '
            f'robot_pos=[{self.robot_pos[0]:.2f}, {self.robot_pos[1]:.2f}], '
            f'had_alternatives={has_alternatives}'
        )

    def _get_frontiers(self, force_refresh: bool = False) -> list:
        """
        Get frontiers with caching to avoid redundant expensive calls.

        Args:
            force_refresh: If True, bypass cache and detect fresh frontiers

        Returns:
            List of SimpleFrontier objects
        """
        current_time = self.get_clock().now()

        # Check cache validity
        if not force_refresh and self.frontier_cache_time is not None:
            time_since_cache = (current_time - self.frontier_cache_time).nanoseconds / 1e9
            if time_since_cache < self.frontier_cache_duration:
                # Cache still valid
                return self.cached_frontiers

        # Cache expired or force refresh - detect new frontiers
        with self.map_lock:
            map_points_copy = self.map_points.copy() if self.map_points is not None else None

        if map_points_copy is None:
            return []

        frontiers = self.frontier_detector.detect(map_points_copy, self.robot_pos, self.robot_yaw)

        # Update cache
        self.cached_frontiers = frontiers
        self.frontier_cache_time = current_time

        return frontiers

    def _filter_unreachable_frontiers(self, frontiers: list, robot_pos: np.ndarray) -> list:
        """
        Filter out frontiers that were unreachable from similar robot positions.

        This is a position-dependent filter that only removes frontiers if:
        1. Frontier is near a previously unreachable one (within 0.5m)
        2. Robot is near the position where it was unreachable (within 2m)
        3. No alternatives existed at that position

        This allows re-approaching frontiers from different angles/positions.

        Args:
            frontiers: List of SimpleFrontier objects to filter
            robot_pos: Current robot position [x, y]

        Returns:
            List of frontiers that are potentially reachable from current position
        """
        valid_frontiers = []

        for f in frontiers:
            is_unreachable = False

            for ur in self.unreachable_frontiers:
                # Check if this frontier is near a previously unreachable one
                frontier_match = np.linalg.norm(f.position - ur['frontier_pos']) < 0.5

                # Check if robot is near the position where it was unreachable
                position_match = np.linalg.norm(robot_pos - ur['robot_pos']) < 2.0

                # Only filter if:
                # 1. Same frontier location AND
                # 2. Robot at similar position AND
                # 3. No alternatives existed at that position
                if frontier_match and position_match and not ur['has_alternatives']:
                    is_unreachable = True
                    self.get_logger().debug(
                        f'  Filtered unreachable frontier [{f.position[0]:.2f}, {f.position[1]:.2f}] '
                        f'(previously unreachable from [{ur["robot_pos"][0]:.2f}, {ur["robot_pos"][1]:.2f}])'
                    )
                    break

            if not is_unreachable:
                valid_frontiers.append(f)

        return valid_frontiers

    def _should_replan_to_new_frontier(self, new_frontiers: list, current_goal: np.ndarray) -> tuple:
        """
        Check if robot should abandon current path for a better frontier.

        IMPORTANT: This function should NOT filter current_goal by min_frontier_distance.
        The robot has already committed to the current goal, and filtering it out would
        cause premature replanning as the robot approaches (within 2m of) the goal.

        Only NEW candidate frontiers are filtered by distance to prevent selecting
        frontiers too close to the robot.

        Returns:
            (should_replan: bool, new_goal: np.ndarray or None, reason: str)
        """
        if len(new_frontiers) == 0:
            return False, None, ""

        # Filter out unreachable frontiers (same logic as _handle_detect_frontiers)
        reachable = self._filter_unreachable_frontiers(new_frontiers, self.robot_pos)

        if len(reachable) == 0:
            return False, None, ""

        # Find current goal in new frontier list (BEFORE distance filtering!)
        # This ensures we can detect if current goal is still valid, even if it's now < 2m away
        current_frontier = None
        current_score = 0.0
        for f in reachable:
            if np.linalg.norm(f.position - current_goal) < 0.5:  # Same frontier if within 0.5m
                current_frontier = f
                current_score = f.score
                break

        # REASON 1: Current goal has disappeared (blocked, unreachable, or no longer valid)
        if current_frontier is None:
            # Current goal disappeared from frontier list (truly gone, not just filtered by distance)
            # Find best alternative that meets distance requirement
            distant_candidates = [f for f in reachable
                                  if np.linalg.norm(f.position - self.robot_pos) >= self.min_frontier_distance]

            if len(distant_candidates) == 0:
                # No valid alternatives, keep current goal
                return False, None, ""

            best_new_frontier = distant_candidates[0]
            reason = f"Current goal disappeared from frontier list"
            return True, best_new_frontier.position, reason

        # Current goal is still valid! Check if we should switch to a better one

        # Filter NEW candidate frontiers by distance (exclude current goal and frontiers too close)
        candidate_frontiers = []
        for f in reachable:
            # Skip current frontier (we're already going there!)
            if np.linalg.norm(f.position - current_goal) < 0.5:
                continue

            # Only consider frontiers that meet distance requirement
            dist_to_robot = np.linalg.norm(f.position - self.robot_pos)
            if dist_to_robot >= self.min_frontier_distance:
                candidate_frontiers.append(f)

        if len(candidate_frontiers) == 0:
            # No valid alternatives, keep current goal
            return False, None, ""

        # Best alternative frontier
        best_new_frontier = candidate_frontiers[0]
        best_new_pos = best_new_frontier.position
        best_new_score = best_new_frontier.score

        # REASON 2: Score improvement (new frontier significantly better)
        score_improvement = best_new_score - current_score
        if score_improvement > self.replan_score_threshold:
            reason = f"Better frontier found (score: {current_score:.3f} → {best_new_score:.3f}, +{score_improvement:.3f})"
            return True, best_new_pos, reason

        # REASON 3: Distance improvement (new frontier much closer)
        dist_to_current = np.linalg.norm(current_goal - self.robot_pos)
        dist_to_new = np.linalg.norm(best_new_pos - self.robot_pos)
        distance_improvement = dist_to_current - dist_to_new

        if distance_improvement > self.replan_distance_threshold:
            reason = f"Closer frontier found (dist: {dist_to_current:.2f}m → {dist_to_new:.2f}m, -{distance_improvement:.2f}m)"
            return True, best_new_pos, reason

        # No replanning needed - current goal is still best
        return False, None, ""

    def _check_scan_for_obstacles(self) -> tuple:
        """
        Check laser scan for obstacles in front of the robot.

        Returns:
            (obstacle_detected: bool, min_distance: float, avoidance_direction: float)
            - obstacle_detected: True if obstacle within danger zone
            - min_distance: Minimum distance to obstacle in front sector
            - avoidance_direction: Suggested steering direction (-1=left, 0=straight, 1=right)
        """
        if not self.enable_reactive_avoidance:
            return False, float('inf'), 0.0

        with self.scan_lock:
            scan = self.scan_data

        if scan is None:
            # No scan data available - degraded mode
            return False, float('inf'), 0.0

        # Scan parameters
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment
        ranges = np.array(scan.ranges)

        # Replace inf/nan with max range
        max_range = scan.range_max
        ranges = np.where(np.isfinite(ranges), ranges, max_range)

        # Divide scan into sectors: front, left, right
        num_readings = len(ranges)

        # Front sector: -scan_angular_range/2 to +scan_angular_range/2
        # Calculate indices for front sector
        half_angular_range = self.scan_angular_range / 2.0

        # Find indices corresponding to front sector
        angles = angle_min + np.arange(num_readings) * angle_increment
        front_mask = np.abs(angles) <= half_angular_range
        left_mask = (angles > half_angular_range) & (angles <= np.pi/2)
        right_mask = (angles < -half_angular_range) & (angles >= -np.pi/2)

        # Get minimum distances in each sector
        front_ranges = ranges[front_mask]
        left_ranges = ranges[left_mask]
        right_ranges = ranges[right_mask]

        min_front = np.min(front_ranges) if len(front_ranges) > 0 else max_range
        min_left = np.min(left_ranges) if len(left_ranges) > 0 else max_range
        min_right = np.min(right_ranges) if len(right_ranges) > 0 else max_range

        # Determine if obstacle detected
        obstacle_detected = min_front < self.scan_danger_distance

        # Determine avoidance direction (steer away from closest obstacle)
        # If obstacle in front, steer towards the more open side
        avoidance_direction = 0.0
        if obstacle_detected:
            if min_left > min_right:
                avoidance_direction = -1.0  # Steer left (more space on left)
            else:
                avoidance_direction = 1.0   # Steer right (more space on right)

        return obstacle_detected, min_front, avoidance_direction

    def _check_path_deviation(self) -> tuple:
        """
        Check if robot has deviated too far from the planned path.

        Returns:
            (is_deviated: bool, min_distance_to_path: float)
            - is_deviated: True if robot is too far from any point on path
            - min_distance_to_path: Minimum distance to the path
        """
        if not self.enable_path_deviation_check:
            return False, 0.0

        if self.current_path is None or len(self.current_path) == 0:
            return False, 0.0

        # Check at regular intervals (not every loop iteration)
        current_time = self.get_clock().now()
        if self.last_deviation_check_time is not None:
            time_since_check = (current_time - self.last_deviation_check_time).nanoseconds / 1e9
            if time_since_check < self.path_deviation_check_interval:
                return False, 0.0  # Not time to check yet

        self.last_deviation_check_time = current_time

        # Calculate minimum distance from robot to any point on remaining path
        # Only check from current waypoint onwards (not behind us)
        remaining_path = self.current_path[self.current_waypoint_index:]

        if len(remaining_path) == 0:
            return False, 0.0

        distances = np.linalg.norm(remaining_path - self.robot_pos, axis=1)
        min_distance = np.min(distances)

        # Check if deviation exceeds threshold
        is_deviated = min_distance > self.path_deviation_threshold

        return is_deviated, min_distance

    def _is_stuck(self) -> bool:
        """
        BUG FIX #14: Detect if robot is stuck.

        Returns True if:
        1. Robot hasn't moved >0.1m in the last 5 seconds, OR
        2. Robot has been in EXECUTE_PATH state for >30 seconds

        This prevents infinite loops when robot gets stuck on obstacles
        or when path is unreachable.
        """
        current_time = self.get_clock().now()

        # Check 1: Global timeout (30 seconds in EXECUTE_PATH)
        if self.execute_path_start_time is not None:
            time_in_state = (current_time - self.execute_path_start_time).nanoseconds / 1e9
            if time_in_state > self.stuck_timeout:
                self.get_logger().warn(
                    f'[STUCK DETECTION] Global timeout: {time_in_state:.1f}s > {self.stuck_timeout}s in EXECUTE_PATH'
                )
                return True

        # Check 2: Movement check (every 5 seconds)
        if self.last_stuck_check_time is None or self.last_stuck_check_position is None:
            # Initialize
            self.last_stuck_check_time = current_time
            self.last_stuck_check_position = self.robot_pos.copy()
            return False

        time_since_check = (current_time - self.last_stuck_check_time).nanoseconds / 1e9

        if time_since_check >= self.stuck_check_window:
            # Check if robot has moved enough
            distance_moved = np.linalg.norm(self.robot_pos - self.last_stuck_check_position)

            if distance_moved < self.stuck_distance_threshold:
                self.get_logger().warn(
                    f'[STUCK DETECTION] Insufficient movement: {distance_moved:.3f}m < {self.stuck_distance_threshold}m '
                    f'in {time_since_check:.1f}s'
                )
                return True

            # Update check window
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

    def _convert_scan_to_points(self, scan: LaserScan) -> np.ndarray:
        """
        Convert LaserScan to 2D points in robot frame.

        Args:
            scan: LaserScan message

        Returns:
            Nx2 array of [x, y] points in robot frame (relative to base_scan)
        """
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment

        # Replace inf/nan with max range (these are not obstacles)
        valid_mask = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)

        # Generate angles for each reading
        num_readings = len(ranges)
        angles = angle_min + np.arange(num_readings) * angle_increment

        # Convert polar to Cartesian (robot frame)
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Filter to valid points only
        points = np.column_stack([x[valid_mask], y[valid_mask]])

        return points

    def _transform_points_to_global(self, points: np.ndarray, robot_pos: np.ndarray, robot_yaw: float) -> np.ndarray:
        """
        Transform points from robot frame to global frame.

        Args:
            points: Nx2 array of [x, y] points in robot frame
            robot_pos: [x, y] robot position in global frame
            robot_yaw: Robot heading in radians

        Returns:
            Nx2 array of [x, y] points in global frame
        """
        if points is None or len(points) == 0:
            return np.array([]).reshape(0, 2)

        # Rotation matrix
        cos_yaw = np.cos(robot_yaw)
        sin_yaw = np.sin(robot_yaw)

        # Apply rotation and translation
        x_global = robot_pos[0] + points[:, 0] * cos_yaw - points[:, 1] * sin_yaw
        y_global = robot_pos[1] + points[:, 0] * sin_yaw + points[:, 1] * cos_yaw

        points_global = np.column_stack([x_global, y_global])

        return points_global

    def _get_combined_obstacle_map(self) -> np.ndarray:
        """
        Combine static map points with recent LiDAR scan points.

        OPTIMIZATION: Converts scan lazily only when needed for planning.
        This avoids wasting 90-99% of scan conversions that never get used.

        Returns:
            Nx2 array of [x, y] obstacle points (static map + recent scan)
        """
        combined = []

        # Add static map points (from submaps)
        if self.map_points is not None and len(self.map_points) > 0:
            combined.append(self.map_points[:, :2])  # Take only x,y (drop z)

        # Add recent scan points (convert lazily here)
        with self.scan_lock:
            scan = self.scan_data

        if scan is not None and self.robot_pos is not None and self.robot_yaw is not None:
            # Convert scan to 2D points on-demand
            scan_points_robot_frame = self._convert_scan_to_points(scan)

            # Transform to global frame
            scan_points_global = self._transform_points_to_global(
                scan_points_robot_frame,
                self.robot_pos,
                self.robot_yaw
            )

            if len(scan_points_global) > 0:
                combined.append(scan_points_global)

        # Combine all obstacle sources
        if len(combined) == 0:
            return np.array([]).reshape(0, 2)

        obstacle_map = np.vstack(combined)

        return obstacle_map

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

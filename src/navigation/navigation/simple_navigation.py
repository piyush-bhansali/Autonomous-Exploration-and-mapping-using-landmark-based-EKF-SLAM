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

        # Dynamic replanning thresholds
        self.replan_score_threshold = 0.30  # Replan if new frontier score improves by >30% (increased from 15% to prevent oscillation)
        self.replan_distance_threshold = 3.0  # Replan if new frontier is >3m closer

        # Components (created when map received)
        self.frontier_detector = SimpleFrontierDetector(self.robot_radius)
        self.path_planner = None
        self.controller = SmoothedPurePursuit(
            lookahead_distance=0.6,
            max_linear_velocity=0.2,
            min_linear_velocity=0.1,
            max_angular_velocity=0.8,
            angular_smoothing_factor=0.3,
            goal_tolerance=0.3,
            velocity_gain=0.5
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

            # Create/update path planner
            self.path_planner = RRTStar(
                self.map_points,
                robot_radius=self.robot_radius,
                step_size=0.4,
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

        # Extract yaw from quaternion using utility function
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_yaw = quaternion_to_yaw(qx, qy, qz, qw)

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
        # BUG FIX #15: Acquire lock when reading map_points
        with self.map_lock:
            map_points_copy = self.map_points.copy() if self.map_points is not None else None

        if map_points_copy is None:
            return

        self.get_logger().info(f'[STATE: DETECT_FRONTIERS] Map: {len(map_points_copy)} points, Explored: {len(self.explored_positions)} positions')

        # Detect frontiers
        frontiers = self.frontier_detector.detect(map_points_copy, self.robot_pos, self.robot_yaw)

        # Store for continuous visualization
        self.all_frontiers = frontiers

        if len(frontiers) == 0:
            self.get_logger().warn('[STATE: DETECT_FRONTIERS] No frontiers found - exploration complete')
            self.state = State.DONE
            return

        self.get_logger().info(f'[STATE: DETECT_FRONTIERS] Found {len(frontiers)} total frontiers')

        # Filter out explored frontiers (use 2.0m threshold to avoid revisiting)
        unexplored = []
        for f in frontiers:
            min_dist_to_explored = float('inf')
            for exp_pos in self.explored_positions:
                dist = np.linalg.norm(f.position - exp_pos)
                min_dist_to_explored = min(min_dist_to_explored, dist)

            if min_dist_to_explored > 2.0 or len(self.explored_positions) == 0:
                unexplored.append(f)
            else:
                self.get_logger().debug(f'  Frontier at [{f.position[0]:.2f}, {f.position[1]:.2f}] too close to explored position (dist={min_dist_to_explored:.2f}m)')

        if len(unexplored) == 0:
            self.get_logger().warn(f'[STATE: DETECT_FRONTIERS] All {len(frontiers)} frontiers already explored (within 2.0m of {len(self.explored_positions)} explored positions)')
            for i, exp_pos in enumerate(self.explored_positions):
                self.get_logger().warn(f'  Explored position #{i+1}: [{exp_pos[0]:.2f}, {exp_pos[1]:.2f}]')
            self.state = State.DONE
            return

        self.get_logger().info(f'[STATE: DETECT_FRONTIERS] {len(unexplored)} unexplored frontiers available (filtered {len(frontiers) - len(unexplored)})')

        # Select best frontier
        best_frontier = unexplored[0]
        self.current_goal = best_frontier.position

        self.get_logger().info(
            f'[STATE: DETECT_FRONTIERS] Selected frontier #{1} at [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}] '
            f'(score={best_frontier.score:.3f})'
        )

        # Transition to planning
        self.state = State.PLAN_PATH
        self.get_logger().info('[STATE: DETECT_FRONTIERS → PLAN_PATH]')

    def _handle_plan_path(self):
        """Plan path to current goal"""
        # BUG FIX #15: Acquire lock when accessing path_planner
        with self.map_lock:
            path_planner = self.path_planner

        if path_planner is None:
            self.get_logger().error('[STATE: PLAN_PATH] Path planner not initialized!')
            return

        if self.current_goal is None:
            self.get_logger().error('[STATE: PLAN_PATH] No current goal set!')
            self.state = State.DETECT_FRONTIERS
            return

        self.get_logger().info(f'[STATE: PLAN_PATH] Planning from [{self.robot_pos[0]:.2f}, {self.robot_pos[1]:.2f}] to [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}]')

        # Plan with RRT*
        path = path_planner.plan(self.robot_pos, self.current_goal)

        if path is None:
            self.get_logger().warn('[STATE: PLAN_PATH] Planning FAILED - marking as explored, trying next frontier')
            self.explored_positions.append(self.current_goal.copy())
            self.current_goal = None
            self.state = State.DETECT_FRONTIERS
            return

        self.get_logger().info(f'[STATE: PLAN_PATH] Success! {len(path)} waypoints, distance: {self._path_length(path):.2f}m')

        # Start path execution
        self.current_path = path
        self.current_waypoint_index = 0
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

    def _handle_execute_path(self):
        """Execute path using pure pursuit"""
        # BUG FIX #14: Check for stuck condition
        if self.stuck_detection_enabled and self._is_stuck():
            self.get_logger().warn('[STATE: EXECUTE_PATH] Robot is STUCK! Aborting current path and replanning...')

            # Stop the robot
            cmd = Twist()
            self.cmd_pub.publish(cmd)

            # Mark current goal as explored (avoid re-selecting same unreachable goal)
            if self.current_goal is not None:
                self.explored_positions.append(self.current_goal.copy())

            # Clear current path and goal
            self.current_path = None
            self.current_goal = None

            # Transition back to frontier detection
            self.state = State.DETECT_FRONTIERS
            self.get_logger().info('[STATE: EXECUTE_PATH → DETECT_FRONTIERS] Recovering from stuck condition...')
            return

        # Check if path complete
        if self.current_waypoint_index >= len(self.current_path):
            self.get_logger().info(f'[STATE: EXECUTE_PATH] ✓ Path complete! Reached frontier at [{self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}]')

            # Mark current goal as explored
            self.explored_positions.append(self.current_goal.copy())

            # Stop the robot
            cmd = Twist()
            self.cmd_pub.publish(cmd)

            # Clear current path and goal
            self.current_path = None
            self.current_goal = None

            # Transition to detect new frontiers
            self.state = State.DETECT_FRONTIERS
            self.get_logger().info('[STATE: EXECUTE_PATH → DETECT_FRONTIERS] Searching for next frontier...')
            return

        # RE-DETECT FRONTIERS: Update frontier list as map grows during execution
        # This ensures we're always navigating to the best available frontier
        # BUG FIX #15: Acquire lock when reading map_points
        with self.map_lock:
            map_points_copy = self.map_points.copy() if self.map_points is not None else None

        if map_points_copy is not None:
            frontiers = self.frontier_detector.detect(map_points_copy, self.robot_pos, self.robot_yaw)
            self.all_frontiers = frontiers  # Update for visualization

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
                self.state = State.PLAN_PATH
                self.get_logger().info('[STATE: EXECUTE_PATH → PLAN_PATH] Replanning to better frontier')
                return

        # Check if reached current waypoint
        waypoint = self.current_path[self.current_waypoint_index]
        dist_to_waypoint = np.linalg.norm(self.robot_pos - waypoint)

        if dist_to_waypoint < 0.15:  # Waypoint tolerance (reduced for better tracking)
            self.current_waypoint_index += 1
            return

        # Compute control
        v, w = self.controller.compute_control(
            self.robot_pos,
            self.robot_yaw,
            self.current_path,
            self.current_waypoint_index
        )

        # Log progress periodically
        if self.current_waypoint_index % 10 == 0:
            progress = (self.current_waypoint_index / len(self.current_path)) * 100
            dist_to_goal = np.linalg.norm(self.robot_pos - self.current_path[-1])
            self.get_logger().info(
                f'[STATE: EXECUTE_PATH] Waypoint {self.current_waypoint_index}/{len(self.current_path)} '
                f'({progress:.1f}%), dist to goal: {dist_to_goal:.2f}m, v={v:.2f}, w={w:.2f}'
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

    def _should_replan_to_new_frontier(self, new_frontiers: list, current_goal: np.ndarray) -> tuple:
        """
        Check if robot should abandon current path for a better frontier.

        Returns:
            (should_replan: bool, new_goal: np.ndarray or None, reason: str)
        """
        if len(new_frontiers) == 0:
            return False, None, ""

        # Filter out explored frontiers (same logic as _handle_detect_frontiers)
        unexplored = []
        for f in new_frontiers:
            min_dist_to_explored = float('inf')
            for exp_pos in self.explored_positions:
                dist = np.linalg.norm(f.position - exp_pos)
                min_dist_to_explored = min(min_dist_to_explored, dist)

            if min_dist_to_explored > 2.0 or len(self.explored_positions) == 0:
                unexplored.append(f)

        if len(unexplored) == 0:
            return False, None, ""

        # Best frontier from new detection
        best_new_frontier = unexplored[0]
        best_new_pos = best_new_frontier.position
        best_new_score = best_new_frontier.score

        # Find current goal in new frontier list
        current_frontier = None
        current_score = 0.0
        for f in unexplored:
            if np.linalg.norm(f.position - current_goal) < 0.5:  # Same frontier if within 0.5m
                current_frontier = f
                current_score = f.score
                break

        # REASON 1: Current goal has disappeared (blocked, explored, or no longer valid)
        if current_frontier is None:
            reason = f"Current goal disappeared from frontier list"
            return True, best_new_pos, reason

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

        # No replanning needed
        return False, None, ""

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

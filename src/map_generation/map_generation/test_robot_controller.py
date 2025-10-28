#!/usr/bin/env python3
"""
Test Robot Controller - Moves robot in patterns to demonstrate mapping issues

This script moves the robot in specific patterns to test:
1. Linear drift accumulation
2. Loop closure failure detection
3. Map consistency when revisiting areas
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time
from enum import Enum
from map_generation.utils import (
    quaternion_to_yaw,
    yaw_to_quaternion,
    quaternion_to_rotation_matrix
)


class MovementPattern(Enum):
    """Different movement patterns for testing"""
    SQUARE = 1           # 4x4m square - returns to start (tests loop closure)
    LONG_CORRIDOR = 2    # Long straight path (tests linear drift)
    FIGURE_EIGHT = 3     # Figure-8 pattern (tests complex trajectories)
    SPIRAL = 4           # Outward spiral (tests continuous rotation)


class TestRobotController(Node):
    """
    Controls robot movement for systematic mapping tests.
    Publishes velocity commands and tracks actual position.
    """

    def __init__(self, robot_name='tb3_1', pattern=MovementPattern.LONG_CORRIDOR):
        super().__init__('test_robot_controller')

        # Parameters
        self.robot_name = robot_name
        self.pattern = pattern

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10
        )

        # Subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )

        # State tracking
        self.current_pose = None
        self.start_pose = None
        self.poses_history = []

        # Movement parameters
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 0.5  # rad/s

        # Test state
        self.test_started = False
        self.test_completed = False
        self.movement_step = 0

        self.get_logger().info(f'Test Controller initialized for {robot_name}')
        self.get_logger().info(f'Pattern: {pattern.name}')
        self.get_logger().info('Waiting for odometry...')

    def odom_callback(self, msg):
        """Track robot position from odometry"""
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ),
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }

        # Record pose history
        if self.test_started:
            self.poses_history.append(self.current_pose.copy())

        # Initialize start pose
        if self.start_pose is None:
            self.start_pose = self.current_pose.copy()
            self.get_logger().info(f'Start position: ({self.current_pose["x"]:.2f}, {self.current_pose["y"]:.2f})')

    
    def move_forward(self, distance, speed=None):
        """Move forward by specified distance"""
        if speed is None:
            speed = self.linear_speed

        if self.current_pose is None:
            return

        start_x = self.current_pose['x']
        start_y = self.current_pose['y']
        start_time = time.time()
        timeout = (distance / speed) * 5.0  # 5x expected time as timeout

        twist = Twist()
        twist.linear.x = speed

        rate = self.create_rate(10)  # 10 Hz

        while rclpy.ok():
            # Check timeout
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'⚠️  Movement timeout after {timeout:.1f}s')
                break

            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.0)  # Process callbacks

            dx = self.current_pose['x'] - start_x
            dy = self.current_pose['y'] - start_y
            dist_traveled = np.sqrt(dx**2 + dy**2)

            if dist_traveled >= distance:
                break

            rate.sleep()  # Maintain consistent 10 Hz loop rate

        self.stop()
        self.get_logger().info(f'Moved forward {dist_traveled:.2f}m (target: {distance:.2f}m)')

    def rotate(self, angle, speed=None):
        """Rotate by specified angle (radians)"""
        if speed is None:
            speed = self.angular_speed

        if self.current_pose is None:
            return

        start_theta = self.current_pose['theta']
        target_rotation = abs(angle)
        start_time = time.time()
        timeout = (target_rotation / speed) * 5.0  # 5x expected time as timeout

        twist = Twist()
        twist.angular.z = speed if angle > 0 else -speed

        rate = self.create_rate(10)  # 10 Hz

        while rclpy.ok():
            # Check timeout
            if time.time() - start_time > timeout:
                self.get_logger().warn(f'⚠️  Rotation timeout after {timeout:.1f}s')
                break

            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.0)  # Process callbacks

            # Calculate angle difference using atan2 to handle wraparound
            angle_diff = abs(np.arctan2(np.sin(self.current_pose['theta'] - start_theta),
                                        np.cos(self.current_pose['theta'] - start_theta)))

            if angle_diff >= target_rotation:
                break

            rate.sleep()  # Maintain consistent 10 Hz loop rate

        self.stop()
        angle_rotated = abs(np.arctan2(np.sin(self.current_pose['theta'] - start_theta),
                                       np.cos(self.current_pose['theta'] - start_theta)))
        self.get_logger().info(f'Rotated {np.degrees(angle_rotated):.1f}° (target: {np.degrees(angle):.1f}°)')

    def stop(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)  # Let it settle

    def get_distance_from_start(self):
        """Calculate distance from starting position"""
        if self.start_pose is None or self.current_pose is None:
            return 0.0

        dx = self.current_pose['x'] - self.start_pose['x']
        dy = self.current_pose['y'] - self.start_pose['y']
        return np.sqrt(dx**2 + dy**2)

    # ============================================================================
    # MOVEMENT PATTERNS
    # ============================================================================

    def execute_square_pattern(self):
        """
        Execute 4x4m square pattern - returns to start.
        This tests loop closure: final position should match start position.
        """
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('EXECUTING SQUARE PATTERN (4x4m)')
        self.get_logger().info('This tests LOOP CLOSURE detection')
        self.get_logger().info('='*60 + '\n')

        side_length = 4.0  # meters

        for i in range(4):
            self.get_logger().info(f'\n--- Side {i+1}/4 ---')
            self.move_forward(side_length)
            time.sleep(1)
            self.rotate(np.pi/2)  # 90 degrees left
            time.sleep(1)

        # Report final position vs start
        drift = self.get_distance_from_start()
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'SQUARE PATTERN COMPLETED')
        self.get_logger().info(f'Final position: ({self.current_pose["x"]:.3f}, {self.current_pose["y"]:.3f})')
        self.get_logger().info(f'Start position: ({self.start_pose["x"]:.3f}, {self.start_pose["y"]:.3f})')
        self.get_logger().info(f'Drift from start: {drift:.3f}m')
        self.get_logger().info('='*60 + '\n')

        if drift > 0.3:
            self.get_logger().warn(f'⚠️  SIGNIFICANT DRIFT DETECTED: {drift:.3f}m')
            self.get_logger().warn('⚠️  This indicates odometry errors and/or loop closure failure')

    def execute_long_corridor(self):
        """
        Execute long straight path (10m forward only).
        This tests linear drift accumulation.
        """
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('EXECUTING LONG CORRIDOR PATTERN')
        self.get_logger().info('This tests LINEAR DRIFT accumulation')
        self.get_logger().info('='*60 + '\n')

        corridor_length = 10.0  # meters

        self.get_logger().info('--- Going forward 10m ---')
        self.move_forward(corridor_length)
        time.sleep(1)

        # Report final position
        distance_traveled = self.get_distance_from_start()
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'CORRIDOR PATTERN COMPLETED')
        self.get_logger().info(f'Final position: ({self.current_pose["x"]:.3f}, {self.current_pose["y"]:.3f})')
        self.get_logger().info(f'Start position: ({self.start_pose["x"]:.3f}, {self.start_pose["y"]:.3f})')
        self.get_logger().info(f'Distance traveled: {distance_traveled:.3f}m')
        self.get_logger().info('='*60 + '\n')

    def execute_figure_eight(self):
        """
        Execute figure-8 pattern with two loops.
        Tests complex trajectory tracking.
        """
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('EXECUTING FIGURE-8 PATTERN')
        self.get_logger().info('This tests COMPLEX TRAJECTORY tracking')
        self.get_logger().info('='*60 + '\n')

        # Each loop is made of 4 quarter circles
        segment_length = 1.0

        for loop in range(2):
            self.get_logger().info(f'\n--- Loop {loop+1}/2 ---')
            direction = 1 if loop == 0 else -1  # Right circle, then left circle

            for i in range(8):
                self.move_forward(segment_length)
                self.rotate(direction * np.pi/4)  # 45 degrees
                time.sleep(0.3)

        drift = self.get_distance_from_start()
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'FIGURE-8 PATTERN COMPLETED')
        self.get_logger().info(f'Drift from start: {drift:.3f}m')
        self.get_logger().info('='*60 + '\n')

    def execute_spiral(self):
        """
        Execute outward spiral pattern.
        Tests continuous rotation and increasing radius.
        """
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('EXECUTING SPIRAL PATTERN')
        self.get_logger().info('This tests CONTINUOUS ROTATION')
        self.get_logger().info('='*60 + '\n')

        num_segments = 12
        initial_length = 0.5
        length_increment = 0.3

        for i in range(num_segments):
            segment_length = initial_length + i * length_increment
            self.get_logger().info(f'Segment {i+1}/{num_segments}: {segment_length:.2f}m')
            self.move_forward(segment_length)
            self.rotate(np.pi/3)  # 60 degrees
            time.sleep(0.3)

        drift = self.get_distance_from_start()
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'SPIRAL PATTERN COMPLETED')
        self.get_logger().info(f'Final distance from start: {drift:.3f}m')
        self.get_logger().info('='*60 + '\n')

    # ============================================================================
    # MAIN TEST EXECUTION
    # ============================================================================

    def run_test(self):
        """Execute the selected test pattern"""
        # Wait for odometry
        while self.current_pose is None and rclpy.ok():
            self.get_logger().info('Waiting for odometry...', throttle_duration_sec=2.0)
            rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info('\n🚀 Starting test in 3 seconds...')
        time.sleep(3)

        self.test_started = True

        # Execute pattern
        if self.pattern == MovementPattern.SQUARE:
            self.execute_square_pattern()
        elif self.pattern == MovementPattern.LONG_CORRIDOR:
            self.execute_long_corridor()
        elif self.pattern == MovementPattern.FIGURE_EIGHT:
            self.execute_figure_eight()
        elif self.pattern == MovementPattern.SPIRAL:
            self.execute_spiral()

        self.test_completed = True
        self.save_trajectory()

    def save_trajectory(self):
        """Save trajectory data for analysis"""
        if not self.poses_history:
            return

        import os
        output_dir = f'./test_results/{self.robot_name}'
        os.makedirs(output_dir, exist_ok=True)

        filename = f'{output_dir}/{self.pattern.name.lower()}_trajectory.txt'

        with open(filename, 'w') as f:
            f.write('# timestamp x y theta\n')
            for pose in self.poses_history:
                f.write(f'{pose["timestamp"]:.3f} {pose["x"]:.6f} {pose["y"]:.6f} {pose["theta"]:.6f}\n')

        self.get_logger().info(f'✅ Trajectory saved to: {filename}')


def main(args=None):
    rclpy.init(args=args)

    import sys

    # Parse command line arguments
    robot_name = 'tb3_1'
    pattern = MovementPattern.SQUARE

    if len(sys.argv) > 1:
        robot_name = sys.argv[1]

    if len(sys.argv) > 2:
        pattern_name = sys.argv[2].upper()
        if pattern_name in MovementPattern.__members__:
            pattern = MovementPattern[pattern_name]
        else:
            print(f'Unknown pattern: {sys.argv[2]}')
            print(f'Available patterns: {[p.name for p in MovementPattern]}')
            return

    node = TestRobotController(robot_name, pattern)

    try:
        node.run_test()
        node.get_logger().info('\n' + '='*60)
        node.get_logger().info('✅ TEST COMPLETED SUCCESSFULLY!')
        node.get_logger().info('='*60)

        # Display map if it exists
        map_file = f'./test_results/submaps/{robot_name}/global_map.pcd'
        import os
        if os.path.exists(map_file):
            node.get_logger().info(f'\n📊 Map saved to: {map_file}')
            node.get_logger().info('🔍 Displaying map statistics and visualization...\n')

            # Display map stats and visualization
            try:
                import open3d as o3d
                import numpy as np
                pcd = o3d.io.read_point_cloud(map_file)
                pts = np.asarray(pcd.points)

                print('='*60)
                print('GLOBAL MAP STATISTICS')
                print('='*60)
                print(f'Total points: {len(pts)}')
                print(f'X range: {pts[:,0].min():.2f} to {pts[:,0].max():.2f} m')
                print(f'Y range: {pts[:,1].min():.2f} to {pts[:,1].max():.2f} m')
                print('='*60)

                # Try to visualize (may fail in headless environments)
                try:
                    o3d.visualization.draw_geometries([pcd], window_name=f'Global Map - {pattern.name}')
                except Exception as viz_error:
                    print(f'Note: Visualization window failed (headless environment): {viz_error}')
                    print('Map file saved successfully - view it later with Open3D')

            except Exception as e:
                node.get_logger().error(f'Failed to display map: {e}')
        else:
            node.get_logger().warn(f'⚠️  Map file not found: {map_file}')
            node.get_logger().warn('    Robot may not have moved enough to create submaps (threshold: 2m)')

    except KeyboardInterrupt:
        node.get_logger().info('\n⚠️  Test interrupted by user')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

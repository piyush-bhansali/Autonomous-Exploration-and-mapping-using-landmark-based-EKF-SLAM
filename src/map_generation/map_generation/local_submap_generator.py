#!/usr/bin/env python3
"""
Local Submap Generator - Clean Implementation

Subscribes to laser scans and odometry, generates submaps, and stitches them together.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import os
from threading import Lock

from map_generation.submap_stitcher import SubmapStitcher
from map_generation.ekf_lib import EKF
from map_generation.utils import (
    quaternion_to_yaw,
    yaw_to_quaternion,
    normalize_angle,
    quaternion_to_rotation_matrix
)


class LocalSubmapGenerator(Node):
    """Generates local submaps from laser scans and stitches them together"""

    def __init__(self):
        super().__init__('local_submap_generator')

        # Declare parameters
        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('use_ekf', True)
        self.declare_parameter('distance_threshold', 2.0)
        self.declare_parameter('angle_threshold', 0.5)
        self.declare_parameter('min_scans_per_submap', 50)
        self.declare_parameter('save_directory', './submaps')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('visualize_open3d', False)
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('gpu_device_id', 0)

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.use_ekf = self.get_parameter('use_ekf').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.angle_threshold = self.get_parameter('angle_threshold').value
        self.min_scans = self.get_parameter('min_scans_per_submap').value
        self.save_dir = self.get_parameter('save_directory').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.visualize_open3d = self.get_parameter('visualize_open3d').value
        self.use_gpu = self.get_parameter('use_gpu').value
        self.gpu_device_id = self.get_parameter('gpu_device_id').value

        # Create save directory
        self.save_dir = os.path.join(self.save_dir, self.robot_name)
        os.makedirs(self.save_dir, exist_ok=True)

        # Initialize EKF if needed
        self.ekf_initialized = False
        if self.use_ekf:
            self.ekf = EKF()

        # Initialize stitcher
        self.stitcher = SubmapStitcher(
            voxel_size=self.voxel_size,
            use_gpu=self.use_gpu,
            gpu_device_id=self.gpu_device_id
        )

        # QoS profiles - match what the bridge publishes
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to laser scan with proper QoS
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            sensor_qos
        )
        self.get_logger().info(f'✓ Subscribed to: /{self.robot_name}/scan')

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info(f'✓ Subscribed to: /{self.robot_name}/odom')

        # Subscribe to IMU if using EKF
        if self.use_ekf:
            self.imu_sub = self.create_subscription(
                Imu,
                f'/{self.robot_name}/imu',
                self.imu_callback,
                10
            )
            self.get_logger().info(f'✓ Subscribed to: /{self.robot_name}/imu')

        # State variables
        self.current_pose = None
        self.submap_start_pose = None
        self.current_submap_points = []
        self.submap_id = 0
        self.data_lock = Lock()

        # Debug counters
        self.scan_count = 0
        self.total_scans_received = 0

        # Debug timer
        self.create_timer(5.0, self.debug_callback)

        self.get_logger().info('='*60)
        self.get_logger().info(f'Local Submap Generator Started')
        self.get_logger().info(f'  Robot: {self.robot_name}')
        self.get_logger().info(f'  EKF: {self.use_ekf}')
        self.get_logger().info(f'  Distance threshold: {self.distance_threshold}m')
        self.get_logger().info(f'  Angle threshold: {self.angle_threshold}rad')
        self.get_logger().info(f'  Min scans: {self.min_scans}')
        self.get_logger().info(f'  Save directory: {self.save_dir}')
        self.get_logger().info(f'  GPU: {self.use_gpu}')
        self.get_logger().info('='*60)

    def debug_callback(self):
        """Debug timer to monitor scan reception"""
        self.get_logger().info(
            f'DEBUG: Scans/5s: {self.scan_count} | '
            f'Total scans: {self.total_scans_received} | '
            f'Submap points: {len(self.current_submap_points)} | '
            f'Pose: {self.current_pose is not None}'
        )
        self.scan_count = 0

    def imu_callback(self, msg):
        """IMU callback for EKF"""
        if not self.use_ekf or not self.ekf_initialized:
            return

        # Get angular velocity
        omega = msg.angular_velocity.z
        self.ekf.predict_imu(omega)

    def odom_callback(self, msg):
        """Odometry callback"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = quaternion_to_yaw(qx, qy, qz, qw)

        if self.use_ekf:
            vx = msg.twist.twist.linear.x

            if not self.ekf_initialized:
                self.ekf.initialize(x, y, theta, vx, 0.0)
                self.ekf_initialized = True
                self.get_logger().info(f'✓ EKF initialized at ({x:.2f}, {y:.2f})')
            else:
                self.ekf.update(x, y, theta, vx)

            # Get state from EKF
            state = self.ekf.get_state()
            x = state['x']
            y = state['y']
            theta = state['theta']
            qx, qy, qz, qw = yaw_to_quaternion(theta)

        # Update current pose
        with self.data_lock:
            self.current_pose = {
                'x': x,
                'y': y,
                'z': 0.0,
                'qx': qx,
                'qy': qy,
                'qz': qz,
                'qw': qw,
                'timestamp': self.get_clock().now().nanoseconds
            }

            # Initialize submap start pose
            if self.submap_start_pose is None:
                self.submap_start_pose = self.current_pose.copy()

    def scan_callback(self, msg):
        """Laser scan callback"""
        self.scan_count += 1
        self.total_scans_received += 1

        if self.current_pose is None:
            if self.total_scans_received == 1:
                self.get_logger().warn('⚠ Scan received but no pose yet - waiting for odometry...')
            return

        # Convert scan to world points
        points_world = self.scan_to_world_points(msg, self.current_pose)

        if len(points_world) == 0:
            self.get_logger().warn('⚠ No valid points in scan!', throttle_duration_sec=5.0)
            return

        # Add to current submap
        with self.data_lock:
            self.current_submap_points.extend(points_world.tolist())

        # Log progress
        if self.total_scans_received % 10 == 0:
            self.get_logger().info(
                f'Scan #{self.total_scans_received}: {len(points_world)} pts | '
                f'Submap: {len(self.current_submap_points)} pts'
            )

        # Check if we should create a submap
        if self.should_create_submap():
            self.create_submap()

    def scan_to_world_points(self, scan_msg, pose):
        """Convert laser scan to points in world frame"""
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        ranges = np.array(scan_msg.ranges)

        # Filter valid ranges
        valid = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max) & np.isfinite(ranges)

        if not np.any(valid):
            return np.array([])

        valid_angles = angles[valid]
        valid_ranges = ranges[valid]

        # Convert to Cartesian in robot frame
        x_robot = valid_ranges * np.cos(valid_angles)
        y_robot = valid_ranges * np.sin(valid_angles)
        z_robot = np.zeros_like(x_robot)

        # Transform to world frame
        R = quaternion_to_rotation_matrix(pose['qx'], pose['qy'], pose['qz'], pose['qw'])
        t = np.array([pose['x'], pose['y'], pose['z']])

        points_robot = np.column_stack((x_robot, y_robot, z_robot))
        points_world = (R @ points_robot.T).T + t

        return points_world

    def should_create_submap(self):
        """Check if we should create a new submap"""
        # Need minimum number of points
        min_points = self.min_scans * 100  # Assume ~100 points per scan
        if len(self.current_submap_points) < min_points:
            return False

        if self.submap_start_pose is None or self.current_pose is None:
            return False

        # Calculate distance traveled
        dx = self.current_pose['x'] - self.submap_start_pose['x']
        dy = self.current_pose['y'] - self.submap_start_pose['y']
        distance = np.sqrt(dx**2 + dy**2)

        # Calculate angle change
        theta_start = quaternion_to_yaw(
            self.submap_start_pose['qx'], self.submap_start_pose['qy'],
            self.submap_start_pose['qz'], self.submap_start_pose['qw']
        )
        theta_current = quaternion_to_yaw(
            self.current_pose['qx'], self.current_pose['qy'],
            self.current_pose['qz'], self.current_pose['qw']
        )
        angle_diff = abs(normalize_angle(theta_current - theta_start))

        # Check thresholds
        if distance >= self.distance_threshold:
            self.get_logger().info(f'✓ Distance threshold reached: {distance:.2f}m')
            return True

        if angle_diff >= self.angle_threshold:
            self.get_logger().info(f'✓ Angle threshold reached: {angle_diff:.2f}rad')
            return True

        return False

    def create_submap(self):
        """Create and stitch submap"""
        if len(self.current_submap_points) == 0:
            return

        with self.data_lock:
            points = np.array(self.current_submap_points)

            self.get_logger().info('='*60)
            self.get_logger().info(f'Creating Submap {self.submap_id}')
            self.get_logger().info(f'  Points: {len(points)}')
            self.get_logger().info(f'  Position: ({self.current_pose["x"]:.2f}, {self.current_pose["y"]:.2f})')

            # Add to stitcher
            success = self.stitcher.add_and_stitch_submap(
                points=points,
                submap_id=self.submap_id,
                start_pose=self.submap_start_pose,
                end_pose=self.current_pose
            )

            if success:
                self.submap_id += 1

                # Save global map
                map_file = os.path.join(self.save_dir, 'global_map.pcd')
                self.stitcher.save_global_map(map_file)
                self.get_logger().info(f'✓ Saved global map: {map_file}')

            # Reset for next submap
            self.current_submap_points = []
            self.submap_start_pose = self.current_pose.copy()
            self.get_logger().info('='*60)

    def shutdown(self):
        """Clean shutdown"""
        # Save final map
        if self.submap_id > 0:
            map_file = os.path.join(self.save_dir, 'global_map.pcd')
            self.stitcher.save_global_map(map_file)
            self.get_logger().info(f'✓ Final map saved: {map_file}')


def main(args=None):
    rclpy.init(args=args)
    node = LocalSubmapGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

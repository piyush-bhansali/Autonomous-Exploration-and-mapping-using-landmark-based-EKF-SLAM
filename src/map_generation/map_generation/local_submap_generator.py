#!/usr/bin/env python3
"""
Local Submap Generator - Clean Implementation

Subscribes to laser scans and odometry, generates submaps, and stitches them together.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np
import os
from threading import Lock
import struct

from map_generation.submap_stitcher import SubmapStitcher
from map_generation.ekf_lib import EKF
from map_generation.utils import (
    quaternion_to_yaw,
    yaw_to_quaternion,
    quaternion_to_rotation_matrix
)


class LocalSubmapGenerator(Node):
    """Generates local submaps from laser scans and stitches them together"""

    def __init__(self):
        super().__init__('local_submap_generator')

        # Declare parameters
        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('scans_per_submap', 250)
        self.declare_parameter('min_distance_between_submaps', 1.5)
        self.declare_parameter('min_rotation_between_submaps', 60.0)  # degrees
        self.declare_parameter('save_directory', './submaps')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('feature_method', 'hybrid')
        self.declare_parameter('enable_loop_closure', False)

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.scans_per_submap = self.get_parameter('scans_per_submap').value
        self.min_distance_between_submaps = self.get_parameter('min_distance_between_submaps').value
        self.min_rotation_between_submaps = np.radians(self.get_parameter('min_rotation_between_submaps').value)
        self.save_dir = self.get_parameter('save_directory').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.feature_method = self.get_parameter('feature_method').value
        self.enable_loop_closure = self.get_parameter('enable_loop_closure').value

        # Create save directory
        self.save_dir = os.path.join(self.save_dir, self.robot_name)
        os.makedirs(self.save_dir, exist_ok=True)

        # Initialize EKF (always enabled)
        self.ekf = EKF()
        self.ekf_initialized = False

        # Initialize stitcher (GPU always enabled)
        self.stitcher = SubmapStitcher(
            voxel_size=self.voxel_size,
            feature_extraction_method=self.feature_method,
            enable_loop_closure=self.enable_loop_closure
        )

        # QoS profiles - match what the Gazebo bridge publishes (BEST_EFFORT)
        # This prevents message loss due to QoS incompatibility
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Changed from RELIABLE to match bridge
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

        # Publishers for visualization (always enabled)
        self.current_submap_pub = self.create_publisher(
            PointCloud2,
            f'/{self.robot_name}/current_submap',
            10
        )
        self.global_map_pub = self.create_publisher(
            PointCloud2,
            f'/{self.robot_name}/global_map',
            10
        )
        self.get_logger().info(f'✓ Publishing point clouds to: /{self.robot_name}/current_submap and /{self.robot_name}/global_map')

        # Subscribe to IMU (always enabled for EKF)
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
        self.current_submap_points = []  # List of numpy arrays (one per scan)
        self.submap_id = 0
        self.data_lock = Lock()

        # Scan counting for submap creation
        self.scans_in_current_submap = 0
        self.total_scans_processed = 0

        # Visualization timer - publish current submap every 2 seconds
        self.create_timer(2.0, self.publish_current_submap)

        self.get_logger().info(f'Local Submap Generator initialized for {self.robot_name}')
        self.get_logger().info(f'Scans per submap: {self.scans_per_submap}, Min distance: {self.min_distance_between_submaps}m, Min rotation: {np.degrees(self.min_rotation_between_submaps):.0f}°')

    def publish_current_submap(self):
        """Publish current submap and global map for visualization"""
        # NOTE: Current submap shows odometry-based positioning (before ICP correction)
        # This may show drift artifacts. Disable if you only want to see ICP-corrected global map.

        # Publish current submap being built (DISABLED - shows odometry drift)
        # if len(self.current_submap_points) > 0 and self.submap_start_pose is not None:
        #     with self.data_lock:
        #         points_world = np.vstack(self.current_submap_points)  # Already in world frame
        #
        #     # Points are already in world frame (odom), no transformation needed
        #     pc2_msg = self.numpy_to_pointcloud2(points_world, 'odom', self.get_clock().now().to_msg())
        #     self.current_submap_pub.publish(pc2_msg)

        # ALWAYS publish global map if available (not just on submap creation)
        # Only publish ICP-corrected global map, not pre-ICP submaps
        global_points = self.stitcher.get_global_map_points()
        if global_points is not None and len(global_points) > 0:
            pc2_msg = self.numpy_to_pointcloud2(global_points, 'odom', self.get_clock().now().to_msg())
            self.global_map_pub.publish(pc2_msg)
        # Removed fallback that showed pre-ICP donut artifact

    def numpy_to_pointcloud2(self, points, frame_id, stamp):
        """Convert numpy array to PointCloud2 message"""
        # Create header
        header = Header()
        header.frame_id = frame_id
        header.stamp = stamp

        # Create PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Pack points into binary data
        cloud_data = []
        for point in points:
            cloud_data.append(struct.pack('fff', point[0], point[1], point[2]))

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.data = b''.join(cloud_data)

        return msg

    def imu_callback(self, msg):
        """IMU callback for EKF"""
        if not self.ekf_initialized:
            return

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

        # Update EKF with odometry
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
                'theta': theta,  # Add theta for rotation-based submap creation
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
        if self.current_pose is None:
            self.get_logger().warn('Waiting for odometry to initialize EKF before processing scans...', throttle_duration_sec=5.0)
            return

        # Convert scan to submap-local points (relative to submap start pose)
        # This ensures all scans within a submap are in a consistent coordinate frame
        points_submap = self.scan_to_submap_points(msg, self.current_pose, self.submap_start_pose)

        if len(points_submap) == 0:
            return

        # Add to current submap (store as numpy array, no conversion)
        with self.data_lock:
            self.current_submap_points.append(points_submap)
            self.scans_in_current_submap += 1
            self.total_scans_processed += 1

        # Check if we should create a submap
        if self.should_create_submap():
            self.create_submap()

    def scan_to_world_points_with_lidar_offset(self, scan_msg, pose):
       
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        ranges = np.array(scan_msg.ranges)

        # Filter valid ranges
        valid = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max) & np.isfinite(ranges)
        if not np.any(valid):
            return np.array([])

        valid_angles = angles[valid]
        valid_ranges = ranges[valid]

        # Step 1: Convert scan to Cartesian coordinates in LiDAR frame
        # These are measurements FROM the LiDAR sensor position
        x_lidar = valid_ranges * np.cos(valid_angles)
        y_lidar = valid_ranges * np.sin(valid_angles)
        z_lidar = np.zeros_like(x_lidar)
        points_lidar_frame = np.column_stack((x_lidar, y_lidar, z_lidar))

        # Step 2: Get robot base_link pose in world frame
        R_base_to_world = quaternion_to_rotation_matrix(pose['qx'], pose['qy'], pose['qz'], pose['qw'])
        t_base_world = np.array([pose['x'], pose['y'], pose['z']])

        # Step 3: LiDAR offset from base_link (constant in base_link frame)
        # This is the static transform: base_link → base_scan
        LIDAR_OFFSET_IN_BASE_FRAME = np.array([-0.064, 0.0, 0.121])  # [x, y, z] in meters

        # Step 4: Calculate LiDAR position in world frame
        # The offset rotates with the robot, so we must rotate it first
        t_lidar_world = t_base_world + (R_base_to_world @ LIDAR_OFFSET_IN_BASE_FRAME)

        # Step 5: Transform scan points from LiDAR frame to world frame
        # The scan points are measured in the LiDAR frame, so we:
        # 1. Rotate them by robot orientation (same rotation as base_link)
        # 2. Translate them to LiDAR's world position (not base_link's position!)
        points_world = (R_base_to_world @ points_lidar_frame.T).T + t_lidar_world

        return points_world

    def scan_to_submap_points(self, scan_msg, current_pose, submap_start_pose):
        """
        Convert laser scan to points in WORLD frame using manual transformation

        Uses odometry pose + hard-coded LiDAR offset to transform scan points.
        This is more reliable than TF2 for single-robot scenarios.

        The LiDAR is offset from base_link by:
        - X: -0.064m (6.4cm backward)
        - Y: 0m
        - Z: 0.121m (12.1cm up)

        Args:
            scan_msg: LaserScan message
            current_pose: Current robot pose from EKF-fused odometry
            submap_start_pose: Pose at submap start (unused - kept for API compatibility)

        Returns:
            np.ndarray: Points in WORLD coordinate frame (N x 3)
        """
        # Use manual transformation (more reliable than TF2)
        return self.scan_to_world_points_with_lidar_offset(scan_msg, current_pose)

    def should_create_submap(self):
        """Check if we should create a new submap"""
        # Need valid poses
        if self.submap_start_pose is None or self.current_pose is None:
            return False

        # Calculate LINEAR distance traveled since submap start
        dx = self.current_pose['x'] - self.submap_start_pose['x']
        dy = self.current_pose['y'] - self.submap_start_pose['y']
        distance = np.sqrt(dx**2 + dy**2)

        # Calculate ANGULAR displacement (rotation) since submap start
        # Use atan2 to handle angle wraparound correctly
        dtheta = abs(np.arctan2(
            np.sin(self.current_pose['theta'] - self.submap_start_pose['theta']),
            np.cos(self.current_pose['theta'] - self.submap_start_pose['theta'])
        ))

        # Case 0: Initial submap (bootstrap for navigation)
        # Create first submap after minimal scans even without movement
        # This allows navigation to start planning from initial position
        if self.submap_id == 0 and self.scans_in_current_submap >= 30:
            self.get_logger().info(
                f'Creating initial submap {self.submap_id} (bootstrap): '
                f'{self.scans_in_current_submap} scans, '
                f'{distance:.2f}m, {np.degrees(dtheta):.1f}°'
            )
            return True

        # IMPROVED LOGIC: Prioritize rotation for in-place rotation scenarios
        # Case 1: Sufficient rotation with minimum scans (handles in-place rotation)
        if dtheta >= self.min_rotation_between_submaps and self.scans_in_current_submap >= 30:
            self.get_logger().info(
                f'Creating submap {self.submap_id} (rotation trigger): '
                f'{self.scans_in_current_submap} scans, '
                f'{distance:.2f}m, {np.degrees(dtheta):.1f}°'
            )
            return True

        # Case 2: Normal submap creation (translation-based)
        if self.scans_in_current_submap >= self.scans_per_submap:
            if distance >= self.min_distance_between_submaps:
                self.get_logger().info(
                    f'Creating submap {self.submap_id} (distance trigger): '
                    f'{self.scans_in_current_submap} scans, '
                    f'{distance:.2f}m, {np.degrees(dtheta):.1f}°'
                )
                return True

        return False

    def create_submap(self):
        """Create and stitch submap"""
        if len(self.current_submap_points) == 0:
            return

        with self.data_lock:
            points = np.vstack(self.current_submap_points)  # Efficiently stack numpy arrays
            scan_count = self.scans_in_current_submap
            end_pose = self.current_pose.copy()  # Capture end pose

            self.get_logger().info(f'Creating submap {self.submap_id}: {scan_count} scans, {len(points)} points')
            self.get_logger().info(f'  Start pose: ({self.submap_start_pose["x"]:.3f}, {self.submap_start_pose["y"]:.3f}, θ={np.degrees(self.submap_start_pose["theta"]):.1f}°)')
            self.get_logger().info(f'  End pose:   ({end_pose["x"]:.3f}, {end_pose["y"]:.3f}, θ={np.degrees(end_pose["theta"]):.1f}°)')

            # Add to stitcher
            success = self.stitcher.add_and_stitch_submap(
                points=points,
                submap_id=self.submap_id,
                start_pose=self.submap_start_pose,
                end_pose=end_pose,
                scan_count=scan_count
            )

            if success:
                # Get global map statistics
                global_points = self.stitcher.get_global_map_points()
                global_size = len(global_points) if global_points is not None else 0

                self.get_logger().info(f'✓ Submap {self.submap_id} stitched! Global map now has {global_size} points')

                self.submap_id += 1

                # Save global map periodically (every 5 submaps)
                if self.submap_id % 5 == 0:
                    map_file = os.path.join(self.save_dir, 'global_map.pcd')
                    self.stitcher.save_global_map(map_file)
                    self.get_logger().info(f'Saved global map: {map_file}')

                # Publish global map for visualization and navigation
                if global_points is not None and len(global_points) > 0:
                    pc2_msg = self.numpy_to_pointcloud2(global_points, 'odom', self.get_clock().now().to_msg())
                    self.global_map_pub.publish(pc2_msg)

            # Reset for next submap - use same pose as end_pose for continuity
            self.current_submap_points = []
            self.scans_in_current_submap = 0
            self.submap_start_pose = end_pose

    def shutdown(self):
        """Clean shutdown"""
        if self.submap_id > 0:
            map_file = os.path.join(self.save_dir, 'global_map.pcd')
            self.stitcher.save_global_map(map_file)
            self.get_logger().info(f'Final map saved: {map_file}')


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

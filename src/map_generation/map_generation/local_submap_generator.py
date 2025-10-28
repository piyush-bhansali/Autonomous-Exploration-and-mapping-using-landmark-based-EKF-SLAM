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
        self.declare_parameter('save_directory', './submaps')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('feature_method', 'hybrid')
        self.declare_parameter('enable_loop_closure', False)

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.scans_per_submap = self.get_parameter('scans_per_submap').value
        self.min_distance_between_submaps = self.get_parameter('min_distance_between_submaps').value
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
        self.get_logger().info(f'Scans per submap: {self.scans_per_submap}, Min distance: {self.min_distance_between_submaps}m')

    def publish_current_submap(self):
        """Publish current submap for visualization"""
        if len(self.current_submap_points) > 0:
            with self.data_lock:
                points = np.vstack(self.current_submap_points)  # Stack numpy arrays efficiently
            pc2_msg = self.numpy_to_pointcloud2(points, 'map', self.get_clock().now().to_msg())
            self.current_submap_pub.publish(pc2_msg)

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
            return

        # Convert scan to world points
        points_world = self.scan_to_world_points(msg, self.current_pose)

        if len(points_world) == 0:
            return

        # Add to current submap (store as numpy array, no conversion)
        with self.data_lock:
            self.current_submap_points.append(points_world)
            self.scans_in_current_submap += 1
            self.total_scans_processed += 1

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
        # Check scan count requirement
        if self.scans_in_current_submap < self.scans_per_submap:
            return False

        # Need valid poses
        if self.submap_start_pose is None or self.current_pose is None:
            return False

        # Calculate distance traveled since submap start
        dx = self.current_pose['x'] - self.submap_start_pose['x']
        dy = self.current_pose['y'] - self.submap_start_pose['y']
        distance = np.sqrt(dx**2 + dy**2)

        # Check minimum distance requirement
        if distance < self.min_distance_between_submaps:
            return False

        self.get_logger().info(f'Creating submap {self.submap_id}: {self.scans_in_current_submap} scans, {distance:.2f}m')
        return True

    def create_submap(self):
        """Create and stitch submap"""
        if len(self.current_submap_points) == 0:
            return

        with self.data_lock:
            points = np.vstack(self.current_submap_points)  # Efficiently stack numpy arrays
            scan_count = self.scans_in_current_submap
            end_pose = self.current_pose.copy()  # Capture end pose

            # Add to stitcher
            success = self.stitcher.add_and_stitch_submap(
                points=points,
                submap_id=self.submap_id,
                start_pose=self.submap_start_pose,
                end_pose=end_pose,
                scan_count=scan_count
            )

            if success:
                self.submap_id += 1

                # Save global map
                map_file = os.path.join(self.save_dir, 'global_map.pcd')
                self.stitcher.save_global_map(map_file)

                # Save individual submaps
                submaps_dir = os.path.join(self.save_dir, 'submaps')
                self.stitcher.save_submaps(submaps_dir)

                # Publish global map for visualization
                global_points = self.stitcher.get_global_map_points()
                if global_points is not None and len(global_points) > 0:
                    pc2_msg = self.numpy_to_pointcloud2(global_points, 'map', self.get_clock().now().to_msg())
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

#!/usr/bin/env python3

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
import open3d as o3d
import open3d.core as o3c

from map_generation.submap_stitcher import SubmapStitcher
from map_generation.ekf_lib import EKF
from map_generation.utils import (
    quaternion_to_yaw,
    yaw_to_quaternion,
    quaternion_to_rotation_matrix
)


class LocalSubmapGenerator(Node):
    
    def __init__(self):
        super().__init__('local_submap_generator')

        # Declare parameters
        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('scans_per_submap', 50)
        self.declare_parameter('min_distance_between_submaps', 2.0)  
        self.declare_parameter('save_directory', './submaps')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('feature_method', 'hybrid')
        self.declare_parameter('enable_loop_closure', False)
        self.declare_parameter('enable_scan_to_map_icp', True)

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.scans_per_submap = self.get_parameter('scans_per_submap').value
        self.min_distance_between_submaps = self.get_parameter('min_distance_between_submaps').value
        self.save_dir = self.get_parameter('save_directory').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.feature_method = self.get_parameter('feature_method').value
        self.enable_loop_closure = self.get_parameter('enable_loop_closure').value
        self.enable_scan_to_map_icp = self.get_parameter('enable_scan_to_map_icp').value

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

        # GPU device for scan-to-map ICP
        if o3c.cuda.is_available():
            self.device = o3c.Device("CUDA:0")
            self.get_logger().info("✓ GPU acceleration enabled for scan-to-map ICP")
        else:
            self.device = o3c.Device("CPU:0")
            self.get_logger().info("⚠ GPU not available, using CPU for ICP")

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

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10
        )

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

        # Subscribe to IMU (always enabled for EKF)
        self.imu_sub = self.create_subscription(
            Imu,
            f'/{self.robot_name}/imu',
            self.imu_callback,
            10
        )

        # State variables
        self.current_pose = None
        self.submap_start_pose = None
        self.current_submap_points = []  # List of numpy arrays (one per scan)
        self.submap_id = 0
        self.data_lock = Lock()

        # Scan counting for submap creation
        self.scans_in_current_submap = 0
        self.total_scans_processed = 0

        # Visualization timer - publish global map every 1 second for real-time navigation
        self.create_timer(1.0, self.publish_current_submap)

        self.get_logger().info(f'✓ Local Submap Generator initialized ({self.robot_name}): {self.scans_per_submap} scans/submap, ICP: {"ON" if self.enable_scan_to_map_icp else "OFF"}')

    def publish_current_submap(self):
        
        global_points = self.stitcher.get_global_map_points()
        if global_points is not None and len(global_points) > 0:
            pc2_msg = self.numpy_to_pointcloud2(global_points, 'odom', self.get_clock().now().to_msg())
            self.global_map_pub.publish(pc2_msg)

    def numpy_to_pointcloud2(self, points, frame_id, stamp):
        """Convert numpy array to PointCloud2 message (vectorized for performance)"""
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

        # Vectorized packing (much faster than Python loop)
        # Ensure float32 dtype and flatten to 1D array
        points_flat = points.astype(np.float32).flatten()
        cloud_data = points_flat.tobytes()

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.data = cloud_data

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

    def scan_to_map_icp(self, scan_points_world: np.ndarray, accumulated_points_world: np.ndarray,
                        current_pose: dict) -> tuple:
       
        # Need at least 50 points for reliable ICP
        if len(scan_points_world) < 50 or len(accumulated_points_world) < 100:
            return scan_points_world, None

        # Run ICP on every scan for best accuracy (GPU memory optimized via downsampling)
        try:
            # CRITICAL: Downsample BEFORE transferring to GPU to reduce memory
            # Limit accumulated points to 5000 maximum
            if len(accumulated_points_world) > 5000:
                indices = np.random.choice(len(accumulated_points_world), 5000, replace=False)
                accumulated_points_world = accumulated_points_world[indices]

            # Limit scan points to 2000 maximum (typical scan is ~360 points)
            if len(scan_points_world) > 2000:
                indices = np.random.choice(len(scan_points_world), 2000, replace=False)
                scan_points_world = scan_points_world[indices]

            # Convert to tensor point clouds on GPU
            scan_tensor = o3c.Tensor(scan_points_world.astype(np.float32), dtype=o3c.float32, device=self.device)
            accum_tensor = o3c.Tensor(accumulated_points_world.astype(np.float32), dtype=o3c.float32, device=self.device)

            source_pcd = o3d.t.geometry.PointCloud(self.device)
            source_pcd.point.positions = scan_tensor

            target_pcd = o3d.t.geometry.PointCloud(self.device)
            target_pcd.point.positions = accum_tensor

            # Voxel downsample on GPU (efficient, further reduces points)
            source_pcd = source_pcd.voxel_down_sample(self.voxel_size * 2.0)
            target_pcd = target_pcd.voxel_down_sample(self.voxel_size * 2.0)

            # Identity initial guess (both clouds already in world frame)
            init_transform = o3c.Tensor(np.eye(4), dtype=o3c.float32, device=self.device)

            reg_result = o3d.t.pipelines.registration.icp(
                source=source_pcd,
                target=target_pcd,
                max_correspondence_distance=0.3,  # 30cm tolerance for scan matching
                init_source_to_target=init_transform,
                estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
                criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=20  # Fast convergence for real-time
                )
            )

            transform = reg_result.transformation.cpu().numpy()
            fitness = float(reg_result.fitness)

            # CRITICAL: Explicitly delete GPU tensors to free memory
            del scan_tensor, accum_tensor, source_pcd, target_pcd, init_transform, reg_result

            # Constrain to 2D (safety - no vertical movement)
            transform[2, 0:3] = [0, 0, 1]  # No Z translation, preserve rotation
            transform[2, 3] = 0             # No Z offset
            transform[0:2, 2] = [0, 0]      # No X,Y coupling with Z axis

            # Only accept if fitness is reasonable (>35% for downsampled scan-to-map)
            if fitness > 0.35:
                # Apply correction to ORIGINAL scan points (not downsampled)
                points_homogeneous = np.hstack([scan_points_world, np.ones((len(scan_points_world), 1))])
                points_corrected = (transform @ points_homogeneous.T).T[:, :3]

                # Extract pose correction from transform
                dx = transform[0, 3]
                dy = transform[1, 3]
                dtheta = np.arctan2(transform[1, 0], transform[0, 0])

                pose_correction = {
                    'dx': dx,
                    'dy': dy,
                    'dtheta': dtheta,
                    'fitness': fitness
                }

                return points_corrected, pose_correction
            else:
                return scan_points_world, None

        except Exception as e:
            self.get_logger().warn(f'Scan-to-map ICP failed: {e}', throttle_duration_sec=5.0)
            return scan_points_world, None

    def scan_callback(self, msg):
        """Laser scan callback with scan-to-map ICP correction"""
        if self.current_pose is None:
            self.get_logger().warn('Waiting for odometry to initialize EKF before processing scans...', throttle_duration_sec=5.0)
            return

        # Convert scan to world frame using odometry
        points_world = self.scan_to_submap_points(msg, self.current_pose, self.submap_start_pose)

        if len(points_world) == 0:
            return

        # SCAN-TO-MAP ICP: Refine pose if we have accumulated points
        if self.enable_scan_to_map_icp and len(self.current_submap_points) >= 5:
            with self.data_lock:
                # Stack accumulated points
                accumulated_points = np.vstack(self.current_submap_points)

                # Perform ICP to get correction transform and pose update
                points_world, pose_correction = self.scan_to_map_icp(
                    points_world, accumulated_points, self.current_pose
                )

                # Apply pose correction to current pose estimate
                if pose_correction is not None:
                    self.current_pose['x'] += pose_correction['dx']
                    self.current_pose['y'] += pose_correction['dy']
                    self.current_pose['theta'] += pose_correction['dtheta']
                    self.current_pose['theta'] = np.arctan2(
                        np.sin(self.current_pose['theta']),
                        np.cos(self.current_pose['theta'])
                    )  # Normalize angle

        # Add to current submap (store as numpy array, no conversion)
        with self.data_lock:
            self.current_submap_points.append(points_world)
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
        
        # Use manual transformation (more reliable than TF2)
        return self.scan_to_world_points_with_lidar_offset(scan_msg, current_pose)

    def should_create_submap(self):
        
        # Need valid poses
        if self.submap_start_pose is None or self.current_pose is None:
            return False

        # Calculate LINEAR distance traveled since submap start
        dx = self.current_pose['x'] - self.submap_start_pose['x']
        dy = self.current_pose['y'] - self.submap_start_pose['y']
        distance = np.sqrt(dx**2 + dy**2)

        # Case 1: Initial submap (bootstrap for navigation)
        if self.submap_id == 0 and self.scans_in_current_submap >= 100:
            return True

        # Case 2: Distance-based submap creation (360° LiDAR)
        if distance >= self.min_distance_between_submaps and self.scans_in_current_submap >= self.scans_per_submap:
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

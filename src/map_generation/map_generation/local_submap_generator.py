#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import numpy as np
from threading import Lock, Thread
import os
import time
import open3d as o3d

# Import EKF and utilities
from map_generation.submap_stitcher import SubmapStitcher
from map_generation.ekf_lib import EKF
from map_generation.utils import (
    quaternion_to_yaw,
    yaw_to_quaternion,
    normalize_angle,
    quaternion_to_rotation_matrix
)


class LocalSubmapGenerator(Node):
    """
    Submap generator with live Open3D visualization.
    """

    def __init__(self, robot_name=None):
        super().__init__('local_submap_generator')

        # Parameters
        self.declare_parameter('robot_name', robot_name or 'tb3_1')
        self.declare_parameter('use_ekf', True)
        self.declare_parameter('distance_threshold', 2.0)
        self.declare_parameter('angle_threshold', 0.5)
        self.declare_parameter('min_scans_per_submap', 50)
        self.declare_parameter('save_directory', './submaps')
        self.declare_parameter('voxel_size', 0.05)
        # Visualization and GPU parameters
        self.declare_parameter('visualize_open3d', True)
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('gpu_device_id', 0)

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

        # Initialize EKF if enabled
        if self.use_ekf:
            self.ekf = EKF()
            self.ekf_initialized = False
            self.last_imu_time = None
        else:
            self.ekf = None

        # Initialize stitcher with GPU support
        self.stitcher = SubmapStitcher(
            voxel_size=self.voxel_size,
            icp_max_correspondence_dist=0.5,
            icp_fitness_threshold=0.2,
            use_gpu=self.use_gpu,
            gpu_device_id=self.gpu_device_id
        )

        # --- Open3D Visualization Setup ---
        self.vis = None
        if self.visualize_open3d:
            self.init_open3d()

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            10
        )

        if self.use_ekf:
            self.imu_sub = self.create_subscription(
                Imu,
                f'/{self.robot_name}/imu',
                self.imu_callback,
                10
            )
            self.odom_sub = self.create_subscription(
                Odometry,
                f'/{self.robot_name}/odom',
                self.odom_callback_ekf,
                10
            )
        else:
            self.odom_sub = self.create_subscription(
                Odometry,
                f'/{self.robot_name}/odom',
                self.odom_callback_raw,
                10
            )

        # State variables
        self.current_submap_points = []
        self.current_scan_world = None  # Store current scan in world frame
        self.current_pose = None
        self.submap_start_pose = None
        self.submap_id = 0

        self.data_lock = Lock()

        self.get_logger().info(f'Started: {self.robot_name} | EKF: {self.use_ekf} | Open3D Vis: {self.visualize_open3d} | GPU: {self.use_gpu}')
        self.get_logger().info(f'Thresholds: distance={self.distance_threshold}m, '
                               f'angle={self.angle_threshold}rad, min_scans={self.min_scans}')
        if self.use_gpu:
            self.get_logger().info(f'GPU Device: CUDA:{self.gpu_device_id}')

    # ============================================================================
    # OPEN3D VISUALIZATION
    # ============================================================================

    def init_open3d(self):
        """Initialize the Open3D visualizer."""
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name=f'Global Map - {self.robot_name}', width=1280, height=720)
        
        # Point clouds to be displayed
        self.o3d_global_map = o3d.geometry.PointCloud()
        self.o3d_current_scan = o3d.geometry.PointCloud()
        
        # Add initial empty geometries
        self.vis.add_geometry(self.o3d_global_map)
        self.vis.add_geometry(self.o3d_current_scan)

        # Set render options
        opt = self.vis.get_render_option()
        opt.point_size = 2.0
        opt.background_color = np.asarray([0.15, 0.15, 0.15]) # Dark gray

        # Start visualization thread
        self.vis_thread = Thread(target=self.visualization_thread_func)
        self.vis_thread.start()
        self.get_logger().info('🚀 Open3D visualization started.')

    def visualization_thread_func(self):
        """The function that runs in a separate thread to update the visualization."""
        while rclpy.ok():
            with self.data_lock:
                # Update global map geometry
                if len(self.stitcher.global_map.points) > 0:
                    self.o3d_global_map.points = self.stitcher.global_map.points
                    self.o3d_global_map.paint_uniform_color([0.7, 0.7, 0.7])  # Light gray
                    self.vis.update_geometry(self.o3d_global_map)

                # Update current scan geometry
                if self.current_scan_world is not None and len(self.current_scan_world) > 0:
                    self.o3d_current_scan.points = o3d.utility.Vector3dVector(self.current_scan_world)
                    self.o3d_current_scan.paint_uniform_color([1.0, 0.2, 0.2])  # Red
                    self.vis.update_geometry(self.o3d_current_scan)

            # Poll events and update renderer
            if not self.vis.poll_events():
                break # Window was closed
            self.vis.update_renderer()
            
            time.sleep(0.05) # ~20 Hz update rate

    # ============================================================================
    # CALLBACKS
    # ============================================================================

    def imu_callback(self, msg):
        """IMU callback - EKF prediction"""
        if not self.use_ekf or not self.ekf_initialized:
            return

        with self.data_lock:
            current_time = self.get_clock().now()
            if self.last_imu_time is None:
                self.last_imu_time = current_time
                return

            dt = (current_time - self.last_imu_time).nanoseconds / 1e9
            self.last_imu_time = current_time

            if dt <= 0.0 or dt > 0.5:
                return

            self.ekf.predict(
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                dt
            )
            self.update_pose_from_ekf()

    def odom_callback_ekf(self, msg):
        """Odometry callback - EKF update"""
        with self.data_lock:
            x_meas = msg.pose.pose.position.x
            y_meas = msg.pose.pose.position.y
            theta_meas = quaternion_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            vx_meas = msg.twist.twist.linear.x

            if not self.ekf_initialized:
                self.ekf.initialize(x_meas, y_meas, theta_meas, vx_meas, 0.0)
                self.ekf_initialized = True
                self.last_imu_time = self.get_clock().now()
                self.update_pose_from_ekf()
                self.submap_start_pose = self.current_pose.copy()
                self.get_logger().info(f'EKF initialized at ({x_meas:.2f}, {y_meas:.2f})')
                return

            self.ekf.update(x_meas, y_meas, theta_meas, vx_meas)
            self.update_pose_from_ekf()

    def odom_callback_raw(self, msg):
        """Odometry callback without EKF"""
        with self.data_lock:
            self.current_pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': 0.0,
                'qx': msg.pose.pose.orientation.x,
                'qy': msg.pose.pose.orientation.y,
                'qz': msg.pose.pose.orientation.z,
                'qw': msg.pose.pose.orientation.w,
                'timestamp': self.get_clock().now().nanoseconds
            }

            if self.submap_start_pose is None:
                self.submap_start_pose = self.current_pose.copy()

    def update_pose_from_ekf(self):
        """Update current_pose from EKF state"""
        state = self.ekf.get_state()
        qx, qy, qz, qw = yaw_to_quaternion(state['theta'])

        self.current_pose = {
            'x': state['x'],
            'y': state['y'],
            'z': 0.0,
            'qx': qx,
            'qy': qy,
            'qz': qz,
            'qw': qw,
            'timestamp': self.get_clock().now().nanoseconds
        }

    def scan_callback(self, msg):
        """Laser scan callback"""
        if self.current_pose is None:
            return

        with self.data_lock:
            points_world = self.scan_to_world_points(msg, self.current_pose)

            if len(points_world) == 0:
                return

            # Store current scan for visualization
            self.current_scan_world = points_world

            # Add to current submap
            self.current_submap_points.extend(points_world.tolist())

            if self.should_create_submap():
                self.create_and_stitch_submap()

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
        # Note: Changed condition from 'len(self.current_submap_points) < self.min_scans'
        # to a check on the number of *scans* accumulated, which is what min_scans implies.
        # This requires tracking the number of scans. For simplicity, we'll keep the point-based
        # logic but it's a potential area for refinement. A simple heuristic is used here.
        if len(self.current_submap_points) < self.min_scans * 100: # Heuristic: assume ~100 points/scan
            return False

        if self.submap_start_pose is None:
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

        if distance >= self.distance_threshold:
            self.get_logger().info(f'Creating submap: distance threshold reached ({distance:.2f}m)')
            return True

        if angle_diff >= self.angle_threshold:
            self.get_logger().info(f'Creating submap: angle threshold reached ({angle_diff:.2f}rad)')
            return True

        return False

    def create_and_stitch_submap(self):
        """Create current submap and stitch to global map"""
        if len(self.current_submap_points) == 0:
            return

        points = np.array(self.current_submap_points)

        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'Creating Submap {self.submap_id}')
        self.get_logger().info(f'  Points: {len(points)}')
        self.get_logger().info(f'  Position: ({self.current_pose["x"]:.2f}, {self.current_pose["y"]:.2f})')

        success = self.stitcher.add_and_stitch_submap(
            points=points,
            submap_id=self.submap_id,
            start_pose=self.submap_start_pose,
            end_pose=self.current_pose
        )

        if success:
            self.submap_id += 1

            # Save intermediate result every 5 submaps
            if self.submap_id % 5 == 0:
                intermediate_file = os.path.join(self.save_dir, f'map_after_{self.submap_id}_submaps.pcd')
                self.stitcher.save_global_map(intermediate_file)
                self.get_logger().info(f'Saved intermediate map: {intermediate_file}')

        # Reset for next submap
        self.current_submap_points = []
        self.submap_start_pose = self.current_pose.copy()

        self.get_logger().info(f'{"="*60}\n')

    def on_shutdown(self):
        """Handle node shutdown gracefully."""
        self.get_logger().info('\nShutting down gracefully...')
        
        # Save the final map
        # Process any remaining points
        with self.data_lock:
            if len(self.current_submap_points) >= self.min_scans * 20: # Heuristic
                self.get_logger().info('Creating final submap from remaining points...')
                self.create_and_stitch_submap()

        # Save global map
        output_file = os.path.join(self.save_dir, 'global_map.pcd')
        self.stitcher.save_global_map(output_file)

        # Get and log statistics
        stats = self.stitcher.get_statistics()
        self.get_logger().info(f'\nFinal Statistics:')
        self.get_logger().info(f'  Total submaps: {stats["num_submaps"]}')
        self.get_logger().info(f'  Global map points: {stats["global_map_points"]}')
        self.get_logger().info(f'  Map saved to: {output_file}')
        
        # Close Open3D window
        if self.vis:
            self.vis.destroy_window()

# ============================================================================
# MAIN
# ============================================================================

def main(args=None):
    rclpy.init(args=args)

    import sys
    robot_name = None
    if len(sys.argv) > 1 and not sys.argv[1].startswith('--'):
        robot_name = sys.argv[1]

    node = LocalSubmapGenerator(robot_name)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
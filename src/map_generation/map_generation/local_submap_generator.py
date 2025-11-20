#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
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
    numpy_to_pointcloud2
)
from map_generation.mapping_utils import (
    scan_to_map_icp,
    scan_to_world_points_with_lidar_offset,
    publish_global_map
)


class LocalSubmapGenerator(Node):
    
    def __init__(self):
        super().__init__('local_submap_generator')

        # Declare parameters
        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('scans_per_submap', 50)
        self.declare_parameter('save_directory', './submaps')
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('feature_method', 'hybrid')
        self.declare_parameter('enable_loop_closure', False)
        self.declare_parameter('enable_scan_to_map_icp', True)

        # Get parameters
        self.robot_name = self.get_parameter('robot_name').value
        self.scans_per_submap = self.get_parameter('scans_per_submap').value
        self.save_dir = self.get_parameter('save_directory').value
        self.voxel_size = self.get_parameter('voxel_size').value
        self.feature_method = self.get_parameter('feature_method').value
        self.enable_loop_closure = self.get_parameter('enable_loop_closure').value
        self.enable_scan_to_map_icp = self.get_parameter('enable_scan_to_map_icp').value

        # Create save directory
        self.save_dir = os.path.join(self.save_dir, self.robot_name)
        os.makedirs(self.save_dir, exist_ok=True)

        # Initialize EKF
        self.ekf = EKF()
        self.ekf_initialized = False

        self.stitcher = SubmapStitcher(
            voxel_size=self.voxel_size,
            feature_extraction_method=self.feature_method,
            enable_loop_closure=self.enable_loop_closure
        )

        if o3c.cuda.is_available():
            self.device = o3c.Device("CUDA:0")
            self.get_logger().info("GPU acceleration enabled for scan-to-map ICP")
        else:
            self.device = o3c.Device("CPU:0")
            self.get_logger().info("GPU not available, using CPU for ICP")

        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.imu_sub = self.create_subscription(
            Imu,
            f'/{self.robot_name}/imu',
            self.imu_callback,
            qos_profile_sensor_data
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            qos_profile_system_default
        )

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

        # State variables
        self.current_pose = None
        self.submap_start_pose = None
        self.current_submap_points = []
        self.submap_id = 0
        self.data_lock = Lock()

        # Scan counting for submap creation
        self.scans_in_current_submap = 0
        self.total_scans_processed = 0

        # Timer to publish global map at 1 Hz for visualization
        self.create_timer(1.0, self._publish_global_map_callback)

    def _publish_global_map_callback(self):
        
        global_points = self.stitcher.get_global_map_points()
        publish_global_map(
            global_points,
            self.global_map_pub,
            self.get_clock()
        )

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
                'theta': theta,  
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
        
        if self.current_pose is None:
            self.get_logger().warn('Waiting for odometry to initialize EKF before processing scans...', throttle_duration_sec=5.0)
            return

        # Convert scan to world frame using odometry
        points_world = scan_to_world_points_with_lidar_offset(
            msg,
            self.current_pose
        )

        if len(points_world) == 0:
            return

        # SCAN-TO-MAP ICP: Refine pose if we have accumulated points
        if self.enable_scan_to_map_icp and len(self.current_submap_points) >= 5:
            with self.data_lock:
                # Stack accumulated points
                accumulated_points = np.vstack(self.current_submap_points)

                # Perform ICP to get correction transform and pose update
                points_world, pose_correction = scan_to_map_icp(
                    points_world,
                    accumulated_points,
                    self.device,
                    self.voxel_size,
                    self.get_logger()
                )

                if pose_correction is not None:
                    self.current_pose['x'] += pose_correction['dx']
                    self.current_pose['y'] += pose_correction['dy']
                    self.current_pose['theta'] += pose_correction['dtheta']
                    self.current_pose['theta'] = np.arctan2(
                        np.sin(self.current_pose['theta']),
                        np.cos(self.current_pose['theta'])
                    )

                    qx, qy, qz, qw = yaw_to_quaternion(self.current_pose['theta'])
                    self.current_pose['qx'] = qx
                    self.current_pose['qy'] = qy
                    self.current_pose['qz'] = qz
                    self.current_pose['qw'] = qw

                    self.ekf.state[0] = self.current_pose['x']
                    self.ekf.state[1] = self.current_pose['y']
                    self.ekf.state[2] = self.current_pose['theta']

                    self.ekf.P[0, 0] *= 0.5  
                    self.ekf.P[1, 1] *= 0.5  
                    self.ekf.P[2, 2] *= 0.5  

        with self.data_lock:
            self.current_submap_points.append(points_world)
            self.scans_in_current_submap += 1
            self.total_scans_processed += 1

        if self.should_create_submap():
            self.create_submap()

    def should_create_submap(self):
        
        if self.submap_start_pose is None or self.current_pose is None:
            return False

        if self.scans_in_current_submap >= self.scans_per_submap:
            return True

        return False

    def create_submap(self):

        if len(self.current_submap_points) == 0:
            return

        with self.data_lock:
            points = np.vstack(self.current_submap_points)  
            scan_count = self.scans_in_current_submap
            end_pose = self.current_pose.copy()  

            self.get_logger().info(f'Creating submap {self.submap_id}: {scan_count} scans, {len(points)} points')

            success = self.stitcher.add_and_stitch_submap(
                points=points,
                submap_id=self.submap_id,
                start_pose=self.submap_start_pose,
                end_pose=end_pose,
                scan_count=scan_count
            )

            if success:

                global_points = self.stitcher.get_global_map_points()
                global_size = len(global_points) if global_points is not None else 0

                self.get_logger().info(f'✓ Submap {self.submap_id} stitched! Global map now has {global_size} points')

                self.submap_id += 1

                if self.submap_id % 5 == 0:
                    map_file = os.path.join(self.save_dir, 'global_map.pcd')
                    self.stitcher.save_global_map(map_file)

                # Publish global map for visualization and navigation
                publish_global_map(
                    global_points,
                    self.global_map_pub,
                    self.get_clock()
                )

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

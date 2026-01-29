#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
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
    compute_relative_pose,
    transform_scan_to_relative_frame,
    publish_global_map,
    quaternion_to_rotation_matrix
)
from multi_robot_mapping.qos_profiles import SCAN_QOS, ODOM_QOS, IMU_QOS
import csv
import time


class LocalSubmapGenerator(Node):
    
    def __init__(self):
        super().__init__('local_submap_generator')

        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('save_directory', './submaps')
        self.declare_parameter('enable_loop_closure', True)

        self.robot_name = self.get_parameter('robot_name').value
        self.save_dir = self.get_parameter('save_directory').value
        self.enable_loop_closure = self.get_parameter('enable_loop_closure').value

        self.scans_per_submap = 50      
        self.voxel_size = 0.05         
        self.feature_method = 'hybrid'  
        
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
            SCAN_QOS  
        )

        self.imu_sub = self.create_subscription(
            Imu,
            f'/{self.robot_name}/imu',
            self.imu_callback,
            IMU_QOS  
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            ODOM_QOS
        )

        self.ground_truth_sub = self.create_subscription(
            PoseStamped,
            f'/{self.robot_name}/ground_truth_pose',
            self.ground_truth_callback,
            10
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
        self.ekf_pose_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/ekf_pose',
            10
        )
        self.ekf_path_pub = self.create_publisher(
            Path,
            f'/{self.robot_name}/ekf_path',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # State variables
        self.ekf_path = Path()  
        self.ekf_path.header.frame_id = f'{self.robot_name}/odom'
        self.current_pose = None
        self.submap_start_pose = None
        self.current_submap_points = []
        self.submap_id = 0
        self.data_lock = Lock()

        self.scans_in_current_submap = 0
        self.total_scans_processed = 0

        self.create_timer(1.0, self._publish_global_map_callback)

        self.latest_odom_pose = None

        # Ground truth tracking and CSV logging
        self.ground_truth_pose = None
        self.csv_file_path = os.path.join(self.save_dir, 'ekf_vs_groundtruth.csv')
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'ekf_x', 'ekf_y', 'ekf_theta', 'gt_x', 'gt_y', 'gt_theta', 'pos_error', 'orient_error'])
        self.get_logger().info(f'EKF vs Ground Truth data will be saved to: {self.csv_file_path}')

    def _publish_global_map_callback(self):

        global_points = self.stitcher.get_global_map_points()
        publish_global_map(
            global_points,
            self.global_map_pub,
            self.get_clock(),
            frame_id=f'{self.robot_name}/odom'
        )

    def _publish_ekf_pose(self):
        
        if self.current_pose is None:
            return

        current_time = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = f'{self.robot_name}/odom'
        t.child_frame_id = f'{self.robot_name}/base_footprint'
        t.transform.translation.x = self.current_pose['x']
        t.transform.translation.y = self.current_pose['y']
        t.transform.translation.z = self.current_pose['z']
        t.transform.rotation.x = self.current_pose['qx']
        t.transform.rotation.y = self.current_pose['qy']
        t.transform.rotation.z = self.current_pose['qz']
        t.transform.rotation.w = self.current_pose['qw']
        self.tf_broadcaster.sendTransform(t)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = f'{self.robot_name}/odom'
        pose_msg.pose.position.x = self.current_pose['x']
        pose_msg.pose.position.y = self.current_pose['y']
        pose_msg.pose.position.z = self.current_pose['z']
        pose_msg.pose.orientation.x = self.current_pose['qx']
        pose_msg.pose.orientation.y = self.current_pose['qy']
        pose_msg.pose.orientation.z = self.current_pose['qz']
        pose_msg.pose.orientation.w = self.current_pose['qw']
        self.ekf_pose_pub.publish(pose_msg)

        if len(self.ekf_path.poses) == 0:
            self.ekf_path.poses.append(pose_msg)
        else:
            last_pose = self.ekf_path.poses[-1]
            dx = self.current_pose['x'] - last_pose.pose.position.x
            dy = self.current_pose['y'] - last_pose.pose.position.y
            dist = np.sqrt(dx**2 + dy**2)

            if dist > 0.1:  
                self.ekf_path.poses.append(pose_msg)

        self.ekf_path.header.stamp = current_time
        self.ekf_path_pub.publish(self.ekf_path)

    def imu_callback(self, msg):
        
        if not self.ekf_initialized:
            return

        omega = msg.angular_velocity.z

        self.ekf.predict_imu(omega)

    def odom_callback(self, msg):
        
        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta_odom = quaternion_to_yaw(qx, qy, qz, qw)

        vx = msg.twist.twist.linear.x
        

        self.latest_odom_pose = {
            'x': x_odom,
            'y': y_odom,
            'z': 0.0,  
            'theta': theta_odom,
            'vx': vx,
            'qx': qx,
            'qy': qy,
            'qz': qz,
            'qw': qw
        }

        if not self.ekf_initialized:
            self.ekf.initialize(x_odom, y_odom, theta_odom, vx, 0.0)
            self.ekf_initialized = True
            self.get_logger().info(
                f'EKF initialized at odom pose: ({x_odom:.3f}, {y_odom:.3f}, {np.degrees(theta_odom):.2f}°)'
            )
        else:
            self.ekf.update(x_odom, y_odom, theta_odom, vx_odom=vx)

        state = self.ekf.get_state()
        x = state['x']
        y = state['y']
        theta = state['theta']
        qx, qy, qz, qw = yaw_to_quaternion(theta)

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

            if self.submap_start_pose is None:
                self.submap_start_pose = self.current_pose.copy()

        self._publish_ekf_pose()

    def ground_truth_callback(self, msg):
        """Store ground truth pose for comparison with EKF"""
        x_gt = msg.pose.position.x
        y_gt = msg.pose.position.y

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        theta_gt = quaternion_to_yaw(qx, qy, qz, qw)

        self.ground_truth_pose = {
            'x': x_gt,
            'y': y_gt,
            'theta': theta_gt,
            'timestamp': self.get_clock().now().nanoseconds
        }

        # Log to CSV if EKF is initialized
        self._log_ekf_vs_groundtruth()

    def _log_ekf_vs_groundtruth(self):
        """Log EKF and ground truth data to CSV file"""
        if not self.ekf_initialized or self.ground_truth_pose is None or self.current_pose is None:
            return

        # Calculate errors
        pos_error = np.sqrt(
            (self.current_pose['x'] - self.ground_truth_pose['x'])**2 +
            (self.current_pose['y'] - self.ground_truth_pose['y'])**2
        )

        angle_diff = self.current_pose['theta'] - self.ground_truth_pose['theta']
        orient_error = abs(np.arctan2(np.sin(angle_diff), np.cos(angle_diff)))

        # Write to CSV
        timestamp_sec = self.get_clock().now().nanoseconds / 1e9
        self.csv_writer.writerow([
            timestamp_sec,
            self.current_pose['x'],
            self.current_pose['y'],
            self.current_pose['theta'],
            self.ground_truth_pose['x'],
            self.ground_truth_pose['y'],
            self.ground_truth_pose['theta'],
            pos_error,
            orient_error
        ])
        self.csv_file.flush()  # Ensure data is written immediately

    def _apply_pose_correction(self, dx: float, dy: float, dtheta: float,
                               measurement_type: str, vx_odom=None, base_pose=None):
        """Apply pose correction to EKF and update current pose.

        Args:
            dx: X correction in meters
            dy: Y correction in meters
            dtheta: Theta correction in radians
            measurement_type: Type of measurement ('icp', 'loop_closure', 'odom')
            vx_odom: Optional odometry linear velocity
            base_pose: Base pose dict to apply correction to (defaults to self.current_pose)
        """
        if base_pose is None:
            base_pose = self.current_pose

        corrected_x = base_pose['x'] + dx
        corrected_y = base_pose['y'] + dy
        corrected_theta = base_pose['theta'] + dtheta
        corrected_theta = np.arctan2(np.sin(corrected_theta), np.cos(corrected_theta))

        self.ekf.update(corrected_x, corrected_y, corrected_theta,
                       vx_odom=vx_odom, measurement_type=measurement_type)

        state = self.ekf.get_state()
        qx, qy, qz, qw = yaw_to_quaternion(state['theta'])

        self.current_pose['x'] = state['x']
        self.current_pose['y'] = state['y']
        self.current_pose['theta'] = state['theta']
        self.current_pose['qx'] = qx
        self.current_pose['qy'] = qy
        self.current_pose['qz'] = qz
        self.current_pose['qw'] = qw

        return state

    def scan_callback(self, msg):

        if self.latest_odom_pose is None:
            self.get_logger().warn('Waiting for odometry before processing scans...', throttle_duration_sec=5.0)
            return

        if self.submap_start_pose is None:
            self.submap_start_pose = self.current_pose.copy()

        relative_pose = compute_relative_pose(self.current_pose, self.submap_start_pose)

        points_local = transform_scan_to_relative_frame(msg, relative_pose)

        if len(points_local) == 0:
            return

        if len(self.current_submap_points) >= 5:
            with self.data_lock:
                
                accumulated_local = np.vstack(self.current_submap_points)

                points_local_corrected, pose_correction = scan_to_map_icp(
                    points_local,           
                    accumulated_local,      
                    self.device,
                    self.voxel_size,
                    self.get_logger()
                )

                if pose_correction is not None:
                   
                    R_local_to_world = quaternion_to_rotation_matrix(
                        self.submap_start_pose['qx'], self.submap_start_pose['qy'],
                        self.submap_start_pose['qz'], self.submap_start_pose['qw']
                    )

                    correction_local = np.array([pose_correction['dx'], pose_correction['dy'], 0.0])
                    correction_world = R_local_to_world @ correction_local

                    correction_distance = np.linalg.norm(correction_world[:2])
                    correction_angle = np.abs(pose_correction['dtheta'])

                    if correction_distance < 0.5 and correction_angle < np.radians(45):

                        self._apply_pose_correction(
                            dx=correction_world[0],
                            dy=correction_world[1],
                            dtheta=pose_correction['dtheta'],
                            measurement_type='icp',
                            vx_odom=self.latest_odom_pose['vx'],
                            base_pose=self.latest_odom_pose
                        )

                        self._publish_ekf_pose()

                        points_local = points_local_corrected
                    else:
                        self.get_logger().warn(
                            f'Scan-to-map ICP correction REJECTED (too large): '
                            f'dist={correction_distance:.3f}m, angle={np.degrees(correction_angle):.2f}°',
                            throttle_duration_sec=5.0
                        )

        with self.data_lock:
            self.current_submap_points.append(points_local)
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
            self.get_logger().warn('Cannot create submap: no points collected')
            return

        with self.data_lock:
           
            valid_points = [p for p in self.current_submap_points if len(p) > 0]

            if len(valid_points) == 0:
                self.get_logger().warn('Cannot create submap: all point arrays are empty')
                return

            try:
                points = np.vstack(valid_points)
            except ValueError as e:
                self.get_logger().error(f'Failed to stack submap points: {e}')
                return

            if len(points) < 50:
                self.get_logger().warn(f'Submap has too few points ({len(points)}), skipping')
                return

            scan_count = self.scans_in_current_submap
            end_pose = self.current_pose.copy()


            R = quaternion_to_rotation_matrix(
                self.submap_start_pose['qx'], self.submap_start_pose['qy'],
                self.submap_start_pose['qz'], self.submap_start_pose['qw']
            )
            t = np.array([
                self.submap_start_pose['x'],
                self.submap_start_pose['y'],
                self.submap_start_pose.get('z', 0.0)
            ])

            T_local_to_world = np.eye(4)
            T_local_to_world[0:3, 0:3] = R
            T_local_to_world[0:3, 3] = t

            points_homogeneous = np.column_stack([points, np.ones(len(points))])
            points_world_viz = (T_local_to_world @ points_homogeneous.T).T[:, :3]

            submap_msg = numpy_to_pointcloud2(
                points_world_viz,
                f'{self.robot_name}/odom',
                self.get_clock().now().to_msg()
            )
            self.current_submap_pub.publish(submap_msg)

            success, pose_correction = self.stitcher.integrate_submap_to_global_map(
                points=points, 
                submap_id=self.submap_id,
                start_pose=self.submap_start_pose,
                end_pose=end_pose,
                scan_count=scan_count,
                transformation_matrix=T_local_to_world 
            )

            if success:

                if pose_correction is not None:
                    if 'loop_closure' in pose_correction:
                        lc_correction = pose_correction['loop_closure']

                        state = self._apply_pose_correction(
                            dx=lc_correction['dx'],
                            dy=lc_correction['dy'],
                            dtheta=lc_correction['dtheta'],
                            measurement_type='loop_closure',
                            vx_odom=None
                        )

                        self.get_logger().warn(
                            f' LOOP CLOSURE POSE CORRECTION APPLIED ✓✓✓\n'
                            f'    Submap {lc_correction["submap_id"]} matched with submap {lc_correction["loop_match_id"]}\n'
                            f'    Position correction: dx={lc_correction["dx"]:.3f}m, dy={lc_correction["dy"]:.3f}m\n'
                            f'    Orientation correction: dθ={np.degrees(lc_correction["dtheta"]):.2f}°\n'
                            f'    New robot pose: ({state["x"]:.3f}, {state["y"]:.3f}, {np.degrees(state["theta"]):.2f}°)'
                        )

                    elif pose_correction.get('type') == 'submap_icp':
                        self._apply_pose_correction(
                            dx=pose_correction['dx'],
                            dy=pose_correction['dy'],
                            dtheta=pose_correction['dtheta'],
                            measurement_type='icp',
                            vx_odom=None
                        )

                    self._publish_ekf_pose()

                global_points = self.stitcher.get_global_map_points()
                global_size = len(global_points) if global_points is not None else 0

                self.get_logger().info(f'Submap {self.submap_id} → Global map: {global_size} points')

                self.submap_id += 1

                if self.submap_id % 5 == 0:
                    map_file = os.path.join(self.save_dir, 'global_map.pcd')
                    self.stitcher.save_global_map(map_file)

                publish_global_map(
                    global_points,
                    self.global_map_pub,
                    self.get_clock(),
                    frame_id=f'{self.robot_name}/odom'
                )

            self.current_submap_points = []
            self.scans_in_current_submap = 0
            self.submap_start_pose = end_pose

    def shutdown(self):
        """Clean shutdown"""
        if self.submap_id > 0:
            map_file = os.path.join(self.save_dir, 'global_map.pcd')
            self.stitcher.save_global_map(map_file)
            self.get_logger().info(f'Final map saved: {map_file}')

        # Close CSV file
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f'EKF vs Ground Truth data saved to: {self.csv_file_path}')


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

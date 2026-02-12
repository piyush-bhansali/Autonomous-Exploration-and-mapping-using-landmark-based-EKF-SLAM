#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
import numpy as np
import os
from threading import Lock

from map_generation.submap_stitcher import SubmapStitcher
from map_generation.feature_slam_manager import FeatureSLAMManager
from map_generation.mapping_utils import (
    publish_global_map,
    publish_feature_markers
)
from map_generation.transform_utils import (
    compute_map_to_odom_transform,
    compute_relative_motion_2d,
    normalize_angle,
    quaternion_to_yaw,
    yaw_to_quaternion,
    numpy_to_pointcloud2,
    quaternion_to_rotation_matrix
)
from map_generation.evaluation_utils import (
    ConfidenceTracker,
    compute_ekf_confidence
)
from autonomous_exploration.qos_profiles import (
    SCAN_QOS,
    ODOM_QOS,
    POSE_QOS,
    MAP_QOS,
    PATH_QOS,
    VISUALIZATION_QOS
)


class LocalSubmapGeneratorFeature(Node):
    
    def __init__(self):
        super().__init__('local_submap_generator_feature')

        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('save_directory', './submaps')
        self.robot_name = self.get_parameter('robot_name').value
        self.save_dir = self.get_parameter('save_directory').value

        self.scans_per_submap = 50
        self.voxel_size = 0.05

        self.save_dir = os.path.join(self.save_dir, self.robot_name)
        os.makedirs(self.save_dir, exist_ok=True)

        # Initialize Feature SLAM Manager
        self.slam_manager = FeatureSLAMManager(
            landmark_timeout_scans=50,
            min_observations_for_init=2,
            min_points_per_line=5,
            line_fit_threshold=0.03,
            min_line_length=0.3,
            corner_angle_threshold=50.0,
            max_gap=0.2,
            max_mahalanobis_dist=5.99,
            max_euclidean_dist=3.5,
            wall_gap_tolerance=0.5
        )

        self.stitcher = SubmapStitcher(
            voxel_size=self.voxel_size
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            SCAN_QOS
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            ODOM_QOS
        )

        self.current_submap_pub = self.create_publisher(
            PointCloud2,
            f'/{self.robot_name}/current_submap',
            MAP_QOS
        )
        self.global_map_pub = self.create_publisher(
            PointCloud2,
            f'/{self.robot_name}/global_map',
            MAP_QOS
        )
        self.ekf_pose_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/ekf_pose',
            POSE_QOS
        )
        self.ekf_path_pub = self.create_publisher(
            Path,
            f'/{self.robot_name}/ekf_path',
            PATH_QOS
        )

        self.feature_markers_pub = self.create_publisher(
            MarkerArray,
            f'/{self.robot_name}/scan_features',
            VISUALIZATION_QOS
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'map'  
        self.current_pose = None 
        self.submap_start_pose = None
        self.submap_id = 0
        self.data_lock = Lock()

        self.scans_in_current_submap = 0

        self.create_timer(1.0, self._publish_global_map_callback)

        self.last_odom_pose = None
        self.latest_odom_pose = None
        self.latest_odom_timestamp = None
        self.latest_odom_msg = None

        confidence_csv_path = os.path.join(self.save_dir, 'submap_confidence.csv')
        self.confidence_tracker = ConfidenceTracker(confidence_csv_path)

        self.create_timer(0.1, self._publish_tf_callback)


    def _publish_global_map_callback(self):
        
        global_points = self.stitcher.get_global_map_points()
        publish_global_map(
            global_points,
            self.global_map_pub,
            self.get_clock(),
            frame_id='map'
        )

    def _publish_map_to_odom_tf(self):
        
        if self.current_pose is None:
            self.get_logger().warn(
                'Cannot publish map->odom TF: EKF not initialized',
                throttle_duration_sec=2.0
            )
            return

        if self.latest_odom_pose is None or self.latest_odom_timestamp is None:
            self.get_logger().warn(
                'Cannot publish map->odom TF: No odometry received yet',
                throttle_duration_sec=2.0
            )
            return

        # EKF state in map frame
        ekf_state = (
            self.current_pose['x'],
            self.current_pose['y'],
            self.current_pose['theta']
        )

        # Odometry state in odom frame
        odom_pose = (
            self.latest_odom_pose['x'],
            self.latest_odom_pose['y'],
            self.latest_odom_pose['theta']
        )

        # Compute map->odom transform correction
        x_correction, y_correction, theta_correction = compute_map_to_odom_transform(
            ekf_state, odom_pose
        )

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()  
        t.header.frame_id = 'map'  
        t.child_frame_id = f'{self.robot_name}/odom'  
        t.transform.translation.x = x_correction
        t.transform.translation.y = y_correction
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(theta_correction)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def _publish_tf_callback(self):
        
        if self.current_pose is None:
            return
        
        self._publish_map_to_odom_tf()

    def _publish_ekf_pose(self):
       
        if self.current_pose is None:
            return

        current_time = self.get_clock().now().to_msg()

        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = 'map'  
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

    def odom_callback(self, msg):
                
        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta_odom = quaternion_to_yaw(qx, qy, qz, qw)

        current_odom_pose = (x_odom, y_odom, theta_odom)

        self.latest_odom_pose = {
            'x': x_odom,
            'y': y_odom,
            'z': 0.0,
            'theta': theta_odom,
            'qx': qx,
            'qy': qy,
            'qz': qz,
            'qw': qw
        }
        self.latest_odom_timestamp = msg.header.stamp
        self.latest_odom_msg = msg

        if not self.slam_manager.is_initialized():

            self.slam_manager.initialize_pose(x_odom, y_odom, theta_odom)
            self.last_odom_pose = current_odom_pose

            self.get_logger().info(
                f'EKF SLAM initialized at ({x_odom:.3f}, {y_odom:.3f}, {np.degrees(theta_odom):.1f}°) '
                f'in {msg.header.frame_id}'
            )

            state = self.slam_manager.get_robot_pose()
            qx, qy, qz, qw = yaw_to_quaternion(state['theta'])

            with self.data_lock:
                self.current_pose = {
                    'x': state['x'],
                    'y': state['y'],
                    'z': 0.0,
                    'theta': state['theta'],
                    'qx': qx,
                    'qy': qy,
                    'qz': qz,
                    'qw': qw,
                    'timestamp': self.get_clock().now().nanoseconds
                }

                if self.submap_start_pose is None:
                    self.submap_start_pose = self.current_pose.copy()

            return

        if self.last_odom_pose is not None:

            delta_d, delta_theta = compute_relative_motion_2d(
                current_odom_pose,
                self.last_odom_pose
            )

            self.slam_manager.predict_motion(delta_d, delta_theta)

        self.last_odom_pose = current_odom_pose

        state = self.slam_manager.get_robot_pose()
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

    def _apply_pose_correction(self, dx: float, dy: float, dtheta: float,
                               measurement_type: str, base_pose=None,
                               measurement_covariance: np.ndarray = None):
        
        if base_pose is None:
            base_pose = self.current_pose

        corrected_x = base_pose['x'] + dx
        corrected_y = base_pose['y'] + dy
        corrected_theta = base_pose['theta'] + dtheta
        corrected_theta = normalize_angle(corrected_theta)

        if measurement_covariance is None:
            measurement_covariance = np.diag([0.1, 0.1, 0.05]) ** 2

        self.slam_manager.ekf.update(
            corrected_x, corrected_y, corrected_theta,
            measurement_covariance=measurement_covariance,
            measurement_type=measurement_type
        )

        state = self.slam_manager.get_robot_pose()
        qx, qy, qz, qw = yaw_to_quaternion(state['theta'])

        self.current_pose['x'] = state['x']
        self.current_pose['y'] = state['y']
        self.current_pose['z'] = 0.0
        self.current_pose['theta'] = state['theta']
        self.current_pose['qx'] = qx
        self.current_pose['qy'] = qy
        self.current_pose['qz'] = qz
        self.current_pose['qw'] = qw

        return state

    def _sync_pose_from_ekf(self):
        """Synchronize current_pose from EKF state."""
        state = self.slam_manager.get_robot_pose()
        qx, qy, qz, qw = yaw_to_quaternion(state['theta'])

        with self.data_lock:
            self.current_pose['x'] = state['x']
            self.current_pose['y'] = state['y']
            self.current_pose['z'] = 0.0
            self.current_pose['theta'] = state['theta']
            self.current_pose['qx'] = qx
            self.current_pose['qy'] = qy
            self.current_pose['qz'] = qz
            self.current_pose['qw'] = qw

    def scan_callback(self, msg):

        if self.latest_odom_pose is None:
            self.get_logger().warn('Waiting for odometry before processing scans...', throttle_duration_sec=5.0)
            return

        self._process_scan_feature_mode(msg)

    def _process_scan_feature_mode(self, msg):
        """Process scan in Feature mode: delegate to FeatureSLAMManager."""

        if not self.slam_manager.is_initialized():
            return

        # Process scan through SLAM manager
        stats = self.slam_manager.process_scan(msg)

        if stats['num_features'] == 0:
            return

        # Synchronize pose from EKF to current_pose
        self._sync_pose_from_ekf()

        # Publish feature markers for visualization
        publish_feature_markers(
            stats['observed_features'],
            msg.header.stamp,
            self.feature_markers_pub,
            self.robot_name
        )

        # Log statistics
        self.get_logger().info(
            f"Feature SLAM: {stats['num_features']} features "
            f"({stats['num_corners']} corners, {stats['num_walls']} walls), "
            f"{stats['num_matched']} matched, {stats['num_unmatched']} unmatched, "
            f"{stats['num_landmarks']} EKF landmarks, "
            f"FeatureMap: {stats['num_walls_stored']} walls, {stats['num_corners_stored']} corners",
            throttle_duration_sec=1.0
        )

        # Initialize submap tracking
        if self.submap_start_pose is None:
            self.submap_start_pose = self.current_pose.copy()

        # Increment scan counter
        with self.data_lock:
            self.scans_in_current_submap += 1

        if self.should_create_submap():
            self.create_submap_from_features()

    def should_create_submap(self):
        
        if self.submap_start_pose is None or self.current_pose is None:
            return False

        if self.scans_in_current_submap >= self.scans_per_submap:
            return True

        return False

    def create_submap_from_features(self):
       
        # Generate point cloud from features (5cm spacing)
        points_map_frame = self.slam_manager.generate_point_cloud(spacing=0.05)

        if len(points_map_frame) == 0:
            self.get_logger().warn('Cannot create submap: no features stored in FeatureMap')
            with self.data_lock:
                self.scans_in_current_submap = 0
            self.submap_start_pose = self.current_pose.copy()
            return

        if len(points_map_frame) < 50:
            self.get_logger().warn(
                f'FeatureMap has too few points ({len(points_map_frame)}), skipping submap creation'
            )
            with self.data_lock:
                self.scans_in_current_submap = 0
            self.submap_start_pose = self.current_pose.copy()
            return

        with self.data_lock:
            scan_count = self.scans_in_current_submap
            end_pose = self.current_pose.copy()

            # Compute submap confidence from current EKF state
            confidence_metrics = compute_ekf_confidence(
                self.slam_manager.ekf,
                self.slam_manager.is_initialized(),
                self.get_clock().now().nanoseconds
            )

            # Log confidence metrics for thesis analysis
            self.confidence_tracker.log_confidence(self.submap_id, confidence_metrics)

            # Transform points from map frame to submap local frame
            # Submap local frame is defined by submap_start_pose
            R_world_to_local = quaternion_to_rotation_matrix(
                self.submap_start_pose['qx'], self.submap_start_pose['qy'],
                self.submap_start_pose['qz'], self.submap_start_pose['qw']
            ).T  # Transpose for inverse rotation

            t_world = np.array([
                self.submap_start_pose['x'],
                self.submap_start_pose['y'],
                self.submap_start_pose.get('z', 0.0)
            ])

            # Transform: p_local = R_world_to_local @ (p_map - t_world)
            points_local = (R_world_to_local @ (points_map_frame - t_world).T).T

            # Publish current submap for visualization (in map frame)
            submap_msg = numpy_to_pointcloud2(
                points_map_frame,
                'map',
                self.get_clock().now().to_msg()
            )
            self.current_submap_pub.publish(submap_msg)

            # Create transformation matrix for stitcher (local to world)
            R_local_to_world = R_world_to_local.T
            T_local_to_world = np.eye(4)
            T_local_to_world[0:3, 0:3] = R_local_to_world
            T_local_to_world[0:3, 3] = t_world

            # Integrate into global map
            success, pose_correction = self.stitcher.integrate_submap_to_global_map(
                points=points_local,
                submap_id=self.submap_id,
                start_pose=self.submap_start_pose,
                end_pose=end_pose,
                scan_count=scan_count,
                transformation_matrix=T_local_to_world
            )

            if success:
                # Apply pose correction if provided
                if pose_correction is not None:
                    if pose_correction.get('type') == 'submap_icp':
                        self._apply_pose_correction(
                            dx=pose_correction['dx'],
                            dy=pose_correction['dy'],
                            dtheta=pose_correction['dtheta'],
                            measurement_type='icp',
                            measurement_covariance=pose_correction.get('covariance')
                        )

                    self._publish_ekf_pose()

                global_points = self.stitcher.get_global_map_points()
                global_size = len(global_points) if global_points is not None else 0

                num_walls, num_corners = self.slam_manager.get_feature_map().get_feature_count()
                self.get_logger().info(
                    f'Submap {self.submap_id} created from FeatureMap: '
                    f'{num_walls} walls, {num_corners} corners → '
                    f'{len(points_map_frame)} interpolated points → '
                    f'Global map: {global_size} points'
                )

                self.submap_id += 1

                # Save global map periodically
                if self.submap_id % 5 == 0:
                    map_file = os.path.join(self.save_dir, 'global_map.pcd')
                    self.stitcher.save_global_map(map_file)

                # Publish updated global map
                publish_global_map(
                    global_points,
                    self.global_map_pub,
                    self.get_clock(),
                    frame_id='map'
                )

            # Reset scan counter but KEEP features (they persist across submaps)
            self.scans_in_current_submap = 0
            self.submap_start_pose = end_pose

    def shutdown(self):
        """Clean shutdown"""
        if self.submap_id > 0:
            map_file = os.path.join(self.save_dir, 'global_map.pcd')
            self.stitcher.save_global_map(map_file)
            self.get_logger().info(f'Final map saved: {map_file}')

        # Close confidence tracker
        if hasattr(self, 'confidence_tracker'):
            self.confidence_tracker.close()


def main(args=None):
    rclpy.init(args=args)
    node = LocalSubmapGeneratorFeature()

    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

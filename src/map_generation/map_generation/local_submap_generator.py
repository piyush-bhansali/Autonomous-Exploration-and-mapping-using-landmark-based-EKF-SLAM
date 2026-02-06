#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
import numpy as np
import os
from threading import Lock
import open3d.core as o3c

from map_generation.submap_stitcher import SubmapStitcher
from map_generation.ekf_slam import LandmarkEKFSLAM
from map_generation.landmark_features import LandmarkFeatureExtractor
from map_generation.data_association import associate_landmarks
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
from map_generation.transform_utils import (
    compute_map_to_odom_transform,
    compute_relative_motion_2d,
    normalize_angle
)
from autonomous_exploration.qos_profiles import SCAN_QOS, ODOM_QOS
import csv


class LocalSubmapGenerator(Node):
    
    def __init__(self):
        super().__init__('local_submap_generator')

        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('save_directory', './submaps')
        self.robot_name = self.get_parameter('robot_name').value
        self.save_dir = self.get_parameter('save_directory').value

        self.scans_per_submap = 50      
        self.voxel_size = 0.05         
        self.feature_method = 'hybrid'  
        
        self.save_dir = os.path.join(self.save_dir, self.robot_name)
        os.makedirs(self.save_dir, exist_ok=True)

        self.ekf = LandmarkEKFSLAM(
            max_landmark_range=5.0,           
            landmark_timeout_scans=50,        
            min_observations_for_init=2       
        )
        self.ekf_initialized = False

        
        self.feature_extractor = LandmarkFeatureExtractor(
            min_points_per_line=8,           
            line_fit_threshold=0.03,         
            min_line_length=0.3,             
            corner_angle_threshold=25.0      
        )

        self.stitcher = SubmapStitcher(
            voxel_size=self.voxel_size,
            feature_extraction_method=self.feature_method
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

        self.feature_markers_pub = self.create_publisher(
            MarkerArray,
            f'/{self.robot_name}/scan_features',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.ekf_path = Path()
        self.ekf_path.header.frame_id = 'map'  
        self.current_pose = None 
        self.submap_start_pose = None
        self.current_submap_points = []
        self.submap_id = 0
        self.data_lock = Lock()

        self.scans_in_current_submap = 0

        self.create_timer(1.0, self._publish_global_map_callback)

        self.last_odom_pose = None  
        self.latest_odom_pose = None  
        self.latest_odom_timestamp = None  

        self.ground_truth_pose = None
        self.csv_file_path = os.path.join(self.save_dir, 'ekf_vs_groundtruth.csv')
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'ekf_x', 'ekf_y', 'ekf_theta', 'gt_x', 'gt_y', 'gt_theta', 'pos_error', 'orient_error'])
        self.get_logger().info(f'EKF vs Ground Truth data will be saved to: {self.csv_file_path}')

    def _publish_global_map_callback(self):
        """Publish global map in map frame."""
        global_points = self.stitcher.get_global_map_points()
        publish_global_map(
            global_points,
            self.global_map_pub,
            self.get_clock(),
            frame_id='map'  
        )

    def _publish_map_to_odom_tf(self):
        
        if self.current_pose is None or self.latest_odom_pose is None or self.latest_odom_timestamp is None:
            self.get_logger().warn(
                f'Cannot publish map->odom TF: current_pose={self.current_pose is not None}, '
                f'latest_odom_pose={self.latest_odom_pose is not None}',
                throttle_duration_sec=5.0
            )
            return

       
        ekf_state = (
            self.current_pose['x'],
            self.current_pose['y'],
            self.current_pose['theta']
        )

        
        odom_pose = (
            self.latest_odom_pose['x'],
            self.latest_odom_pose['y'],
            self.latest_odom_pose['theta']
        )

        
        x_correction, y_correction, theta_correction = compute_map_to_odom_transform(
            ekf_state, odom_pose
        )

        
        t = TransformStamped()
        t.header.stamp = self.latest_odom_timestamp
        t.header.frame_id = 'map'  # Parent frame
        t.child_frame_id = f'{self.robot_name}/odom'  # Child frame
        t.transform.translation.x = x_correction
        t.transform.translation.y = y_correction
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(theta_correction)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def _publish_odom_to_base_footprint_tf(self, odom_msg):
        
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = f'{self.robot_name}/odom'
        t.child_frame_id = f'{self.robot_name}/base_footprint'

        # Copy pose directly from odometry message
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z

        t.transform.rotation.x = odom_msg.pose.pose.orientation.x
        t.transform.rotation.y = odom_msg.pose.pose.orientation.y
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

    def _publish_ekf_pose(self):
       
        if self.current_pose is None:
            return

        current_time = self.get_clock().now().to_msg()

        # Publish EKF pose as PoseStamped (in map frame)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = current_time
        pose_msg.header.frame_id = 'map'  # EKF pose is in map frame
        pose_msg.pose.position.x = self.current_pose['x']
        pose_msg.pose.position.y = self.current_pose['y']
        pose_msg.pose.position.z = self.current_pose['z']
        pose_msg.pose.orientation.x = self.current_pose['qx']
        pose_msg.pose.orientation.y = self.current_pose['qy']
        pose_msg.pose.orientation.z = self.current_pose['qz']
        pose_msg.pose.orientation.w = self.current_pose['qw']
        self.ekf_pose_pub.publish(pose_msg)

        # Update EKF path (in map frame)
        if len(self.ekf_path.poses) == 0:
            self.ekf_path.poses.append(pose_msg)
        else:
            last_pose = self.ekf_path.poses[-1]
            dx = self.current_pose['x'] - last_pose.pose.position.x
            dy = self.current_pose['y'] - last_pose.pose.position.y
            dist = np.sqrt(dx**2 + dy**2)

            if dist > 0.1:  # Add point every 10cm
                self.ekf_path.poses.append(pose_msg)

        self.ekf_path.header.stamp = current_time
        self.ekf_path_pub.publish(self.ekf_path)

    def odom_callback(self, msg):
        
        # Extract current odometry pose (in odom frame)
        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta_odom = quaternion_to_yaw(qx, qy, qz, qw)

        # Current pose tuple for computations
        current_odom_pose = (x_odom, y_odom, theta_odom)

        # Store for map→odom transform computation
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

        # Initialize EKF on first odometry reading
        if not self.ekf_initialized:
            # Initialize EKF state in map frame (initially aligned with odom frame)
            self.ekf.initialize(x_odom, y_odom, theta_odom)
            self.ekf_initialized = True
            self.last_odom_pose = current_odom_pose

            self.get_logger().info(
                f'EKF initialized at pose: ({x_odom:.3f}, {y_odom:.3f}, {np.degrees(theta_odom):.2f}°)'
            )

            # Initialize current pose from EKF
            state = self.ekf.get_state()
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

        # Compute relative motion from previous odometry reading
        if self.last_odom_pose is not None:
            # Compute control input [Δd, Δθ]
            delta_d, delta_theta = compute_relative_motion_2d(
                current_odom_pose,
                self.last_odom_pose
            )

            # Apply EKF prediction with motion model
            self.ekf.predict_with_relative_motion(delta_d, delta_theta)

        # Update stored pose
        self.last_odom_pose = current_odom_pose

        # Get updated EKF state (in map frame)
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

        # Publish odom → base_footprint transform (from odometry)
        self._publish_odom_to_base_footprint_tf(msg)

        # Publish map → odom transform and EKF pose
        self._publish_map_to_odom_tf()
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
                               measurement_type: str, base_pose=None,
                               measurement_covariance: np.ndarray = None):
        """
        Apply pose correction to EKF and update current pose.

        This is used for ICP corrections.

        Args:
            dx: X correction in meters
            dy: Y correction in meters
            dtheta: Theta correction in radians
            measurement_type: Type of measurement ('icp')
            base_pose: Base pose dict to apply correction to (defaults to self.current_pose)
        """
        if base_pose is None:
            base_pose = self.current_pose

        # Compute corrected pose in map frame
        corrected_x = base_pose['x'] + dx
        corrected_y = base_pose['y'] + dy
        corrected_theta = base_pose['theta'] + dtheta
        corrected_theta = normalize_angle(corrected_theta)

        # Apply EKF measurement update
        if measurement_covariance is None:
            measurement_covariance = np.diag([0.1, 0.1, 0.05]) ** 2

        self.ekf.update(
            corrected_x, corrected_y, corrected_theta,
            measurement_covariance=measurement_covariance,
            measurement_type=measurement_type
        )

        # Get updated state from EKF
        state = self.ekf.get_state()
        qx, qy, qz, qw = yaw_to_quaternion(state['theta'])

        # Update current pose
        self.current_pose['x'] = state['x']
        self.current_pose['y'] = state['y']
        self.current_pose['theta'] = state['theta']
        self.current_pose['qx'] = qx
        self.current_pose['qy'] = qy
        self.current_pose['qz'] = qz
        self.current_pose['qw'] = qw

        return state

    def _process_scan_features(self, scan_msg):
        """
        Landmark-based SLAM using statistical data association.

        Workflow:
        1. Extract features with covariances (scipy-based)
        2. Associate to existing landmarks (Mahalanobis distance)
        3. Update EKF for matched landmarks
        4. Prune old landmarks (sliding window)
        """
        if not self.ekf_initialized:
            return

        # Extract features with proper covariances
        # Features returned with 'type', 'covariance', and parameterization:
        # - Walls: 'rho', 'alpha' (Hessian Normal Form)
        # - Corners: 'position' [x, y] (Cartesian)
        observed_features = self.feature_extractor.extract_features(scan_msg)

        if len(observed_features) == 0:
            return

        # Data association: match features to landmarks
        matched, unmatched = associate_landmarks(
            observed_features,
            self.ekf,
            max_mahalanobis_dist=5.99,  # Chi-squared 2-DOF, 95% confidence
            max_euclidean_dist=1.0       # Spatial gating (1m)
        )

        # Update EKF with matched landmarks
        for feat_idx, landmark_id in matched:
            feature = observed_features[feat_idx]

            # Extract observation based on feature type
            if feature['type'] == 'wall':
                # Wall in Hessian form (rho, alpha) in robot frame
                z_rho = feature['rho']
                z_alpha = feature['alpha']

                self.ekf.update_landmark_observation(
                    landmark_id=landmark_id,
                    z_x=z_rho,
                    z_y=z_alpha,
                    scan_number=self.ekf.current_scan_number,
                    measurement_covariance=feature['covariance']
                )

            elif feature['type'] == 'corner':
                # Corner in Cartesian (x, y) in robot frame
                z_x, z_y = feature['position']

                # Update EKF with this landmark observation
                self.ekf.update_landmark_observation(
                    landmark_id=landmark_id,
                    z_x=z_x,
                    z_y=z_y,
                    scan_number=self.ekf.current_scan_number,
                    measurement_covariance=feature['covariance']
                )

        # Prune old landmarks (sliding window)
        self.ekf.prune_landmarks(self.ekf.current_scan_number)

        # Update current pose from EKF after landmark updates
        self._sync_pose_from_ekf()

        # Visualize features in RViz
        self._publish_feature_markers(observed_features, scan_msg.header.stamp)

        # Log statistics
        num_landmarks = len(self.ekf.landmarks)
        num_corners = sum(1 for f in observed_features if f['type'] == 'corner')
        num_walls = sum(1 for f in observed_features if f['type'] == 'wall')

        self.get_logger().info(
            f"Landmark SLAM: {len(observed_features)} features "
            f"({num_corners} corners, {num_walls} walls), "
            f"{len(matched)} matched, {len(unmatched)} unmatched, "
            f"{num_landmarks} total landmarks",
            throttle_duration_sec=1.0
        )

    def _sync_pose_from_ekf(self):
        """Synchronize current_pose from EKF state."""
        state = self.ekf.get_state()
        qx, qy, qz, qw = yaw_to_quaternion(state['theta'])

        with self.data_lock:
            self.current_pose['x'] = state['x']
            self.current_pose['y'] = state['y']
            self.current_pose['theta'] = state['theta']
            self.current_pose['qx'] = qx
            self.current_pose['qy'] = qy
            self.current_pose['qz'] = qz
            self.current_pose['qw'] = qw

    def _publish_feature_markers(self, features, scan_timestamp):
        """
        Publish visualization markers for extracted features.

        Corners: Red spheres
        Walls: Blue cylinders

        Args:
            features: List of extracted features
            scan_timestamp: Timestamp from scan message for TF sync
        """
        marker_array = MarkerArray()

        for i, feature in enumerate(features):
            marker = Marker()
            marker.header.frame_id = f'{self.robot_name}/base_scan'  # Features in robot frame
            marker.header.stamp = scan_timestamp  # Use scan timestamp for TF sync
            marker.ns = 'scan_features'
            marker.id = i
            marker.action = Marker.ADD
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 200000000  # 0.2 seconds

            if feature['type'] == 'corner':
                # Corners: Red spheres at Cartesian position
                marker.type = Marker.SPHERE
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red

                # Position from Cartesian coordinates
                marker.pose.position.x = float(feature['position'][0])
                marker.pose.position.y = float(feature['position'][1])
                marker.pose.position.z = 0.0

            elif feature['type'] == 'wall':
                # Walls: Blue cylinders at closest point on wall
                marker.type = Marker.CYLINDER
                marker.scale.x = 0.08  # Diameter
                marker.scale.y = 0.08
                marker.scale.z = 0.2   # Height
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.6)  # Blue

                # Wall in Hessian form: ρ = x*cos(α) + y*sin(α)
                # Closest point on wall to origin: (ρ*cos(α), ρ*sin(α))
                rho = feature['rho']
                alpha = feature['alpha']

                marker.pose.position.x = float(rho * np.cos(alpha))
                marker.pose.position.y = float(rho * np.sin(alpha))
                marker.pose.position.z = 0.0

                # Orient cylinder along wall direction (perpendicular to normal)
                marker.pose.orientation.z = np.sin(alpha / 2.0)
                marker.pose.orientation.w = np.cos(alpha / 2.0)

            marker_array.markers.append(marker)

            # Add text label showing feature info
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'feature_labels'
            text_marker.id = i + 1000  # Offset to avoid ID collision
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = marker.pose.position.x
            text_marker.pose.position.y = marker.pose.position.y
            text_marker.pose.position.z = 0.3  # Above the feature
            text_marker.scale.z = 0.1  # Text size
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White

            # Different labels for walls vs corners
            if feature['type'] == 'wall':
                text_marker.text = f"WALL\n{feature['strength']:.2f}m"
            else:
                text_marker.text = f"CORNER\n{feature['strength']:.0f}°"

            text_marker.lifetime = marker.lifetime

            marker_array.markers.append(text_marker)

        self.feature_markers_pub.publish(marker_array)

    def scan_callback(self, msg):

        if self.latest_odom_pose is None:
            self.get_logger().warn('Waiting for odometry before processing scans...', throttle_duration_sec=5.0)
            return

        # Feature-based EKF update (runs every scan for high-frequency updates)
        self._process_scan_features(msg)

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
                            base_pose=self.current_pose,
                            measurement_covariance=pose_correction.get('covariance')
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


            # Extract features from submap and add new landmarks to EKF
            submap_features = self.feature_extractor.extract_features_from_points(points)
            if len(submap_features) > 0:
                matched, unmatched = associate_landmarks(
                    submap_features,
                    self.ekf,
                    max_mahalanobis_dist=5.99,
                    max_euclidean_dist=1.0
                )

                for feat_idx in unmatched:
                    feature = submap_features[feat_idx]
                    if feature['type'] == 'wall':
                        z_rho = feature['rho']
                        z_alpha = feature['alpha']
                        self.ekf.add_landmark(
                            z_x=z_rho,
                            z_y=z_alpha,
                            feature=feature,
                            scan_number=self.ekf.current_scan_number
                        )
                    elif feature['type'] == 'corner':
                        z_x, z_y = feature['position']
                        self.ekf.add_landmark(
                            z_x=z_x,
                            z_y=z_y,
                            feature=feature,
                            scan_number=self.ekf.current_scan_number
                        )

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
                'map',  # Submaps are in global map frame
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

                self.get_logger().info(f'Submap {self.submap_id} → Global map: {global_size} points')

                self.submap_id += 1

                if self.submap_id % 5 == 0:
                    map_file = os.path.join(self.save_dir, 'global_map.pcd')
                    self.stitcher.save_global_map(map_file)

                publish_global_map(
                    global_points,
                    self.global_map_pub,
                    self.get_clock(),
                    frame_id='map'  # Global map is in map frame
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
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

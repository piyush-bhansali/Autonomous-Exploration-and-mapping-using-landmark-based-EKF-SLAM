#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, JointState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu
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
    transform_scan_to_world_frame,
    publish_global_map
)


class LocalSubmapGenerator(Node):
    
    def __init__(self):
        super().__init__('local_submap_generator')

        # Declare parameters
        # NOTE: use_sim_time is automatically declared by ROS 2, don't declare it again!
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

        # Store latest raw odometry for ICP correction
        self.latest_odom_pose = None  # Will store {'x', 'y', 'theta', 'vx'}

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

        # Use RELIABLE QoS to match the bridge publisher
        odom_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            odom_qos
        )

        # Subscribe to joint_states to monitor wheel velocities
        self.joint_states_sub = self.create_subscription(
            JointState,
            f'/{self.robot_name}/joint_states',
            self.joint_states_callback,
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

        # TF broadcaster for publishing odom -> base_footprint transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # State variables
        self.ekf_path = Path()  # Store EKF trajectory
        self.ekf_path.header.frame_id = f'{self.robot_name}/odom'
        self.current_pose = None
        self.submap_start_pose = None
        self.current_submap_points = []
        self.submap_id = 0
        self.data_lock = Lock()
        self.update_count = 0  # For debug logging rate limiting

        # Scan counting for submap creation
        self.scans_in_current_submap = 0
        self.total_scans_processed = 0

        # Timer to publish global map at 1 Hz for visualization
        self.create_timer(1.0, self._publish_global_map_callback)

        # Timer for debug statistics (2 Hz)
        self.create_timer(0.5, self._debug_statistics_callback)

        # Store last odometry for comparison
        self.last_odom_pose = None

    def _publish_global_map_callback(self):

        global_points = self.stitcher.get_global_map_points()
        publish_global_map(
            global_points,
            self.global_map_pub,
            self.get_clock(),
            frame_id=f'{self.robot_name}/odom'
        )

    def _debug_statistics_callback(self):
        """Periodic debug output comparing EKF vs Odometry"""
        if not self.ekf_initialized or self.last_odom_pose is None:
            return

        ekf_state = self.ekf.get_state()
        ekf_uncertainty = self.ekf.get_uncertainty()

        # Calculate error vs last odometry
        error_x = ekf_state['x'] - self.last_odom_pose['x']
        error_y = ekf_state['y'] - self.last_odom_pose['y']
        error_theta = ekf_state['theta'] - self.last_odom_pose['theta']
        error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))
        error_pos = np.sqrt(error_x**2 + error_y**2)

        self.get_logger().info(
            f'[EKF STATUS] '
            f'Position Error: {error_pos*1000:.1f}mm | '
            f'Angle Error: {np.degrees(error_theta):.2f}° | '
            f'Uncertainty: σ_pos={ekf_uncertainty["sigma_x"]*1000:.1f}mm, σ_θ={np.degrees(ekf_uncertainty["sigma_theta"]):.1f}° | '
            f'EKF vel: {ekf_state["vx"]:.3f} m/s'
        )

    def _publish_ekf_pose(self):
        """Publish current EKF-estimated pose and trajectory."""
        if self.current_pose is None:
            return

        current_time = self.get_clock().now().to_msg()

        # Publish TF transform: {robot_name}/odom -> {robot_name}/base_footprint
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

        # Publish current EKF pose
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

        # Add to trajectory path (sample every 10cm to avoid too many points)
        if len(self.ekf_path.poses) == 0:
            # First pose
            self.ekf_path.poses.append(pose_msg)
        else:
            last_pose = self.ekf_path.poses[-1]
            dx = self.current_pose['x'] - last_pose.pose.position.x
            dy = self.current_pose['y'] - last_pose.pose.position.y
            dist = np.sqrt(dx**2 + dy**2)

            if dist > 0.1:  # Only add if moved >10cm
                self.ekf_path.poses.append(pose_msg)

        # Publish trajectory path
        self.ekf_path.header.stamp = current_time
        self.ekf_path_pub.publish(self.ekf_path)

    def imu_callback(self, msg):
        """IMU callback for EKF"""
        if not self.ekf_initialized:
            return

        omega = msg.angular_velocity.z

        # Log IMU data periodically (every 200th message ~= 1 Hz at 200 Hz IMU)
        if not hasattr(self, 'imu_count'):
            self.imu_count = 0
        self.imu_count += 1

        if self.imu_count % 200 == 0:
            self.get_logger().info(
                f'[IMU DEBUG] ω={omega:.3f} rad/s ({np.degrees(omega):.1f}°/s)'
            )

        self.ekf.predict_imu(omega)

    def joint_states_callback(self, msg):
        """Monitor wheel joint velocities for debugging"""
        if not hasattr(self, 'joint_count'):
            self.joint_count = 0
            self.joint_last_log_time = self.get_clock().now()

        self.joint_count += 1

        # Log wheel velocities every 2 seconds
        current_time = self.get_clock().now()
        time_since_last_log = (current_time - self.joint_last_log_time).nanoseconds / 1e9

        if time_since_last_log >= 2.0:
            # TurtleBot3 has wheel_left_joint and wheel_right_joint
            # Joint names should be in msg.name, velocities in msg.velocity
            try:
                left_idx = msg.name.index('wheel_left_joint')
                right_idx = msg.name.index('wheel_right_joint')

                left_vel = msg.velocity[left_idx]  # rad/s
                right_vel = msg.velocity[right_idx]  # rad/s

                # Calculate expected linear/angular velocities from wheel speeds
                # For differential drive: v = r * (ωL + ωR) / 2, ω = r * (ωR - ωL) / L
                wheel_radius = 0.033  # meters (from SDF)
                wheel_separation = 0.287  # meters (from SDF)

                expected_linear_vel = wheel_radius * (left_vel + right_vel) / 2.0
                expected_angular_vel = wheel_radius * (right_vel - left_vel) / wheel_separation

                self.get_logger().info(
                    f'[WHEEL VEL] left={left_vel:.3f} rad/s, right={right_vel:.3f} rad/s'
                )
                self.get_logger().info(
                    f'[WHEEL CALC] Expected: v={expected_linear_vel:.3f} m/s, '
                    f'ω={expected_angular_vel:.3f} rad/s ({np.degrees(expected_angular_vel):.1f}°/s)'
                )

                self.joint_last_log_time = current_time
            except (ValueError, IndexError) as e:
                if self.joint_count == 1:  # Only log once to avoid spam
                    self.get_logger().warn(
                        f'[WHEEL VEL] Could not find wheel joints: {msg.name}'
                    )

    def odom_callback(self, msg):
        """Odometry callback"""
        # Add detailed odometry debugging
        if not hasattr(self, 'odom_count'):
            self.odom_count = 0
            self.odom_last_log_time = self.get_clock().now()
            self.odom_first_pose = None
            self.odom_last_pose = None
            self.odom_last_timestamp = None

        self.odom_count += 1

        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta_odom = quaternion_to_yaw(qx, qy, qz, qw)

        # Update EKF with odometry
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vtheta = msg.twist.twist.angular.z

        # Track first pose for total distance calculation
        if self.odom_first_pose is None:
            self.odom_first_pose = (x_odom, y_odom, theta_odom)

        # Calculate change since last odometry message
        odom_delta_dist = 0.0
        odom_delta_time = 0.0
        if self.odom_last_pose is not None and self.odom_last_timestamp is not None:
            dx = x_odom - self.odom_last_pose[0]
            dy = y_odom - self.odom_last_pose[1]
            odom_delta_dist = np.sqrt(dx**2 + dy**2)

            # Calculate time delta
            current_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            last_stamp = self.odom_last_timestamp
            odom_delta_time = current_stamp - last_stamp

        # Store current pose for next iteration
        self.odom_last_pose = (x_odom, y_odom, theta_odom)
        self.odom_last_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        # Log raw odometry message every 2 seconds
        current_time = self.get_clock().now()
        time_since_last_log = (current_time - self.odom_last_log_time).nanoseconds / 1e9
        if time_since_last_log >= 2.0:
            # Calculate total distance traveled from start
            total_x_dist = x_odom - self.odom_first_pose[0]
            total_y_dist = y_odom - self.odom_first_pose[1]
            total_odom_dist = np.sqrt(total_x_dist**2 + total_y_dist**2)

            self.get_logger().info(
                f'[ODOM RAW] frame_id={msg.header.frame_id}, '
                f'child_frame_id={msg.child_frame_id}'
            )
            self.get_logger().info(
                f'[ODOM RAW] pos=({x_odom:.3f}, {y_odom:.3f}), '
                f'θ={np.degrees(theta_odom):.2f}°, '
                f'vel=({vx:.3f}, {vy:.3f}) m/s, ω={vtheta:.3f} rad/s'
            )
            self.get_logger().info(
                f'[ODOM RAW] Total dist from start: {total_odom_dist:.3f}m, '
                f'Count: {self.odom_count}'
            )
            if odom_delta_time > 0:
                self.get_logger().info(
                    f'[ODOM RAW] Delta: {odom_delta_dist:.4f}m in {odom_delta_time:.3f}s, '
                    f'avg speed: {odom_delta_dist/odom_delta_time:.3f} m/s'
                )

            self.odom_last_log_time = current_time

        # Store latest raw odometry for scan transformation and ICP correction
        # This is BEFORE EKF filtering, so it's the direct sensor measurement
        self.latest_odom_pose = {
            'x': x_odom,
            'y': y_odom,
            'z': 0.0,  # 2D robot, always at ground level
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
                f'[EKF INIT] Odom: x={x_odom:.3f}, y={y_odom:.3f}, θ={theta_odom:.3f}, vx={vx:.3f}'
            )
            # Note: EKF state is now initialized, will be used below to publish TF
        else:
            # Store EKF state BEFORE update
            ekf_state_before = self.ekf.get_state()

            self.ekf.update(x_odom, y_odom, theta_odom, vx)

            # Get EKF state AFTER update
            ekf_state_after = self.ekf.get_state()

            # Calculate errors
            error_x = ekf_state_after['x'] - x_odom
            error_y = ekf_state_after['y'] - y_odom
            error_theta = ekf_state_after['theta'] - theta_odom
            error_theta = np.arctan2(np.sin(error_theta), np.cos(error_theta))  # Wrap to [-π, π]
            error_pos = np.sqrt(error_x**2 + error_y**2)

            # Log every 50th message (~1 Hz at 50 Hz odometry)
            if self.update_count % 50 == 0:
                self.get_logger().info(
                    f'[EKF DEBUG] '
                    f'Odom: ({x_odom:.3f}, {y_odom:.3f}, {theta_odom:.2f}°) | '
                    f'EKF: ({ekf_state_after["x"]:.3f}, {ekf_state_after["y"]:.3f}, {np.degrees(ekf_state_after["theta"]):.2f}°) | '
                    f'Error: {error_pos:.3f}m, {np.degrees(error_theta):.2f}° | '
                    f'Vel: odom_vx={vx:.3f}, odom_vy={vy:.3f}, odom_ω={vtheta:.3f}'
                )

            self.update_count += 1

        # Always update current_pose and publish TF after initialization OR update

        # Store last odometry for comparison
        self.last_odom_pose = {'x': x_odom, 'y': y_odom, 'theta': theta_odom}

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

        # Publish EKF pose and TF transform after every odometry update
        self._publish_ekf_pose()

    def scan_callback(self, msg):

        # Wait for raw odometry (not EKF) to be available
        if self.latest_odom_pose is None:
            self.get_logger().warn('Waiting for odometry before processing scans...', throttle_duration_sec=5.0)
            return

        # Convert scan to world frame using RAW ODOMETRY (not EKF-filtered pose)
        # This prevents feedback loop: ICP measures error in odometry, not in EKF output
        points_world = transform_scan_to_world_frame(
            msg,
            self.latest_odom_pose
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
                    # Validate correction magnitude before applying
                    correction_distance = np.sqrt(pose_correction['dx']**2 + pose_correction['dy']**2)
                    correction_angle = np.abs(pose_correction['dtheta'])

                    # Stricter thresholds: reject large jumps (>20cm or >15 degrees)
                    if correction_distance < 0.2 and correction_angle < np.radians(15):
                        # CRITICAL FIX: Apply ICP correction to RAW ODOMETRY, not EKF output!
                        # ICP measured the error in the odometry-based scan transformation,
                        # so we correct the odometry and feed it to EKF as a new measurement
                        corrected_x = self.latest_odom_pose['x'] + pose_correction['dx']
                        corrected_y = self.latest_odom_pose['y'] + pose_correction['dy']
                        corrected_theta = self.latest_odom_pose['theta'] + pose_correction['dtheta']
                        corrected_theta = np.arctan2(np.sin(corrected_theta), np.cos(corrected_theta))

                        # Feed ICP-corrected odometry as a high-accuracy measurement to EKF
                        # This tells EKF: "odometry says X, but ICP confirms it should be X + correction"
                        self.ekf.update(corrected_x, corrected_y, corrected_theta,
                                      vx_odom=self.latest_odom_pose['vx'],
                                      measurement_type='icp')

                        # Update current_pose from EKF state (for TF publishing and next iteration)
                        state = self.ekf.get_state()
                        qx, qy, qz, qw = yaw_to_quaternion(state['theta'])

                        self.current_pose['x'] = state['x']
                        self.current_pose['y'] = state['y']
                        self.current_pose['theta'] = state['theta']
                        self.current_pose['qx'] = qx
                        self.current_pose['qy'] = qy
                        self.current_pose['qz'] = qz
                        self.current_pose['qw'] = qw

                        # Publish updated EKF pose after scan-to-map ICP correction
                        self._publish_ekf_pose()

                        self.get_logger().info(
                            f'[ICP CORRECTION] Applied to odometry: '
                            f'dx={pose_correction["dx"]:.3f}m, dy={pose_correction["dy"]:.3f}m, '
                            f'dθ={np.degrees(pose_correction["dtheta"]):.2f}° | '
                            f'Odom: ({self.latest_odom_pose["x"]:.3f}, {self.latest_odom_pose["y"]:.3f}) → '
                            f'Corrected: ({corrected_x:.3f}, {corrected_y:.3f})',
                            throttle_duration_sec=2.0
                        )
                    else:
                        self.get_logger().warn(
                            f'Scan-to-map ICP correction REJECTED (too large): '
                            f'dist={correction_distance:.3f}m, angle={np.degrees(correction_angle):.2f}°',
                            throttle_duration_sec=5.0
                        )

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
            self.get_logger().warn('Cannot create submap: no points collected')
            return

        with self.data_lock:
            # Filter out empty arrays before stacking
            valid_points = [p for p in self.current_submap_points if len(p) > 0]

            if len(valid_points) == 0:
                self.get_logger().warn('Cannot create submap: all point arrays are empty')
                return

            # Stack with error handling
            try:
                points = np.vstack(valid_points)
            except ValueError as e:
                self.get_logger().error(f'Failed to stack submap points: {e}')
                return

            # Validate minimum point count
            if len(points) < 50:
                self.get_logger().warn(f'Submap has too few points ({len(points)}), skipping')
                return

            scan_count = self.scans_in_current_submap
            end_pose = self.current_pose.copy()

            self.get_logger().info(f'Creating submap {self.submap_id}: {scan_count} scans, {len(points)} points')

            # Publish current submap for visualization (yellow in RViz)
            submap_msg = numpy_to_pointcloud2(
                points,
                f'{self.robot_name}/odom',
                self.get_clock().now().to_msg()
            )
            self.current_submap_pub.publish(submap_msg)

            success, pose_correction = self.stitcher.integrate_submap_to_global_map(
                points=points,
                submap_id=self.submap_id,
                start_pose=self.submap_start_pose,
                end_pose=end_pose,
                scan_count=scan_count
            )

            if success:
                # Apply submap-to-map ICP correction to robot pose
                if pose_correction is not None:
                    # Validate correction magnitude (already validated in stitcher, but double-check)
                    correction_distance = np.sqrt(pose_correction['dx']**2 + pose_correction['dy']**2)
                    correction_angle = np.abs(pose_correction['dtheta'])

                    # Submap ICP corrections are typically larger, so use more lenient thresholds
                    if correction_distance < 0.5 and correction_angle < np.radians(20):
                        # Apply correction through EKF update (not direct addition!)
                        corrected_x = self.current_pose['x'] + pose_correction['dx']
                        corrected_y = self.current_pose['y'] + pose_correction['dy']
                        corrected_theta = self.current_pose['theta'] + pose_correction['dtheta']
                        corrected_theta = np.arctan2(np.sin(corrected_theta), np.cos(corrected_theta))

                        # Feed corrected pose as ICP measurement to EKF
                        # ICP provides high-accuracy measurements, so use 'icp' measurement type
                        # The Kalman filter will automatically adjust covariance based on R_icp
                        self.ekf.update(corrected_x, corrected_y, corrected_theta, vx_odom=None, measurement_type='icp')

                        # Update current_pose from EKF state
                        state = self.ekf.get_state()
                        qx, qy, qz, qw = yaw_to_quaternion(state['theta'])

                        self.current_pose['x'] = state['x']
                        self.current_pose['y'] = state['y']
                        self.current_pose['theta'] = state['theta']
                        self.current_pose['qx'] = qx
                        self.current_pose['qy'] = qy
                        self.current_pose['qz'] = qz
                        self.current_pose['qw'] = qw

                        self.get_logger().info(
                            f'Applied submap ICP correction: dx={pose_correction["dx"]:.3f}m, '
                            f'dy={pose_correction["dy"]:.3f}m, dθ={np.degrees(pose_correction["dtheta"]):.2f}°'
                        )

                        # Publish updated EKF pose after submap-to-map ICP correction
                        self._publish_ekf_pose()
                    else:
                        self.get_logger().warn(
                            f'Submap ICP correction REJECTED (too large): '
                            f'dist={correction_distance:.3f}m, angle={np.degrees(correction_angle):.2f}°'
                        )

                global_points = self.stitcher.get_global_map_points()
                global_size = len(global_points) if global_points is not None else 0

                self.get_logger().info(f'✓ Submap {self.submap_id} stitched! Global map now has {global_size} points')

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

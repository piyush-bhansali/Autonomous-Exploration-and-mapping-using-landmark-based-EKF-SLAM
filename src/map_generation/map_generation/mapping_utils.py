#!/usr/bin/env python3

import numpy as np
from typing import Tuple, Optional, Dict
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation
import open3d as o3d
import open3d.core as o3c

from map_generation.transform_utils import quaternion_to_rotation_matrix, numpy_to_pointcloud2


def scan_to_map_icp(
    scan_points_world: np.ndarray,
    accumulated_points_world: np.ndarray,
    device: o3c.Device,
    voxel_size: float,
    logger=None,
    lidar_noise_sigma: float = 0.01
) -> Tuple[np.ndarray, Optional[dict]]:
        
    if len(scan_points_world) < 50 or len(accumulated_points_world) < 50:
        return scan_points_world, None

    try:
        
        scan_tensor = o3c.Tensor(scan_points_world.astype(np.float32), dtype=o3c.float32, device=device)
        accum_tensor = o3c.Tensor(accumulated_points_world.astype(np.float32), dtype=o3c.float32, device=device)

        source_pcd = o3d.t.geometry.PointCloud(device)
        source_pcd.point.positions = scan_tensor

        target_pcd = o3d.t.geometry.PointCloud(device)
        target_pcd.point.positions = accum_tensor

        source_pcd = source_pcd.voxel_down_sample(voxel_size * 2.0)
        target_pcd = target_pcd.voxel_down_sample(voxel_size * 2.0)

        init_transform = o3c.Tensor(np.eye(4), dtype=o3c.float32, device=device)

        reg_result = o3d.t.pipelines.registration.icp(
            source=source_pcd,
            target=target_pcd,
            max_correspondence_distance=0.1,
            init_source_to_target=init_transform,
            estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=20  
            )
        )

        transform = reg_result.transformation.cpu().numpy()
        fitness = float(reg_result.fitness)

        del scan_tensor, accum_tensor, source_pcd, target_pcd, init_transform, reg_result

        transform[2, 0:3] = [0, 0, 1]  
        transform[2, 3] = 0            
        transform[0:2, 2] = [0, 0]      

        if fitness > 0.25:
            points_homogeneous = np.hstack([scan_points_world, np.ones((len(scan_points_world), 1))])
            points_corrected = (transform @ points_homogeneous.T).T[:, :3]

            dx = transform[0, 3]
            dy = transform[1, 3]
            dtheta = np.arctan2(transform[1, 0], transform[0, 0])

            covariance = None
            try:
                # Estimate Hessian-based covariance with fixed LiDAR noise
                # P = sigma^2 * A^(-1) where A = sum(J^T @ J)
                tree = KDTree(accumulated_points_world[:, :2])
                transformed_source = points_corrected[:, :2]
                distances, indices = tree.query(transformed_source)

                max_dist = 0.1
                inlier_mask = distances < max_dist
                if np.count_nonzero(inlier_mask) >= 6:
                    src_inliers = scan_points_world[inlier_mask][:, :2]
                    tgt_inliers = accumulated_points_world[indices[inlier_mask]][:, :2]

                    c = np.cos(dtheta)
                    s = np.sin(dtheta)
                    R = np.array([[c, -s], [s, c]])
                    dR_dtheta = np.array([[-s, -c], [c, -s]])

                    # Compute Hessian (Information Matrix)
                    A = np.zeros((3, 3))

                    for p_s, p_t in zip(src_inliers, tgt_inliers):
                        dp_dtheta = dR_dtheta @ p_s

                        J = np.array([
                            [-1.0, 0.0, -dp_dtheta[0]],
                            [0.0, -1.0, -dp_dtheta[1]]
                        ])

                        # Accumulate Hessian
                        A += J.T @ J

                    # Use fixed LiDAR noise from TurtleBot3 LDS-01 specs (±10mm)
                    # σ = 0.01m → σ² = 0.0001
                    # This avoids the "optimism" problem where more points → smaller σ²
                    sigma2 = lidar_noise_sigma ** 2

                    try:
                        A_inv = np.linalg.inv(A)
                    except np.linalg.LinAlgError:
                        # Use pseudo-inverse if singular (e.g., in featureless corridors)
                        A_inv = np.linalg.pinv(A)

                    covariance = sigma2 * A_inv
            except Exception as e:
                if logger:
                    logger.warn(f'ICP covariance estimation failed: {e}', throttle_duration_sec=5.0)

            pose_correction = {
                'dx': dx,
                'dy': dy,
                'dtheta': dtheta,
                'fitness': fitness,
                'covariance': covariance
            }

            return points_corrected, pose_correction
        else:
            return scan_points_world, None

    except Exception as e:
        if logger:
            logger.warn(f'Scan-to-map ICP failed: {e}', throttle_duration_sec=5.0)
        return scan_points_world, None


def publish_global_map(
    global_points: Optional[np.ndarray],
    publisher,
    clock,
    frame_id
) -> None:
    """
    Publish global map point cloud.

    Args:
        global_points: (N x 3) numpy array of points
        publisher: ROS2 publisher for PointCloud2
        clock: ROS2 clock for timestamp
        frame_id: Frame ID for the point cloud (typically 'map')
    """
    if global_points is not None and len(global_points) > 0:
        pc2_msg = numpy_to_pointcloud2(
            global_points,
            frame_id,
            clock.now().to_msg()
        )
        publisher.publish(pc2_msg)


def compute_relative_pose(current_pose: dict, reference_pose: dict) -> dict:
    
    R_current = quaternion_to_rotation_matrix(
        current_pose['qx'], current_pose['qy'],
        current_pose['qz'], current_pose['qw']
    )
    t_current = np.array([current_pose['x'], current_pose['y'], current_pose.get('z', 0.0)])

    T_world_to_current = np.eye(4)
    T_world_to_current[0:3, 0:3] = R_current
    T_world_to_current[0:3, 3] = t_current

    R_ref = quaternion_to_rotation_matrix(
        reference_pose['qx'], reference_pose['qy'],
        reference_pose['qz'], reference_pose['qw']
    )
    t_ref = np.array([reference_pose['x'], reference_pose['y'], reference_pose.get('z', 0.0)])

    T_world_to_ref = np.eye(4)
    T_world_to_ref[0:3, 0:3] = R_ref
    T_world_to_ref[0:3, 3] = t_ref

    T_ref_to_current = np.linalg.inv(T_world_to_ref) @ T_world_to_current

    relative_position = T_ref_to_current[0:3, 3]

    relative_rotation = Rotation.from_matrix(T_ref_to_current[0:3, 0:3])
    quat = relative_rotation.as_quat()  # Returns [x, y, z, w]

    return {
        'x': relative_position[0],
        'y': relative_position[1],
        'z': relative_position[2],
        'qx': quat[0],
        'qy': quat[1],
        'qz': quat[2],
        'qw': quat[3]
    }


def transform_scan_to_relative_frame(
    scan_msg,
    relative_pose: dict
) -> np.ndarray:
    
    # LiDAR offset from base_link
    lidar_offset = np.array([-0.064, 0.0, 0.121])

    # Parse scan
    angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
    ranges = np.array(scan_msg.ranges)

    # Filter valid points
    valid = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max) & np.isfinite(ranges)
    if not np.any(valid):
        return np.array([])

    valid_angles = angles[valid]
    valid_ranges = ranges[valid]

    # Convert to Cartesian in LiDAR frame
    x_lidar = valid_ranges * np.cos(valid_angles)
    y_lidar = valid_ranges * np.sin(valid_angles)
    z_lidar = np.zeros_like(x_lidar)
    points_lidar_frame = np.column_stack((x_lidar, y_lidar, z_lidar))

    # Build transformation from base to submap-local frame using relative pose
    R_base_to_local = quaternion_to_rotation_matrix(
        relative_pose['qx'], relative_pose['qy'],
        relative_pose['qz'], relative_pose['qw']
    )
    t_base_to_local = np.array([relative_pose['x'], relative_pose['y'], relative_pose['z']])

    # Transform LiDAR offset to local frame
    t_lidar_to_local = t_base_to_local + (R_base_to_local @ lidar_offset)

    # Transform points to submap-local frame
    points_local = (R_base_to_local @ points_lidar_frame.T).T + t_lidar_to_local

    return points_local


def publish_feature_markers(
    features: list,
    scan_timestamp,
    publisher,
    robot_name: str
) -> None:
    """
    Publish visualization markers for detected features (walls and corners).

    Args:
        features: List of feature dictionaries with 'type', position/parameters, etc.
        scan_timestamp: Timestamp from scan message for TF synchronization
        publisher: ROS publisher for MarkerArray messages
        robot_name: Name of the robot (for frame_id)
    """
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import ColorRGBA

    marker_array = MarkerArray()

    for i, feature in enumerate(features):
        marker = Marker()
        marker.header.frame_id = f'{robot_name}/base_scan'  # Features in robot frame
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

    publisher.publish(marker_array)

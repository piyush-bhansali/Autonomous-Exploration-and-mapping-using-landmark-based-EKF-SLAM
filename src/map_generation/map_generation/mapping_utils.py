#!/usr/bin/env python3

import numpy as np
from typing import Tuple, Optional, Dict
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation
import open3d as o3d
import open3d.core as o3c

from map_generation.utils import quaternion_to_rotation_matrix, numpy_to_pointcloud2


def scan_to_map_icp(
    scan_points_world: np.ndarray,
    accumulated_points_world: np.ndarray,
    device: o3c.Device,
    voxel_size: float,
    logger=None
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
                # Estimate covariance from point-to-point residuals (2D)
                # P = sigma^2 * (sum_i J_i^T J_i)^-1
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

                    A = np.zeros((3, 3))
                    rss = 0.0
                    dof = 0

                    for p_s, p_t in zip(src_inliers, tgt_inliers):
                        p_pred = (R @ p_s) + np.array([dx, dy])
                        r = p_t - p_pred

                        dp_dtheta = dR_dtheta @ p_s

                        J = np.array([
                            [-1.0, 0.0, -dp_dtheta[0]],
                            [0.0, -1.0, -dp_dtheta[1]]
                        ])

                        A += J.T @ J
                        rss += float(r.T @ r)
                        dof += 2

                    dof = max(dof - 3, 1)
                    sigma2 = rss / dof

                    try:
                        A_inv = np.linalg.inv(A)
                    except np.linalg.LinAlgError:
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


def is_distinctive_submap(
    geometric_descriptors: np.ndarray,
    min_distinctiveness: float = 0.25,
    min_keypoints: int = 15
) -> Tuple[bool, Dict]:
    
    if len(geometric_descriptors) < min_keypoints:
        return False, {'reason': 'too_few_keypoints', 'num_keypoints': len(geometric_descriptors)}

    linearity = geometric_descriptors[:, 0]
    planarity = geometric_descriptors[:, 1]
    scattering = geometric_descriptors[:, 2]
    avg_dist = geometric_descriptors[:, 3]

    mean_linearity = np.mean(linearity)
    if mean_linearity > 0.7: 
        return False, {
            'reason': 'corridor_detected',
            'mean_linearity': float(mean_linearity)
        }

    scattering_variance = np.var(scattering)
    if scattering_variance < 0.01:  # Very uniform
        return False, {
            'reason': 'low_geometric_diversity',
            'scattering_var': float(scattering_variance)
        }

    feature_variance = np.mean([
        np.var(linearity),
        np.var(planarity),
        np.var(scattering),
        np.var(avg_dist)
    ])

    if feature_variance < min_distinctiveness:
        return False, {
            'reason': 'low_feature_diversity',
            'feature_var': float(feature_variance)
        }

    corner_points = (linearity < 0.3) & (scattering > 0.4)
    corner_ratio = np.sum(corner_points) / len(linearity)

    if corner_ratio < 0.1:  
        return False, {
            'reason': 'no_distinctive_features',
            'corner_ratio': float(corner_ratio)
        }

    return True, {
        'mean_linearity': float(mean_linearity),
        'scattering_var': float(scattering_variance),
        'feature_var': float(feature_variance),
        'corner_ratio': float(corner_ratio),
        'num_keypoints': len(geometric_descriptors)
    }


def match_geometric_features(
    descriptors1: np.ndarray,
    descriptors2: np.ndarray,
    max_distance: float = 0.75
) -> np.ndarray:
    
    if len(descriptors1) == 0 or len(descriptors2) == 0:
        return np.array([])

    tree = KDTree(descriptors2)

    distances, indices = tree.query(descriptors1)

    valid = distances < max_distance

    matches = np.column_stack([
        np.arange(len(descriptors1))[valid],
        indices[valid],
        distances[valid]
    ])

    return matches



def publish_global_map(
    global_points: Optional[np.ndarray],
    publisher,
    clock,
    frame_id
) -> None:

    if global_points is not None and len(global_points) > 0:
        # Publish in odom frame (ICP-corrected points)
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

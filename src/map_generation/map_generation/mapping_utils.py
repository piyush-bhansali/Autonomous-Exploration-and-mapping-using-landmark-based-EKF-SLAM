#!/usr/bin/env python3


import numpy as np
from typing import Tuple, Optional, Dict
from scipy.spatial import KDTree
import open3d as o3d
import open3d.core as o3c

from map_generation.utils import quaternion_to_rotation_matrix, quaternion_to_yaw, numpy_to_pointcloud2

def transform_scan_to_world_frame(
    scan_msg,
    pose: dict
) -> np.ndarray:
    """Transform laser scan to world frame considering LiDAR mounting offset."""

    lidar_offset = np.array([-0.064, 0.0, 0.121])

    angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
    ranges = np.array(scan_msg.ranges)

    valid = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max) & np.isfinite(ranges)
    if not np.any(valid):
        return np.array([])

    valid_angles = angles[valid]
    valid_ranges = ranges[valid]

    x_lidar = valid_ranges * np.cos(valid_angles)
    y_lidar = valid_ranges * np.sin(valid_angles)
    z_lidar = np.zeros_like(x_lidar)
    points_lidar_frame = np.column_stack((x_lidar, y_lidar, z_lidar))

    R_base_to_world = quaternion_to_rotation_matrix(pose['qx'], pose['qy'], pose['qz'], pose['qw'])
    t_base_world = np.array([pose['x'], pose['y'], pose['z']])

    t_lidar_world = t_base_world + (R_base_to_world @ lidar_offset)

    points_world = (R_base_to_world @ points_lidar_frame.T).T + t_lidar_world

    return points_world

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
        if logger:
            logger.warn(f'Scan-to-map ICP failed: {e}', throttle_duration_sec=5.0)
        return scan_points_world, None


def match_scan_context(sc1: np.ndarray, sc2: np.ndarray, num_sectors: int = 60) -> Tuple[float, int]:
    """
    Original Scan Context matching (kept for backward compatibility)
    """
    # Reshape to 2D grid (rings × sectors) if needed
    if sc1.ndim == 1:
        sc1 = sc1.reshape(-1, num_sectors)
    elif sc1.shape[0] == 1:
        sc1 = sc1.reshape(-1, num_sectors)

    if sc2.ndim == 1:
        sc2 = sc2.reshape(-1, num_sectors)
    elif sc2.shape[0] == 1:
        sc2 = sc2.reshape(-1, num_sectors)

    best_sim = -1
    best_shift = 0

    # Try all circular shifts
    for shift in range(num_sectors):
        sc2_shifted = np.roll(sc2, shift, axis=1)

        # Cosine similarity
        sc1_flat = sc1.flatten()
        sc2_flat = sc2_shifted.flatten()

        norm1 = np.linalg.norm(sc1_flat)
        norm2 = np.linalg.norm(sc2_flat)

        if norm1 > 0 and norm2 > 0:
            similarity = np.dot(sc1_flat, sc2_flat) / (norm1 * norm2)

            if similarity > best_sim:
                best_sim = similarity
                best_shift = shift

    return best_sim, best_shift


def match_scan_context_scale_invariant(
    sc1: np.ndarray,
    sc2: np.ndarray,
    metadata1: dict,
    metadata2: dict,
    num_rings: int = 20,
    num_sectors: int = 60
) -> Tuple[float, int]:
    """
    Scale-invariant Scan Context matching

    Matches submaps of different sizes by comparing relative occupancy distributions
    rather than absolute range values.

    Args:
        sc1: Scan Context descriptor 1 (flattened or 2D)
        sc2: Scan Context descriptor 2
        metadata1: Metadata from feature extraction (includes max_range_used, r_99th_percentile)
        metadata2: Metadata from feature extraction
        num_rings: Number of radial bins
        num_sectors: Number of angular bins

    Returns:
        (similarity_score, best_rotation_shift)
    """
    # Reshape to 2D grids
    if sc1.ndim == 1:
        grid1 = sc1.reshape(num_rings, num_sectors)
    elif sc1.shape[0] == 1:
        grid1 = sc1.reshape(num_rings, num_sectors)
    else:
        grid1 = sc1

    if sc2.ndim == 1:
        grid2 = sc2.reshape(num_rings, num_sectors)
    elif sc2.shape[0] == 1:
        grid2 = sc2.reshape(num_rings, num_sectors)
    else:
        grid2 = sc2

    # Compute radial occupancy distribution (scale-invariant!)
    ring_occupancy1 = np.sum(grid1, axis=1)  # Sum over sectors
    ring_occupancy2 = np.sum(grid2, axis=1)

    # Normalize to make distribution comparable
    sum1 = np.sum(ring_occupancy1)
    sum2 = np.sum(ring_occupancy2)

    if sum1 > 0:
        ring_occupancy1 = ring_occupancy1 / sum1
    if sum2 > 0:
        ring_occupancy2 = ring_occupancy2 / sum2

    # Compare distributions (1.0 = identical, 0.0 = completely different)
    radial_similarity = 1.0 - np.mean(np.abs(ring_occupancy1 - ring_occupancy2))

    # Search for best rotation with angular pattern matching
    best_angular_sim = -1
    best_shift = 0

    for shift in range(num_sectors):
        grid2_shifted = np.roll(grid2, shift, axis=1)

        # Cosine similarity on full descriptor
        flat1 = grid1.flatten()
        flat2 = grid2_shifted.flatten()

        norm1 = np.linalg.norm(flat1)
        norm2 = np.linalg.norm(flat2)

        if norm1 > 0 and norm2 > 0:
            angular_sim = np.dot(flat1, flat2) / (norm1 * norm2)

            if angular_sim > best_angular_sim:
                best_angular_sim = angular_sim
                best_shift = shift

    # Combined score: radial distribution (scale-invariant) + angular pattern
    # Radial similarity ensures we match similar structures regardless of size
    # Angular similarity ensures rotation alignment
    combined_similarity = 0.4 * radial_similarity + 0.6 * best_angular_sim

    return combined_similarity, best_shift


def match_scan_context_with_voting(
    sc1: np.ndarray,
    sc2: np.ndarray,
    num_rings: int = 20,
    num_sectors: int = 60,
    threshold: float = 0.5
) -> Tuple[float, int, Optional[np.ndarray]]:
    """
    Majority voting Scan Context matching

    Requires that >50% of bins agree (both occupied or both free)
    to reduce false positives from partial matches like corridor ends.

    Args:
        sc1: Scan Context descriptor 1
        sc2: Scan Context descriptor 2
        num_rings: Number of radial bins
        num_sectors: Number of angular bins
        threshold: Minimum agreement ratio (default 0.5 = 50%)

    Returns:
        (agreement_ratio, best_rotation_shift, agreement_mask)
    """
    # Reshape to 2D grids
    if sc1.ndim == 1:
        grid1 = sc1.reshape(num_rings, num_sectors)
    elif sc1.shape[0] == 1:
        grid1 = sc1.reshape(num_rings, num_sectors)
    else:
        grid1 = sc1

    if sc2.ndim == 1:
        grid2 = sc2.reshape(num_rings, num_sectors)
    elif sc2.shape[0] == 1:
        grid2 = sc2.reshape(num_rings, num_sectors)
    else:
        grid2 = sc2

    best_agreement = 0
    best_shift = 0
    best_mask = None

    for shift in range(num_sectors):
        grid2_shifted = np.roll(grid2, shift, axis=1)

        # Bin-wise agreement (1 if both occupied or both empty)
        both_occupied = (grid1 > 0.5) & (grid2_shifted > 0.5)
        both_empty = (grid1 <= 0.5) & (grid2_shifted <= 0.5)
        agreement_mask = both_occupied | both_empty

        # Count agreement percentage
        agreement_ratio = np.sum(agreement_mask) / agreement_mask.size

        if agreement_ratio > best_agreement:
            best_agreement = agreement_ratio
            best_shift = shift
            best_mask = agreement_mask

    # Return 0 if below threshold (reject match)
    if best_agreement < threshold:
        return 0.0, 0, None

    return best_agreement, best_shift, best_mask


def is_distinctive_submap(
    geometric_descriptors: np.ndarray,
    min_distinctiveness: float = 0.25,
    min_keypoints: int = 15
) -> Tuple[bool, Dict]:
    """
    Check if submap has enough geometric features for reliable loop closure

    Rejects:
    - Long corridors (high linearity, low scattering)
    - Planar walls (high planarity, uniform features)
    - Featureless spaces (low variance in descriptors)

    Args:
        geometric_descriptors: N×4 array [linearity, planarity, scattering, avg_dist]
        min_distinctiveness: Minimum feature variance threshold
        min_keypoints: Minimum number of keypoints required

    Returns:
        (is_distinctive, metrics_dict)
    """
    if len(geometric_descriptors) < min_keypoints:
        return False, {'reason': 'too_few_keypoints', 'num_keypoints': len(geometric_descriptors)}

    # Extract feature statistics
    linearity = geometric_descriptors[:, 0]
    planarity = geometric_descriptors[:, 1]
    scattering = geometric_descriptors[:, 2]
    avg_dist = geometric_descriptors[:, 3]

    # Metric 1: High linearity = corridor (walls parallel to viewing direction)
    mean_linearity = np.mean(linearity)
    if mean_linearity > 0.7:  # 70% linear = corridor walls
        return False, {
            'reason': 'corridor_detected',
            'mean_linearity': float(mean_linearity)
        }

    # Metric 2: Low scattering variance = uniform structure (no distinctive features)
    scattering_variance = np.var(scattering)
    if scattering_variance < 0.01:  # Very uniform
        return False, {
            'reason': 'low_geometric_diversity',
            'scattering_var': float(scattering_variance)
        }

    # Metric 3: Feature diversity (variance across all dimensions)
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

    # Metric 4: Check for distinctive corners/intersections
    # Corners have: low linearity, high scattering
    corner_points = (linearity < 0.3) & (scattering > 0.4)
    corner_ratio = np.sum(corner_points) / len(linearity)

    if corner_ratio < 0.1:  # Less than 10% corners
        return False, {
            'reason': 'no_distinctive_features',
            'corner_ratio': float(corner_ratio)
        }

    # Passed all checks - this is a distinctive submap!
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

    # Build KD-tree for descriptor2
    tree = KDTree(descriptors2)

    # For each descriptor in set1, find nearest in set2
    distances, indices = tree.query(descriptors1)

    # Filter by distance threshold
    valid = distances < max_distance

    matches = np.column_stack([
        np.arange(len(descriptors1))[valid],
        indices[valid],
        distances[valid]
    ])

    return matches



def estimate_transform_from_points(source: np.ndarray, target: np.ndarray) -> np.ndarray:
   
    # Center the points
    source_center = np.mean(source[:, :2], axis=0)
    target_center = np.mean(target[:, :2], axis=0)

    source_centered = source[:, :2] - source_center
    target_centered = target[:, :2] - target_center

    # Compute cross-covariance matrix
    H = source_centered.T @ target_centered

    # SVD
    U, _, Vt = np.linalg.svd(H)
    R_2d = Vt.T @ U.T

    # Ensure proper rotation (det = 1)
    if np.linalg.det(R_2d) < 0:
        Vt[-1, :] *= -1
        R_2d = Vt.T @ U.T

    # Compute translation
    t_2d = target_center - R_2d @ source_center

    # Build 4×4 homogeneous transform
    T = np.eye(4)
    T[0:2, 0:2] = R_2d
    T[0:2, 3] = t_2d

    return T


def estimate_transform_from_poses(pose_from: dict, pose_to: dict) -> np.ndarray:
    
    x_from, y_from = pose_from['x'], pose_from['y']
    x_to, y_to = pose_to['x'], pose_to['y']

    theta_from = quaternion_to_yaw(pose_from['qx'], pose_from['qy'],
                                   pose_from['qz'], pose_from['qw'])
    theta_to = quaternion_to_yaw(pose_to['qx'], pose_to['qy'],
                                 pose_to['qz'], pose_to['qw'])

    # Build 2D transformation matrices
    cos_from, sin_from = np.cos(theta_from), np.sin(theta_from)
    cos_to, sin_to = np.cos(theta_to), np.sin(theta_to)

    T_from = np.eye(4)
    T_from[0:2, 0:2] = [[cos_from, -sin_from], [sin_from, cos_from]]
    T_from[0:2, 3] = [x_from, y_from]

    T_to = np.eye(4)
    T_to[0:2, 0:2] = [[cos_to, -sin_to], [sin_to, cos_to]]
    T_to[0:2, 3] = [x_to, y_to]

    # Relative transformation: T_rel = T_to @ inv(T_from)
    return T_to @ np.linalg.inv(T_from)


def publish_global_map(
    global_points: Optional[np.ndarray],
    publisher,
    clock,
    frame_id: str = 'odom'
) -> None:
    
    if global_points is not None and len(global_points) > 0:
        # Publish in odom frame (ICP-corrected points)
        pc2_msg = numpy_to_pointcloud2(
            global_points,
            frame_id,
            clock.now().to_msg()
        )
        publisher.publish(pc2_msg)

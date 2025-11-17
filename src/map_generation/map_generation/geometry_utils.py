#!/usr/bin/env python3
import numpy as np
from typing import Tuple
from scipy.spatial import KDTree


def match_scan_context(sc1: np.ndarray, sc2: np.ndarray, num_sectors: int = 60) -> Tuple[float, int]:
    
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


def match_geometric_features(descriptors1: np.ndarray,
                             descriptors2: np.ndarray,
                             max_distance: float = 0.75) -> np.ndarray:
    
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


def estimate_transform_from_poses(pose_from: dict, pose_to: dict,
                                  quaternion_to_yaw_func) -> np.ndarray:
    
    # Extract positions and orientations
    x_from, y_from = pose_from['x'], pose_from['y']
    x_to, y_to = pose_to['x'], pose_to['y']

    theta_from = quaternion_to_yaw_func(pose_from['qx'], pose_from['qy'],
                                       pose_from['qz'], pose_from['qw'])
    theta_to = quaternion_to_yaw_func(pose_to['qx'], pose_to['qy'],
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

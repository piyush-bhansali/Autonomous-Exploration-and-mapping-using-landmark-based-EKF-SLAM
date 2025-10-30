#!/usr/bin/env python3
"""
Geometry Utilities for 2D SLAM

Shared geometric functions used across feature extraction, loop closure, and mapping.
Centralized to avoid code duplication.
"""

import numpy as np
from typing import Tuple
from scipy.spatial import KDTree


def match_scan_context(sc1: np.ndarray, sc2: np.ndarray, num_sectors: int = 60) -> Tuple[float, int]:
    """
    Match two Scan Context descriptors with rotation search

    Args:
        sc1: First scan context (1×D or rings×sectors)
        sc2: Second scan context (1×D or rings×sectors)
        num_sectors: Number of angular sectors (default: 60)

    Returns:
        (similarity, best_shift)
        similarity: Cosine similarity (0-1, higher is better)
        best_shift: Best circular shift in sectors
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


def match_geometric_features(descriptors1: np.ndarray,
                             descriptors2: np.ndarray,
                             max_distance: float = 0.75) -> np.ndarray:
    """
    Match geometric features between two descriptor sets using nearest neighbor

    Args:
        descriptors1: First descriptor set (N × D)
        descriptors2: Second descriptor set (M × D)
        max_distance: Maximum descriptor distance for valid match

    Returns:
        matches: Array of (index1, index2, distance) for each match
        Shape: (num_matches, 3)
    """
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
    """
    Estimate 2D rigid transformation from point correspondences using SVD

    This finds the optimal rotation and translation that aligns source points
    to target points in the least-squares sense.

    Args:
        source: Source points (N × 3, where Z is typically 0)
        target: Target points (N × 3, where Z is typically 0)

    Returns:
        T: 4×4 homogeneous transformation matrix
    """
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
    """
    Estimate 2D transformation between two poses (from odometry)

    Args:
        pose_from: Source pose dict with keys: x, y, qx, qy, qz, qw
        pose_to: Target pose dict with keys: x, y, qx, qy, qz, qw
        quaternion_to_yaw_func: Function to convert quaternion to yaw angle

    Returns:
        T_rel: 4×4 homogeneous transformation from pose_from to pose_to
    """
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

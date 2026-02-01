#!/usr/bin/env python3
"""
Transform utilities for SLAM frame computations.

Provides helper functions for computing transforms between map, odom, and robot frames.
"""

import numpy as np


def normalize_angle(theta):
    """
    Normalize angle to [-π, π] range.

    Args:
        theta: Angle in radians

    Returns:
        Normalized angle in [-π, π]
    """
    return np.arctan2(np.sin(theta), np.cos(theta))


def pose_to_transform_matrix(x, y, theta):
    """
    Convert 2D pose to 3x3 homogeneous transformation matrix.

    Args:
        x: X position (meters)
        y: Y position (meters)
        theta: Orientation (radians)

    Returns:
        3x3 numpy array representing the transformation
    """
    c = np.cos(theta)
    s = np.sin(theta)

    T = np.array([
        [c, -s, x],
        [s,  c, y],
        [0,  0, 1]
    ])

    return T


def transform_matrix_to_pose(T):
    """
    Extract 2D pose from 3x3 homogeneous transformation matrix.

    Args:
        T: 3x3 transformation matrix

    Returns:
        Tuple (x, y, theta)
    """
    x = T[0, 2]
    y = T[1, 2]
    theta = np.arctan2(T[1, 0], T[0, 0])

    return (x, y, theta)


def compute_map_to_odom_transform(ekf_state, odom_pose):
    """
    Compute the map → odom transformation.

    This transform represents the drift correction needed to align
    the odometry frame with the globally-consistent map frame.

    Mathematical relationship:
        T_map_to_base = T_map_to_odom * T_odom_to_base

    Therefore:
        T_map_to_odom = T_map_to_base * inv(T_odom_to_base)

    Args:
        ekf_state: Robot pose in map frame (x_map, y_map, theta_map)
        odom_pose: Robot pose in odom frame (x_odom, y_odom, theta_odom)

    Returns:
        Tuple (x_correction, y_correction, theta_correction) representing
        the map → odom transform
    """
    # Extract poses
    x_map, y_map, theta_map = ekf_state
    x_odom, y_odom, theta_odom = odom_pose

    # Build transformation matrices
    T_map_to_base = pose_to_transform_matrix(x_map, y_map, theta_map)
    T_odom_to_base = pose_to_transform_matrix(x_odom, y_odom, theta_odom)

    # Compute inverse: T_base_to_odom
    T_base_to_odom = np.linalg.inv(T_odom_to_base)

    # Compute map → odom transform
    T_map_to_odom = T_map_to_base @ T_base_to_odom

    # Extract pose from transform
    x_correction, y_correction, theta_correction = transform_matrix_to_pose(T_map_to_odom)

    return (x_correction, y_correction, theta_correction)


def compute_relative_motion_2d(pose_current, pose_previous):
    """
    Compute relative motion between two consecutive poses.

    This computes the motion in the robot's frame at the previous pose,
    which is used as the control input [Δd, Δθ] for the EKF prediction.

    Args:
        pose_current: Current pose (x1, y1, theta1)
        pose_previous: Previous pose (x0, y0, theta0)

    Returns:
        Tuple (delta_d, delta_theta) where:
            - delta_d: Distance traveled (meters)
            - delta_theta: Change in heading (radians)
    """
    x1, y1, theta1 = pose_current
    x0, y0, theta0 = pose_previous

    # Compute delta in the odom/world frame
    dx_world = x1 - x0
    dy_world = y1 - y0
    delta_theta = normalize_angle(theta1 - theta0)

    # Transform to robot frame at t=0
    # (Motion is more natural to express in the robot's perspective)
    dx_robot = dx_world * np.cos(theta0) + dy_world * np.sin(theta0)
    dy_robot = -dx_world * np.sin(theta0) + dy_world * np.cos(theta0)

    # Compute distance traveled
    delta_d = np.sqrt(dx_robot**2 + dy_robot**2)

    # Preserve sign: forward motion is positive, backward is negative
    if dx_robot < 0:
        delta_d = -delta_d

    return (delta_d, delta_theta)


def estimate_rigid_transform_2d(source_points, target_points, return_format='4x4'):
    """
    Estimate 2D rigid transformation (rotation + translation) using SVD.

    Uses least-squares fitting to find the optimal rigid transformation
    that aligns source points to target points.

    Algorithm:
        1. Center both point clouds (subtract centroids)
        2. Compute cross-covariance matrix H = source_centered^T @ target_centered
        3. Use SVD to extract optimal rotation: R = V @ U^T
        4. Ensure proper rotation (det(R) = 1, not reflection)
        5. Compute translation: t = target_centroid - R @ source_centroid

    Args:
        source_points: Nx2 or Nx3 numpy array (only first 2 columns used)
        target_points: Nx2 or Nx3 numpy array (only first 2 columns used)
        return_format: Output format
            - '4x4': Return 4x4 homogeneous transformation matrix
            - '2d': Return (R_2x2, t_2x1, dx, dy, dtheta)
            - 'pose': Return (dx, dy, dtheta)

    Returns:
        Depends on return_format:
            - '4x4': 4x4 numpy array with [R t; 0 1] structure
            - '2d': Tuple (R, t, dx, dy, dtheta) where R is 2x2, t is 2x1
            - 'pose': Tuple (dx, dy, dtheta)

    Raises:
        ValueError: If points arrays have incompatible shapes or < 2 points
    """
    # Validate inputs
    if len(source_points) < 2 or len(target_points) < 2:
        raise ValueError("Need at least 2 point correspondences")

    if len(source_points) != len(target_points):
        raise ValueError("Source and target must have same number of points")

    # Extract 2D coordinates (first 2 columns)
    source_2d = source_points[:, :2] if source_points.shape[1] >= 2 else source_points
    target_2d = target_points[:, :2] if target_points.shape[1] >= 2 else target_points

    # Compute centroids
    source_center = np.mean(source_2d, axis=0)
    target_center = np.mean(target_2d, axis=0)

    # Center the points
    source_centered = source_2d - source_center
    target_centered = target_2d - target_center

    # Compute cross-covariance matrix
    H = source_centered.T @ target_centered

    # SVD decomposition
    U, _, Vt = np.linalg.svd(H)

    # Compute rotation matrix
    R_2d = Vt.T @ U.T

    # Ensure proper rotation (det(R) = 1, not reflection det(R) = -1)
    if np.linalg.det(R_2d) < 0:
        Vt[-1, :] *= -1
        R_2d = Vt.T @ U.T

    # Compute translation
    t_2d = target_center - R_2d @ source_center

    # Extract translation components
    dx = t_2d[0]
    dy = t_2d[1]

    # Extract rotation angle
    dtheta = np.arctan2(R_2d[1, 0], R_2d[0, 0])

    # Return in requested format
    if return_format == '4x4':
        # Build 4x4 homogeneous transformation matrix
        T = np.eye(4)
        T[0:2, 0:2] = R_2d
        T[0:2, 3] = t_2d
        return T

    elif return_format == '2d':
        # Return 2D rotation matrix, translation vector, and pose
        return R_2d, t_2d, dx, dy, dtheta

    elif return_format == 'pose':
        # Return just the pose (dx, dy, dtheta)
        return dx, dy, dtheta

    else:
        raise ValueError(f"Unknown return_format: {return_format}. Use '4x4', '2d', or 'pose'")

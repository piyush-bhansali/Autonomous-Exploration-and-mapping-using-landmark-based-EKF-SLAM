#!/usr/bin/env python3

import numpy as np
from typing import Dict, Tuple
from scipy.spatial.transform import Rotation


def pose_dict_to_transform(pose: Dict) -> np.ndarray:
    """
    Convert pose dictionary to 4x4 transformation matrix.

    Args:
        pose: Dictionary with keys 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'

    Returns:
        4x4 transformation matrix
    """
    T = np.eye(4)

    # Translation
    T[0, 3] = pose['x']
    T[1, 3] = pose['y']
    T[2, 3] = pose.get('z', 0.0)

    # Rotation from quaternion
    quat = [pose['qx'], pose['qy'], pose['qz'], pose['qw']]
    R = Rotation.from_quat(quat).as_matrix()
    T[0:3, 0:3] = R

    return T


def transform_to_pose_dict(T: np.ndarray) -> Dict:
    """
    Convert 4x4 transformation matrix to pose dictionary.

    Args:
        T: 4x4 transformation matrix

    Returns:
        Dictionary with 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'
    """
    pose = {}

    # Translation
    pose['x'] = T[0, 3]
    pose['y'] = T[1, 3]
    pose['z'] = T[2, 3]

    # Rotation to quaternion
    R = T[0:3, 0:3]
    quat = Rotation.from_matrix(R).as_quat()  # [qx, qy, qz, qw]

    pose['qx'] = quat[0]
    pose['qy'] = quat[1]
    pose['qz'] = quat[2]
    pose['qw'] = quat[3]

    return pose


def transform_pose(pose: Dict, T: np.ndarray) -> Dict:
    """
    Apply transformation to pose.

    Args:
        pose: Original pose dictionary
        T: 4x4 transformation matrix

    Returns:
        Transformed pose dictionary
    """
    T_pose = pose_dict_to_transform(pose)
    T_new = T @ T_pose
    return transform_to_pose_dict(T_new)


def compute_2d_transform(x: float, y: float, theta: float) -> np.ndarray:
    """
    Create 4x4 transformation from 2D pose.

    Args:
        x, y: Translation
        theta: Rotation angle (radians)

    Returns:
        4x4 transformation matrix
    """
    T = np.eye(4)
    T[0, 3] = x
    T[1, 3] = y

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    T[0, 0] = cos_theta
    T[0, 1] = -sin_theta
    T[1, 0] = sin_theta
    T[1, 1] = cos_theta

    return T


def extract_2d_pose(T: np.ndarray) -> Tuple[float, float, float]:
    """
    Extract 2D pose from 4x4 transformation.

    Args:
        T: 4x4 transformation matrix

    Returns:
        (x, y, theta) tuple
    """
    x = T[0, 3]
    y = T[1, 3]
    theta = np.arctan2(T[1, 0], T[0, 0])

    return x, y, theta


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles.

    Args:
        qx, qy, qz, qw: Quaternion components

    Returns:
        (roll, pitch, yaw) in radians
    """
    quat = [qx, qy, qz, qw]
    euler = Rotation.from_quat(quat).as_euler('xyz', degrees=False)
    return euler[0], euler[1], euler[2]


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles to quaternion.

    Args:
        roll, pitch, yaw: Euler angles (radians)

    Returns:
        [qx, qy, qz, qw]
    """
    quat = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_quat()
    return quat


def compute_relative_transform(pose1: Dict, pose2: Dict) -> np.ndarray:
    """
    Compute relative transformation from pose1 to pose2.

    Args:
        pose1: First pose
        pose2: Second pose

    Returns:
        T_1_to_2
    """
    T1 = pose_dict_to_transform(pose1)
    T2 = pose_dict_to_transform(pose2)

    T_1_to_2 = np.linalg.inv(T1) @ T2

    return T_1_to_2


def compute_distance_2d(pose1: Dict, pose2: Dict) -> float:
    """
    Compute 2D Euclidean distance between two poses.

    Args:
        pose1, pose2: Pose dictionaries

    Returns:
        Distance in meters
    """
    dx = pose2['x'] - pose1['x']
    dy = pose2['y'] - pose1['y']

    return np.sqrt(dx**2 + dy**2)


def validate_transform(T: np.ndarray, logger=None) -> bool:
    """
    Validate transformation matrix.

    Checks:
    - Shape is 4x4
    - Bottom row is [0, 0, 0, 1]
    - Rotation part is valid (orthonormal)
    - No NaN or inf values

    Args:
        T: Transformation matrix
        logger: Optional logger for warnings

    Returns:
        True if valid
    """
    # Check shape
    if T.shape != (4, 4):
        if logger:
            logger.error(f'Invalid transform shape: {T.shape}')
        return False

    # Check for NaN/inf
    if not np.all(np.isfinite(T)):
        if logger:
            logger.error('Transform contains NaN or inf values')
        return False

    # Check bottom row
    if not np.allclose(T[3, :], [0, 0, 0, 1]):
        if logger:
            logger.warn(f'Transform bottom row not [0,0,0,1]: {T[3,:]}')
        return False

    # Check rotation matrix (orthonormal)
    R = T[0:3, 0:3]
    R_orthogonal = R @ R.T
    I = np.eye(3)

    if not np.allclose(R_orthogonal, I, atol=1e-3):
        if logger:
            logger.warn(f'Rotation matrix not orthonormal: {R_orthogonal}')
        return False

    # Check determinant (should be +1 for proper rotation)
    det = np.linalg.det(R)
    if not np.isclose(det, 1.0, atol=1e-3):
        if logger:
            logger.warn(f'Rotation matrix determinant not 1: {det}')
        return False

    return True


def apply_2d_constraint(T: np.ndarray) -> np.ndarray:
    """
    Enforce 2D transformation constraints.

    Sets:
    - Z-axis rotation to identity
    - Z translation to 0
    - X,Y coupling with Z to 0

    Args:
        T: Input transformation

    Returns:
        Constrained transformation
    """
    T_constrained = T.copy()

    # No Z translation
    T_constrained[2, 3] = 0.0

    # No Z-axis rotation coupling
    T_constrained[2, 0:3] = [0, 0, 1]
    T_constrained[0:2, 2] = [0, 0]

    return T_constrained


def compute_environment_bounds(points: np.ndarray, margin: float = 0.5) -> Dict:
    """
    Compute environment bounding box from point cloud.

    Args:
        points: Nx3 point cloud
        margin: Safety margin to add (meters)

    Returns:
        Dictionary with 'x_min', 'x_max', 'y_min', 'y_max'
    """
    if len(points) == 0:
        return {'x_min': 0.0, 'x_max': 0.0, 'y_min': 0.0, 'y_max': 0.0}

    x_min = np.min(points[:, 0]) - margin
    x_max = np.max(points[:, 0]) + margin
    y_min = np.min(points[:, 1]) - margin
    y_max = np.max(points[:, 1]) + margin

    return {
        'x_min': float(x_min),
        'x_max': float(x_max),
        'y_min': float(y_min),
        'y_max': float(y_max)
    }


def merge_environment_bounds(bounds_list: list) -> Dict:
    """
    Merge multiple environment bounds into one.

    Args:
        bounds_list: List of bounds dictionaries

    Returns:
        Merged bounds dictionary
    """
    if not bounds_list:
        return {'x_min': 0.0, 'x_max': 0.0, 'y_min': 0.0, 'y_max': 0.0}

    x_min = min(b['x_min'] for b in bounds_list)
    x_max = max(b['x_max'] for b in bounds_list)
    y_min = min(b['y_min'] for b in bounds_list)
    y_max = max(b['y_max'] for b in bounds_list)

    return {
        'x_min': float(x_min),
        'x_max': float(x_max),
        'y_min': float(y_min),
        'y_max': float(y_max)
    }

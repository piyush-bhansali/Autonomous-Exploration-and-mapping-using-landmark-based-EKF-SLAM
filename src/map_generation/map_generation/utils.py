import numpy as np


# ============================================================================
# ANGLE UTILITIES
# ============================================================================

def degrees_to_radians(degrees):
    """Convert degrees to radians"""
    return degrees * np.pi / 180.0


def radians_to_degrees(radians):
    """Convert radians to degrees"""
    return radians * 180.0 / np.pi


def normalize_angle(angle):
    """
    Normalize angle to [-pi, pi]

    Args:
        angle: Angle in radians

    Returns:
        float: Normalized angle in range [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


# ============================================================================
# QUATERNION OPERATIONS
# ============================================================================

def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Convert quaternion to yaw angle (rotation around Z-axis)
    
    Args:
        qx, qy, qz, qw: Quaternion components
    
    Returns:
        float: Yaw angle in radians [-pi, pi]
    
    Formula:
        yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
    
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    """
    Convert yaw angle to quaternion (rotation around Z-axis)
    
    Args:
        yaw: Angle in radians
    
    Returns:
        tuple: (qx, qy, qz, qw) quaternion components
    
    Formula:
        For rotation around Z-axis:
        qx = 0, qy = 0
        qz = sin(yaw/2)
        qw = cos(yaw/2)
    """
    qx = 0.0
    qy = 0.0
    qz = np.sin(yaw / 2.0)
    qw = np.cos(yaw / 2.0)
    return (qx, qy, qz, qw)

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """
    Convert quaternion to 3x3 rotation matrix
    
    Args:
        qx, qy, qz, qw: Quaternion components
    
    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])
    return R

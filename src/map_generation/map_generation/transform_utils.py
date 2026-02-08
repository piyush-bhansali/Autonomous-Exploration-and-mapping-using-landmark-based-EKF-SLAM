#!/usr/bin/env python3
"""
Transform utilities for SLAM frame computations.

Provides helper functions for computing transforms between map, odom, and robot frames.
"""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation


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


# =============================================================================
# ROS2 Message Conversions
# =============================================================================

def numpy_to_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    
    # Create header
    header = Header()
    header.frame_id = frame_id
    header.stamp = stamp

    # Define point cloud fields (x, y, z as FLOAT32)
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    points_flat = points.astype(np.float32).flatten()
    cloud_data = points_flat.tobytes()

    # Build PointCloud2 message
    msg = PointCloud2()
    msg.header = header
    msg.height = 1 
    msg.width = len(points)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = 12 
    msg.row_step = msg.point_step * len(points)
    msg.is_dense = True 
    msg.data = cloud_data

    return msg


# =============================================================================
# Quaternion Conversions (using scipy.spatial.transform.Rotation)
# =============================================================================

def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """
    Extract yaw angle from quaternion.

    Args:
        qx, qy, qz, qw: Quaternion components (scalar-last convention)

    Returns:
        Yaw angle in radians
    """
    rotation = Rotation.from_quat([qx, qy, qz, qw])
    # Extract Euler angles in ZYX convention (yaw, pitch, roll)
    euler = rotation.as_euler('zyx', degrees=False)
    return euler[0]  # Return yaw (z-axis rotation)


def yaw_to_quaternion(yaw: float) -> tuple:
    """
    Convert yaw angle to quaternion (pure z-axis rotation).

    Args:
        yaw: Yaw angle in radians

    Returns:
        Tuple (qx, qy, qz, qw) in scalar-last convention
    """
    rotation = Rotation.from_euler('z', yaw, degrees=False)
    quat = rotation.as_quat()  # Returns [x, y, z, w]
    return (quat[0], quat[1], quat[2], quat[3])


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    Convert quaternion to 3x3 rotation matrix.

    Args:
        qx, qy, qz, qw: Quaternion components (scalar-last convention)

    Returns:
        3x3 rotation matrix as numpy array
    """
    rotation = Rotation.from_quat([qx, qy, qz, qw])
    return rotation.as_matrix()

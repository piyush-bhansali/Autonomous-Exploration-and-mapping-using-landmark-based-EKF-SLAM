#!/usr/bin/env python3


import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation


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

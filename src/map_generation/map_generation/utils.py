#!/usr/bin/env python3


import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


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
# Quaternion Conversions
# =============================================================================

def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return np.arctan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> tuple:
   
    qx = 0.0
    qy = 0.0
    qz = np.sin(yaw / 2.0)
    qw = np.cos(yaw / 2.0)
    return (qx, qy, qz, qw)


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
    ])
    return R

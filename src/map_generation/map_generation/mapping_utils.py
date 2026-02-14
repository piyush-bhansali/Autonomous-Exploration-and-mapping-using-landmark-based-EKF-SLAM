#!/usr/bin/env python3

import numpy as np
from typing import Optional

from map_generation.transform_utils import numpy_to_pointcloud2


def publish_global_map(
    global_points: Optional[np.ndarray],
    publisher,
    clock,
    frame_id
) -> None:
    """
    Publish global map point cloud.

    Args:
        global_points: (N x 3) numpy array of points
        publisher: ROS2 publisher for PointCloud2
        clock: ROS2 clock for timestamp
        frame_id: Frame ID for the point cloud (typically 'map')
    """
    if global_points is not None and len(global_points) > 0:
        pc2_msg = numpy_to_pointcloud2(
            global_points,
            frame_id,
            clock.now().to_msg()
        )
        publisher.publish(pc2_msg)


def publish_feature_markers(
    features: list,
    scan_timestamp,
    publisher,
    robot_name: str
) -> None:
    
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import ColorRGBA

    marker_array = MarkerArray()

    for i, feature in enumerate(features):
        marker = Marker()
        marker.header.frame_id = f'{robot_name}/base_scan'  # Features in robot frame
        marker.header.stamp = scan_timestamp  # Use scan timestamp for TF sync
        marker.ns = 'scan_features'
        marker.id = i
        marker.action = Marker.ADD
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 200000000  # 0.2 seconds

        if feature['type'] == 'corner':
            # Corners: Red spheres at Cartesian position
            marker.type = Marker.SPHERE
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red

            # Position from Cartesian coordinates
            marker.pose.position.x = float(feature['position'][0])
            marker.pose.position.y = float(feature['position'][1])
            marker.pose.position.z = 0.0

        elif feature['type'] == 'wall':
            # Walls: Blue cylinders at closest point on wall
            marker.type = Marker.CYLINDER
            marker.scale.x = 0.08  # Diameter
            marker.scale.y = 0.08
            marker.scale.z = 0.2   # Height
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.6)  # Blue

            # Wall in Hessian form: ρ = x*cos(α) + y*sin(α)
            # Closest point on wall to origin: (ρ*cos(α), ρ*sin(α))
            rho = feature['rho']
            alpha = feature['alpha']

            marker.pose.position.x = float(rho * np.cos(alpha))
            marker.pose.position.y = float(rho * np.sin(alpha))
            marker.pose.position.z = 0.0

            # Orient cylinder along wall direction (perpendicular to normal)
            marker.pose.orientation.z = np.sin(alpha / 2.0)
            marker.pose.orientation.w = np.cos(alpha / 2.0)

        marker_array.markers.append(marker)

        # Add text label showing feature info
        text_marker = Marker()
        text_marker.header = marker.header
        text_marker.ns = 'feature_labels'
        text_marker.id = i + 1000  # Offset to avoid ID collision
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = marker.pose.position.x
        text_marker.pose.position.y = marker.pose.position.y
        text_marker.pose.position.z = 0.3  # Above the feature
        text_marker.scale.z = 0.1  # Text size
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White

        # Different labels for walls vs corners
        if feature['type'] == 'wall':
            text_marker.text = f"WALL\n{feature['strength']:.2f}m"
        else:
            text_marker.text = f"CORNER\n{feature['strength']:.0f}°"

        text_marker.lifetime = marker.lifetime

        marker_array.markers.append(text_marker)

    publisher.publish(marker_array)

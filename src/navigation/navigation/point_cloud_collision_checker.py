#!/usr/bin/env python3
"""
Point Cloud Collision Checker

Fast collision detection for 2D navigation using point cloud maps.
Uses KD-tree for efficient nearest neighbor queries.
"""

import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
from typing import Optional, List, Tuple


class PointCloudCollisionChecker:
    """
    Efficient collision checker for 2D point cloud maps

    Uses spatial indexing (KD-tree) to avoid grid conversion overhead.
    Designed for 2D navigation (ignores Z-axis).
    """

    def __init__(self,
                 point_cloud: o3d.geometry.PointCloud,
                 robot_radius: float = 0.2,
                 safety_margin: float = 0.05):
        """
        Args:
            point_cloud: Open3D point cloud representing the map
            robot_radius: Robot's collision radius (meters)
            safety_margin: Additional safety buffer (meters)
        """
        self.point_cloud = point_cloud
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.total_radius = robot_radius + safety_margin

        # Extract 2D points (X, Y only)
        points_3d = np.asarray(point_cloud.points)
        self.points_2d = points_3d[:, :2]

        # Build KD-tree for fast spatial queries
        self.kdtree = KDTree(self.points_2d)

    def is_collision(self, x: float, y: float) -> bool:
        """
        Check if a 2D position collides with obstacles

        Args:
            x, y: 2D position to check

        Returns:
            True if collision, False if free
        """
        query_point = np.array([x, y])

        # Query nearest obstacle within collision radius
        distances, indices = self.kdtree.query(query_point, k=1)

        return distances < self.total_radius

    def is_path_collision_free(self,
                               start: np.ndarray,
                               end: np.ndarray,
                               resolution: float = 0.05) -> bool:
        """
        Check if a straight-line path is collision-free

        Args:
            start: Start position [x, y]
            end: End position [x, y]
            resolution: Distance between checked points (meters)

        Returns:
            True if path is free, False if collision
        """
        # Compute path length
        distance = np.linalg.norm(end - start)

        if distance < 1e-6:
            return not self.is_collision(start[0], start[1])

        # Number of points to check
        num_points = int(np.ceil(distance / resolution)) + 1

        # Safety check for division by zero
        if num_points < 2:
            return not self.is_collision(start[0], start[1])

        # Interpolate points along path
        for i in range(num_points):
            t = i / (num_points - 1)
            point = start + t * (end - start)

            if self.is_collision(point[0], point[1]):
                return False

        return True

    def get_nearest_obstacle_distance(self, x: float, y: float) -> float:
        """
        Get distance to nearest obstacle

        Args:
            x, y: 2D position

        Returns:
            Distance to nearest obstacle (meters)
        """
        query_point = np.array([x, y])
        distances, _ = self.kdtree.query(query_point, k=1)
        return float(distances)

    def is_region_free(self,
                      center: np.ndarray,
                      radius: float) -> bool:
        """
        Check if a circular region is free of obstacles

        Args:
            center: Center position [x, y]
            radius: Radius of region to check

        Returns:
            True if region is free, False otherwise
        """
        # Query all points within radius + collision radius
        search_radius = radius + self.total_radius
        indices = self.kdtree.query_ball_point(center, search_radius)

        # If any obstacles found within combined radius, region is not free
        return len(indices) == 0

    def get_closest_points_in_radius(self,
                                    center: np.ndarray,
                                    radius: float,
                                    max_points: int = 100) -> np.ndarray:
        """
        Get obstacle points within a radius

        Args:
            center: Center position [x, y]
            radius: Search radius
            max_points: Maximum number of points to return

        Returns:
            Array of obstacle points (N x 2)
        """
        distances, indices = self.kdtree.query(center, k=max_points)

        # Filter by radius
        valid = distances < radius
        return self.points_2d[indices[valid]]

    def update_map(self, new_point_cloud: o3d.geometry.PointCloud):
        """
        Update the collision checker with a new map

        Args:
            new_point_cloud: Updated point cloud map
        """
        self.point_cloud = new_point_cloud
        points_3d = np.asarray(new_point_cloud.points)
        self.points_2d = points_3d[:, :2]
        self.kdtree = KDTree(self.points_2d)

    def get_map_bounds(self) -> Tuple[float, float, float, float]:
        """
        Get the bounding box of the map

        Returns:
            (min_x, max_x, min_y, max_y)
        """
        min_x = float(self.points_2d[:, 0].min())
        max_x = float(self.points_2d[:, 0].max())
        min_y = float(self.points_2d[:, 1].min())
        max_y = float(self.points_2d[:, 1].max())

        return min_x, max_x, min_y, max_y

#!/usr/bin/env python3
"""
Frontier Detector for Point Cloud Maps

Detects exploration frontiers directly from 2D point cloud maps without
grid conversion. Frontiers are boundaries between known free space and
unknown space.
"""

import numpy as np
import open3d as o3d
from scipy.spatial import KDTree
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class Frontier:
    """Represents a frontier for exploration"""
    centroid: np.ndarray  # [x, y] center of frontier
    points: np.ndarray    # Frontier boundary points (N x 2)
    size: int             # Number of points in frontier
    score: float          # Exploration priority score


class FrontierDetector:
    """
    Detect exploration frontiers from 2D point cloud maps

    Strategy:
    1. Sample free space points around the robot
    2. Find points that are near obstacles but far from dense map regions
    3. Cluster frontier points into frontier regions
    4. Rank frontiers by size, distance, and information gain
    """

    def __init__(self,
                 robot_radius: float = 0.2,
                 frontier_search_radius: float = 8.0,  # Increased from 5.0 for better exploration
                 min_frontier_size: int = 5,  # Increased from 3 to reduce noise
                 frontier_cluster_distance: float = 0.5,
                 obstacle_distance_threshold: float = 0.5):
        """
        Args:
            robot_radius: Robot collision radius
            frontier_search_radius: Maximum distance to search for frontiers
            min_frontier_size: Minimum points to form a valid frontier
            frontier_cluster_distance: Distance to cluster frontier points
            obstacle_distance_threshold: Distance from obstacles to consider frontier
        """
        self.robot_radius = robot_radius
        self.frontier_search_radius = frontier_search_radius
        self.min_frontier_size = min_frontier_size
        self.frontier_cluster_distance = frontier_cluster_distance
        self.obstacle_distance_threshold = obstacle_distance_threshold

    def detect_frontiers(self,
                        point_cloud: o3d.geometry.PointCloud,
                        robot_position: np.ndarray) -> List[Frontier]:
        """
        Detect frontiers in the point cloud map

        Args:
            point_cloud: Current map (point cloud)
            robot_position: Current robot position [x, y]

        Returns:
            List of detected frontiers, sorted by score (best first)
        """
        if len(point_cloud.points) < 10:
            return []

        # Extract 2D points
        map_points = np.asarray(point_cloud.points)[:, :2]
        kdtree = KDTree(map_points)

        # Step 1: Sample candidate frontier points in a circular region
        candidate_points = self._sample_frontier_candidates(
            robot_position, map_points, kdtree
        )

        if len(candidate_points) < self.min_frontier_size:
            return []

        # Step 2: Filter candidates based on frontier criteria
        frontier_points = self._filter_frontier_points(
            candidate_points, map_points, kdtree
        )

        if len(frontier_points) < self.min_frontier_size:
            return []

        # Step 3: Cluster frontier points into frontier regions
        frontiers = self._cluster_frontiers(frontier_points, robot_position)

        # Step 4: Score and rank frontiers
        frontiers = self._score_frontiers(frontiers, robot_position, map_points)

        return frontiers

    def _sample_frontier_candidates(self,
                                    robot_position: np.ndarray,
                                    map_points: np.ndarray,
                                    kdtree: KDTree) -> np.ndarray:
        """
        Sample candidate frontier points across the ENTIRE global map

        Strategy: Create a grid covering the map bounds, not just near robot
        """
        # Get map bounds to sample the entire explored area
        if len(map_points) == 0:
            return np.array([]).reshape(0, 2)

        x_min_map = map_points[:, 0].min() - 2.0  # Add 2m buffer
        x_max_map = map_points[:, 0].max() + 2.0
        y_min_map = map_points[:, 1].min() - 2.0
        y_max_map = map_points[:, 1].max() + 2.0

        # Sample resolution - balance between coverage and computation
        sample_resolution = 0.3  # meters

        # Calculate grid dimensions
        x_range = x_max_map - x_min_map
        y_range = y_max_map - y_min_map

        n_samples_x = max(int(x_range / sample_resolution), 10)
        n_samples_y = max(int(y_range / sample_resolution), 10)

        # Limit to reasonable grid size for performance (max 100x100 = 10k points)
        n_samples_x = min(n_samples_x, 100)
        n_samples_y = min(n_samples_y, 100)

        x_samples = np.linspace(x_min_map, x_max_map, n_samples_x)
        y_samples = np.linspace(y_min_map, y_max_map, n_samples_y)

        xx, yy = np.meshgrid(x_samples, y_samples)
        sample_points = np.column_stack([xx.ravel(), yy.ravel()])

        return sample_points

    def _filter_frontier_points(self,
                               candidate_points: np.ndarray,
                               map_points: np.ndarray,
                               kdtree: KDTree) -> np.ndarray:
        """
        Filter candidate points to identify true frontier points

        Frontier criteria:
        - Not too close to obstacles (collision-free)
        - Near the edge of known map (low local density)
        """
        frontier_points = []

        for point in candidate_points:
            # Get distance to nearest obstacle
            dist_to_obstacle, _ = kdtree.query(point, k=1)

            # Criterion 1: Not too close to obstacles (must be navigable)
            # Allow at least 1.5x robot radius clearance
            if dist_to_obstacle < self.robot_radius * 1.5:
                continue

            # Criterion 2: Check if on edge of known space (sparse local density)
            # This identifies boundaries between known free space and unknown space
            neighbors_count = len(kdtree.query_ball_point(point, 0.5))

            # If sparse (<5 neighbors in 0.5m radius), likely at frontier
            # Increased from 3 to reduce false positives from noise
            if neighbors_count < 5:
                frontier_points.append(point)

        return np.array(frontier_points) if frontier_points else np.array([]).reshape(0, 2)

    def _cluster_frontiers(self,
                          frontier_points: np.ndarray,
                          robot_position: np.ndarray) -> List[Frontier]:
        """
        Cluster frontier points into distinct frontier regions using DBSCAN-like approach
        """
        if len(frontier_points) == 0:
            return []

        # Simple clustering: group nearby points
        frontiers = []
        unvisited = set(range(len(frontier_points)))

        while unvisited:
            # Start new cluster
            cluster_points = []
            queue = [unvisited.pop()]

            while queue:
                idx = queue.pop(0)
                cluster_points.append(frontier_points[idx])

                # Find neighbors
                for other_idx in list(unvisited):
                    dist = np.linalg.norm(frontier_points[idx] - frontier_points[other_idx])
                    if dist < self.frontier_cluster_distance:
                        queue.append(other_idx)
                        unvisited.remove(other_idx)

            # Create frontier if large enough
            if len(cluster_points) >= self.min_frontier_size:
                cluster_array = np.array(cluster_points)
                centroid = np.mean(cluster_array, axis=0)

                frontiers.append(Frontier(
                    centroid=centroid,
                    points=cluster_array,
                    size=len(cluster_points),
                    score=0.0  # Will be computed later
                ))

        return frontiers

    def _score_frontiers(self,
                        frontiers: List[Frontier],
                        robot_position: np.ndarray,
                        map_points: np.ndarray) -> List[Frontier]:
        """
        Score frontiers based on:
        - Size (larger = more unexplored area)
        - Distance (closer = less travel cost)
        - Information gain (less explored = higher value)
        """
        for frontier in frontiers:
            # Distance to frontier
            distance = np.linalg.norm(frontier.centroid - robot_position)

            # Normalize distance (closer is better)
            distance_score = np.exp(-distance / 5.0)

            # Size score (larger is better)
            size_score = min(frontier.size / 50.0, 1.0)

            # Information gain: check how sparse the region is
            kdtree = KDTree(map_points)
            neighbors_count = len(kdtree.query_ball_point(frontier.centroid, 2.0))
            info_gain = np.exp(-neighbors_count / 100.0)

            # Combined score
            frontier.score = (
                0.4 * distance_score +
                0.3 * size_score +
                0.3 * info_gain
            )

        # Sort by score (highest first)
        frontiers.sort(key=lambda f: f.score, reverse=True)

        return frontiers

    def visualize_frontiers(self,
                           frontiers: List[Frontier],
                           robot_position: np.ndarray) -> o3d.geometry.PointCloud:
        """
        Create a point cloud visualization of detected frontiers

        Args:
            frontiers: List of frontiers
            robot_position: Robot position [x, y]

        Returns:
            Point cloud with colored frontier points
        """
        if not frontiers:
            return o3d.geometry.PointCloud()

        # Collect all frontier points
        all_points = []
        all_colors = []

        # Color palette for different frontiers
        colors = [
            [1.0, 0.0, 0.0],  # Red (highest priority)
            [1.0, 0.5, 0.0],  # Orange
            [1.0, 1.0, 0.0],  # Yellow
            [0.0, 1.0, 0.0],  # Green
            [0.0, 0.0, 1.0],  # Blue
        ]

        for i, frontier in enumerate(frontiers[:5]):  # Top 5 frontiers
            color = colors[i % len(colors)]
            for point in frontier.points:
                all_points.append([point[0], point[1], 0.0])
                all_colors.append(color)

            # Add centroid marker (brighter)
            all_points.append([frontier.centroid[0], frontier.centroid[1], 0.1])
            all_colors.append([min(c * 1.5, 1.0) for c in color])

        # Add robot position
        all_points.append([robot_position[0], robot_position[1], 0.2])
        all_colors.append([1.0, 1.0, 1.0])  # White

        # Create point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(all_points))
        pcd.colors = o3d.utility.Vector3dVector(np.array(all_colors))

        return pcd

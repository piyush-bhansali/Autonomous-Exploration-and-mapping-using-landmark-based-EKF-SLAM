#!/usr/bin/env python3
"""
Simple Frontier Detector with Clustering

Finds exploration frontiers at the boundary of the known map.
Clusters nearby frontiers and scores clusters by size and information gain.
"""

import numpy as np
from scipy.spatial import KDTree
from sklearn.cluster import DBSCAN
from typing import List, Tuple, Optional


class SimpleFrontier:
    """A frontier cluster for exploration"""
    def __init__(self, position: np.ndarray, score: float, size: int = 1):
        self.position = position  # [x, y] - cluster centroid
        self.score = score
        self.size = size  # Number of frontier points in cluster


class SimpleFrontierDetector:
    """Detects frontiers at map boundary"""

    def __init__(self, robot_radius: float = 0.22):
        self.robot_radius = robot_radius

    def detect(self,
              map_points: np.ndarray,  # Nx3 array of [x,y,z] points
              robot_pos: np.ndarray) -> List[SimpleFrontier]:  # [x, y]
        """
        Find frontiers at boundary between KNOWN and UNKNOWN space.

        Strategy:
        1. Find map bounds from all points
        2. Place frontiers 0.3m BEFORE map boundary (inside mapped area)
        3. Filter for collision-free positions only
        4. Score clusters and return best
        """
        if len(map_points) < 100:
            return []  # Need minimum map size

        # Extract 2D points (x, y)
        points_2d = map_points[:, :2]

        # Build KD-tree for fast queries
        kdtree = KDTree(points_2d)

        # Find FREE SPACE boundary (points far from obstacles)
        free_space_points = self._extract_free_space_boundary(points_2d, kdtree)

        if len(free_space_points) < 10:
            # Fallback to full map bounds
            free_space_points = points_2d

        # Get bounds of free space
        x_min, y_min = free_space_points.min(axis=0)
        x_max, y_max = free_space_points.max(axis=0)

        # Sample frontiers 0.3m BEFORE map boundary (inside mapped area)
        candidates = self._sample_boundary(x_min, x_max, y_min, y_max)

        # Filter candidates: ONLY check collision-free
        # Path planner (RRT*) handles obstacle avoidance and reachability!
        safe_candidates = []
        for candidate in candidates:
            # Check: Not too close to obstacles
            dist, _ = kdtree.query(candidate)
            if dist < self.robot_radius * 2.0:
                continue  # Too close to wall

            safe_candidates.append(candidate)

        if len(safe_candidates) == 0:
            return []

        # CLUSTERING: Group nearby frontiers
        clusters = self._cluster_frontiers(safe_candidates)

        # Score each cluster (not individual points!)
        frontiers = self._score_frontier_clusters(
            clusters, robot_pos, points_2d, kdtree
        )

        return frontiers

    def _sample_boundary(self,
                        x_min: float, x_max: float,
                        y_min: float, y_max: float) -> List[np.ndarray]:
        """
        Sample points at map boundary with equal spacing

        Places points 0.3m BEFORE map edge (inside mapped area)
        Spacing = 0.8m between points
        """
        offset = 0.3  # meters inside boundary (positive = distance from edge)
        spacing = 0.8  # meters between samples

        candidates = []

        # Top edge (y = y_max - offset, inside the boundary)
        x_samples = np.arange(x_min, x_max + spacing, spacing)
        for x in x_samples:
            candidates.append(np.array([x, y_max - offset]))

        # Bottom edge (y = y_min + offset, inside the boundary)
        for x in x_samples:
            candidates.append(np.array([x, y_min + offset]))

        # Right edge (x = x_max - offset, inside the boundary)
        y_samples = np.arange(y_min, y_max + spacing, spacing)
        for y in y_samples:
            candidates.append(np.array([x_max - offset, y]))

        # Left edge (x = x_min + offset, inside the boundary)
        for y in y_samples:
            candidates.append(np.array([x_min + offset, y]))

        return candidates

    def _extract_free_space_boundary(self,
                                     points_2d: np.ndarray,
                                     kdtree: KDTree) -> np.ndarray:
        """
        Extract free space points by finding areas that are:
        1. Far from obstacles (navigable)
        2. Near the robot (already explored)

        This ensures frontiers are at the edge of EXPLORED free space,
        not at the edge of walls.
        """
        # Simply return all points - the line-of-sight check
        # will filter out unreachable frontiers behind walls
        # This is simpler and more reliable than density filtering
        return points_2d

    def _sample_free_space_frontiers(self,
                                     x_min: float, x_max: float,
                                     y_min: float, y_max: float,
                                     robot_pos: np.ndarray,
                                     kdtree: KDTree,
                                     map_points: np.ndarray) -> List[np.ndarray]:
        """
        Sample frontiers in FREE SPACE at edge of explored region.

        A frontier is:
        - In free space (far from obstacles)
        - Has line-of-sight to robot
        - At edge of explored area (leads to unexplored space)
        """
        candidates = []
        grid_size = 0.8  # Sample every 0.8m

        # Create grid of potential frontier points
        x_samples = np.arange(x_min, x_max, grid_size)
        y_samples = np.arange(y_min, y_max, grid_size)

        for x in x_samples:
            for y in y_samples:
                point = np.array([x, y])

                # Check 1: Must be in FREE space (not too close to obstacles)
                dist_to_obstacle, _ = kdtree.query(point)
                if dist_to_obstacle < self.robot_radius * 3.0:
                    continue  # Too close to obstacle

                # Check 2: Must have line-of-sight to robot (reachable!)
                if not self._has_line_of_sight(robot_pos, point, kdtree):
                    continue  # Blocked by wall

                # Check 3: Must be at EDGE of explored space
                # (i.e., nearby space should be unexplored)
                if not self._is_exploration_frontier(point, kdtree):
                    continue  # Not at edge of explored area

                candidates.append(point)

        return candidates

    def _is_exploration_frontier(self,
                                 point: np.ndarray,
                                 kdtree: KDTree) -> bool:
        """
        Check if point is at edge of explored space.

        A point is a frontier if it's in free space BUT
        has unexplored regions nearby (far from any mapped points).
        """
        # Sample 8 directions around the point
        check_distance = 2.0  # Look 2m ahead

        unexplored_count = 0
        for angle in np.linspace(0, 2*np.pi, 8, endpoint=False):
            check_point = point + check_distance * np.array([
                np.cos(angle), np.sin(angle)
            ])

            # Distance to nearest mapped obstacle
            dist, _ = kdtree.query(check_point)

            # If far from any mapped point, likely unexplored
            if dist > 1.5:  # >1.5m from any mapped point = unexplored
                unexplored_count += 1

        # Frontier if at least 2 directions lead to unexplored space
        return unexplored_count >= 2

    def _cluster_frontiers(self, candidates: List[np.ndarray]) -> List[List[np.ndarray]]:
        """
        Cluster nearby frontier points using DBSCAN.

        Parameters:
        - eps: 1.0m (frontiers within 1.0m are same cluster)
        - min_samples: 2 (need at least 2 points to form cluster)

        Returns list of clusters, where each cluster is a list of frontier points.
        """
        if len(candidates) < 2:
            # Not enough points to cluster
            return [[c] for c in candidates]

        # Convert to numpy array for DBSCAN
        points = np.array(candidates)

        # DBSCAN clustering
        # eps=1.0: Points within 1.0m are neighbors
        # min_samples=2: Need 2+ points to form cluster
        clustering = DBSCAN(eps=1.0, min_samples=2).fit(points)

        labels = clustering.labels_

        # Group points by cluster
        clusters = []
        unique_labels = set(labels)

        for label in unique_labels:
            if label == -1:
                # Noise points (not in any cluster) - treat each as individual cluster
                noise_points = points[labels == -1]
                for point in noise_points:
                    clusters.append([point])
            else:
                # Valid cluster
                cluster_points = points[labels == label].tolist()
                clusters.append(cluster_points)

        return clusters

    def _score_frontier_clusters(self,
                                 clusters: List[List[np.ndarray]],
                                 robot_pos: np.ndarray,
                                 map_points: np.ndarray,
                                 kdtree: KDTree) -> List[SimpleFrontier]:
        """
        Score frontier clusters by:
        - Cluster size (cardinality) - 40%
        - Travel cost (distance to centroid) - 30%
        - Information gain (unexplored area) - 30%
        """
        frontiers = []

        for cluster in clusters:
            # Compute cluster centroid
            cluster_array = np.array(cluster)
            centroid = cluster_array.mean(axis=0)

            # 1. CLUSTER SIZE: Bigger clusters = more exploration potential
            cluster_size = len(cluster)
            size_score = min(cluster_size / 10.0, 1.0)  # Normalize to [0, 1]

            # 2. TRAVEL COST: Distance to centroid
            distance = np.linalg.norm(centroid - robot_pos)
            travel_score = np.exp(-distance / 5.0)  # Closer = higher

            # 3. INFORMATION GAIN: Average unexplored area around cluster
            info_gain_score = self._cluster_information_gain(cluster_array, kdtree)

            # COMBINED SCORE
            score = (
                0.40 * size_score +        # Cluster size (bigger = better)
                0.30 * travel_score +      # Travel cost (closer = better)
                0.30 * info_gain_score     # Information gain
            )

            # Create frontier at cluster centroid
            frontiers.append(SimpleFrontier(centroid, score, cluster_size))

        # Sort by score (best first)
        frontiers.sort(key=lambda f: f.score, reverse=True)

        return frontiers

    def _cluster_information_gain(self,
                                  cluster_points: np.ndarray,
                                  kdtree: KDTree) -> float:
        """
        Compute average information gain for a cluster.

        Sample around each cluster point and count unexplored directions.
        """
        total_info_gain = 0.0

        # Sample a few representative points from cluster (not all)
        sample_size = min(5, len(cluster_points))
        sample_indices = np.linspace(0, len(cluster_points)-1, sample_size, dtype=int)

        for idx in sample_indices:
            point = cluster_points[idx]

            # Check 8 directions around this point
            unexplored_count = 0
            check_distance = 2.0

            for angle in np.linspace(0, 2*np.pi, 8, endpoint=False):
                check_point = point + check_distance * np.array([
                    np.cos(angle), np.sin(angle)
                ])

                dist, _ = kdtree.query(check_point)

                if dist > 1.5:  # Unexplored
                    unexplored_count += 1

            total_info_gain += unexplored_count / 8.0

        # Average info gain across sampled points
        return total_info_gain / sample_size

    def _leads_to_unexplored(self,
                            frontier: np.ndarray,
                            kdtree: KDTree) -> bool:
        """
        Check if frontier actually leads to unexplored space.

        A frontier is only useful if there's UNMAPPED space beyond it!
        Sample points beyond the frontier - if they're far from any
        mapped points, that's unexplored territory.
        """
        # Check 8 directions radiating FROM the frontier
        check_distance = 2.0  # Look 2m beyond frontier

        unexplored_directions = 0
        for angle in np.linspace(0, 2*np.pi, 8, endpoint=False):
            # Point beyond the frontier
            check_point = frontier + check_distance * np.array([
                np.cos(angle), np.sin(angle)
            ])

            # Distance to nearest mapped point
            dist, _ = kdtree.query(check_point)

            # If >1.5m from any mapped point = unexplored!
            if dist > 1.5:
                unexplored_directions += 1

        # Frontier is good if at least 2 directions lead to unexplored space
        return unexplored_directions >= 2

    def _has_line_of_sight(self,
                          start: np.ndarray,
                          end: np.ndarray,
                          kdtree: KDTree) -> bool:
        """
        Check if there's a clear line-of-sight between start and end.

        Returns True if no obstacles block the path.
        This prevents selecting frontiers behind walls!
        """
        # Sample points along the line from start to end
        distance = np.linalg.norm(end - start)

        # Check every 0.2m along the path
        num_checks = int(distance / 0.2) + 1
        if num_checks < 2:
            num_checks = 2

        for i in range(num_checks + 1):
            # Interpolate point along line
            alpha = i / num_checks
            point = start + alpha * (end - start)

            # Check distance to nearest obstacle
            dist, _ = kdtree.query(point)

            # If any point along the line is too close to obstacle, blocked!
            if dist < self.robot_radius * 1.5:  # Allow some margin
                return False

        return True

    def _score_frontiers(self,
                        candidates: List[np.ndarray],
                        robot_pos: np.ndarray,
                        map_points: np.ndarray,
                        kdtree: KDTree) -> List[SimpleFrontier]:
        """
        Score frontiers by:
        - Travel cost (distance) - 30%
        - Cardinality (nearby frontier points) - 20%
        - Information gain (unexplored area) - 50%
        """
        frontiers = []

        for candidate in candidates:
            # 1. TRAVEL COST: Distance to frontier
            distance = np.linalg.norm(candidate - robot_pos)
            travel_score = np.exp(-distance / 5.0)  # Closer = higher

            # 2. CARDINALITY: Number of nearby frontier candidates
            nearby_count = sum(
                1 for other in candidates
                if np.linalg.norm(other - candidate) < 2.0
            )
            cardinality_score = min(nearby_count / 20.0, 1.0)

            # 3. INFORMATION GAIN: Unexplored area visibility
            # Count how many sample points around frontier are unexplored
            unexplored_count = 0
            sample_radius = 3.5  # LiDAR range
            num_samples = 16

            for angle in np.linspace(0, 2*np.pi, num_samples, endpoint=False):
                sample_point = candidate + sample_radius * np.array([
                    np.cos(angle), np.sin(angle)
                ])

                # If far from obstacles, likely unexplored
                dist, _ = kdtree.query(sample_point)
                if dist > 1.0:  # >1m from obstacles = unexplored
                    unexplored_count += 1

            info_gain_score = unexplored_count / num_samples

            # COMBINED SCORE
            score = (
                0.30 * travel_score +      # α: Travel cost
                0.20 * cardinality_score + # β: Cardinality
                0.50 * info_gain_score     # γ: Information gain
            )

            frontiers.append(SimpleFrontier(candidate, score))

        # Sort by score (best first)
        frontiers.sort(key=lambda f: f.score, reverse=True)

        return frontiers

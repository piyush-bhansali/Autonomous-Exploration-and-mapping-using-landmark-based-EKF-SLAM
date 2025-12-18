#!/usr/bin/env python3


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
    
    def __init__(self, robot_radius: float = 0.22):
        self.robot_radius = robot_radius

    def detect(self,
              map_points: np.ndarray,  # Nx3 array of [x,y,z] points
              robot_pos: np.ndarray,  # [x, y]
              robot_yaw: float) -> List[SimpleFrontier]: 

        if len(map_points) < 100:
            return []

        points_2d = map_points[:, :2]

        kdtree = KDTree(points_2d)

        free_space_points = self._extract_free_space_boundary(points_2d, kdtree)

        if len(free_space_points) < 10:
            # Fallback to full map bounds
            free_space_points = points_2d

        x_min, y_min = free_space_points.min(axis=0)
        x_max, y_max = free_space_points.max(axis=0)

        candidates = self._sample_boundary(x_min, x_max, y_min, y_max)

       
        safe_candidates = []
        for candidate in candidates:
            # Check 1: Not too close to obstacles
            dist, _ = kdtree.query(candidate)
            
            if dist < self.robot_radius * 2.0:  # 0.44m clearance
                continue  # Too close to wall

            if not self._has_open_direction(candidate, kdtree, robot_pos):
                continue  # No open directions, likely dead-end

            safe_candidates.append(candidate)

        if len(safe_candidates) == 0:
            return []

        # CLUSTERING: Group nearby frontiers
        clusters = self._cluster_frontiers(safe_candidates)

        # Score each cluster (not individual points!)
        frontiers = self._score_frontier_clusters(
            clusters, robot_pos, robot_yaw, points_2d, kdtree
        )

        return frontiers

    def _sample_boundary(self,
                        x_min: float, x_max: float,
                        y_min: float, y_max: float) -> List[np.ndarray]:

        offset = 0.5 
        spacing = 0.5  

        candidates = []

        x_samples = np.arange(x_min, x_max + spacing, spacing)
        for x in x_samples:
            candidates.append(np.array([x, y_max - offset]))

        for x in x_samples:
            candidates.append(np.array([x, y_min + offset]))

        y_samples = np.arange(y_min, y_max + spacing, spacing)
        for y in y_samples:
            candidates.append(np.array([x_max - offset, y]))

        for y in y_samples:
            candidates.append(np.array([x_min + offset, y]))

        return candidates

    def _has_open_direction(self,
                            point: np.ndarray,
                            kdtree: KDTree,
                            robot_pos: np.ndarray,
                            check_distance: float = 1.5) -> bool:

        num_directions = 8
        open_count = 0

        for i in range(num_directions):
            angle = i * (2 * np.pi / num_directions)

            # Direction vector
            direction = np.array([np.cos(angle), np.sin(angle)])
            check_point = point + check_distance * direction

            # Check if path is clear (no obstacles within robot_radius)
            path_clear = self._is_direction_clear(point, check_point, kdtree)

            if path_clear:
                # Check if direction leads AWAY from robot (outward exploration)
                # Vector from robot to frontier
                to_frontier = point - robot_pos

                # Dot product: positive if direction continues away from robot
                dot = np.dot(direction, to_frontier)

                if dot > 0:  # Direction leads outward
                    open_count += 1

        # Require 2 open directions to ensure frontier has substantial unexplored space
        return open_count >= 2

    def _is_direction_clear(self,
                            start: np.ndarray,
                            end: np.ndarray,
                            kdtree: KDTree) -> bool:
        
        # Sample along the line
        distance = np.linalg.norm(end - start)
        num_checks = max(3, int(distance / 0.3))  # Check every 0.3m

        for i in range(num_checks + 1):
            alpha = i / num_checks if num_checks > 0 else 0
            check_point = start + alpha * (end - start)

            # Distance to nearest obstacle
            dist, _ = kdtree.query(check_point)

            # Must have clearance
            if dist < self.robot_radius:
                return False

        return True

    def _extract_free_space_boundary(self,
                                     points_2d: np.ndarray,
                                     kdtree: KDTree) -> np.ndarray:
        
        return points_2d

    def _sample_free_space_frontiers(self,
                                     x_min: float, x_max: float,
                                     y_min: float, y_max: float,
                                     robot_pos: np.ndarray,
                                     kdtree: KDTree,
                                     map_points: np.ndarray) -> List[np.ndarray]:
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
        check_distance = 1.5  # Look 1.5m ahead (reduced from 2.0m for narrow corridors)

        unexplored_count = 0
        for angle in np.linspace(0, 2*np.pi, 8, endpoint=False):
            check_point = point + check_distance * np.array([
                np.cos(angle), np.sin(angle)
            ])

            # Distance to nearest mapped obstacle
            dist, _ = kdtree.query(check_point)

            # TUNED FOR NARROW GAPS: Reduced from 1.5m to 1.0m for confined spaces
            if dist > 1.0:  # >1.0m from any mapped point = unexplored
                unexplored_count += 1

        # Frontier if at least 2 directions lead to unexplored space
        return unexplored_count >= 2

    def _cluster_frontiers(self, candidates: List[np.ndarray]) -> List[List[np.ndarray]]:
        if len(candidates) < 2:
            # Not enough points to cluster
            return [[c] for c in candidates]

        # Convert to numpy array for DBSCAN
        points = np.array(candidates)

        clustering = DBSCAN(eps=1.0, min_samples=1).fit(points)

        labels = clustering.labels_

        # Group points by cluster
        clusters = []
        unique_labels = set(labels)

        for label in unique_labels:
            if label == -1:
                
                noise_points = points[labels == -1]
                for point in noise_points:
                    clusters.append([point])
            else:
                
                cluster_points = points[labels == label].tolist()
                clusters.append(cluster_points)

        return clusters

    def _score_frontier_clusters(self,
                                 clusters: List[List[np.ndarray]],
                                 robot_pos: np.ndarray,
                                 robot_yaw: float,
                                 map_points: np.ndarray,
                                 kdtree: KDTree) -> List[SimpleFrontier]:
        frontiers = []

        for cluster in clusters:
            cluster_array = np.array(cluster)
            centroid = cluster_array.mean(axis=0)
            cluster_size = len(cluster)

            distance = np.linalg.norm(centroid - robot_pos)
            travel_score = 1.0 / (1.0 + distance)  

            dx = centroid[0] - robot_pos[0]
            dy = centroid[1] - robot_pos[1]
            angle_to_frontier = np.arctan2(dy, dx)

            angular_diff = np.arctan2(
                np.sin(angle_to_frontier - robot_yaw),
                np.cos(angle_to_frontier - robot_yaw)
            )

            heading_score = 1.0 - (abs(angular_diff) / np.pi)

            size_score = min(cluster_size / 5.0, 1.0)

            score = (
                0.60 * travel_score +    # Distance (60% - balanced priority on proximity)
                0.15 * heading_score +   # Heading (15% - prevent oscillation)
                0.25 * size_score        # Size (25% - favor larger unexplored areas)
            )

            frontiers.append(SimpleFrontier(centroid, score, cluster_size))

        frontiers.sort(key=lambda f: f.score, reverse=True)
        return frontiers

    def _has_line_of_sight(self,
                          start: np.ndarray,
                          end: np.ndarray,
                          kdtree: KDTree) -> bool:
        distance = np.linalg.norm(end - start)

        num_checks = int(distance / 0.2) + 1
        if num_checks < 2:
            num_checks = 2

        for i in range(num_checks + 1):
            
            alpha = i / num_checks
            point = start + alpha * (end - start)

            dist, _ = kdtree.query(point)
            
            if dist < self.robot_radius * 1.5:  
                return False

        return True

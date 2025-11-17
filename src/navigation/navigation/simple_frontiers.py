#!/usr/bin/env python3
"""
Simple Frontier Detector

Finds exploration frontiers at the boundary of the known map.
Scores them by travel cost, cardinality, and information gain.
"""

import numpy as np
from scipy.spatial import KDTree
from typing import List, Tuple, Optional


class SimpleFrontier:
    """A frontier point for exploration"""
    def __init__(self, position: np.ndarray, score: float):
        self.position = position  # [x, y]
        self.score = score


class SimpleFrontierDetector:
    """Detects frontiers at map boundary"""

    def __init__(self, robot_radius: float = 0.22):
        self.robot_radius = robot_radius

    def detect(self,
              map_points: np.ndarray,  # Nx3 array of [x,y,z] points
              robot_pos: np.ndarray) -> List[SimpleFrontier]:  # [x, y]
        """
        Find frontiers at map boundary

        Steps:
        1. Find map bounds
        2. Sample points at boundary (equal spacing)
        3. Filter collision-free points
        4. Score by distance + size + info gain
        5. Return sorted list (best first)
        """
        if len(map_points) < 100:
            return []  # Need minimum map size

        # Extract 2D points (x, y)
        points_2d = map_points[:, :2]

        # Build KD-tree for fast collision checking
        kdtree = KDTree(points_2d)

        # Step 1: Find map boundary
        x_min, y_min = points_2d.min(axis=0)
        x_max, y_max = points_2d.max(axis=0)

        # Step 2: Sample candidate frontiers at boundary
        candidates = self._sample_boundary(x_min, x_max, y_min, y_max)

        # Step 3: Filter collision-free candidates
        safe_candidates = []
        for candidate in candidates:
            # Distance to nearest obstacle
            dist, _ = kdtree.query(candidate)

            # Require 2x robot radius clearance
            if dist >= self.robot_radius * 2.0:
                safe_candidates.append(candidate)

        if len(safe_candidates) == 0:
            return []

        # Step 4 & 5: Score and sort frontiers
        frontiers = self._score_frontiers(
            safe_candidates, robot_pos, points_2d, kdtree
        )

        return frontiers

    def _sample_boundary(self,
                        x_min: float, x_max: float,
                        y_min: float, y_max: float) -> List[np.ndarray]:
        """
        Sample points at map boundary with equal spacing

        Places points 1.0m beyond current map edge
        Spacing = 0.8m between points
        """
        offset = 1.0  # meters beyond boundary
        spacing = 0.8  # meters between samples

        candidates = []

        # Top edge (y = y_max + offset)
        x_samples = np.arange(x_min, x_max + spacing, spacing)
        for x in x_samples:
            candidates.append(np.array([x, y_max + offset]))

        # Bottom edge (y = y_min - offset)
        for x in x_samples:
            candidates.append(np.array([x, y_min - offset]))

        # Right edge (x = x_max + offset)
        y_samples = np.arange(y_min, y_max + spacing, spacing)
        for y in y_samples:
            candidates.append(np.array([x_max + offset, y]))

        # Left edge (x = x_min - offset)
        for y in y_samples:
            candidates.append(np.array([x_min - offset, y]))

        return candidates

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

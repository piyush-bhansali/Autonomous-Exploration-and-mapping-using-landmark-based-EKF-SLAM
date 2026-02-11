#!/usr/bin/env python3

import numpy as np
from typing import Dict, List, Optional, Tuple


class FeatureMap:
    """
    Manages wall and corner features for feature-based mapping.

    Features persist across submaps for continuous tracking and EKF state.
    Walls can be extended when re-observed with longer segments.
    """

    def __init__(self):
        """Initialize empty feature map."""
        self.walls = {}  # landmark_id -> wall_data
        self.corners = {}  # landmark_id -> corner_data

    def add_wall(self, landmark_id: int, rho: float, alpha: float, start_point: np.ndarray,
                 end_point: np.ndarray, points: np.ndarray):
        """
        Add a new wall feature with the given landmark ID.

        Args:
            landmark_id: ID from EKF SLAM (must match EKF's landmark ID)
            rho: Distance parameter (Hessian form)
            alpha: Angle parameter (Hessian form)
            start_point: [x, y] start of wall segment
            end_point: [x, y] end of wall segment
            points: (N x 2) original scan points on wall
        """
        self.walls[landmark_id] = {
            'rho': rho,
            'alpha': alpha,
            'start_point': np.array(start_point),
            'end_point': np.array(end_point),
            'points': np.array(points),
            'observation_count': 1
        }

    def add_corner(self, landmark_id: int, position: np.ndarray):
        """
        Add a new corner feature with the given landmark ID.

        Args:
            landmark_id: ID from EKF SLAM (must match EKF's landmark ID)
            position: [x, y] corner position
        """
        self.corners[landmark_id] = {
            'position': np.array(position),
            'observation_count': 1
        }

    def update_wall_endpoints(self, landmark_id: int, new_start: np.ndarray,
                               new_end: np.ndarray, new_points: np.ndarray):
        """
        Update wall endpoints to extend coverage.

        Args:
            landmark_id: ID of wall to update
            new_start: [x, y] potentially extended start point
            new_end: [x, y] potentially extended end point
            new_points: (N x 2) new scan points on wall
        """
        if landmark_id not in self.walls:
            return

        wall = self.walls[landmark_id]

        # Compute wall tangent direction from Hessian parameters
        # alpha is the angle of the NORMAL to the wall
        # Tangent direction (along the wall) is perpendicular to normal
        alpha = wall['alpha']
        wall_tangent = np.array([-np.sin(alpha), np.cos(alpha)])

        # Project all four points onto wall tangent to determine extent along wall
        current_start_proj = np.dot(wall['start_point'], wall_tangent)
        current_end_proj = np.dot(wall['end_point'], wall_tangent)
        new_start_proj = np.dot(new_start, wall_tangent)
        new_end_proj = np.dot(new_end, wall_tangent)

        # Take minimum and maximum projections to get extended endpoints
        min_proj = min(current_start_proj, current_end_proj, new_start_proj, new_end_proj)
        max_proj = max(current_start_proj, current_end_proj, new_start_proj, new_end_proj)

        # Reconstruct 2D points from projections
        # We need to find points on the line with these projections
        # The line in Hessian form: rho = x*cos(alpha) + y*sin(alpha)
        # Direction along line: perpendicular to normal = [-sin(alpha), cos(alpha)]

        line_tangent = np.array([-np.sin(alpha), np.cos(alpha)])

        # Get a point on the line (closest to origin)
        line_point = wall['rho'] * np.array([np.cos(alpha), np.sin(alpha)])

        # Reconstruct endpoints using projection values
        wall['start_point'] = line_point + min_proj * line_tangent
        wall['end_point'] = line_point + max_proj * line_tangent

        # Merge points (keep old + new) with a maximum limit to prevent unbounded growth
        # Since we only use endpoints for interpolation, we don't need all historical points
        MAX_POINTS_PER_WALL = 500  # Limit to prevent memory issues in long sessions
        wall['points'] = np.vstack([wall['points'], new_points])
        if len(wall['points']) > MAX_POINTS_PER_WALL:
            # Keep evenly spaced subset of points
            indices = np.linspace(0, len(wall['points']) - 1, MAX_POINTS_PER_WALL, dtype=int)
            wall['points'] = wall['points'][indices]

        wall['observation_count'] += 1

    def update_corner_position(self, landmark_id: int, new_position: np.ndarray):
        """
        Update corner position (average with new observation).

        Args:
            landmark_id: ID of corner to update
            new_position: [x, y] newly observed position
        """
        if landmark_id not in self.corners:
            return

        corner = self.corners[landmark_id]
        count = corner['observation_count']

        # Weighted average
        corner['position'] = (corner['position'] * count + new_position) / (count + 1)
        corner['observation_count'] += 1

    def get_wall(self, landmark_id: int) -> Optional[Dict]:
        """Get wall data by ID."""
        return self.walls.get(landmark_id, None)

    def get_corner(self, landmark_id: int) -> Optional[Dict]:
        """Get corner data by ID."""
        return self.corners.get(landmark_id, None)

    def get_all_walls(self) -> Dict[int, Dict]:
        """Get all walls."""
        return self.walls

    def get_all_corners(self) -> Dict[int, Dict]:
        """Get all corners."""
        return self.corners

    def generate_point_cloud(self, spacing: float = 0.05) -> np.ndarray:
        """
        Generate a point cloud from stored features.

        Interpolates points along wall segments at fixed spacing.
        Includes corner positions.

        Args:
            spacing: Distance between interpolated points on walls (meters)

        Returns:
            (N x 3) point cloud array [x, y, z]
        """
        points = []

        # Generate points from walls
        for wall_id, wall in self.walls.items():
            start = wall['start_point']
            end = wall['end_point']

            # Compute wall length
            length = np.linalg.norm(end - start)

            if length < 1e-6:
                # Degenerate wall - just add start point
                points.append([start[0], start[1], 0.0])
                continue

            # Number of points to interpolate
            num_points = max(2, int(np.ceil(length / spacing)))

            # Interpolate points along wall
            for i in range(num_points):
                t = i / (num_points - 1)  # Parameter from 0 to 1
                point = start + t * (end - start)
                points.append([point[0], point[1], 0.0])

        # Add corner points
        for corner_id, corner in self.corners.items():
            pos = corner['position']
            points.append([pos[0], pos[1], 0.0])

        if len(points) == 0:
            # Return empty array with correct shape
            return np.zeros((0, 3))

        return np.array(points)

    def get_feature_count(self) -> Tuple[int, int]:
        """Get number of walls and corners."""
        return len(self.walls), len(self.corners)

    def remove_landmark(self, landmark_id: int):
        """
        Remove a landmark from the feature map (called when EKF prunes it).

        Args:
            landmark_id: ID of landmark to remove
        """
        # Remove from walls if present
        if landmark_id in self.walls:
            del self.walls[landmark_id]

        # Remove from corners if present
        if landmark_id in self.corners:
            del self.corners[landmark_id]

    def clear_features(self):
        """Clear all features (for reset or new submap if needed)."""
        self.walls.clear()
        self.corners.clear()

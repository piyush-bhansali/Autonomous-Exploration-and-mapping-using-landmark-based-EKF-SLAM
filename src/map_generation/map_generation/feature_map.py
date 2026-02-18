#!/usr/bin/env python3

import numpy as np
from typing import Tuple


class FeatureMap:
    

    def __init__(self):
       
        self.walls = {}  # landmark_id -> wall_data
        self.corners = {}  # landmark_id -> corner_data

    def add_wall(self, landmark_id: int, rho: float, alpha: float, start_point: np.ndarray,
                 end_point: np.ndarray):
       
        self.walls[landmark_id] = {
            'rho': rho,
            'alpha': alpha,
            'start_point': np.array(start_point),
            'end_point': np.array(end_point),
            'observation_count': 1
        }

    def add_corner(self, landmark_id: int, position: np.ndarray):
       
        self.corners[landmark_id] = {
            'position': np.array(position),
            'observation_count': 1
        }

    def update_wall_endpoints(self, landmark_id: int, new_start: np.ndarray,
                               new_end: np.ndarray):
       
        if landmark_id not in self.walls:
            return

        wall = self.walls[landmark_id]

        # If endpoints were reset (None), initialize them with new observation
        if wall['start_point'] is None or wall['end_point'] is None:
            wall['start_point'] = new_start
            wall['end_point'] = new_end
            wall['observation_count'] += 1
            return

        alpha = wall['alpha']
        wall_tangent = np.array([-np.sin(alpha), np.cos(alpha)])

        current_start_proj = np.dot(wall['start_point'], wall_tangent)
        current_end_proj = np.dot(wall['end_point'], wall_tangent)
        new_start_proj = np.dot(new_start, wall_tangent)
        new_end_proj = np.dot(new_end, wall_tangent)

        min_proj = min(current_start_proj, current_end_proj, new_start_proj, new_end_proj)
        max_proj = max(current_start_proj, current_end_proj, new_start_proj, new_end_proj)

        line_point = wall['rho'] * np.array([np.cos(alpha), np.sin(alpha)])

        wall['start_point'] = line_point + min_proj * wall_tangent
        wall['end_point'] = line_point + max_proj * wall_tangent

        
        wall['observation_count'] += 1

    def update_wall_hessian(self, landmark_id: int, rho: float, alpha: float):
        """Sync the stored Hessian parameters (rho, alpha) for a wall landmark.

        Called after each EKF wall update so that generate_point_cloud and
        update_wall_endpoints always use the latest EKF-corrected parameters.
        """
        if landmark_id not in self.walls:
            return
        self.walls[landmark_id]['rho'] = rho
        self.walls[landmark_id]['alpha'] = alpha

    def update_corner_position(self, landmark_id: int, new_position: np.ndarray):
        
        if landmark_id not in self.corners:
            return

        corner = self.corners[landmark_id]
        corner['position'] = new_position
        corner['observation_count'] += 1

    def generate_point_cloud(self, spacing: float = 0.05) -> np.ndarray:
        
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
       
        # Remove from walls if present
        if landmark_id in self.walls:
            del self.walls[landmark_id]

        # Remove from corners if present
        if landmark_id in self.corners:
            del self.corners[landmark_id]

    def reset_wall_endpoints(self):
        
        for wall_id, wall in self.walls.items():
            
            wall['start_point'] = None
            wall['end_point'] = None

#!/usr/bin/env python3

import numpy as np
from typing import Tuple, Optional


class FeatureMap:


    def __init__(self):

        self.walls = {}  # landmark_id -> wall_data
        self.corners = {}  # landmark_id -> corner_data

    def add_wall(self, landmark_id: int, rho: float, alpha: float, start_point: np.ndarray,
                 end_point: np.ndarray):
        """Add a new wall landmark.

        Endpoints are stored as scalar extents (t_min, t_max) along the wall
        tangent [-sin(alpha), cos(alpha)], so they remain consistent with
        (rho, alpha) without needing a separate re-projection step.
        """
        tangent = np.array([-np.sin(alpha), np.cos(alpha)])
        t_s = float(np.dot(start_point, tangent))
        t_e = float(np.dot(end_point,   tangent))
        self.walls[landmark_id] = {
            'rho':   rho,
            'alpha': alpha,
            't_min': min(t_s, t_e),
            't_max': max(t_s, t_e),
            'observation_count': 1
        }

    def add_corner(self, landmark_id: int, position: np.ndarray):

        self.corners[landmark_id] = {
            'position': np.array(position),
            'observation_count': 1
        }

    def update_wall_endpoints(self, landmark_id: int, new_start: np.ndarray,
                               new_end: np.ndarray):
        """Extend the stored wall extent to cover new observation endpoints."""
        if landmark_id not in self.walls:
            return

        wall = self.walls[landmark_id]
        alpha   = wall['alpha']
        tangent = np.array([-np.sin(alpha), np.cos(alpha)])

        new_t_s = float(np.dot(new_start, tangent))
        new_t_e = float(np.dot(new_end,   tangent))
        new_t_lo = min(new_t_s, new_t_e)
        new_t_hi = max(new_t_s, new_t_e)

        # If extents not yet initialised (after a reset), set from new observation
        if wall['t_min'] is None or wall['t_max'] is None:
            wall['t_min'] = new_t_lo
            wall['t_max'] = new_t_hi
        else:
            wall['t_min'] = min(wall['t_min'], new_t_lo)
            wall['t_max'] = max(wall['t_max'], new_t_hi)

        wall['observation_count'] += 1

    def update_wall_hessian(self, landmark_id: int, rho: float, alpha: float):
        """Sync the EKF-corrected Hessian parameters (rho, alpha) for a wall.

        t_min/t_max are scalar extents along the wall tangent and remain
        geometrically consistent across Hessian updates — no re-projection needed.
        """
        if landmark_id not in self.walls:
            return
        wall = self.walls[landmark_id]
        wall['rho']   = rho
        wall['alpha'] = alpha

    def update_corner_position(self, landmark_id: int, new_position: np.ndarray):

        if landmark_id not in self.corners:
            return

        corner = self.corners[landmark_id]
        corner['position'] = new_position
        corner['observation_count'] += 1

    def get_wall_endpoints(self, landmark_id: int) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Reconstruct 2D start/end points from stored (rho, alpha, t_min, t_max).

        Returns (start_point, end_point) or None if extents not yet set.
        """
        if landmark_id not in self.walls:
            return None
        wall = self.walls[landmark_id]
        if wall['t_min'] is None or wall['t_max'] is None:
            return None
        alpha   = wall['alpha']
        rho     = wall['rho']
        tangent = np.array([-np.sin(alpha), np.cos(alpha)])
        normal  = np.array([ np.cos(alpha), np.sin(alpha)])
        line_pt = rho * normal
        return (line_pt + wall['t_min'] * tangent,
                line_pt + wall['t_max'] * tangent)

    def generate_point_cloud(self, spacing: float = 0.05) -> np.ndarray:

        points = []

        # Generate points from walls
        for wall_id, wall in self.walls.items():
            t_min = wall['t_min']
            t_max = wall['t_max']

            # Skip walls whose extents were reset to None (start of new submap)
            if t_min is None or t_max is None:
                continue

            alpha   = wall['alpha']
            rho     = wall['rho']
            tangent = np.array([-np.sin(alpha), np.cos(alpha)])
            normal  = np.array([ np.cos(alpha), np.sin(alpha)])
            line_pt = rho * normal

            start = line_pt + t_min * tangent
            end   = line_pt + t_max * tangent
            length = t_max - t_min

            if length < 1e-6:
                points.append([start[0], start[1], 0.0])
                continue

            num_points = max(2, int(np.ceil(length / spacing)))
            for i in range(num_points):
                u = i / (num_points - 1)
                point = start + u * (end - start)
                points.append([point[0], point[1], 0.0])

        # Add corner points
        for corner_id, corner in self.corners.items():
            pos = corner['position']
            points.append([pos[0], pos[1], 0.0])

        if len(points) == 0:
            return np.zeros((0, 3))

        return np.array(points)

    def get_feature_count(self) -> Tuple[int, int]:
        """Get number of walls and corners."""
        return len(self.walls), len(self.corners)

    def remove_landmark(self, landmark_id: int):

        if landmark_id in self.walls:
            del self.walls[landmark_id]

        if landmark_id in self.corners:
            del self.corners[landmark_id]


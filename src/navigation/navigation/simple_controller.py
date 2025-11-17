#!/usr/bin/env python3

import numpy as np
from typing import List, Tuple


class SimplePurePursuit:
    """Pure pursuit path follower"""

    def __init__(self,
                 lookahead_distance: float = 1.2,
                 linear_velocity: float = 0.2,
                 max_angular_velocity: float = 0.8):
        
        self.lookahead = lookahead_distance
        self.v = linear_velocity
        self.max_w = max_angular_velocity

    def compute_control(self,
                       robot_pos: np.ndarray,  # [x, y]
                       robot_yaw: float,        # radians
                       path: List[np.ndarray],  # List of [x, y] waypoints
                       current_waypoint_index: int) -> Tuple[float, float]:
        """
        Compute velocity commands to follow path

        Returns:
            (linear_vel, angular_vel) in (m/s, rad/s)
        """
        # Find lookahead point on path
        lookahead_point = self._find_lookahead_point(
            robot_pos, path, current_waypoint_index
        )

        if lookahead_point is None:
            # Path complete or no valid lookahead
            return 0.0, 0.0

        # Compute steering angle to lookahead point
        angular_vel = self._compute_steering(robot_pos, robot_yaw, lookahead_point)

        # Velocity limiting: Scale down linear velocity if angular velocity exceeds limit
        # This respects kinematic constraints while maintaining path curvature
        if abs(angular_vel) > self.max_w:
            # Calculate required curvature
            distance = np.linalg.norm(lookahead_point - robot_pos)
            if distance > 0.01:
                curvature = angular_vel / self.v
                # Scale linear velocity to respect max angular velocity
                v_cmd = min(self.v, self.max_w / max(0.001, abs(curvature)))
                angular_vel = np.clip(v_cmd * curvature, -self.max_w, self.max_w)
            else:
                v_cmd = self.v
                angular_vel = np.clip(angular_vel, -self.max_w, self.max_w)
        else:
            v_cmd = self.v

        # Goal slowdown: Reduce velocity when approaching final waypoint
        # This reduces overshoot and provides smoother goal arrival
        dist_to_goal = np.linalg.norm(path[-1] - robot_pos)
        if dist_to_goal < 2.0:  # Within 2m of goal
            slowdown_factor = max(0.3, dist_to_goal / 2.0)  # Slow to 30% minimum
            v_cmd *= slowdown_factor

        return v_cmd, angular_vel

    def _find_lookahead_point(self,
                             robot_pos: np.ndarray,
                             path: List[np.ndarray],
                             start_index: int) -> np.ndarray:
        """Find point on path at lookahead distance"""

        # Search forward from current waypoint
        for i in range(start_index, len(path)):
            waypoint = path[i]
            distance = np.linalg.norm(waypoint - robot_pos)

            # Return first point beyond lookahead distance
            if distance >= self.lookahead:
                return waypoint

        # If no point beyond lookahead, return last waypoint
        if len(path) > 0:
            return path[-1]

        return None

    def _compute_steering(self,
                         robot_pos: np.ndarray,
                         robot_yaw: float,
                         target_point: np.ndarray) -> float:
        
        # Vector from robot to target
        dx = target_point[0] - robot_pos[0]
        dy = target_point[1] - robot_pos[1]

        # Angle to target in global frame
        target_angle = np.arctan2(dy, dx)

        # Angle error (difference from robot heading)
        alpha = target_angle - robot_yaw

        # Normalize to [-π, π]
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        # Distance to target
        distance = np.sqrt(dx**2 + dy**2)

        # Pure pursuit formula: curvature = 2*sin(α)/L
        # Angular velocity = v * curvature
        if distance > 0.01:  # Avoid division by zero
            curvature = (2.0 * np.sin(alpha)) / distance
            angular_velocity = self.v * curvature
        else:
            angular_velocity = 0.0

        return angular_velocity

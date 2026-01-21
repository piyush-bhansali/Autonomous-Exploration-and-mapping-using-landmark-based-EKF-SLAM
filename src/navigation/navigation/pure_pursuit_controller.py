#!/usr/bin/env python3

import numpy as np
from typing import List, Tuple


class PurePursuit:

    def __init__(self,
                 max_linear_velocity: float = 0.2,
                 max_angular_velocity: float = 0.8):

        self.v_max = max_linear_velocity  # Maximum linear velocity
        self.max_w = max_angular_velocity  # Maximum angular velocity

        self.lookahead = 0.8  # Lookahead distance for path following
        self.v_min = 0.08  # Minimum linear velocity
        self.alpha = 0.4  # Angular velocity smoothing factor
        self.goal_tolerance = 0.3  # Distance to goal considered "reached"
        self.velocity_gain = 0.5  # Velocity reduction factor on sharp turns

        self.omega_smooth = 0.0  # Smoothed angular velocity
        self.prev_time = None    # For tracking control frequency

    def compute_control(self,
                       robot_pos: np.ndarray,  # [x, y]
                       robot_yaw: float,        # radians
                       path: List[np.ndarray],  # List of [x, y] waypoints
                       current_waypoint_index: int) -> Tuple[float, float]:

        dist_to_goal = np.linalg.norm(path[-1] - robot_pos)
        if dist_to_goal < self.goal_tolerance:
            self.omega_smooth = 0.0
            return 0.0, 0.0

        lookahead_point = self._find_lookahead_point(
            robot_pos, path, current_waypoint_index
        )

        if lookahead_point is None:
            self.omega_smooth = 0.0
            return 0.0, 0.0

        omega_magnitude = abs(self.omega_smooth)
        velocity_scale = 1.0 - self.velocity_gain * (omega_magnitude / self.max_w)
        velocity_scale = np.clip(velocity_scale, 0.3, 1.0)
        v_adaptive = self.v_max * velocity_scale
        v_adaptive = np.clip(v_adaptive, self.v_min, self.v_max)

        omega_desired = self._compute_steering(robot_pos, robot_yaw, lookahead_point, v_adaptive)

        omega_desired = np.clip(omega_desired, -self.max_w, self.max_w)

        self.omega_smooth = (self.alpha * omega_desired +
                            (1 - self.alpha) * self.omega_smooth)

        return v_adaptive, self.omega_smooth

    def _find_lookahead_point(self,
                             robot_pos: np.ndarray,
                             path: List[np.ndarray],
                             start_index: int) -> np.ndarray:

        for i in range(start_index, len(path)):
            waypoint = path[i]
            distance = np.linalg.norm(waypoint - robot_pos)

            if distance >= self.lookahead:
                return waypoint

        if len(path) > 0:
            return path[-1]

        return None

    def _compute_steering(self,
                         robot_pos: np.ndarray,
                         robot_yaw: float,
                         target_point: np.ndarray,
                         current_velocity: float = None) -> float:

        dx = target_point[0] - robot_pos[0]
        dy = target_point[1] - robot_pos[1]

        target_angle = np.arctan2(dy, dx)

        alpha = target_angle - robot_yaw

        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))

        distance = np.sqrt(dx**2 + dy**2)

        if distance < 0.05:
            return 0.0

        curvature = (2.0 * np.sin(alpha)) / distance

        velocity = current_velocity if current_velocity is not None else self.v_max
        angular_velocity = velocity * curvature

        return angular_velocity

    def reset(self):

        self.omega_smooth = 0.0
        self.prev_time = None

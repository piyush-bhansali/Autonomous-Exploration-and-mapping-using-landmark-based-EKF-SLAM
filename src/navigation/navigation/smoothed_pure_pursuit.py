#!/usr/bin/env python3

import numpy as np
from typing import List, Tuple


class SmoothedPurePursuit:
    

    def __init__(self,
                 lookahead_distance: float = 0.6,
                 max_linear_velocity: float = 0.2,
                 min_linear_velocity: float = 0.1,
                 max_angular_velocity: float = 0.8,
                 angular_smoothing_factor: float = 0.3,
                 goal_tolerance: float = 0.3,
                 velocity_gain: float = 0.5):

        # BUG FIX #7: Validate smoothing factor is in valid range [0, 1]
        if not (0.0 <= angular_smoothing_factor <= 1.0):
            raise ValueError(f"angular_smoothing_factor must be in [0, 1], got {angular_smoothing_factor}")

        self.lookahead = lookahead_distance
        self.v_max = max_linear_velocity  # Maximum linear velocity
        self.v_min = min_linear_velocity  # Minimum linear velocity
        self.max_w = max_angular_velocity
        self.alpha = angular_smoothing_factor
        self.goal_tolerance = goal_tolerance
        self.velocity_gain = velocity_gain  # How much to reduce velocity on turns

        # State variables for smoothing
        self.omega_smooth = 0.0  # Smoothed angular velocity
        self.prev_time = None    # For tracking control frequency

        # Statistics (for debugging/tuning)
        self.stats = {
            'max_omega_change': 0.0,
            'avg_omega_change': 0.0,
            'control_count': 0
        }

    def compute_control(self,
                       robot_pos: np.ndarray,  # [x, y]
                       robot_yaw: float,        # radians
                       path: List[np.ndarray],  # List of [x, y] waypoints
                       current_waypoint_index: int) -> Tuple[float, float]:
        
        # Check if goal reached
        dist_to_goal = np.linalg.norm(path[-1] - robot_pos)
        if dist_to_goal < self.goal_tolerance:
            self.omega_smooth = 0.0  # Reset for next path
            return 0.0, 0.0

        # Find lookahead point on path
        lookahead_point = self._find_lookahead_point(
            robot_pos, path, current_waypoint_index
        )

        if lookahead_point is None:
            # No valid lookahead - stop
            self.omega_smooth = 0.0
            return 0.0, 0.0

        # First compute adaptive velocity based on previous smoothed angular velocity
        # ADAPTIVE VELOCITY: Slow down on sharp turns
        # v = v_max * (1 - k * |ω|) where k = velocity_gain
        # Higher angular velocity → lower linear velocity
        omega_magnitude = abs(self.omega_smooth)
        velocity_scale = 1.0 - self.velocity_gain * (omega_magnitude / self.max_w)
        velocity_scale = np.clip(velocity_scale, 0.5, 1.0)  # Keep between 50% and 100% speed

        v_adaptive = self.v_max * velocity_scale
        v_adaptive = np.clip(v_adaptive, self.v_min, self.v_max)

        # Compute DESIRED angular velocity using ADAPTIVE velocity (not v_max)
        omega_desired = self._compute_steering(robot_pos, robot_yaw, lookahead_point, v_adaptive)

        # Clamp desired angular velocity to kinematic limits
        omega_desired = np.clip(omega_desired, -self.max_w, self.max_w)

        # CRITICAL: Apply exponential moving average smoothing
        # This is what makes the motion predictable to the EKF!
        omega_change = abs(omega_desired - self.omega_smooth)
        self.omega_smooth = (self.alpha * omega_desired +
                            (1 - self.alpha) * self.omega_smooth)

        # Update statistics
        self._update_stats(omega_change)

        # Return ADAPTIVE linear velocity + SMOOTHED angular velocity
        return v_adaptive, self.omega_smooth

    def _find_lookahead_point(self,
                             robot_pos: np.ndarray,
                             path: List[np.ndarray],
                             start_index: int) -> np.ndarray:
        
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
                         target_point: np.ndarray,
                         current_velocity: float = None) -> float:

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

        # Pure pursuit formula: ω = v * (2*sin(α)/L)
        # BUG FIX #5: Increased safety margin from 0.01m to 0.05m to prevent numerical instability
        if distance < 0.05:  # 5cm safety margin
            return 0.0

        curvature = (2.0 * np.sin(alpha)) / distance

        # BUG FIX #6: Use actual velocity for curvature calculation, not v_max
        # If no velocity provided, use v_max as fallback
        velocity = current_velocity if current_velocity is not None else self.v_max
        angular_velocity = velocity * curvature

        return angular_velocity

    def _update_stats(self, omega_change: float):
        
        self.stats['control_count'] += 1
        self.stats['max_omega_change'] = max(self.stats['max_omega_change'], omega_change)

        # Running average of omega changes
        n = self.stats['control_count']
        prev_avg = self.stats['avg_omega_change']
        self.stats['avg_omega_change'] = (prev_avg * (n-1) + omega_change) / n

    def reset(self):
        
        self.omega_smooth = 0.0
        self.prev_time = None
        self.stats = {
            'max_omega_change': 0.0,
            'avg_omega_change': 0.0,
            'control_count': 0
        }

    def get_statistics(self) -> dict:
        
        return {
            **self.stats,
            'smoothing_factor': self.alpha,
            'max_velocity': self.v_max,
            'current_omega_smooth': self.omega_smooth
        }

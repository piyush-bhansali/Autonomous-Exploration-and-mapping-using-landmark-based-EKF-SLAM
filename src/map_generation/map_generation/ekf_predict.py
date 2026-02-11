#!/usr/bin/env python3

import numpy as np
from typing import Dict


class BaseEKF:
    """
    Base Extended Kalman Filter with prediction step.

    Common foundation for both ICP-based (pose-only) and Feature-based (landmark) SLAM.
    Contains shared prediction logic and covariance conditioning.
    """

    def __init__(self):
        """Initialize base EKF state."""
        # Robot state: [x, y, θ] (can be augmented by subclasses)
        self.state = np.zeros(3)

        # Covariance matrix
        self.P = np.eye(3) * 0.01

        # Process noise (odometry)
        # Q[0,0]: distance noise coefficient
        # Q[1,1]: rotation noise coefficient
        self.Q = np.diag([
            0.01,    # Distance noise
            0.005    # Rotation noise
        ])

        # Tracking
        self.initialized = False
        self.current_scan_number = 0

    def initialize(self, x: float, y: float, theta: float):
        """
        Initialize robot pose.

        Args:
            x: Initial x position (meters)
            y: Initial y position (meters)
            theta: Initial orientation (radians)
        """
        self.state = np.array([x, y, theta])
        self.P = np.eye(len(self.state)) * 0.01
        self.initialized = True

    def predict_with_relative_motion(self, delta_d: float, delta_theta: float):
        """
        EKF prediction step using relative motion model.

        Args:
            delta_d: Distance traveled (meters)
            delta_theta: Rotation change (radians)
        """
        if not self.initialized:
            return

        self.current_scan_number += 1

        x, y, theta = self.state[0:3]

        # Predict robot pose using mid-point integration
        theta_mid = theta + delta_theta / 2.0
        x_new = x + delta_d * np.cos(theta_mid)
        y_new = y + delta_d * np.sin(theta_mid)
        theta_new = theta + delta_theta
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))

        self.state[0:3] = [x_new, y_new, theta_new]

        # Jacobian of motion model wrt state
        n = len(self.state)
        F = np.eye(n)
        F[0:3, 0:3] = np.array([
            [1.0, 0.0, -delta_d * np.sin(theta_mid)],
            [0.0, 1.0,  delta_d * np.cos(theta_mid)],
            [0.0, 0.0,  1.0]
        ])

        # Jacobian of motion model wrt noise
        G = np.zeros((n, 2))
        G[0:3, :] = np.array([
            [np.cos(theta_mid), -0.5 * delta_d * np.sin(theta_mid)],
            [np.sin(theta_mid),  0.5 * delta_d * np.cos(theta_mid)],
            [0.0,                1.0]
        ])

        # Motion-scaled process noise
        motion_distance = abs(delta_d)
        motion_rotation = abs(delta_theta)

        # Add minimum variance to prevent zero when stationary
        min_distance_var = 0.0001    # (1cm)²
        min_rotation_var = 0.000001  # (~0.06°)²

        sigma_d_sq = self.Q[0, 0] * motion_distance**2 + min_distance_var
        sigma_theta_sq = self.Q[1, 1] * motion_rotation**2 + min_rotation_var

        Q_scaled = np.diag([sigma_d_sq, sigma_theta_sq])

        # Covariance prediction
        self.P = F @ self.P @ F.T + G @ Q_scaled @ G.T

        # Condition covariance for numerical stability
        self.P = self._condition_covariance(self.P)

    def _condition_covariance(self, P: np.ndarray,
                             min_eigenvalue: float = 1e-6,
                             max_eigenvalue: float = 100.0) -> np.ndarray:
        """
        Condition covariance matrix for numerical stability.

        Ensures:
        - Symmetry (fixes numerical asymmetry)
        - Positive definiteness (clamps eigenvalues)
        - Bounded condition number (prevents ill-conditioning)

        Args:
            P: Covariance matrix to condition
            min_eigenvalue: Minimum eigenvalue (ensures positive definiteness)
            max_eigenvalue: Maximum eigenvalue (prevents unbounded uncertainty)

        Returns:
            Conditioned covariance matrix
        """
        # Force symmetry
        P_sym = (P + P.T) / 2.0

        # Eigenvalue decomposition
        eigvals, eigvecs = np.linalg.eigh(P_sym)

        # Clamp eigenvalues to reasonable range
        eigvals_clamped = np.clip(eigvals, min_eigenvalue, max_eigenvalue)

        # Reconstruct conditioned covariance
        P_conditioned = eigvecs @ np.diag(eigvals_clamped) @ eigvecs.T

        # Add small regularization to ensure strict positive definiteness
        n = len(P)
        P_conditioned += np.eye(n) * 1e-9

        return P_conditioned

    def get_state(self) -> Dict:
        """
        Get current robot pose.

        Returns:
            Dictionary with 'x', 'y', 'theta' keys
        """
        return {
            'x': float(self.state[0]),
            'y': float(self.state[1]),
            'theta': float(self.state[2])
        }

#!/usr/bin/env python3

import numpy as np
from map_generation.ekf_predict import BaseEKF


class ICPPoseEKF(BaseEKF):
    """
    Pose-only EKF for ICP-based SLAM.

    State vector: [x, y, θ] (3-dimensional, robot pose only)
    No landmarks are tracked - this is purely for pose estimation with ICP measurements.

    Used in ICP mapping mode where:
    - Prediction: odometry-based motion model
    - Update: ICP scan-to-map alignment provides direct pose measurements
    """

    def __init__(self):
        """Initialize pose-only EKF for ICP mode."""
        super().__init__()
        # State remains 3D: [x, y, θ]
        # Covariance is 3x3

    def update(self,
               x: float,
               y: float,
               theta: float,
               measurement_covariance: np.ndarray,
               measurement_type: str = "icp"):
        """
        EKF update with direct pose measurement (e.g., from ICP).

        Args:
            x: Measured x position (meters)
            y: Measured y position (meters)
            theta: Measured orientation (radians)
            measurement_covariance: 3x3 measurement noise covariance
            measurement_type: Source of measurement (default: "icp")
        """
        if not self.initialized:
            return

        n = len(self.state)  # Should be 3
        z = np.array([x, y, theta])
        z_pred = self.state[0:3]

        # Innovation with angle normalization
        innovation = z - z_pred
        innovation[2] = np.arctan2(np.sin(innovation[2]), np.cos(innovation[2]))

        # Observation model: direct pose measurement
        # H = I (3x3 identity) for pose-only state
        H = np.zeros((3, n))
        H[0:3, 0:3] = np.eye(3)

        # Innovation covariance
        S = H @ self.P @ H.T + measurement_covariance

        # Kalman gain
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Singular innovation covariance - skip update
            return

        # State update
        self.state = self.state + K @ innovation
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

        # Covariance update (Joseph form for numerical stability)
        I = np.eye(n)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ measurement_covariance @ K.T

        # Condition covariance
        self.P = self._condition_covariance(self.P)

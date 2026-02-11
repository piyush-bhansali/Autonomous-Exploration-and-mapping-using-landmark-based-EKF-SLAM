#!/usr/bin/env python3

import numpy as np
from typing import Dict, Optional

from map_generation.ekf_update_icp import ICPPoseEKF


class ICPSLAMManager:
    """
    Manager for ICP-based SLAM.

    Lightweight wrapper around ICPPoseEKF for pose-only state estimation.
    Provides clean interface for ICP mapping mode without landmark tracking.

    Architecture:
    - Uses ICPPoseEKF (3-state: x, y, θ)
    - No feature extraction
    - No data association
    - No landmark management
    - Only odometry prediction + ICP pose updates
    """

    def __init__(self):
        """Initialize ICP SLAM manager with pose-only EKF."""
        self.ekf = ICPPoseEKF()

    def initialize_pose(self, x: float, y: float, theta: float):
        """
        Initialize robot pose.

        Args:
            x: Initial x position (meters)
            y: Initial y position (meters)
            theta: Initial orientation (radians)
        """
        self.ekf.initialize(x, y, theta)

    def predict_motion(self, delta_d: float, delta_theta: float):
        """
        Perform EKF prediction step with relative motion.

        Args:
            delta_d: Distance traveled (meters)
            delta_theta: Rotation (radians)
        """
        if not self.ekf.initialized:
            return

        self.ekf.predict_with_relative_motion(delta_d, delta_theta)

    def update_pose_measurement(self,
                                x: float,
                                y: float,
                                theta: float,
                                measurement_covariance: Optional[np.ndarray] = None):
        """
        Update EKF with pose measurement (e.g., from ICP alignment).

        Args:
            x: Measured x position (meters)
            y: Measured y position (meters)
            theta: Measured orientation (radians)
            measurement_covariance: 3x3 measurement covariance (optional)
        """
        if not self.ekf.initialized:
            return

        # Default covariance if not provided
        if measurement_covariance is None:
            measurement_covariance = np.diag([0.1, 0.1, 0.05]) ** 2

        self.ekf.update(x, y, theta, measurement_covariance, measurement_type="icp")

    def get_robot_pose(self) -> Optional[Dict]:
        """
        Get current robot pose from EKF.

        Returns:
            Dictionary with 'x', 'y', 'theta' or None if not initialized
        """
        if not self.ekf.initialized:
            return None

        return self.ekf.get_state()

    def is_initialized(self) -> bool:
        """Check if EKF is initialized."""
        return self.ekf.initialized

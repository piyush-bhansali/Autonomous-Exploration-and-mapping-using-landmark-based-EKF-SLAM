#!/usr/bin/env python3

import csv
import numpy as np
from typing import Dict


class ConfidenceTracker:
    """Tracks and logs submap confidence metrics for thesis analysis."""

    def __init__(self, csv_file_path: str, logger=None):
        """
        Initialize confidence tracker.

        Args:
            csv_file_path: Path to CSV file for logging confidence data
            logger: ROS logger instance (optional, unused)
        """
        self.csv_file_path = csv_file_path

        # Open CSV file for writing
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'submap_id', 'timestamp', 'confidence', 'information',
            'robot_uncertainty', 'num_landmarks'
        ])

    def log_confidence(self, submap_id: int, confidence_metrics: Dict):
        """
        Log submap confidence metrics to CSV file.

        Args:
            submap_id: Submap identifier
            confidence_metrics: Dict with keys: confidence, information,
                               robot_uncertainty, num_landmarks, timestamp
        """
        self.csv_writer.writerow([
            submap_id,
            confidence_metrics['timestamp'],
            confidence_metrics['confidence'],
            confidence_metrics['information'],
            confidence_metrics['robot_uncertainty'],
            confidence_metrics['num_landmarks']
        ])
        self.csv_file.flush()  # Ensure data is written immediately

    def close(self):
        """Close CSV file and cleanup."""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()


def compute_ekf_confidence(ekf_slam, ekf_initialized: bool, current_time_ns: int) -> Dict:
    """
    Compute submap confidence using information-theoretic metric.

    Information matrix = inverse of covariance.
    Higher information = more precise state estimate.

    Args:
        ekf_slam: LandmarkEKFSLAM instance
        ekf_initialized: Whether EKF has been initialized
        current_time_ns: Current time in nanoseconds

    Returns:
        dict: Confidence metrics including:
            - confidence: Normalized confidence score [0, 1]
            - information: Trace of information matrix
            - robot_uncertainty: Trace of pose covariance
            - num_landmarks: Number of landmarks in EKF state
            - timestamp: Current time in nanoseconds
    """
    if not ekf_initialized:
        return {
            'confidence': 0.0,
            'information': 0.0,
            'robot_uncertainty': float('inf'),
            'num_landmarks': 0,
            'timestamp': current_time_ns
        }

    # Robot pose covariance (x, y, theta)
    P_robot = ekf_slam.P[0:3, 0:3]

    # Robot uncertainty (trace of covariance)
    robot_uncertainty = np.trace(P_robot)

    # Information matrix (inverse of covariance)
    try:
        I_robot = np.linalg.inv(P_robot)

        # Information = trace of information matrix
        information = np.trace(I_robot)

        # Normalize to [0, 1] range using exponential decay
        # Tunable scaling factor: 10.0 (adjust based on typical information values)
        confidence = 1.0 - np.exp(-information / 10.0)

    except np.linalg.LinAlgError:
        # Singular covariance = infinite uncertainty, zero information
        information = 0.0
        confidence = 0.0

    return {
        'confidence': float(confidence),
        'information': float(information),
        'robot_uncertainty': float(robot_uncertainty),
        'num_landmarks': len(ekf_slam.landmarks),
        'timestamp': current_time_ns
    }

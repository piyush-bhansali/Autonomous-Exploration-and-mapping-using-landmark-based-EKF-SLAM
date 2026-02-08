#!/usr/bin/env python3

import csv
import numpy as np
from typing import Optional, Dict
from geometry_msgs.msg import PoseStamped

from map_generation.utils import quaternion_to_yaw


class GroundTruthTracker:
    """Tracks ground truth pose and compares it with EKF estimates."""

    def __init__(self, csv_file_path: str, logger=None):
        """
        Initialize ground truth tracker.

        Args:
            csv_file_path: Path to CSV file for logging comparison data
            logger: ROS logger instance (optional)
        """
        self.ground_truth_pose: Optional[Dict] = None
        self.csv_file_path = csv_file_path
        self.logger = logger

        # Open CSV file for writing
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'ekf_x', 'ekf_y', 'ekf_theta',
            'gt_x', 'gt_y', 'gt_theta', 'pos_error', 'orient_error'
        ])

        if self.logger:
            self.logger.info(f'Ground truth tracking enabled. Data will be saved to: {self.csv_file_path}')

    def update_ground_truth(self, msg: PoseStamped, current_timestamp_ns: int):
        """
        Update ground truth pose from PoseStamped message.

        Args:
            msg: PoseStamped message containing ground truth pose
            current_timestamp_ns: Current timestamp in nanoseconds
        """
        x_gt = msg.pose.position.x
        y_gt = msg.pose.position.y

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        theta_gt = quaternion_to_yaw(qx, qy, qz, qw)

        self.ground_truth_pose = {
            'x': x_gt,
            'y': y_gt,
            'theta': theta_gt,
            'timestamp': current_timestamp_ns
        }

    def log_comparison(self, ekf_pose: Dict, ekf_initialized: bool, current_timestamp_ns: int):
        """
        Log comparison between EKF and ground truth poses.

        Args:
            ekf_pose: Dictionary with keys 'x', 'y', 'theta' for EKF pose
            ekf_initialized: Whether EKF has been initialized
            current_timestamp_ns: Current timestamp in nanoseconds
        """
        if not ekf_initialized or self.ground_truth_pose is None or ekf_pose is None:
            return

        # Calculate position error
        pos_error = np.sqrt(
            (ekf_pose['x'] - self.ground_truth_pose['x'])**2 +
            (ekf_pose['y'] - self.ground_truth_pose['y'])**2
        )

        # Calculate orientation error (normalized angle difference)
        angle_diff = ekf_pose['theta'] - self.ground_truth_pose['theta']
        orient_error = abs(np.arctan2(np.sin(angle_diff), np.cos(angle_diff)))

        # Write to CSV
        timestamp_sec = current_timestamp_ns / 1e9
        self.csv_writer.writerow([
            timestamp_sec,
            ekf_pose['x'],
            ekf_pose['y'],
            ekf_pose['theta'],
            self.ground_truth_pose['x'],
            self.ground_truth_pose['y'],
            self.ground_truth_pose['theta'],
            pos_error,
            orient_error
        ])
        self.csv_file.flush()  # Ensure data is written immediately

    def close(self):
        """Close CSV file and cleanup."""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            if self.logger:
                self.logger.info(f'Ground truth tracking data saved to: {self.csv_file_path}')

#!/usr/bin/env python3

import csv
import numpy as np
from typing import Dict


CONF_TAU = 1e6


class ConfidenceTracker:
    
    def __init__(self, csv_file_path: str, logger=None):
       
        self.csv_file_path = csv_file_path

        # Open CSV file for writing
        self.csv_file = open(self.csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'submap_id', 'timestamp', 'confidence', 'information',
            'robot_x_std', 'robot_y_std', 'robot_theta_std',
            'num_landmarks'
        ])

    def log_confidence(self, submap_id: int, confidence_metrics: Dict):
       
        self.csv_writer.writerow([
            submap_id,
            confidence_metrics['timestamp'],
            confidence_metrics['confidence'],
            confidence_metrics['information'],
            confidence_metrics['robot_x_std'],
            confidence_metrics['robot_y_std'],
            confidence_metrics['robot_theta_std'],
            confidence_metrics['num_landmarks']
        ])
        self.csv_file.flush()  # Ensure data is written immediately

    def close(self):
        
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()


def _landmark_information_sum(ekf_slam) -> float:
    
    if not hasattr(ekf_slam, 'landmarks'):
        return 0.0

    info_sum = 0.0

    for lm_data in ekf_slam.landmarks.values():
        idx = lm_data.get('state_index')
        if idx is None:
            continue

        if idx + 1 >= ekf_slam.P.shape[0]:
            continue

        P_lm = ekf_slam.P[idx:idx + 2, idx:idx + 2]
        if P_lm.shape != (2, 2):
            continue

        try:
            I_lm = np.linalg.inv(P_lm)
        except np.linalg.LinAlgError:
            I_lm = np.linalg.pinv(P_lm)

        info_sum += float(np.trace(I_lm))

    return float(info_sum)


def compute_ekf_confidence(ekf_slam, ekf_initialized: bool, current_time_ns: int) -> Dict:
    
    if not ekf_initialized:
        return {
            'confidence': 0.0,
            'information': 0.0,
            'robot_x_std': float('inf'),
            'robot_y_std': float('inf'),
            'robot_theta_std': float('inf'),
            'num_landmarks': 0,
            'timestamp': current_time_ns
        }

    P_robot = ekf_slam.P[0:3, 0:3]

    robot_x_std = float(np.sqrt(max(P_robot[0, 0], 0.0)))
    robot_y_std = float(np.sqrt(max(P_robot[1, 1], 0.0)))
    robot_theta_std = float(np.sqrt(max(P_robot[2, 2], 0.0)))

    information = _landmark_information_sum(ekf_slam)

    if information <= 0.0:
        confidence = 0.0
    else:
        confidence = 1.0 - np.exp(-information / CONF_TAU)

    num_landmarks = len(ekf_slam.landmarks) if hasattr(ekf_slam, 'landmarks') else 0

    return {
        'confidence': float(confidence),
        'information': float(information),
        'robot_x_std': robot_x_std,
        'robot_y_std': robot_y_std,
        'robot_theta_std': robot_theta_std,
        'num_landmarks': int(num_landmarks),
        'timestamp': current_time_ns
    }


def log_groundtruth_row(csv_writer, timestamp_ns: int,
                        ekf_x: float, ekf_y: float, ekf_theta: float,
                        gt_x: float, gt_y: float, gt_theta: float,
                        pos_error: float, orient_error: float,
                        P_xx: float, P_yy: float, P_thetatheta: float):
    """Write a row to the ground truth comparison CSV.

    Args:
        csv_writer: CSV writer object
        timestamp_ns: Timestamp in nanoseconds
        ekf_x, ekf_y, ekf_theta: EKF estimated pose
        gt_x, gt_y, gt_theta: Ground truth pose (relative to initial)
        pos_error: Position error magnitude
        orient_error: Orientation error in radians
        P_xx, P_yy, P_thetatheta: Diagonal covariance elements
    """
    csv_writer.writerow([
        timestamp_ns,
        f'{ekf_x:.6f}', f'{ekf_y:.6f}', f'{ekf_theta:.6f}',
        f'{gt_x:.6f}', f'{gt_y:.6f}', f'{gt_theta:.6f}',
        f'{pos_error:.6f}', f'{orient_error:.6f}',
        f'{P_xx:.9f}', f'{P_yy:.9f}', f'{P_thetatheta:.9f}'
    ])


def init_groundtruth_csv(csv_path: str):
    """Initialize ground truth CSV file with headers.

    Args:
        csv_path: Path to CSV file

    Returns:
        Tuple of (csv_file, csv_writer)
    """
    import os
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    csv_file = open(csv_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        'timestamp_ns',
        'ekf_x', 'ekf_y', 'ekf_theta',
        'gt_x', 'gt_y', 'gt_theta',
        'pos_error', 'orient_error',
        'P_xx', 'P_yy', 'P_thetatheta'
    ])
    csv_file.flush()
    return csv_file, csv_writer

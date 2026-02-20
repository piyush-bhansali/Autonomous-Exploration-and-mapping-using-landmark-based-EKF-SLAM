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
            'robot_uncertainty', 'num_landmarks'
        ])

    def log_confidence(self, submap_id: int, confidence_metrics: Dict):
       
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
            'robot_uncertainty': float('inf'),
            'num_landmarks': 0,
            'timestamp': current_time_ns
        }

    P_robot = ekf_slam.P[0:3, 0:3]

    robot_uncertainty = np.trace(P_robot)

    information = _landmark_information_sum(ekf_slam)

    if information <= 0.0:
        confidence = 0.0
    else:
        confidence = 1.0 - np.exp(-information / CONF_TAU)

    num_landmarks = len(ekf_slam.landmarks) if hasattr(ekf_slam, 'landmarks') else 0

    return {
        'confidence': float(confidence),
        'information': float(information),
        'robot_uncertainty': float(robot_uncertainty),
        'num_landmarks': int(num_landmarks),
        'timestamp': current_time_ns
    }

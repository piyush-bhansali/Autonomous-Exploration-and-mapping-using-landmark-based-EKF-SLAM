#!/usr/bin/env python3

import csv
import numpy as np
from typing import Dict


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

    try:
        I_robot = np.linalg.inv(P_robot)

        information = np.trace(I_robot)

        confidence = 1.0 - np.exp(-information / 10.0)

    except np.linalg.LinAlgError:
        
        information = 0.0
        confidence = 0.0

    num_landmarks = len(ekf_slam.landmarks) if hasattr(ekf_slam, 'landmarks') else 0

    return {
        'confidence': float(confidence),
        'information': float(information),
        'robot_uncertainty': float(robot_uncertainty),
        'num_landmarks': int(num_landmarks),
        'timestamp': current_time_ns
    }

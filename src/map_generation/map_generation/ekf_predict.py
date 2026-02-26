#!/usr/bin/env python3

import numpy as np
from typing import Dict


class BaseEKF:
    
    def __init__(self):
              
        self.state = np.zeros(3)
      
        self.P = np.eye(3) * 0.001
       
        self.Q = np.diag([
            0.000008,    # Distance noise
            0.00002    # Rotation noise
        ])

        self.initialized = False
        self.current_scan_number = 0

    def initialize(self, x: float, y: float, theta: float):
        
        self.state = np.array([x, y, theta])
        self.P = np.eye(len(self.state)) * 0.01
        self.initialized = True

    def predict_with_relative_motion(self, delta_d: float, delta_theta: float):
       
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

        # Process noise covariance
        Q_scaled = self.Q

        # Covariance prediction
        self.P = F @ self.P @ F.T + G @ Q_scaled @ G.T

        # Condition covariance for numerical stability
        self.P = self._condition_covariance(self.P)

    def _condition_covariance(self, P: np.ndarray,
                             min_eigenvalue: float = 1e-6,
                             max_eigenvalue: float = 100.0) -> np.ndarray:
        
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

        return {
            'x': float(self.state[0]),
            'y': float(self.state[1]),
            'theta': float(self.state[2])
        }

    def get_robot_covariance(self) -> np.ndarray:
        """Get the robot pose covariance (3x3).

        Returns:
            3x3 covariance matrix for [x, y, theta]
        """
        return self.P[0:3, 0:3].copy()

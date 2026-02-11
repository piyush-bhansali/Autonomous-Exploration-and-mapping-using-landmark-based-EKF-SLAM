#!/usr/bin/env python3

import numpy as np
from typing import Dict, Tuple, Optional, List


class LandmarkEKFSLAM:
  
    def __init__(self,
                 max_landmark_range: float = 5.0,
                 landmark_timeout_scans: int = 50,
                 min_observations_for_init: int = 2):
        
        # Robot state: [x, y, θ]
        self.state = np.zeros(3)

        # Covariance matrix 
        self.P = np.eye(3) * 0.01

        # Process noise (odometry)
        self.Q = np.diag([
            0.01,   
            0.005   
        ])
       
        self.landmarks = {}
        self.next_landmark_id = 0

        # Parameters
        self.max_landmark_range = max_landmark_range
        self.landmark_timeout_scans = landmark_timeout_scans
        self.min_observations_for_init = min_observations_for_init

        # Statistics
        self.initialized = False
        self.current_scan_number = 0
        self.landmark_update_count = 0

    def _condition_covariance(self, P: np.ndarray, min_eigenvalue: float = 1e-6,
                             max_eigenvalue: float = 100.0) -> np.ndarray:
      
        P_sym = (P + P.T) / 2.0

        eigvals, eigvecs = np.linalg.eigh(P_sym)

        eigvals_clamped = np.clip(eigvals, min_eigenvalue, max_eigenvalue)

        P_conditioned = eigvecs @ np.diag(eigvals_clamped) @ eigvecs.T

        n = len(P)
        P_conditioned += np.eye(n) * 1e-9

        return P_conditioned

    def initialize(self, x: float, y: float, theta: float):
        """Initialize robot pose."""
        self.state = np.array([x, y, theta])
        self.P = np.eye(3) * 0.01
        self.initialized = True

    def predict_with_relative_motion(self, delta_d: float, delta_theta: float):
       
        if not self.initialized:
            return

        self.current_scan_number += 1

        x, y, theta = self.state[0:3]

        # Predict robot pose
        theta_mid = theta + delta_theta / 2.0
        x_new = x + delta_d * np.cos(theta_mid)
        y_new = y + delta_d * np.sin(theta_mid)
        theta_new = theta + delta_theta
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))

        self.state[0:3] = [x_new, y_new, theta_new]

        n = len(self.state)
        F = np.eye(n)
        F[0:3, 0:3] = np.array([
            [1.0, 0.0, -delta_d * np.sin(theta_mid)],
            [0.0, 1.0,  delta_d * np.cos(theta_mid)],
            [0.0, 0.0,  1.0]
        ])

        G = np.zeros((n, 2))
        G[0:3, :] = np.array([
            [np.cos(theta_mid), -0.5 * delta_d * np.sin(theta_mid)],
            [np.sin(theta_mid),  0.5 * delta_d * np.cos(theta_mid)],
            [0.0,                1.0]
        ])

        motion_distance = abs(delta_d)
        motion_rotation = abs(delta_theta)

        # Add minimum variance to prevent zero when stationary
        min_distance_var = 0.0001  # (1cm)²
        min_rotation_var = 0.000001  # (~0.06°)²

        sigma_d_sq = self.Q[0, 0] * motion_distance**2 + min_distance_var
        sigma_theta_sq = self.Q[1, 1] * motion_rotation**2 + min_rotation_var

        Q_scaled = np.diag([sigma_d_sq, sigma_theta_sq])

        self.P = F @ self.P @ F.T + G @ Q_scaled @ G.T

        self.P = self._condition_covariance(self.P)

    def add_landmark(self,
                    z_x: float,
                    z_y: float,
                    feature: Dict,
                    scan_number: int) -> int:
        
        x_r, y_r, theta_r = self.state[0:3]

        R_obs = feature.get('covariance', np.eye(2) * 0.05**2)

        if feature['type'] == 'wall':
            
            rho_r = z_x
            alpha_r = z_y

            alpha_m = alpha_r + theta_r
            alpha_m = np.arctan2(np.sin(alpha_m), np.cos(alpha_m))

            cos_a = np.cos(alpha_m)
            sin_a = np.sin(alpha_m)

            rho_m = rho_r + x_r * cos_a + y_r * sin_a

            self.state = np.append(self.state, [rho_m, alpha_m])

            # Jacobian wrt robot pose
            d_rho_d_theta = -x_r * sin_a + y_r * cos_a
            H_r = np.array([
                [cos_a, sin_a, d_rho_d_theta],
                [0.0, 0.0, 1.0]
            ])

            # Jacobian wrt observation (rho_r, alpha_r)
            H_z = np.array([
                [1.0, d_rho_d_theta],
                [0.0, 1.0]
            ])

        else:
            # Corner in Cartesian (x, y) in robot frame
            cos_theta = np.cos(theta_r)
            sin_theta = np.sin(theta_r)

            # Landmark position in map frame
            lm_x = x_r + cos_theta * z_x - sin_theta * z_y
            lm_y = y_r + sin_theta * z_x + cos_theta * z_y

            # Augment state vector
            self.state = np.append(self.state, [lm_x, lm_y])

            # Jacobian wrt robot pose
            H_r = np.array([
                [1.0, 0.0, -sin_theta * z_x - cos_theta * z_y],
                [0.0, 1.0,  cos_theta * z_x - sin_theta * z_y]
            ])

            # Jacobian wrt observation (rho_r, alpha_r)
            H_z = np.array([
                [cos_theta, -sin_theta],
                [sin_theta,  cos_theta]
            ])

        # Landmark covariance (uncertainty propagation)
        P_rr = self.P[0:3, 0:3]  # Robot pose covariance
        P_lm = H_r @ P_rr @ H_r.T + H_z @ R_obs @ H_z.T

        # P_xl = P_xx_all * H_r^T  -> (N x 3) * (3 x 2) = (N x 2)
        P_rl = self.P[:, 0:3] @ H_r.T

        # Augment covariance matrix
        n_old = len(self.P)
        P_new = np.zeros((n_old + 2, n_old + 2))
        P_new[0:n_old, 0:n_old] = self.P
        P_new[0:n_old, n_old:n_old+2] = P_rl
        P_new[n_old:n_old+2, 0:n_old] = P_rl.T
        P_new[n_old:n_old+2, n_old:n_old+2] = P_lm

        self.P = P_new

        # Register landmark
        landmark_id = self.next_landmark_id
        self.next_landmark_id += 1

        state_index = len(self.state) - 2  # Index of landmark x in state

        self.landmarks[landmark_id] = {
            'state_index': state_index,
            'last_seen': scan_number,
            'observations': 1,
            'feature_type': feature['type']
        }

        return landmark_id

    def update_landmark_observation(self,
                                    landmark_id: int,
                                    z_x: float,
                                    z_y: float,
                                    scan_number: int,
                                    measurement_covariance: np.ndarray):
        
        if landmark_id not in self.landmarks:
            return

        lm_data = self.landmarks[landmark_id]
        idx = lm_data['state_index']

        # Update observation count and timestamp
        lm_data['observations'] += 1
        lm_data['last_seen'] = scan_number

        # Robot pose
        x_r, y_r, theta_r = self.state[0:3]

        # Observation Jacobian
        n = len(self.state)
        H = np.zeros((2, n))

        if lm_data['feature_type'] == 'wall':
            # Wall in Hessian form
            lm_rho = self.state[idx]
            lm_alpha = self.state[idx + 1]

            rho_pred = lm_rho - (x_r * np.cos(lm_alpha) + y_r * np.sin(lm_alpha))
            alpha_pred = lm_alpha - theta_r
            alpha_pred = np.arctan2(np.sin(alpha_pred), np.cos(alpha_pred))

            z_pred = np.array([rho_pred, alpha_pred])
            z_obs = np.array([z_x, z_y])

            innovation = z_obs - z_pred
            innovation[1] = np.arctan2(np.sin(innovation[1]), np.cos(innovation[1]))

            # ∂h/∂robot_pose
            H[0, 0] = -np.cos(lm_alpha)
            H[0, 1] = -np.sin(lm_alpha)
            H[0, 2] = 0.0

            H[1, 0] = 0.0
            H[1, 1] = 0.0
            H[1, 2] = -1.0

            # ∂h/∂landmark
            H[0, idx] = 1.0
            H[0, idx+1] = x_r * np.sin(lm_alpha) - y_r * np.cos(lm_alpha)

            H[1, idx] = 0.0
            H[1, idx+1] = 1.0
        else:
            # Landmark position
            lm_x = self.state[idx]
            lm_y = self.state[idx + 1]

            # Predicted observation (landmark in robot frame)
            # Rotation by -θ_r to transform from map to robot frame
            dx = lm_x - x_r
            dy = lm_y - y_r

            cos_theta = np.cos(-theta_r)
            sin_theta = np.sin(-theta_r)

            z_pred_x = cos_theta * dx - sin_theta * dy
            z_pred_y = sin_theta * dx + cos_theta * dy

            # Innovation
            innovation = np.array([z_x - z_pred_x, z_y - z_pred_y])

            # Observation model: h(x) = R(-θ_r) * (lm - robot_pos)
            # where R(-θ_r) = [[cos(-θ_r), -sin(-θ_r)], [sin(-θ_r), cos(-θ_r)]]
            #               = [[cos(θ_r), sin(θ_r)], [-sin(θ_r), cos(θ_r)]]
            #
            # Jacobian ∂h/∂robot_pose
            c = np.cos(theta_r)
            s = np.sin(theta_r)

            H[0, 0] = -c
            H[0, 1] = -s
            H[0, 2] = -s * dx + c * dy

            H[1, 0] = s
            H[1, 1] = -c
            H[1, 2] = -c * dx - s * dy

            # Jacobian ∂h/∂landmark (using R(-θ_r) from line 268-269)
            H[0, idx] = cos_theta
            H[0, idx+1] = -sin_theta

            H[1, idx] = sin_theta
            H[1, idx+1] = cos_theta

        # Innovation covariance
        S = H @ self.P @ H.T + measurement_covariance

        # Outlier rejection using Mahalanobis distance test
        try:
            S_inv = np.linalg.inv(S)
            mahal_dist_sq = float(innovation.T @ S_inv @ innovation)
        except np.linalg.LinAlgError:
            # Singular covariance - reject update
            return

        # Chi-squared test (2-DOF, 99.7% confidence interval = 13.8)
        # This rejects ~0.3% of valid measurements but catches most outliers
        chi_sq_threshold = 13.8
        if mahal_dist_sq > chi_sq_threshold:
            # Reject outlier - innovation too large given uncertainty
            return

        # Kalman gain
        try:
            K = self.P @ H.T @ S_inv
        except np.linalg.LinAlgError:
            return

        # State update
        self.state = self.state + K @ innovation

        # Normalize robot orientation
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

        # Covariance update (Joseph form)
        I = np.eye(n)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ measurement_covariance @ K.T

        # Condition covariance
        self.P = self._condition_covariance(self.P)

        self.landmark_update_count += 1

    def update(self,
               x: float,
               y: float,
               theta: float,
               measurement_covariance: np.ndarray,
               measurement_type: str = "icp"):
        """EKF pose measurement update (e.g., ICP)."""
        if not self.initialized:
            return

        n = len(self.state)
        z = np.array([x, y, theta])
        z_pred = self.state[0:3]

        # Innovation with angle normalization
        innovation = z - z_pred
        innovation[2] = np.arctan2(np.sin(innovation[2]), np.cos(innovation[2]))

        H = np.zeros((3, n))
        H[0:3, 0:3] = np.eye(3)

        S = H @ self.P @ H.T + measurement_covariance

        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        self.state = self.state + K @ innovation
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

        I = np.eye(n)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ measurement_covariance @ K.T

        # Condition covariance
        self.P = self._condition_covariance(self.P)

    def prune_landmarks(self, current_scan_number: int) -> List[int]:
        """
        Prune landmarks that haven't been seen recently.

        Returns:
            List of pruned landmark IDs (for FeatureMap synchronization)
        """
        to_remove = []

        for lm_id, lm_data in self.landmarks.items():
            scans_since_seen = current_scan_number - lm_data['last_seen']


            if (scans_since_seen > self.landmark_timeout_scans or
                (lm_data['observations'] < self.min_observations_for_init and
                 scans_since_seen > 10)):
                to_remove.append(lm_id)

        # Remove landmarks (highest index first to avoid invalidating indices)
        to_remove.sort(key=lambda lm_id: self.landmarks[lm_id]['state_index'], reverse=True)

        for lm_id in to_remove:
            self._remove_landmark(lm_id)

        return to_remove

    def _remove_landmark(self, landmark_id: int):
       
        if landmark_id not in self.landmarks:
            return

        idx = self.landmarks[landmark_id]['state_index']

        # Remove from state vector
        self.state = np.delete(self.state, [idx, idx+1])

        # Remove from covariance matrix
        rows_to_keep = [i for i in range(len(self.P)) if i not in [idx, idx+1]]
        self.P = self.P[np.ix_(rows_to_keep, rows_to_keep)]

        # Update indices of landmarks that came after this one
        for lm_id, lm_data in self.landmarks.items():
            if lm_data['state_index'] > idx:
                lm_data['state_index'] -= 2

        # Remove from database
        del self.landmarks[landmark_id]

    def get_state(self) -> Dict:
        """Get current robot pose."""
        return {
            'x': self.state[0],
            'y': self.state[1],
            'theta': self.state[2]
        }

    def get_landmark_positions(self) -> Dict[int, Tuple[float, float]]:
        
        positions = {}
        for lm_id, lm_data in self.landmarks.items():
            idx = lm_data['state_index']
            positions[lm_id] = (self.state[idx], self.state[idx+1])
        return positions

    def get_landmark_covariance(self, landmark_id: int) -> Optional[np.ndarray]:
       
        if landmark_id not in self.landmarks:
            return None

        idx = self.landmarks[landmark_id]['state_index']
        return self.P[idx:idx+2, idx:idx+2]

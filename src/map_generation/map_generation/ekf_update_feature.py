#!/usr/bin/env python3

import numpy as np
from typing import Dict, List, Tuple, Optional
from map_generation.ekf_predict import BaseEKF
from map_generation.transform_utils import robot_wall_to_map_frame, normalize_angle


class LandmarkEKFSLAM(BaseEKF):


    def __init__(self,
                 landmark_timeout_scans: int = 50,
                 min_observations_for_init: int = 2):

        super().__init__()

        self.landmarks = {}
        self.next_landmark_id = 0

        self.landmark_timeout_scans = landmark_timeout_scans
        self.min_observations_for_init = min_observations_for_init

        self.landmark_update_count = 0

    def add_landmark(self,
                    z_x: float,
                    z_y: float,
                    feature: Dict,
                    scan_number: int) -> int:

        x_r, y_r, theta_r = self.state[0:3]

        R_obs = feature.get('covariance', np.eye(2) * 0.05**2)

        if feature['type'] == 'wall':
            rho_m, alpha_m = robot_wall_to_map_frame(z_x, z_y, x_r, y_r, theta_r)

            self.state = np.append(self.state, [rho_m, alpha_m])

            cos_a = np.cos(alpha_m)
            sin_a = np.sin(alpha_m)

            d_rho_d_theta = -x_r * sin_a + y_r * cos_a
            H_r = np.array([
                [cos_a, sin_a, d_rho_d_theta],
                [0.0, 0.0, 1.0]
            ])

            H_z = np.array([
                [1.0, d_rho_d_theta],
                [0.0, 1.0]
            ])

        else:
            
            cos_theta = np.cos(theta_r)
            sin_theta = np.sin(theta_r)

            lm_x = x_r + cos_theta * z_x - sin_theta * z_y
            lm_y = y_r + sin_theta * z_x + cos_theta * z_y

            self.state = np.append(self.state, [lm_x, lm_y])

            H_r = np.array([
                [1.0, 0.0, -sin_theta * z_x - cos_theta * z_y],
                [0.0, 1.0,  cos_theta * z_x - sin_theta * z_y]
            ])

            H_z = np.array([
                [cos_theta, -sin_theta],
                [sin_theta,  cos_theta]
            ])

        P_rr = self.P[0:3, 0:3]  
        P_lm = H_r @ P_rr @ H_r.T + H_z @ R_obs @ H_z.T

        P_rl = self.P[:, 0:3] @ H_r.T

        n_old = len(self.P)
        P_new = np.zeros((n_old + 2, n_old + 2))
        P_new[0:n_old, 0:n_old] = self.P
        P_new[0:n_old, n_old:n_old+2] = P_rl
        P_new[n_old:n_old+2, 0:n_old] = P_rl.T
        P_new[n_old:n_old+2, n_old:n_old+2] = P_lm

        self.P = P_new

        landmark_id = self.next_landmark_id
        self.next_landmark_id += 1

        state_index = len(self.state) - 2 

        self.landmarks[landmark_id] = {
            'state_index': state_index,
            'last_seen': scan_number,
            'observations': 1,
            'feature_type': feature['type']
        }

        return landmark_id

    def _build_observation(self, landmark_id: int) -> Optional[Tuple[np.ndarray, np.ndarray]]:
       
        if landmark_id not in self.landmarks:
            return None

        lm_data = self.landmarks[landmark_id]
        idx = lm_data['state_index']
        x_r, y_r, theta_r = self.state[0:3]
        n = len(self.state)
        H = np.zeros((2, n))

        if lm_data['feature_type'] == 'wall':
            lm_rho = self.state[idx]
            lm_alpha = self.state[idx + 1]

            rho_pred = lm_rho - (x_r * np.cos(lm_alpha) + y_r * np.sin(lm_alpha))
            alpha_pred = normalize_angle(lm_alpha - theta_r)
            z_pred = np.array([rho_pred, alpha_pred])

            H[0, 0] = -np.cos(lm_alpha)
            H[0, 1] = -np.sin(lm_alpha)
            H[0, 2] = 0.0
            H[1, 0] = 0.0
            H[1, 1] = 0.0
            H[1, 2] = -1.0

            H[0, idx]     = 1.0
            H[0, idx + 1] = x_r * np.sin(lm_alpha) - y_r * np.cos(lm_alpha)
            H[1, idx]     = 0.0
            H[1, idx + 1] = 1.0

        else:
            lm_x = self.state[idx]
            lm_y = self.state[idx + 1]

            dx = lm_x - x_r
            dy = lm_y - y_r

            cos_neg = np.cos(-theta_r)
            sin_neg = np.sin(-theta_r)

            z_pred = np.array([
                cos_neg * dx - sin_neg * dy,
                sin_neg * dx + cos_neg * dy
            ])

            c = np.cos(theta_r)
            s = np.sin(theta_r)
            H[0, 0] = -c
            H[0, 1] = -s
            H[0, 2] = -s * dx + c * dy

            H[1, 0] = s
            H[1, 1] = -c
            H[1, 2] = -c * dx - s * dy

            H[0, idx]     = cos_neg
            H[0, idx + 1] = -sin_neg
            H[1, idx]     = sin_neg
            H[1, idx + 1] = cos_neg

        return H, z_pred

    def update_landmark_observation(self,
                                    landmark_id: int,
                                    z_x: float,
                                    z_y: float,
                                    scan_number: int,
                                    measurement_covariance: np.ndarray):

        if landmark_id not in self.landmarks:
            return

        lm_data = self.landmarks[landmark_id]
        lm_data['observations'] += 1
        lm_data['last_seen'] = scan_number

        result = self._build_observation(landmark_id)
        if result is None:
            return
        H, z_pred = result

        z_obs = np.array([z_x, z_y])
        innovation = z_obs - z_pred

        if lm_data['feature_type'] == 'wall':
            innovation[1] = normalize_angle(innovation[1])

        S = H @ self.P @ H.T + measurement_covariance

        try:
            S_inv = np.linalg.inv(S)
            mahal_dist_sq = float(innovation.T @ S_inv @ innovation)
        except np.linalg.LinAlgError:
            return

        chi_sq_threshold = 5.99
        if mahal_dist_sq > chi_sq_threshold:
            return

        try:
            K = self.P @ H.T @ S_inv
        except np.linalg.LinAlgError:
            return

        n = len(self.state)

        self.state = self.state + K @ innovation

        self.state[2] = normalize_angle(self.state[2])

     
        for lm_data in self.landmarks.values():
            if lm_data['feature_type'] == 'wall':
                i = lm_data['state_index']
                if self.state[i] < 0.0:
                    self.state[i] = -self.state[i]
                    self.state[i + 1] = normalize_angle(self.state[i + 1] + np.pi)

        I = np.eye(n)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ measurement_covariance @ K.T

        self.P = self._condition_covariance(self.P)

        self.landmark_update_count += 1

    def prune_landmarks(self, current_scan_number: int) -> List[int]:

        to_remove = []

        for lm_id, lm_data in self.landmarks.items():
            scans_since_seen = current_scan_number - lm_data['last_seen']

            if (scans_since_seen > self.landmark_timeout_scans or
                (lm_data['observations'] < self.min_observations_for_init and
                 scans_since_seen > 10)):
                to_remove.append(lm_id)

        to_remove.sort(key=lambda lm_id: self.landmarks[lm_id]['state_index'], reverse=True)

        for lm_id in to_remove:
            self._remove_landmark(lm_id)

        return to_remove

    def _remove_landmark(self, landmark_id: int):
      
        if landmark_id not in self.landmarks:
            return

        idx = self.landmarks[landmark_id]['state_index']

        self.state = np.delete(self.state, [idx, idx+1])

        rows_to_keep = [i for i in range(len(self.P)) if i not in [idx, idx+1]]
        self.P = self.P[np.ix_(rows_to_keep, rows_to_keep)]

        for lm_id, lm_data in self.landmarks.items():
            if lm_data['state_index'] > idx:
                lm_data['state_index'] -= 2

        del self.landmarks[landmark_id]

    def update(self,
               x: float,
               y: float,
               theta: float,
               measurement_covariance: np.ndarray,
               measurement_type: str = "pose_measurement"):

        if not self.initialized:
            return

        n = len(self.state)
        z = np.array([x, y, theta])
        z_pred = self.state[0:3]

        innovation = z - z_pred
        innovation[2] = normalize_angle(innovation[2])

        H = np.zeros((3, n))
        H[0:3, 0:3] = np.eye(3)

        S = H @ self.P @ H.T + measurement_covariance

        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return
        
        self.state = self.state + K @ innovation
        self.state[2] = normalize_angle(self.state[2])

        I = np.eye(n)
        I_KH = I - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ measurement_covariance @ K.T

        self.P = self._condition_covariance(self.P)

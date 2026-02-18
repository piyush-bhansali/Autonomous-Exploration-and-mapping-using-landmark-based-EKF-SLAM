#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Tuple
from map_generation.transform_utils import robot_wall_to_map_frame, normalize_angle


def associate_landmarks(
    observed_features: List[Dict],
    ekf_slam,
    chi_sq_gate: float = 5.99,
    max_euclidean_dist: float = 6.0,
    wall_angle_tolerance: float = 0.349066,  
    wall_rho_tolerance: float = 0.5,  
    return_extension_info: bool = False
) -> Tuple[List[Tuple[int, int]], List[int], Dict]:

    matched = []
    unmatched = []
    extension_info = {}

    if len(ekf_slam.landmarks) == 0:
      
        if return_extension_info:
            return matched, list(range(len(observed_features))), extension_info
        else:
            return matched, list(range(len(observed_features)))

    x_r, y_r, theta_r = ekf_slam.state[0:3]

    for feat_idx, feature in enumerate(observed_features):
        best_landmark_id = None
        best_mahal_sq = float('inf')

        for landmark_id, lm_data in ekf_slam.landmarks.items():

            if feature['type'] != lm_data['feature_type']:
                continue

            idx = lm_data['state_index']

            if feature['type'] == 'wall':

                lm_rho = ekf_slam.state[idx]
                lm_alpha = ekf_slam.state[idx + 1]

                rho_pred = lm_rho - (x_r * np.cos(lm_alpha) + y_r * np.sin(lm_alpha))
                alpha_pred = normalize_angle(lm_alpha - theta_r)

                if abs(rho_pred) > max_euclidean_dist:
                    continue

                alpha_diff = abs(normalize_angle(alpha_pred - feature['alpha']))
                if alpha_diff > wall_angle_tolerance:
                    continue

                obs_rho_map, _ = robot_wall_to_map_frame(
                    feature['rho'], feature['alpha'], x_r, y_r, theta_r
                )
                if abs(lm_rho - obs_rho_map) > wall_rho_tolerance:
                    continue

            elif feature['type'] == 'corner':
               
                lm_x = ekf_slam.state[idx]
                lm_y = ekf_slam.state[idx + 1]
                dist_to_corner = np.sqrt((lm_x - x_r)**2 + (lm_y - y_r)**2)
                if dist_to_corner > max_euclidean_dist:
                    continue

            else:
                continue

            result = ekf_slam._build_observation(landmark_id)
            if result is None:
                continue
            H, z_pred = result

            if feature['type'] == 'wall':
                z_obs = np.array([feature['rho'], feature['alpha']])
                innovation = z_obs - z_pred
                innovation[1] = normalize_angle(innovation[1])
            else:
                z_obs = feature['position']
                innovation = z_obs - z_pred

            S = H @ ekf_slam.P @ H.T + feature['covariance']

            try:
                S_inv = np.linalg.inv(S)
                mahal_dist_sq = float(innovation.T @ S_inv @ innovation)
            except np.linalg.LinAlgError:
                continue

            if mahal_dist_sq < chi_sq_gate and mahal_dist_sq < best_mahal_sq:
                best_mahal_sq = mahal_dist_sq
                best_landmark_id = landmark_id

        if best_landmark_id is not None:
            matched.append((feat_idx, best_landmark_id))

            if return_extension_info and feature['type'] == 'wall':
                obs_start = feature.get('start_point', None)
                obs_end = feature.get('end_point', None)

                if obs_start is not None and obs_end is not None:
                    c = np.cos(theta_r)
                    s = np.sin(theta_r)
                    R = np.array([[c, -s], [s, c]])

                    start_map = R @ obs_start + np.array([x_r, y_r])
                    end_map = R @ obs_end + np.array([x_r, y_r])

                    extension_info[feat_idx] = {
                        'new_start': start_map,
                        'new_end': end_map
                    }
        else:
            unmatched.append(feat_idx)

    if return_extension_info:
        return matched, unmatched, extension_info
    else:
        return matched, unmatched

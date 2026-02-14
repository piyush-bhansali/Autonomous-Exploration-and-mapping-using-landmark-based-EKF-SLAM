#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Tuple


def associate_landmarks(
    observed_features: List[Dict],
    ekf_slam,
    feature_map=None,
    max_mahalanobis_dist: float = 5.99,
    max_euclidean_dist: float = 6.0,
    wall_angle_tolerance: float = 0.349066,  # 20 degrees in radians
    wall_rho_tolerance: float = 0.5,  # 0.5m tolerance for parallel wall detection
    return_extension_info: bool = False
) -> Tuple[List[Tuple[int, int]], List[int], Dict]:

    matched = []
    unmatched = []
    extension_info = {}  

    if len(ekf_slam.landmarks) == 0:
        # No landmarks yet - all observations are new
        if return_extension_info:
            return matched, list(range(len(observed_features))), extension_info
        else:
            return matched, list(range(len(observed_features)))

    # Robot pose
    x_r, y_r, theta_r = ekf_slam.state[0:3]

    for feat_idx, feature in enumerate(observed_features):
        best_landmark_id = None
        best_mahalanobis = float('inf')

        # Try matching with each existing landmark
        for landmark_id, lm_data in ekf_slam.landmarks.items():
            # Type consistency check
            if feature['type'] != lm_data['feature_type']:
                continue

            # Get landmark state
            idx = lm_data['state_index']

            if feature['type'] == 'wall':
                # Wall features (Hessian form)
                lm_rho = ekf_slam.state[idx]
                lm_alpha = ekf_slam.state[idx + 1]

                # Compute predicted observation in robot frame FIRST
                # (used for both pre-checks and innovation)
                rho_pred = lm_rho - (x_r * np.cos(lm_alpha) + y_r * np.sin(lm_alpha))
                alpha_pred = lm_alpha - theta_r
                alpha_pred = np.arctan2(np.sin(alpha_pred), np.cos(alpha_pred))

                # Spatial gating: wall must be within sensor range (robot frame rho)
                if abs(rho_pred) > max_euclidean_dist:
                    continue

                # Check 1: Alpha similarity - BOTH in robot frame
                # feature['alpha'] is robot frame, alpha_pred is landmark projected to robot frame
                # No 180° ambiguity: Hessian form enforces rho > 0 which uniquely determines
                # the normal direction. Applying 180° symmetry would cause opposite walls
                # (e.g. north/south corridor walls at equal distance) to falsely match.
                alpha_diff = abs(np.arctan2(np.sin(alpha_pred - feature['alpha']),
                                            np.cos(alpha_pred - feature['alpha'])))
                if alpha_diff > wall_angle_tolerance:
                    continue  # Different wall orientation

                # Check 2: Rho compatibility - transform observed rho to map frame
                obs_alpha_map = feature['alpha'] + theta_r
                obs_alpha_map = np.arctan2(np.sin(obs_alpha_map), np.cos(obs_alpha_map))
                obs_rho_map = feature['rho'] + (x_r * np.cos(obs_alpha_map) + y_r * np.sin(obs_alpha_map))

                # Handle negative rho
                if obs_rho_map < 0:
                    obs_rho_map = -obs_rho_map

                rho_diff = abs(lm_rho - obs_rho_map)
                if rho_diff > wall_rho_tolerance:
                    continue  # Parallel wall at different distance

                z_pred = np.array([rho_pred, alpha_pred])
                z_obs = np.array([feature['rho'], feature['alpha']])

                # Innovation
                innovation = z_obs - z_pred
                innovation[1] = np.arctan2(np.sin(innovation[1]), np.cos(innovation[1]))

                # Observation Jacobian for walls (Hessian form)
                n = len(ekf_slam.state)
                H = np.zeros((2, n))

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

            elif feature['type'] == 'corner':
                # Corner features (Cartesian)
                lm_x = ekf_slam.state[idx]
                lm_y = ekf_slam.state[idx + 1]

                # Spatial gating: Euclidean distance
                dist_to_corner = np.sqrt((lm_x - x_r)**2 + (lm_y - y_r)**2)

                if dist_to_corner > max_euclidean_dist:
                    continue

                # Predicted observation (corner in robot frame)
                dx = lm_x - x_r
                dy = lm_y - y_r

                cos_theta = np.cos(-theta_r)
                sin_theta = np.sin(-theta_r)

                z_pred_x = cos_theta * dx - sin_theta * dy
                z_pred_y = sin_theta * dx + cos_theta * dy

                z_pred = np.array([z_pred_x, z_pred_y])
                z_obs = feature['position']

                # Innovation
                innovation = z_obs - z_pred

                # Observation Jacobian for corners (Cartesian)
                n = len(ekf_slam.state)
                H = np.zeros((2, n))

                # ∂h/∂robot_pose
                H[0, 0] = -cos_theta
                H[0, 1] =  sin_theta
                H[0, 2] = -dx * sin_theta - dy * cos_theta

                H[1, 0] = -sin_theta
                H[1, 1] = -cos_theta
                H[1, 2] =  dx * cos_theta - dy * sin_theta

                # ∂h/∂landmark
                H[0, idx] = cos_theta
                H[0, idx+1] = -sin_theta

                H[1, idx] = sin_theta
                H[1, idx+1] = cos_theta

            else:
                continue

            S = H @ ekf_slam.P @ H.T + feature['covariance']

            try:
                S_inv = np.linalg.inv(S)
                mahal_dist_sq = innovation.T @ S_inv @ innovation
                mahal_dist = float(np.sqrt(mahal_dist_sq))
            except np.linalg.LinAlgError:
                continue

            if mahal_dist_sq < max_mahalanobis_dist**2 and mahal_dist < best_mahalanobis:
                # Wall matching now relies on Hessian parameters only (no endpoint checks)
                best_mahalanobis = mahal_dist
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
                        'landmark_id': best_landmark_id,
                        'new_start': start_map,
                        'new_end': end_map
                    }
        else:
            unmatched.append(feat_idx)

    if return_extension_info:
        return matched, unmatched, extension_info
    else:
        return matched, unmatched

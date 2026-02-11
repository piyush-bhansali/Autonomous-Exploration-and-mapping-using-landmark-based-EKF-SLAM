#!/usr/bin/env python3

import numpy as np
from typing import List, Dict, Tuple


def associate_landmarks(
    observed_features: List[Dict],
    ekf_slam,
    max_mahalanobis_dist: float = 5.99,  # Chi-squared 2-DOF, 95% confidence
    max_euclidean_dist: float = 1.0,
    return_extension_info: bool = False
) -> Tuple[List[Tuple[int, int]], List[int], Dict]:

    matched = []
    unmatched = []
    extension_info = {}  # Will contain wall extension data if return_extension_info=True

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

                # Spatial gating: check if wall is within range
                dist_to_wall = abs(lm_rho - (x_r * np.cos(lm_alpha) + y_r * np.sin(lm_alpha)))

                if dist_to_wall > max_euclidean_dist:
                    continue

                # Predicted observation (wall in robot frame)
                rho_pred = lm_rho - (x_r * np.cos(lm_alpha) + y_r * np.sin(lm_alpha))
                alpha_pred = lm_alpha - theta_r
                alpha_pred = np.arctan2(np.sin(alpha_pred), np.cos(alpha_pred))  # Normalize

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

            # Innovation covariance
            S = H @ ekf_slam.P @ H.T + feature['covariance']

            # Mahalanobis distance
            try:
                S_inv = np.linalg.inv(S)
                mahal_dist_sq = innovation.T @ S_inv @ innovation
                mahal_dist = float(np.sqrt(mahal_dist_sq))
            except np.linalg.LinAlgError:
                continue

            # Chi-squared gating
            if mahal_dist_sq < max_mahalanobis_dist**2 and mahal_dist < best_mahalanobis:
                best_mahalanobis = mahal_dist
                best_landmark_id = landmark_id

        if best_landmark_id is not None:
            matched.append((feat_idx, best_landmark_id))

            # Check for wall extension if requested
            if return_extension_info and feature['type'] == 'wall':
                # Transform observed endpoints from robot frame to map frame
                obs_start = feature.get('start_point', None)
                obs_end = feature.get('end_point', None)

                if obs_start is not None and obs_end is not None:
                    # Rotation matrix robot -> map
                    c = np.cos(theta_r)
                    s = np.sin(theta_r)
                    R = np.array([[c, -s], [s, c]])

                    # Transform to map frame
                    start_map = R @ obs_start + np.array([x_r, y_r])
                    end_map = R @ obs_end + np.array([x_r, y_r])

                    # Transform points to map frame
                    obs_points = feature.get('points', np.array([]))
                    if len(obs_points) > 0:
                        points_map = (R @ obs_points.T).T + np.array([x_r, y_r])
                    else:
                        points_map = np.array([])

                    extension_info[feat_idx] = {
                        'landmark_id': best_landmark_id,
                        'new_start': start_map,
                        'new_end': end_map,
                        'new_points': points_map
                    }
        else:
            unmatched.append(feat_idx)

    if return_extension_info:
        return matched, unmatched, extension_info
    else:
        return matched, unmatched

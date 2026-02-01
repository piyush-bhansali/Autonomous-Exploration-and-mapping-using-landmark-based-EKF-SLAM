#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import LaserScan
from typing import List, Dict, Tuple, Optional
from map_generation.transform_utils import estimate_rigid_transform_2d


class LidarLineCornerExtractor:
    

    def __init__(self,
                 min_points_per_line: int = 8,
                 line_fit_threshold: float = 0.03,  # 3cm
                 min_line_length: float = 0.3,      # 30cm
                 corner_angle_threshold: float = 25.0):  # degrees
        
        self.min_points = min_points_per_line
        self.fit_threshold = line_fit_threshold
        self.min_length = min_line_length
        self.corner_angle = np.radians(corner_angle_threshold)

    def extract_features(self, scan_msg: LaserScan) -> List[Dict]:
        
        # 1. Convert scan to Cartesian points
        points = self._scan_to_cartesian(scan_msg)

        if len(points) < self.min_points:
            return []

        # 2. Extract line segments using incremental fitting
        lines = self._extract_lines(points)

        # 3. Detect corners at line intersections and range discontinuities
        corners = self._detect_corners(points, lines)

        # 4. Combine all features
        features = []

        # Add line features
        for line in lines:
            features.append({
                'type': 'line',
                'position': (line['start'] + line['end']) / 2.0,  # Midpoint
                'start': line['start'],
                'end': line['end'],
                'orientation': line['angle'],
                'strength': line['length'],  # Line length as strength
                'num_points': line['num_points']
            })

        # Add corner features
        for corner in corners:
            features.append({
                'type': 'corner',
                'position': corner['position'],
                'orientation': corner['angle'],
                'strength': corner['sharpness'],  # Angle change as strength
                'num_points': 1
            })

        return features

    def _scan_to_cartesian(self, scan_msg: LaserScan) -> np.ndarray:
        """Convert LaserScan to [N x 2] Cartesian points in robot frame."""
        ranges = np.array(scan_msg.ranges)
        num_points = len(ranges)
        angles = np.linspace(
            scan_msg.angle_min,
            scan_msg.angle_max,
            num_points
        )

        # Filter valid ranges
        valid_mask = (
            (ranges >= scan_msg.range_min) &
            (ranges <= scan_msg.range_max) &
            np.isfinite(ranges)
        )

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        # Convert to Cartesian
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)

        return np.column_stack([x, y])

    def _extract_lines(self, points: np.ndarray) -> List[Dict]:
        
        if len(points) < self.min_points:
            return []

        lines = []
        current_line_points = [points[0]]
        current_line_indices = [0]

        for i in range(1, len(points)):
            current_line_points.append(points[i])
            current_line_indices.append(i)

            if len(current_line_points) >= 2:
                # Fit line to current points
                line_params = self._fit_line_least_squares(np.array(current_line_points))

                # Check if new point fits the line
                dist = self._point_to_line_distance(points[i], line_params)

                if dist > self.fit_threshold:
                    # Point doesn't fit - finalize previous line
                    if len(current_line_points) > self.min_points:
                        line = self._create_line_feature(
                            np.array(current_line_points[:-1]),
                            current_line_indices[:-1]
                        )
                        if line is not None:
                            lines.append(line)

                    # Start new line with current point
                    current_line_points = [points[i]]
                    current_line_indices = [i]

        # Handle last line
        if len(current_line_points) > self.min_points:
            line = self._create_line_feature(
                np.array(current_line_points),
                current_line_indices
            )
            if line is not None:
                lines.append(line)

        return lines

    def _fit_line_least_squares(self, points: np.ndarray) -> Dict:
        """Fit line using least squares."""
        if len(points) < 2:
            return None

        # Compute centroid
        centroid = np.mean(points, axis=0)

        # Center points
        centered = points - centroid

        # SVD to find principal direction
        if len(points) == 2:
            # For 2 points, direction is just the vector between them
            direction = points[1] - points[0]
            direction = direction / (np.linalg.norm(direction) + 1e-10)
        else:
            U, S, Vt = np.linalg.svd(centered)
            direction = Vt[0]  # First right singular vector

        # Normal vector (perpendicular to line)
        normal = np.array([-direction[1], direction[0]])

        # Distance from origin (ax + by = c form)
        rho = np.dot(normal, centroid)

        return {
            'centroid': centroid,
            'direction': direction,
            'normal': normal,
            'rho': rho
        }

    def _point_to_line_distance(self, point: np.ndarray, line_params: Dict) -> float:
        """Compute perpendicular distance from point to line."""
        if line_params is None:
            return float('inf')

        # Distance = |ax + by - c| / sqrt(a² + b²)
        normal = line_params['normal']
        rho = line_params['rho']

        dist = abs(np.dot(normal, point) - rho)
        return dist

    def _create_line_feature(self, points: np.ndarray, indices: List[int]) -> Optional[Dict]:
        """Create line feature from points."""
        if len(points) < 2:
            return None

        # Start and end points
        start = points[0]
        end = points[-1]

        # Line length
        length = np.linalg.norm(end - start)

        # Skip short lines
        if length < self.min_length:
            return None

        # Line angle
        direction = end - start
        angle = np.arctan2(direction[1], direction[0])

        return {
            'start': start,
            'end': end,
            'length': length,
            'angle': angle,
            'num_points': len(points),
            'indices': indices
        }

    def _detect_corners(self, points: np.ndarray, lines: List[Dict]) -> List[Dict]:
        
        corners = []

        # Method 1: Line intersections
        for i in range(len(lines) - 1):
            line1 = lines[i]
            line2 = lines[i + 1]

            # Check if lines are consecutive in scan sequence
            if line1['indices'][-1] + 1 == line2['indices'][0]:
                # Check if line endpoints are actually close in space
                # (reject if there's a large gap like a doorway)
                endpoint_distance = np.linalg.norm(line1['end'] - line2['start'])

                if endpoint_distance < 0.5:  # Lines must be within 50cm
                    # Compute angle difference
                    angle_diff = abs(line2['angle'] - line1['angle'])
                    angle_diff = min(angle_diff, 2 * np.pi - angle_diff)  # Wrap to [0, π]

                    if angle_diff > self.corner_angle:
                        # Sharp corner detected
                        # Corner position is end of first line / start of second
                        corner_pos = (line1['end'] + line2['start']) / 2.0

                        corners.append({
                            'position': corner_pos,
                            'angle': (line1['angle'] + line2['angle']) / 2.0,
                            'sharpness': np.degrees(angle_diff)  # Larger = sharper corner
                        })

        # Method 2: Range discontinuities (occlusions only)
        # Only detect corners where robot is getting CLOSER (convex corners)
        # Reject lateral jumps (gaps/doorways)
        for i in range(1, len(points)):
            # Distance jump between consecutive scan points
            dist_jump = np.linalg.norm(points[i] - points[i-1])

            if dist_jump > 0.5:  # Large jump (> 50cm)
                # Check if this is an occlusion (convex corner) or just a gap
                # Occlusion: One point is much closer than the other
                range_before = np.linalg.norm(points[i-1])
                range_after = np.linalg.norm(points[i])

                range_diff = abs(range_after - range_before)

                # Only accept if one side is significantly closer (occlusion)
                # Reject if both sides are similar distance (gap/doorway)
                if range_diff > 0.3:  # At least 30cm range difference
                    # Occlusion corner - use the CLOSER point
                    if range_before < range_after:
                        corner_pos = points[i-1]  # Before jump is closer
                    else:
                        corner_pos = points[i]  # After jump is closer

                    # Angle from robot to corner
                    angle = np.arctan2(corner_pos[1], corner_pos[0])

                    corners.append({
                        'position': corner_pos,
                        'angle': angle,
                        'sharpness': 90.0  # Default sharpness for occlusions
                    })

        return corners


class FeatureMatcher:
    

    def __init__(self,
                 chi2_gate_threshold: float = 5.99,  # 95% confidence, 2 DOF
                 feature_measurement_noise: float = 0.05,  # 5cm
                 max_matches_per_feature: int = 1):
        
        self.gate_threshold = chi2_gate_threshold
        self.R = np.eye(2) * (feature_measurement_noise ** 2)
        self.max_matches = max_matches_per_feature

    def match_features(self,
                      features_current: List[Dict],
                      features_previous: List[Dict],
                      robot_state: np.ndarray,
                      robot_covariance: np.ndarray,
                      odom_delta: np.ndarray) -> List[Tuple[int, int, float]]:
        
        if len(features_current) == 0 or len(features_previous) == 0:
            return []

        matches = []
        dx_odom, dy_odom, dtheta_odom = odom_delta

        # For each current feature, find best match in previous features
        for i, feat_curr in enumerate(features_current):
            best_match_idx = None
            best_mahal_dist = float('inf')

            for j, feat_prev in enumerate(features_previous):
                # Type consistency check
                if feat_curr['type'] != feat_prev['type']:
                    continue

                # Predict where previous feature should appear
                predicted_pos = self._predict_feature_position(
                    feat_prev['position'],
                    dx_odom, dy_odom, dtheta_odom
                )

                # Observed position
                observed_pos = feat_curr['position']

                # Innovation (residual)
                innovation = observed_pos - predicted_pos

                # Innovation covariance S = H P H^T + R
                S = self._compute_innovation_covariance(
                    feat_prev['position'],
                    robot_covariance,
                    dtheta_odom
                )

                # Mahalanobis distance squared
                try:
                    S_inv = np.linalg.inv(S)
                    mahal_dist_sq = innovation.T @ S_inv @ innovation
                except np.linalg.LinAlgError:
                    continue  # Singular covariance

                # Chi-squared gating test
                if mahal_dist_sq <= self.gate_threshold:
                    mahal_dist = np.sqrt(mahal_dist_sq)

                    if mahal_dist < best_mahal_dist:
                        best_mahal_dist = mahal_dist
                        best_match_idx = j

            # Store best match if found
            if best_match_idx is not None:
                matches.append((i, best_match_idx, best_mahal_dist))

        return matches

    def _predict_feature_position(self,
                                  prev_pos: np.ndarray,
                                  dx: float,
                                  dy: float,
                                  dtheta: float) -> np.ndarray:
        
        x_prev, y_prev = prev_pos

        # Apply inverse rotation
        cos_th = np.cos(-dtheta)
        sin_th = np.sin(-dtheta)

        # Translate then rotate
        x_trans = x_prev - dx
        y_trans = y_prev - dy

        x_pred = cos_th * x_trans - sin_th * y_trans
        y_pred = sin_th * x_trans + cos_th * y_trans

        return np.array([x_pred, y_pred])

    def _compute_innovation_covariance(self,
                                      feature_pos: np.ndarray,
                                      P_robot: np.ndarray,
                                      dtheta: float) -> np.ndarray:
        
        x_f, y_f = feature_pos

        # Jacobian (2x3): How feature position changes with robot state
        # For inverse transform: p_new = R(-θ) * (p_old - [dx, dy])

        cos_th = np.cos(-dtheta)
        sin_th = np.sin(-dtheta)

        H = np.array([
            [-cos_th,  sin_th, -x_f * sin_th - y_f * cos_th],
            [-sin_th, -cos_th,  x_f * cos_th - y_f * sin_th]
        ])

        # Innovation covariance
        S = H @ P_robot @ H.T + self.R

        return S

    def compute_transform_from_matches(self,
                                      features_current: List[Dict],
                                      features_previous: List[Dict],
                                      matches: List[Tuple[int, int, float]]) -> Optional[Dict]:
        
        if len(matches) < 3:  # Need at least 3 matches for robust estimate
            return None

        # Extract matched positions
        idx_curr = [m[0] for m in matches]
        idx_prev = [m[1] for m in matches]

        points_curr = np.array([features_current[i]['position'] for i in idx_curr])
        points_prev = np.array([features_previous[j]['position'] for j in idx_prev])

        # Estimate rigid transformation using shared utility function
        R, translation, dx, dy, dtheta = estimate_rigid_transform_2d(
            points_prev, points_curr, return_format='2d'
        )

        # Compute centroids for residual calculation
        centroid_curr = np.mean(points_curr, axis=0)
        centroid_prev = np.mean(points_prev, axis=0)
        prev_centered = points_prev - centroid_prev

        # Compute residuals (for quality assessment)
        transformed_prev = (R @ prev_centered.T).T + centroid_curr
        residuals = np.linalg.norm(points_curr - transformed_prev, axis=1)
        mean_error = np.mean(residuals)

        # Estimate covariance (simplified: proportional to residual)
        # Better: Use covariance propagation from feature uncertainties
        cov_scale = max(mean_error, 0.01)
        transform_cov = np.eye(3) * (cov_scale ** 2)

        return {
            'transform': np.array([dx, dy, dtheta]),
            'num_matches': len(matches),
            'residual_error': mean_error,
            'covariance': transform_cov,
            'matched_indices': matches
        }

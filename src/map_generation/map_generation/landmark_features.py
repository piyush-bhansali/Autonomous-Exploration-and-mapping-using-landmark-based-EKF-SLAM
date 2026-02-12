#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import LaserScan
from typing import List, Dict, Tuple, Optional
from sklearn.decomposition import PCA


class LandmarkFeatureExtractor:


    def __init__(self,
                 min_points_per_line: int = 5,
                 min_line_length: float = 0.3,
                 max_gap: float = 0.2,
                 split_residual_threshold: float = 0.03,
                 merge_residual_threshold: float = 0.03,
                 merge_angle_threshold_deg: float = 12.0,
                 corner_angle_threshold: float = 50.0,
                 corner_neighbor_range: int = 6,
                 lidar_noise_sigma: float = 0.01):
       
        self.min_points = min_points_per_line
        self.min_length = min_line_length
        self.max_gap = max_gap
        self.split_threshold = split_residual_threshold
        self.merge_threshold = merge_residual_threshold
        self.merge_angle = np.radians(merge_angle_threshold_deg)
        self.corner_angle = np.radians(corner_angle_threshold)
        self.corner_neighbor_range = corner_neighbor_range

        self.lidar_sigma = lidar_noise_sigma  # σ = 0.01m → σ² = 0.0001

    def extract_features(self, scan_msg: LaserScan) -> List[Dict]:
        
        points = self._scan_to_cartesian(scan_msg)

        return self.extract_features_from_points(points)

    def extract_features_from_points(self, points: np.ndarray) -> List[Dict]:
        if len(points) < self.min_points:
            return []

        pts_2d = points[:, :2]

        lines, corners = self._extract_lines_and_corners(pts_2d)

        features = []

        # Add wall features
        for line in lines:
            rho = float(line['rho'])
            alpha = float(line['alpha'])
            cov = self._compute_wall_covariance(line)

            features.append({
                'type': 'wall',
                'rho': rho,
                'alpha': alpha,
                'covariance': cov,
                'strength': line['length'],
                'num_points': line['num_points'],
                'start_point': line['points'][0],
                'end_point': line['points'][-1],
                'points': line['points']
            })

        # Add corner features
        for corner in corners:
            cov = self._compute_corner_covariance(corner)

            features.append({
                'type': 'corner',
                'position': corner['position'],
                'covariance': cov,
                'strength': corner['sharpness'],
                'num_points': 1
            })

        return features

    def _scan_to_cartesian(self, scan_msg: LaserScan) -> np.ndarray:
        ranges = np.array(scan_msg.ranges)
        num_points = len(ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, num_points)

        valid_mask = (
            (ranges >= scan_msg.range_min) &
            (ranges <= scan_msg.range_max) &
            np.isfinite(ranges)
        )

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)

        return np.column_stack([x, y])

    def _extract_lines_and_corners(self, points: np.ndarray) -> Tuple[List[Dict], List[Dict]]:
        if len(points) < self.min_points:
            return [], []

        continuous_segments = self._split_on_gaps(points)

        all_lines = []
        all_corners = []

        for seg_points in continuous_segments:
            if len(seg_points) < self.min_points:
                continue

            intervals = self._split_segment_recursive(seg_points, 0, len(seg_points) - 1)
            lines = self._build_segments_from_intervals(seg_points, intervals)
            lines = self._merge_adjacent_segments(lines)
            corners = self._extract_corners_from_adjacent_lines(lines)

            all_lines.extend(lines)
            all_corners.extend(corners)

        return all_lines, all_corners

    def _split_on_gaps(self, points: np.ndarray) -> List[np.ndarray]:
        if len(points) < 2:
            return [points]

        dists = np.linalg.norm(points[1:] - points[:-1], axis=1)
        gap_indices = np.where(dists > self.max_gap)[0]

        segments = []
        start_idx = 0

        for gap_idx in gap_indices:
            end_idx = gap_idx + 1
            if end_idx - start_idx >= self.min_points:
                segments.append(points[start_idx:end_idx])
            start_idx = end_idx

        if len(points) - start_idx >= self.min_points:
            segments.append(points[start_idx:])

        return segments if segments else [points]

    def _split_segment_recursive(
        self, points: np.ndarray, start_idx: int, end_idx: int
    ) -> List[Tuple[int, int]]:
        """Recursively split one ordered segment by max residual to an endpoint line."""
        if end_idx - start_idx + 1 < self.min_points:
            return []

        segment_points = points[start_idx:end_idx + 1]
        line_params = self._fit_line_endpoints(segment_points)
        residuals = np.array([self._point_to_line_distance(p, line_params) for p in segment_points])

        split_local_idx = int(np.argmax(residuals))
        max_residual = float(residuals[split_local_idx])
        split_idx = start_idx + split_local_idx

        if max_residual <= self.split_threshold:
            return [(start_idx, end_idx)]

        # Do not split at segment boundaries.
        if split_idx <= start_idx or split_idx >= end_idx:
            return [(start_idx, end_idx)]

        left_count = split_idx - start_idx + 1
        right_count = end_idx - split_idx
        if left_count < self.min_points or right_count < self.min_points:
            return [(start_idx, end_idx)]

        left_intervals = self._split_segment_recursive(points, start_idx, split_idx)
        right_intervals = self._split_segment_recursive(points, split_idx + 1, end_idx)

        if len(left_intervals) == 0 or len(right_intervals) == 0:
            return [(start_idx, end_idx)]

        return left_intervals + right_intervals

    def _build_segments_from_intervals(
        self, points: np.ndarray, intervals: List[Tuple[int, int]]
    ) -> List[Dict]:
        """Convert split intervals to line segment dictionaries."""
        segments = []
        for start_idx, end_idx in intervals:
            if end_idx - start_idx + 1 < self.min_points:
                continue

            segment_points = points[start_idx:end_idx + 1]
            line = self._make_line_dict(segment_points)
            if line is not None:
                segments.append(line)

        return segments

    def _merge_adjacent_segments(self, segments: List[Dict]) -> List[Dict]:
        """Merge neighboring segments if geometry and fit criteria are satisfied."""
        if len(segments) <= 1:
            return segments

        merged = [segments[0]]
        for current in segments[1:]:
            previous = merged[-1]
            if self._should_merge_segments(previous, current):
                merged_points = np.vstack([previous['points'], current['points']])
                merged_line = self._make_line_dict(merged_points)
                if merged_line is not None:
                    merged[-1] = merged_line
                else:
                    merged.append(current)
            else:
                merged.append(current)

        return merged

    def _should_merge_segments(self, left: Dict, right: Dict) -> bool:
        angle = self._acute_angle_between_directions(left['direction'], right['direction'])
        if angle > self.merge_angle:
            return False

        endpoint_gap = np.linalg.norm(left['points'][-1] - right['points'][0])
        if endpoint_gap > self.max_gap:
            return False

        merged_points = np.vstack([left['points'], right['points']])
        merged_params = self._fit_line_endpoints(merged_points)
        max_dist = max(self._point_to_line_distance(p, merged_params) for p in merged_points)
        return max_dist <= self.merge_threshold

    def _extract_corners_from_adjacent_lines(self, lines: List[Dict]) -> List[Dict]:
        """Create corners from adjacent merged lines."""
        corners = []
        for i in range(len(lines) - 1):
            left = lines[i]
            right = lines[i + 1]

            angle = self._acute_angle_between_directions(left['direction'], right['direction'])
            if angle < self.corner_angle:
                continue

            corner_pos = self._compute_line_intersection(left, right)
            if corner_pos is None:
                corner_pos = 0.5 * (left['points'][-1] + right['points'][0])

            left_neighbors = left['points'][-self.corner_neighbor_range:]
            right_neighbors = right['points'][:self.corner_neighbor_range]
            neighbors = np.vstack([left_neighbors, right_neighbors])

            corners.append({
                'position': corner_pos,
                'sharpness': np.degrees(angle),
                'neighbors': neighbors
            })

        return corners

    def _make_line_dict(self, points: np.ndarray) -> Optional[Dict]:
        """Build validated line dictionary from ordered points."""
        if len(points) < self.min_points:
            return None

        length = np.linalg.norm(points[-1] - points[0])
        if length < self.min_length:
            return None

        line_params = self._fit_line_endpoints(points)
        rho, alpha = self._convert_line_to_hessian(points)
        return {
            'points': points,
            'length': length,
            'num_points': len(points),
            'centroid': line_params['centroid'],
            'direction': line_params['direction'],
            'rho': rho,
            'alpha': alpha
        }

    def _compute_line_intersection(self, line_a: Dict, line_b: Dict) -> Optional[np.ndarray]:
        """Compute intersection point between two lines (if not near-parallel)."""
        if 'rho' in line_a and 'alpha' in line_a:
            rho_a = float(line_a['rho'])
            alpha_a = float(line_a['alpha'])
        else:
            rho_a, alpha_a = self._convert_line_to_hessian(line_a['points'])

        if 'rho' in line_b and 'alpha' in line_b:
            rho_b = float(line_b['rho'])
            alpha_b = float(line_b['alpha'])
        else:
            rho_b, alpha_b = self._convert_line_to_hessian(line_b['points'])

        A = np.array([
            [np.cos(alpha_a), np.sin(alpha_a)],
            [np.cos(alpha_b), np.sin(alpha_b)]
        ])
        b = np.array([rho_a, rho_b])

        det = np.linalg.det(A)
        if abs(det) < 1e-6:
            return None

        return np.linalg.solve(A, b)

    def _acute_angle_between_directions(self, dir_a: np.ndarray, dir_b: np.ndarray) -> float:
        """Return acute angle between two direction vectors in radians."""
        dot = np.clip(np.dot(dir_a, dir_b), -1.0, 1.0)
        return np.arccos(np.abs(dot))

    def _fit_line_endpoints(self, points: np.ndarray) -> Dict:
       
        if len(points) < 2:
            return {
                'centroid': np.zeros(2),
                'direction': np.array([1.0, 0.0]),
                'point_on_line': np.zeros(2)
            }
        start, centroid, direction, _ = self._endpoint_line_primitives(points)

        return {
            'centroid': centroid,
            'direction': direction,
            'point_on_line': start
        }

    def _endpoint_line_primitives(
        self, points: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, bool]:
        
        start = points[0]
        end = points[-1]
        delta = end - start
        norm = np.linalg.norm(delta)

        if norm < 1e-9:
            # Defensive fallback for degenerate intervals.
            dists = np.linalg.norm(points - start, axis=1)
            far_idx = int(np.argmax(dists))
            delta = points[far_idx] - start
            norm = np.linalg.norm(delta)
            if norm < 1e-9:
                centroid = 0.5 * (start + end)
                return start, centroid, np.array([1.0, 0.0]), False

        centroid = 0.5 * (start + end)
        direction = delta / norm
        return start, centroid, direction, True

    def _point_to_line_distance(self, point: np.ndarray, line_params: Dict) -> float:
        point_on_line = line_params.get('point_on_line', line_params['centroid'])
        direction = line_params['direction']

        # Vector from line anchor to point
        v = point - point_on_line

        # Project onto line
        proj_length = np.dot(v, direction)
        proj = proj_length * direction

        # Perpendicular distance
        perp = v - proj
        return np.linalg.norm(perp)

    def _convert_line_to_hessian(self, points: np.ndarray) -> Tuple[float, float]:
        if len(points) < 2:
            return 0.0, 0.0
        _, line_point, direction, valid_direction = self._endpoint_line_primitives(points)
        if not valid_direction:
            return 0.0, 0.0

        # Normal to line (perpendicular, 90° rotation)
        normal = np.array([-direction[1], direction[0]])

        # Distance from origin to line along normal
        rho = np.dot(line_point, normal)

        # Ensure rho is positive (flip normal if needed)
        if rho < 0:
            rho = -rho
            normal = -normal

        # Angle of normal vector
        alpha = np.arctan2(normal[1], normal[0])

        return rho, alpha


    def _compute_wall_covariance(self, line: Dict) -> np.ndarray:
       
        points = line['points']
        if len(points) < 2:
            return np.eye(2) * 0.1

        if 'alpha' in line:
            alpha = float(line['alpha'])
        else:
            _, alpha = self._convert_line_to_hessian(points)

        cos_a = np.cos(alpha)
        sin_a = np.sin(alpha)

        A = np.zeros((2, 2))

        for px, py in points:
            # Jacobian of constraint c = rho - (px*cos(alpha) + py*sin(alpha))
            # ∂c/∂rho = 1.0
            # ∂c/∂alpha = px*sin(alpha) - py*cos(alpha)
            dr_d_rho = 1.0
            dr_d_alpha = px * sin_a - py * cos_a

            J = np.array([[dr_d_rho, dr_d_alpha]])

            A += J.T @ J

        sigma2 = self.lidar_sigma ** 2

        try:
            A_inv = np.linalg.inv(A)
        except np.linalg.LinAlgError:
            # Singular matrix (degenerate case) - use pseudo-inverse
            A_inv = np.linalg.pinv(A)

        
        return sigma2 * A_inv

    def _compute_corner_covariance(self, corner: Dict) -> np.ndarray:

        neighbors = corner.get('neighbors', None)
        if neighbors is None or len(neighbors) < 3:
            
            sigma2 = self.lidar_sigma ** 2
            return np.eye(2) * sigma2 * 10  

        n_points = len(neighbors)

        pca = PCA(n_components=2)
        pca.fit(neighbors)

        # Eigenvalues represent spread of neighboring points
        # Large spread → high uncertainty (large covariance)
        # Small spread → low uncertainty (small covariance)
        evs = np.maximum(pca.explained_variance_, 1e-6)

        # More observations → lower uncertainty
        effective_n = min(n_points, 30)
        cov_eigenvalues = evs / effective_n

        # Construct covariance matrix in PCA basis
        V = pca.components_.T  # Eigenvectors as columns
        Lambda = np.diag(cov_eigenvalues)
        corner_cov = V @ Lambda @ V.T

        sigma2 = self.lidar_sigma ** 2

        # Scale by measurement noise and add minimum uncertainty
        return sigma2 * (corner_cov + np.eye(2) * 1e-6)

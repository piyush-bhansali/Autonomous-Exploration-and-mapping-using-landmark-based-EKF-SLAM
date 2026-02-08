#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import LaserScan
from typing import List, Dict, Tuple
from sklearn.decomposition import PCA


class LandmarkFeatureExtractor:
   

    def __init__(self,
                 min_points_per_line: int = 10,
                 line_fit_threshold: float = 0.05,
                 min_line_length: float = 0.5,
                 corner_angle_threshold: float = 45.0):
        self.min_points = min_points_per_line
        self.fit_threshold = line_fit_threshold
        self.min_length = min_line_length
        self.corner_angle = np.radians(corner_angle_threshold)

    def extract_features(self, scan_msg: LaserScan) -> List[Dict]:
        
        # Convert scan to points
        points = self._scan_to_cartesian(scan_msg)

        return self.extract_features_from_points(points)

    def extract_features_from_points(self, points: np.ndarray) -> List[Dict]:

        if len(points) < self.min_points:
            return []

        pts_2d = points[:, :2] if points.shape[1] >= 2 else points

        # For laser scans, points are already ordered sequentially
        # Don't re-sort them - this preserves the scan order and gap information

        # Extract line segments with gap detection
        lines = self._extract_lines(pts_2d)

        # Detect corners from line intersections (not just adjacent lines)
        corners = self._detect_corners_from_lines(lines)

        features = []

        # Add wall features (Hessian form)
        for line in lines:
            rho, alpha = self._convert_line_to_hessian(line['points'])

            # Compute covariance based on line quality
            cov = self._compute_wall_covariance(line)

            features.append({
                'type': 'wall',
                'rho': rho,
                'alpha': alpha,
                'covariance': cov,
                'strength': line['length'],  # For filtering
                'num_points': line['num_points']
            })

        for corner in corners:
            cov = self._compute_corner_covariance(corner)

            features.append({
                'type': 'corner',
                'position': corner['position'],  # [x, y]
                'covariance': cov,
                'strength': corner['sharpness'],  # Angle change in degrees
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

    def _split_on_gaps(self, points: np.ndarray, max_gap: float = 0.2) -> List[Tuple[np.ndarray, int]]:
        """
        Split point cloud into continuous segments based on inter-point distance.
        Returns list of (segment_points, original_start_index) tuples.
        """
        if len(points) < 2:
            return [(points, 0)]

        # Compute distances between consecutive points
        dists = np.linalg.norm(points[1:] - points[:-1], axis=1)

        # Find gap indices (where distance exceeds threshold)
        gap_indices = np.where(dists > max_gap)[0]

        # Split into continuous segments
        segments = []
        start_idx = 0

        for gap_idx in gap_indices:
            end_idx = gap_idx + 1  # Include the point before the gap
            if end_idx - start_idx >= self.min_points:
                segments.append((points[start_idx:end_idx], start_idx))
            start_idx = end_idx

        # Add final segment
        if len(points) - start_idx >= self.min_points:
            segments.append((points[start_idx:], start_idx))

        return segments if segments else [(points, 0)]

    def _extract_lines(self, points: np.ndarray) -> List[Dict]:

        if len(points) < self.min_points:
            return []

        # Step 1: Detect gaps in the scan and split into continuous segments
        continuous_segments = self._split_on_gaps(points, max_gap=0.2)

        # Step 2: Apply split-and-merge to each continuous segment
        all_segments = []
        for seg_points, seg_offset in continuous_segments:
            stack = [(0, len(seg_points))]
            local_segments = []

            while stack:
                start, end = stack.pop()
                if (end - start) < self.min_points:
                    continue

                pts = seg_points[start:end]
                line_params = self._fit_line_pca(pts)
                dists = np.array([self._point_to_line_distance(p, line_params) for p in pts])
                max_idx = int(np.argmax(dists))
                max_dist = float(dists[max_idx])

                can_split = (
                    max_dist > self.fit_threshold and
                    (end - start) >= 2 * self.min_points and
                    max_idx >= self.min_points and
                    ((end - start) - max_idx) >= self.min_points
                )

                if can_split:
                    split_idx = start + max_idx
                    stack.append((split_idx, end))
                    stack.append((start, split_idx + 1))
                else:
                    # Store with global indices
                    local_segments.append((start, end, seg_points, seg_offset))

            all_segments.extend(local_segments)

        # Step 3: Build line features
        lines = []
        for start, end, seg_points, seg_offset in all_segments:
            pts = seg_points[start:end]
            if len(pts) < self.min_points:
                continue
            length = np.linalg.norm(pts[-1] - pts[0])
            if length < self.min_length:
                continue
            line_params = self._fit_line_pca(pts)
            lines.append({
                'points': pts,
                'length': length,
                'num_points': len(pts),
                'start_idx': seg_offset + start,
                'end_idx': seg_offset + end - 1,
                'centroid': line_params['centroid'],
                'direction': line_params['direction']
            })

        return lines

    def _pca_direction(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        
        if len(points) < 2:
            return np.zeros(2), np.array([1.0, 0.0]), np.array([0.0, 0.0])

        pca = PCA(n_components=2)
        pca.fit(points)
        centroid = pca.mean_
        direction = pca.components_[0]
        eigenvalues = pca.explained_variance_

        return centroid, direction, eigenvalues

    def _fit_line_pca(self, points: np.ndarray) -> Dict:
        centroid, direction, eigenvalues = self._pca_direction(points)

        return {
            'centroid': centroid,
            'direction': direction,
            'variance': float(eigenvalues[0]) if len(eigenvalues) > 0 else 0.0
        }

    def _point_to_line_distance(self, point: np.ndarray, line_params: Dict) -> float:
        
        centroid = line_params['centroid']
        direction = line_params['direction']

        # Vector from centroid to point
        v = point - centroid

        # Project onto line
        proj_length = np.dot(v, direction)
        proj = proj_length * direction

        # Perpendicular distance
        perp = v - proj
        return np.linalg.norm(perp)

    def _convert_line_to_hessian(self, points: np.ndarray) -> Tuple[float, float]:
        centroid, direction, _ = self._pca_direction(points)

        # Normal to line (perpendicular, 90° rotation)
        normal = np.array([-direction[1], direction[0]])

        # Distance from origin to line along normal
        rho = np.dot(centroid, normal)

        # Ensure rho is positive (flip normal if needed)
        if rho < 0:
            rho = -rho
            normal = -normal

        # Angle of normal vector
        alpha = np.arctan2(normal[1], normal[0])

        return rho, alpha

    def _detect_corners_from_lines(self, lines: List[Dict]) -> List[Dict]:
        """
        Detect corners by finding line intersections.
        Checks all line pairs within spatial proximity, not just adjacent ones.
        """
        corners = []
        if len(lines) < 2:
            return corners

        # Maximum distance between line endpoints to consider for corner detection
        max_proximity = 1.0  # meters
        neighbor_span = 6

        # Check all pairs of lines (not just adjacent)
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                line_a = lines[i]
                line_b = lines[j]

                dir_a = line_a['direction']
                dir_b = line_b['direction']

                # Check angle between lines (without abs to distinguish direction)
                dot = np.clip(np.dot(dir_a, dir_b), -1.0, 1.0)
                angle = np.arccos(np.abs(dot))  # Angle between 0 and π/2

                # Require significant angle change (not parallel)
                if angle < self.corner_angle:
                    continue

                # Check spatial proximity: are the line endpoints close?
                # Get all endpoint combinations
                end_a = line_a['points'][-1]
                start_a = line_a['points'][0]
                end_b = line_b['points'][-1]
                start_b = line_b['points'][0]

                # Find closest endpoint pair
                distances = [
                    np.linalg.norm(end_a - start_b),
                    np.linalg.norm(end_a - end_b),
                    np.linalg.norm(start_a - start_b),
                    np.linalg.norm(start_a - end_b)
                ]
                min_dist = min(distances)
                min_idx = distances.index(min_dist)

                # Skip if lines are too far apart
                if min_dist > max_proximity:
                    continue

                # Compute intersection of infinite lines: p = p0 + t*d
                p0 = line_a['centroid']
                d0 = dir_a
                p1 = line_b['centroid']
                d1 = dir_b

                denom = d0[0] * d1[1] - d0[1] * d1[0]
                if np.abs(denom) < 1e-6:  # Nearly parallel
                    continue

                # Solve for intersection
                t = ((p1[0] - p0[0]) * d1[1] - (p1[1] - p0[1]) * d1[0]) / denom
                intersection = p0 + t * d0

                # Validate intersection is near the endpoint pair
                if min_idx == 0:  # end_a <-> start_b
                    ref_point = 0.5 * (end_a + start_b)
                    neighbors = np.vstack([line_a['points'][-neighbor_span:],
                                          line_b['points'][:neighbor_span]])
                elif min_idx == 1:  # end_a <-> end_b
                    ref_point = 0.5 * (end_a + end_b)
                    neighbors = np.vstack([line_a['points'][-neighbor_span:],
                                          line_b['points'][-neighbor_span:]])
                elif min_idx == 2:  # start_a <-> start_b
                    ref_point = 0.5 * (start_a + start_b)
                    neighbors = np.vstack([line_a['points'][:neighbor_span],
                                          line_b['points'][:neighbor_span]])
                else:  # start_a <-> end_b
                    ref_point = 0.5 * (start_a + end_b)
                    neighbors = np.vstack([line_a['points'][:neighbor_span],
                                          line_b['points'][-neighbor_span:]])

                # If intersection is far from endpoints, use midpoint
                if np.linalg.norm(intersection - ref_point) > max_proximity:
                    intersection = ref_point

                # Avoid duplicate corners
                is_duplicate = False
                for existing in corners:
                    if np.linalg.norm(existing['position'] - intersection) < 0.1:
                        is_duplicate = True
                        break

                if not is_duplicate:
                    corners.append({
                        'position': intersection,
                        'sharpness': np.degrees(angle),
                        'neighbors': neighbors
                    })

        return corners

    def _compute_wall_covariance(self, line: Dict) -> np.ndarray:
        points = line['points']
        if len(points) < 2:
            return np.eye(2) * 0.1

        rho, alpha = self._convert_line_to_hessian(points)

        cos_a = np.cos(alpha)
        sin_a = np.sin(alpha)

        # Residuals: r_i = n·p_i - rho
        x = points[:, 0]
        y = points[:, 1]
        residuals = cos_a * x + sin_a * y - rho

        # Jacobian wrt [rho, alpha]
        dr_d_rho = -np.ones_like(x)
        dr_d_alpha = -sin_a * x + cos_a * y

        J = np.column_stack([dr_d_rho, dr_d_alpha])

        H = J.T @ J
        dof = max(len(points) - 2, 1)
        sigma2_residual = float(residuals.T @ residuals) / dof

        try:
            H_inv = np.linalg.inv(H)
        except np.linalg.LinAlgError:
            H_inv = np.linalg.pinv(H)

        # Geometric uncertainty from fit
        P_geometric = sigma2_residual * H_inv

        # Add minimum measurement uncertainty (sensor noise floor)
        # Typical LiDAR: ~1cm range, ~0.5° angular
        min_cov = np.diag([0.01**2, np.radians(0.5)**2])

        # Combine uncertainties
        return P_geometric + min_cov

    def _compute_corner_covariance(self, corner: Dict) -> np.ndarray:
        neighbors = corner.get('neighbors', None)
        if neighbors is None or len(neighbors) < 3:
            # Fallback: isotropic uncertainty
            return np.eye(2) * 0.05**2

        centroid, direction, eigenvalues = self._pca_direction(neighbors)
        if len(eigenvalues) < 2:
            return np.eye(2) * 0.05**2

        # Compute proper sample covariance with Bessel's correction
        centered = neighbors - centroid
        n_samples = len(neighbors)
        cov_sample = (centered.T @ centered) / max(n_samples - 1, 1)

        # Add minimum measurement uncertainty (sensor noise floor)
        min_cov = np.eye(2) * 0.01**2  # 1cm in each direction

        # Combine uncertainties
        return cov_sample + min_cov

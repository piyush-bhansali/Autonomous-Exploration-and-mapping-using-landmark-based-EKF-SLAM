#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import LaserScan
from typing import List, Dict, Tuple
from sklearn.decomposition import PCA


class LandmarkFeatureExtractor:


    def __init__(self,
                 min_points_per_line: int = 5,
                 line_fit_threshold: float = 0.03,
                 min_line_length: float = 0.3,
                 corner_angle_threshold: float = 50.0,
                 max_gap: float = 0.2,
                 lidar_noise_sigma: float = 0.01):
        """
        Simplified feature extractor using angle-based breakpoint detection.

        Args:
            min_points_per_line: Minimum points to form a line segment (default: 5)
            line_fit_threshold: Max perpendicular distance for line validation (default: 0.03m)
            min_line_length: Minimum length of a line segment (default: 0.3m)
            corner_angle_threshold: Angle change to detect corners in degrees (default: 50°)
            max_gap: Maximum distance between consecutive points (default: 0.2m)
            lidar_noise_sigma: LiDAR measurement noise std dev (default: 0.01m)
        """
        self.min_points = min_points_per_line
        self.fit_threshold = line_fit_threshold
        self.min_length = min_line_length
        self.corner_angle = np.radians(corner_angle_threshold)
        self.max_gap = max_gap

        self.lidar_sigma = lidar_noise_sigma  # σ = 0.01m → σ² = 0.0001

    def extract_features(self, scan_msg: LaserScan) -> List[Dict]:
        
        points = self._scan_to_cartesian(scan_msg)

        return self.extract_features_from_points(points)

    def extract_features_from_points(self, points: np.ndarray) -> List[Dict]:
        """
        Extract wall and corner features using simplified angle-based detection.
        """
        if len(points) < self.min_points:
            return []

        pts_2d = points[:, :2] if points.shape[1] >= 2 else points

        # New simplified approach: detect breakpoints then fit lines
        lines, corners = self._extract_lines_and_corners(pts_2d)

        features = []

        # Add wall features
        for line in lines:
            rho, alpha = self._convert_line_to_hessian(line['points'])
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
        """
        Simplified algorithm: detect breakpoints from angle changes, then fit lines.

        Returns:
            (lines, corners) - both as lists of dictionaries
        """
        if len(points) < self.min_points:
            return [], []

        # Step 1: Split on gaps (occlusions)
        continuous_segments = self._split_on_gaps(points)

        all_lines = []
        all_corners = []

        # Step 2: For each continuous segment, detect angle-based breakpoints
        for seg_points, seg_offset in continuous_segments:
            if len(seg_points) < self.min_points:
                continue

            # Find breakpoints based on angle changes
            breakpoints = self._find_breakpoints(seg_points)

            # Fit lines between breakpoints and extract corners
            lines, corners = self._fit_lines_at_breakpoints(seg_points, breakpoints)

            all_lines.extend(lines)
            all_corners.extend(corners)

        return all_lines, all_corners

    def _split_on_gaps(self, points: np.ndarray) -> List[Tuple[np.ndarray, int]]:
        """Split point cloud into continuous segments based on distance gaps."""
        if len(points) < 2:
            return [(points, 0)]

        # Compute distances between consecutive points
        dists = np.linalg.norm(points[1:] - points[:-1], axis=1)

        # Find gap indices (where distance exceeds threshold)
        gap_indices = np.where(dists > self.max_gap)[0]

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

    def _find_breakpoints(self, points: np.ndarray) -> List[int]:
        """
        Detect breakpoints (corners) based on angle changes between consecutive points.
        Uses a sliding window to reduce noise sensitivity.

        Returns:
            List of indices where sharp angle changes occur
        """
        if len(points) < 3:
            return []

        breakpoints = [0]  # Always start at beginning

        # Use a window to smooth out noise (look ahead/behind more points)
        window = 3  # Look at 3 points back and 3 points forward

        for i in range(window, len(points) - window):
            # Vector from point (i-window) to i
            v1 = points[i] - points[i - window]
            # Vector from point i to (i+window)
            v2 = points[i + window] - points[i]

            # Skip if vectors are too small (noisy/close points)
            len_v1 = np.linalg.norm(v1)
            len_v2 = np.linalg.norm(v2)

            if len_v1 < 0.05 or len_v2 < 0.05:
                continue

            # Normalize
            v1 = v1 / len_v1
            v2 = v2 / len_v2

            # Compute angle between vectors
            dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
            angle = np.arccos(dot)

            # If angle change exceeds threshold, mark as breakpoint (corner)
            if angle > self.corner_angle:
                # Avoid duplicates too close together
                if not breakpoints or (i - breakpoints[-1]) > window:
                    breakpoints.append(i)

        breakpoints.append(len(points) - 1)  # Always end at last point

        return breakpoints

    def _fit_lines_at_breakpoints(self, points: np.ndarray, breakpoints: List[int]) -> Tuple[List[Dict], List[Dict]]:
        """
        Fit line segments between breakpoints and extract corner features.

        Returns:
            (lines, corners)
        """
        lines = []
        corners = []

        for i in range(len(breakpoints) - 1):
            start_idx = breakpoints[i]
            end_idx = breakpoints[i + 1]

            # Extract points for this segment
            segment_points = points[start_idx:end_idx + 1]

            if len(segment_points) < self.min_points:
                continue

            # Fit line using PCA
            line_params = self._fit_line_pca(segment_points)

            # Validate line fit quality
            max_dist = max([self._point_to_line_distance(p, line_params) for p in segment_points])
            if max_dist > self.fit_threshold:
                # Line doesn't fit well - skip or recursively split
                continue

            # Check minimum length
            length = np.linalg.norm(segment_points[-1] - segment_points[0])
            if length < self.min_length:
                continue

            # Store line
            lines.append({
                'points': segment_points,
                'length': length,
                'num_points': len(segment_points),
                'centroid': line_params['centroid'],
                'direction': line_params['direction']
            })

            # Extract corner at breakpoint (except at start/end of scan)
            if i > 0:  # Not the first segment
                corner_idx = breakpoints[i]
                corner_pos = points[corner_idx]

                # Get neighboring points for covariance calculation
                neighbor_range = 6
                neighbor_start = max(0, corner_idx - neighbor_range)
                neighbor_end = min(len(points), corner_idx + neighbor_range + 1)
                neighbors = points[neighbor_start:neighbor_end]

                # Compute sharpness (angle between adjacent segments)
                if i < len(breakpoints) - 1:
                    prev_dir = lines[-1]['direction'] if lines else np.array([1.0, 0.0])
                    next_segment_points = points[breakpoints[i]:breakpoints[i+1]+1]
                    if len(next_segment_points) >= 2:
                        next_params = self._fit_line_pca(next_segment_points)
                        next_dir = next_params['direction']

                        dot = np.clip(np.dot(prev_dir, next_dir), -1.0, 1.0)
                        angle = np.arccos(np.abs(dot))
                        sharpness = np.degrees(angle)
                    else:
                        sharpness = 45.0
                else:
                    sharpness = 45.0

                corners.append({
                    'position': corner_pos,
                    'sharpness': sharpness,
                    'neighbors': neighbors
                })

        return lines, corners

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


    def _compute_wall_covariance(self, line: Dict) -> np.ndarray:
       
        points = line['points']
        if len(points) < 2:
            return np.eye(2) * 0.1

        rho, alpha = self._convert_line_to_hessian(points)

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

        corner_pos = corner['position']
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

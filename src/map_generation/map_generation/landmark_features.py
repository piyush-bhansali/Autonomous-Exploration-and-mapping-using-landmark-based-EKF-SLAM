#!/usr/bin/env python3
"""
Landmark features with proper parameterization using scipy:
- Walls/Lines: Hessian Normal Form (ρ, α)
- Corners: Cartesian (x, y)

Uses scipy for geometry:
- SVD for line fitting
- Proper covariance calculation based on feature quality
"""

import numpy as np
from sensor_msgs.msg import LaserScan
from typing import List, Dict, Tuple
from scipy.linalg import svd


class LandmarkFeatureExtractor:
   

    def __init__(self,
                 min_points_per_line: int = 8,
                 line_fit_threshold: float = 0.03,
                 min_line_length: float = 0.3,
                 corner_angle_threshold: float = 25.0):
        self.min_points = min_points_per_line
        self.fit_threshold = line_fit_threshold
        self.min_length = min_line_length
        self.corner_angle = np.radians(corner_angle_threshold)

    def extract_features(self, scan_msg: LaserScan) -> List[Dict]:
        
        # Convert scan to points
        points = self._scan_to_cartesian(scan_msg)

        return self.extract_features_from_points(points)

    def extract_features_from_points(self, points: np.ndarray) -> List[Dict]:
        """Extract wall/corner features from Nx2 or Nx3 point arrays."""
        if len(points) < self.min_points:
            return []

        pts_2d = points[:, :2] if points.shape[1] >= 2 else points

        # Extract line segments
        lines = self._extract_lines(pts_2d)

        # Detect corners
        corners = self._detect_corners(pts_2d)

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

    def _extract_lines(self, points: np.ndarray) -> List[Dict]:
        """Extract line segments using incremental fitting."""
        if len(points) < self.min_points:
            return []

        lines = []
        current_line_points = []

        for i in range(len(points)):
            current_line_points.append(points[i])

            if len(current_line_points) >= 2:
                # Fit line
                line_params = self._fit_line_pca(np.array(current_line_points))

                # Check if new point fits
                dist = self._point_to_line_distance(points[i], line_params)

                if dist > self.fit_threshold:
                    # Finalize previous line
                    if len(current_line_points) > self.min_points:
                        line_pts = np.array(current_line_points[:-1])
                        length = np.linalg.norm(line_pts[-1] - line_pts[0])

                        if length >= self.min_length:
                            lines.append({
                                'points': line_pts,
                                'length': length,
                                'num_points': len(line_pts)
                            })

                    # Start new line
                    current_line_points = [points[i]]

        # Handle last line
        if len(current_line_points) > self.min_points:
            line_pts = np.array(current_line_points)
            length = np.linalg.norm(line_pts[-1] - line_pts[0])

            if length >= self.min_length:
                lines.append({
                    'points': line_pts,
                    'length': length,
                    'num_points': len(line_pts)
                })

        return lines

    def _pca_direction(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute PCA using SciPy SVD.

        Returns:
            centroid: mean of points
            direction: principal direction (unit vector)
            eigenvalues: variance along principal axes
        """
        if len(points) == 0:
            return np.zeros(2), np.array([1.0, 0.0]), np.array([0.0, 0.0])

        centroid = np.mean(points, axis=0)
        centered = points - centroid

        # SVD on centered data
        _, s_vals, vt = svd(centered, full_matrices=False)
        direction = vt[0]

        # Eigenvalues of covariance matrix
        n = max(len(points) - 1, 1)
        eigenvalues = (s_vals ** 2) / n

        return centroid, direction, eigenvalues

    def _fit_line_pca(self, points: np.ndarray) -> Dict:
        centroid, direction, eigenvalues = self._pca_direction(points)

        return {
            'centroid': centroid,
            'direction': direction,
            'variance': float(eigenvalues[0]) if len(eigenvalues) > 0 else 0.0
        }

    def _point_to_line_distance(self, point: np.ndarray, line_params: Dict) -> float:
        """Distance from point to line."""
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

    def _detect_corners(self, points: np.ndarray) -> List[Dict]:
       
        corners = []

        # Simple corner detection via range jumps
        for i in range(1, len(points) - 1):
            prev = points[i-1]
            curr = points[i]
            next_pt = points[i+1]

            # Range discontinuity
            dist_prev = np.linalg.norm(curr - prev)
            dist_next = np.linalg.norm(next_pt - curr)

            if dist_prev > 0.5 or dist_next > 0.5:
                continue

            # Check angle
            v1 = curr - prev
            v2 = next_pt - curr

            v1_norm = v1 / (np.linalg.norm(v1) + 1e-6)
            v2_norm = v2 / (np.linalg.norm(v2) + 1e-6)

            dot_product = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
            angle = np.arccos(dot_product)

            if angle > self.corner_angle:
                sharpness = np.degrees(angle)
                corners.append({
                    'position': curr,
                    'sharpness': sharpness,
                    'index': i,
                    'neighbors': points[max(0, i-6):min(len(points), i+7)]
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
        sigma2 = float(residuals.T @ residuals) / dof

        try:
            H_inv = np.linalg.inv(H)
        except np.linalg.LinAlgError:
            H_inv = np.linalg.pinv(H)

        return sigma2 * H_inv

    def _compute_corner_covariance(self, corner: Dict) -> np.ndarray:
        neighbors = corner.get('neighbors', None)
        if neighbors is None or len(neighbors) < 3:
            sharpness = corner['sharpness']
            sigma = np.clip(0.15 - sharpness / 1000.0, 0.03, 0.1)
            return np.eye(2) * sigma**2

        centroid, direction, eigenvalues = self._pca_direction(neighbors)
        if len(eigenvalues) < 2:
            return np.eye(2) * 0.05**2

        # Build covariance from PCA spread
        # Principal axis = direction, orthogonal axis = [-dy, dx]
        direction = direction / (np.linalg.norm(direction) + 1e-12)
        ortho = np.array([-direction[1], direction[0]])
        R = np.column_stack([direction, ortho])

        cov = R @ np.diag(eigenvalues[:2]) @ R.T
        cov = cov / max(len(neighbors), 1)

        return cov

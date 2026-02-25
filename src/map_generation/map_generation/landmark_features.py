#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import LaserScan
from typing import List, Dict, Tuple, Optional


class LandmarkFeatureExtractor:

    def __init__(self,
                 min_points_per_line: int = 5,
                 min_line_length: float = 0.3,
                 corner_angle_threshold: float = 45.0,
                 max_gap: float = 0.5,
                 lidar_noise_sigma: float = 0.01,
                 merge_angle_tolerance: float = 0.350,
                 merge_rho_tolerance: float = 0.2,
                 grow_residual_threshold: float = 0.03):
      
        self.min_points = min_points_per_line
        self.min_length = min_line_length
        self.corner_angle = np.radians(corner_angle_threshold)
        self.max_gap = max_gap
        self.merge_angle_tol = merge_angle_tolerance
        self.merge_residual_tol = merge_rho_tolerance
        self.grow_residual_threshold = grow_residual_threshold

        self.lidar_sigma = lidar_noise_sigma

    def extract_features(self, scan_msg: LaserScan) -> List[Dict]:
        points = self.scan_to_cartesian(scan_msg)
        if len(points) < self.min_points:
            return []

        pts_2d = points[:, :2] if points.shape[1] >= 2 else points
        lines, corners = self.extract_lines_and_corners(pts_2d)

        features = []

        for line in lines:
            cov = line.get('covariance', self.compute_wall_covariance(line))
            features.append({
                'type': 'wall',
                'rho': line['rho'],
                'alpha': line['alpha'],
                'covariance': cov,
                'strength': line['length'],
                'num_points': line['num_points'],
                'start_point': line['points'][0],
                'end_point': line['points'][-1],
                'points': line['points']
            })

        for corner in corners:
            cov = corner.get('covariance')
            if cov is None:
                sigma2 = self.lidar_sigma ** 2
                cov = np.eye(2) * sigma2 * 10
            features.append({
                'type': 'corner',
                'position': corner['position'],
                'covariance': cov,
                'strength': corner['sharpness'],
                'num_points': 1
            })

        return features

    def scan_to_cartesian(self, scan_msg: LaserScan) -> np.ndarray:
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

    def extract_lines_and_corners(self, points: np.ndarray) -> Tuple[List[Dict], List[Dict]]:
        if len(points) < self.min_points:
            return [], []

        continuous_segments = self.split_on_gaps(points)

        all_lines: List[Dict] = []
        all_corners: List[Dict] = []

        for seg_points, _ in continuous_segments:
            if len(seg_points) < self.min_points:
                continue

            grown_segments = self.grow_lines_incremental(seg_points)
            merged_segments = self.merge_adjacent_lines(grown_segments)

            valid_lines = []
            for seg in merged_segments:
                line = self.make_line_dict(seg)
                if line is not None:
                    valid_lines.append(line)

            corners = self.extract_corners_from_adjacent_lines(valid_lines)

            all_lines.extend(valid_lines)
            all_corners.extend(corners)

        return all_lines, all_corners

    def split_on_gaps(self, points: np.ndarray) -> List[Tuple[np.ndarray, int]]:
        if len(points) < 2:
            return [(points, 0)]

        dists = np.linalg.norm(points[1:] - points[:-1], axis=1)
        gap_indices = np.where(dists > self.max_gap)[0]

        segments = []
        start_idx = 0

        for gap_idx in gap_indices:
            end_idx = gap_idx + 1
            if end_idx - start_idx >= self.min_points:
                segments.append((points[start_idx:end_idx], start_idx))
            start_idx = end_idx

        if len(points) - start_idx >= self.min_points:
            segments.append((points[start_idx:], start_idx))

        return segments if segments else [(points, 0)]

    def grow_lines_incremental(self, points: np.ndarray) -> List[np.ndarray]:
        n = len(points)
        if n < self.min_points:
            return []

        segments: List[np.ndarray] = []
        start = 0
        i = start + 1

        while i < n:
            candidate = points[start:i + 1]
            residual = self.segment_residual_tls(candidate)

            if residual <= self.grow_residual_threshold:
                i += 1
                continue

            # Finalize previous valid segment (exclude violating point i).
            segment = self.try_finalize_segment(points[start:i])
            if segment is not None:
                segments.append(segment)

            # Restart with shared boundary (i-1), retaining continuity once.
            start = max(i - 1, start + 1)
            if start >= n - 1:
                break
            i = start + 1

        tail = self.try_finalize_segment(points[start:n])
        if tail is not None:
            segments.append(tail)

        return segments

    def fit_line_tls(self, points: np.ndarray):
        
        centroid = points.mean(axis=0)
        centered = points - centroid
        
        _, _, Vt = np.linalg.svd(centered, full_matrices=False)
        direction = Vt[0]  # principal axis (line direction)
        normal = np.array([-direction[1], direction[0]])  # perpendicular
        return centroid, direction, normal

    def segment_residual_tls(self, segment_points: np.ndarray) -> float:
        
        if len(segment_points) < 2:
            return 0.0
        if len(segment_points) == 2:
            return 0.0 

        centroid, _, normal = self.fit_line_tls(segment_points)
        dists = np.abs((segment_points - centroid) @ normal)
        return float(np.max(dists))

    def try_finalize_segment(self, segment_points: np.ndarray) -> Optional[np.ndarray]:
        if len(segment_points) < self.min_points:
            return None

        length = np.linalg.norm(segment_points[-1] - segment_points[0])
        if length < self.min_length:
            return None

        return segment_points

    def merge_adjacent_lines(self, segments: List[np.ndarray]) -> List[np.ndarray]:
        if len(segments) < 2:
            return segments

        merged = [segments[0]]

        for seg in segments[1:]:
            current = merged[-1]
            dir_curr = self.segment_direction_endpoints(current)
            dir_seg = self.segment_direction_endpoints(seg)
            angle_diff = self.acute_angle_between_dirs(dir_curr, dir_seg)
            endpoint_gap = np.linalg.norm(seg[0] - current[-1])

            if len(current) > 0 and len(seg) > 0 and np.allclose(current[-1], seg[0]):
                candidate = np.vstack([current, seg[1:]])
            else:
                candidate = np.vstack([current, seg])

            residual = self.segment_residual_tls(candidate)

            if (angle_diff <= self.merge_angle_tol and
                    endpoint_gap <= self.max_gap and
                    residual <= self.merge_residual_tol):
                merged[-1] = candidate
            else:
                merged.append(seg)

        return merged

    def segment_direction_endpoints(self, points: np.ndarray) -> np.ndarray:
        if len(points) < 2:
            return np.array([1.0, 0.0])

        delta = points[-1] - points[0]
        norm = np.linalg.norm(delta)
        if norm < 1e-9:
            return np.array([1.0, 0.0])
        return delta / norm

    def acute_angle_between_dirs(self, d1: np.ndarray, d2: np.ndarray) -> float:
        dot = np.clip(np.dot(d1, d2), -1.0, 1.0)
        return np.arccos(abs(dot))

    def convert_line_to_hessian(self, points: np.ndarray) -> Tuple[float, float]:
       
        centroid, _, normal = self.fit_line_tls(points)

        rho = float(np.dot(centroid, normal))
        if rho < 0.0:
            rho = -rho
            normal = -normal

        alpha = float(np.arctan2(normal[1], normal[0]))
        return rho, alpha

    def make_line_dict(self, points: np.ndarray) -> Optional[Dict]:
        segment = self.try_finalize_segment(points)
        if segment is None:
            return None

        rho, alpha = self.convert_line_to_hessian(segment)
        direction = self.segment_direction_endpoints(segment)

        line = {
            'points': segment,
            'length': float(np.linalg.norm(segment[-1] - segment[0])),
            'num_points': len(segment),
            'centroid': 0.5 * (segment[0] + segment[-1]),
            'direction': direction,
            'rho': rho,
            'alpha': alpha
        }
        line['covariance'] = self.compute_wall_covariance(line)
        return line

    def compute_line_intersection(self, line_a: Dict, line_b: Dict) -> Optional[np.ndarray]:
        rho_a = float(line_a['rho'])
        alpha_a = float(line_a['alpha'])
        rho_b = float(line_b['rho'])
        alpha_b = float(line_b['alpha'])

        A = np.array([
            [np.cos(alpha_a), np.sin(alpha_a)],
            [np.cos(alpha_b), np.sin(alpha_b)]
        ])
        b = np.array([rho_a, rho_b])

        det = np.linalg.det(A)
        if abs(det) < 1e-6:
            return None

        return np.linalg.solve(A, b)

    def extract_corners_from_adjacent_lines(self, lines: List[Dict]) -> List[Dict]:
        corners = []

        for i in range(1, len(lines)):
            left = lines[i - 1]
            right = lines[i]

            angle = self.acute_angle_between_dirs(left['direction'], right['direction'])
            if angle < self.corner_angle:
                continue

            corner_pos = self.compute_line_intersection(left, right)
            if corner_pos is None:
                corner_pos = 0.5 * (left['points'][-1] + right['points'][0])

            corner_cov = self.compute_corner_covariance(left, right)
            if corner_cov is None:
                continue

            corners.append({
                'position': corner_pos,
                'sharpness': float(np.degrees(angle)),
                'covariance': corner_cov
            })

        return corners

    def compute_wall_covariance(self, line: Dict) -> np.ndarray:
        points = line['points']
        sigma2 = self.lidar_sigma ** 2

        rho, alpha = self.convert_line_to_hessian(points)

        cos_a = np.cos(alpha)
        sin_a = np.sin(alpha)

        A = np.zeros((2, 2))
        for px, py in points:
            dr_d_alpha = px * sin_a - py * cos_a
            J = np.array([[1.0, dr_d_alpha]])
            A += J.T @ J

        min_eig_A = sigma2 * 0.1 
        eigvals, eigvecs = np.linalg.eigh(A)
        eigvals_clamped = np.maximum(eigvals, min_eig_A)
        A_inv = eigvecs @ np.diag(1.0 / eigvals_clamped) @ eigvecs.T

        cov = sigma2 * A_inv

        min_cov_eig = sigma2 * 1e-4
        eigvals_out, eigvecs_out = np.linalg.eigh(cov)
        eigvals_out = np.maximum(eigvals_out, min_cov_eig)
        return eigvecs_out @ np.diag(eigvals_out) @ eigvecs_out.T

    def compute_corner_covariance(self, left: Dict, right: Dict) -> Optional[np.ndarray]:
        rho_a = float(left['rho'])
        alpha_a = float(left['alpha'])
        rho_b = float(right['rho'])
        alpha_b = float(right['alpha'])

        A = np.array([
            [np.cos(alpha_a), np.sin(alpha_a)],
            [np.cos(alpha_b), np.sin(alpha_b)]
        ])
        b = np.array([rho_a, rho_b])

        det = np.linalg.det(A)
        if abs(det) < 0.1:
            return None

        try:
            A_inv = np.linalg.inv(A)
            x = A_inv @ b
        except np.linalg.LinAlgError:
            return None

        dA_dalpha1 = np.array([
            [-np.sin(alpha_a), np.cos(alpha_a)],
            [0.0, 0.0]
        ])
        dA_dalpha2 = np.array([
            [0.0, 0.0],
            [-np.sin(alpha_b), np.cos(alpha_b)]
        ])

        dx_drho1   = A_inv @ np.array([1.0, 0.0])
        dx_drho2   = A_inv @ np.array([0.0, 1.0])
        dx_dalpha1 = -A_inv @ (dA_dalpha1 @ x)
        dx_dalpha2 = -A_inv @ (dA_dalpha2 @ x)

        J = np.column_stack([dx_drho1, dx_dalpha1, dx_drho2, dx_dalpha2])

        cov1 = left.get('covariance', self.compute_wall_covariance(left))
        cov2 = right.get('covariance', self.compute_wall_covariance(right))

        Sigma_theta = np.zeros((4, 4))
        Sigma_theta[:2, :2] = cov1
        Sigma_theta[2:, 2:] = cov2

        cov_corner = J @ Sigma_theta @ J.T

        cov_corner = 0.5 * (cov_corner + cov_corner.T)  
        eigvals, eigvecs = np.linalg.eigh(cov_corner)
        min_cov_eig = (self.lidar_sigma ** 2) * 1e-4
        eigvals = np.maximum(eigvals, min_cov_eig)
        return eigvecs @ np.diag(eigvals) @ eigvecs.T

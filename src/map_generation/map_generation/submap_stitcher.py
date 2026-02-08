#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import open3d.core as o3c
from typing import Optional, Tuple, Dict
from scipy.spatial import KDTree
import time


class SubmapStitcher:
    
    def __init__(self,
                 voxel_size: float = 0.05,
                 icp_max_correspondence_dist: float = 0.05,
                 icp_fitness_threshold: float = 0.45,
                 lidar_noise_sigma: float = 0.01):

        self.voxel_size = voxel_size
        self.icp_max_dist = icp_max_correspondence_dist
        self.icp_fitness_threshold = icp_fitness_threshold
        # LiDAR noise from TurtleBot3 LDS-01 specs (±10mm accuracy)
        self.lidar_noise_sigma = lidar_noise_sigma  # σ = 0.01m → σ² = 0.0001
        # GPU configuration (always enabled)
        if o3c.cuda.is_available():
            self.device = o3c.Device("CUDA:0")
        else:
            self.device = o3c.Device("CPU:0")

        # Submap storage
        self.submaps = []
        self.global_map_tensor = o3d.t.geometry.PointCloud(self.device)

        # Cache for numpy conversion (optimization)
        self._cached_numpy_map = None
        self._map_dirty = True

    def process_submap(self, points: np.ndarray, submap_id: int) -> o3d.t.geometry.PointCloud:

        points_tensor = o3c.Tensor(points.astype(np.float32), dtype=o3c.float32, device=self.device)
        pcd_tensor = o3d.t.geometry.PointCloud(self.device)
        pcd_tensor.point.positions = points_tensor
        pcd_tensor = pcd_tensor.voxel_down_sample(voxel_size=self.voxel_size)

        return pcd_tensor

    def align_submap_with_icp(self,
                       source: o3d.t.geometry.PointCloud,
                       target: o3d.t.geometry.PointCloud,
                       initial_guess: Optional[np.ndarray] = None) -> Tuple[bool, np.ndarray, float, Optional[np.ndarray]]:

        if initial_guess is None:
            initial_guess = np.eye(4)

        # source and target are already GPU tensors - no conversion needed!
        source_tensor = source
        target_tensor = target

        # Point-to-Point ICP
        init_transform = o3c.Tensor(initial_guess, dtype=o3c.float32, device=self.device)

        reg_result = o3d.t.pipelines.registration.icp(
            source=source_tensor,
            target=target_tensor,
            max_correspondence_distance=self.icp_max_dist,
            init_source_to_target=init_transform,
            estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=50,
                relative_fitness=1e-6,
                relative_rmse=1e-6
            )
        )

        transform = reg_result.transformation.cpu().numpy()
        transform[2, 0:3] = [0, 0, 1]  # No Z translation, preserve rotation
        transform[2, 3] = 0             # No Z offset
        transform[0:2, 2] = [0, 0]      # No X,Y coupling with Z axis

        success = reg_result.fitness >= self.icp_fitness_threshold

        # Compute Hessian-based covariance (P = σ² * A^(-1))
        covariance = self._compute_icp_covariance(
            source_tensor,
            target_tensor,
            transform
        )

        return success, transform, reg_result.fitness, covariance

    def _compute_icp_covariance(self,
                                source: o3d.t.geometry.PointCloud,
                                target: o3d.t.geometry.PointCloud,
                                transform: np.ndarray) -> Optional[np.ndarray]:
        """
        Compute Hessian-based covariance for ICP alignment.

        Returns 3x3 covariance matrix for (x, y, theta) based on:
        P = σ² * A^(-1)

        where A is the Hessian (information matrix) from point correspondences.
        """
        try:
            # Convert tensors to numpy arrays for KDTree
            source_points = source.point.positions.cpu().numpy()[:, :2]  # 2D (x, y)
            target_points = target.point.positions.cpu().numpy()[:, :2]

            if len(source_points) < 6 or len(target_points) < 6:
                return None

            # Extract 2D pose from transform
            dx = transform[0, 3]
            dy = transform[1, 3]
            dtheta = np.arctan2(transform[1, 0], transform[0, 0])

            # Rotation matrix and derivative
            c = np.cos(dtheta)
            s = np.sin(dtheta)
            R = np.array([[c, -s], [s, c]])
            dR_dtheta = np.array([[-s, -c], [c, -s]])

            # Find correspondences using KDTree
            tree = KDTree(target_points)
            transformed_source = (R @ source_points.T).T + np.array([dx, dy])
            distances, indices = tree.query(transformed_source)

            # Filter inliers
            max_dist = self.icp_max_dist
            inlier_mask = distances < max_dist

            if np.count_nonzero(inlier_mask) < 6:
                return None

            src_inliers = source_points[inlier_mask]
            tgt_inliers = target_points[indices[inlier_mask]]

            # Compute Hessian A = sum(J^T @ J)
            A = np.zeros((3, 3))

            for p_s, p_t in zip(src_inliers, tgt_inliers):
                # Jacobian of predicted point wrt pose (x, y, theta)
                dp_dtheta = dR_dtheta @ p_s

                # Jacobian: residual = target - predicted
                # ∂r/∂x = -1, ∂r/∂y = -1, ∂r/∂θ = -dp_dtheta
                J = np.array([
                    [-1.0, 0.0, -dp_dtheta[0]],
                    [0.0, -1.0, -dp_dtheta[1]]
                ])

                # Accumulate Hessian (Information Matrix)
                A += J.T @ J

            # Use fixed LiDAR noise instead of estimating from residuals
            # This avoids the "optimism" problem where more points → smaller σ²
            sigma2 = self.lidar_noise_sigma ** 2

            # Covariance: P = σ² * A^(-1)
            try:
                A_inv = np.linalg.inv(A)
            except np.linalg.LinAlgError:
                # Use pseudo-inverse if singular (e.g., in featureless corridors)
                A_inv = np.linalg.pinv(A)

            covariance = sigma2 * A_inv

            return covariance

        except Exception as e:
            # Return None if covariance computation fails
            return None

    def integrate_submap_to_global_map(self,
                                       points: np.ndarray,
                                       submap_id: int,
                                       start_pose: dict,
                                       end_pose: dict,
                                       scan_count: int,
                                       transformation_matrix: np.ndarray) -> Tuple[bool, Optional[dict]]:

        pcd_tensor = self.process_submap(points, submap_id)

        if len(pcd_tensor.point.positions) < 50:
            return False, None

        pose_center = {
            'x': (start_pose['x'] + end_pose['x']) / 2,
            'y': (start_pose['y'] + end_pose['y']) / 2,
            'z': 0.0,
            'qx': end_pose['qx'],
            'qy': end_pose['qy'],
            'qz': end_pose['qz'],
            'qw': end_pose['qw']
        }

        global_transform = transformation_matrix

        if submap_id == 0:
            
            transform_tensor = o3c.Tensor(global_transform, dtype=o3c.float32, device=self.device)
            pcd_world_tensor = pcd_tensor.transform(transform_tensor)
            self.global_map_tensor = pcd_world_tensor

            submap_data = {
                'id': submap_id,
                'point_cloud': pcd_tensor,
                'pose_start': start_pose,
                'pose_end': end_pose,
                'pose_center': pose_center,
                'global_transform': global_transform,
                'scan_count': scan_count,
                'timestamp_created': time.time()
            }

            self.submaps.append(submap_data)

            return True, None  

        success, global_transform_refined, fitness, covariance = self.align_submap_with_icp(
            source=pcd_tensor,
            target=self.global_map_tensor,
            initial_guess=global_transform
        )

        icp_correction = np.linalg.inv(global_transform) @ global_transform_refined
        correction_translation = np.linalg.norm(icp_correction[:2, 3])
        correction_rotation = np.abs(np.arctan2(icp_correction[1, 0], icp_correction[0, 0]))

        if correction_translation > 1.0 or correction_rotation > np.radians(20):
            global_transform_refined = global_transform
            success = False

        if not success or fitness < self.icp_fitness_threshold:
            global_transform_refined = global_transform

        pose_correction = None
        if success and correction_translation > 0.01:
            pose_correction = {
                'dx': icp_correction[0, 3],
                'dy': icp_correction[1, 3],
                'dtheta': np.arctan2(icp_correction[1, 0], icp_correction[0, 0]),
                'type': 'submap_icp',
                'covariance': covariance  # Hessian-based measurement uncertainty
            }
        transform_refined_tensor = o3c.Tensor(global_transform_refined, dtype=o3c.float32, device=self.device)
        pcd_aligned_tensor = pcd_tensor.transform(transform_refined_tensor)


        submap_data = {
            'id': submap_id,
            'point_cloud': pcd_tensor,  # In submap-local frame
            'pose_start': start_pose,
            'pose_end': end_pose,
            'pose_center': pose_center,
            'global_transform': global_transform_refined,
            'scan_count': scan_count,
            'timestamp_created': time.time()
        }

        self.submaps.append(submap_data)

        self.global_map_tensor.point.positions = o3c.concatenate([
            self.global_map_tensor.point.positions,
            pcd_aligned_tensor.point.positions
        ], axis=0)

        self.global_map_tensor = self.global_map_tensor.voxel_down_sample(self.voxel_size)

        # Invalidate cache after modifying global map
        self._map_dirty = True

        return True, pose_correction

    def save_global_map(self, filepath: str):
        """Save the global map using Tensor I/O (no Legacy conversion)"""
        if len(self.global_map_tensor.point.positions) == 0:
            return

        # Use Tensor I/O API (no conversion needed)
        o3d.t.io.write_point_cloud(filepath, self.global_map_tensor, write_ascii=False, compressed=False)

    def get_global_map_points(self) -> Optional[np.ndarray]:
        
        if 'positions' not in self.global_map_tensor.point or len(self.global_map_tensor.point.positions) == 0:
            return None

        if self._map_dirty or self._cached_numpy_map is None:
            self._cached_numpy_map = self.global_map_tensor.point.positions.cpu().numpy()
            self._map_dirty = False

        return self._cached_numpy_map

    def _rebuild_global_map(self):
        """Rebuild global map after global transform updates."""
        # Clear current global map
        self.global_map_tensor = o3d.t.geometry.PointCloud(self.device)

        # Re-stitch all submaps with current transforms
        for submap in self.submaps:

            pcd_tensor = submap['point_cloud']
            transform_tensor = o3c.Tensor(submap['global_transform'], dtype=o3c.float32, device=self.device)
            pcd_transformed = pcd_tensor.transform(transform_tensor)

            if len(self.global_map_tensor.point.positions) == 0:
                self.global_map_tensor = pcd_transformed
            else:
                self.global_map_tensor.point.positions = o3c.concatenate([
                    self.global_map_tensor.point.positions,
                    pcd_transformed.point.positions
                ], axis=0)

        self.global_map_tensor = self.global_map_tensor.voxel_down_sample(self.voxel_size)

        self._map_dirty = True

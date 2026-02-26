#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import open3d.core as o3c
from typing import Optional, Tuple, Dict, List
import time


class SubmapStitcher:

    def __init__(self,
                 voxel_size: float = 0.05,
                 lidar_noise_sigma: float = 0.01):

        self.voxel_size = voxel_size
        self.lidar_noise_sigma = lidar_noise_sigma

        if o3c.cuda.is_available():
            self.device = o3c.Device("CUDA:0")
        else:
            self.device = o3c.Device("CPU:0")

        # Submap storage
        self.submaps = []
        self.global_map_tensor = o3d.t.geometry.PointCloud(self.device)

        # Global wall registry keyed by landmark_id — used for feature-based alignment
        # Each entry: {rho, alpha, start_point, end_point} in globally-aligned map frame
        self.global_walls: Dict[int, Dict] = {}

        # Cache for numpy conversion
        self._cached_numpy_map = None
        self._map_dirty = True

    def process_submap(self, points: np.ndarray) -> o3d.t.geometry.PointCloud:
        points_tensor = o3c.Tensor(points.astype(np.float32), dtype=o3c.float32, device=self.device)
        pcd_tensor = o3d.t.geometry.PointCloud(self.device)
        pcd_tensor.point.positions = points_tensor
        pcd_tensor = pcd_tensor.voxel_down_sample(voxel_size=self.voxel_size)
        return pcd_tensor

    # ------------------------------------------------------------------
    # Point Cloud ICP Alignment
    # ------------------------------------------------------------------

    def point_cloud_icp_align(self,
                              source_tensor: o3d.t.geometry.PointCloud,
                              target_tensor: o3d.t.geometry.PointCloud,
                              initial_guess: np.ndarray = None) -> Tuple[bool, Optional[Dict]]:
        """Align submap to global map using hybrid tensor ICP + Censi covariance.

        Hybrid approach:
        1. Runs ICP on GPU tensors for speed (Open3D's optimized implementation)
        2. Converts to legacy format to extract correspondences
        3. Computes Censi-style covariance from correspondence geometry

        Args:
            source_tensor: Source point cloud (submap) as Open3D tensor
            target_tensor: Target point cloud (global map) as Open3D tensor
            initial_guess: Initial 4x4 transformation (default: identity)

        Returns:
            (success, pose_correction_dict with Censi covariance or None)
        """
        if len(source_tensor.point.positions) < 20 or len(target_tensor.point.positions) < 20:
            print("[SubmapStitcher] ICP failed: insufficient points")
            return False, None

        if initial_guess is None:
            initial_guess = np.eye(4)

        # Convert initial guess to tensor
        init_transform = o3c.Tensor(initial_guess, dtype=o3c.float32, device=self.device)

        # Run ICP using Open3D's optimized implementation
        reg_result = o3d.t.pipelines.registration.icp(
            source=source_tensor,
            target=target_tensor,
            max_correspondence_distance=0.03,  # 3σ odometry drift
            init_source_to_target=init_transform,
            estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=50,
                relative_fitness=1e-6,
                relative_rmse=1e-6
            )
        )

        # Check convergence quality
        if reg_result.fitness < 0.3:
            print(f"[SubmapStitcher] ICP failed: low fitness ({reg_result.fitness:.3f} < 0.3)")
            return False, None

        if reg_result.inlier_rmse > 0.05:
            print(f"[SubmapStitcher] ICP failed: high RMSE ({reg_result.inlier_rmse:.4f} > 0.05)")
            return False, None

        # Extract transformation matrix
        T = reg_result.transformation.cpu().numpy()
        dx = float(T[0, 3])
        dy = float(T[1, 3])
        dtheta = float(np.arctan2(T[1, 0], T[0, 0]))

        # Convert tensors to legacy for correspondence extraction
        source_legacy = source_tensor.to_legacy()
        target_legacy = target_tensor.to_legacy()

        # Apply ICP transformation to source points
        source_transformed = source_legacy.transform(T)

        # Find nearest neighbor correspondences using final transformation
        kdtree = o3d.geometry.KDTreeFlann(target_legacy)
        correspondences = []
        max_corr_dist_sq = 0.03 * 0.03  # 0.03m threshold squared

        for i in range(len(source_transformed.points)):
            [k, idx, dist] = kdtree.search_knn_vector_3d(source_transformed.points[i], 1)
            if dist[0] < max_corr_dist_sq:
                correspondences.append((i, idx[0]))

        if len(correspondences) < 20:
            print(f"[SubmapStitcher] ICP failed: only {len(correspondences)} correspondences after NN search")
            return False, None

        # Compute Censi-style covariance from correspondence geometry
        c, s = np.cos(dtheta), np.sin(dtheta)
        H = np.zeros((3, 3))  # Hessian for [tx, ty, theta]

        for src_idx, tgt_idx in correspondences:
            # Get source point in 2D (before transformation in local frame)
            p_src = np.array(source_legacy.points[src_idx][:2])

            # Jacobian of transformation wrt pose [tx, ty, theta]
            # ∂(R*p + t)/∂tx = [1, 0]
            # ∂(R*p + t)/∂ty = [0, 1]
            # ∂(R*p + t)/∂θ  = [-sin(θ)*px - cos(θ)*py, cos(θ)*px - sin(θ)*py]
            J = np.array([
                [1.0, 0.0, -s * p_src[0] - c * p_src[1]],
                [0.0, 1.0,  c * p_src[0] - s * p_src[1]]
            ])
            H += J.T @ J

        # Stabilize Hessian via eigenvalue clamping
        eigvals, eigvecs = np.linalg.eigh(H)
        eigvals = np.maximum(eigvals, 1e-6)  # Prevent singularity
        H_stable = eigvecs @ np.diag(eigvals) @ eigvecs.T

        # Covariance = σ² * H⁻¹ (Censi method)
        try:
            covariance = (self.lidar_noise_sigma ** 2) * np.linalg.inv(H_stable)
        except np.linalg.LinAlgError:
            covariance = (self.lidar_noise_sigma ** 2) * np.linalg.pinv(H_stable)

        pose_correction = {
            'dx': dx,
            'dy': dy,
            'dtheta': dtheta,
            'type': 'point_cloud_icp',
            'covariance': covariance
        }

        print(
            f"[SubmapStitcher] ICP align | "
            f"fitness={reg_result.fitness:.3f}  "
            f"rmse={reg_result.inlier_rmse:.4f}m  "
            f"correspondences={len(correspondences)}  "
            f"dx={dx:.4f}m  dy={dy:.4f}m  dtheta={np.degrees(dtheta):.3f}deg  "
            f"cov_xx={covariance[0,0]:.2e}  cov_yy={covariance[1,1]:.2e}  cov_th={covariance[2,2]:.2e}"
        )

        return True, pose_correction

    def _accumulate_walls(self, feature_map, R: np.ndarray, t: np.ndarray):
        """Apply correction (R, t) to current submap walls and upsert into global_walls.

        Wall extents in feature_map are stored as (t_min, t_max) scalars; they are
        reconstructed as 2D endpoints, corrected, then stored back as t_min/t_max
        in the global wall's own tangent coordinate system.

        For existing landmark IDs: extend the tangential extent.
        For new IDs: insert directly.
        """
        dtheta = float(np.arctan2(R[1, 0], R[0, 0]))

        for lm_id, wall in feature_map.walls.items():
            if wall.get('t_min') is None or wall.get('t_max') is None:
                continue

            # Reconstruct 2D endpoints from stored scalar extents
            alpha_src   = float(wall['alpha'])
            rho_src     = float(wall['rho'])
            tangent_src = np.array([-np.sin(alpha_src), np.cos(alpha_src)])
            normal_src  = np.array([ np.cos(alpha_src), np.sin(alpha_src)])
            line_pt_src = rho_src * normal_src
            start_raw   = line_pt_src + float(wall['t_min']) * tangent_src
            end_raw     = line_pt_src + float(wall['t_max']) * tangent_src

            # Apply SVD correction
            start_corr = R @ start_raw + t
            end_corr   = R @ end_raw   + t

            # Corrected Hessian params
            alpha_corr = float(np.arctan2(
                np.sin(alpha_src + dtheta),
                np.cos(alpha_src + dtheta)
            ))
            normal_corr  = np.array([ np.cos(alpha_corr), np.sin(alpha_corr)])
            tangent_corr = np.array([-np.sin(alpha_corr), np.cos(alpha_corr)])
            centroid_corr = 0.5 * (start_corr + end_corr)
            rho_corr = float(np.dot(centroid_corr, normal_corr))
            if rho_corr < 0.0:
                rho_corr    = -rho_corr
                alpha_corr  = float(np.arctan2(
                    np.sin(alpha_corr + np.pi),
                    np.cos(alpha_corr + np.pi)
                ))
                normal_corr  = -normal_corr
                tangent_corr = -tangent_corr

            # Project corrected endpoints onto corrected wall tangent → store as scalars
            t_s = float(np.dot(start_corr, tangent_corr))
            t_e = float(np.dot(end_corr,   tangent_corr))
            t_min_corr = min(t_s, t_e)
            t_max_corr = max(t_s, t_e)

            if lm_id not in self.global_walls:
                self.global_walls[lm_id] = {
                    'rho':   rho_corr,
                    'alpha': alpha_corr,
                    't_min': t_min_corr,
                    't_max': t_max_corr
                }
            else:
                # Extend existing extent along the stored global wall's tangent
                existing  = self.global_walls[lm_id]
                alpha_e   = float(existing['alpha'])
                tangent_e = np.array([-np.sin(alpha_e), np.cos(alpha_e)])
                proj_s = float(np.dot(start_corr, tangent_e))
                proj_e = float(np.dot(end_corr,   tangent_e))
                existing['t_min'] = min(existing['t_min'], proj_s, proj_e)
                existing['t_max'] = max(existing['t_max'], proj_s, proj_e)

    # ------------------------------------------------------------------
    # Global map management
    # ------------------------------------------------------------------

    def integrate_submap_to_global_map(self,
                                        points: np.ndarray,
                                        submap_id: int,
                                        start_pose: dict,
                                        end_pose: dict,
                                        scan_count: int,
                                        feature_map=None) -> Tuple[bool, Optional[dict]]:

        pcd_tensor = self.process_submap(points)

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

        identity_R = np.eye(2)
        identity_t = np.zeros(2)

        if submap_id == 0:
            # Points already in map frame — place directly into global map
            self.global_map_tensor = pcd_tensor

            submap_data = {
                'id': submap_id,
                'point_cloud': pcd_tensor,
                'pose_start': start_pose,
                'pose_end': end_pose,
                'pose_center': pose_center,
                'scan_count': scan_count,
                'timestamp_created': time.time()
            }
            self.submaps.append(submap_data)

            # Seed the global wall registry with first submap's walls (no correction)
            if feature_map is not None:
                self._accumulate_walls(feature_map, identity_R, identity_t)

            return True, None

        # ------ Submap > 0: attempt point cloud ICP alignment ------

        pose_correction = None
        pcd_aligned_tensor = pcd_tensor   # default: points already in map frame

        if len(self.global_map_tensor.point.positions) > 0:
            # Attempt ICP alignment using tensors directly (no numpy conversion needed)
            success, pose_correction = self.point_cloud_icp_align(pcd_tensor, self.global_map_tensor)

            if success:
                # Apply ICP correction to map-frame points
                dx      = pose_correction['dx']
                dy      = pose_correction['dy']
                dtheta  = pose_correction['dtheta']
                c, s    = np.cos(dtheta), np.sin(dtheta)

                correction_4x4 = np.eye(4)
                correction_4x4[0, 0] = c;  correction_4x4[0, 1] = -s;  correction_4x4[0, 3] = dx
                correction_4x4[1, 0] = s;  correction_4x4[1, 1] =  c;  correction_4x4[1, 3] = dy

                transform_tensor = o3c.Tensor(correction_4x4, dtype=o3c.float32, device=self.device)
                pcd_aligned_tensor = pcd_tensor.transform(transform_tensor)

                # Update landmark endpoints with ICP correction
                R_2x2 = np.array([[c, -s], [s, c]])
                t_2d  = np.array([dx, dy])
                if feature_map is not None:
                    self._accumulate_walls(feature_map, R_2x2, t_2d)
            else:
                # ICP failed: add points with EKF pose as-is
                # Confidence tracker will reflect degraded accuracy via EKF covariance
                if feature_map is not None:
                    self._accumulate_walls(feature_map, identity_R, identity_t)
        else:
            # No global map yet (shouldn't happen for submap_id > 0)
            if feature_map is not None:
                self._accumulate_walls(feature_map, identity_R, identity_t)

        submap_data = {
            'id': submap_id,
            'point_cloud': pcd_tensor,
            'pose_start': start_pose,
            'pose_end': end_pose,
            'pose_center': pose_center,
            'scan_count': scan_count,
            'timestamp_created': time.time()
        }
        self.submaps.append(submap_data)

        self.global_map_tensor.point.positions = o3c.concatenate([
            self.global_map_tensor.point.positions,
            pcd_aligned_tensor.point.positions
        ], axis=0)

        self.global_map_tensor = self.global_map_tensor.voxel_down_sample(self.voxel_size)

        self._map_dirty = True

        return True, pose_correction

    def save_global_map(self, filepath: str):
        """Save the global map using Tensor I/O."""
        if len(self.global_map_tensor.point.positions) == 0:
            return
        o3d.t.io.write_point_cloud(filepath, self.global_map_tensor, write_ascii=False, compressed=False)

    def get_global_map_points(self) -> Optional[np.ndarray]:
        if 'positions' not in self.global_map_tensor.point or len(self.global_map_tensor.point.positions) == 0:
            return None

        if self._map_dirty or self._cached_numpy_map is None:
            self._cached_numpy_map = self.global_map_tensor.point.positions.cpu().numpy()
            self._map_dirty = False

        return self._cached_numpy_map

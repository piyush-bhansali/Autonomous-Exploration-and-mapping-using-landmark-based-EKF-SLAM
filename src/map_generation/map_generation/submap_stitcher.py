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

    def process_submap(self, points: np.ndarray, submap_id: int) -> o3d.t.geometry.PointCloud:
        points_tensor = o3c.Tensor(points.astype(np.float32), dtype=o3c.float32, device=self.device)
        pcd_tensor = o3d.t.geometry.PointCloud(self.device)
        pcd_tensor.point.positions = points_tensor
        pcd_tensor = pcd_tensor.voxel_down_sample(voxel_size=self.voxel_size)
        return pcd_tensor

    # ------------------------------------------------------------------
    # Feature-based alignment
    # ------------------------------------------------------------------

    def _compute_overlap_correspondences(self,
                                          src_wall: Dict,
                                          tgt_wall: Dict,
                                          n_samples: int = 8) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Compute point-pair correspondences from the overlapping section of two matched walls.

        Both walls are expressed in (approximate) map frame. Uses the target wall's
        Hessian direction as canonical so that target points lie exactly on the
        EKF-corrected global line.

        Returns a list of (p_src, p_tgt) pairs, each a 2D np.ndarray.
        """
        alpha_tgt = float(tgt_wall['alpha'])
        alpha_src = float(src_wall['alpha'])

        tangent = np.array([-np.sin(alpha_tgt), np.cos(alpha_tgt)])
        normal_tgt = np.array([np.cos(alpha_tgt), np.sin(alpha_tgt)])
        normal_src = np.array([np.cos(alpha_src), np.sin(alpha_src)])

        s_start = np.asarray(src_wall['start_point'], dtype=float)
        s_end   = np.asarray(src_wall['end_point'],   dtype=float)
        t_start = np.asarray(tgt_wall['start_point'], dtype=float)
        t_end   = np.asarray(tgt_wall['end_point'],   dtype=float)

        # Project all four endpoints onto the tangent axis
        ts0, ts1 = np.dot(s_start, tangent), np.dot(s_end, tangent)
        tt0, tt1 = np.dot(t_start, tangent), np.dot(t_end, tangent)

        t_lo = max(min(ts0, ts1), min(tt0, tt1))
        t_hi = min(max(ts0, ts1), max(tt0, tt1))

        if t_hi - t_lo < 0.3:
            return []   # Insufficient overlap

        rho_tgt = float(tgt_wall['rho'])
        rho_src = float(src_wall['rho'])

        pairs = []
        for t in np.linspace(t_lo, t_hi, n_samples):
            p_tgt = rho_tgt * normal_tgt + t * tangent
            p_src = rho_src * normal_src + t * tangent
            pairs.append((p_src, p_tgt))

        return pairs

    def _svd_align(self, pairs: List[Tuple[np.ndarray, np.ndarray]]) -> Optional[Tuple]:
        """Solve for the 2D rigid body transform (R, t) that maps source to target.

        Uses SVD on the cross-covariance matrix of centred point pairs.
        Returns (R_2x2, t_2d, condition_number) or None if underdetermined.
        """
        if len(pairs) < 3:
            return None

        src_pts = np.array([p[0] for p in pairs])   # (N, 2)
        tgt_pts = np.array([p[1] for p in pairs])   # (N, 2)

        c_src = src_pts.mean(axis=0)
        c_tgt = tgt_pts.mean(axis=0)

        q_src = src_pts - c_src
        q_tgt = tgt_pts - c_tgt

        # 2×2 cross-covariance
        H = q_src.T @ q_tgt   # (2, 2)

        U, S, Vt = np.linalg.svd(H)

        # Reflection guard (det check)
        d = np.linalg.det(Vt.T @ U.T)
        D = np.diag([1.0, np.sign(d)])
        R = Vt.T @ D @ U.T

        t = c_tgt - R @ c_src

        condition_number = float(S[0]) / (float(S[1]) + 1e-9)

        return R, t, condition_number, S

    def feature_align_submaps(self, feature_map) -> Tuple[bool, Optional[Dict]]:
        """Attempt to align a new submap against the accumulated global walls.

        Uses shared landmark IDs as correspondences — no re-matching needed.
        The overlap section of each matched wall pair provides point pairs;
        SVD gives the rigid correction in one shot.

        Returns (success, pose_correction_dict or None).
        """
        if not feature_map or not self.global_walls:
            return False, None

        shared_ids = set(feature_map.walls.keys()) & set(self.global_walls.keys())

        if not shared_ids:
            return False, None

        all_pairs: List[Tuple[np.ndarray, np.ndarray]] = []

        for lm_id in shared_ids:
            src_wall = feature_map.walls[lm_id]
            tgt_wall = self.global_walls[lm_id]

            # Skip if endpoints are None (wall seen but not yet extended)
            if (src_wall.get('start_point') is None or src_wall.get('end_point') is None or
                    tgt_wall.get('start_point') is None or tgt_wall.get('end_point') is None):
                continue

            pairs = self._compute_overlap_correspondences(src_wall, tgt_wall)
            all_pairs.extend(pairs)

        result = self._svd_align(all_pairs)
        if result is None:
            return False, None

        R, t, condition_number, S = result

        # Corridor degeneracy check: all walls parallel → one singular value near zero
        if condition_number > 100.0:
            return False, None

        # Extract 2D pose correction
        dtheta = float(np.arctan2(R[1, 0], R[0, 0]))
        dx = float(t[0])
        dy = float(t[1])

        # Covariance from SVD singular values
        # S[1] (weaker singular value) captures the worst-case translational constraint;
        # mean squared distance of source points from their centroid captures the
        # rotational constraint (further spread → better-conditioned rotation estimate).
        src_pts = np.array([p[0] for p in all_pairs])
        c_src = src_pts.mean(axis=0)
        mean_sq_dist = float(np.mean(np.sum((src_pts - c_src) ** 2, axis=1)))

        sigma2 = self.lidar_noise_sigma ** 2
        cov_xy = sigma2 / (float(S[1]) + 1e-6)
        cov_th = sigma2 / (len(all_pairs) * mean_sq_dist + 1e-6)
        covariance = np.diag([cov_xy, cov_xy, cov_th])

        pose_correction = {
            'dx': dx,
            'dy': dy,
            'dtheta': dtheta,
            'type': 'submap_feature',
            'covariance': covariance
        }

        print(
            f"[SubmapStitcher] SVD align | "
            f"shared_walls={len(shared_ids)}  pairs={len(all_pairs)}  "
            f"cond={condition_number:.1f}  "
            f"dx={dx:.4f}m  dy={dy:.4f}m  dtheta={np.degrees(dtheta):.3f}deg  "
            f"cov_xy={cov_xy:.2e}  cov_th={cov_th:.2e}"
        )

        return True, pose_correction

    def _accumulate_walls(self, feature_map, R: np.ndarray, t: np.ndarray):
        """Apply correction (R, t) to current submap walls and upsert into global_walls.

        For existing landmark IDs: extend the endpoint extent.
        For new IDs: insert directly.
        """
        for lm_id, wall in feature_map.walls.items():
            if wall.get('start_point') is None or wall.get('end_point') is None:
                continue

            # Apply correction to endpoints
            start_corr = R @ np.asarray(wall['start_point'], dtype=float) + t
            end_corr   = R @ np.asarray(wall['end_point'],   dtype=float) + t

            # Corrected Hessian params (alpha shifts by dtheta, rho recomputed)
            dtheta = float(np.arctan2(R[1, 0], R[0, 0]))
            alpha_corr = float(np.arctan2(
                np.sin(wall['alpha'] + dtheta),
                np.cos(wall['alpha'] + dtheta)
            ))
            normal = np.array([np.cos(alpha_corr), np.sin(alpha_corr)])
            # rho from corrected centroid
            centroid_corr = 0.5 * (start_corr + end_corr)
            rho_corr = float(np.dot(centroid_corr, normal))
            if rho_corr < 0.0:
                rho_corr = -rho_corr
                alpha_corr = float(np.arctan2(
                    np.sin(alpha_corr + np.pi),
                    np.cos(alpha_corr + np.pi)
                ))
                normal = -normal

            if lm_id not in self.global_walls:
                self.global_walls[lm_id] = {
                    'rho': rho_corr,
                    'alpha': alpha_corr,
                    'start_point': start_corr,
                    'end_point': end_corr
                }
            else:
                # Extend existing endpoint extent along the stored wall's tangent
                existing = self.global_walls[lm_id]
                alpha_e = float(existing['alpha'])
                tangent_e = np.array([-np.sin(alpha_e), np.cos(alpha_e)])

                projs = [
                    np.dot(existing['start_point'], tangent_e),
                    np.dot(existing['end_point'],   tangent_e),
                    np.dot(start_corr,              tangent_e),
                    np.dot(end_corr,                tangent_e),
                ]
                t_min, t_max = min(projs), max(projs)

                normal_e = np.array([np.cos(alpha_e), np.sin(alpha_e)])
                line_pt = existing['rho'] * normal_e
                tangent_e_vec = tangent_e

                existing['start_point'] = line_pt + t_min * tangent_e_vec
                existing['end_point']   = line_pt + t_max * tangent_e_vec

    # ------------------------------------------------------------------
    # Global map management
    # ------------------------------------------------------------------

    def integrate_submap_to_global_map(self,
                                        points: np.ndarray,
                                        submap_id: int,
                                        start_pose: dict,
                                        end_pose: dict,
                                        scan_count: int,
                                        transformation_matrix: np.ndarray,
                                        feature_map=None) -> Tuple[bool, Optional[dict]]:

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

        identity_R = np.eye(2)
        identity_t = np.zeros(2)

        if submap_id == 0:
            transform_tensor = o3c.Tensor(transformation_matrix, dtype=o3c.float32, device=self.device)
            pcd_world_tensor = pcd_tensor.transform(transform_tensor)
            self.global_map_tensor = pcd_world_tensor

            submap_data = {
                'id': submap_id,
                'point_cloud': pcd_tensor,
                'pose_start': start_pose,
                'pose_end': end_pose,
                'pose_center': pose_center,
                'global_transform': transformation_matrix,
                'scan_count': scan_count,
                'timestamp_created': time.time()
            }
            self.submaps.append(submap_data)

            # Seed the global wall registry with first submap's walls (no correction)
            if feature_map is not None:
                self._accumulate_walls(feature_map, identity_R, identity_t)

            return True, None

        # ------ Submap > 0: attempt feature-based alignment ------

        global_transform = transformation_matrix
        pose_correction = None

        if feature_map is not None and self.global_walls:
            success, pose_correction = self.feature_align_submaps(feature_map)

            if success:
                # Build corrected 4×4 transform from SVD result
                dx      = pose_correction['dx']
                dy      = pose_correction['dy']
                dtheta  = pose_correction['dtheta']
                c, s    = np.cos(dtheta), np.sin(dtheta)

                correction_4x4 = np.eye(4)
                correction_4x4[0, 0] = c;  correction_4x4[0, 1] = -s;  correction_4x4[0, 3] = dx
                correction_4x4[1, 0] = s;  correction_4x4[1, 1] =  c;  correction_4x4[1, 3] = dy

                global_transform = correction_4x4 @ transformation_matrix

                # Accumulate walls with the SVD correction applied
                R_2x2 = np.array([[c, -s], [s, c]])
                t_2d  = np.array([dx, dy])
                self._accumulate_walls(feature_map, R_2x2, t_2d)
            else:
                # Alignment failed (degenerate geometry): add points with EKF pose as-is.
                # Confidence tracker will reflect degraded accuracy via EKF covariance.
                self._accumulate_walls(feature_map, identity_R, identity_t)
        else:
            if feature_map is not None:
                self._accumulate_walls(feature_map, identity_R, identity_t)

        transform_tensor = o3c.Tensor(global_transform, dtype=o3c.float32, device=self.device)
        pcd_aligned_tensor = pcd_tensor.transform(transform_tensor)

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

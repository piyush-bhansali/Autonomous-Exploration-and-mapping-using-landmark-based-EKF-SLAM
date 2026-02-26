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

        self.submaps = []
        self.global_map_tensor = o3d.t.geometry.PointCloud(self.device)

        self.global_walls: Dict[int, Dict] = {}

        self._cached_numpy_map = None
        self._map_dirty = True

    def process_submap(self, points: np.ndarray) -> o3d.t.geometry.PointCloud:
        points_tensor = o3c.Tensor(points.astype(np.float32), dtype=o3c.float32, device=self.device)
        pcd_tensor = o3d.t.geometry.PointCloud(self.device)
        pcd_tensor.point.positions = points_tensor
        pcd_tensor = pcd_tensor.voxel_down_sample(voxel_size=self.voxel_size)
        return pcd_tensor


    def point_cloud_icp_align(self,
                              source_tensor: o3d.t.geometry.PointCloud,
                              target_tensor: o3d.t.geometry.PointCloud,
                              initial_guess: np.ndarray = None) -> Tuple[bool, Optional[Dict]]:
        
        if len(source_tensor.point.positions) < 20 or len(target_tensor.point.positions) < 20:
            print("[SubmapStitcher] ICP failed: insufficient points")
            return False, None

        if initial_guess is None:
            initial_guess = np.eye(4)

        init_transform = o3c.Tensor(initial_guess, dtype=o3c.float32, device=self.device)

        reg_result = o3d.t.pipelines.registration.icp(
            source=source_tensor,
            target=target_tensor,
            max_correspondence_distance=0.05, 
            init_source_to_target=init_transform,
            estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=50,
                relative_fitness=1e-6,
                relative_rmse=1e-6
            )
        )

        if reg_result.fitness < 0.5:
            print(f"[SubmapStitcher] ICP failed: low fitness ({reg_result.fitness:.3f} < 0.5)")
            return False, None

        if reg_result.inlier_rmse > 0.05:
            print(f"[SubmapStitcher] ICP failed: high RMSE ({reg_result.inlier_rmse:.4f} > 0.05)")
            return False, None

        T = reg_result.transformation.cpu().numpy()
        dx = float(T[0, 3])
        dy = float(T[1, 3])
        dtheta = float(np.arctan2(T[1, 0], T[0, 0]))

        source_legacy = source_tensor.to_legacy()
        target_legacy = target_tensor.to_legacy()

        source_transformed = source_legacy.transform(T)

        kdtree = o3d.geometry.KDTreeFlann(target_legacy)
        correspondences = []
        max_corr_dist_sq = 0.03 * 0.03  

        for i in range(len(source_transformed.points)):
            [k, idx, dist] = kdtree.search_knn_vector_3d(source_transformed.points[i], 1)
            if dist[0] < max_corr_dist_sq:
                correspondences.append((i, idx[0]))

        if len(correspondences) < 20:
            print(f"[SubmapStitcher] ICP failed: only {len(correspondences)} correspondences after NN search")
            return False, None

        c, s = np.cos(dtheta), np.sin(dtheta)
        H = np.zeros((3, 3))  

        for src_idx, tgt_idx in correspondences:
            p_src = np.array(source_legacy.points[src_idx][:2])

            J = np.array([
                [1.0, 0.0, -s * p_src[0] - c * p_src[1]],
                [0.0, 1.0,  c * p_src[0] - s * p_src[1]]
            ])
            H += J.T @ J

        eigvals, eigvecs = np.linalg.eigh(H)
        eigvals = np.maximum(eigvals, 1e-6)  
        H_stable = eigvecs @ np.diag(eigvals) @ eigvecs.T

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

        return True, pose_correction

    def _accumulate_walls(self, feature_map, R: np.ndarray, t: np.ndarray):
        
        dtheta = float(np.arctan2(R[1, 0], R[0, 0]))

        for lm_id, wall in feature_map.walls.items():
            if wall.get('t_min') is None or wall.get('t_max') is None:
                continue

            alpha_src   = float(wall['alpha'])
            rho_src     = float(wall['rho'])
            tangent_src = np.array([-np.sin(alpha_src), np.cos(alpha_src)])
            normal_src  = np.array([ np.cos(alpha_src), np.sin(alpha_src)])
            line_pt_src = rho_src * normal_src
            start_raw   = line_pt_src + float(wall['t_min']) * tangent_src
            end_raw     = line_pt_src + float(wall['t_max']) * tangent_src

            start_corr = R @ start_raw + t
            end_corr   = R @ end_raw   + t

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
                existing  = self.global_walls[lm_id]
                alpha_e   = float(existing['alpha'])
                tangent_e = np.array([-np.sin(alpha_e), np.cos(alpha_e)])
                proj_s = float(np.dot(start_corr, tangent_e))
                proj_e = float(np.dot(end_corr,   tangent_e))
                existing['t_min'] = min(existing['t_min'], proj_s, proj_e)
                existing['t_max'] = max(existing['t_max'], proj_s, proj_e)


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

            if feature_map is not None:
                self._accumulate_walls(feature_map, identity_R, identity_t)

            return True, None

        pose_correction = None
        pcd_aligned_tensor = pcd_tensor   # default: points already in map frame

        if len(self.global_map_tensor.point.positions) > 0:
            
            success, pose_correction = self.point_cloud_icp_align(pcd_tensor, self.global_map_tensor)

            if success:
               
                dx      = pose_correction['dx']
                dy      = pose_correction['dy']
                dtheta  = pose_correction['dtheta']
                c, s    = np.cos(dtheta), np.sin(dtheta)

                correction_4x4 = np.eye(4)
                correction_4x4[0, 0] = c;  correction_4x4[0, 1] = -s;  correction_4x4[0, 3] = dx
                correction_4x4[1, 0] = s;  correction_4x4[1, 1] =  c;  correction_4x4[1, 3] = dy

                transform_tensor = o3c.Tensor(correction_4x4, dtype=o3c.float32, device=self.device)
                pcd_aligned_tensor = pcd_tensor.transform(transform_tensor)

                R_2x2 = np.array([[c, -s], [s, c]])
                t_2d  = np.array([dx, dy])
                if feature_map is not None:
                    self._accumulate_walls(feature_map, R_2x2, t_2d)
            else:
                
                if feature_map is not None:
                    self._accumulate_walls(feature_map, identity_R, identity_t)
        else:
            
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

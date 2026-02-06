#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import open3d.core as o3c
from typing import Optional, Tuple, Dict
import time
from map_generation.feature_extractor import FeatureExtractor


class SubmapStitcher:
    
    def __init__(self,
                 voxel_size: float = 0.05,
                 icp_max_correspondence_dist: float = 0.05,
                 icp_fitness_threshold: float = 0.45,
                 feature_extraction_method: str = 'hybrid'):

        self.voxel_size = voxel_size
        self.icp_max_dist = icp_max_correspondence_dist
        self.icp_fitness_threshold = icp_fitness_threshold
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

        # Feature extraction (downsampling done in process_submap)
        self.feature_extractor = FeatureExtractor(
            method=feature_extraction_method
        )

    def process_submap(self, points: np.ndarray, submap_id: int) -> o3d.t.geometry.PointCloud:

        points_tensor = o3c.Tensor(points.astype(np.float32), dtype=o3c.float32, device=self.device)
        pcd_tensor = o3d.t.geometry.PointCloud(self.device)
        pcd_tensor.point.positions = points_tensor
        pcd_tensor = pcd_tensor.voxel_down_sample(voxel_size=self.voxel_size)

        return pcd_tensor

    def extract_features(self, pcd: o3d.t.geometry.PointCloud, submap_id: int) -> Optional[Dict]:
       
        try:
            features = self.feature_extractor.extract(pcd)

            if features['method'] == 'hybrid':
                num_keypoints = features['metadata']['geometric']['num_keypoints']
            else:
                num_keypoints = features['metadata']['num_keypoints']

            if num_keypoints == 0:
                return None

            return features

        except Exception as e:
            return None

    def align_submap_with_icp(self,
                       source: o3d.t.geometry.PointCloud,
                       target: o3d.t.geometry.PointCloud,
                       initial_guess: Optional[np.ndarray] = None) -> Tuple[bool, np.ndarray, float]:
        
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

        return success, transform, reg_result.fitness

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

        features = self.extract_features(pcd_tensor, submap_id)

        global_transform = transformation_matrix

        if submap_id == 0:
            
            transform_tensor = o3c.Tensor(global_transform, dtype=o3c.float32, device=self.device)
            pcd_world_tensor = pcd_tensor.transform(transform_tensor)
            self.global_map_tensor = pcd_world_tensor

            submap_data = {
                'id': submap_id,
                'point_cloud': pcd_tensor,  
                'features': features,
                'pose_start': start_pose,
                'pose_end': end_pose,
                'pose_center': pose_center,
                'global_transform': global_transform,  
                'scan_count': scan_count,
                'timestamp_created': time.time()
            }

            self.submaps.append(submap_data)

            return True, None  

        success, global_transform_refined, fitness = self.align_submap_with_icp(
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
                'type': 'submap_icp'
            }
        transform_refined_tensor = o3c.Tensor(global_transform_refined, dtype=o3c.float32, device=self.device)
        pcd_aligned_tensor = pcd_tensor.transform(transform_refined_tensor)


        submap_data = {
            'id': submap_id,
            'point_cloud': pcd_tensor,  # In submap-local frame
            'features': features,
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

#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import open3d.core as o3c
from typing import Optional, Tuple, Dict, List
import time
import os
from map_generation.feature_extractor import FeatureExtractor
from map_generation.loop_closure_detector import LoopClosureDetector


class SubmapStitcher:
    
    def __init__(self,
                 voxel_size: float = 0.05,
                 icp_max_correspondence_dist: float = 0.1,
                 icp_fitness_threshold: float = 0.45,
                 feature_extraction_method: str = 'hybrid',
                 enable_loop_closure: bool = False):

        self.voxel_size = voxel_size
        self.icp_max_dist = icp_max_correspondence_dist
        self.icp_fitness_threshold = icp_fitness_threshold

        # GPU configuration (always enabled)
        if o3c.cuda.is_available():
            self.device = o3c.Device("CUDA:0")
        else:
            self.device = o3c.Device("CPU:0")

        # Submap storage (for loop closure)
        self.submaps = []
        self.global_map_tensor = o3d.t.geometry.PointCloud(self.device)
        self.transforms = []

        # Feature extraction (GPU always enabled)
        self.feature_extractor = FeatureExtractor(
            method=feature_extraction_method,
            voxel_size=voxel_size
        )

        # Loop closure
        self.enable_loop_closure = enable_loop_closure
        if enable_loop_closure:
            self.loop_closure_detector = LoopClosureDetector()
        else:
            self.loop_closure_detector = None

    def process_submap(self, points: np.ndarray, submap_id: int) -> o3d.t.geometry.PointCloud:

        points_tensor = o3c.Tensor(points.astype(np.float32), dtype=o3c.float32, device=self.device)
        pcd_tensor = o3d.t.geometry.PointCloud(self.device)
        pcd_tensor.point.positions = points_tensor
        pcd_tensor = pcd_tensor.voxel_down_sample(voxel_size=self.voxel_size)

        return pcd_tensor

    def extract_features(self, pcd: o3d.geometry.PointCloud, submap_id: int) -> Optional[Dict]:

        try:
            features = self.feature_extractor.extract(pcd)

            # Handle different metadata structures for hybrid vs single method
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
                max_iteration=100, 
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
                                       scan_count: int) -> Tuple[bool, Optional[dict]]:

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

        features = None
        if self.enable_loop_closure:
            pcd_legacy = pcd_tensor.to_legacy()
            features = self.extract_features(pcd_legacy, submap_id)

        if submap_id == 0:

            self.global_map_tensor = pcd_tensor
            initial_transform = np.eye(4)

            submap_data = {
                'id': submap_id,
                'point_cloud': pcd_tensor.to_legacy(),  # Only convert for storage
                'features': features,
                'pose_start': start_pose,
                'pose_end': end_pose,
                'pose_center': pose_center,
                'global_transform': initial_transform,
                'scan_count': scan_count,
                'timestamp_created': time.time()
            }

            self.submaps.append(submap_data)
            self.transforms.append(initial_transform)

            # Add to loop closure spatial index
            if self.loop_closure_detector is not None:
                self.loop_closure_detector.add_submap_to_index(
                    submap_id,
                    np.array([pose_center['x'], pose_center['y']])
                )

            return True, None  # No correction for first submap

        # Points are already in world frame, use identity for ICP
        initial_transform = np.eye(4)

        success, icp_refinement, fitness = self.align_submap_with_icp(
            source=pcd_tensor,
            target=self.global_map_tensor,
            initial_guess=initial_transform
        )

        # ICP returns refinement transform relative to initial_guess
        # Final transform = initial_guess (from odometry) composed with ICP refinement
        # Since ICP already applies initial_guess internally, icp_refinement is the TOTAL transform
        final_transform = icp_refinement

        # Calculate ICP correction (difference from odometry prediction)
        icp_correction = np.linalg.inv(initial_transform) @ final_transform
        correction_translation = np.linalg.norm(icp_correction[:2, 3])
        correction_rotation = np.abs(np.arctan2(icp_correction[1, 0], icp_correction[0, 0]))

        if correction_translation > 0.5 or correction_rotation > np.radians(10):  # ICP correction too large
            final_transform = initial_transform  # Reject ICP, use odometry
            success = False

        if not success or fitness < self.icp_fitness_threshold:
            final_transform = initial_transform  # Reject ICP, use odometry

        # Extract pose correction for robot pose update (relative to odometry)
        pose_correction = None
        if success and correction_translation > 0.01:  # Only if non-trivial correction (>1cm)
            pose_correction = {
                'dx': icp_correction[0, 3],
                'dy': icp_correction[1, 3],
                'dtheta': np.arctan2(icp_correction[1, 0], icp_correction[0, 0])
            }

        transform_tensor = o3c.Tensor(final_transform, dtype=o3c.float32, device=self.device)
        pcd_aligned_tensor = pcd_tensor.transform(transform_tensor)

        submap_data = {
            'id': submap_id,
            'point_cloud': pcd_aligned_tensor.to_legacy(),  # Store ICP-aligned version
            'features': features,
            'pose_start': start_pose,
            'pose_end': end_pose,
            'pose_center': pose_center,
            'global_transform': final_transform,
            'scan_count': scan_count,
            'timestamp_created': time.time()
        }

        self.submaps.append(submap_data)
        self.transforms.append(final_transform)

        if self.loop_closure_detector is not None:
            self.loop_closure_detector.add_submap_to_index(
                submap_id,
                np.array([pose_center['x'], pose_center['y']])
            )

        if self.enable_loop_closure:
            if features is not None:
                loop_closure = self.loop_closure_detector.detect_loop_closure(
                    submap_data,
                    self.submaps,
                    min_time_separation=30.0
                )

                if loop_closure is not None:
                    print(f"  ✓ Loop closure: submap {submap_id} ↔ {loop_closure['match_id']}")

                    optimized = self._optimize_pose_graph_with_loop_closure(
                        current_id=submap_id,
                        loop_match_id=loop_closure['match_id'],
                        loop_transform=loop_closure.get('transform', np.eye(4))
                    )

                    if optimized:
                        self._rebuild_global_map()

        self.global_map_tensor.point.positions = o3c.concatenate([
            self.global_map_tensor.point.positions,
            pcd_aligned_tensor.point.positions
        ], axis=0)

        self.global_map_tensor = self.global_map_tensor.voxel_down_sample(self.voxel_size)

        self.global_map_tensor, _ = self.global_map_tensor.remove_statistical_outliers(nb_neighbors=10, std_ratio=3.0)

        return True, pose_correction

    def save_global_map(self, filepath: str):
        """Save the global map with final cleanup"""
        if len(self.global_map_tensor.point.positions) == 0:
            return

        global_map_legacy = self.global_map_tensor.to_legacy()
        o3d.io.write_point_cloud(filepath, global_map_legacy)

    def get_global_map_points(self) -> Optional[np.ndarray]:

        if 'positions' not in self.global_map_tensor.point or len(self.global_map_tensor.point.positions) == 0:
            return None
        return self.global_map_tensor.point.positions.cpu().numpy()

    def _optimize_pose_graph_with_loop_closure(self,
                                               current_id: int,
                                               loop_match_id: int,
                                               loop_transform: np.ndarray) -> bool:
        if len(self.submaps) < 3:
            return False

        current_pose = self.submaps[current_id]['global_transform']
        matched_pose = self.submaps[loop_match_id]['global_transform']

        # Measured relative transform from loop closure
        measured_relative = loop_transform

        # Current relative transform (from odometry chain)
        current_relative = np.linalg.inv(matched_pose) @ current_pose

        # Calculate error
        error_transform = measured_relative @ np.linalg.inv(current_relative)

        # Extract 2D error (translation and rotation)
        error_x = error_transform[0, 3]
        error_y = error_transform[1, 3]
        error_theta = np.arctan2(error_transform[1, 0], error_transform[0, 0])

        # Only optimize if error is significant (>10cm or >5°)
        if abs(error_x) < 0.1 and abs(error_y) < 0.1 and abs(error_theta) < np.radians(5):
            return False

        # Distribute error linearly across submaps in the loop
        num_submaps_in_loop = current_id - loop_match_id

        for i in range(loop_match_id + 1, current_id + 1):
            # Linear interpolation factor
            alpha = (i - loop_match_id) / num_submaps_in_loop

            # Correction for this submap
            correction_x = alpha * error_x
            correction_y = alpha * error_y
            correction_theta = alpha * error_theta

            # Build correction transform
            cos_theta = np.cos(correction_theta)
            sin_theta = np.sin(correction_theta)
            correction_T = np.array([
                [cos_theta, -sin_theta, 0, correction_x],
                [sin_theta, cos_theta, 0, correction_y],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])

            # Apply correction to submap transform
            old_transform = self.submaps[i]['global_transform'].copy()
            new_transform = correction_T @ old_transform

            self.submaps[i]['global_transform'] = new_transform
            self.transforms[i] = new_transform

        return True

    def _rebuild_global_map(self):
        """Rebuild global map after loop closure optimization"""
        # Clear current global map
        self.global_map_tensor = o3d.t.geometry.PointCloud(self.device)

        # Re-stitch all submaps with optimized transforms
        for submap in self.submaps:
            # Transform submap to optimized pose (use tensor operations)
            pcd_legacy = o3d.geometry.PointCloud(submap['point_cloud'])
            pcd_legacy.transform(submap['global_transform'])

            # Convert to tensor and add to global map
            pcd_tensor = o3d.t.geometry.PointCloud.from_legacy(
                pcd_legacy, dtype=o3c.float32, device=self.device
            )

            # Concatenate with existing global map
            if len(self.global_map_tensor.point.positions) == 0:
                self.global_map_tensor = pcd_tensor
            else:
                self.global_map_tensor.point.positions = o3c.concatenate([
                    self.global_map_tensor.point.positions,
                    pcd_tensor.point.positions
                ], axis=0)

        # Final downsample to prevent duplicates
        self.global_map_tensor = self.global_map_tensor.voxel_down_sample(self.voxel_size)

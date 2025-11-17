#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import open3d.core as o3c
from typing import Optional, Tuple, Dict, List
import time
import os
from map_generation.utils import quaternion_to_yaw
from map_generation.feature_extractor import FeatureExtractor
from map_generation.loop_closure_detector import LoopClosureDetector
from map_generation.geometry_utils import estimate_transform_from_poses


class SubmapStitcher:
    
    def __init__(self,
                 voxel_size: float = 0.05,
                 icp_max_correspondence_dist: float = 0.25,
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
        self.prev_submap_pose = None  # Track previous submap pose for odometry-based ICP initial guess

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
            print(f"  ✗ Feature extraction failed for submap {submap_id}: {e}")
            return None

    def compute_submap_bounds(self, pcd_tensor: o3d.t.geometry.PointCloud) -> Dict:
        
        points = pcd_tensor.point.positions.cpu().numpy()
        return {
            'min_x': float(points[:, 0].min()),
            'max_x': float(points[:, 0].max()),
            'min_y': float(points[:, 1].min()),
            'max_y': float(points[:, 1].max())
        }

    def estimate_2d_transform(self, pose_from: dict, pose_to: dict) -> np.ndarray:
        
        return estimate_transform_from_poses(pose_from, pose_to, quaternion_to_yaw)

    def register_icp_2d(self,
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
                max_iteration=100,  # Reduced from 200 for speed
                relative_fitness=1e-6,  # Stop when fitness improvement < 0.0001%
                relative_rmse=1e-6      # Stop when RMSE improvement < 0.0001%
            )
        )

        # Constrain transformation to 2D (safety measure)
        transform = reg_result.transformation.cpu().numpy()
        # Keep only X, Y translation and rotation around Z
        transform[2, 0:3] = [0, 0, 1]  # No Z translation, preserve rotation
        transform[2, 3] = 0             # No Z offset
        transform[0:2, 2] = [0, 0]      # No X,Y coupling with Z axis

        success = reg_result.fitness >= self.icp_fitness_threshold

        return success, transform, reg_result.fitness

    def add_and_stitch_submap(self,
                             points: np.ndarray,
                             submap_id: int,
                             start_pose: dict,
                             end_pose: dict,
                             scan_count: int) -> bool:

        # Process submap (returns GPU tensor)
        pcd_tensor = self.process_submap(points, submap_id)

        if len(pcd_tensor.point.positions) < 50:
            return False

        # Compute submap center pose
        pose_center = {
            'x': (start_pose['x'] + end_pose['x']) / 2,
            'y': (start_pose['y'] + end_pose['y']) / 2,
            'z': 0.0,
            'qx': end_pose['qx'],
            'qy': end_pose['qy'],
            'qz': end_pose['qz'],
            'qw': end_pose['qw']
        }

        # Extract features only if loop closure is enabled (convert to legacy only if needed)
        features = None
        if self.enable_loop_closure:
            pcd_legacy = pcd_tensor.to_legacy()
            features = self.extract_features(pcd_legacy, submap_id)

        # Compute bounds (works with tensor)
        bounds = self.compute_submap_bounds(pcd_tensor)

        # First submap: initialize global map
        if submap_id == 0:
            # Points are already in world frame and already a GPU tensor - just copy
            self.global_map_tensor = pcd_tensor
            initial_transform = np.eye(4)  # Identity (no transform needed for world frame)

            # Store submap (convert to legacy only for storage)
            submap_data = {
                'id': submap_id,
                'point_cloud': pcd_tensor.to_legacy(),  # Only convert for storage
                'features': features,
                'pose_start': start_pose,
                'pose_end': end_pose,
                'pose_center': pose_center,
                'global_transform': initial_transform,
                'scan_count': scan_count,
                'bounds': bounds,
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

            # Update previous pose for next submap's odometry-based initial guess
            self.prev_submap_pose = pose_center

            return True

        # For subsequent submaps: use ICP to correct accumulated drift
        if self.prev_submap_pose is not None:
            # Compute relative transform from previous submap to current submap
            initial_transform = self.estimate_2d_transform(self.prev_submap_pose, pose_center)
        else:
            # First submap after initialization: use identity
            initial_transform = np.eye(4)

        # Run ICP registration to correct long-term accumulated drift (tensor-native, stays on GPU!)
        success, final_transform, fitness = self.register_icp_2d(
            source=pcd_tensor,
            target=self.global_map_tensor,
            initial_guess=initial_transform
        )

        # Check if transform is reasonable (not too large)
        translation = np.linalg.norm(final_transform[:2, 3])
        rotation = np.abs(np.arctan2(final_transform[1, 0], final_transform[0, 0]))

        # Reject if correction is unreasonably large (indicates ICP failure)
        if translation > 2.0 or rotation > np.radians(30):  # 2m or 30 degrees
            final_transform = np.eye(4)
            success = False

        # Use consistent fitness threshold
        if not success or fitness < self.icp_fitness_threshold:
            final_transform = np.eye(4)
        else:
            print(f"  ✓ ICP: t={translation:.3f}m, r={np.degrees(rotation):.1f}°, fitness={fitness:.3f}")

        # Transform submap to align with global map (tensor-native, stays on GPU!)
        transform_tensor = o3c.Tensor(final_transform, dtype=o3c.float32, device=self.device)
        pcd_aligned_tensor = pcd_tensor.transform(transform_tensor)

        # Store submap (convert to legacy only for storage)
        submap_data = {
            'id': submap_id,
            'point_cloud': pcd_tensor.to_legacy(),  # Only convert for storage
            'features': features,
            'pose_start': start_pose,
            'pose_end': end_pose,
            'pose_center': pose_center,
            'global_transform': final_transform,
            'scan_count': scan_count,
            'bounds': bounds,
            'timestamp_created': time.time()
        }

        self.submaps.append(submap_data)
        self.transforms.append(final_transform)

        # Add to loop closure spatial index
        if self.loop_closure_detector is not None:
            self.loop_closure_detector.add_submap_to_index(
                submap_id,
                np.array([pose_center['x'], pose_center['y']])
            )

        # Check for loop closure (skip entirely if disabled for performance)
        if self.enable_loop_closure:
            if features is not None:
                loop_closure = self.loop_closure_detector.detect_loop_closure(
                    submap_data,
                    self.submaps,
                    min_time_separation=30.0
                )

                if loop_closure is not None:
                    print(f"  ✓ Loop closure: submap {submap_id} ↔ {loop_closure['match_id']}")

                    # Apply loop closure correction via pose graph optimization
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

        # Voxel downsample to merge overlapping points (5cm voxels) - stays on GPU
        self.global_map_tensor = self.global_map_tensor.voxel_down_sample(self.voxel_size)

        # Outlier removal to clean ICP alignment errors (relaxed parameters) - stays on GPU
        self.global_map_tensor, _ = self.global_map_tensor.remove_statistical_outliers(nb_neighbors=10, std_ratio=3.0)

        # Update previous pose for next submap's odometry-based initial guess
        self.prev_submap_pose = pose_center

        return True

    def save_global_map(self, filepath: str):
        """Save the global map with final cleanup"""
        if len(self.global_map_tensor.point.positions) == 0:
            return

        # Convert to legacy only for saving (downsampling already done during stitching)
        global_map_legacy = self.global_map_tensor.to_legacy()
        o3d.io.write_point_cloud(filepath, global_map_legacy)

    def save_submaps(self, directory: str):

        os.makedirs(directory, exist_ok=True)

        for submap in self.submaps:
            submap_file = os.path.join(directory, f"submap_{submap['id']:04d}.pcd")
            o3d.io.write_point_cloud(submap_file, submap['point_cloud'])

            # Save metadata
            metadata_file = os.path.join(directory, f"submap_{submap['id']:04d}_metadata.npz")
            np.savez(
                metadata_file,
                id=submap['id'],
                global_transform=submap['global_transform'],
                pose_center=np.array([submap['pose_center']['x'],
                                     submap['pose_center']['y'],
                                     submap['pose_center']['z']]),
                scan_count=submap['scan_count'],
                timestamp=submap['timestamp_created']
            )

    def get_global_map_points(self) -> Optional[np.ndarray]:
        """Get global map points as numpy array (optimized GPU→CPU transfer)"""
        if len(self.global_map_tensor.point.positions) == 0:
            return None
        # Direct GPU→CPU transfer, returns numpy array
        return self.global_map_tensor.point.positions.cpu().numpy()

    def get_submap(self, submap_id: int) -> Optional[Dict]:
        """Get submap by ID"""
        for submap in self.submaps:
            if submap['id'] == submap_id:
                return submap
        return None

    def get_statistics(self) -> dict:
        """Get mapping statistics"""
        stats = {
            'num_submaps': len(self.submaps),
            'global_map_points': len(self.global_map.points),
            'total_transforms': len(self.transforms)
        }

        if self.loop_closure_detector is not None:
            stats.update(self.loop_closure_detector.get_statistics())

        return stats

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

        # Clear current global map
        self.global_map = o3d.geometry.PointCloud()

        # Re-stitch all submaps with optimized transforms
        for submap in self.submaps:
            # Transform submap to optimized pose
            pcd_aligned = o3d.geometry.PointCloud(submap['point_cloud'])
            pcd_aligned.transform(submap['global_transform'])

            # Add to global map
            self.global_map += pcd_aligned

        # Final downsample to prevent duplicates
        global_map_tensor = o3d.t.geometry.PointCloud.from_legacy(
            self.global_map, dtype=o3c.float32, device=self.device
        )
        global_map_tensor = global_map_tensor.voxel_down_sample(self.voxel_size)
        self.global_map = global_map_tensor.to_legacy()

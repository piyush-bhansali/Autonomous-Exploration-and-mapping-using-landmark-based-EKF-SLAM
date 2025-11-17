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
                 icp_max_correspondence_dist: float = 0.15,  # 3x voxel size - tighter for accuracy
                 icp_fitness_threshold: float = 0.5,  # Require 50% point alignment for quality
                 feature_extraction_method: str = 'hybrid',
                 enable_loop_closure: bool = False):

        self.voxel_size = voxel_size
        self.icp_max_dist = icp_max_correspondence_dist
        self.icp_fitness_threshold = icp_fitness_threshold

        # GPU configuration (always enabled)
        if o3c.cuda.is_available():
            self.device = o3c.Device("CUDA:0")
            print(f"✓ SubmapStitcher: GPU enabled on {self.device}")
        else:
            self.device = o3c.Device("CPU:0")
            print("⚠ SubmapStitcher: GPU not available, using CPU")

        # Submap storage (for loop closure)
        self.submaps = []
        self.global_map = o3d.geometry.PointCloud()
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
            print("✓ Loop closure detection enabled")
        else:
            self.loop_closure_detector = None
            print("ℹ Loop closure detection disabled")

    def process_submap(self, points: np.ndarray, submap_id: int) -> o3d.geometry.PointCloud:
        # Convert numpy array to tensor PointCloud for GPU processing
        points_tensor = o3c.Tensor(points.astype(np.float32), dtype=o3c.float32, device=self.device)
        pcd_tensor = o3d.t.geometry.PointCloud(self.device)
        pcd_tensor.point.positions = points_tensor

        # GPU-accelerated voxel downsampling and outlier removal
        pcd_tensor = pcd_tensor.voxel_down_sample(voxel_size=self.voxel_size)
        pcd_tensor, _ = pcd_tensor.remove_statistical_outliers(nb_neighbors=20, std_ratio=2.5)

        pcd = pcd_tensor.to_legacy()
        print(f"  Submap {submap_id}: {len(points)} → {len(pcd.points)} points")
        return pcd

    def extract_features(self, pcd: o3d.geometry.PointCloud, submap_id: int) -> Optional[Dict]:

        print(f"  Extracting features for submap {submap_id}...")

        try:
            features = self.feature_extractor.extract(pcd)

            # Handle different metadata structures for hybrid vs single method
            if features['method'] == 'hybrid':
                # Hybrid mode nests metadata
                num_keypoints = features['metadata']['geometric']['num_keypoints']
                descriptor_dim = features['metadata']['geometric']['descriptor_dim']
            else:
                # Single method mode has flat metadata
                num_keypoints = features['metadata']['num_keypoints']
                descriptor_dim = features['metadata']['descriptor_dim']

            if num_keypoints == 0:
                print(f"  ⚠ No features extracted for submap {submap_id}")
                return None

            print(f"  ✓ Features: {num_keypoints} keypoints, "
                  f"{descriptor_dim}-D descriptors "
                  f"({features['metadata']['computation_time']:.2f}s)")

            return features

        except Exception as e:
            print(f"  ✗ Feature extraction failed for submap {submap_id}: {e}")
            return None

    def compute_submap_bounds(self, pcd: o3d.geometry.PointCloud) -> Dict:
        points = np.asarray(pcd.points)
        return {
            'min_x': float(points[:, 0].min()),
            'max_x': float(points[:, 0].max()),
            'min_y': float(points[:, 1].min()),
            'max_y': float(points[:, 1].max())
        }

    def estimate_2d_transform(self, pose_from: dict, pose_to: dict) -> np.ndarray:
        """Estimate 2D transformation between poses (in global frame)"""
        return estimate_transform_from_poses(pose_from, pose_to, quaternion_to_yaw)

    def register_icp_2d(self,
                       source: o3d.geometry.PointCloud,
                       target: o3d.geometry.PointCloud,
                       initial_guess: Optional[np.ndarray] = None) -> Tuple[bool, np.ndarray, float]:
        """ICP registration for 2D point clouds (Z=0)"""
        if initial_guess is None:
            initial_guess = np.eye(4)

        # Convert legacy PointCloud to tensor PointCloud for GPU processing
        source_tensor = o3d.t.geometry.PointCloud.from_legacy(source, dtype=o3c.float32, device=self.device)
        target_tensor = o3d.t.geometry.PointCloud.from_legacy(target, dtype=o3c.float32, device=self.device)

        # Point-to-Point ICP
        init_transform = o3c.Tensor(initial_guess, dtype=o3c.float32, device=self.device)

        reg_result = o3d.t.pipelines.registration.icp(
            source=source_tensor,
            target=target_tensor,
            max_correspondence_distance=self.icp_max_dist,
            init_source_to_target=init_transform,
            estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
        )

        # Constrain transformation to 2D (safety measure)
        transform = reg_result.transformation.cpu().numpy()
        transform[2, :] = [0, 0, 1, 0]
        transform[:, 2] = [0, 0, 1, 0]

        success = reg_result.fitness >= self.icp_fitness_threshold
        print(f"  ICP: fitness={reg_result.fitness:.3f}, success={success}")

        return success, transform, reg_result.fitness

    def add_and_stitch_submap(self,
                             points: np.ndarray,
                             submap_id: int,
                             start_pose: dict,
                             end_pose: dict,
                             scan_count: int) -> bool:
        
        print(f"\nProcessing submap {submap_id}", flush=True)

        # Process submap
        pcd = self.process_submap(points, submap_id)

        if len(pcd.points) < 50:
            print(f"  ⚠ Submap {submap_id} has too few points, skipping")
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

        # Extract features
        features = self.extract_features(pcd, submap_id)

        # Compute bounds
        bounds = self.compute_submap_bounds(pcd)

        # First submap: initialize global map
        if submap_id == 0:
            # Points are already in world frame, just copy to global map
            self.global_map = o3d.geometry.PointCloud(pcd)
            initial_transform = np.eye(4)  # Identity (no transform needed for world frame)

            # Store submap
            submap_data = {
                'id': submap_id,
                'point_cloud': pcd,
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

            print(f"  Submap {submap_id}: Added as first submap (world frame)")
            return True

        # For subsequent submaps: use ICP to correct odometry drift
        # Even though points are in world frame based on odometry, ICP refines alignment
        print(f"  DEBUG: Submap {submap_id} entering ICP path, global_map has {len(self.global_map.points)} points", flush=True)

        # Get previous submap transform for initial guess
        prev_submap = self.submaps[-1]
        prev_transform = prev_submap['global_transform']

        # Compute odometry-based transform between submaps
        odom_transform = self.estimate_2d_transform(prev_submap['pose_center'], pose_center)

        # Initial guess: previous transform + odometry change
        initial_transform = prev_transform @ odom_transform

        # Run ICP registration to correct drift
        print(f"  Running ICP for submap {submap_id}...", flush=True)
        success, final_transform, fitness = self.register_icp_2d(
            source=pcd,
            target=self.global_map,
            initial_guess=initial_transform
        )

        if not success:
            print(f"  ⚠ ICP failed (fitness={fitness:.3f}), using odometry-based transform")
            final_transform = initial_transform

        # Transform submap to align with global map
        pcd_aligned = o3d.geometry.PointCloud(pcd)
        pcd_aligned.transform(final_transform)

        # Store submap
        submap_data = {
            'id': submap_id,
            'point_cloud': pcd,
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

        # Check for loop closure
        if self.enable_loop_closure and features is not None:
            loop_closure = self.loop_closure_detector.detect_loop_closure(
                submap_data,
                self.submaps,
                min_time_separation=30.0
            )

            if loop_closure is not None:
                print(f"  ✓ Loop closure detected: submap {submap_id} matches submap {loop_closure['match_id']}")

                # Apply loop closure correction via pose graph optimization
                optimized = self._optimize_pose_graph_with_loop_closure(
                    current_id=submap_id,
                    loop_match_id=loop_closure['match_id'],
                    loop_transform=loop_closure.get('transform', np.eye(4))
                )

                if optimized:
                    print(f"  ✓ Pose graph optimized, reconstructing global map...")
                    # Rebuild global map with optimized transforms
                    self._rebuild_global_map()
                else:
                    print(f"  ⚠ Pose graph optimization failed, continuing with current map")

        # Add ICP-aligned submap to global map
        self.global_map += pcd_aligned
        print(f"  Submap {submap_id}: Merged to global map (ICP-aligned)")

        # ALWAYS downsample global map after adding new submap to prevent duplicate points
        print(f"  Before downsampling: {len(self.global_map.points)} points")
        global_map_tensor = o3d.t.geometry.PointCloud.from_legacy(self.global_map, dtype=o3c.float32, device=self.device)
        global_map_tensor = global_map_tensor.voxel_down_sample(self.voxel_size)
        self.global_map = global_map_tensor.to_legacy()

        print(f"  Submap {submap_id} stitched: {len(self.global_map.points)} total points (after voxel downsample)")

        return True

    def save_global_map(self, filepath: str):
        """Save the global map with final cleanup"""
        if len(self.global_map.points) == 0:
            print("No points in global map")
            return

        # Final cleanup: voxel downsample and outlier removal
        final_map_tensor = o3d.t.geometry.PointCloud.from_legacy(self.global_map, dtype=o3c.float32, device=self.device)
        final_map_tensor = final_map_tensor.voxel_down_sample(self.voxel_size)
        final_map_tensor, _ = final_map_tensor.remove_statistical_outliers(nb_neighbors=20, std_ratio=2.0)

        final_map = final_map_tensor.to_legacy()
        o3d.io.write_point_cloud(filepath, final_map)
        print(f"Saved global map: {filepath} ({len(final_map.points)} points)")

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

        print(f"Saved {len(self.submaps)} submaps to {directory}")

    def get_global_map_points(self) -> Optional[np.ndarray]:
        """Get global map points as numpy array"""
        if len(self.global_map.points) == 0:
            return None
        return np.asarray(self.global_map.points)

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
        """
        Optimize pose graph when loop closure is detected

        Uses simple graph relaxation to distribute error across all submaps
        More sophisticated implementation would use g2o or similar

        Args:
            current_id: Current submap ID
            loop_match_id: ID of matched submap (earlier in trajectory)
            loop_transform: Measured transform between current and matched submap

        Returns:
            True if optimization succeeded
        """
        if len(self.submaps) < 3:
            return False

        print(f"  Optimizing pose graph: {len(self.submaps)} submaps, loop {current_id} -> {loop_match_id}")

        # Simple loop closure: distribute error linearly across submaps in the loop
        # This is a basic implementation - production code would use g2o or GTSAM

        # Calculate error between measured loop closure and current transforms
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

        print(f"  Loop closure error: dx={error_x:.3f}m, dy={error_y:.3f}m, dθ={np.degrees(error_theta):.1f}°")

        # Only optimize if error is significant (>10cm or >5°)
        if abs(error_x) < 0.1 and abs(error_y) < 0.1 and abs(error_theta) < np.radians(5):
            print(f"  Error too small, skipping optimization")
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

        print(f"  ✓ Optimized {num_submaps_in_loop} submap poses")
        return True

    def _rebuild_global_map(self):
        """
        Rebuild global map from scratch using optimized transforms

        Called after loop closure optimization to apply corrected poses
        """
        print(f"  Rebuilding global map with optimized poses...")

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
        print(f"  Before final downsample: {len(self.global_map.points)} points")
        global_map_tensor = o3d.t.geometry.PointCloud.from_legacy(
            self.global_map, dtype=o3c.float32, device=self.device
        )
        global_map_tensor = global_map_tensor.voxel_down_sample(self.voxel_size)
        self.global_map = global_map_tensor.to_legacy()

        print(f"  ✓ Global map rebuilt: {len(self.global_map.points)} points")

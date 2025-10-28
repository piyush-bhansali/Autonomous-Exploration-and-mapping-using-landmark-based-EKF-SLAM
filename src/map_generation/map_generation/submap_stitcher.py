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


class SubmapStitcher:
    
    def __init__(self,
                 voxel_size: float = 0.05,
                 icp_max_correspondence_dist: float = 0.1,
                 icp_fitness_threshold: float = 0.3,
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
        pcd_tensor, _ = pcd_tensor.remove_statistical_outliers(nb_neighbors=20, std_ratio=2.0)

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
        # Extract positions and orientations
        x_from, y_from = pose_from['x'], pose_from['y']
        x_to, y_to = pose_to['x'], pose_to['y']

        theta_from = quaternion_to_yaw(pose_from['qx'], pose_from['qy'],
                                       pose_from['qz'], pose_from['qw'])
        theta_to = quaternion_to_yaw(pose_to['qx'], pose_to['qy'],
                                     pose_to['qz'], pose_to['qw'])

        # Build 2D transformation matrices
        cos_from, sin_from = np.cos(theta_from), np.sin(theta_from)
        cos_to, sin_to = np.cos(theta_to), np.sin(theta_to)

        T_from = np.eye(4)
        T_from[0:2, 0:2] = [[cos_from, -sin_from], [sin_from, cos_from]]
        T_from[0:2, 3] = [x_from, y_from]

        T_to = np.eye(4)
        T_to[0:2, 0:2] = [[cos_to, -sin_to], [sin_to, cos_to]]
        T_to[0:2, 3] = [x_to, y_to]

        # Relative transformation: T_rel = T_to @ inv(T_from)
        return T_to @ np.linalg.inv(T_from)

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
            criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
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
        
        print(f"\nProcessing submap {submap_id}")

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
            self.global_map = o3d.geometry.PointCloud(pcd)
            initial_transform = np.eye(4)

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

            print(f"  Submap {submap_id}: Added as first submap")
            return True

        # For subsequent submaps: register and stitch
        prev_submap = self.submaps[-1]

        # Compute initial global transform from odometry
        odom_transform = self.estimate_2d_transform(prev_submap['pose_end'], start_pose)
        initial_global_transform = prev_submap['global_transform'] @ odom_transform

        # ICP registration: align new submap to global map
        if len(self.global_map.points) > 0:
            success, final_global_transform, fitness = self.register_icp_2d(
                source=pcd,
                target=self.global_map,
                initial_guess=initial_global_transform
            )
            if not success:
                final_global_transform = initial_global_transform
        else:
            final_global_transform = initial_global_transform

        # Store submap
        submap_data = {
            'id': submap_id,
            'point_cloud': pcd,
            'features': features,
            'pose_start': start_pose,
            'pose_end': end_pose,
            'pose_center': pose_center,
            'global_transform': final_global_transform,
            'scan_count': scan_count,
            'bounds': bounds,
            'timestamp_created': time.time()
        }

        self.submaps.append(submap_data)
        self.transforms.append(final_global_transform)

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
                print(f"  Loop closure detected: submap {submap_id} matches submap {loop_closure['match_id']}")
                # TODO: Apply loop closure correction to global map

        # Add transformed submap to global map
        pcd_final = o3d.geometry.PointCloud(pcd)
        pcd_final.transform(final_global_transform)
        self.global_map += pcd_final

        # Downsample global map if too large
        if len(self.global_map.points) > 100000:
            global_map_tensor = o3d.t.geometry.PointCloud.from_legacy(self.global_map, dtype=o3c.float32, device=self.device)
            global_map_tensor = global_map_tensor.voxel_down_sample(self.voxel_size)
            self.global_map = global_map_tensor.to_legacy()

        print(f"  Submap {submap_id} stitched: {len(self.global_map.points)} total points")

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

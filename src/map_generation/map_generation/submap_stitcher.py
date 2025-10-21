#!/usr/bin/env python3
"""
Submap Stitcher - GPU-accelerated version using Open3D CUDA
"""

import numpy as np
import open3d as o3d
import open3d.core as o3c
from typing import Optional, Tuple
from map_generation.utils import quaternion_to_yaw


class SubmapStitcher:


    def __init__(self,
                 voxel_size: float = 0.05,
                 icp_max_correspondence_dist: float = 0.5,
                 icp_fitness_threshold: float = 0.2,
                 use_gpu: bool = True,
                 gpu_device_id: int = 0):

        self.voxel_size = voxel_size
        self.icp_max_dist = icp_max_correspondence_dist
        self.icp_fitness_threshold = icp_fitness_threshold

        # GPU configuration
        self.use_gpu = use_gpu
        if self.use_gpu and o3c.cuda.is_available():
            self.device = o3c.Device(f"CUDA:{gpu_device_id}")
            print(f"✓ GPU acceleration enabled: {self.device}")
        else:
            self.device = o3c.Device("CPU:0")
            if self.use_gpu:
                print("⚠ WARNING: GPU requested but CUDA not available, falling back to CPU")
            else:
                print("ℹ Using CPU for processing")

        # Storage
        self.submaps = []
        self.global_map = o3d.geometry.PointCloud()
        self.transforms = []  # Global transform for each submap

    def numpy_to_tensor_pcd(self, points: np.ndarray) -> o3d.t.geometry.PointCloud:
        """Convert numpy array to Open3D tensor-based PointCloud (GPU-compatible)"""
        # Create tensor on the selected device
        points_tensor = o3c.Tensor(points.astype(np.float32), dtype=o3c.float32, device=self.device)
        pcd = o3d.t.geometry.PointCloud(self.device)
        pcd.point.positions = points_tensor
        return pcd

    def tensor_to_legacy_pcd(self, tensor_pcd: o3d.t.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Convert tensor-based PointCloud to legacy PointCloud for visualization"""
        return tensor_pcd.to_legacy()

    def legacy_to_tensor_pcd(self, legacy_pcd: o3d.geometry.PointCloud) -> o3d.t.geometry.PointCloud:
        """Convert legacy PointCloud to tensor-based PointCloud"""
        points = np.asarray(legacy_pcd.points).astype(np.float32)
        return self.numpy_to_tensor_pcd(points)

    def process_submap(self,
                       points: np.ndarray,
                       submap_id: int) -> o3d.geometry.PointCloud:

        # Convert to tensor-based PointCloud for GPU processing
        pcd_tensor = self.numpy_to_tensor_pcd(points)

        # GPU-accelerated voxel downsampling
        pcd_tensor = pcd_tensor.voxel_down_sample(voxel_size=self.voxel_size)

        # GPU-accelerated statistical outlier removal
        pcd_tensor, _ = pcd_tensor.remove_statistical_outliers(nb_neighbors=20, std_ratio=2.0)

        # Convert back to legacy PointCloud for compatibility
        pcd = self.tensor_to_legacy_pcd(pcd_tensor)

        print(f"Submap {submap_id}: {len(points)} → {len(pcd.points)} points (GPU-processed)")

        return pcd
    
    def estimate_2d_transform(self,
                             pose_from: dict,
                             pose_to: dict) -> np.ndarray:
        
        # Extract positions
        x_from, y_from = pose_from['x'], pose_from['y']
        x_to, y_to = pose_to['x'], pose_to['y']
        
        # Extract orientations
        theta_from = quaternion_to_yaw(
            pose_from['qx'], pose_from['qy'],
            pose_from['qz'], pose_from['qw']
        )
        theta_to = quaternion_to_yaw(
            pose_to['qx'], pose_to['qy'],
            pose_to['qz'], pose_to['qw']
        )
        
        # Build transformation matrices
        T_from = np.eye(4)
        T_from[0:2, 0:2] = np.array([
            [np.cos(theta_from), -np.sin(theta_from)],
            [np.sin(theta_from), np.cos(theta_from)]
        ])
        T_from[0:2, 3] = [x_from, y_from]
        
        T_to = np.eye(4)
        T_to[0:2, 0:2] = np.array([
            [np.cos(theta_to), -np.sin(theta_to)],
            [np.sin(theta_to), np.cos(theta_to)]
        ])
        T_to[0:2, 3] = [x_to, y_to]
        
        # Relative transformation
        T_rel = T_to @ np.linalg.inv(T_from)
        
        return T_rel
    
    def register_icp_2d(self,
                       source: o3d.geometry.PointCloud,
                       target: o3d.geometry.PointCloud,
                       initial_guess: Optional[np.ndarray] = None) -> Tuple[bool, np.ndarray, float]:

        if initial_guess is None:
            initial_guess = np.eye(4)

        # Convert to tensor-based PointClouds for GPU processing
        source_tensor = self.legacy_to_tensor_pcd(source)
        target_tensor = self.legacy_to_tensor_pcd(target)

        # Ensure 2D by zeroing Z coordinates
        source_points = source_tensor.point.positions.cpu().numpy()
        target_points = target_tensor.point.positions.cpu().numpy()
        source_points[:, 2] = 0
        target_points[:, 2] = 0

        # Create new tensor point clouds with 2D constraints
        source_2d = self.numpy_to_tensor_pcd(source_points)
        target_2d = self.numpy_to_tensor_pcd(target_points)

        # GPU-accelerated normal estimation
        source_2d.estimate_normals(max_nn=30, radius=self.voxel_size * 2)
        target_2d.estimate_normals(max_nn=30, radius=self.voxel_size * 2)

        # Convert initial guess to tensor
        init_transform = o3c.Tensor(initial_guess, dtype=o3c.float32, device=self.device)

        # GPU-accelerated Point-to-Plane ICP
        reg_result = o3d.t.pipelines.registration.icp(
            source=source_2d,
            target=target_2d,
            max_correspondence_distance=self.icp_max_dist,
            init_source_to_target=init_transform,
            estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=100
            )
        )

        # Get transformation as numpy array
        transform = reg_result.transformation.cpu().numpy()

        # Constrain to 2D
        transform[2, :] = [0, 0, 1, 0]
        transform[:, 2] = [0, 0, 1, 0]

        # Calculate fitness
        fitness = reg_result.fitness

        success = fitness >= self.icp_fitness_threshold

        print(f"  ICP (GPU): fitness={fitness:.3f}, rmse={reg_result.inlier_rmse:.3f}, success={success}")

        return success, transform, fitness
    
    def add_and_stitch_submap(self,
                             points: np.ndarray,
                             submap_id: int,
                             start_pose: dict,
                             end_pose: dict) -> bool:
        
        # Process submap
        pcd = self.process_submap(points, submap_id)
        
        if len(pcd.points) < 50:
            print(f"Submap {submap_id} has too few points, skipping")
            return False
        
        # First submap: initialize global map
        if submap_id == 0:
            self.global_map = o3d.geometry.PointCloud(pcd)
            self.transforms.append(np.eye(4))
            self.submaps.append({
                'id': submap_id,
                'pcd': pcd,
                'start_pose': start_pose,
                'end_pose': end_pose,
                'global_transform': np.eye(4)
            })
            
            print(f"Submap {submap_id}: Added as first submap (base)")
            return True
        
        # For subsequent submaps, compute initial transform from odometry
        prev_submap = self.submaps[-1]
        
        # Transform from previous submap's end to current submap's start
        odom_transform = self.estimate_2d_transform(
            prev_submap['end_pose'],
            start_pose
        )
        
        # Global transform = previous global transform @ relative transform
        initial_global_transform = prev_submap['global_transform'] @ odom_transform
        
        print(f"Submap {submap_id}: Registering with global map")
        print(f"  Initial transform from odometry: dx={odom_transform[0,3]:.2f}m, dy={odom_transform[1,3]:.2f}m")
        
        # Transform submap to global coordinates
        pcd_transformed = o3d.geometry.PointCloud(pcd)
        pcd_transformed.transform(initial_global_transform)
        
        # ICP registration against global map
        if len(self.global_map.points) > 0:
            success, refined_transform, fitness = self.register_icp_2d(
                source=pcd_transformed,
                target=self.global_map,
                initial_guess=np.eye(4)  # Already in approximate position
            )
            
            if success:
                final_global_transform = refined_transform @ initial_global_transform
                print(f"  ICP refinement successful")
            else:
                final_global_transform = initial_global_transform
                print(f"  WARNING: ICP failed, using odometry estimate")
        else:
            final_global_transform = initial_global_transform
            print(f"  No global map for ICP, using odometry")
        
        # Store submap info
        self.submaps.append({
            'id': submap_id,
            'pcd': pcd,
            'start_pose': start_pose,
            'end_pose': end_pose,
            'global_transform': final_global_transform
        })
        self.transforms.append(final_global_transform)
        
        # Add transformed submap to global map
        pcd_final = o3d.geometry.PointCloud(pcd)
        pcd_final.transform(final_global_transform)
        self.global_map += pcd_final

        # GPU-accelerated downsampling of global map if too large
        if len(self.global_map.points) > 100000:
            # Convert to tensor, downsample on GPU, convert back
            global_map_tensor = self.legacy_to_tensor_pcd(self.global_map)
            global_map_tensor = global_map_tensor.voxel_down_sample(self.voxel_size)
            self.global_map = self.tensor_to_legacy_pcd(global_map_tensor)
            print(f"  Global map downsampled (GPU) to {len(self.global_map.points)} points")
        
        print(f"Submap {submap_id}: Stitched successfully")
        print(f"  Global map now has {len(self.global_map.points)} points\n")
        
        return True
    
    def save_global_map(self, filepath: str):
        """Save the global map with GPU-accelerated final cleanup"""
        if len(self.global_map.points) == 0:
            print("No points in global map")
            return

        # GPU-accelerated final cleanup
        final_map_tensor = self.legacy_to_tensor_pcd(self.global_map)
        final_map_tensor = final_map_tensor.voxel_down_sample(self.voxel_size)
        final_map_tensor, _ = final_map_tensor.remove_statistical_outliers(
            nb_neighbors=20, std_ratio=2.0
        )

        # Convert back to legacy for saving
        final_map = self.tensor_to_legacy_pcd(final_map_tensor)

        o3d.io.write_point_cloud(filepath, final_map)
        print(f"Saved global map (GPU-processed): {filepath} ({len(final_map.points)} points)")
    
    def get_statistics(self) -> dict:
        """Get mapping statistics"""
        return {
            'num_submaps': len(self.submaps),
            'global_map_points': len(self.global_map.points),
            'total_transforms': len(self.transforms)
        }
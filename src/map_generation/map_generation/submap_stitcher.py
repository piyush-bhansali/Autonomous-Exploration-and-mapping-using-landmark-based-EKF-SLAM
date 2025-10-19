#!/usr/bin/env python3
"""
Submap Stitcher - Corrected version
"""

import numpy as np
import open3d as o3d
from typing import Optional, Tuple
from map_generation.utils import quaternion_to_yaw


class SubmapStitcher:
    
    
    def __init__(self,
                 voxel_size: float = 0.05,
                 icp_max_correspondence_dist: float = 0.5,
                 icp_fitness_threshold: float = 0.2):
        
        self.voxel_size = voxel_size
        self.icp_max_dist = icp_max_correspondence_dist
        self.icp_fitness_threshold = icp_fitness_threshold
        
        # Storage
        self.submaps = []
        self.global_map = o3d.geometry.PointCloud()
        self.transforms = []  # Global transform for each submap
    
    def process_submap(self,
                       points: np.ndarray,
                       submap_id: int) -> o3d.geometry.PointCloud:
       
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # Downsample
        pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        
        # Remove outliers
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        
        print(f"Submap {submap_id}: {len(points)} → {len(pcd.points)} points")
        
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
        
        # Ensure 2D by zeroing Z coordinates
        source_2d = o3d.geometry.PointCloud(source)
        target_2d = o3d.geometry.PointCloud(target)
        
        source_points = np.asarray(source_2d.points)
        target_points = np.asarray(target_2d.points)
        source_points[:, 2] = 0
        target_points[:, 2] = 0
        source_2d.points = o3d.utility.Vector3dVector(source_points)
        target_2d.points = o3d.utility.Vector3dVector(target_points)
        
        # Estimate normals
        source_2d.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 2, 
                max_nn=30
            )
        )
        target_2d.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_size * 2, 
                max_nn=30
            )
        )
        
        # Point-to-plane ICP
        reg_result = o3d.pipelines.registration.registration_icp(
            source_2d, target_2d,
            max_correspondence_distance=self.icp_max_dist,
            init=initial_guess,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=100
            )
        )
        
        # Constrain to 2D
        transform = reg_result.transformation.copy()
        transform[2, :] = [0, 0, 1, 0]
        transform[:, 2] = [0, 0, 1, 0]
        
        success = reg_result.fitness >= self.icp_fitness_threshold
        
        print(f"  ICP: fitness={reg_result.fitness:.3f}, rmse={reg_result.inlier_rmse:.3f}, success={success}")
        
        return success, transform, reg_result.fitness
    
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
        
        # Downsample global map if too large
        if len(self.global_map.points) > 100000:
            self.global_map = self.global_map.voxel_down_sample(self.voxel_size)
            print(f"  Global map downsampled to {len(self.global_map.points)} points")
        
        print(f"Submap {submap_id}: Stitched successfully")
        print(f"  Global map now has {len(self.global_map.points)} points\n")
        
        return True
    
    def save_global_map(self, filepath: str):
        """Save the global map"""
        if len(self.global_map.points) == 0:
            print("No points in global map")
            return
        
        # Final cleanup
        final_map = self.global_map.voxel_down_sample(self.voxel_size)
        final_map, _ = final_map.remove_statistical_outlier(
            nb_neighbors=20, std_ratio=2.0
        )
        
        o3d.io.write_point_cloud(filepath, final_map)
        print(f"Saved global map: {filepath} ({len(final_map.points)} points)")
    
    def get_statistics(self) -> dict:
        """Get mapping statistics"""
        return {
            'num_submaps': len(self.submaps),
            'global_map_points': len(self.global_map.points),
            'total_transforms': len(self.transforms)
        }
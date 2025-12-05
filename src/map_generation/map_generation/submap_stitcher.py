#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import open3d.core as o3c
from typing import Optional, Tuple, Dict, List
import time
import os
from map_generation.feature_extractor import FeatureExtractor
from map_generation.loop_closure_detector import LoopClosureDetector
from map_generation.gtsam_optimizer import GTSAMOptimizer


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
            self.gtsam_optimizer = GTSAMOptimizer(
                translation_noise_sigma=0.1,  # 10cm std dev for loop closure
                rotation_noise_sigma=0.1       # ~5.7 degrees std dev
            )
        else:
            self.loop_closure_detector = None
            self.gtsam_optimizer = None

    def _pose_to_matrix(self, pose: dict) -> np.ndarray:
        """
        Convert pose dictionary to 4×4 homogeneous transformation matrix

        Args:
            pose: Dictionary with keys 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'

        Returns:
            4×4 transformation matrix
        """
        from scipy.spatial.transform import Rotation

        # Extract position
        x = pose['x']
        y = pose['y']
        z = pose.get('z', 0.0)

        # Extract quaternion (x, y, z, w)
        qx = pose['qx']
        qy = pose['qy']
        qz = pose['qz']
        qw = pose['qw']

        # Convert quaternion to rotation matrix
        rotation = Rotation.from_quat([qx, qy, qz, qw])
        rotation_matrix = rotation.as_matrix()

        # Build 4×4 homogeneous transform
        transform = np.eye(4)
        transform[0:3, 0:3] = rotation_matrix
        transform[0:3, 3] = [x, y, z]

        return transform

    def _transform_points_to_local(self, points_world: np.ndarray, submap_origin_pose: dict) -> np.ndarray:
        """
        Transform points from world frame to submap-local frame

        The submap-local frame has its origin at submap_origin_pose (pose_start).
        This allows all points in the submap to be expressed relative to where
        the submap began, making the entire submap transformable as a rigid body.

        Args:
            points_world: N×3 array of points in world frame (tb3_1/odom)
            submap_origin_pose: Pose dict for the submap origin (pose_start)

        Returns:
            N×3 array of points in submap-local frame
        """
        # _pose_to_matrix gives us T_local_to_world (transforms points FROM local TO world)
        # We need T_world_to_local, which is the inverse
        T_local_to_world = self._pose_to_matrix(submap_origin_pose)
        T_world_to_local = np.linalg.inv(T_local_to_world)

        # Transform points: world → submap-local
        points_homogeneous = np.column_stack([points_world, np.ones(len(points_world))])
        points_local_homogeneous = (T_world_to_local @ points_homogeneous.T).T
        points_local = points_local_homogeneous[:, 0:3]

        return points_local

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
        """
        Integrate submap into global map using submap-local coordinate frames

        Points come in world frame (tb3_1/odom). We transform them to submap-local
        frame (relative to pose_start), then store global_transform as the absolute
        pose of pose_start. This allows GTSAM to optimize submap poses as rigid bodies.

        Args:
            points: N×3 array in world frame (accumulated from scan-to-map ICP)
            submap_id: Unique submap identifier
            start_pose: Robot pose at submap start (world frame)
            end_pose: Robot pose at submap end (world frame)
            scan_count: Number of scans in this submap

        Returns:
            (success, pose_correction)
        """
        # Step 1: Transform points from world → submap-local frame
        points_local = self._transform_points_to_local(points, start_pose)

        # DEBUG: Log transformation info
        print(f"\n{'='*60}")
        print(f"SUBMAP {submap_id} INTEGRATION DEBUG")
        print(f"{'='*60}")
        print(f"Start pose: x={start_pose['x']:.3f}, y={start_pose['y']:.3f}, "
              f"qz={start_pose['qz']:.3f}, qw={start_pose['qw']:.3f}")
        print(f"Points in world frame: {len(points)} points")
        print(f"  World frame center: {np.mean(points, axis=0)}")
        print(f"  World frame extent: min={np.min(points, axis=0)}, max={np.max(points, axis=0)}")
        print(f"Points in local frame: {len(points_local)} points")
        print(f"  Local frame center: {np.mean(points_local, axis=0)}")
        print(f"  Local frame extent: min={np.min(points_local, axis=0)}, max={np.max(points_local, axis=0)}")

        # Step 2: Process submap (downsample) in local frame
        pcd_tensor = self.process_submap(points_local, submap_id)

        if len(pcd_tensor.point.positions) < 50:
            print(f"SKIPPED: Too few points after downsampling ({len(pcd_tensor.point.positions)})")
            return False, None

        # Step 3: Calculate pose_center for spatial search (still needed for loop closure)
        pose_center = {
            'x': (start_pose['x'] + end_pose['x']) / 2,
            'y': (start_pose['y'] + end_pose['y']) / 2,
            'z': 0.0,
            'qx': end_pose['qx'],
            'qy': end_pose['qy'],
            'qz': end_pose['qz'],
            'qw': end_pose['qw']
        }

        # Step 4: Extract features from local-frame points
        features = None
        if self.enable_loop_closure:
            pcd_legacy = pcd_tensor.to_legacy()
            features = self.extract_features(pcd_legacy, submap_id)

        # Step 5: Calculate global_transform as absolute pose of pose_start
        global_transform = self._pose_to_matrix(start_pose)

        print(f"\nGlobal transform matrix (pose_start → world):")
        print(f"{global_transform}")
        print(f"Translation: [{global_transform[0,3]:.3f}, {global_transform[1,3]:.3f}, {global_transform[2,3]:.3f}]")
        print(f"Rotation (yaw): {np.arctan2(global_transform[1,0], global_transform[0,0]):.3f} rad")

        if submap_id == 0:
            # First submap: Store directly without ICP alignment
            # Transform submap to world frame for global map
            transform_tensor = o3c.Tensor(global_transform, dtype=o3c.float32, device=self.device)
            pcd_world_tensor = pcd_tensor.transform(transform_tensor)
            self.global_map_tensor = pcd_world_tensor

            # Memory optimization: Only store point cloud if loop closure is disabled
            # For loop closure, we only need features after extraction
            if self.enable_loop_closure:
                # Store minimal point cloud (decimated) for ICP verification only
                pcd_decimated = pcd_tensor.to_legacy().voxel_down_sample(self.voxel_size * 3.0)
                stored_pcd = pcd_decimated
            else:
                stored_pcd = pcd_tensor.to_legacy()

            submap_data = {
                'id': submap_id,
                'point_cloud': stored_pcd,  # In submap-local frame
                'features': features,
                'pose_start': start_pose,
                'pose_end': end_pose,
                'pose_center': pose_center,
                'global_transform': global_transform,  # Absolute pose of pose_start
                'scan_count': scan_count,
                'timestamp_created': time.time()
            }

            self.submaps.append(submap_data)
            self.transforms.append(global_transform)

            # Add to loop closure spatial index
            if self.loop_closure_detector is not None:
                self.loop_closure_detector.add_submap_to_index(
                    submap_id,
                    np.array([pose_center['x'], pose_center['y']])
                )

            return True, None  # No correction for first submap

        # Step 6: ICP alignment for submap-to-map refinement
        # We do ICP on LOCAL frame points with global_transform as initial guess
        # This way ICP directly gives us the refined global_transform
        print(f"\nICP Alignment:")
        print(f"  Source (submap in LOCAL frame): {len(pcd_tensor.point.positions)} points")
        print(f"  Target (global map): {len(self.global_map_tensor.point.positions)} points")

        # ICP with global_transform as initial guess
        # ICP will transform source by initial_guess, then find refinement
        success, global_transform_refined, fitness = self.align_submap_with_icp(
            source=pcd_tensor,
            target=self.global_map_tensor,
            initial_guess=global_transform  # Use odometry estimate as starting point
        )

        print(f"  ICP Success: {success}, Fitness: {fitness:.4f}")
        print(f"  ICP refined global transform:")
        print(f"{global_transform_refined}")

        print(f"\nRefined global transform (after ICP):")
        print(f"  Translation: [{global_transform_refined[0,3]:.3f}, {global_transform_refined[1,3]:.3f}, {global_transform_refined[2,3]:.3f}]")
        print(f"  Rotation (yaw): {np.arctan2(global_transform_refined[1,0], global_transform_refined[0,0]):.3f} rad")

        # Calculate ICP correction (difference from odometry-based placement)
        icp_correction = np.linalg.inv(global_transform) @ global_transform_refined
        correction_translation = np.linalg.norm(icp_correction[:2, 3])
        correction_rotation = np.abs(np.arctan2(icp_correction[1, 0], icp_correction[0, 0]))

        print(f"\nICP Correction:")
        print(f"  Translation: {correction_translation:.4f} m")
        print(f"  Rotation: {np.degrees(correction_rotation):.2f} deg")

        # Relaxed thresholds to allow larger corrections for accumulated drift
        if correction_translation > 1.0 or correction_rotation > np.radians(20):
            # ICP correction too large - reject and use odometry
            global_transform_refined = global_transform
            success = False

        if not success or fitness < self.icp_fitness_threshold:
            # ICP failed - use odometry
            global_transform_refined = global_transform

        # Extract pose correction for robot pose update (relative to odometry)
        pose_correction = None
        if success and correction_translation > 0.01:  # Only if non-trivial correction (>1cm)
            pose_correction = {
                'dx': icp_correction[0, 3],
                'dy': icp_correction[1, 3],
                'dtheta': np.arctan2(icp_correction[1, 0], icp_correction[0, 0])
            }

        # Transform local-frame submap to world frame using refined transform
        transform_refined_tensor = o3c.Tensor(global_transform_refined, dtype=o3c.float32, device=self.device)
        pcd_aligned_tensor = pcd_tensor.transform(transform_refined_tensor)

        # Memory optimization: Store local-frame point cloud (decimated)
        # We store in local frame so GTSAM can re-transform with optimized poses
        if self.enable_loop_closure:
            pcd_decimated = pcd_tensor.to_legacy().voxel_down_sample(self.voxel_size * 3.0)
            stored_pcd = pcd_decimated
        else:
            stored_pcd = pcd_tensor.to_legacy()

        submap_data = {
            'id': submap_id,
            'point_cloud': stored_pcd,  # In submap-local frame
            'features': features,
            'pose_start': start_pose,
            'pose_end': end_pose,
            'pose_center': pose_center,
            'global_transform': global_transform_refined,  # Absolute pose of pose_start (refined by ICP)
            'scan_count': scan_count,
            'timestamp_created': time.time()
        }

        self.submaps.append(submap_data)
        self.transforms.append(global_transform_refined)

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

                    # Use GTSAM for pose graph optimization
                    optimized_transforms = self.gtsam_optimizer.optimize_pose_graph(
                        submaps=self.submaps,
                        loop_closure=loop_closure
                    )

                    if optimized_transforms is not None:
                        # Apply optimized transforms to submaps
                        for i, optimized_transform in enumerate(optimized_transforms):
                            self.submaps[i]['global_transform'] = optimized_transform
                            self.transforms[i] = optimized_transform

                        # Rebuild global map with optimized poses
                        self._rebuild_global_map()

                        # Print optimization statistics
                        stats = self.gtsam_optimizer.get_statistics()
                        if stats['last_error'] is not None:
                            error_reduction = stats['last_error']['improvement']
                            print(f"    GTSAM optimization: error reduced by {error_reduction:.4f}")

        self.global_map_tensor.point.positions = o3c.concatenate([
            self.global_map_tensor.point.positions,
            pcd_aligned_tensor.point.positions
        ], axis=0)

        self.global_map_tensor = self.global_map_tensor.voxel_down_sample(self.voxel_size)

        # DISABLED: Aggressive statistical outlier removal was progressively deleting sparse wall points
        # Walls are naturally sparse (2D surfaces) so they get flagged as "outliers" after many cycles
        # self.global_map_tensor, _ = self.global_map_tensor.remove_statistical_outliers(nb_neighbors=10, std_ratio=3.0)

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

    # REMOVED: Old linear optimization replaced by GTSAM
    # The _optimize_pose_graph_with_loop_closure method has been replaced
    # with GTSAMOptimizer.optimize_pose_graph() which provides proper
    # pose graph optimization using factor graphs

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

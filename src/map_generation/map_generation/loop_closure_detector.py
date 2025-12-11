#!/usr/bin/env python3


import numpy as np
import open3d as o3d
import open3d.core as o3c
from typing import List, Optional, Tuple, Dict
from scipy.spatial import KDTree

# Import shared mapping utilities
from map_generation.mapping_utils import (
    match_scan_context_cosine,
    match_scan_context_with_voting,
    match_geometric_features,
    is_distinctive_submap
)


class LoopClosureDetector: 
    

    def __init__(self,
                 spatial_search_radius: float = 5.0,
                 scan_context_threshold: float = 0.7,
                 min_feature_matches: int = 15,
                 ransac_threshold: float = 0.1,
                 icp_fitness_threshold: float = 0.3):
        
        self.spatial_search_radius = spatial_search_radius
        self.scan_context_threshold = scan_context_threshold
        self.min_feature_matches = min_feature_matches
        self.ransac_threshold = ransac_threshold
        self.icp_fitness_threshold = icp_fitness_threshold

        # Spatial index for fast submap lookup
        self.submap_positions = []  # List of [x, y] positions
        self.spatial_index = None    # KDTree for spatial queries

        # Statistics
        self.stats = {
            'num_checks': 0,
            'num_candidates': 0,
            'num_verified': 0,
            'num_accepted': 0
        }

    def add_submap_to_index(self, submap_id: int, position: np.ndarray):
        
        self.submap_positions.append({
            'id': submap_id,
            'position': position
        })

        # Rebuild spatial index
        positions = np.array([s['position'] for s in self.submap_positions])
        self.spatial_index = KDTree(positions)

    def detect_loop_closure(self,
                           current_submap: Dict,
                           submap_database: List[Dict],
                           min_time_separation: float = 30.0) -> Optional[Dict]:

        self.stats['num_checks'] += 1

        current_id = current_submap['id']
        if current_submap['features'] is not None and 'geometric' in current_submap['features']:
            geom_descriptors = current_submap['features']['geometric']
            if len(geom_descriptors) > 0:
                is_distinctive, metrics = is_distinctive_submap(geom_descriptors)

                if not is_distinctive:
                    # Skip loop closure for corridors/featureless areas
                    print(f"  [Loop Closure] Submap {current_id} skipped: {metrics['reason']}")
                    return None

        # Stage 1: Spatial + Scan Context Pre-filtering
        candidates = self._stage1_coarse_matching(
            current_submap,
            submap_database,
            min_time_separation
        )

        if len(candidates) == 0:
            print(f"  [Loop Closure] Submap {current_id}: No candidates found in Stage 1 (Scan Context)")
            return None

        self.stats['num_candidates'] += len(candidates)
        print(f"  [Loop Closure] Submap {current_id}: Found {len(candidates)} candidates from Stage 1")

        # Stage 2: Geometric Feature Verification
        loop_closure = self._stage2_fine_verification(
            current_submap,
            candidates
        )

        if loop_closure is not None:
            self.stats['num_accepted'] += 1
        else:
            print(f"  [Loop Closure] Submap {current_id}: No valid loop closure after Stage 2 (Geometric+ICP)")

        return loop_closure

    def _stage1_coarse_matching(self,
                                current_submap: Dict,
                                submap_database: List[Dict],
                                min_time_separation: float) -> List[Dict]:
        
        candidates = []

        # Get current Scan Context
        if current_submap['features'] is None:
            return candidates

        current_features = current_submap['features']

        # Check if hybrid features available
        if 'scan_context' not in current_features:
            return candidates

        sc_current = current_features['scan_context']
        current_pos = np.array([current_submap['pose_center']['x'],
                               current_submap['pose_center']['y']])
        current_time = current_submap['timestamp_created']

        # Spatial filtering first
        if self.spatial_index is not None and len(self.submap_positions) > 0:
            # Find submaps within radius
            positions = np.array([s['position'] for s in self.submap_positions])
            distances = np.linalg.norm(positions - current_pos, axis=1)
            spatial_candidates = np.where(distances < self.spatial_search_radius)[0]
        else:
            spatial_candidates = range(len(submap_database))

        # Scan Context matching on spatially filtered candidates
        for idx in spatial_candidates:
            if idx >= len(submap_database):
                continue

            candidate = submap_database[idx]

            # Time separation check
            time_diff = current_time - candidate['timestamp_created']
            if time_diff < min_time_separation:
                continue

            # Check if candidate is distinctive (skip corridors)
            if candidate['features'] is not None and 'geometric' in candidate['features']:
                geom_candidate = candidate['features']['geometric']
                if len(geom_candidate) > 0:
                    is_candidate_distinctive, _ = is_distinctive_submap(geom_candidate)
                    if not is_candidate_distinctive:
                        continue  # Skip non-distinctive candidates

            # Get candidate Scan Context
            if candidate['features'] is None or 'scan_context' not in candidate['features']:
                continue

            sc_candidate = candidate['features']['scan_context']

            # Cosine similarity Scan Context matching (rotation-invariant)
            similarity, best_rotation = match_scan_context_cosine(
                sc_current, sc_candidate
            )

            # Majority voting check (additional robustness)
            agreement_ratio, voting_rotation, _ = match_scan_context_with_voting(
                sc_current, sc_candidate
            )

            # Accept if both scale-invariant similarity AND majority voting pass
            if similarity > 0.65 and agreement_ratio > 0.5:
                candidates.append({
                    'submap': candidate,
                    'similarity': similarity,
                    'agreement_ratio': agreement_ratio,
                    'estimated_rotation_sectors': best_rotation
                })

        # Sort by combined score (similarity + agreement)
        candidates.sort(key=lambda x: 0.6 * x['similarity'] + 0.4 * x['agreement_ratio'], reverse=True)

        return candidates[:5]  # Return top 5 candidates

    def _stage2_fine_verification(self,
                                  current_submap: Dict,
                                  candidates: List[Dict]) -> Optional[Dict]:
        
        current_features = current_submap['features']

        # Check if geometric features available
        if 'geometric' not in current_features or len(current_features['geometric']) == 0:
            return None

        geom_current = current_features['geometric']

        # Try each candidate in order of similarity
        for i, candidate in enumerate(candidates):
            candidate_submap = candidate['submap']

            # Get candidate geometric features
            if candidate_submap['features'] is None:
                continue

            candidate_features = candidate_submap['features']

            if 'geometric' not in candidate_features or len(candidate_features['geometric']) == 0:
                continue

            geom_candidate = candidate_features['geometric']

            # Match geometric features
            matches = match_geometric_features(geom_current, geom_candidate)

            if len(matches) < self.min_feature_matches:
                continue

            # Get keypoint correspondences
            current_keypoint_indices = current_features['keypoint_indices']
            candidate_keypoint_indices = candidate_features['keypoint_indices']

            # Access Tensor PointCloud positions
            current_points = current_submap['point_cloud'].point.positions.cpu().numpy()
            candidate_points = candidate_submap['point_cloud'].point.positions.cpu().numpy()

            # Extract matched keypoint positions
            source_pts = []
            target_pts = []

            for match in matches:
                idx_curr = int(match[0])
                idx_cand = int(match[1])

                if idx_curr < len(current_keypoint_indices) and idx_cand < len(candidate_keypoint_indices):
                    src_pt_idx = current_keypoint_indices[idx_curr]
                    tgt_pt_idx = candidate_keypoint_indices[idx_cand]

                    if src_pt_idx < len(current_points) and tgt_pt_idx < len(candidate_points):
                        source_pts.append(current_points[src_pt_idx])
                        target_pts.append(candidate_points[tgt_pt_idx])

            if len(source_pts) < self.min_feature_matches:
                continue

            source_pts = np.array(source_pts)
            target_pts = np.array(target_pts)

            # RANSAC: Estimate transformation and filter outliers
            transform, inliers = self._ransac_2d(source_pts, target_pts)

            num_inliers = np.sum(inliers)
            inlier_ratio = num_inliers / len(inliers)

            if num_inliers < self.min_feature_matches:
                continue

            # ICP: Refine transformation
            final_transform, fitness = self._refine_with_icp(
                current_submap['point_cloud'],
                candidate_submap['point_cloud'],
                transform
            )

            if fitness >= self.icp_fitness_threshold:
                self.stats['num_verified'] += 1

                return {
                    'current_id': current_submap['id'],
                    'match_id': candidate_submap['id'],
                    'transform': final_transform,
                    'fitness': fitness,
                    'num_inliers': int(num_inliers),
                    'scan_context_similarity': candidate['similarity'],
                    'method': 'two_stage_scan_context_geometric_ransac_icp'
                }

        return None

    def _ransac_2d(self, source_pts: np.ndarray, target_pts: np.ndarray,
                   max_iterations: int = 1000) -> Tuple[np.ndarray, np.ndarray]:
        """
        RANSAC for 2D rigid transformation estimation

        Returns:
            (best_transform, inlier_mask)
        """
        n_points = len(source_pts)
        best_transform = np.eye(4)
        best_inliers = np.zeros(n_points, dtype=bool)
        best_inlier_count = 0

        for _ in range(max_iterations):
            # Sample 3 random point pairs (minimum for 2D rigid transform)
            if n_points < 3:
                break

            indices = np.random.choice(n_points, 3, replace=False)

            # Estimate transform from 3 pairs
            T = self._estimate_2d_transform(
                source_pts[indices],
                target_pts[indices]
            )

            # Apply transform to all source points
            source_homogeneous = np.column_stack([source_pts, np.ones(n_points)])
            source_transformed = (T @ source_homogeneous.T).T[:, :3]

            # Compute distances to target points
            distances = np.linalg.norm(source_transformed - target_pts, axis=1)

            # Count inliers
            inliers = distances < self.ransac_threshold
            inlier_count = np.sum(inliers)

            # Keep best
            if inlier_count > best_inlier_count:
                best_inlier_count = inlier_count
                best_inliers = inliers
                best_transform = T

        # Refine using all inliers
        if best_inlier_count >= 3:
            best_transform = self._estimate_2d_transform(
                source_pts[best_inliers],
                target_pts[best_inliers]
            )

        return best_transform, best_inliers

    def _estimate_2d_transform(self, source: np.ndarray, target: np.ndarray) -> np.ndarray:
        """
        Estimate 2D rigid transformation from point correspondences

        Uses SVD to find optimal rotation and translation
        """
        # Center the points
        source_center = np.mean(source[:, :2], axis=0)
        target_center = np.mean(target[:, :2], axis=0)

        source_centered = source[:, :2] - source_center
        target_centered = target[:, :2] - target_center

        # Compute cross-covariance matrix
        H = source_centered.T @ target_centered

        # SVD
        U, _, Vt = np.linalg.svd(H)
        R_2d = Vt.T @ U.T

        # Ensure proper rotation (det = 1)
        if np.linalg.det(R_2d) < 0:
            Vt[-1, :] *= -1
            R_2d = Vt.T @ U.T

        # Compute translation
        t_2d = target_center - R_2d @ source_center

        # Build 4×4 homogeneous transform
        T = np.eye(4)
        T[0:2, 0:2] = R_2d
        T[0:2, 3] = t_2d

        return T

    def _refine_with_icp(self, source_pcd: o3d.t.geometry.PointCloud,
                        target_pcd: o3d.t.geometry.PointCloud,
                        initial_transform: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Refine transformation using GPU-accelerated Tensor ICP

        Returns:
            (refined_transform, fitness)
        """
        # Convert initial transform to Tensor (use device from source point cloud)
        device = source_pcd.device
        init_transform_tensor = o3c.Tensor(initial_transform, dtype=o3c.float32, device=device)

        # Use Tensor ICP (GPU-accelerated)
        reg_result = o3d.t.pipelines.registration.icp(
            source=source_pcd,
            target=target_pcd,
            max_correspondence_distance=0.5,
            init_source_to_target=init_transform_tensor,
            estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
        )

        # Convert result back to NumPy
        transform = reg_result.transformation.cpu().numpy()
        fitness = float(reg_result.fitness)

        return transform, fitness

    def get_statistics(self) -> dict:
        """Get loop closure detection statistics"""
        stats = self.stats.copy()

        if stats['num_checks'] > 0:
            stats['avg_candidates_per_check'] = stats['num_candidates'] / stats['num_checks']
            stats['acceptance_rate'] = stats['num_accepted'] / stats['num_checks']

        return stats

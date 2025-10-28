#!/usr/bin/env python3


import numpy as np
import open3d as o3d
from typing import List, Optional, Tuple, Dict
from scipy.spatial import KDTree


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

        print(f"\n🔍 Two-stage loop closure check for submap {current_submap['id']}...")

        # Stage 1: Spatial + Scan Context Pre-filtering
        candidates = self._stage1_coarse_matching(
            current_submap,
            submap_database,
            min_time_separation
        )

        if len(candidates) == 0:
            print("  Stage 1: No candidates found")
            return None

        self.stats['num_candidates'] += len(candidates)
        print(f"  Stage 1: Found {len(candidates)} candidates from Scan Context")

        # Stage 2: Geometric Feature Verification
        loop_closure = self._stage2_fine_verification(
            current_submap,
            candidates
        )

        if loop_closure is not None:
            self.stats['num_accepted'] += 1
            print(f"  ✓ Loop closure confirmed: submap {current_submap['id']} ↔ submap {loop_closure['match_id']}")

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
            print("  ⚠ No Scan Context features available")
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

            # Get candidate Scan Context
            if candidate['features'] is None or 'scan_context' not in candidate['features']:
                continue

            sc_candidate = candidate['features']['scan_context']

            # Rotation-invariant Scan Context matching
            similarity, best_rotation = self._match_scan_context(sc_current, sc_candidate)

            if similarity > self.scan_context_threshold:
                candidates.append({
                    'submap': candidate,
                    'similarity': similarity,
                    'estimated_rotation_sectors': best_rotation
                })

        # Sort by similarity (best first)
        candidates.sort(key=lambda x: x['similarity'], reverse=True)

        return candidates[:5]  # Return top 5 candidates

    def _stage2_fine_verification(self,
                                  current_submap: Dict,
                                  candidates: List[Dict]) -> Optional[Dict]:
        
        current_features = current_submap['features']

        # Check if geometric features available
        if 'geometric' not in current_features or len(current_features['geometric']) == 0:
            print("  ⚠ No geometric features for verification")
            return None

        geom_current = current_features['geometric']
        keypoints_current = current_features['keypoints']

        # Try each candidate in order of similarity
        for i, candidate in enumerate(candidates):
            candidate_submap = candidate['submap']

            print(f"  Stage 2: Verifying candidate {i+1}/{len(candidates)} "
                  f"(submap {candidate_submap['id']}, similarity={candidate['similarity']:.3f})")

            # Get candidate geometric features
            if candidate_submap['features'] is None:
                continue

            candidate_features = candidate_submap['features']

            if 'geometric' not in candidate_features or len(candidate_features['geometric']) == 0:
                print("    ⚠ Candidate has no geometric features")
                continue

            geom_candidate = candidate_features['geometric']

            # Match geometric features
            matches = self._match_geometric_features(geom_current, geom_candidate)

            if len(matches) < self.min_feature_matches:
                print(f"    ✗ Too few matches: {len(matches)} < {self.min_feature_matches}")
                continue

            print(f"    ✓ Found {len(matches)} geometric feature matches")

            # Get keypoint correspondences
            current_keypoint_indices = current_features['keypoint_indices']
            candidate_keypoint_indices = candidate_features['keypoint_indices']

            current_points = np.asarray(current_submap['point_cloud'].points)
            candidate_points = np.asarray(candidate_submap['point_cloud'].points)

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
                print(f"    ✗ Too few valid correspondences: {len(source_pts)}")
                continue

            source_pts = np.array(source_pts)
            target_pts = np.array(target_pts)

            # RANSAC: Estimate transformation and filter outliers
            transform, inliers = self._ransac_2d(source_pts, target_pts)

            num_inliers = np.sum(inliers)
            inlier_ratio = num_inliers / len(inliers)

            print(f"    RANSAC: {num_inliers}/{len(inliers)} inliers ({inlier_ratio*100:.1f}%)")

            if num_inliers < self.min_feature_matches:
                print(f"    ✗ Too few RANSAC inliers")
                continue

            # ICP: Refine transformation
            final_transform, fitness = self._refine_with_icp(
                current_submap['point_cloud'],
                candidate_submap['point_cloud'],
                transform
            )

            print(f"    ICP: fitness={fitness:.3f}")

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
            else:
                print(f"    ✗ ICP fitness too low: {fitness:.3f} < {self.icp_fitness_threshold}")

        return None

    def _match_scan_context(self, sc1: np.ndarray, sc2: np.ndarray) -> Tuple[float, int]:
        
        # Reshape to 2D grid (rings × sectors)
        num_sectors = 60  # Default from feature extractor

        if sc1.shape[0] == 1:
            sc1 = sc1.reshape(-1, num_sectors)
        if sc2.shape[0] == 1:
            sc2 = sc2.reshape(-1, num_sectors)

        best_sim = -1
        best_shift = 0

        # Try all circular shifts
        for shift in range(num_sectors):
            sc2_shifted = np.roll(sc2, shift, axis=1)

            # Cosine similarity
            sc1_flat = sc1.flatten()
            sc2_flat = sc2_shifted.flatten()

            norm1 = np.linalg.norm(sc1_flat)
            norm2 = np.linalg.norm(sc2_flat)

            if norm1 > 0 and norm2 > 0:
                similarity = np.dot(sc1_flat, sc2_flat) / (norm1 * norm2)

                if similarity > best_sim:
                    best_sim = similarity
                    best_shift = shift

        return best_sim, best_shift

    def _match_geometric_features(self, desc1: np.ndarray, desc2: np.ndarray) -> np.ndarray:
        
        if len(desc1) == 0 or len(desc2) == 0:
            return np.array([])

        # Build KD-tree for descriptor2
        tree = KDTree(desc2)

        # For each descriptor in set1, find nearest in set2
        distances, indices = tree.query(desc1)

        # Filter by distance threshold (0.75 for 4D geometric features)
        valid = distances < 0.75

        matches = np.column_stack([
            np.arange(len(desc1))[valid],
            indices[valid],
            distances[valid]
        ])

        return matches

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

    def _refine_with_icp(self, source_pcd: o3d.geometry.PointCloud,
                        target_pcd: o3d.geometry.PointCloud,
                        initial_transform: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Refine transformation using ICP

        Returns:
            (refined_transform, fitness)
        """
        # Use Open3D's ICP
        reg_result = o3d.pipelines.registration.registration_icp(
            source_pcd,
            target_pcd,
            max_correspondence_distance=0.5,
            init=initial_transform,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
        )

        return reg_result.transformation, reg_result.fitness

    def get_statistics(self) -> dict:
        """Get loop closure detection statistics"""
        stats = self.stats.copy()

        if stats['num_checks'] > 0:
            stats['avg_candidates_per_check'] = stats['num_candidates'] / stats['num_checks']
            stats['acceptance_rate'] = stats['num_accepted'] / stats['num_checks']

        return stats

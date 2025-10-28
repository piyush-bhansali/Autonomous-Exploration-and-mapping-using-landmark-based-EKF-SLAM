#!/usr/bin/env python3

import numpy as np
import open3d as o3d
from typing import Dict, Tuple


class FeatureExtractor:
    
    def __init__(self,
                 method: str = 'hybrid',
                 voxel_size: float = 0.05):
        
        self.method = method
        self.voxel_size = voxel_size

        # Scan Context parameters
        self.sc_params = {
            'max_range': 6.0,       # Max range for Scan Context (meters)
            'num_rings': 20,        # Radial bins
            'num_sectors': 60,      # Angular bins
        }

        # Geometric feature parameters
        self.geom_params = {
            'neighbor_radius': voxel_size * 5,
            'min_neighbors': 5
        }

    def extract(self, point_cloud: o3d.geometry.PointCloud) -> Dict:
        
        import time
        start_time = time.time()

        # Downsample
        pcd_down = point_cloud.voxel_down_sample(self.voxel_size)

        # Extract features based on method
        if self.method == 'hybrid':
            # Extract BOTH for two-stage loop closure
            sc_desc, sc_meta = self._extract_scan_context(pcd_down)
            keypoints, keypoint_indices = self._extract_keypoints_uniform(pcd_down)
            if len(keypoints.points) > 0:
                geom_desc, geom_meta = self._extract_geometric(pcd_down, keypoint_indices)
            else:
                geom_desc = np.array([])
                geom_meta = {'num_keypoints': 0, 'descriptor_dim': 0}

            elapsed = time.time() - start_time

            return {
                'method': 'hybrid',
                'scan_context': sc_desc,
                'geometric': geom_desc,
                'keypoints': keypoints,
                'keypoint_indices': keypoint_indices,
                'metadata': {
                    'scan_context': sc_meta,
                    'geometric': geom_meta,
                    'computation_time': elapsed
                }
            }

        elif self.method == 'scan_context':
            descriptors, metadata = self._extract_scan_context(pcd_down)
            keypoints = o3d.geometry.PointCloud()
            keypoint_indices = np.array([])

            elapsed = time.time() - start_time
            metadata['computation_time'] = elapsed

            return {
                'method': self.method,
                'keypoints': keypoints,
                'keypoint_indices': keypoint_indices,
                'descriptors': descriptors,
                'metadata': metadata
            }

        elif self.method == 'geometric':
            keypoints, keypoint_indices = self._extract_keypoints_uniform(pcd_down)
            if len(keypoints.points) == 0:
                print("⚠ No keypoints found!")
                return self._empty_result()
            descriptors, metadata = self._extract_geometric(pcd_down, keypoint_indices)

            elapsed = time.time() - start_time
            metadata['computation_time'] = elapsed

            return {
                'method': self.method,
                'keypoints': keypoints,
                'keypoint_indices': keypoint_indices,
                'descriptors': descriptors,
                'metadata': metadata
            }

        else:
            raise ValueError(f"Unknown feature method: {self.method}. Use 'hybrid', 'scan_context' or 'geometric'")

    def _extract_scan_context(self, pcd: o3d.geometry.PointCloud) -> Tuple[np.ndarray, Dict]:
        """
        Extract Scan Context descriptor (global 2D descriptor)

        CRITICAL: Re-centers point cloud to submap center before computing polar coordinates.
        This ensures Scan Context describes SHAPE independent of world position.
        """
        points = np.asarray(pcd.points)

        # Convert to 2D (project to XY plane)
        points_2d = points[:, :2]

        # CRITICAL FIX: Compute submap center and re-center point cloud
        # This makes Scan Context translation-invariant (describes shape, not position)
        center = np.mean(points_2d, axis=0)
        points_centered = points_2d - center

        # Polar coordinates FROM SUBMAP CENTER (not world origin!)
        r = np.sqrt(points_centered[:, 0]**2 + points_centered[:, 1]**2)
        theta = np.arctan2(points_centered[:, 1], points_centered[:, 0])

        # Adaptive max_range: Use actual data extent or configured value
        r_99th = np.percentile(r, 99) if len(r) > 0 else 1.0
        max_range_adaptive = max(r_99th * 1.1, 1.0)  # 10% margin, minimum 1m

        # Use larger of adaptive or configured max_range
        max_range = max(max_range_adaptive, self.sc_params['max_range'])

        num_rings = self.sc_params['num_rings']
        num_sectors = self.sc_params['num_sectors']

        scan_context = np.zeros((num_rings, num_sectors))

        points_used = 0
        points_discarded = 0

        for i in range(len(points)):
            if r[i] > max_range:
                points_discarded += 1
                continue

            points_used += 1

            # Bin indices
            ring_idx = int(r[i] / max_range * num_rings)
            sector_idx = int((theta[i] + np.pi) / (2 * np.pi) * num_sectors)

            # Clip to valid range
            ring_idx = min(ring_idx, num_rings - 1)
            sector_idx = min(sector_idx, num_sectors - 1)

            # Occupancy (can also use max height for 3D)
            scan_context[ring_idx, sector_idx] = 1.0

        # Flatten to 1D descriptor
        descriptor = scan_context.flatten()

        # Compute descriptor density (how many bins are occupied)
        occupancy = np.sum(scan_context > 0) / (num_rings * num_sectors)

        metadata = {
            'descriptor_dim': len(descriptor),
            'num_rings': num_rings,
            'num_sectors': num_sectors,
            'max_range_used': float(max_range),
            'center': center.tolist(),
            'points_used': points_used,
            'points_discarded': points_discarded,
            'occupancy': float(occupancy)
        }

        print(f"✓ Scan Context: {num_rings}×{num_sectors} = {len(descriptor)}D, "
              f"range={max_range:.2f}m, center=({center[0]:.2f}, {center[1]:.2f}), "
              f"used={points_used}/{len(points)} ({100*points_used/len(points):.1f}%), "
              f"occupancy={occupancy:.1%}")

        return descriptor.reshape(1, -1), metadata  # Return as (1, D) for consistency

    def _extract_keypoints_uniform(self, pcd: o3d.geometry.PointCloud,
                                   ratio: float = 0.1) -> Tuple[o3d.geometry.PointCloud, np.ndarray]:
        
        num_points = len(pcd.points)
        num_keypoints = max(int(num_points * ratio), 50)  # At least 50 keypoints

        # Uniform sampling
        indices = np.random.choice(num_points, min(num_keypoints, num_points), replace=False)
        indices = np.sort(indices)

        keypoints = pcd.select_by_index(indices.tolist())

        print(f"✓ Uniform sampling: {len(indices)}/{num_points} keypoints ({100*ratio:.1f}%)")

        return keypoints, indices

    def _extract_geometric(self, pcd: o3d.geometry.PointCloud,
                          keypoint_indices: np.ndarray) -> Tuple[np.ndarray, Dict]:
        
        print("Extracting geometric features...")

        points = np.asarray(pcd.points)
        pcd_tree = o3d.geometry.KDTreeFlann(pcd)

        descriptors = []

        for idx in keypoint_indices:
            # Find neighbors
            [k, neighbor_idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[idx], 30)

            if k < self.geom_params['min_neighbors']:
                descriptors.append(np.zeros(4))
                continue

            # Get neighbor points
            neighbor_points = points[neighbor_idx, :]

            # Compute covariance matrix
            cov = np.cov(neighbor_points.T)
            eigenvalues = np.linalg.eigvalsh(cov)
            eigenvalues = np.sort(eigenvalues)[::-1]

            # Geometric features
            lambda1, lambda2, lambda3 = eigenvalues + 1e-10  # Avoid division by zero

            linearity = (lambda1 - lambda2) / lambda1
            planarity = (lambda2 - lambda3) / lambda1
            scattering = lambda3 / lambda1

            # Local point density as curvature estimate
            avg_dist = np.mean(np.linalg.norm(neighbor_points - points[idx], axis=1))

            descriptor = np.array([linearity, planarity, scattering, avg_dist])
            descriptors.append(descriptor)

        descriptors = np.array(descriptors)  # N × 4

        metadata = {
            'num_keypoints': len(descriptors),
            'descriptor_dim': 4
        }

        print(f"✓ Geometric features: {descriptors.shape}")

        return descriptors, metadata

    def _empty_result(self) -> Dict:
        """Return empty result when feature extraction fails"""
        return {
            'method': self.method,
            'keypoints': o3d.geometry.PointCloud(),
            'keypoint_indices': np.array([]),
            'descriptors': np.array([]),
            'metadata': {
                'num_keypoints': 0,
                'descriptor_dim': 0,
                'computation_time': 0.0
            }
        }

    @staticmethod
    def match_scan_context(sc1: np.ndarray, sc2: np.ndarray) -> Tuple[float, int]:
        
        num_sectors = sc1.shape[1]

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

    @staticmethod
    def match_features(descriptors1: np.ndarray,
                      descriptors2: np.ndarray,
                      max_distance: float = 0.75) -> np.ndarray:
        
        from scipy.spatial import KDTree

        if len(descriptors1) == 0 or len(descriptors2) == 0:
            return np.array([])

        # Build KD-tree for descriptor2
        tree = KDTree(descriptors2)

        # For each descriptor in set1, find nearest in set2
        distances, indices = tree.query(descriptors1)

        # Filter by distance threshold
        valid = distances < max_distance

        matches = np.column_stack([
            np.arange(len(descriptors1))[valid],
            indices[valid],
            distances[valid]
        ])

        return matches


def test_feature_extractor():
    """Test feature extraction on sample 2D data"""
    print("=" * 60)
    print("Testing 2D Feature Extractor")
    print("=" * 60)

    # Create sample 2D point cloud (rectangle)
    points_2d = []
    # Bottom edge
    for x in np.linspace(0, 5, 50):
        points_2d.append([x, 0, 0])
    # Right edge
    for y in np.linspace(0, 3, 30):
        points_2d.append([5, y, 0])
    # Top edge
    for x in np.linspace(5, 0, 50):
        points_2d.append([x, 3, 0])
    # Left edge
    for y in np.linspace(3, 0, 30):
        points_2d.append([0, y, 0])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points_2d))

    # Test both methods
    methods = ['scan_context', 'geometric']

    for method in methods:
        print(f"\n--- Testing {method.upper()} ---")
        extractor = FeatureExtractor(method=method)
        result = extractor.extract(pcd)

        print(f"Method: {result['method']}")
        print(f"Descriptor shape: {result['descriptors'].shape}")
        print(f"Descriptor dim: {result['metadata']['descriptor_dim']}")
        print(f"Time: {result['metadata']['computation_time']:.3f}s")

    print("\n" + "=" * 60)
    print("✓ Feature extractor test complete!")


if __name__ == '__main__':
    test_feature_extractor()

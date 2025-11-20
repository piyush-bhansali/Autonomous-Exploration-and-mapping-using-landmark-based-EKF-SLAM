#!/usr/bin/env python3

import numpy as np
import open3d as o3d
from typing import Dict, Tuple

# Import shared mapping utilities
from map_generation.mapping_utils import match_scan_context, match_geometric_features


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
            # Extract only Scan Context
            sc_desc, sc_meta = self._extract_scan_context(pcd_down)
            elapsed = time.time() - start_time

            return {
                'method': 'scan_context',
                'scan_context': sc_desc,
                'descriptors': sc_desc,
                'keypoints': o3d.geometry.PointCloud(),
                'keypoint_indices': np.array([]),
                'metadata': {
                    **sc_meta,
                    'computation_time': elapsed
                }
            }

        elif self.method == 'geometric':
            # Extract only geometric features
            keypoints, keypoint_indices = self._extract_keypoints_uniform(pcd_down)
            if len(keypoints.points) > 0:
                geom_desc, geom_meta = self._extract_geometric(pcd_down, keypoint_indices)
            else:
                geom_desc = np.array([])
                geom_meta = {'num_keypoints': 0, 'descriptor_dim': 0}

            elapsed = time.time() - start_time

            return {
                'method': 'geometric',
                'geometric': geom_desc,
                'descriptors': geom_desc,
                'keypoints': keypoints,
                'keypoint_indices': keypoint_indices,
                'metadata': {
                    **geom_meta,
                    'computation_time': elapsed
                }
            }

        else:
            raise ValueError(f"Unknown feature method: {self.method}. Use 'hybrid', 'scan_context' or 'geometric'")

    def _extract_scan_context(self, pcd: o3d.geometry.PointCloud) -> Tuple[np.ndarray, Dict]:
        
        points = np.asarray(pcd.points)
        points_2d = points[:, :2]
        center = np.mean(points_2d, axis=0)
        points_centered = points_2d - center

        r = np.sqrt(points_centered[:, 0]**2 + points_centered[:, 1]**2)
        theta = np.arctan2(points_centered[:, 1], points_centered[:, 0])

        r_99th = np.percentile(r, 99) if len(r) > 0 else 1.0
        max_range_adaptive = max(r_99th * 1.1, 1.0)  # 10% margin, minimum 1m

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

        return descriptor.reshape(1, -1), metadata

    def _extract_keypoints_uniform(self, pcd: o3d.geometry.PointCloud,
                                   ratio: float = 0.1) -> Tuple[o3d.geometry.PointCloud, np.ndarray]:
        
        num_points = len(pcd.points)
        num_keypoints = max(int(num_points * ratio), 50)  # At least 50 keypoints

        # Uniform sampling
        indices = np.random.choice(num_points, min(num_keypoints, num_points), replace=False)
        indices = np.sort(indices)

        keypoints = pcd.select_by_index(indices.tolist())
        return keypoints, indices

    def _extract_geometric(self, pcd: o3d.geometry.PointCloud,
                          keypoint_indices: np.ndarray) -> Tuple[np.ndarray, Dict]:

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

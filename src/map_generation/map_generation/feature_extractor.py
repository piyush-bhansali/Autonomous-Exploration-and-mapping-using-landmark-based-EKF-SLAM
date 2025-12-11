#!/usr/bin/env python3

import numpy as np
import open3d as o3d
from typing import Dict, Tuple

# Import shared mapping utilities
from map_generation.mapping_utils import match_geometric_features


class FeatureExtractor:
    
    def __init__(self, method: str = 'hybrid'):

        self.method = method

        # Scan Context parameters
        self.sc_params = {
            'max_range': 10.0,      
            'num_rings': 20,        
            'num_sectors': 60,      
        }

        # Geometric feature parameters
        self.geom_params = {
            'min_neighbors': 5
        }

    def extract(self, point_cloud: o3d.t.geometry.PointCloud) -> Dict:
        """
        Extract features from Tensor PointCloud using GPU-accelerated operations.

        Args:
            point_cloud: Tensor PointCloud (GPU or CPU)

        Returns:
            Dictionary containing features and metadata
        """
        import time
        start_time = time.time()

        # All operations now use Tensor API directly (no Legacy conversion!)

        if self.method == 'hybrid':
            sc_desc, sc_meta = self._extract_scan_context(point_cloud)
            keypoint_indices = self._extract_keypoints_uniform(point_cloud)
            if len(keypoint_indices) > 0:
                geom_desc, geom_meta = self._extract_geometric(point_cloud, keypoint_indices)
            else:
                geom_desc = np.array([])
                geom_meta = {'num_keypoints': 0, 'descriptor_dim': 0}

            elapsed = time.time() - start_time

            return {
                'method': 'hybrid',
                'scan_context': sc_desc,
                'geometric': geom_desc,
                'keypoint_indices': keypoint_indices,
                'metadata': {
                    'scan_context': sc_meta,
                    'geometric': geom_meta,
                    'computation_time': elapsed
                }
            }

        elif self.method == 'scan_context':
            sc_desc, sc_meta = self._extract_scan_context(point_cloud)
            elapsed = time.time() - start_time

            return {
                'method': 'scan_context',
                'scan_context': sc_desc,
                'descriptors': sc_desc,
                'keypoint_indices': np.array([]),
                'metadata': {
                    **sc_meta,
                    'computation_time': elapsed
                }
            }

        elif self.method == 'geometric':
            keypoint_indices = self._extract_keypoints_uniform(point_cloud)
            if len(keypoint_indices) > 0:
                geom_desc, geom_meta = self._extract_geometric(point_cloud, keypoint_indices)
            else:
                geom_desc = np.array([])
                geom_meta = {'num_keypoints': 0, 'descriptor_dim': 0}

            elapsed = time.time() - start_time

            return {
                'method': 'geometric',
                'geometric': geom_desc,
                'descriptors': geom_desc,
                'keypoint_indices': keypoint_indices,
                'metadata': {
                    **geom_meta,
                    'computation_time': elapsed
                }
            }

        else:
            raise ValueError(f"Unknown feature method: {self.method}. Use 'hybrid', 'scan_context' or 'geometric'")

    def _extract_scan_context(self, pcd: o3d.t.geometry.PointCloud) -> Tuple[np.ndarray, Dict]:
        """
        Extract Scan Context descriptor from Tensor PointCloud.

        Args:
            pcd: Tensor PointCloud (GPU or CPU)

        Returns:
            (descriptor, metadata) tuple
        """
        # Convert to NumPy once (efficient single transfer)
        points = pcd.point.positions.cpu().numpy()
        points_2d = points[:, :2]
        center = np.mean(points_2d, axis=0)
        points_centered = points_2d - center

        r = np.sqrt(points_centered[:, 0]**2 + points_centered[:, 1]**2)
        theta = np.arctan2(points_centered[:, 1], points_centered[:, 0])

        # Compute actual extent for metadata (debugging/validation)
        r_99th = np.percentile(r, 99) if len(r) > 0 else 1.0

        max_range = self.sc_params['max_range']

        num_rings = self.sc_params['num_rings']
        num_sectors = self.sc_params['num_sectors']

        scan_context = np.zeros((num_rings, num_sectors))

        for i in range(len(points)):
            if r[i] > max_range:
                continue

            # Bin indices
            ring_idx = int(r[i] / max_range * num_rings)
            sector_idx = int((theta[i] + np.pi) / (2 * np.pi) * num_sectors)

            # Clip to valid range
            ring_idx = min(ring_idx, num_rings - 1)
            sector_idx = min(sector_idx, num_sectors - 1)

            # Occupancy (can also use max height for 3D)
            scan_context[ring_idx, sector_idx] = 1.0

        descriptor = scan_context.flatten()

        if r_99th < 1.0:
            pass

        metadata = {
            'num_rings': num_rings,
            'num_sectors': num_sectors
        }

        return descriptor.reshape(1, -1), metadata

    def _extract_keypoints_uniform(self, pcd: o3d.t.geometry.PointCloud,
                                   ratio: float = 0.1) -> np.ndarray:
        """
        Extract keypoint indices using uniform sampling from Tensor PointCloud.

        Args:
            pcd: Tensor PointCloud
            ratio: Ratio of points to sample as keypoints

        Returns:
            Array of keypoint indices
        """
        num_points = len(pcd.point.positions)
        num_keypoints = max(int(num_points * ratio), 50)

        # Uniform sampling
        indices = np.random.choice(num_points, min(num_keypoints, num_points), replace=False)
        indices = np.sort(indices)

        return indices

    def _extract_geometric(self, pcd: o3d.t.geometry.PointCloud,
                          keypoint_indices: np.ndarray) -> Tuple[np.ndarray, Dict]:
        """
        Extract geometric features using GPU-accelerated Tensor NNS.

        Args:
            pcd: Tensor PointCloud (GPU or CPU)
            keypoint_indices: Indices of keypoints in the point cloud

        Returns:
            (descriptors, metadata) tuple
        """
        import open3d.core as o3c

        # Build GPU-accelerated NNS index
        nns = o3d.core.nns.NearestNeighborSearch(pcd.point.positions)
        nns.knn_index()

        # Transfer points to CPU once (more efficient than multiple transfers)
        points = pcd.point.positions.cpu().numpy()

        descriptors = []

        for idx in keypoint_indices:
            # Query on GPU (or CPU if device is CPU)
            query_pt = pcd.point.positions[idx].reshape(1, 3)
            indices, distances = nns.knn_search(query_pt, knn=30)

            # Get neighbor indices
            neighbor_idx = indices[0].cpu().numpy()
            k = len(neighbor_idx)

            if k < self.geom_params['min_neighbors']:
                descriptors.append(np.zeros(4))
                continue

            neighbor_points = points[neighbor_idx, :]

            # Compute covariance and eigenvalues (CPU computation)
            cov = np.cov(neighbor_points.T)
            eigenvalues = np.linalg.eigvalsh(cov)
            eigenvalues = np.sort(eigenvalues)[::-1]

            lambda1, lambda2, lambda3 = eigenvalues + 1e-10

            linearity = (lambda1 - lambda2) / lambda1
            planarity = (lambda2 - lambda3) / lambda1
            scattering = lambda3 / lambda1

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
            'keypoint_indices': np.array([]),
            'descriptors': np.array([]),
            'metadata': {
                'num_keypoints': 0,
                'descriptor_dim': 0,
                'computation_time': 0.0
            }
        }

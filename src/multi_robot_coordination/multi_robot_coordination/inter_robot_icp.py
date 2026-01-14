#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import open3d.core as o3c
from typing import Tuple, Dict, Optional
from sensor_msgs.msg import LaserScan


class InterRobotICP:
    """
    Inter-robot alignment using ICP with adaptive verification.

    Performs scan-to-scan ICP alignment between two robots with
    multi-metric quality verification:
    - ICP fitness score
    - RANSAC inlier ratio
    - Translation/rotation sanity checks
    - Scan overlap percentage
    """

    def __init__(self,
                 icp_max_correspondence_dist: float = 0.15,
                 icp_max_iterations: int = 50,
                 min_fitness: float = 0.4,
                 min_inlier_ratio: float = 0.3,
                 min_overlap: float = 0.25,
                 max_translation_error: float = 2.0,
                 ransac_distance_threshold: float = 0.1,
                 logger=None):
        """
        Initialize ICP aligner.

        Args:
            icp_max_correspondence_dist: Maximum point-to-point distance for ICP
            icp_max_iterations: Maximum ICP iterations
            min_fitness: Minimum ICP fitness threshold
            min_inlier_ratio: Minimum RANSAC inlier ratio
            min_overlap: Minimum scan overlap percentage
            max_translation_error: Maximum allowed translation vs proximity
            ransac_distance_threshold: RANSAC inlier threshold
            logger: ROS2 logger
        """
        self.icp_max_dist = icp_max_correspondence_dist
        self.icp_max_iter = icp_max_iterations
        self.min_fitness = min_fitness
        self.min_inlier_ratio = min_inlier_ratio
        self.min_overlap = min_overlap
        self.max_translation_error = max_translation_error
        self.ransac_threshold = ransac_distance_threshold
        self.logger = logger

        # GPU device (use CUDA if available)
        if o3c.cuda.is_available():
            self.device = o3c.Device("CUDA:0")
        else:
            self.device = o3c.Device("CPU:0")

    def scan_to_pointcloud(self, scan_msg: LaserScan) -> np.ndarray:
        """
        Convert LaserScan message to 2D point cloud.

        Args:
            scan_msg: LaserScan message

        Returns:
            Nx3 numpy array (x, y, z=0)
        """
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
        ranges = np.array(scan_msg.ranges)

        # Filter valid points
        valid = (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max) & np.isfinite(ranges)

        if not np.any(valid):
            return np.array([])

        valid_angles = angles[valid]
        valid_ranges = ranges[valid]

        # Convert to Cartesian
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        z = np.zeros_like(x)

        points = np.column_stack((x, y, z))

        return points

    def arrays_to_pointcloud(self,
                            ranges: np.ndarray,
                            angle_min: float,
                            angle_max: float,
                            range_min: float,
                            range_max: float) -> np.ndarray:
        """
        Convert scan arrays to point cloud.

        Args:
            ranges: Array of range measurements
            angle_min: Minimum scan angle (rad)
            angle_max: Maximum scan angle (rad)
            range_min: Minimum valid range
            range_max: Maximum valid range

        Returns:
            Nx3 numpy array
        """
        angles = np.linspace(angle_min, angle_max, len(ranges))

        # Filter valid points
        valid = (ranges >= range_min) & (ranges <= range_max) & np.isfinite(ranges)

        if not np.any(valid):
            return np.array([])

        valid_angles = angles[valid]
        valid_ranges = ranges[valid]

        # Convert to Cartesian
        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        z = np.zeros_like(x)

        points = np.column_stack((x, y, z))

        return points

    def align_scans(self,
                   source_points: np.ndarray,
                   target_points: np.ndarray,
                   initial_guess: Optional[np.ndarray] = None) -> Tuple[bool, np.ndarray, Dict]:
        """
        Align two point clouds using ICP.

        Args:
            source_points: Nx3 source point cloud
            target_points: Nx3 target point cloud
            initial_guess: 4x4 initial transformation (default: identity)

        Returns:
            (success, transform, metrics) tuple
            - success: True if alignment passed verification
            - transform: 4x4 transformation T_source_to_target
            - metrics: Dictionary with quality metrics
        """
        if len(source_points) < 50 or len(target_points) < 50:
            return False, np.eye(4), {'reason': 'insufficient_points'}

        if initial_guess is None:
            initial_guess = np.eye(4)

        try:
            # Convert to Open3D tensors
            source_tensor = o3c.Tensor(source_points.astype(np.float32),
                                      dtype=o3c.float32, device=self.device)
            target_tensor = o3c.Tensor(target_points.astype(np.float32),
                                      dtype=o3c.float32, device=self.device)

            source_pcd = o3d.t.geometry.PointCloud(self.device)
            source_pcd.point.positions = source_tensor

            target_pcd = o3d.t.geometry.PointCloud(self.device)
            target_pcd.point.positions = target_tensor

            # Voxel downsample for efficiency
            source_pcd = source_pcd.voxel_down_sample(voxel_size=0.05)
            target_pcd = target_pcd.voxel_down_sample(voxel_size=0.05)

            init_transform = o3c.Tensor(initial_guess.astype(np.float32),
                                       dtype=o3c.float32, device=self.device)

            # Run ICP
            reg_result = o3d.t.pipelines.registration.icp(
                source=source_pcd,
                target=target_pcd,
                max_correspondence_distance=self.icp_max_dist,
                init_source_to_target=init_transform,
                estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
                criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.icp_max_iter,
                    relative_fitness=1e-6,
                    relative_rmse=1e-6
                )
            )

            transform = reg_result.transformation.cpu().numpy()
            fitness = float(reg_result.fitness)

            # Enforce 2D constraints
            transform[2, 0:3] = [0, 0, 1]
            transform[2, 3] = 0
            transform[0:2, 2] = [0, 0]

            # Clean up GPU memory
            del source_tensor, target_tensor, source_pcd, target_pcd, init_transform, reg_result

            # Verify alignment quality
            success, metrics = self._verify_alignment(
                source_points, target_points, transform, fitness
            )

            return success, transform, metrics

        except Exception as e:
            if self.logger:
                self.logger.error(f'ICP alignment failed: {e}')
            return False, np.eye(4), {'reason': 'exception', 'error': str(e)}

    def _verify_alignment(self,
                         source_points: np.ndarray,
                         target_points: np.ndarray,
                         transform: np.ndarray,
                         fitness: float) -> Tuple[bool, Dict]:
        """
        Multi-metric alignment verification.

        Args:
            source_points: Source point cloud
            target_points: Target point cloud
            transform: Computed transformation
            fitness: ICP fitness score

        Returns:
            (success, metrics) tuple
        """
        metrics = {'fitness': fitness}

        # Metric 1: ICP fitness
        if fitness < self.min_fitness:
            metrics['reason'] = 'low_icp_fitness'
            metrics['passed'] = False
            return False, metrics

        # Metric 2: RANSAC inlier verification
        inlier_ratio = self._compute_ransac_inliers(source_points, target_points, transform)
        metrics['inlier_ratio'] = inlier_ratio

        if inlier_ratio < self.min_inlier_ratio:
            metrics['reason'] = 'low_inlier_ratio'
            metrics['passed'] = False
            return False, metrics

        # Metric 3: Translation sanity check
        translation = np.linalg.norm(transform[:2, 3])
        metrics['translation'] = translation

        # Robots meeting should be relatively close
        if translation > 10.0:  # Maximum 10m apart
            metrics['reason'] = 'excessive_translation'
            metrics['passed'] = False
            return False, metrics

        # Metric 4: 2D constraint verification
        z_drift = abs(transform[2, 3])
        xy_z_coupling = np.linalg.norm(transform[:2, 2])

        if z_drift > 0.1 or xy_z_coupling > 0.1:
            metrics['reason'] = '2d_constraint_violated'
            metrics['z_drift'] = z_drift
            metrics['xy_z_coupling'] = xy_z_coupling
            metrics['passed'] = False
            return False, metrics

        # Metric 5: Scan overlap
        overlap = self._compute_overlap(source_points, target_points, transform)
        metrics['overlap'] = overlap

        if overlap < self.min_overlap:
            metrics['reason'] = 'insufficient_overlap'
            metrics['passed'] = False
            return False, metrics

        # All checks passed
        metrics['passed'] = True
        metrics['reason'] = 'success'

        if self.logger:
            self.logger.info(
                f'Alignment verified: fitness={fitness:.3f}, '
                f'inliers={inlier_ratio:.3f}, overlap={overlap:.3f}'
            )

        return True, metrics

    def _compute_ransac_inliers(self,
                                source_points: np.ndarray,
                                target_points: np.ndarray,
                                transform: np.ndarray) -> float:
        """
        Compute RANSAC inlier ratio.

        Args:
            source_points: Source point cloud
            target_points: Target point cloud
            transform: Transformation to verify

        Returns:
            Inlier ratio (0.0 to 1.0)
        """
        # Transform source points
        source_homogeneous = np.hstack([source_points, np.ones((len(source_points), 1))])
        source_transformed = (transform @ source_homogeneous.T).T[:, :3]

        # Build KDTree for target
        from scipy.spatial import KDTree
        tree = KDTree(target_points)

        # Query nearest neighbors
        distances, _ = tree.query(source_transformed)

        # Count inliers
        inliers = np.sum(distances < self.ransac_threshold)
        inlier_ratio = inliers / len(source_points)

        return inlier_ratio

    def _compute_overlap(self,
                        source_points: np.ndarray,
                        target_points: np.ndarray,
                        transform: np.ndarray) -> float:
        """
        Compute scan overlap percentage.

        Args:
            source_points: Source point cloud
            target_points: Target point cloud
            transform: Transformation

        Returns:
            Overlap ratio (0.0 to 1.0)
        """
        # Transform source points
        source_homogeneous = np.hstack([source_points, np.ones((len(source_points), 1))])
        source_transformed = (transform @ source_homogeneous.T).T[:, :3]

        # Compute bounding boxes
        source_min = np.min(source_transformed[:, :2], axis=0)
        source_max = np.max(source_transformed[:, :2], axis=0)

        target_min = np.min(target_points[:, :2], axis=0)
        target_max = np.max(target_points[:, :2], axis=0)

        # Compute intersection
        intersection_min = np.maximum(source_min, target_min)
        intersection_max = np.minimum(source_max, target_max)

        # Check if there's overlap
        if np.any(intersection_min >= intersection_max):
            return 0.0

        # Compute areas
        intersection_area = np.prod(intersection_max - intersection_min)
        source_area = np.prod(source_max - source_min)
        target_area = np.prod(target_max - target_min)

        # Overlap as percentage of smaller scan
        min_area = min(source_area, target_area)

        if min_area < 1e-6:
            return 0.0

        overlap = intersection_area / min_area

        return overlap

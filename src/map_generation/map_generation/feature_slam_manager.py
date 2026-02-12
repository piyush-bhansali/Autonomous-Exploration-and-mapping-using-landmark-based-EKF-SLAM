#!/usr/bin/env python3

import numpy as np
from typing import Dict, List, Tuple, Optional
from sensor_msgs.msg import LaserScan

from map_generation.ekf_update_feature import LandmarkEKFSLAM
from map_generation.landmark_features import LandmarkFeatureExtractor
from map_generation.data_association import associate_landmarks
from map_generation.feature_map import FeatureMap


class FeatureSLAMManager:
   

    def __init__(self,
                 landmark_timeout_scans: int = 50,
                 min_observations_for_init: int = 2,
                 min_points_per_line: int = 5,
                 line_fit_threshold: float = 0.03,
                 min_line_length: float = 0.3,
                 corner_angle_threshold: float = 50.0,
                 max_gap: float = 0.2,
                 max_mahalanobis_dist: float = 5.99,
                 max_euclidean_dist: float = 2.0,
                 wall_gap_tolerance: float = 0.5):
       
        # Initialize EKF SLAM
        self.ekf = LandmarkEKFSLAM(
            landmark_timeout_scans=landmark_timeout_scans,
            min_observations_for_init=min_observations_for_init
        )
        self.ekf_initialized = False

        # Initialize feature extractor
        self.feature_extractor = LandmarkFeatureExtractor(
            min_points_per_line=min_points_per_line,
            min_line_length=min_line_length,
            split_residual_threshold=line_fit_threshold,
            merge_residual_threshold=line_fit_threshold,
            merge_angle_threshold_deg=12.0,
            corner_angle_threshold=corner_angle_threshold,
            max_gap=max_gap,
            corner_neighbor_range=6
        )

        # Initialize feature map
        self.feature_map = FeatureMap()

        # Data association parameters
        self.max_mahalanobis_dist = max_mahalanobis_dist
        self.max_euclidean_dist = max_euclidean_dist
        self.wall_gap_tolerance = wall_gap_tolerance

        # Statistics
        self.total_scans_processed = 0

    def initialize_pose(self, x: float, y: float, theta: float):
        
        self.ekf.initialize(x, y, theta)
        self.ekf_initialized = True

    def predict_motion(self, delta_d: float, delta_theta: float):
        
        if not self.ekf_initialized:
            return

        self.ekf.predict_with_relative_motion(delta_d, delta_theta)

    def process_scan(self, scan_msg: LaserScan) -> Dict:
       
        if not self.ekf_initialized:
            return self._empty_stats()

        self.ekf.current_scan_number += 1

        # Extract features from scan (in robot frame)
        observed_features = self.feature_extractor.extract_features(scan_msg)

        if len(observed_features) == 0:
            return self._empty_stats()

        # Data association with wall extension info
        matched, unmatched, extension_info = associate_landmarks(
            observed_features,
            self.ekf,
            self.feature_map,
            max_mahalanobis_dist=self.max_mahalanobis_dist,
            max_euclidean_dist=self.max_euclidean_dist,
            wall_gap_tolerance=self.wall_gap_tolerance,
            return_extension_info=True
        )

        # Process matched features: EKF update + feature map extension
        self._process_matched_features(observed_features, matched, extension_info)

        # Add new landmarks for unmatched features
        self._process_unmatched_features(observed_features, unmatched)

        # Prune landmarks that haven't been seen recently
        self._prune_old_landmarks()

        # Update statistics
        self.total_scans_processed += 1

        # Return processing statistics
        num_walls_stored, num_corners_stored = self.feature_map.get_feature_count()
        num_corners = sum(1 for f in observed_features if f['type'] == 'corner')
        num_walls = sum(1 for f in observed_features if f['type'] == 'wall')

        return {
            'num_features': len(observed_features),
            'num_walls': num_walls,
            'num_corners': num_corners,
            'num_matched': len(matched),
            'num_unmatched': len(unmatched),
            'num_landmarks': len(self.ekf.landmarks),
            'num_walls_stored': num_walls_stored,
            'num_corners_stored': num_corners_stored,
            'observed_features': observed_features
        }

    def _process_matched_features(self, observed_features: List[Dict],
                                   matched: List[Tuple[int, int]],
                                   extension_info: Dict):
      
        for feat_idx, landmark_id in matched:
            feature = observed_features[feat_idx]

            # EKF update
            if feature['type'] == 'wall':
                z_rho = feature['rho']
                z_alpha = feature['alpha']

                self.ekf.update_landmark_observation(
                    landmark_id=landmark_id,
                    z_x=z_rho,
                    z_y=z_alpha,
                    scan_number=self.ekf.current_scan_number,
                    measurement_covariance=feature['covariance']
                )

            elif feature['type'] == 'corner':
                z_x, z_y = feature['position']

                self.ekf.update_landmark_observation(
                    landmark_id=landmark_id,
                    z_x=z_x,
                    z_y=z_y,
                    scan_number=self.ekf.current_scan_number,
                    measurement_covariance=feature['covariance']
                )

                # Transform corner from robot frame to map frame
                corner_map = self._transform_point_to_map(z_x, z_y)

                # Update FeatureMap corner position
                self.feature_map.update_corner_position(
                    landmark_id=landmark_id,
                    new_position=corner_map
                )

            # Wall extension: update FeatureMap endpoints if wall matched
            if feat_idx in extension_info:
                ext = extension_info[feat_idx]
                self.feature_map.update_wall_endpoints(
                    landmark_id=ext['landmark_id'],
                    new_start=ext['new_start'],
                    new_end=ext['new_end'],
                    new_points=ext['new_points']
                )

    def _process_unmatched_features(self, observed_features: List[Dict],
                                     unmatched: List[int]):
        """Add new landmarks for unmatched features."""
        for feat_idx in unmatched:
            feature = observed_features[feat_idx]

            if feature['type'] == 'wall':
                self._add_wall_landmark(feature)

            elif feature['type'] == 'corner':
                self._add_corner_landmark(feature)

    def _add_wall_landmark(self, feature: Dict):
        """Add a new wall landmark to EKF and FeatureMap."""
        # Add to EKF
        z_rho = feature['rho']
        z_alpha = feature['alpha']

        landmark_id = self.ekf.add_landmark(
            z_x=z_rho,
            z_y=z_alpha,
            feature=feature,
            scan_number=self.ekf.current_scan_number
        )

        # Transform wall parameters from robot frame to map frame
        x_r, y_r, theta_r = self.ekf.state[0:3]

        # Robot-frame Hessian to map-frame Hessian
        # First compute alpha_map (angle in map frame)
        alpha_map = z_alpha + theta_r
        alpha_map = np.arctan2(np.sin(alpha_map), np.cos(alpha_map))

        # Then use alpha_map to compute rho_map (not z_alpha!)
        rho_map = z_rho + (x_r * np.cos(alpha_map) + y_r * np.sin(alpha_map))

        # Ensure rho is positive
        if rho_map < 0:
            rho_map = -rho_map
            alpha_map = alpha_map + np.pi
            alpha_map = np.arctan2(np.sin(alpha_map), np.cos(alpha_map))

        # Transform endpoints from robot frame to map frame
        c = np.cos(theta_r)
        s = np.sin(theta_r)
        R = np.array([[c, -s], [s, c]])

        start_map = R @ feature['start_point'] + np.array([x_r, y_r])
        end_map = R @ feature['end_point'] + np.array([x_r, y_r])

        # Transform points from robot frame to map frame
        points_map = (R @ feature['points'].T).T + np.array([x_r, y_r])

        # Add to FeatureMap (using EKF's landmark_id)
        self.feature_map.add_wall(
            landmark_id=landmark_id,
            rho=rho_map,
            alpha=alpha_map,
            start_point=start_map,
            end_point=end_map,
            points=points_map
        )

    def _add_corner_landmark(self, feature: Dict):
        """Add a new corner landmark to EKF and FeatureMap."""
        # Add to EKF
        z_x, z_y = feature['position']

        landmark_id = self.ekf.add_landmark(
            z_x=z_x,
            z_y=z_y,
            feature=feature,
            scan_number=self.ekf.current_scan_number
        )

        # Transform corner from robot frame to map frame
        corner_map = self._transform_point_to_map(z_x, z_y)

        # Add to FeatureMap (using EKF's landmark_id)
        self.feature_map.add_corner(
            landmark_id=landmark_id,
            position=corner_map
        )

    def _prune_old_landmarks(self):
        """Prune landmarks that haven't been seen recently."""
        pruned_ids = self.ekf.prune_landmarks(self.ekf.current_scan_number)

        # Synchronize FeatureMap: remove pruned landmarks
        for landmark_id in pruned_ids:
            self.feature_map.remove_landmark(landmark_id)

    def _transform_point_to_map(self, x_robot: float, y_robot: float) -> np.ndarray:
        """Transform a point from robot frame to map frame."""
        x_r, y_r, theta_r = self.ekf.state[0:3]
        c = np.cos(theta_r)
        s = np.sin(theta_r)
        R = np.array([[c, -s], [s, c]])

        return R @ np.array([x_robot, y_robot]) + np.array([x_r, y_r])

    def _empty_stats(self) -> Dict:
        """Return empty statistics dictionary."""
        return {
            'num_features': 0,
            'num_walls': 0,
            'num_corners': 0,
            'num_matched': 0,
            'num_unmatched': 0,
            'num_landmarks': 0,
            'num_walls_stored': 0,
            'num_corners_stored': 0,
            'observed_features': []
        }

    def get_robot_pose(self) -> Optional[Dict]:
        
        if not self.ekf_initialized:
            return None

        return self.ekf.get_state()

    def get_feature_map(self) -> FeatureMap:
        """Get reference to feature map."""
        return self.feature_map

    def generate_point_cloud(self, spacing: float = 0.05) -> np.ndarray:
       
        return self.feature_map.generate_point_cloud(spacing=spacing)

    def is_initialized(self) -> bool:
        """Check if EKF is initialized."""
        return self.ekf_initialized

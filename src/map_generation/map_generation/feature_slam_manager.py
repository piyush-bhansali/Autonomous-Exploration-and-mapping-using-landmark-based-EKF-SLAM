#!/usr/bin/env python3

import numpy as np
from typing import Dict, List, Tuple, Optional
from sensor_msgs.msg import LaserScan

from map_generation.ekf_update_feature import LandmarkEKFSLAM
from map_generation.landmark_features import LandmarkFeatureExtractor
from map_generation.data_association import associate_landmarks
from map_generation.feature_map import FeatureMap
from map_generation.transform_utils import robot_wall_to_map_frame, rotate_point_2d


class FeatureSLAMManager:

    def __init__(self):

        # Initialize EKF SLAM
        self.ekf = LandmarkEKFSLAM(
            landmark_timeout_scans=25,
            min_observations_for_init=2
        )

        # Initialize feature extractor (incremental line growing + adjacent merge)
        self.feature_extractor = LandmarkFeatureExtractor(
            min_points_per_line=8,
            min_line_length=0.5,
            corner_angle_threshold=45.0,
            max_gap=0.5,
            merge_angle_tolerance=0.35,
            merge_rho_tolerance=0.15,
            grow_residual_threshold=0.05
        )

        # Initialize feature map
        self.feature_map = FeatureMap()

        # Data association parameters
        self.chi_sq_gate = 5.99
        self.max_euclidean_dist = 6.0
        self.wall_angle_tolerance = 0.349066
        self.wall_rho_tolerance = 0.5
        self.max_gap_ext = 1 
        # Statistics
        self.total_scans_processed = 0

    def initialize_pose(self, x: float, y: float, theta: float):
       
        self.ekf.initialize(x, y, theta)

    def predict_motion(self, delta_d: float, delta_theta: float):

        if not self.ekf.initialized:
            return

        self.ekf.predict_with_relative_motion(delta_d, delta_theta)

    def process_scan(self, scan_msg: LaserScan) -> Dict:

        if not self.ekf.initialized:
            return self._empty_stats()

        # Extract features from scan (in robot frame)
        observed_features = self.feature_extractor.extract_features(scan_msg)

        if len(observed_features) == 0:
            return self._empty_stats()

        # Data association with wall extension info and gap check
        matched, unmatched, extension_info = associate_landmarks(
            observed_features,
            self.ekf,
            chi_sq_gate=self.chi_sq_gate,
            max_euclidean_dist=self.max_euclidean_dist,
            wall_angle_tolerance=self.wall_angle_tolerance,
            wall_rho_tolerance=self.wall_rho_tolerance,
            return_extension_info=True,
            feature_map=self.feature_map,
            max_gap_ext=self.max_gap_ext
        )

        # Process matched features: EKF update + feature map extension
        self._process_matched_features(observed_features, matched, extension_info)

        # Add new landmarks for unmatched features
        self._process_unmatched_features(observed_features, unmatched)

        # Prune landmarks that haven't been seen recently
        self._prune_old_landmarks()

        # Update statistics
        self.total_scans_processed += 1

        # Build per-feature landmark ID map for visualisation (None = unmatched)
        feature_ids = [None] * len(observed_features)
        for feat_idx, landmark_id in matched:
            feature_ids[feat_idx] = landmark_id

        # Return processing statistics
        num_walls_stored, num_corners_stored = self.feature_map.get_feature_count()
        num_walls = num_corners = 0
        for f in observed_features:
            if f['type'] == 'wall':
                num_walls += 1
            elif f['type'] == 'corner':
                num_corners += 1

        return {
            'num_features': len(observed_features),
            'num_walls': num_walls,
            'num_corners': num_corners,
            'num_matched': len(matched),
            'num_unmatched': len(unmatched),
            'num_landmarks': len(self.ekf.landmarks),
            'num_walls_stored': num_walls_stored,
            'num_corners_stored': num_corners_stored,
            'observed_features': observed_features,
            'feature_ids': feature_ids
        }

    def _process_matched_features(self, observed_features: List[Dict],
                                   matched: List[Tuple[int, int]],
                                   extension_info: Dict):

        for feat_idx, landmark_id in matched:
            feature = observed_features[feat_idx]

            # EKF update
            if feature['type'] == 'wall':
                self.ekf.update_landmark_observation(
                    landmark_id=landmark_id,
                    z_x=feature['rho'],
                    z_y=feature['alpha'],
                    scan_number=self.ekf.current_scan_number,
                    measurement_covariance=feature['covariance']
                )

            elif feature['type'] == 'corner':
                self.ekf.update_landmark_observation(
                    landmark_id=landmark_id,
                    z_x=feature['position'][0],
                    z_y=feature['position'][1],
                    scan_number=self.ekf.current_scan_number,
                    measurement_covariance=feature['covariance']
                )

                # Use EKF state directly for corner position (map frame)
                if landmark_id in self.ekf.landmarks:
                    lm_idx = self.ekf.landmarks[landmark_id]['state_index']
                    corner_map = self.ekf.state[lm_idx:lm_idx + 2]
                    self.feature_map.update_corner_position(
                        landmark_id=landmark_id,
                        new_position=corner_map
                    )

            # Wall extension: update FeatureMap endpoints if wall matched
            if feat_idx in extension_info:
                ext = extension_info[feat_idx]
                self.feature_map.update_wall_endpoints(
                    landmark_id=landmark_id,
                    new_start=ext['new_start'],
                    new_end=ext['new_end']
                )

        # Sync FeatureMap wall parameters with latest EKF state
        # More efficient: iterate over walls in feature_map rather than all EKF landmarks
        for lm_id in self.feature_map.walls.keys():
            if lm_id in self.ekf.landmarks and self.ekf.landmarks[lm_id]['feature_type'] == 'wall':
                lm_idx = self.ekf.landmarks[lm_id]['state_index']
                self.feature_map.update_wall_hessian(
                    lm_id,
                    rho=float(self.ekf.state[lm_idx]),
                    alpha=float(self.ekf.state[lm_idx + 1])
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
        landmark_id = self.ekf.add_landmark(
            z_x=feature['rho'],
            z_y=feature['alpha'],
            feature=feature,
            scan_number=self.ekf.current_scan_number
        )

        # Transform wall parameters and endpoints from robot frame to map frame
        x_r, y_r, theta_r = self.ekf.state[0:3]
        rho_map, alpha_map = robot_wall_to_map_frame(
            feature['rho'], feature['alpha'], x_r, y_r, theta_r
        )

        start_map = rotate_point_2d(feature['start_point'], x_r, y_r, theta_r)
        end_map = rotate_point_2d(feature['end_point'], x_r, y_r, theta_r)

        self.feature_map.add_wall(
            landmark_id=landmark_id,
            rho=rho_map,
            alpha=alpha_map,
            start_point=start_map,
            end_point=end_map
        )

    def _add_corner_landmark(self, feature: Dict):
        """Add a new corner landmark to EKF and FeatureMap."""
        z_x, z_y = feature['position']

        landmark_id = self.ekf.add_landmark(
            z_x=z_x,
            z_y=z_y,
            feature=feature,
            scan_number=self.ekf.current_scan_number
        )

        # Use EKF state directly for corner position (map frame)
        lm_idx = self.ekf.landmarks[landmark_id]['state_index']
        corner_map = self.ekf.state[lm_idx:lm_idx + 2]

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

        if not self.ekf.initialized:
            return None

        return self.ekf.get_state()

    def get_feature_map(self) -> FeatureMap:
        """Get reference to feature map."""
        return self.feature_map

    def generate_point_cloud(self, spacing: float = 0.05) -> np.ndarray:
        return self.feature_map.generate_point_cloud(spacing=spacing)

    def is_initialized(self) -> bool:
        """Check if EKF is initialized."""
        return self.ekf.initialized

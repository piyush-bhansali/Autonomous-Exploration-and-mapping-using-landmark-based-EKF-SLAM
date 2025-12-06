#!/usr/bin/env python3

import numpy as np
import gtsam
from typing import List, Dict, Optional, Tuple


class GTSAMOptimizer:
    
    def __init__(self,
                 translation_noise_sigma: float = 0.1,
                 rotation_noise_sigma: float = 0.1,
                 max_translation: float = 5.0,
                 max_rotation: float = np.pi):
        
        self.translation_sigma = translation_noise_sigma
        self.rotation_sigma = rotation_noise_sigma
        self.max_translation = max_translation
        self.max_rotation = max_rotation

        # Statistics
        self.optimization_count = 0
        self.last_optimization_error = None

    def optimize_pose_graph(self,
                           submaps: List[Dict],
                           loop_closure: Dict) -> Optional[List[np.ndarray]]:
       
        try:
            # Create factor graph and initial estimates
            graph = gtsam.NonlinearFactorGraph()
            initial_estimate = gtsam.Values()

            # Noise models
            # Loop closure constraint noise (based on ICP accuracy)
            loop_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([self.translation_sigma, self.translation_sigma, self.rotation_sigma])
            )

            # Prior noise for first pose (anchor the graph)
            prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([0.001, 0.001, 0.001])  # Very tight prior on first pose
            )

            # Add all submap poses to initial estimate
            for submap in submaps:
                submap_id = submap['id']
                transform = submap['global_transform']

                # Extract 2D pose from 4x4 transform
                x = transform[0, 3]
                y = transform[1, 3]
                theta = np.arctan2(transform[1, 0], transform[0, 0])

                pose2 = gtsam.Pose2(x, y, theta)
                initial_estimate.insert(submap_id, pose2)

            # Add prior on first pose (anchor the graph)
            first_submap = submaps[0]
            first_transform = first_submap['global_transform']
            first_pose = gtsam.Pose2(
                first_transform[0, 3],
                first_transform[1, 3],
                np.arctan2(first_transform[1, 0], first_transform[0, 0])
            )
            graph.add(gtsam.PriorFactorPose2(0, first_pose, prior_noise))

            odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([self.translation_sigma, self.translation_sigma, self.rotation_sigma])
            )

            for i in range(len(submaps) - 1):
                current_submap = submaps[i]
                next_submap = submaps[i + 1]

                # Compute relative transformation from current to next
                T_current = current_submap['global_transform']
                T_next = next_submap['global_transform']
                T_relative = np.linalg.inv(T_current) @ T_next

                # Extract 2D relative pose
                dx = T_relative[0, 3]
                dy = T_relative[1, 3]
                dtheta = np.arctan2(T_relative[1, 0], T_relative[0, 0])

                relative_pose = gtsam.Pose2(dx, dy, dtheta)

                # Add odometry constraint between consecutive submaps
                graph.add(gtsam.BetweenFactorPose2(
                    current_submap['id'], next_submap['id'], relative_pose, odometry_noise
                ))

            # Add loop closure constraint
            current_id = loop_closure['current_id']
            match_id = loop_closure['match_id']
            loop_transform = loop_closure['transform']

            # Extract relative pose from loop closure transform
            # This is the measured transformation from match_id to current_id
            dx = loop_transform[0, 3]
            dy = loop_transform[1, 3]
            dtheta = np.arctan2(loop_transform[1, 0], loop_transform[0, 0])

            # Validate transform before adding to graph
            translation_magnitude = np.sqrt(dx**2 + dy**2)
            rotation_magnitude = np.abs(dtheta)

            if translation_magnitude > self.max_translation:
                print(f"GTSAM: Loop closure translation too large ({translation_magnitude:.2f}m > {self.max_translation}m), rejecting")
                return None

            if rotation_magnitude > self.max_rotation:
                print(f"GTSAM: Loop closure rotation too large ({np.degrees(rotation_magnitude):.1f}° > {np.degrees(self.max_rotation):.1f}°), rejecting")
                return None

            # Check for degenerate transform (near-zero determinant)
            det = loop_transform[0, 0] * loop_transform[1, 1] - loop_transform[0, 1] * loop_transform[1, 0]
            if abs(det) < 0.1:  # Determinant should be ~1 for valid rotation
                print(f"GTSAM: Degenerate loop closure transform (det={det:.3f}), rejecting")
                return None

            relative_pose = gtsam.Pose2(dx, dy, dtheta)

            graph.add(gtsam.BetweenFactorPose2(
                match_id, current_id, relative_pose, loop_noise
            ))

            # Optimize using Levenberg-Marquardt
            params = gtsam.LevenbergMarquardtParams()
            params.setVerbosity("ERROR")  # Suppress optimization output
            params.setMaxIterations(100)
            params.setRelativeErrorTol(1e-5)

            optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
            result = optimizer.optimize()

            # Calculate optimization error
            initial_error = graph.error(initial_estimate)
            final_error = graph.error(result)
            self.last_optimization_error = {
                'initial': initial_error,
                'final': final_error,
                'improvement': initial_error - final_error
            }

            # Extract optimized poses and convert back to 4x4 transforms
            optimized_transforms = []
            for submap in submaps:
                submap_id = submap['id']
                optimized_pose = result.atPose2(submap_id)

                # Convert Pose2 back to 4x4 transform
                x = optimized_pose.x()
                y = optimized_pose.y()
                theta = optimized_pose.theta()

                transform = np.eye(4)
                transform[0, 0] = np.cos(theta)
                transform[0, 1] = -np.sin(theta)
                transform[1, 0] = np.sin(theta)
                transform[1, 1] = np.cos(theta)
                transform[0, 3] = x
                transform[1, 3] = y

                optimized_transforms.append(transform)

            self.optimization_count += 1

            return optimized_transforms

        except Exception as e:
            print(f"GTSAM optimization failed: {e}")
            return None

    def optimize_with_multiple_loop_closures(self,
                                            submaps: List[Dict],
                                            loop_closures: List[Dict]) -> Optional[List[np.ndarray]]:
        
        try:
            graph = gtsam.NonlinearFactorGraph()
            initial_estimate = gtsam.Values()

            # Noise models
            loop_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([self.translation_sigma, self.translation_sigma, self.rotation_sigma])
            )
            prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([0.001, 0.001, 0.001])
            )

            # Add all poses
            for submap in submaps:
                submap_id = submap['id']
                transform = submap['global_transform']

                x = transform[0, 3]
                y = transform[1, 3]
                theta = np.arctan2(transform[1, 0], transform[0, 0])

                pose2 = gtsam.Pose2(x, y, theta)
                initial_estimate.insert(submap_id, pose2)

            # Add prior on first pose
            first_submap = submaps[0]
            first_transform = first_submap['global_transform']
            first_pose = gtsam.Pose2(
                first_transform[0, 3],
                first_transform[1, 3],
                np.arctan2(first_transform[1, 0], first_transform[0, 0])
            )
            graph.add(gtsam.PriorFactorPose2(0, first_pose, prior_noise))

            # Add odometry constraints between consecutive submaps
            odometry_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([self.translation_sigma, self.translation_sigma, self.rotation_sigma])
            )

            for i in range(len(submaps) - 1):
                current_submap = submaps[i]
                next_submap = submaps[i + 1]

                T_current = current_submap['global_transform']
                T_next = next_submap['global_transform']
                T_relative = np.linalg.inv(T_current) @ T_next

                dx = T_relative[0, 3]
                dy = T_relative[1, 3]
                dtheta = np.arctan2(T_relative[1, 0], T_relative[0, 0])

                relative_pose = gtsam.Pose2(dx, dy, dtheta)

                graph.add(gtsam.BetweenFactorPose2(
                    current_submap['id'], next_submap['id'], relative_pose, odometry_noise
                ))

            # Add all loop closure constraints
            for loop_closure in loop_closures:
                current_id = loop_closure['current_id']
                match_id = loop_closure['match_id']
                loop_transform = loop_closure['transform']

                dx = loop_transform[0, 3]
                dy = loop_transform[1, 3]
                dtheta = np.arctan2(loop_transform[1, 0], loop_transform[0, 0])

                relative_pose = gtsam.Pose2(dx, dy, dtheta)

                graph.add(gtsam.BetweenFactorPose2(
                    match_id, current_id, relative_pose, loop_noise
                ))

            # Optimize
            params = gtsam.LevenbergMarquardtParams()
            params.setVerbosity("ERROR")
            params.setMaxIterations(100)
            params.setRelativeErrorTol(1e-5)

            optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial_estimate, params)
            result = optimizer.optimize()

            # Extract optimized transforms
            optimized_transforms = []
            for submap in submaps:
                submap_id = submap['id']
                optimized_pose = result.atPose2(submap_id)

                x = optimized_pose.x()
                y = optimized_pose.y()
                theta = optimized_pose.theta()

                transform = np.eye(4)
                transform[0, 0] = np.cos(theta)
                transform[0, 1] = -np.sin(theta)
                transform[1, 0] = np.sin(theta)
                transform[1, 1] = np.cos(theta)
                transform[0, 3] = x
                transform[1, 3] = y

                optimized_transforms.append(transform)

            self.optimization_count += 1

            return optimized_transforms

        except Exception as e:
            print(f"GTSAM optimization failed: {e}")
            return None

    def get_statistics(self) -> Dict:
        """Get optimizer statistics"""
        stats = {
            'optimization_count': self.optimization_count,
            'last_error': self.last_optimization_error
        }
        return stats

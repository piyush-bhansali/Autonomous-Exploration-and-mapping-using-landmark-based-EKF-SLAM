#!/usr/bin/env python3

import numpy as np
from typing import Dict, Optional, List, Tuple, Set
from collections import deque
import time


class DistributedFrameManager:
    """
    Manages distributed pose graph for multi-robot coordination.

    Maintains relative transformations between robots and computes
    transitive transforms using graph traversal (BFS).

    Key features:
    - Distributed pose graph: stores T_robotA_to_robotB edges
    - Graph traversal: computes T_A_to_C from T_A_to_B and T_B_to_C
    - Frame updates: handles reference frame changes
    - Consistency checking: validates loop closures
    """

    def __init__(self, robot_id: str, logger=None):
        """
        Initialize frame manager.

        Args:
            robot_id: Unique robot identifier (e.g., 'tb3_1')
            logger: ROS2 logger for debugging
        """
        self.robot_id = robot_id
        self.logger = logger

        # Reference frame (initially self)
        self.reference_frame_id = robot_id

        # Pose graph: adjacency list representation
        # Structure: {from_robot: {to_robot: T_from_to_to, ...}, ...}
        self.pose_graph: Dict[str, Dict[str, np.ndarray]] = {}

        # Current transform from reference frame to this robot
        self.T_ref_to_self = np.eye(4)

        # Set of known robots
        self.known_robots: Set[str] = {robot_id}

        # Timestamp of last update
        self.last_update_time = time.time()

        # Statistics
        self.num_alignments = 0
        self.num_frame_updates = 0

    def add_edge(self, from_robot: str, to_robot: str, transform: np.ndarray) -> bool:
        """
        Add directed edge to pose graph.

        Args:
            from_robot: Source robot ID
            to_robot: Target robot ID
            transform: 4x4 transformation matrix T_from_to_to

        Returns:
            True if edge added successfully
        """
        if transform.shape != (4, 4):
            if self.logger:
                self.logger.error(f'Invalid transform shape: {transform.shape}')
            return False

        # Add to graph
        if from_robot not in self.pose_graph:
            self.pose_graph[from_robot] = {}

        self.pose_graph[from_robot][to_robot] = transform.copy()

        # Add reverse edge (inverse transform)
        if to_robot not in self.pose_graph:
            self.pose_graph[to_robot] = {}

        self.pose_graph[to_robot][from_robot] = np.linalg.inv(transform)

        # Update known robots
        self.known_robots.add(from_robot)
        self.known_robots.add(to_robot)

        self.num_alignments += 1
        self.last_update_time = time.time()

        if self.logger:
            self.logger.info(f'Added edge: {from_robot} → {to_robot}')

        return True

    def get_transform(self, from_robot: str, to_robot: str) -> Optional[np.ndarray]:
        """
        Compute transformation from source to target robot using BFS.

        Args:
            from_robot: Source robot ID
            to_robot: Target robot ID

        Returns:
            4x4 transformation matrix T_from_to_to, or None if no path exists
        """
        if from_robot == to_robot:
            return np.eye(4)

        if from_robot not in self.pose_graph:
            if self.logger:
                self.logger.warn(f'Robot {from_robot} not in pose graph')
            return None

        # BFS to find shortest path
        queue = deque([(from_robot, np.eye(4))])
        visited = {from_robot}

        while queue:
            current_robot, T_from_to_current = queue.popleft()

            # Check if reached target
            if current_robot == to_robot:
                return T_from_to_current

            # Explore neighbors
            if current_robot in self.pose_graph:
                for neighbor, T_current_to_neighbor in self.pose_graph[current_robot].items():
                    if neighbor not in visited:
                        visited.add(neighbor)
                        T_from_to_neighbor = T_from_to_current @ T_current_to_neighbor
                        queue.append((neighbor, T_from_to_neighbor))

        # No path found
        if self.logger:
            self.logger.warn(f'No path from {from_robot} to {to_robot}')
        return None

    def update_reference_frame(self, new_reference_id: str) -> Optional[np.ndarray]:
        """
        Change reference frame and compute transformation.

        Args:
            new_reference_id: New reference frame robot ID

        Returns:
            Transformation T_old_ref_to_new_ref, or None if failed
        """
        if new_reference_id == self.reference_frame_id:
            return np.eye(4)

        # Compute transform from old reference to new reference
        T_old_to_new = self.get_transform(self.reference_frame_id, new_reference_id)

        if T_old_to_new is None:
            if self.logger:
                self.logger.error(f'Cannot change reference frame: no path to {new_reference_id}')
            return None

        # Update reference frame ID
        old_ref = self.reference_frame_id
        self.reference_frame_id = new_reference_id

        # Update T_ref_to_self
        T_new_to_self = self.get_transform(new_reference_id, self.robot_id)
        if T_new_to_self is not None:
            self.T_ref_to_self = T_new_to_self

        self.num_frame_updates += 1

        if self.logger:
            self.logger.info(f'Reference frame updated: {old_ref} → {new_reference_id}')

        return T_old_to_new

    def get_transform_to_reference(self, robot_id: str) -> Optional[np.ndarray]:
        """
        Get transformation from reference frame to specified robot.

        Args:
            robot_id: Target robot ID

        Returns:
            T_ref_to_robot
        """
        return self.get_transform(self.reference_frame_id, robot_id)

    def check_loop_consistency(self, from_robot: str, to_robot: str,
                               measured_transform: np.ndarray,
                               threshold: float = 0.5) -> Tuple[bool, float]:
        """
        Check if new measurement is consistent with existing path.

        Used for loop closure verification in distributed pose graphs.

        Args:
            from_robot: Source robot
            to_robot: Target robot
            measured_transform: Directly measured T_from_to_to
            threshold: Maximum translation error (meters)

        Returns:
            (is_consistent, error) tuple
        """
        # Compute transform via existing path
        computed_transform = self.get_transform(from_robot, to_robot)

        if computed_transform is None:
            # No existing path, cannot check consistency
            return True, 0.0

        # Compute difference
        diff = np.linalg.inv(computed_transform) @ measured_transform

        # Extract translation error
        translation_error = np.linalg.norm(diff[:2, 3])

        # Extract rotation error
        rotation_error = np.abs(np.arctan2(diff[1, 0], diff[0, 0]))

        is_consistent = translation_error < threshold and rotation_error < np.radians(15)

        if self.logger and not is_consistent:
            self.logger.warn(
                f'Loop closure inconsistency: {from_robot} → {to_robot}, '
                f'trans_error={translation_error:.3f}m, '
                f'rot_error={np.degrees(rotation_error):.1f}°'
            )

        return is_consistent, translation_error

    def get_all_robot_positions(self) -> Dict[str, np.ndarray]:
        """
        Get positions of all known robots in reference frame.

        Returns:
            Dictionary {robot_id: [x, y, z]}
        """
        positions = {}

        for robot_id in self.known_robots:
            T = self.get_transform_to_reference(robot_id)
            if T is not None:
                positions[robot_id] = T[:3, 3]

        return positions

    def get_statistics(self) -> Dict:
        """Get statistics about the pose graph."""
        return {
            'robot_id': self.robot_id,
            'reference_frame': self.reference_frame_id,
            'num_known_robots': len(self.known_robots),
            'num_edges': sum(len(neighbors) for neighbors in self.pose_graph.values()) // 2,
            'num_alignments': self.num_alignments,
            'num_frame_updates': self.num_frame_updates,
            'known_robots': list(self.known_robots),
            'last_update_time': self.last_update_time
        }

    def visualize_graph(self) -> str:
        """
        Generate text visualization of pose graph.

        Returns:
            String representation of graph structure
        """
        lines = [f'\n=== Pose Graph for {self.robot_id} ===']
        lines.append(f'Reference Frame: {self.reference_frame_id}')
        lines.append(f'Known Robots: {len(self.known_robots)}')
        lines.append('\nEdges:')

        visited_edges = set()
        for from_robot, neighbors in self.pose_graph.items():
            for to_robot, T in neighbors.items():
                edge = tuple(sorted([from_robot, to_robot]))
                if edge not in visited_edges:
                    visited_edges.add(edge)

                    # Extract translation
                    dx, dy = T[0, 3], T[1, 3]
                    dist = np.sqrt(dx**2 + dy**2)
                    theta = np.degrees(np.arctan2(T[1, 0], T[0, 0]))

                    lines.append(f'  {from_robot} ↔ {to_robot}: '
                               f'dist={dist:.2f}m, θ={theta:.1f}°')

        return '\n'.join(lines)

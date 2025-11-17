#!/usr/bin/env python3
import numpy as np
from scipy.spatial import KDTree
from typing import List, Optional


class RRTNode:
    """Node in RRT tree"""
    def __init__(self, position: np.ndarray):
        self.position = position  # [x, y]
        self.parent = None
        self.cost = 0.0


class SimpleRRTStar:
    """RRT* path planner with collision checking"""

    def __init__(self,
                 map_points: np.ndarray,  # Nx3 obstacles
                 robot_radius: float = 0.22,
                 step_size: float = 0.8,
                 goal_bias: float = 0.5,
                 max_iterations: int = 3000):
        
        # Extract 2D points and build KD-tree for collision checking
        self.obstacles_2d = map_points[:, :2]
        self.kdtree = KDTree(self.obstacles_2d)

        self.robot_radius = robot_radius
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.max_iterations = max_iterations

        # Map bounds for sampling
        self.x_min, self.y_min = self.obstacles_2d.min(axis=0)
        self.x_max, self.y_max = self.obstacles_2d.max(axis=0)

        # Add 5m margin for sampling
        self.x_min -= 5.0
        self.x_max += 5.0
        self.y_min -= 5.0
        self.y_max += 5.0

    def plan(self,
            start: np.ndarray,  # [x, y]
            goal: np.ndarray) -> Optional[List[np.ndarray]]:  # [x, y]
        """
        Plan path from start to goal using RRT*

        Returns:
            List of waypoints [x,y] or None if no path found
        """
        # Check if start/goal are valid
        if self._is_collision(start) or self._is_collision(goal):
            return None

        # Initialize tree with start node
        start_node = RRTNode(start)
        nodes = [start_node]
        node_kdtree = None  # Rebuilt periodically

        # RRT* main loop
        for iteration in range(self.max_iterations):
            # Sample random point (or goal with probability goal_bias)
            if np.random.random() < self.goal_bias:
                sample = goal
            else:
                sample = self._sample_random_point()

            # Find nearest node in tree
            nearest_node = self._find_nearest_node(nodes, sample, node_kdtree)

            # Steer towards sample
            new_pos = self._steer(nearest_node.position, sample)

            # Check collision
            if self._is_path_collision_free(nearest_node.position, new_pos):
                # Create new node
                new_node = RRTNode(new_pos)
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + np.linalg.norm(new_pos - nearest_node.position)

                # Add to tree
                nodes.append(new_node)

                # Rebuild KD-tree every 50 nodes for efficiency
                if len(nodes) % 50 == 0:
                    positions = np.array([n.position for n in nodes])
                    node_kdtree = KDTree(positions)

                # Check if goal reached
                if np.linalg.norm(new_pos - goal) < 0.5:
                    # Found path!
                    path = self._extract_path(new_node)
                    # Smooth path
                    path = self._smooth_path(path)
                    return path

        # No path found
        return None

    def _sample_random_point(self) -> np.ndarray:
        """Sample random point in map bounds"""
        x = np.random.uniform(self.x_min, self.x_max)
        y = np.random.uniform(self.y_min, self.y_max)
        return np.array([x, y])

    def _find_nearest_node(self, nodes: List[RRTNode], point: np.ndarray, kdtree) -> RRTNode:
        """Find nearest node to point"""
        if kdtree is not None and len(nodes) > 10:
            # Use KD-tree for fast search
            _, idx = kdtree.query(point)
            return nodes[idx]
        else:
            # Linear search for small trees
            return min(nodes, key=lambda n: np.linalg.norm(n.position - point))

    def _steer(self, from_pos: np.ndarray, to_pos: np.ndarray) -> np.ndarray:
        """Steer from from_pos towards to_pos by step_size"""
        direction = to_pos - from_pos
        distance = np.linalg.norm(direction)

        if distance <= self.step_size:
            return to_pos
        else:
            # Limit to step_size
            return from_pos + (direction / distance) * self.step_size

    def _is_collision(self, point: np.ndarray) -> bool:
        """Check if point collides with obstacles"""
        # Find nearest obstacle
        dist, _ = self.kdtree.query(point)
        return dist < self.robot_radius

    def _is_path_collision_free(self, start: np.ndarray, end: np.ndarray) -> bool:
        """Check if straight line path is collision-free"""
        # Sample points along path
        distance = np.linalg.norm(end - start)
        num_checks = int(distance / 0.2) + 1  # Check every 20cm

        for i in range(num_checks + 1):
            alpha = i / num_checks
            point = start + alpha * (end - start)

            if self._is_collision(point):
                return False

        return True

    def _extract_path(self, goal_node: RRTNode) -> List[np.ndarray]:
        """Extract path from tree by following parent pointers"""
        path = []
        node = goal_node

        while node is not None:
            path.append(node.position.copy())
            node = node.parent

        # Reverse to get start -> goal
        path.reverse()
        return path

    def _smooth_path(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """
        Smooth path by removing unnecessary waypoints

        Strategy: ALWAYS try direct start->goal connection first
        """
        if len(path) < 3:
            return path

        # ALWAYS try direct connection first (for corridors!)
        if self._is_path_collision_free(path[0], path[-1]):
            return [path[0], path[-1]]

        # Try shortcuts
        smoothed = [path[0]]

        i = 0
        while i < len(path) - 1:
            # Try to skip ahead as far as possible
            for j in range(len(path) - 1, i + 1, -1):
                if self._is_path_collision_free(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                # Couldn't skip, move to next
                i += 1
                if i < len(path):
                    smoothed.append(path[i])

        return smoothed

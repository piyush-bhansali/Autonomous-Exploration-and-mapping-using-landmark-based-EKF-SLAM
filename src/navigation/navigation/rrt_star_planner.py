#!/usr/bin/env python3
"""
RRT* Path Planner for Point Cloud Maps

Optimal sampling-based path planning directly on point cloud maps.
No grid conversion required - uses KD-tree collision checking.
"""

import numpy as np
from typing import List, Optional, Tuple
from dataclasses import dataclass
import time
from scipy.spatial import KDTree


@dataclass(eq=False)  # Disable auto-generated __eq__ to avoid numpy array comparison issues
class RRTNode:
    """Node in the RRT* tree"""
    position: np.ndarray  # [x, y]
    parent: Optional['RRTNode'] = None
    cost: float = 0.0  # Cost from start
    children: List['RRTNode'] = None

    def __post_init__(self):
        if self.children is None:
            self.children = []


class RRTStarPlanner:
    """
    RRT* path planner for 2D point cloud maps

    Features:
    - Asymptotically optimal paths
    - Rewiring for path improvement
    - Direct point cloud collision checking (no grids)
    - Dynamic obstacle handling
    """

    def __init__(self,
                 collision_checker,
                 step_size: float = 0.5,
                 goal_sample_rate: float = 0.1,
                 search_radius: float = 1.0,
                 max_iterations: int = 3000,
                 goal_tolerance: float = 0.3):
        """
        Args:
            collision_checker: PointCloudCollisionChecker instance
            step_size: Maximum distance to extend tree
            goal_sample_rate: Probability of sampling goal (0-1)
            search_radius: Radius for finding near neighbors for rewiring
            max_iterations: Maximum planning iterations
            goal_tolerance: Distance to goal to consider success
        """
        self.collision_checker = collision_checker
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.max_iterations = max_iterations
        self.goal_tolerance = goal_tolerance

        # Tree storage
        self.nodes = []
        self.start_node = None
        self.goal_node = None
        self.node_kdtree = None  # KD-tree for fast nearest neighbor search
        self.kdtree_rebuild_interval = 50  # Rebuild every N nodes for efficiency

    def plan(self,
             start: np.ndarray,
             goal: np.ndarray,
             planning_bounds: Optional[Tuple[float, float, float, float]] = None) -> Optional[List[np.ndarray]]:
        """
        Plan a path from start to goal

        Args:
            start: Start position [x, y]
            goal: Goal position [x, y]
            planning_bounds: (min_x, max_x, min_y, max_y) for sampling

        Returns:
            Path as list of [x, y] waypoints, or None if planning failed
        """
        start_time = time.time()

        # Validate start and goal
        if self.collision_checker.is_collision(start[0], start[1]):
            return None

        if self.collision_checker.is_collision(goal[0], goal[1]):
            return None

        # Set planning bounds
        if planning_bounds is None:
            planning_bounds = self.collision_checker.get_map_bounds()

        self.planning_bounds = planning_bounds

        # Initialize tree
        self.start_node = RRTNode(position=start, cost=0.0)
        self.nodes = [self.start_node]
        self.goal_node = None
        self.node_kdtree = None  # Will be built on first use

        # RRT* main loop
        for iteration in range(self.max_iterations):
            # Sample random point (or goal)
            if np.random.random() < self.goal_sample_rate:
                sample_point = goal
            else:
                sample_point = self._sample_random_point()

            # Find nearest node in tree
            nearest_node = self._get_nearest_node(sample_point)

            # Steer towards sample point
            new_position = self._steer(nearest_node.position, sample_point)

            # Check collision
            if not self.collision_checker.is_path_collision_free(
                nearest_node.position, new_position
            ):
                continue

            # Find near neighbors for rewiring
            near_nodes = self._get_near_nodes(new_position)

            # Choose best parent (lowest cost)
            best_parent = nearest_node
            best_cost = nearest_node.cost + np.linalg.norm(new_position - nearest_node.position)

            for near_node in near_nodes:
                potential_cost = near_node.cost + np.linalg.norm(new_position - near_node.position)

                if potential_cost < best_cost:
                    # Check if connection is collision-free
                    if self.collision_checker.is_path_collision_free(near_node.position, new_position):
                        best_parent = near_node
                        best_cost = potential_cost

            # Create new node
            new_node = RRTNode(
                position=new_position,
                parent=best_parent,
                cost=best_cost
            )

            best_parent.children.append(new_node)
            self.nodes.append(new_node)

            # Rebuild KD-tree periodically for efficiency
            if len(self.nodes) % self.kdtree_rebuild_interval == 0:
                self._rebuild_kdtree()

            # Rewire tree: check if nearby nodes can be improved through new node
            for near_node in near_nodes:
                if near_node is best_parent:
                    continue

                potential_cost = new_node.cost + np.linalg.norm(near_node.position - new_position)

                if potential_cost < near_node.cost:
                    # Check collision
                    if self.collision_checker.is_path_collision_free(new_position, near_node.position):
                        # Rewire: change parent
                        if near_node.parent:
                            near_node.parent.children.remove(near_node)

                        near_node.parent = new_node
                        near_node.cost = potential_cost
                        new_node.children.append(near_node)

                        # Propagate cost update to descendants
                        self._propagate_cost_to_children(near_node)

            # Check if goal reached
            distance_to_goal = np.linalg.norm(new_position - goal)
            if distance_to_goal < self.goal_tolerance:
                # Connect to goal
                if self.collision_checker.is_path_collision_free(new_position, goal):
                    goal_cost = new_node.cost + distance_to_goal
                    goal_node = RRTNode(position=goal, parent=new_node, cost=goal_cost)
                    new_node.children.append(goal_node)
                    self.nodes.append(goal_node)
                    self.goal_node = goal_node

                    return self._extract_path()

        return None

    def _sample_random_point(self) -> np.ndarray:
        """Sample a random point within planning bounds"""
        min_x, max_x, min_y, max_y = self.planning_bounds

        x = np.random.uniform(min_x, max_x)
        y = np.random.uniform(min_y, max_y)

        return np.array([x, y])

    def _rebuild_kdtree(self):
        """Rebuild KD-tree for fast nearest neighbor queries"""
        if len(self.nodes) > 0:
            positions = np.array([node.position for node in self.nodes])
            self.node_kdtree = KDTree(positions)

    def _get_nearest_node(self, point: np.ndarray) -> RRTNode:
        """Find nearest node in tree to given point using KD-tree"""
        if len(self.nodes) == 0:
            return None

        # Rebuild KD-tree if needed
        if self.node_kdtree is None:
            self._rebuild_kdtree()

        # Use KD-tree for O(log N) search instead of O(N)
        distance, index = self.node_kdtree.query(point)
        return self.nodes[index]

    def _steer(self, from_pos: np.ndarray, to_pos: np.ndarray) -> np.ndarray:
        """
        Steer from one position towards another, limited by step size

        Args:
            from_pos: Starting position
            to_pos: Target position

        Returns:
            New position (at most step_size away from from_pos)
        """
        direction = to_pos - from_pos
        distance = np.linalg.norm(direction)

        if distance < self.step_size:
            return to_pos

        # Limit to step size
        direction = direction / distance  # Normalize
        return from_pos + direction * self.step_size

    def _get_near_nodes(self, point: np.ndarray) -> List[RRTNode]:
        """
        Get all nodes within search radius of point

        This is used for rewiring in RRT*
        """
        near_nodes = []

        for node in self.nodes:
            distance = np.linalg.norm(node.position - point)
            if distance < self.search_radius:
                near_nodes.append(node)

        return near_nodes

    def _propagate_cost_to_children(self, node: RRTNode):
        """
        Update costs of all descendants after rewiring

        This ensures cost-to-come is accurate throughout tree
        """
        for child in node.children:
            child.cost = node.cost + np.linalg.norm(child.position - node.position)
            self._propagate_cost_to_children(child)

    def _extract_path(self) -> List[np.ndarray]:
        """
        Extract path from start to goal by backtracking through tree

        Returns:
            Path as list of waypoints [start, ..., goal]
        """
        if self.goal_node is None:
            return None

        path = []
        current_node = self.goal_node

        while current_node is not None:
            path.append(current_node.position.copy())
            current_node = current_node.parent

        # Reverse to get start -> goal
        path.reverse()

        return path

    def smooth_path(self, path: List[np.ndarray], iterations: int = 50) -> List[np.ndarray]:
        """
        Smooth path by shortcutting where possible

        Args:
            path: Original path
            iterations: Number of smoothing attempts

        Returns:
            Smoothed path
        """
        if path is None or len(path) < 3:
            return path

        smoothed_path = [p.copy() for p in path]

        for _ in range(iterations):
            if len(smoothed_path) < 3:
                break

            # Pick two random waypoints
            i = np.random.randint(0, len(smoothed_path) - 2)
            j = np.random.randint(i + 2, len(smoothed_path))

            # Try to connect them directly
            if self.collision_checker.is_path_collision_free(
                smoothed_path[i], smoothed_path[j]
            ):
                # Remove intermediate waypoints
                smoothed_path = smoothed_path[:i+1] + smoothed_path[j:]

        return smoothed_path

    def get_tree_visualization_points(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get tree edges for visualization

        Returns:
            (edge_starts, edge_ends) as Nx2 arrays
        """
        edge_starts = []
        edge_ends = []

        for node in self.nodes:
            if node.parent is not None:
                edge_starts.append(node.parent.position)
                edge_ends.append(node.position)

        return np.array(edge_starts), np.array(edge_ends)

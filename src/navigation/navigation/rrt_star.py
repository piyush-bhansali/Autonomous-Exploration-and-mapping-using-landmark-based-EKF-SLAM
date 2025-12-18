#!/usr/bin/env python3
import numpy as np
from scipy.spatial import KDTree
from typing import List, Optional
from collections import deque


class RRTNode:
    """Node in RRT* tree"""
    def __init__(self, position: np.ndarray):
        self.position = position  # [x, y]
        self.parent = None
        self.cost = 0.0
        self.children = []  # Track children for cost propagation


class RRTStar:

    def __init__(self,
                 map_points: np.ndarray,  # Nx3 obstacles
                 robot_radius: float = 0.22,
                 step_size: float = 0.4,
                 goal_bias: float = 0.5,
                 max_iterations: int = 3000,
                 gamma: float = 2.0,  # RRT* gamma parameter for shrinking radius
                 workspace_bounds: Optional[tuple] = None):  # ((x_min, x_max), (y_min, y_max))

        # Extract 2D points and build KD-tree for collision checking
        self.obstacles_2d = map_points[:, :2]
        self.kdtree = KDTree(self.obstacles_2d)

        self.robot_radius = robot_radius
        self.step_size = step_size  # eta (max connection distance)
        self.goal_bias = goal_bias
        self.max_iterations = max_iterations
        self.gamma = gamma  # Gamma parameter for shrinking rewire radius

        # Map bounds for sampling
        if workspace_bounds is not None:
            # Use provided workspace bounds (better for known environments)
            (self.x_min, self.x_max), (self.y_min, self.y_max) = workspace_bounds
        else:
            # Derive from obstacle extents with margin (fallback)
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
        
        # Check if start/goal are valid
        if self._is_collision(start) or self._is_collision(goal):
            return None

        # Initialize tree with start node
        start_node = RRTNode(start)
        nodes = [start_node]
        node_kdtree = None  # Rebuilt periodically
        kdtree_built_at = 0  # Track when KD-tree was last built

        # RRT* main loop
        for iteration in range(self.max_iterations):
            # Sample random point (or goal with probability goal_bias)
            if np.random.random() < self.goal_bias:
                sample = goal
            else:
                sample = self._sample_random_point()

            # Find nearest node in tree (with hybrid search)
            nearest_node = self._find_nearest_node(nodes, sample, node_kdtree, kdtree_built_at)

            # Steer towards sample
            new_pos = self._steer(nearest_node.position, sample)

            # Check collision
            if self._is_path_collision_free(nearest_node.position, new_pos):
                # RRT*: Compute shrinking rewire radius based on number of nodes
                # r_n = min(eta, gamma * (log(n) / n)^(1/d)) where d=2 for 2D
                n = len(nodes) + 1  # Include the new node
                if n > 1:
                    rewire_radius = min(self.step_size, self.gamma * np.sqrt(np.log(n) / n))
                else:
                    rewire_radius = self.step_size

                # RRT*: Find near neighbors (with hybrid search)
                near_nodes = self._find_near_neighbors(nodes, new_pos, node_kdtree, kdtree_built_at, rewire_radius)

                # RRT*: Choose best parent (minimize cost)
                # Cache distances to avoid redundant calculations
                near_distances = {}

                best_parent = nearest_node
                edge_dist = np.linalg.norm(new_pos - nearest_node.position)
                best_cost = nearest_node.cost + edge_dist
                near_distances[id(nearest_node)] = edge_dist

                for near_node in near_nodes:
                    edge_dist = np.linalg.norm(new_pos - near_node.position)
                    near_distances[id(near_node)] = edge_dist
                    potential_cost = near_node.cost + edge_dist

                    if potential_cost < best_cost:
                        if self._is_path_collision_free(near_node.position, new_pos):
                            best_parent = near_node
                            best_cost = potential_cost

                # Create new node with best parent
                new_node = RRTNode(new_pos)
                new_node.parent = best_parent
                new_node.cost = best_cost
                best_parent.children.append(new_node)  # Track parent-child relationship

                # Add to tree
                nodes.append(new_node)

                # RRT*: Rewire tree - check if path through new_node is better for neighbors
                for near_node in near_nodes:
                    # Skip if this is the parent we just chose (already optimal connection)
                    if near_node is best_parent:
                        continue

                    # Reuse cached distance
                    edge_dist = near_distances.get(id(near_node))
                    if edge_dist is None:
                        edge_dist = np.linalg.norm(near_node.position - new_pos)

                    potential_cost = new_node.cost + edge_dist

                    if potential_cost < near_node.cost:
                        if self._is_path_collision_free(new_pos, near_node.position):
                            # Rewire: new_node is better parent for near_node
                            old_parent = near_node.parent
                            if old_parent is not None:
                                old_parent.children.remove(near_node)  # Remove from old parent

                            near_node.parent = new_node
                            new_node.children.append(near_node)  # Add to new parent

                            # Propagate cost update to all descendants
                            self._propagate_cost_to_descendants(near_node, potential_cost)

                # Rebuild KD-tree every 50 nodes for efficiency
                if len(nodes) % 50 == 0:
                    positions = np.array([n.position for n in nodes])
                    node_kdtree = KDTree(positions)
                    kdtree_built_at = len(nodes)  # Track rebuild point

                # Check if goal reached
                dist_to_goal = np.linalg.norm(new_pos - goal)
                if dist_to_goal < 0.5:
                    # Found path! Check if we can connect directly to goal
                    if self._is_path_collision_free(new_pos, goal):
                        # Create final goal node
                        goal_node = RRTNode(goal)
                        goal_node.parent = new_node
                        goal_node.cost = new_node.cost + dist_to_goal  # Reuse calculated distance
                        new_node.children.append(goal_node)

                        path = self._extract_path(goal_node)
                    else:
                        # Can't reach exact goal, use near-goal node
                        path = self._extract_path(new_node)

                    # Smooth path
                    path = self._smooth_path(path)
                    return path

        # No path found
        return None

    def _propagate_cost_to_descendants(self, node: RRTNode, new_cost: float):
        """
        Propagate cost update to all descendants using BFS.
        When a node's cost changes due to rewiring, all descendants must update.
        """
        # Update the node's cost
        node.cost = new_cost

        # BFS to propagate to all descendants
        queue = deque(node.children)

        while queue:
            child = queue.popleft()

            # Update child's cost based on its parent
            edge_cost = np.linalg.norm(child.position - child.parent.position)
            child.cost = child.parent.cost + edge_cost

            # Add child's children to queue
            queue.extend(child.children)

    def _sample_random_point(self) -> np.ndarray:
        """
        Sample random collision-free point in map bounds.
        Uses rejection sampling to avoid wasting iterations on obstacle samples.
        """
        max_attempts = 50  # Prevent infinite loop in highly cluttered spaces

        for _ in range(max_attempts):
            x = np.random.uniform(self.x_min, self.x_max)
            y = np.random.uniform(self.y_min, self.y_max)
            point = np.array([x, y])

            # Accept only collision-free samples
            if not self._is_collision(point):
                return point

        # Fallback: if we can't find free space after max_attempts,
        # return the last sample anyway (let the collision check reject it later)
        return point

    def _find_nearest_node(self, nodes: List[RRTNode], point: np.ndarray, kdtree, kdtree_built_at: int) -> RRTNode:
        """
        Find nearest node to point using hybrid search.
        Queries KD-tree for bulk of nodes, then linearly scans recent nodes.
        """
        if kdtree is not None and len(nodes) > 10:
            # Query KD-tree for nodes in the tree when it was built
            dist_kdtree, idx_kdtree = kdtree.query(point)
            best_node = nodes[idx_kdtree]
            best_dist = dist_kdtree

            # Also check "tail" nodes added since KD-tree was built
            for i in range(kdtree_built_at, len(nodes)):
                dist = np.linalg.norm(nodes[i].position - point)
                if dist < best_dist:
                    best_dist = dist
                    best_node = nodes[i]

            return best_node
        else:
            # Linear search for small trees
            return min(nodes, key=lambda n: np.linalg.norm(n.position - point))

    def _find_near_neighbors(self, nodes: List[RRTNode], point: np.ndarray, kdtree, kdtree_built_at: int, rewire_radius: float) -> List[RRTNode]:
        """
        Find all nodes within rewire_radius using hybrid search.
        Queries KD-tree for bulk of nodes, then linearly scans recent nodes.
        """
        if kdtree is not None and len(nodes) > 10:
            # Query KD-tree for nodes within radius (only nodes that were in the tree)
            indices = kdtree.query_ball_point(point, rewire_radius)
            near_nodes = [nodes[i] for i in indices]

            # Also check "tail" nodes added since KD-tree was built
            # Note: kdtree_built_at is the number of nodes when tree was built
            for i in range(kdtree_built_at, len(nodes)):
                if np.linalg.norm(nodes[i].position - point) <= rewire_radius:
                    near_nodes.append(nodes[i])

            return near_nodes
        else:
            # Linear search for small trees
            return [
                n for n in nodes
                if np.linalg.norm(n.position - point) <= rewire_radius
            ]

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
        """
        Check if straight line path is collision-free.
        Sampling resolution tied to robot radius to avoid missing obstacles.
        """
        distance = np.linalg.norm(end - start)

        # Handle very short distances (points are essentially the same)
        if distance < 1e-6:
            return not self._is_collision(start)

        # Sample at intervals of robot_radius/2 to ensure we don't miss obstacles
        # that could fit between check points
        check_interval = self.robot_radius / 2.0
        num_checks = max(1, int(np.ceil(distance / check_interval)))

        for i in range(num_checks + 1):
            alpha = i / num_checks if num_checks > 0 else 0.0
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
        Smooth path by removing unnecessary waypoints while preserving obstacle avoidance.

        Uses incremental shortcutting to reduce waypoints without creating dangerous
        straight lines through obstacles.
        """
        if len(path) < 3:
            return path

        # Incremental shortcut smoothing (safe approach)
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

        # Densify path with intermediate waypoints for smoother curves
        densified = self._densify_path(smoothed, spacing=0.2)

        return densified

    def _densify_path(self, path: List[np.ndarray], spacing: float = 0.2) -> List[np.ndarray]:
        """
        Add intermediate waypoints along path segments for smoother curve following.

        Args:
            path: List of waypoints
            spacing: Desired spacing between waypoints (meters)

        Returns:
            Densified path with intermediate points
        """
        if len(path) < 2:
            return path

        densified = [path[0]]

        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]

            # Calculate segment length
            segment_length = np.linalg.norm(end - start)

            # Number of intermediate points needed
            if segment_length > spacing:
                num_points = int(np.ceil(segment_length / spacing))

                # Add intermediate points
                for j in range(1, num_points):
                    alpha = j / num_points
                    intermediate = start + alpha * (end - start)
                    densified.append(intermediate)

            # Add end point
            densified.append(end)

        return densified

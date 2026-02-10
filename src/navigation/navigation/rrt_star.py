#!/usr/bin/env python3
import numpy as np
from scipy.spatial import KDTree
from typing import List, Optional
from collections import deque


class RRTNode:
    """Node in RRT* tree"""
    def __init__(self, position: np.ndarray):
        self.position = position  
        self.parent = None
        self.cost = 0.0
        self.children = []  


class RRTStar:

    def __init__(self,
                 map_points: np.ndarray,
                 robot_radius: float = 0.22,
                 workspace_bounds: Optional[tuple] = None,
                 obstacle_kdtree: Optional[KDTree] = None):

        self.obstacles_2d = map_points[:, :2]

        if obstacle_kdtree is not None:
            self.kdtree = obstacle_kdtree
        else:
            self.kdtree = KDTree(self.obstacles_2d)

        self.robot_radius = robot_radius
        self.safety_margin = 0.5
        self.relaxed_safety_margin = 0.25  
        self.step_size = 0.2
        self.goal_bias = 0.5
        self.max_iterations = 1500
        self.gamma = 2.0

        if workspace_bounds is not None:
            (self.x_min, self.x_max), (self.y_min, self.y_max) = workspace_bounds
        else:
            
            self.x_min, self.y_min = self.obstacles_2d.min(axis=0)
            self.x_max, self.y_max = self.obstacles_2d.max(axis=0)

        self.x_min -= 10
        self.x_max += 10
        self.y_min -= 10
        self.y_max += 10

    def plan(self,
            start: np.ndarray,
            goal: np.ndarray,
            logger=None) -> Optional[List[np.ndarray]]:

        start_collision = self._is_collision(start)
        start_collision_relaxed = self._is_collision_relaxed(start)
        goal_collision = self._is_collision(goal)

        recovery_mode = start_collision and not start_collision_relaxed

        if recovery_mode:
            if logger:
                logger.warn(f'  RRT* RECOVERY MODE: Start position [{start[0]:.2f}, {start[1]:.2f}] is close to obstacles')
                logger.warn(f'    Using relaxed safety margin ({self.relaxed_safety_margin}m) for start position')

        if start_collision_relaxed or goal_collision:
            if logger:
                if start_collision_relaxed:
                    logger.error(f'  RRT* FAILURE: START position [{start[0]:.2f}, {start[1]:.2f}] is in COLLISION (obstacle within {self.relaxed_safety_margin}m)')
                if goal_collision:
                    logger.error(f'  RRT* FAILURE: GOAL position [{goal[0]:.2f}, {goal[1]:.2f}] is in COLLISION (obstacle within {self.safety_margin}m)')
            return None

        start_node = RRTNode(start)
        nodes = [start_node]
        node_kdtree = None
        kdtree_built_at = 0

        for _ in range(self.max_iterations):

            if np.random.random() < self.goal_bias:
                sample = goal
            else:
                sample = self._sample_random_point(use_relaxed=recovery_mode)

            nearest_node = self._find_nearest_node(nodes, sample, node_kdtree, kdtree_built_at)

            new_pos = self._steer(nearest_node.position, sample)

            if self._is_path_collision_free(nearest_node.position, new_pos, use_relaxed=recovery_mode):

                n = len(nodes) + 1  
                if n > 1:
                    rewire_radius = min(self.step_size, self.gamma * np.sqrt(np.log(n) / n))
                else:
                    rewire_radius = self.step_size

                near_nodes = self._find_near_neighbors(nodes, new_pos, node_kdtree, kdtree_built_at, rewire_radius)

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
                        if self._is_path_collision_free(near_node.position, new_pos, use_relaxed=recovery_mode):
                            best_parent = near_node
                            best_cost = potential_cost

                new_node = RRTNode(new_pos)
                new_node.parent = best_parent
                new_node.cost = best_cost
                best_parent.children.append(new_node)  

                nodes.append(new_node)

                for near_node in near_nodes:
                    
                    if near_node is best_parent:
                        continue

                    edge_dist = near_distances.get(id(near_node))
                    if edge_dist is None:
                        edge_dist = np.linalg.norm(near_node.position - new_pos)

                    potential_cost = new_node.cost + edge_dist

                    if potential_cost < near_node.cost:
                        if self._is_path_collision_free(new_pos, near_node.position, use_relaxed=recovery_mode):

                            old_parent = near_node.parent
                            if old_parent is not None:
                                old_parent.children.remove(near_node)  

                            near_node.parent = new_node
                            new_node.children.append(near_node) 

                            self._propagate_cost_to_descendants(near_node, potential_cost)

                if len(nodes) % 50 == 0:
                    positions = np.array([n.position for n in nodes])
                    node_kdtree = KDTree(positions)
                    kdtree_built_at = len(nodes)  
                dist_to_goal = np.linalg.norm(new_pos - goal)
                if dist_to_goal < 0.5:

                    if self._is_path_collision_free(new_pos, goal, use_relaxed=recovery_mode):
                        # Create final goal node
                        goal_node = RRTNode(goal)
                        goal_node.parent = new_node
                        goal_node.cost = new_node.cost + dist_to_goal 
                        new_node.children.append(goal_node)

                        path = self._extract_path(goal_node)
                    else:
                        
                        path = self._extract_path(new_node)

                    path = self._smooth_path(path)
                    return path

        if logger:
            logger.error(f'  RRT* FAILURE: Max iterations ({self.max_iterations}) reached without finding path')
            logger.error(f'    Nodes explored: {len(nodes)}')
            logger.error(f'    Closest node to goal: {np.linalg.norm(nodes[-1].position - goal):.2f}m')
        return None

    def _propagate_cost_to_descendants(self, node: RRTNode, new_cost: float):
       
        node.cost = new_cost

        queue = deque(node.children)

        while queue:
            child = queue.popleft()

            edge_cost = np.linalg.norm(child.position - child.parent.position)
            child.cost = child.parent.cost + edge_cost

            queue.extend(child.children)

    def _sample_random_point(self, use_relaxed: bool = False) -> np.ndarray:

        max_attempts = 50

        for _ in range(max_attempts):
            x = np.random.uniform(self.x_min, self.x_max)
            y = np.random.uniform(self.y_min, self.y_max)
            point = np.array([x, y])

            # Use relaxed collision check in recovery mode
            collision_check = self._is_collision_relaxed if use_relaxed else self._is_collision
            if not collision_check(point):
                return point

        return point

    def _find_nearest_node(self, nodes: List[RRTNode], point: np.ndarray, kdtree, kdtree_built_at: int) -> RRTNode:
        
        if kdtree is not None:
           
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
            return min(nodes, key=lambda n: np.linalg.norm(n.position - point))

    def _find_near_neighbors(self, nodes: List[RRTNode], point: np.ndarray, kdtree, kdtree_built_at: int, rewire_radius: float) -> List[RRTNode]:
        
        if kdtree is not None:
            
            indices = kdtree.query_ball_point(point, rewire_radius)
            near_nodes = [nodes[i] for i in indices]

            for i in range(kdtree_built_at, len(nodes)):
                if np.linalg.norm(nodes[i].position - point) <= rewire_radius:
                    near_nodes.append(nodes[i])

            return near_nodes
        else:
            
            return [
                n for n in nodes
                if np.linalg.norm(n.position - point) <= rewire_radius
            ]

    def _steer(self, from_pos: np.ndarray, to_pos: np.ndarray) -> np.ndarray:

        direction = to_pos - from_pos
        distance = np.linalg.norm(direction)

        if distance <= self.step_size:
            return to_pos
        else:
            return from_pos + (direction / distance) * self.step_size

    def _is_collision(self, point: np.ndarray) -> bool:

        dist, _ = self.kdtree.query(point)
        return dist < self.safety_margin

    def _is_collision_relaxed(self, point: np.ndarray) -> bool:
        
        dist, _ = self.kdtree.query(point)
        return dist < self.relaxed_safety_margin

    def _is_path_collision_free(self, start: np.ndarray, end: np.ndarray, use_relaxed: bool = False) -> bool:

        distance = np.linalg.norm(end - start)

        collision_check = self._is_collision_relaxed if use_relaxed else self._is_collision

        if distance < 1e-6:
            return not collision_check(start)

        check_interval = self.robot_radius / 2.0
        num_checks = max(1, int(np.ceil(distance / check_interval)))

        for i in range(num_checks + 1):
            alpha = i / num_checks if num_checks > 0 else 0.0
            point = start + alpha * (end - start)

            if collision_check(point):
                return False

        return True

    def _extract_path(self, goal_node: RRTNode) -> List[np.ndarray]:
        
        path = []
        node = goal_node

        while node is not None:
            path.append(node.position.copy())
            node = node.parent

        # Reverse to get start -> goal
        path.reverse()
        return path

    def _smooth_path(self, path: List[np.ndarray]) -> List[np.ndarray]:
       
        if len(path) < 3:
            return path

        smoothed = [path[0]]

        i = 0
        while i < len(path) - 1:
            
            for j in range(len(path) - 1, i + 1, -1):
                if self._is_path_collision_free(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                
                i += 1
                if i < len(path):
                    smoothed.append(path[i])

        densified = self._densify_path(smoothed, spacing=0.2)

        return densified

    def _densify_path(self, path: List[np.ndarray], spacing: float = 0.2) -> List[np.ndarray]:
        
        if len(path) < 2:
            return path

        densified = [path[0]]

        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]

            segment_length = np.linalg.norm(end - start)

            if segment_length > spacing:
                num_points = int(np.ceil(segment_length / spacing))

                for j in range(1, num_points):
                    alpha = j / num_points
                    intermediate = start + alpha * (end - start)
                    densified.append(intermediate)
 
            densified.append(end)

        return densified

#!/usr/bin/env python3
"""
Frontier-Based Exploration Controller

Coordinates frontier detection and RRT* path planning for autonomous exploration.
"""

import numpy as np
import open3d as o3d
from typing import List, Optional, Tuple
from enum import Enum
from dataclasses import dataclass
import time
import logging

from navigation.frontier_detector import FrontierDetector, Frontier
from navigation.rrt_star_planner import RRTStarPlanner
from navigation.point_cloud_collision_checker import PointCloudCollisionChecker

# Set up logging
logger = logging.getLogger('FrontierExplorer')
logger.setLevel(logging.INFO)


class ExplorationState(Enum):
    """States of the exploration controller"""
    IDLE = 0
    DETECTING_FRONTIERS = 1
    PLANNING_PATH = 2
    EXECUTING_PATH = 3
    EXPLORATION_COMPLETE = 4
    FAILED = 5


@dataclass
class ExplorationStatus:
    """Status information for exploration"""
    state: ExplorationState
    current_frontier: Optional[Frontier] = None
    current_path: Optional[List[np.ndarray]] = None
    path_index: int = 0
    total_distance_traveled: float = 0.0
    num_frontiers_explored: int = 0
    exploration_start_time: float = 0.0


class FrontierExplorer:
    """
    Main exploration controller

    Manages the exploration loop:
    1. Detect frontiers
    2. Select best frontier
    3. Plan path with RRT*
    4. Execute path
    5. Repeat
    """

    def __init__(self,
                 robot_radius: float = 0.2,
                 waypoint_tolerance: float = 0.3,  # Increased from 0.2 for odometry noise
                 max_exploration_time: float = 3600.0,
                 min_progress_distance: float = 0.5):
        """
        Args:
            robot_radius: Robot collision radius
            waypoint_tolerance: Distance to waypoint to consider reached
            max_exploration_time: Maximum exploration time (seconds)
            min_progress_distance: Minimum distance to move for progress
        """
        self.robot_radius = robot_radius
        self.waypoint_tolerance = waypoint_tolerance
        self.max_exploration_time = max_exploration_time
        self.min_progress_distance = min_progress_distance

        # Components
        self.frontier_detector = FrontierDetector(robot_radius=robot_radius)
        self.collision_checker = None
        self.planner = None

        # State
        self.status = ExplorationStatus(
            state=ExplorationState.IDLE,
            exploration_start_time=time.time()
        )

        # History tracking
        self.explored_frontier_positions = []  # Track explored frontiers
        self.position_history = []

        # State recovery tracking
        self.last_state_change_time = time.time()
        self.stuck_threshold = 5.0  # seconds - default for most states
        self.planning_stuck_threshold = 15.0  # seconds - allow more time for RRT* planning

        # Stuck detection during path execution
        self.last_waypoint_progress_time = time.time()
        self.last_waypoint_position = None
        self.waypoint_stuck_threshold = 10.0  # seconds without progress to waypoint

    def update_map(self, point_cloud: o3d.geometry.PointCloud):
        """
        Update the exploration map

        Args:
            point_cloud: Latest point cloud map
        """
        if len(point_cloud.points) < 10:
            logger.warning(f"Map has only {len(point_cloud.points)} points - need at least 10")
            return

        try:
            # Update or create collision checker
            if self.collision_checker is None:
                logger.info(f"Creating collision checker with {len(point_cloud.points)} points")
                self.collision_checker = PointCloudCollisionChecker(
                    point_cloud,
                    robot_radius=self.robot_radius
                )
                logger.info("Collision checker created successfully")
            else:
                self.collision_checker.update_map(point_cloud)
                logger.info(f"Collision checker updated with {len(point_cloud.points)} points")

            # Update or create planner
            if self.planner is None:
                logger.info("Creating RRT* planner")
                self.planner = RRTStarPlanner(
                    collision_checker=self.collision_checker,
                    step_size=0.5,
                    search_radius=1.0,
                    max_iterations=3000
                )
                logger.info("RRT* planner created successfully")

        except Exception as e:
            logger.error(f"Error updating map: {e}")
            import traceback
            logger.error(traceback.format_exc())

    def step(self, robot_position: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Execute one step of exploration

        Args:
            robot_position: Current robot position [x, y]

        Returns:
            (exploration_active, target_velocity)
            - exploration_active: True if still exploring, False if done
            - target_velocity: [vx, vy] target velocity (None if no motion)
        """
        # Check exploration timeout
        elapsed = time.time() - self.status.exploration_start_time
        if elapsed > self.max_exploration_time:
            self.status.state = ExplorationState.EXPLORATION_COMPLETE
            return False, None

        # Update position history
        self.position_history.append(robot_position.copy())

        # Check if stuck in same state for too long
        # Different timeouts for different states
        current_time = time.time()
        if self.status.state != ExplorationState.EXECUTING_PATH and self.status.state != ExplorationState.IDLE:
            time_in_state = current_time - self.last_state_change_time

            # Use longer threshold for PLANNING_PATH (RRT* needs time)
            threshold = self.planning_stuck_threshold if self.status.state == ExplorationState.PLANNING_PATH else self.stuck_threshold

            if time_in_state > threshold:
                logger.error(f"STUCK in state {self.status.state.name} for {time_in_state:.1f}s! Forcing reset to DETECTING_FRONTIERS")
                self.status.state = ExplorationState.DETECTING_FRONTIERS
                self.status.current_frontier = None
                self.status.current_path = None
                self.last_state_change_time = current_time

        # Store previous state to detect transitions
        prev_state = self.status.state

        # State machine
        if self.status.state == ExplorationState.IDLE:
            result = self._handle_idle_state(robot_position)

        elif self.status.state == ExplorationState.DETECTING_FRONTIERS:
            result = self._handle_detecting_frontiers_state(robot_position)

        elif self.status.state == ExplorationState.PLANNING_PATH:
            result = self._handle_planning_path_state(robot_position)

        elif self.status.state == ExplorationState.EXECUTING_PATH:
            result = self._handle_executing_path_state(robot_position)

        elif self.status.state == ExplorationState.EXPLORATION_COMPLETE:
            return False, None

        elif self.status.state == ExplorationState.FAILED:
            return False, None

        else:
            return False, None

        # Track state changes
        if prev_state != self.status.state:
            logger.info(f"State transition: {prev_state.name} → {self.status.state.name}")
            self.last_state_change_time = current_time

        return result

    def _handle_idle_state(self, robot_position: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """Start exploration"""
        logger.info("IDLE → DETECTING_FRONTIERS")
        self.status.state = ExplorationState.DETECTING_FRONTIERS
        return True, None

    def _handle_detecting_frontiers_state(self, robot_position: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """Detect frontiers and select best one"""
        logger.info(f"Detecting frontiers at position {robot_position}")
        if self.collision_checker is None:
            logger.error("CRITICAL: No collision checker! Map update failed.")
            return True, None

        # Detect frontiers
        logger.info("Calling frontier_detector.detect_frontiers()...")

        try:
            frontiers = self.frontier_detector.detect_frontiers(
                self.collision_checker.point_cloud,
                robot_position
            )
            logger.info(f"Frontier detection complete. Found {len(frontiers)} frontiers")
        except Exception as e:
            logger.error(f"Frontier detection failed: {e}")
            import traceback
            logger.error(traceback.format_exc())
            # Use random exploration as fallback
            frontiers = []

        if not frontiers:
            # No frontiers detected - use random exploration as fallback
            # Generate a random goal at medium range (3-8m) for better exploration
            max_attempts = 10
            random_goal = None

            for attempt in range(max_attempts):
                angle = np.random.uniform(0, 2 * np.pi)
                distance = np.random.uniform(3.0, 8.0)  # Increased from 1-3m to 3-8m
                candidate_goal = robot_position + np.array([
                    distance * np.cos(angle),
                    distance * np.sin(angle)
                ])

                # Check if goal is collision-free
                if self.collision_checker and not self.collision_checker.is_collision(candidate_goal[0], candidate_goal[1]):
                    random_goal = candidate_goal
                    break

            if random_goal is None:
                # All attempts failed, use closer fallback
                angle = np.random.uniform(0, 2 * np.pi)
                distance = np.random.uniform(1.0, 2.0)
                random_goal = robot_position + np.array([
                    distance * np.cos(angle),
                    distance * np.sin(angle)
                ])

            logger.info(f"No frontiers - using random goal: {random_goal}")

            # Create a fake frontier at random goal
            from navigation.frontier_detector import Frontier
            self.status.current_frontier = Frontier(
                centroid=random_goal,
                points=np.array([random_goal]),
                size=1,
                score=1.0
            )
            self.status.state = ExplorationState.PLANNING_PATH
            logger.info("DETECTING_FRONTIERS → PLANNING_PATH (random exploration)")
            return True, None

        # Filter out previously explored frontiers
        frontiers = self._filter_explored_frontiers(frontiers)

        if not frontiers:
            # All frontiers explored - try random exploration
            angle = np.random.uniform(0, 2 * np.pi)
            distance = np.random.uniform(1.0, 3.0)
            random_goal = robot_position + np.array([
                distance * np.cos(angle),
                distance * np.sin(angle)
            ])

            from navigation.frontier_detector import Frontier
            self.status.current_frontier = Frontier(
                centroid=random_goal,
                points=np.array([random_goal]),
                size=1,
                score=1.0
            )
            self.status.state = ExplorationState.PLANNING_PATH
            return True, None

        # Select best frontier
        best_frontier = frontiers[0]
        self.status.current_frontier = best_frontier

        # Transition to planning
        self.status.state = ExplorationState.PLANNING_PATH
        return True, None

    def _handle_planning_path_state(self, robot_position: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """Plan path to selected frontier"""
        goal_str = f"{self.status.current_frontier.centroid}" if self.status.current_frontier else "None"
        logger.info(f"Planning path from {robot_position} to {goal_str}")

        if self.status.current_frontier is None:
            logger.warning("No frontier selected, going back to detection")
            self.status.state = ExplorationState.DETECTING_FRONTIERS
            return True, None

        # Plan path with RRT*
        goal = self.status.current_frontier.centroid
        logger.info("Calling RRT* planner...")
        path = self.planner.plan(robot_position, goal)

        if path is None:
            logger.warning("Planning FAILED - marking frontier as explored")
            # Mark as explored to avoid retrying
            self.explored_frontier_positions.append(goal)
            self.status.state = ExplorationState.DETECTING_FRONTIERS
            return True, None

        logger.info(f"Planning SUCCESS - {len(path)} waypoints")

        # Smooth path
        path = self.planner.smooth_path(path)
        logger.info(f"Path smoothed to {len(path)} waypoints")

        self.status.current_path = path
        self.status.path_index = 0

        # Transition to execution
        self.status.state = ExplorationState.EXECUTING_PATH
        logger.info("PLANNING_PATH → EXECUTING_PATH")
        return True, None

    def _handle_executing_path_state(self, robot_position: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """Execute planned path"""
        if self.status.current_path is None:
            logger.error("No path to execute!")
            self.status.state = ExplorationState.DETECTING_FRONTIERS
            return True, None

        # Get current waypoint
        if self.status.path_index >= len(self.status.current_path):
            # Path complete
            logger.info("Path complete! Frontier reached.")
            self.explored_frontier_positions.append(
                self.status.current_frontier.centroid
            )
            self.status.num_frontiers_explored += 1
            self.status.state = ExplorationState.DETECTING_FRONTIERS
            return True, None

        target_waypoint = self.status.current_path[self.status.path_index]

        # Check if waypoint reached
        distance_to_waypoint = np.linalg.norm(robot_position - target_waypoint)

        # Stuck detection: Check if robot making progress toward waypoint
        current_time = time.time()
        if self.last_waypoint_position is not None:
            last_distance = np.linalg.norm(self.last_waypoint_position - target_waypoint)
            progress = last_distance - distance_to_waypoint

            # If making progress, reset timer
            if progress > 0.05:  # At least 5cm progress
                self.last_waypoint_progress_time = current_time

        # Check if stuck (no progress for threshold time)
        time_without_progress = current_time - self.last_waypoint_progress_time
        if time_without_progress > self.waypoint_stuck_threshold:
            logger.warning(f"STUCK! No progress to waypoint for {time_without_progress:.1f}s. Skipping to next waypoint or replanning.")

            # Try skipping to next waypoint
            if self.status.path_index < len(self.status.current_path) - 1:
                self.status.path_index += 1
                logger.info(f"Skipping to waypoint {self.status.path_index}")
                self.last_waypoint_progress_time = current_time
            else:
                # Last waypoint, replan
                logger.info("Last waypoint unreachable, replanning...")
                self.status.state = ExplorationState.DETECTING_FRONTIERS
                return True, None

        self.last_waypoint_position = robot_position.copy()

        if distance_to_waypoint < self.waypoint_tolerance:
            # Move to next waypoint
            self.status.path_index += 1
            self.last_waypoint_progress_time = current_time  # Reset timer
            logger.info(f"Waypoint {self.status.path_index}/{len(self.status.current_path)} reached")
            return True, None

        # Compute velocity command towards waypoint
        direction = target_waypoint - robot_position
        direction = direction / np.linalg.norm(direction)  # Normalize

        # Simple proportional control
        max_speed = 0.3  # Reduced from 0.5 to 0.3 for more careful navigation
        speed = min(max_speed, distance_to_waypoint * 0.5)

        target_velocity = direction * speed

        logger.info(f"Moving to waypoint {self.status.path_index}: dist={distance_to_waypoint:.2f}m, vel={target_velocity}")
        return True, target_velocity

    def _filter_explored_frontiers(self, frontiers: List[Frontier]) -> List[Frontier]:
        """
        Filter out frontiers that are too close to previously explored ones

        Args:
            frontiers: List of detected frontiers

        Returns:
            Filtered list of unexplored frontiers
        """
        unexplored = []

        for frontier in frontiers:
            too_close = False

            for explored_pos in self.explored_frontier_positions:
                distance = np.linalg.norm(frontier.centroid - explored_pos)
                if distance < 0.6:  # Within 0.6m of explored frontier (reduced from 1.0m)
                    too_close = True
                    break

            if not too_close:
                unexplored.append(frontier)

        return unexplored

    def get_status(self) -> ExplorationStatus:
        """Get current exploration status"""
        return self.status

    def get_statistics(self) -> dict:
        """Get exploration statistics"""
        elapsed = time.time() - self.status.exploration_start_time

        return {
            'state': self.status.state.name,
            'num_frontiers_explored': self.status.num_frontiers_explored,
            'total_distance_traveled': self.status.total_distance_traveled,
            'elapsed_time': elapsed,
            'num_waypoints_in_path': len(self.status.current_path) if self.status.current_path else 0,
            'current_waypoint_index': self.status.path_index
        }

    def reset(self):
        """Reset exploration state"""
        self.status = ExplorationStatus(
            state=ExplorationState.IDLE,
            exploration_start_time=time.time()
        )
        self.explored_frontier_positions = []
        self.position_history = []

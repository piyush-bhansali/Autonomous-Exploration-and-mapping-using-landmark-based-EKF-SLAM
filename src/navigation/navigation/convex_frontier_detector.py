#!/usr/bin/env python3

import numpy as np
from scipy.spatial import ConvexHull, KDTree
from shapely.geometry import Polygon, MultiPolygon
from sklearn.cluster import DBSCAN
from typing import List, Optional


class SimpleFrontier:
    """A frontier cluster for exploration"""
    def __init__(self, position: np.ndarray, score: float, size: int = 1):
        self.position = position  # [x, y] - cluster centroid
        self.score = score
        self.size = size  # Number of frontier points in cluster


class ConvexFrontierDetector:
    """
    Frontier detector using convex hull for boundary extraction.

    Algorithm:
    1. Compute convex hull of point cloud
    2. Offset boundary inward by specified distance
    3. Sample frontier candidates along offset boundary
    4. Validate candidates (clearance, open directions)
    5. Cluster nearby frontiers with DBSCAN
    6. Score and rank clusters
    """

    def __init__(self,
                 robot_radius: float = 0.22,
                 frontier_spacing: float = 0.5):
        """
        Initialize convex hull frontier detector.

        Args:
            robot_radius: Robot radius for collision checking (meters)
            frontier_spacing: Spacing between frontier candidates (meters)
        """
        self.robot_radius = robot_radius
        self.frontier_spacing = frontier_spacing

        # Backend parameters (hardcoded for hallway exploration)
        self.boundary_offset = 0.10  # Small offset to preserve narrow corridors

        # Cache for performance
        self._kdtree_cache = None
        self._cached_map_hash = None

        # Store last hull for visualization
        self.last_hull_polygon = None
        self.last_offset_polygon = None

    def detect(self,
              map_points: np.ndarray,  # Nx3 array of [x,y,z] points
              robot_pos: np.ndarray,   # [x, y]
              robot_yaw: float) -> List[SimpleFrontier]:
        
        if len(map_points) < 100:
            return []

        # Extract 2D points
        points_2d = map_points[:, :2]

        # Build/cache KD-tree for collision checking
        map_hash = hash(map_points.tobytes())
        if self._cached_map_hash != map_hash:
            self._kdtree_cache = KDTree(points_2d)
            self._cached_map_hash = map_hash

        kdtree = self._kdtree_cache

        # Step 1: Compute convex hull
        try:
            hull = ConvexHull(points_2d)
            hull_coords = points_2d[hull.vertices]
            hull_polygon = Polygon(hull_coords)
        except Exception as e:
            print(f"[HULL] Failed to compute convex hull: {e}")
            return []

        # Step 2: Offset boundary inward
        offset_polygon = self._offset_polygon_inward(hull_polygon, self.boundary_offset)

        # Store for visualization
        self.last_hull_polygon = hull_polygon
        self.last_offset_polygon = offset_polygon

        if offset_polygon is None or offset_polygon.is_empty:
            return []

        # Step 3: Sample frontier candidates along offset boundary
        candidates = self._sample_boundary_points(offset_polygon, self.frontier_spacing)

        if len(candidates) == 0:
            return []

        # Step 4: Validate candidates (safety checks)
        safe_candidates = []
        for candidate in candidates:
            if self._validate_frontier_candidate(candidate, kdtree, offset_polygon):
                safe_candidates.append(candidate)

        if len(safe_candidates) == 0:
            return []

        # Step 5: Cluster nearby frontiers
        clusters = self._cluster_frontiers(safe_candidates)

        # Step 6: Score and rank clusters
        frontiers = self._score_frontier_clusters(
            clusters, robot_pos, robot_yaw
        )

        return frontiers

    def _offset_polygon_inward(self, polygon: Polygon, offset: float) -> Optional[Polygon]:
        """
        Offset polygon boundary inward by specified distance.

        Args:
            polygon: Input polygon
            offset: Offset distance (meters, positive = inward)

        Returns:
            Offset polygon, or None if result is empty
        """
        try:
            # Negative buffer = inward offset
            offset_poly = polygon.buffer(-offset)

            # Handle MultiPolygon result (can happen with complex shapes)
            if isinstance(offset_poly, MultiPolygon):
                # Take largest polygon
                largest = max(offset_poly.geoms, key=lambda p: p.area)
                return largest

            if offset_poly.is_empty:
                return None

            return offset_poly

        except Exception:
            return None

    def _sample_boundary_points(self, polygon: Polygon, spacing: float) -> List[np.ndarray]:
        """
        Sample points at regular intervals along polygon boundary.

        Args:
            polygon: Input polygon
            spacing: Distance between samples (meters)

        Returns:
            List of 2D points along boundary
        """
        candidates = []

        try:
            # Get exterior boundary
            boundary = polygon.exterior

            # Total boundary length
            length = boundary.length

            # Number of samples
            num_samples = max(3, int(length / spacing))

            # Sample at regular intervals
            for i in range(num_samples):
                # Distance along boundary
                distance = (i / num_samples) * length

                # Interpolate point at this distance
                point = boundary.interpolate(distance)

                candidates.append(np.array([point.x, point.y]))

            return candidates

        except Exception:
            return []

    def _validate_frontier_candidate(self,
                                     candidate: np.ndarray,
                                     kdtree: KDTree,
                                     hull_polygon: Polygon) -> bool:
        """
        Validate that frontier candidate is safe and has exploration potential.

        Args:
            candidate: Candidate position [x, y]
            kdtree: KD-tree of obstacle points
            hull_polygon: Convex hull polygon for outward direction check

        Returns:
            True if candidate is valid
        """
        # Check 1: Minimum clearance from obstacles
        dist, _ = kdtree.query(candidate)

        if dist < self.robot_radius * 2.0:  # 0.44m clearance for TurtleBot3
            return False

        # Check 2: Has open directions in outward cone
        if not self._has_open_direction(candidate, kdtree, hull_polygon):
            return False

        return True

    def _has_open_direction(self,
                           point: np.ndarray,
                           kdtree: KDTree,
                           hull_polygon: Polygon,
                           check_distance: float = 1.5) -> bool:
        """
        Check if frontier has open directions in outward hemisphere (±90° from hull normal).

        Args:
            point: Frontier candidate position
            kdtree: KD-tree of obstacle points
            hull_polygon: Convex hull polygon for computing outward normal
            check_distance: Distance to probe in each direction

        Returns:
            True if at least 2 directions are open within outward hemisphere
        """
        # Compute outward normal at this frontier point
        outward_normal = self._compute_boundary_normal(point, hull_polygon)

        num_directions = 8
        open_count = 0

        # Angular threshold: ±90° hemisphere
        # cos(90°) = 0.0
        cos_threshold = 0.0

        for i in range(num_directions):
            angle = i * (2 * np.pi / num_directions)
            direction = np.array([np.cos(angle), np.sin(angle)])
            check_point = point + check_distance * direction

            # Check if path is clear
            path_clear = self._is_direction_clear(point, check_point, kdtree)

            if path_clear:
                # Check if direction is within ±60° of outward normal
                dot = np.dot(direction, outward_normal)

                if dot > cos_threshold:  # Within 60° of outward direction
                    open_count += 1

        # Require at least 2 open directions
        return open_count >= 2

    def _is_direction_clear(self,
                           start: np.ndarray,
                           end: np.ndarray,
                           kdtree: KDTree) -> bool:
        """
        Check if straight line path is collision-free.

        Args:
            start: Start position
            end: End position
            kdtree: KD-tree of obstacles

        Returns:
            True if path is clear
        """
        distance = np.linalg.norm(end - start)
        num_checks = max(3, int(distance / 0.3))

        for i in range(num_checks + 1):
            alpha = i / num_checks if num_checks > 0 else 0
            check_point = start + alpha * (end - start)

            # Check clearance
            dist, _ = kdtree.query(check_point)

            if dist < self.robot_radius:
                return False

        return True

    def _compute_boundary_normal(self, point: np.ndarray, polygon: Polygon) -> np.ndarray:
        """
        Compute outward normal vector at a point on polygon boundary.

        The normal points perpendicular to the boundary, away from the polygon's interior,
        indicating the direction of unexplored space.

        Args:
            point: Point on boundary [x, y]
            polygon: The boundary polygon

        Returns:
            Normalized outward normal vector [nx, ny]
        """
        from shapely.geometry import Point as ShapelyPoint

        # Project point onto boundary to get nearest location
        boundary = polygon.exterior
        shapely_point = ShapelyPoint(point.tolist())
        distance_along = boundary.project(shapely_point)

        # Get nearby points for tangent estimation
        epsilon = 0.15  # 15cm offset
        dist_before = max(0, distance_along - epsilon)
        dist_after = min(boundary.length, distance_along + epsilon)

        point_before = boundary.interpolate(dist_before)
        point_after = boundary.interpolate(dist_after)

        # Tangent vector along boundary
        tangent = np.array([
            point_after.x - point_before.x,
            point_after.y - point_before.y
        ])
        tangent_norm = np.linalg.norm(tangent)

        if tangent_norm < 1e-6:
            # Degenerate case: point away from centroid
            centroid = np.array([polygon.centroid.x, polygon.centroid.y])
            outward = point - centroid
            return outward / (np.linalg.norm(outward) + 1e-10)

        tangent = tangent / tangent_norm

        # Normal: perpendicular to tangent (two options)
        normal_1 = np.array([-tangent[1], tangent[0]])
        normal_2 = np.array([tangent[1], -tangent[0]])

        # Choose normal pointing away from centroid
        centroid = np.array([polygon.centroid.x, polygon.centroid.y])
        to_centroid = centroid - point

        if np.dot(normal_1, to_centroid) < 0:
            return normal_1
        else:
            return normal_2

    def _cluster_frontiers(self, candidates: List[np.ndarray]) -> List[List[np.ndarray]]:
        """
        Cluster nearby frontier points using DBSCAN.

        Args:
            candidates: List of frontier candidate positions

        Returns:
            List of clusters (each cluster is list of positions)
        """
        if len(candidates) < 2:
            return [[c] for c in candidates]

        # Convert to numpy array
        points = np.array(candidates)

        # DBSCAN clustering
        clustering = DBSCAN(eps=1.0, min_samples=1).fit(points)
        labels = clustering.labels_

        # Group points by cluster
        clusters = []
        unique_labels = set(labels)

        for label in unique_labels:
            if label == -1:
                # Noise points - treat individually
                noise_points = points[labels == -1]
                for point in noise_points:
                    clusters.append([point])
            else:
                # Cluster points
                cluster_points = points[labels == label].tolist()
                clusters.append(cluster_points)

        return clusters

    def _score_frontier_clusters(self,
                                 clusters: List[List[np.ndarray]],
                                 robot_pos: np.ndarray,
                                 robot_yaw: float) -> List[SimpleFrontier]:
        """
        Score and rank frontier clusters.

        Scoring formula:
            score = 0.60 * travel_score + 0.15 * heading_score + 0.25 * size_score

        Args:
            clusters: List of frontier clusters
            robot_pos: Current robot position
            robot_yaw: Current robot heading

        Returns:
            Sorted list of SimpleFrontier objects (best first)
        """
        frontiers = []

        for cluster in clusters:
            cluster_array = np.array(cluster)
            centroid = cluster_array.mean(axis=0)
            cluster_size = len(cluster)

            # Distance score (closer is better)
            distance = np.linalg.norm(centroid - robot_pos)
            travel_score = 1.0 / (1.0 + distance)

            # Heading score (aligned with current heading is better)
            dx = centroid[0] - robot_pos[0]
            dy = centroid[1] - robot_pos[1]
            angle_to_frontier = np.arctan2(dy, dx)

            angular_diff = np.arctan2(
                np.sin(angle_to_frontier - robot_yaw),
                np.cos(angle_to_frontier - robot_yaw)
            )

            heading_score = 1.0 - (abs(angular_diff) / np.pi)

            # Size score (larger clusters are better)
            size_score = min(cluster_size / 5.0, 1.0)

            # Combined score
            score = (
                0.60 * travel_score +    # Distance (60%)
                0.15 * heading_score +   # Heading (15%)
                0.25 * size_score        # Size (25%)
            )

            frontiers.append(SimpleFrontier(centroid, score, cluster_size))

        # Sort by score (descending)
        frontiers.sort(key=lambda f: f.score, reverse=True)
        return frontiers

    def get_hull_visualization_data(self):
        """
        Get convex hull boundary data for visualization.

        Returns:
            Dictionary with:
                - 'hull_boundary': List of [x, y] points forming original hull
                - 'offset_boundary': List of [x, y] points forming offset hull
                - 'has_data': Boolean indicating if visualization data is available
        """
        if self.last_hull_polygon is None:
            return {'has_data': False}

        result = {'has_data': True}

        # Extract hull boundary coordinates
        hull_coords = list(self.last_hull_polygon.exterior.coords)
        result['hull_boundary'] = [[p[0], p[1]] for p in hull_coords]

        # Extract offset boundary coordinates if available
        if self.last_offset_polygon is not None and not self.last_offset_polygon.is_empty:
            offset_coords = list(self.last_offset_polygon.exterior.coords)
            result['offset_boundary'] = [[p[0], p[1]] for p in offset_coords]
        else:
            result['offset_boundary'] = []

        return result

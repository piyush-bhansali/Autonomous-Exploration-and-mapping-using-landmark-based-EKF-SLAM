#!/usr/bin/env python3

import numpy as np
from scipy.spatial import ConvexHull, KDTree
from shapely.geometry import Polygon, MultiPolygon
from sklearn.cluster import DBSCAN
from typing import List, Optional


class SimpleFrontier:
    
    def __init__(self, position: np.ndarray, score: float, size: int = 1):
        self.position = position  
        self.score = score
        self.size = size 

class ConvexFrontierDetector:

    def __init__(self, robot_radius: float = 0.22):

        self.robot_radius = robot_radius

        self.frontier_spacing = 0.5  
        self.boundary_offset = 0.10  

        self._kdtree_cache = None
        self._cached_map_hash = None

        self.last_hull_polygon = None
        self.last_offset_polygon = None

    def detect(self,
              map_points: np.ndarray,  # Nx3 array of [x,y,z] points
              robot_pos: np.ndarray,   # [x, y]
              robot_yaw: float,
              obstacle_kdtree: Optional[KDTree] = None) -> List[SimpleFrontier]:
        
        if len(map_points) < 100:
            return []

        points_2d = map_points[:, :2]

        if obstacle_kdtree is not None:
            kdtree = obstacle_kdtree
        else:
            map_hash = hash(map_points.tobytes())
            if self._cached_map_hash != map_hash:
                self._kdtree_cache = KDTree(points_2d)
                self._cached_map_hash = map_hash
            kdtree = self._kdtree_cache

        try:
            hull = ConvexHull(points_2d)
            hull_coords = points_2d[hull.vertices]
            hull_polygon = Polygon(hull_coords)
        except Exception as e:
            print(f"[HULL] Failed to compute convex hull: {e}")
            return []

        offset_polygon = self._offset_polygon_inward(hull_polygon, self.boundary_offset)

        self.last_hull_polygon = hull_polygon
        self.last_offset_polygon = offset_polygon

        if offset_polygon is None or offset_polygon.is_empty:
            return []

        candidates = self._sample_boundary_points(offset_polygon, self.frontier_spacing)

        if len(candidates) == 0:
            return []

        safe_candidates = []
        for candidate in candidates:
            if self._validate_frontier_candidate(candidate, kdtree, offset_polygon):
                safe_candidates.append(candidate)

        if len(safe_candidates) == 0:
            return []

        clusters = self._cluster_frontiers(safe_candidates)

        frontiers = self._score_frontier_clusters(
            clusters, robot_pos, robot_yaw
        )

        return frontiers

    def _offset_polygon_inward(self, polygon: Polygon, offset: float) -> Optional[Polygon]:
       
        try:
            offset_poly = polygon.buffer(-offset)

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
       
        candidates = []

        try:
            boundary = polygon.exterior

            length = boundary.length

            num_samples = max(3, int(length / spacing))

            for i in range(num_samples):
                
                distance = (i / num_samples) * length

                point = boundary.interpolate(distance)

                candidates.append(np.array([point.x, point.y]))

            return candidates

        except Exception:
            return []

    def _validate_frontier_candidate(self,
                                     candidate: np.ndarray,
                                     kdtree: KDTree,
                                     hull_polygon: Polygon) -> bool:
       
        dist, _ = kdtree.query(candidate)

        if dist < self.robot_radius * 2.0:  
            return False

        if not self._has_open_direction(candidate, kdtree, hull_polygon):
            return False

        return True

    def _has_open_direction(self,
                           point: np.ndarray,
                           kdtree: KDTree,
                           hull_polygon: Polygon,
                           check_distance: float = 1.5) -> bool:
        
        outward_normal = self._compute_boundary_normal(point, hull_polygon)

        num_directions = 8
        open_count = 0

        cos_threshold = 0.0

        for i in range(num_directions):
            angle = i * (2 * np.pi / num_directions)
            direction = np.array([np.cos(angle), np.sin(angle)])
            check_point = point + check_distance * direction

            # Check if path is clear
            path_clear = self._is_direction_clear(point, check_point, kdtree)

            if path_clear:
                
                dot = np.dot(direction, outward_normal)

                if dot > cos_threshold:  
                    open_count += 1

        return open_count >= 2

    def _is_direction_clear(self,
                           start: np.ndarray,
                           end: np.ndarray,
                           kdtree: KDTree) -> bool:
        
        distance = np.linalg.norm(end - start)
        num_checks = max(3, int(distance / 0.3))

        for i in range(num_checks + 1):
            alpha = i / num_checks if num_checks > 0 else 0
            check_point = start + alpha * (end - start)

            dist, _ = kdtree.query(check_point)

            if dist < self.robot_radius:
                return False

        return True

    def _compute_boundary_normal(self, point: np.ndarray, polygon: Polygon) -> np.ndarray:
        
        from shapely.geometry import Point as ShapelyPoint

        boundary = polygon.exterior
        shapely_point = ShapelyPoint(point.tolist())
        distance_along = boundary.project(shapely_point)

        epsilon = 0.15  # 15cm offset
        dist_before = max(0, distance_along - epsilon)
        dist_after = min(boundary.length, distance_along + epsilon)

        point_before = boundary.interpolate(dist_before)
        point_after = boundary.interpolate(dist_after)

        tangent = np.array([
            point_after.x - point_before.x,
            point_after.y - point_before.y
        ])
        tangent_norm = np.linalg.norm(tangent)

        if tangent_norm < 1e-6:
            
            centroid = np.array([polygon.centroid.x, polygon.centroid.y])
            outward = point - centroid
            return outward / (np.linalg.norm(outward) + 1e-10)

        tangent = tangent / tangent_norm

        normal_1 = np.array([-tangent[1], tangent[0]])
        normal_2 = np.array([tangent[1], -tangent[0]])

        centroid = np.array([polygon.centroid.x, polygon.centroid.y])
        to_centroid = centroid - point

        if np.dot(normal_1, to_centroid) < 0:
            return normal_1
        else:
            return normal_2

    def _cluster_frontiers(self, candidates: List[np.ndarray]) -> List[List[np.ndarray]]:
        
        if len(candidates) < 2:
            return [[c] for c in candidates]

        points = np.array(candidates)

        clustering = DBSCAN(eps=1.0, min_samples=1).fit(points)
        labels = clustering.labels_

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
        
        frontiers = []

        for cluster in clusters:
            cluster_array = np.array(cluster)
            centroid = cluster_array.mean(axis=0)
            cluster_size = len(cluster)

            distance = np.linalg.norm(centroid - robot_pos)
            travel_score = 1.0 / (1.0 + distance)

            dx = centroid[0] - robot_pos[0]
            dy = centroid[1] - robot_pos[1]
            angle_to_frontier = np.arctan2(dy, dx)

            angular_diff = np.arctan2(
                np.sin(angle_to_frontier - robot_yaw),
                np.cos(angle_to_frontier - robot_yaw)
            )

            heading_score = 1.0 - (abs(angular_diff) / np.pi)

            size_score = min(cluster_size / 5.0, 1.0)

            score = (
                0.60 * travel_score +    # Distance (60%)
                0.15 * heading_score +   # Heading (15%)
                0.25 * size_score        # Size (25%)
            )

            frontiers.append(SimpleFrontier(centroid, score, cluster_size))

        frontiers.sort(key=lambda f: f.score, reverse=True)
        return frontiers

    def get_hull_visualization_data(self):
        
        if self.last_hull_polygon is None:
            return {'has_data': False}

        result = {'has_data': True}

        hull_coords = list(self.last_hull_polygon.exterior.coords)
        result['hull_boundary'] = [[p[0], p[1]] for p in hull_coords]

        if self.last_offset_polygon is not None and not self.last_offset_polygon.is_empty:
            offset_coords = list(self.last_offset_polygon.exterior.coords)
            result['offset_boundary'] = [[p[0], p[1]] for p in offset_coords]
        else:
            result['offset_boundary'] = []

        return result

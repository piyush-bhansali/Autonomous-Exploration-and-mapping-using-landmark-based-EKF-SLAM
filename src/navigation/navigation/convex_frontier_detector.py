#!/usr/bin/env python3

import numpy as np
from scipy.spatial import ConvexHull, KDTree
from shapely.geometry import Polygon, MultiPolygon, Point as ShapelyPoint
from sklearn.cluster import DBSCAN
from typing import List, Optional, Dict, Tuple


class SimpleFrontier:
    
    def __init__(self, position: np.ndarray, score: float, size: int = 1):
        self.position = position
        self.score = score
        self.size = size


class ConvexFrontierDetector:

    def __init__(self, robot_radius: float = 0.22):
        self.robot_radius = robot_radius
        self.frontier_spacing = 0.5
        self.boundary_offset = 0.5

        self._kdtree_cache = None
        self._cached_map_hash = None

        self.last_hull_polygon = None
        self.last_offset_polygon = None

    def detect(self,
              map_points: np.ndarray,
              robot_pos: np.ndarray,
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
            hull_polygon = Polygon(points_2d[hull.vertices])
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

        safe_candidates = [
            candidate for candidate in candidates
            if self._validate_frontier_candidate(candidate, kdtree, offset_polygon)
        ]

        if len(safe_candidates) == 0:
            return []

        clusters = self._cluster_frontiers(safe_candidates)

        return self._score_frontier_clusters(clusters, robot_pos, robot_yaw)

    def _offset_polygon_inward(self, polygon: Polygon, offset: float) -> Optional[Polygon]:
        
        try:
            offset_poly = polygon.buffer(-offset)

            if isinstance(offset_poly, MultiPolygon):
                # Return largest component if split into multiple polygons
                return max(offset_poly.geoms, key=lambda p: p.area)

            return offset_poly if not offset_poly.is_empty else None

        except Exception:
            return None

    def _sample_boundary_points(self, polygon: Polygon, spacing: float) -> List[np.ndarray]:
        
        try:
            boundary = polygon.exterior
            num_samples = max(3, int(boundary.length / spacing))

            candidates = []
            for i in range(num_samples):
                distance = (i / num_samples) * boundary.length
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

        if dist < 0.5:
            return False

        return self._has_open_direction(candidate, kdtree, hull_polygon)

    def _has_open_direction(self,
                           point: np.ndarray,
                           kdtree: KDTree,
                           hull_polygon: Polygon,
                           check_distance: float = 1.5) -> bool:
        
        boundary = hull_polygon.exterior
        shapely_point = ShapelyPoint(point.tolist())
        distance_along = boundary.project(shapely_point)

        epsilon = 0.1  
        dist_before = max(0, distance_along - epsilon)
        dist_after = min(boundary.length, distance_along + epsilon)

        point_before = boundary.interpolate(dist_before)
        point_after = boundary.interpolate(dist_after)

        tangent = np.array([point_after.x - point_before.x,
                           point_after.y - point_before.y])
        tangent_norm = np.linalg.norm(tangent)

        if tangent_norm < 1e-6:
            centroid = np.array([hull_polygon.centroid.x, hull_polygon.centroid.y])
            radial_direction = point - centroid
            radial_norm = np.linalg.norm(radial_direction)
            if radial_norm < 1e-10:
                return False  

            normal = radial_direction / radial_norm
        else:
            
            tangent = tangent / tangent_norm
            normal = np.array([-tangent[1], tangent[0]])

        normal_angle = np.arctan2(normal[1], normal[0])

        relative_angles = [-60, -30, 0, 30, 60]
        open_count = 0

        for rel_angle_deg in relative_angles:
            global_angle = normal_angle + np.radians(rel_angle_deg)
            direction = np.array([np.cos(global_angle), np.sin(global_angle)])
            check_point = point + check_distance * direction

            if self._is_direction_clear(point, check_point, kdtree):
                open_count += 1

        return open_count >= 3

    def _is_direction_clear(self,
                           start: np.ndarray,
                           end: np.ndarray,
                           kdtree: KDTree) -> bool:
        
        distance = np.linalg.norm(end - start)
        num_checks = max(3, int(distance / 0.3))

        for i in range(num_checks + 1):
            alpha = i / num_checks
            check_point = start + alpha * (end - start)

            dist, _ = kdtree.query(check_point)
            if dist < self.robot_radius:
                return False

        return True

    def _cluster_frontiers(self, candidates: List[np.ndarray]) -> List[Tuple[np.ndarray, int]]:
       
        if len(candidates) < 2:
            return [(candidates[0], 1)]

        points = np.array(candidates)
        clustering = DBSCAN(eps=1.0, min_samples=1).fit(points)
        labels = clustering.labels_

        clusters = []
        for label in set(labels):
            cluster_points = points[labels == label]
            centroid = cluster_points.mean(axis=0)
            cluster_size = len(cluster_points)
            clusters.append((centroid, cluster_size))

        return clusters

    def _score_frontier_clusters(self,
                                 clusters: List[Tuple[np.ndarray, int]],
                                 robot_pos: np.ndarray,
                                 robot_yaw: float) -> List[SimpleFrontier]:
       
        if len(clusters) == 0:
            return []

        angles = []
        for centroid, _ in clusters:
            angle_to_frontier = np.arctan2(centroid[1] - robot_pos[1],
                                          centroid[0] - robot_pos[0])
            angles.append(angle_to_frontier)

        angle_points = np.array([[np.cos(a), np.sin(a)] for a in angles])

        # Chord distance on unit circle: d = 2·sin(Δθ/2), so 60° → d = 1.0
        direction_clustering = DBSCAN(eps=1.0, min_samples=1).fit(angle_points)
        direction_labels = direction_clustering.labels_

        directional_counts = {}
        for label in set(direction_labels):
            count = np.sum(direction_labels == label)
            directional_counts[label] = count

        total_frontier_count = len(clusters)

        frontiers = []
        for idx, (centroid, cluster_size) in enumerate(clusters):
            
            distance = np.linalg.norm(centroid - robot_pos)
            travel_score = 1.0 / (1.0 + distance)

            angle_to_frontier = angles[idx]
            angular_diff = np.arctan2(np.sin(angle_to_frontier - robot_yaw),
                                     np.cos(angle_to_frontier - robot_yaw))
            heading_score = 1.0 - (abs(angular_diff) / np.pi)

            direction_label = direction_labels[idx]
            directional_count = directional_counts[direction_label]
            size_score = directional_count / total_frontier_count

            # Combined score
            score = (0.60 * travel_score +
                    0.15 * heading_score +
                    0.25 * size_score)

            frontiers.append(SimpleFrontier(centroid, score, cluster_size))

        frontiers.sort(key=lambda f: f.score, reverse=True)
        return frontiers

    def get_hull_visualization_data(self) -> Dict:
        
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

#!/usr/bin/env python3


import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud2


def calculate_path_length(path: list) -> float:
    
    total = 0.0
    for i in range(len(path) - 1):
        total += np.linalg.norm(path[i+1] - path[i])
    return total


def find_nearest_waypoint_index(path: np.ndarray, robot_pos: np.ndarray, lookahead: int = 2) -> int:
    
    if path is None or len(path) == 0:
        return 0

    distances = np.linalg.norm(path - robot_pos, axis=1)
    nearest_idx = np.argmin(distances)
    target_idx = nearest_idx + lookahead
    target_idx = min(target_idx, len(path) - 1)

    return target_idx


def convert_scan_to_points(scan: LaserScan) -> np.ndarray:
    
    ranges = np.array(scan.ranges)
    angle_min = scan.angle_min
    angle_increment = scan.angle_increment

    valid_mask = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)

    num_readings = len(ranges)
    angles = angle_min + np.arange(num_readings) * angle_increment

    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    points = np.column_stack([x[valid_mask], y[valid_mask]])

    return points


def transform_points_to_global(points: np.ndarray, robot_pos: np.ndarray, robot_yaw: float) -> np.ndarray:
    
    if points is None or len(points) == 0:
        return np.array([]).reshape(0, 2)

    # Rotation matrix
    cos_yaw = np.cos(robot_yaw)
    sin_yaw = np.sin(robot_yaw)

    # Apply rotation and translation
    x_global = robot_pos[0] + points[:, 0] * cos_yaw - points[:, 1] * sin_yaw
    y_global = robot_pos[1] + points[:, 0] * sin_yaw + points[:, 1] * cos_yaw

    points_global = np.column_stack([x_global, y_global])

    return points_global


def parse_pointcloud2(msg: PointCloud2) -> np.ndarray:
   
    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32)
    ])

    # Reshape raw data into structured array
    points_structured = np.frombuffer(msg.data, dtype=dtype)

    # Convert to regular Nx3 array
    points = np.zeros((len(points_structured), 3), dtype=np.float32)
    points[:, 0] = points_structured['x']
    points[:, 1] = points_structured['y']
    points[:, 2] = points_structured['z']

    return points


def check_scan_for_obstacles(
    scan: LaserScan,
    emergency_distance: float,
    angular_range: float
) -> tuple:
    
    if scan is None:
        return False, float('inf'), 0.0

    angle_min = scan.angle_min
    angle_max = scan.angle_max
    angle_increment = scan.angle_increment
    ranges = np.array(scan.ranges)

    max_range = scan.range_max
    ranges = np.where(np.isfinite(ranges), ranges, max_range)

    num_readings = len(ranges)

    half_angular_range = angular_range / 2.0

    angles = angle_min + np.arange(num_readings) * angle_increment
    front_mask = np.abs(angles) <= half_angular_range
    left_mask = (angles > half_angular_range) & (angles <= np.pi/2)
    right_mask = (angles < -half_angular_range) & (angles >= -np.pi/2)

    front_ranges = ranges[front_mask]
    left_ranges = ranges[left_mask]
    right_ranges = ranges[right_mask]

    min_front = np.min(front_ranges) if len(front_ranges) > 0 else max_range
    min_left = np.min(left_ranges) if len(left_ranges) > 0 else max_range
    min_right = np.min(right_ranges) if len(right_ranges) > 0 else max_range

    obstacle_detected = min_front < emergency_distance

    avoidance_direction = 0.0
    if obstacle_detected:
        if min_left > min_right:
            avoidance_direction = -1.0
        else:
            avoidance_direction = 1.0

    return obstacle_detected, min_front, avoidance_direction


def check_path_deviation(
    robot_pos: np.ndarray,
    current_path: np.ndarray,
    current_waypoint_index: int,
    deviation_threshold: float
) -> tuple:
    
    if current_path is None or len(current_path) == 0:
        return False, 0.0

    remaining_path = current_path[current_waypoint_index:]

    if len(remaining_path) == 0:
        return False, 0.0

    distances = np.linalg.norm(remaining_path - robot_pos, axis=1)
    min_distance = np.min(distances)

    # Check if deviation exceeds threshold
    is_deviated = min_distance > deviation_threshold

    return is_deviated, min_distance


def check_if_stuck(
    robot_pos: np.ndarray,
    last_check_position: np.ndarray,
    time_since_check: float,
    stuck_check_window: float,
    stuck_distance_threshold: float
) -> tuple:
    
    if last_check_position is None:
        return False, 0.0

    if time_since_check < stuck_check_window:
        return False, 0.0

    # Check if robot has moved enough
    distance_moved = np.linalg.norm(robot_pos - last_check_position)

    is_stuck = distance_moved < stuck_distance_threshold

    return is_stuck, distance_moved

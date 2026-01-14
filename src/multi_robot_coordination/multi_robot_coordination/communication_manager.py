#!/usr/bin/env python3

import numpy as np
from typing import Dict, Optional, Callable
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class CommunicationManager:
    """
    Manages ROS2 communication for multi-robot coordination.

    Handles:
    - Broadcasting robot info (position, status)
    - Listening for other robots
    - Alignment service (server + clients)
    - Environment bounds sharing
    """

    def __init__(self, node: Node, robot_id: str):
        """
        Initialize communication manager.

        Args:
            node: ROS2 node
            robot_id: Robot identifier
        """
        self.node = node
        self.robot_id = robot_id
        self.logger = node.get_logger()

        # Import messages (delayed to avoid circular imports)
        from multi_robot_coordination.msg import RobotInfo, EnvironmentBounds, FrameUpdate
        from multi_robot_coordination.srv import RequestAlignment

        self.RobotInfo = RobotInfo
        self.EnvironmentBounds = EnvironmentBounds
        self.FrameUpdate = FrameUpdate
        self.RequestAlignment = RequestAlignment

        # Publishers
        self.info_pub = node.create_publisher(
            RobotInfo,
            '/multi_robot/robot_info',
            10
        )

        self.bounds_pub = node.create_publisher(
            EnvironmentBounds,
            '/multi_robot/environment_bounds',
            10
        )

        # Subscribers
        self.info_sub = node.create_subscription(
            RobotInfo,
            '/multi_robot/robot_info',
            self._robot_info_callback,
            10
        )

        self.bounds_sub = node.create_subscription(
            EnvironmentBounds,
            '/multi_robot/environment_bounds',
            self._environment_bounds_callback,
            10
        )

        # Alignment service (server)
        self.alignment_service = node.create_service(
            RequestAlignment,
            f'/{robot_id}/request_alignment',
            self._handle_alignment_request
        )

        # Alignment service clients (one per discovered robot)
        self.alignment_clients: Dict[str, rclpy.client.Client] = {}

        # Callbacks (set externally)
        self.robot_info_callback: Optional[Callable] = None
        self.environment_bounds_callback: Optional[Callable] = None
        self.alignment_request_handler: Optional[Callable] = None

        # Storage
        self.latest_info_from_robots: Dict[str, RobotInfo] = {}
        self.latest_bounds_from_robots: Dict[str, EnvironmentBounds] = {}

        self.logger.info(f'CommunicationManager initialized for {robot_id}')

    def _robot_info_callback(self, msg):
        """Internal callback for robot info messages."""
        if msg.robot_id == self.robot_id:
            return  # Ignore own messages

        # Store latest info
        self.latest_info_from_robots[msg.robot_id] = msg

        # Call external callback if set
        if self.robot_info_callback is not None:
            self.robot_info_callback(msg)

    def _environment_bounds_callback(self, msg):
        """Internal callback for environment bounds messages."""
        if msg.robot_id == self.robot_id:
            return

        # Store latest bounds
        self.latest_bounds_from_robots[msg.robot_id] = msg

        # Call external callback if set
        if self.environment_bounds_callback is not None:
            self.environment_bounds_callback(msg)

    def _handle_alignment_request(self, request, response):
        """Internal service callback for alignment requests."""
        # Call external handler if set
        if self.alignment_request_handler is not None:
            success, transform, metrics = self.alignment_request_handler(request)

            response.success = success
            response.transform_matrix = transform.flatten().tolist()
            response.fitness = metrics.get('fitness', 0.0)
            response.inlier_ratio = metrics.get('inlier_ratio', 0.0)
            response.overlap = metrics.get('overlap', 0.0)
            response.failure_reason = metrics.get('reason', '')
        else:
            response.success = False
            response.failure_reason = 'No handler set'

        return response

    def broadcast_robot_info(self,
                            x: float,
                            y: float,
                            theta: float,
                            quaternion: np.ndarray,
                            reference_frame_id: str,
                            num_submaps: int):
        """
        Broadcast robot information.

        Args:
            x, y: Position in reference frame
            theta: Heading angle (radians)
            quaternion: Orientation [qx, qy, qz, qw]
            reference_frame_id: Reference frame ID
            num_submaps: Number of submaps
        """
        msg = self.RobotInfo()
        msg.robot_id = self.robot_id
        msg.x = x
        msg.y = y
        msg.theta = theta
        msg.quaternion = quaternion.tolist()
        msg.reference_frame_id = reference_frame_id
        msg.num_submaps = num_submaps
        msg.timestamp = self.node.get_clock().now().to_msg()

        self.info_pub.publish(msg)

    def broadcast_environment_bounds(self,
                                     x_min: float,
                                     x_max: float,
                                     y_min: float,
                                     y_max: float):
        """
        Broadcast environment boundary information.

        Args:
            x_min, x_max: X bounds
            y_min, y_max: Y bounds
        """
        msg = self.EnvironmentBounds()
        msg.robot_id = self.robot_id
        msg.x_min = x_min
        msg.x_max = x_max
        msg.y_min = y_min
        msg.y_max = y_max
        msg.timestamp = self.node.get_clock().now().to_msg()

        self.bounds_pub.publish(msg)

    def request_alignment(self,
                         target_robot_id: str,
                         current_scan: LaserScan,
                         position: np.ndarray,
                         timeout_sec: float = 5.0) -> Optional[Dict]:
        """
        Request alignment with another robot.

        Args:
            target_robot_id: ID of robot to align with
            current_scan: Current LiDAR scan
            position: Current position [x, y, theta]
            timeout_sec: Service call timeout

        Returns:
            Response dictionary or None if failed
        """
        # Create client if doesn't exist
        if target_robot_id not in self.alignment_clients:
            self.alignment_clients[target_robot_id] = self.node.create_client(
                self.RequestAlignment,
                f'/{target_robot_id}/request_alignment'
            )

        client = self.alignment_clients[target_robot_id]

        # Wait for service
        if not client.wait_for_service(timeout_sec=2.0):
            self.logger.warn(f'Alignment service for {target_robot_id} not available')
            return None

        # Create request
        request = self.RequestAlignment.Request()
        request.requesting_robot_id = self.robot_id
        request.current_scan_ranges = current_scan.ranges
        request.scan_angle_min = current_scan.angle_min
        request.scan_angle_max = current_scan.angle_max
        request.scan_angle_increment = current_scan.angle_increment
        request.range_min = current_scan.range_min
        request.range_max = current_scan.range_max
        request.position = position.tolist()

        # Call service synchronously
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

            if future.result() is not None:
                response = future.result()

                result = {
                    'success': response.success,
                    'transform': np.array(response.transform_matrix).reshape(4, 4),
                    'fitness': response.fitness,
                    'inlier_ratio': response.inlier_ratio,
                    'overlap': response.overlap,
                    'failure_reason': response.failure_reason
                }

                if response.success:
                    self.logger.info(
                        f'Alignment with {target_robot_id} successful: '
                        f'fitness={response.fitness:.3f}'
                    )
                else:
                    self.logger.warn(
                        f'Alignment with {target_robot_id} failed: {response.failure_reason}'
                    )

                return result
            else:
                self.logger.error(f'Alignment service call to {target_robot_id} failed')
                return None

        except Exception as e:
            self.logger.error(f'Exception during alignment request: {e}')
            return None

    def get_known_robots(self) -> list:
        """Get list of known robot IDs."""
        return list(self.latest_info_from_robots.keys())

    def get_robot_info(self, robot_id: str) -> Optional[Dict]:
        """
        Get latest info for a specific robot.

        Returns:
            Dictionary with robot info or None
        """
        if robot_id not in self.latest_info_from_robots:
            return None

        msg = self.latest_info_from_robots[robot_id]

        return {
            'robot_id': msg.robot_id,
            'position': np.array([msg.x, msg.y]),
            'theta': msg.theta,
            'quaternion': np.array(msg.quaternion),
            'reference_frame_id': msg.reference_frame_id,
            'num_submaps': msg.num_submaps,
            'timestamp': msg.timestamp
        }

    def get_robot_distance(self, robot_id: str, own_position: np.ndarray) -> Optional[float]:
        """
        Get distance to another robot.

        Args:
            robot_id: Target robot ID
            own_position: Own position [x, y]

        Returns:
            Distance in meters or None
        """
        info = self.get_robot_info(robot_id)
        if info is None:
            return None

        other_pos = info['position']
        distance = np.linalg.norm(own_position - other_pos)

        return distance

    def cleanup(self):
        """Cleanup resources."""
        for client in self.alignment_clients.values():
            self.node.destroy_client(client)

        self.alignment_clients.clear()

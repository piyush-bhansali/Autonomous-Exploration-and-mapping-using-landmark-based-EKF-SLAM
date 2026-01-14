#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from enum import Enum
import time
from typing import Dict, Optional

from multi_robot_coordination.frame_manager import DistributedFrameManager
from multi_robot_coordination.communication_manager import CommunicationManager
from multi_robot_coordination.inter_robot_icp import InterRobotICP
from multi_robot_coordination import coordination_utils as utils


class CoordinatorState(Enum):
    """State machine for robot coordination."""
    SOLO = 1              # No other robots detected
    DETECTING = 2         # Other robot(s) visible, checking proximity
    ALIGNING = 3          # Performing ICP alignment
    ALIGNED = 4           # Successfully aligned to shared reference
    COLLABORATIVE = 5     # Multiple robots, exchanging updates


class RobotCoordinator(Node):
    """
    Main coordination node for multi-robot SLAM.

    Manages:
    - Robot detection and proximity checking
    - Inter-robot alignment via ICP
    - Distributed frame graph
    - Communication with other robots
    """

    def __init__(self):
        super().__init__('robot_coordinator')

        # Parameters
        self.declare_parameter('robot_name', 'tb3_1')
        self.declare_parameter('proximity_threshold', 3.0)
        self.declare_parameter('alignment_timeout', 10.0)
        self.declare_parameter('info_broadcast_rate', 1.0)
        self.declare_parameter('bounds_broadcast_rate', 0.1)

        self.robot_id = self.get_parameter('robot_name').value
        self.proximity_threshold = self.get_parameter('proximity_threshold').value
        self.alignment_timeout = self.get_parameter('alignment_timeout').value
        self.info_rate = self.get_parameter('info_broadcast_rate').value
        self.bounds_rate = self.get_parameter('bounds_broadcast_rate').value

        # State
        self.state = CoordinatorState.SOLO
        self.alignment_in_progress = False

        # Current robot state
        self.current_position = np.array([0.0, 0.0])
        self.current_theta = 0.0
        self.current_quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self.current_pose_dict = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                                 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}

        # Latest scan
        self.latest_scan: Optional[LaserScan] = None

        # Environment bounds
        self.environment_bounds = {'x_min': 0.0, 'x_max': 0.0, 'y_min': 0.0, 'y_max': 0.0}

        # Number of submaps (would be received from mapping module)
        self.num_submaps = 0

        # Nearby robots tracking
        self.nearby_robots: Dict[str, Dict] = {}
        self.aligned_robots = set()  # Robots we've already aligned with

        # Core components
        self.frame_manager = DistributedFrameManager(self.robot_id, self.get_logger())
        self.comm_manager = CommunicationManager(self, self.robot_id)
        self.icp_aligner = InterRobotICP(logger=self.get_logger())

        # Set communication callbacks
        self.comm_manager.robot_info_callback = self._handle_robot_info
        self.comm_manager.alignment_request_handler = self._handle_alignment_request

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.robot_id}/odom',
            self._odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_id}/scan',
            self._scan_callback,
            10
        )

        # Publishers (for frame updates to mapping module)
        from multi_robot_coordination.msg import FrameUpdate
        self.frame_update_pub = self.create_publisher(
            FrameUpdate,
            f'/{self.robot_id}/frame_update',
            10
        )

        # Timers
        self.create_timer(1.0 / self.info_rate, self._broadcast_info_timer)
        self.create_timer(1.0 / self.bounds_rate, self._broadcast_bounds_timer)
        self.create_timer(0.5, self._check_for_alignment_timer)
        self.create_timer(2.0, self._update_state_timer)

        self.get_logger().info(f'RobotCoordinator initialized for {self.robot_id}')
        self.get_logger().info(f'State: {self.state.name}')

    def _odom_callback(self, msg: Odometry):
        """Update current robot pose from odometry."""
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y

        quat = msg.pose.pose.orientation
        self.current_quaternion = np.array([quat.x, quat.y, quat.z, quat.w])

        # Extract yaw
        _, _, self.current_theta = utils.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)

        # Update pose dict
        self.current_pose_dict = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'qx': quat.x,
            'qy': quat.y,
            'qz': quat.z,
            'qw': quat.w
        }

    def _scan_callback(self, msg: LaserScan):
        """Store latest scan for alignment."""
        self.latest_scan = msg

    def _handle_robot_info(self, msg):
        """Process robot info from other robots."""
        robot_id = msg.robot_id

        # Calculate distance
        other_pos = np.array([msg.x, msg.y])
        distance = np.linalg.norm(self.current_position - other_pos)

        # Update nearby robots
        self.nearby_robots[robot_id] = {
            'distance': distance,
            'position': other_pos,
            'theta': msg.theta,
            'quaternion': np.array(msg.quaternion),
            'reference_frame': msg.reference_frame_id,
            'num_submaps': msg.num_submaps,
            'last_seen': self.get_clock().now()
        }

    def _broadcast_info_timer(self):
        """Periodically broadcast robot information."""
        # Transform position to reference frame if needed
        pos_in_ref = self.current_position.copy()
        theta_in_ref = self.current_theta
        quat_in_ref = self.current_quaternion.copy()

        # If we've aligned, transform to reference frame
        if self.state in [CoordinatorState.ALIGNED, CoordinatorState.COLLABORATIVE]:
            T_ref_to_self = self.frame_manager.T_ref_to_self
            current_transform = utils.pose_dict_to_transform(self.current_pose_dict)
            pose_in_ref_transform = T_ref_to_self @ current_transform
            pose_in_ref = utils.transform_to_pose_dict(pose_in_ref_transform)

            pos_in_ref = np.array([pose_in_ref['x'], pose_in_ref['y']])
            _, _, theta_in_ref = utils.quaternion_to_euler(
                pose_in_ref['qx'], pose_in_ref['qy'],
                pose_in_ref['qz'], pose_in_ref['qw']
            )
            quat_in_ref = np.array([pose_in_ref['qx'], pose_in_ref['qy'],
                                   pose_in_ref['qz'], pose_in_ref['qw']])

        self.comm_manager.broadcast_robot_info(
            x=pos_in_ref[0],
            y=pos_in_ref[1],
            theta=theta_in_ref,
            quaternion=quat_in_ref,
            reference_frame_id=self.frame_manager.reference_frame_id,
            num_submaps=self.num_submaps
        )

    def _broadcast_bounds_timer(self):
        """Periodically broadcast environment bounds."""
        self.comm_manager.broadcast_environment_bounds(
            x_min=self.environment_bounds['x_min'],
            x_max=self.environment_bounds['x_max'],
            y_min=self.environment_bounds['y_min'],
            y_max=self.environment_bounds['y_max']
        )

    def _check_for_alignment_timer(self):
        """Periodically check if alignment needed."""
        if self.alignment_in_progress:
            return

        # Clean up stale robot detections (> 3 seconds old)
        current_time = self.get_clock().now()
        stale_robots = []
        for robot_id, info in self.nearby_robots.items():
            age = (current_time - info['last_seen']).nanoseconds / 1e9
            if age > 3.0:
                stale_robots.append(robot_id)

        for robot_id in stale_robots:
            del self.nearby_robots[robot_id]

        # Check for nearby robots
        for robot_id, info in self.nearby_robots.items():
            # Skip if already aligned
            if robot_id in self.aligned_robots:
                continue

            # Check proximity
            if info['distance'] < self.proximity_threshold:
                self.get_logger().info(
                    f'Robot {robot_id} nearby (distance={info["distance"]:.2f}m), '
                    f'initiating alignment...'
                )
                self._initiate_alignment(robot_id)
                break  # Align with one robot at a time

    def _update_state_timer(self):
        """Update state machine."""
        if len(self.nearby_robots) == 0:
            if self.state != CoordinatorState.SOLO and not self.alignment_in_progress:
                # No robots nearby anymore, but keep aligned state
                if len(self.aligned_robots) > 0:
                    self.state = CoordinatorState.ALIGNED
                else:
                    self.state = CoordinatorState.SOLO
        else:
            if self.state == CoordinatorState.SOLO:
                self.state = CoordinatorState.DETECTING

            if len(self.aligned_robots) > 0:
                self.state = CoordinatorState.COLLABORATIVE

    def _initiate_alignment(self, other_robot_id: str):
        """Initiate alignment with another robot."""
        if self.latest_scan is None:
            self.get_logger().warn('No scan available for alignment')
            return

        self.alignment_in_progress = True
        self.state = CoordinatorState.ALIGNING

        # Request alignment via service
        position_array = np.array([self.current_position[0],
                                  self.current_position[1],
                                  self.current_theta])

        result = self.comm_manager.request_alignment(
            target_robot_id=other_robot_id,
            current_scan=self.latest_scan,
            position=position_array,
            timeout_sec=self.alignment_timeout
        )

        if result is not None and result['success']:
            # Alignment succeeded
            transform = result['transform']

            self.get_logger().info(
                f'\n{"="*80}\n'
                f'  ✓ INTER-ROBOT ALIGNMENT SUCCESSFUL\n'
                f'{"="*80}\n'
                f'  This Robot:     {self.robot_id}\n'
                f'  Target Robot:   {other_robot_id}\n'
                f'  ICP Fitness:    {result["fitness"]:.4f}\n'
                f'  Inlier Ratio:   {result["inlier_ratio"]:.4f}\n'
                f'  Overlap:        {result["overlap"]:.4f}\n'
                f'{"-"*80}'
            )

            # Add to pose graph
            self.frame_manager.add_edge(self.robot_id, other_robot_id, transform)

            # Check if need to update reference frame
            other_ref_frame = self.nearby_robots[other_robot_id]['reference_frame']

            if other_ref_frame != self.robot_id:
                # Other robot uses different reference, adopt it
                self.get_logger().info(
                    f'Updating reference frame: {self.frame_manager.reference_frame_id} → {other_ref_frame}'
                )

                T_old_to_new = self.frame_manager.update_reference_frame(other_ref_frame)

                if T_old_to_new is not None:
                    # Publish frame update to mapping module
                    self._publish_frame_update(T_old_to_new)

            # Mark as aligned
            self.aligned_robots.add(other_robot_id)
            self.state = CoordinatorState.ALIGNED

            # Print pose graph
            self.get_logger().info(self.frame_manager.visualize_graph())

        else:
            failure_reason = result['failure_reason'] if result else 'Service call failed'
            self.get_logger().warn(f'Alignment with {other_robot_id} failed: {failure_reason}')
            self.state = CoordinatorState.DETECTING

        self.alignment_in_progress = False

    def _handle_alignment_request(self, request) -> tuple:
        """
        Handle alignment request from another robot.

        Returns:
            (success, transform, metrics) tuple
        """
        requesting_robot = request.requesting_robot_id

        self.get_logger().info(f'Received alignment request from {requesting_robot}')

        # Check if we have a scan
        if self.latest_scan is None:
            return False, np.eye(4), {'reason': 'no_scan_available'}

        # Convert request scan to point cloud
        source_points = self.icp_aligner.arrays_to_pointcloud(
            ranges=np.array(request.current_scan_ranges),
            angle_min=request.scan_angle_min,
            angle_max=request.scan_angle_max,
            range_min=request.range_min,
            range_max=request.range_max
        )

        # Convert our scan to point cloud
        target_points = self.icp_aligner.scan_to_pointcloud(self.latest_scan)

        if len(source_points) < 50 or len(target_points) < 50:
            return False, np.eye(4), {'reason': 'insufficient_points'}

        # Perform ICP alignment
        success, transform, metrics = self.icp_aligner.align_scans(
            source_points=source_points,
            target_points=target_points
        )

        if success:
            self.get_logger().info(
                f'Alignment request from {requesting_robot} successful: '
                f'fitness={metrics["fitness"]:.3f}'
            )

            # Add to pose graph
            self.frame_manager.add_edge(requesting_robot, self.robot_id, transform)

            # Mark as aligned
            self.aligned_robots.add(requesting_robot)

        return success, transform, metrics

    def _publish_frame_update(self, T_old_to_new: np.ndarray):
        """Publish frame update to mapping module."""
        from multi_robot_coordination.msg import FrameUpdate

        msg = FrameUpdate()
        msg.from_frame = 'old_reference'
        msg.to_frame = self.frame_manager.reference_frame_id
        msg.transform_matrix = T_old_to_new.flatten().tolist()
        msg.timestamp = self.get_clock().now().to_msg()

        self.frame_update_pub.publish(msg)

        self.get_logger().info(f'Published frame update to mapping module')


def main(args=None):
    rclpy.init(args=args)
    node = RobotCoordinator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.comm_manager.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

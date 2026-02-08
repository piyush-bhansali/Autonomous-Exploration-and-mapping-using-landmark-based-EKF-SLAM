#!/usr/bin/env python3

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


# =============================================================================
# SENSOR DATA QoS PROFILES
# =============================================================================

SCAN_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Sensor data typically uses BEST_EFFORT
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

IMU_QOS = QoSProfile(
    depth=50,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

ODOM_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

JOINT_STATE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)


# =============================================================================
# MAP DATA QoS PROFILES
# =============================================================================

MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)


# =============================================================================
# POSE AND TRANSFORM QoS PROFILES
# =============================================================================

POSE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,  # Pose updates should be reliable
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

TF_QOS = QoSProfile(
    depth=100,  # Higher depth for transform history
    reliability=ReliabilityPolicy.RELIABLE,  # TF must be reliable
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)


# =============================================================================
# CONTROL AND COMMAND QoS PROFILES
# =============================================================================

CMD_VEL_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,  # Commands should be reliable
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)


# =============================================================================
# NAVIGATION QoS PROFILES
# =============================================================================

PATH_QOS = QoSProfile(
    depth=1,  # Only latest path needed
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Latch latest path
    history=HistoryPolicy.KEEP_LAST
)


# =============================================================================
# VISUALIZATION QoS PROFILES
# =============================================================================

VISUALIZATION_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,  # Must be RELIABLE for RViz MarkerArray compatibility
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

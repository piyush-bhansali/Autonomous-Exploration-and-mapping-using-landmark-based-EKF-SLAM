#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from map_generation.local_submap_generator_icp import LocalSubmapGeneratorICP
from map_generation.local_submap_generator_feature import LocalSubmapGeneratorFeature


def main(args=None):
    """
    Main entry point for local submap generator.

    Reads the 'mapping_mode' parameter to determine which SLAM mode to use:
    - 'icp': ICP-based mapping with pose-only EKF
    - 'feature': Feature-based mapping with landmark EKF-SLAM
    """
    rclpy.init(args=args)

    # Create temporary node to read parameter
    temp_node = Node('_temp_param_reader')
    temp_node.declare_parameter('mapping_mode', 'icp')
    mapping_mode = temp_node.get_parameter('mapping_mode').value
    temp_node.destroy_node()

    # Instantiate appropriate node based on mode
    if mapping_mode == 'feature':
        node = LocalSubmapGeneratorFeature()
    else:
        node = LocalSubmapGeneratorICP()

    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, 'shutdown'):
            node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

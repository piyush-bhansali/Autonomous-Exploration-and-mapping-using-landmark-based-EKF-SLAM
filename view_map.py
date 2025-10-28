#!/usr/bin/env python3
"""
Simple script to view the generated map using Open3D
"""

import open3d as o3d
import numpy as np
import sys

def view_map(pcd_file):
    """Load and visualize a PCD file"""
    print(f"Loading map from: {pcd_file}")

    # Load the point cloud
    pcd = o3d.io.read_point_cloud(pcd_file)

    # Get statistics
    points = np.asarray(pcd.points)

    print("="*60)
    print("MAP STATISTICS")
    print("="*60)
    print(f"Total points: {len(points)}")

    if len(points) > 0:
        print(f"X range: {points[:,0].min():.2f} to {points[:,0].max():.2f} m")
        print(f"Y range: {points[:,1].min():.2f} to {points[:,1].max():.2f} m")
        print(f"Z range: {points[:,2].min():.2f} to {points[:,2].max():.2f} m")

        # Calculate map dimensions
        x_span = points[:,0].max() - points[:,0].min()
        y_span = points[:,1].max() - points[:,1].min()
        print(f"\nMap dimensions: {x_span:.2f}m x {y_span:.2f}m")

    print("="*60)

    # Color the point cloud for better visualization
    if len(points) > 0:
        # Color by height (Z value)
        colors = np.zeros((len(points), 3))
        z_normalized = (points[:,2] - points[:,2].min()) / (points[:,2].max() - points[:,2].min() + 1e-6)
        colors[:,0] = 0.2  # R
        colors[:,1] = 0.5 + 0.5 * z_normalized  # G
        colors[:,2] = 0.8  # B
        pcd.colors = o3d.utility.Vector3dVector(colors)

    # Create coordinate frame for reference
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=1.0, origin=[0, 0, 0]
    )

    # Visualize
    print("\nVisualization Controls:")
    print("  - Mouse: Rotate view")
    print("  - Scroll: Zoom in/out")
    print("  - Ctrl+Mouse: Pan")
    print("  - Q or ESC: Close window")
    print("\nOpening visualization window...")

    try:
        o3d.visualization.draw_geometries(
            [pcd, coordinate_frame],
            window_name="Generated Map",
            width=1024,
            height=768,
            left=50,
            top=50,
            point_show_normal=False
        )
    except Exception as e:
        print(f"\n❌ Visualization failed: {e}")
        print("\nThis might be due to:")
        print("  - Running on WSL without X server")
        print("  - No display available")
        print("  - Missing OpenGL support")
        print("\nThe map data is still valid - try viewing on a system with display support.")
        return False

    return True

if __name__ == "__main__":
    # Default map file location
    map_file = "./test_results/submaps/tb3_1/global_map.pcd"

    # Check if custom file provided
    if len(sys.argv) > 1:
        map_file = sys.argv[1]

    import os
    if not os.path.exists(map_file):
        print(f"❌ Error: Map file not found: {map_file}")
        print(f"\nUsage: python3 view_map.py [path_to_map.pcd]")
        print(f"Default: {map_file}")
        sys.exit(1)

    view_map(map_file)

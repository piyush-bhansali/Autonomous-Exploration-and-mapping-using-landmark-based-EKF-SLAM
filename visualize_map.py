#!/usr/bin/env python3
"""
Standalone Map Visualization Tool

Visualizes saved .pcd map files using Open3D.
No ROS2 required - can be used offline to inspect saved maps.

Usage:
    python3 visualize_map.py <path_to_map.pcd>
    python3 visualize_map.py ./submaps/tb3_1/global_map.pcd
    python3 visualize_map.py ./test_results/submaps/tb3_1/global_map.pcd
"""

import sys
import os
import numpy as np
import open3d as o3d


def visualize_map(map_file: str):
    """Load and visualize a point cloud map"""

    if not os.path.exists(map_file):
        print(f"❌ Error: File not found: {map_file}")
        return

    print(f"Loading map from: {map_file}")

    try:
        # Load point cloud
        pcd = o3d.io.read_point_cloud(map_file)

        if len(pcd.points) == 0:
            print("❌ Error: Point cloud is empty!")
            return

        # Get statistics
        points = np.asarray(pcd.points)

        print("\n" + "="*60)
        print("GLOBAL MAP STATISTICS")
        print("="*60)
        print(f"Total points: {len(points):,}")
        print(f"\nBounds:")
        print(f"  X: {points[:,0].min():.2f} to {points[:,0].max():.2f} m (range: {points[:,0].max() - points[:,0].min():.2f} m)")
        print(f"  Y: {points[:,1].min():.2f} to {points[:,1].max():.2f} m (range: {points[:,1].max() - points[:,1].min():.2f} m)")
        print(f"  Z: {points[:,2].min():.2f} to {points[:,2].max():.2f} m (range: {points[:,2].max() - points[:,2].min():.2f} m)")

        # Compute centroid
        centroid = points.mean(axis=0)
        print(f"\nCentroid: ({centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f})")

        # Estimate map density
        bbox = pcd.get_axis_aligned_bounding_box()
        volume = bbox.volume()
        if volume > 0:
            density = len(points) / volume
            print(f"\nPoint density: {density:.2f} points/m³")

        print("="*60)
        print("\n🔍 Opening visualization window...")
        print("\nControls:")
        print("  - Mouse drag: Rotate view")
        print("  - Mouse wheel: Zoom in/out")
        print("  - Shift + mouse drag: Pan")
        print("  - R: Reset view")
        print("  - Q or ESC: Quit")
        print("="*60 + "\n")

        # Color the point cloud (optional - green)
        pcd.paint_uniform_color([0.0, 1.0, 0.0])  # Green

        # Create coordinate frame for reference
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=1.0,
            origin=[0, 0, 0]
        )

        # Visualize
        o3d.visualization.draw_geometries(
            [pcd, coord_frame],
            window_name=f"Global Map - {os.path.basename(map_file)}",
            width=1600,
            height=900,
            left=50,
            top=50,
            point_show_normal=False,
            mesh_show_wireframe=False,
            mesh_show_back_face=False
        )

        print("✅ Visualization closed")

    except Exception as e:
        print(f"❌ Error loading or visualizing map: {e}")
        import traceback
        traceback.print_exc()


def main():
    """Main entry point"""

    if len(sys.argv) < 2:
        print("Usage: python3 visualize_map.py <path_to_map.pcd>")
        print("\nExamples:")
        print("  python3 visualize_map.py ./submaps/tb3_1/global_map.pcd")
        print("  python3 visualize_map.py ./test_results/submaps/tb3_1/global_map.pcd")
        print("\nCommon map locations:")
        print("  - ./submaps/tb3_1/global_map.pcd (exploration mode)")
        print("  - ./test_results/submaps/tb3_1/global_map.pcd (test mode)")
        sys.exit(1)

    map_file = sys.argv[1]
    visualize_map(map_file)


if __name__ == '__main__':
    main()

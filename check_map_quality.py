#!/usr/bin/env python3
"""Quick script to check map quality"""

import open3d as o3d
import numpy as np

# Load map
pcd = o3d.io.read_point_cloud("./test_results/submaps/tb3_1/global_map.pcd")
points = np.asarray(pcd.points)

print("=" * 60)
print("GLOBAL MAP ANALYSIS")
print("=" * 60)
print(f"Total points: {len(points)}")
print(f"X range: {points[:, 0].min():.2f} to {points[:, 0].max():.2f} m")
print(f"Y range: {points[:, 1].min():.2f} to {points[:, 1].max():.2f} m")
print(f"Z range: {points[:, 2].min():.2f} to {points[:, 2].max():.2f} m")
print()

# Check for duplicated/overlapping points (indicator of thick walls)
from scipy.spatial import cKDTree
tree = cKDTree(points)

# Count points with neighbors within 5cm
close_neighbors = []
for i in range(min(1000, len(points))):  # Sample 1000 points
    neighbors = tree.query_ball_point(points[i], r=0.05)
    close_neighbors.append(len(neighbors))

avg_density = np.mean(close_neighbors)
print(f"Average points within 5cm: {avg_density:.1f}")
if avg_density > 10:
    print("⚠ WARNING: High point density suggests overlapping/thick walls")
elif avg_density > 5:
    print("⚠ CAUTION: Moderate point density - some overlap likely")
else:
    print("✓ Point density looks good - thin walls")

print()

# Analyze point distribution along corridor
# Assuming robot moved in X direction
x_bins = np.linspace(points[:, 0].min(), points[:, 0].max(), 20)
y_range_per_bin = []

for i in range(len(x_bins) - 1):
    mask = (points[:, 0] >= x_bins[i]) & (points[:, 0] < x_bins[i+1])
    if np.sum(mask) > 0:
        y_range = points[mask, 1].max() - points[mask, 1].min()
        y_range_per_bin.append(y_range)

if y_range_per_bin:
    avg_corridor_width = np.mean(y_range_per_bin)
    std_corridor_width = np.std(y_range_per_bin)
    print(f"Corridor width: {avg_corridor_width:.2f} ± {std_corridor_width:.2f} m")
    if std_corridor_width > 0.5:
        print("⚠ WARNING: High variation in corridor width - poor alignment!")
    else:
        print("✓ Corridor width is consistent")

print("=" * 60)

#!/usr/bin/env python3
"""
GPU Verification Script for Map Generation Package

This script verifies that your system is properly configured for GPU-accelerated
point cloud processing with Open3D.
"""

import sys

def check_cuda_availability():
    """Check if CUDA is available on the system"""
    try:
        import subprocess
        result = subprocess.run(['nvidia-smi'], capture_output=True, text=True)
        if result.returncode == 0:
            print("✓ NVIDIA GPU detected")
            print(result.stdout.split('\n')[0:10])  # Print first few lines
            return True
        else:
            print("✗ nvidia-smi command failed")
            return False
    except FileNotFoundError:
        print("✗ nvidia-smi not found - NVIDIA drivers may not be installed")
        return False

def check_open3d():
    """Check Open3D installation and CUDA support"""
    try:
        import open3d as o3d
        print(f"\n✓ Open3D installed")
        print(f"  Version: {o3d.__version__}")
        return True
    except ImportError:
        print("\n✗ Open3D not installed")
        print("  Install with: pip install open3d")
        return False

def check_open3d_cuda():
    """Check if Open3D has CUDA support"""
    try:
        import open3d.core as o3c
        print("\n✓ Open3D core module available")

        if o3c.cuda.is_available():
            print("✓ CUDA support enabled in Open3D")
            device_count = o3c.cuda.device_count()
            print(f"  GPU devices available: {device_count}")

            for i in range(device_count):
                device = o3c.Device(f"CUDA:{i}")
                print(f"    Device {i}: {device}")

            # Test GPU operation
            print("\n  Testing GPU tensor operation...")
            import numpy as np
            test_data = np.random.rand(1000, 3).astype(np.float32)
            tensor_gpu = o3c.Tensor(test_data, dtype=o3c.float32, device=o3c.Device("CUDA:0"))
            print(f"  ✓ Successfully created tensor on GPU: shape={tensor_gpu.shape}")

            return True
        else:
            print("✗ CUDA not available in Open3D")
            print("  Open3D was built without CUDA support")
            print("  You need to rebuild Open3D with -DBUILD_CUDA_MODULE=ON")
            return False

    except ImportError as e:
        print("\n✗ Open3D core module not available")
        print(f"  Error: {e}")
        print("  You may be using an old version of Open3D")
        print("  Upgrade with: pip install --upgrade open3d>=0.15.0")
        return False
    except Exception as e:
        print(f"\n✗ Error testing CUDA: {e}")
        return False

def check_dependencies():
    """Check other required dependencies"""
    print("\nChecking dependencies...")
    deps = {
        'numpy': 'numpy',
        'rclpy': 'rclpy (ROS 2 Python)',
    }

    all_good = True
    for module_name, display_name in deps.items():
        try:
            __import__(module_name)
            print(f"  ✓ {display_name}")
        except ImportError:
            print(f"  ✗ {display_name} not installed")
            all_good = False

    return all_good

def test_point_cloud_gpu():
    """Test point cloud processing on GPU"""
    try:
        import open3d as o3d
        import open3d.core as o3c
        import numpy as np

        if not o3c.cuda.is_available():
            print("\n⚠ Skipping point cloud GPU test - CUDA not available")
            return False

        print("\nTesting GPU point cloud operations...")

        # Create test point cloud
        points = np.random.rand(10000, 3).astype(np.float32)
        points_tensor = o3c.Tensor(points, dtype=o3c.float32, device=o3c.Device("CUDA:0"))

        pcd = o3d.t.geometry.PointCloud(o3c.Device("CUDA:0"))
        pcd.point.positions = points_tensor

        print(f"  ✓ Created point cloud on GPU: {len(pcd.point.positions)} points")

        # Test voxel downsampling
        pcd_down = pcd.voxel_down_sample(voxel_size=0.1)
        print(f"  ✓ Voxel downsampling on GPU: {len(pcd.point.positions)} → {len(pcd_down.point.positions)} points")

        # Test normal estimation
        pcd_down.estimate_normals(max_nn=30, radius=0.2)
        print(f"  ✓ Normal estimation on GPU")

        print("  ✓ All GPU point cloud operations successful!")
        return True

    except Exception as e:
        print(f"\n✗ GPU point cloud test failed: {e}")
        return False

def main():
    print("="*70)
    print("GPU Acceleration Verification for Map Generation")
    print("="*70)

    results = []

    # Check CUDA
    results.append(("NVIDIA GPU", check_cuda_availability()))

    # Check Open3D
    results.append(("Open3D Installation", check_open3d()))

    # Check Open3D CUDA
    results.append(("Open3D CUDA Support", check_open3d_cuda()))

    # Check dependencies
    results.append(("Dependencies", check_dependencies()))

    # Test point cloud GPU operations
    results.append(("GPU Point Cloud Operations", test_point_cloud_gpu()))

    # Summary
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)

    all_passed = True
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{status}: {name}")
        if not passed:
            all_passed = False

    print("="*70)

    if all_passed:
        print("\n🎉 All checks passed! GPU acceleration is ready to use.")
        print("\nYour feature-based submap generator will automatically use GPU acceleration for submap stitching.")
        print("You should see messages like:")
        print("  ✓ GPU acceleration enabled: CUDA:0")
        print("  Submap 0: 5000 → 1200 points (GPU-processed)")
        return 0
    else:
        print("\n⚠ Some checks failed. GPU acceleration may not work properly.")
        print("\nRefer to GPU_SETUP.md for installation instructions.")
        print("\nThe system will fall back to CPU processing if GPU is not available.")
        return 1

if __name__ == "__main__":
    sys.exit(main())

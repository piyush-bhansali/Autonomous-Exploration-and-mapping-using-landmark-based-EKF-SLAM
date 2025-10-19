import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import open3d as o3d
import numpy as np
import sys

class LocalMap(Node):
  def __init__(self, robot_name=None):
    super().__init__('local_map')

    # Declare parameters
    self.declare_parameter('robot_name', robot_name or 'tb3_1')
    self.declare_parameter('scan_topic', '')
    self.declare_parameter('update_rate', 20.0)
    self.declare_parameter('max_range', 3.5)
    self.declare_parameter('window_width', 1280)
    self.declare_parameter('window_height', 720)

    # Get parameters
    self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
    scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
    self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
    self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
    window_width = self.get_parameter('window_width').get_parameter_value().integer_value
    window_height = self.get_parameter('window_height').get_parameter_value().integer_value

    # Auto-detect scan topic if not provided
    if not scan_topic:
      scan_topic = f'/{self.robot_name}/scan'

    # Log startup information
    self.get_logger().info('=' * 60)
    self.get_logger().info(f'Live Laser Scan Visualizer')
    self.get_logger().info('=' * 60)
    self.get_logger().info(f'Robot name: {self.robot_name}')
    self.get_logger().info(f'Scan topic: {scan_topic}')
    self.get_logger().info(f'Update rate: {self.update_rate} Hz')
    self.get_logger().info(f'Max range: {self.max_range} m')
    self.get_logger().info('=' * 60)

    # Create timer to check topics
    self.create_timer(2.0, self.check_topics_once)

    # Subscribe to laser scan
    self.scan_sub = self.create_subscription(
      LaserScan,
      scan_topic,
      self.scan_callback,
      10
    )

    # Initialize data storage
    self.current_points = None
    self.scan_count = 0
    self.topic_checked = False

    # Create Open3D visualizer
    self.vis = o3d.visualization.Visualizer()
    self.vis.create_window(
      window_name=f'{self.robot_name} - Live Laser Scan',
      width=window_width,
      height=window_height
    )

    # Create point cloud for laser scan
    self.pcd = o3d.geometry.PointCloud()
    self.vis.add_geometry(self.pcd)

    # Add coordinate frame at origin
    self.frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    self.vis.add_geometry(self.frame)

    # Add robot base indicator (small sphere at origin)
    self.robot_base = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
    self.robot_base.paint_uniform_color([1, 0, 0])  # Red
    self.vis.add_geometry(self.robot_base)

    # Add invisible bounding box to fix view bounds
    # This prevents Open3D from auto-fitting to current scan data
    bbox_size = 15.0  # 15x15 meter viewing area
    bbox_points = np.array([
      [-bbox_size, -bbox_size, 0],
      [bbox_size, -bbox_size, 0],
      [-bbox_size, bbox_size, 0],
      [bbox_size, bbox_size, 0],
    ])
    self.bbox = o3d.geometry.PointCloud()
    self.bbox.points = o3d.utility.Vector3dVector(bbox_points)
    self.bbox.paint_uniform_color([0.9, 0.9, 0.9])  # Light gray corners
    self.vis.add_geometry(self.bbox)

    # Set render options
    render_option = self.vis.get_render_option()
    render_option.point_size = 5.0  # Make points visible
    render_option.background_color = np.array([0.95, 0.95, 0.95])  # Light background

    # Camera setup - bird's eye view
    view_ctl = self.vis.get_view_control()
    
    # Set constant clipping planes to prevent auto-adjustment
    view_ctl.set_constant_z_near(0.1)
    view_ctl.set_constant_z_far(100.0)

    # Zoom out to see entire area
    for _ in range(300):
      view_ctl.scale(-1.0)

    # Set viewing direction (top-down)
    view_ctl.set_lookat([0, 0, 0])    # Look at origin
    view_ctl.set_up([0, -1, 0])       # Y-axis up
    view_ctl.set_front([0, 0, -1])    # Look down Z-axis

    # Create timer for visualization updates
    update_period = 1.0 / self.update_rate
    self.create_timer(update_period, self.update_visualization)

    self.get_logger().info('Waiting for laser scan data...')
    self.get_logger().info('Make sure multi_robot_mapping is running!')
    self.get_logger().info('View is locked to 15x15m area')
  
  def check_topics_once(self):
    if self.topic_checked:
        return
        
    self.topic_checked = True

  def scan_callback(self, msg):

    num_rays = len(msg.ranges)
    angles = np.linspace(msg.angle_min, msg.angle_max, num_rays)
    ranges = np.array(msg.ranges)

    # Filter out invalid readings
    valid_indices = (ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)
    valid_angles = angles[valid_indices]
    valid_ranges = ranges[valid_indices]
        
    if len(valid_ranges) == 0:
        if self.scan_count % 20 == 0:
            self.get_logger().warn('No valid laser readings in current scan')
        self.scan_count += 1
        return
    
    # Convert polar to Cartesian coordinates
    x = valid_ranges * np.cos(valid_angles)
    y = valid_ranges * np.sin(valid_angles)
    z = np.zeros_like(x)  # All points at z=0 (2D scan)

    points = np.column_stack((x, y, z))
        
    # Store for visualization
    self.current_points = points
        
    # Log info
    self.scan_count += 1
    if self.scan_count == 1:
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'✓ First scan received from {self.robot_name}!')
        self.get_logger().info(f'  Points: {len(valid_ranges)}/{num_rays}')
        self.get_logger().info(f'  Range: {msg.range_min:.2f} - {msg.range_max:.2f} m')
        self.get_logger().info(f'  Angle: {np.degrees(msg.angle_min):.1f}° - {np.degrees(msg.angle_max):.1f}°')
        self.get_logger().info('=' * 60)
    elif self.scan_count % 100 == 0:
        self.get_logger().info(
            f'Scans: {self.scan_count} | '
            f'Current points: {len(valid_ranges)}/{num_rays}'
        )

  def update_visualization(self): 
     
    if self.current_points is None or len(self.current_points) == 0:
       return

    self.pcd.points = o3d.utility.Vector3dVector(self.current_points) 

    distances = np.linalg.norm(self.current_points[:, :2], axis=1)

    # Normalize distances
    normalized_dist = np.clip(distances / self.max_range, 0, 1)
        
    # Create color gradient (red to blue based on distance)
    colors = np.zeros((len(distances), 3))
    colors[:, 0] = 1 - normalized_dist  # Red decreases with distance
    colors[:, 2] = normalized_dist      # Blue increases with distance
        
    self.pcd.colors = o3d.utility.Vector3dVector(colors)
        
    # Update geometry
    self.vis.update_geometry(self.pcd)
        
    # Poll events and render
    self.vis.poll_events()
    self.vis.update_renderer()   

def main(args=None):
    rclpy.init(args=args)
    
    # Check for command line argument for robot name
    robot_name = None
    if len(sys.argv) > 1 and not sys.argv[1].startswith('--'):
        robot_name = sys.argv[1]
    
    node = LocalMap(robot_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down visualizer...')
    finally:
        node.vis.destroy_window()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
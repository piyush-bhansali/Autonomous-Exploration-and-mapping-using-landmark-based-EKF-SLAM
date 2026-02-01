from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'autonomous_exploration'

def files_only(pattern):
    """Helper to filter out directories from glob results"""
    return [p for p in glob(pattern, recursive=True) if os.path.isfile(p)]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package marker file
        ('share/ament_index/resource_index/packages', 
            [f'resource/{package_name}']),
        
        # Package.xml
        (f'share/{package_name}', ['package.xml']),
        
        # Launch files (must end with .launch.py)
        (os.path.join('share', package_name, 'launch'), 
            files_only('launch/*.launch.py')),
        
        # World files (.sdf for Gazebo)
        (os.path.join('share', package_name, 'worlds'), 
            files_only('worlds/*.sdf')),

        # Add to data_files:
        (os.path.join('share', package_name, 'models/turtlebot3_waffle_pi'),    
            files_only('models/turtlebot3_waffle_pi/*')),

        # Meshes for sensors, base and wheels
        (os.path.join('share', package_name, 'meshes/bases'),
            files_only('meshes/bases/*.stl')),
        (os.path.join('share', package_name, 'meshes/wheels'),
            files_only('meshes/wheels/*.stl')),
        (os.path.join('share', package_name, 'meshes/sensors'),
            files_only('meshes/sensors/*.stl')), 

        # urdf file
        (os.path.join('share', package_name, 'urdf'),
            files_only('urdf/*.urdf')),
      
        
        # Config files (if you add YAML parameter files later)
        (os.path.join('share', package_name, 'config'), 
            files_only('config/*.yaml')),
        
        # RViz configs (if you create custom RViz views)
        (os.path.join('share', package_name, 'rviz'), 
            files_only('rviz/*.rviz')),
        
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    
    maintainer='piyush',
    maintainer_email='piyushbhansali8@gmail.com',
    
    description='Autonomous exploration system with TurtleBot3 SLAM in Gazebo Harmonic',
    license='Apache-2.0',
    
    tests_require=['pytest'],
    extras_require={'test': ['pytest']},
    
    entry_points={
        'console_scripts': [
        ],
    },
)
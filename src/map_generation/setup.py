from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'map_generation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='piyush',
    maintainer_email='piyushbhansali8@gmail.com',
    description='SLAM mapping with EKF fusion and ICP corrections',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'local_map = map_generation.local_map:main',
          'local_submap_generator = map_generation.local_submap_generator:main',
          'test_robot_controller = map_generation.test_robot_controller:main',
        ],
    },
)

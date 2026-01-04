from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'shapely>=2.0',
        'scikit-learn',
    ],
    zip_safe=True,
    maintainer='piyush',
    maintainer_email='piyushbhansali8@gmail.com',
    description='Frontier-based exploration with RRT* path planning for point cloud maps',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_navigation = navigation.simple_navigation:main',
            'test_straight_line = navigation.test_straight_line:main',
        ],
    },
)

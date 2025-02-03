import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'px4_offboard': 'scripts'},  # ⬅️ Assicura che gli script siano in "scripts/"
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/resource', glob('resource/*.rviz'))
    ],
    install_requires=[
        'setuptools',
        'cv_bridge',
        'image_transport',
        'compressed_image_transport',
        'sensor_msgs'
    ],
    zip_safe=True,
    maintainer='Martina',
    maintainer_email='martinaliberatoscioli@gmail.com',
    description='PX4 Offboard package for ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'drone_control = px4_offboard.drone_control:main',
            'visualizer = px4_offboard.visualizer:main',
            'velocity_control = px4_offboard.velocity_control:main',
            'control = px4_offboard.control:main',
            'processes = px4_offboard.processes:main',
            'detection = px4_offboard.detection:main',
            'camera_publisher_node = px4_offboard.camera_publisher_node:main',
            'camera_tf2_publisher = px4_offboard.camera_tf2_publisher:main',
            'odometry_logger = px4_offboard.odometry_logger:main',
            'waypoint_navigation = px4_offboard.waypoint_navigation:main'
        ],
    },
)

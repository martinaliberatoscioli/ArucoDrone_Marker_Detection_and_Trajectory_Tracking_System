from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # Cerca automaticamente i pacchetti
    package_dir={'': '.'},  # Usa la directory corrente
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.[pxy][yma]*')),
        ('share/' + package_name + '/resource', glob('resource/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Martina',
    maintainer_email='tuo@email.com',
    description='PX4 Offboard package for ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'drone_control = px4_offboard.scripts.drone_control:main',
            'visualizer = px4_offboard.scripts.visualizer:main',
            'velocity_control = px4_offboard.scripts.velocity_control:main',
            'control = px4_offboard.scripts.control:main',
            'processes = px4_offboard.scripts.processes:main',
            'detection = px4_offboard.scripts.detection:main',
            'camera_publisher_node = px4_offboard.scripts.camera_publisher_node:main',
            'camera_tf2_publisher = px4_offboard.scripts.camera_tf2_publisher:main',
            'odometry_logger = px4_offboard.scripts.odometry_logger:main',
        ],
    },
)

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')

    return LaunchDescription([
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='control',
            name='control',
            prefix='gnome-terminal --',
        ),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='velocity_control',
            name='velocity_control' 
        ),
        
        Node(
            package='px4_offboard',
            executable='drone_control',  
            name='drone_control',
            output='screen'
        ),
        
        # Esegui il nodo parameter_bridge per la comunicazione tra ROS 2 e Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            ],
            output='screen'
        ),

        Node(
            package='px4_offboard', 
            executable='detection', 
            name='parallelepiped_marker_detector',
            output='screen'
        ),
        
        # Aggiungi il nodo per camera_publisher
        Node(
            package='px4_offboard',  # Il pacchetto dove si trova il nodo
            executable='camera_publisher_node',  # Il nome dell'eseguibile del nodo
            name='camera_publisher',  # Nome del nodo
            output='screen'  # Output a schermo
        ),
        
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        ),
        
        # Avvia QGroundControl
        ExecuteProcess(
            cmd=[os.path.expanduser('~/QGroundControl.AppImage')],
            output='screen'
        )
    ])


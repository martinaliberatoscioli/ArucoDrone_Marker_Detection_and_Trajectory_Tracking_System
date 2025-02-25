from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    rviz_config_path = os.path.join(package_dir, 'resource', 'visualize.rviz') 

    return LaunchDescription([
        LogInfo(msg='Avvio del bridge ROS-Gazebo per la camera RGBD'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        LogInfo(msg='Avvio del nodo visualizer'),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer'
        ),

        LogInfo(msg='Avvio del nodo processes'),
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),
        
        LogInfo(msg='Avvio del nodo di controllo della velocità'),
        Node(
            package='px4_offboard',
            executable='offboard',
            name='offboard_control',
            output = 'screen'
        ),

    
        LogInfo(msg='Avvio del nodo di rilevamento ArUco Marker'),
        Node(
            package='px4_offboard',
            executable='detection',
            name='parallelepiped_marker_detector',
            output='screen'
        ),

        LogInfo(msg='Avvio del nodo camera_publisher'),
        Node(
            package='px4_offboard',
            executable='camera_publisher_node',
            name='camera_publisher',
            output='screen',
            
        ),

        LogInfo(msg='Avvio di RViz per la visualizzazione'),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_path]
        ),

        LogInfo(msg='Avvio di QGroundControl'),
        ExecuteProcess(
            cmd=[os.path.expanduser('~/QGroundControl.AppImage')],
            output='screen'
        )
    ])

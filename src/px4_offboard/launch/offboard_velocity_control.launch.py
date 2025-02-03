from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')

    return LaunchDescription([
        # Avvia il server di Gazebo (Ignition Gazebo)
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v 4', os.path.join(package_dir, 'worlds', 'aruco.sdf')],
            output='screen'
        ),

        # Avvia PX4 SITL con Gazebo
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 'cd ~/PX4-Autopilot && PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam'],
            output='screen'
        ),

        # Bridge ROS-Gazebo per la camera RGBD
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen'
        ),

        # Avvia il nodo visualizer
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer'
        ),

        # Avvia il nodo processes (che gestisce altri processi)
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --'
        ),

        # Avvia il nodo di controllo
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='control',
            name='control',
            prefix='gnome-terminal --'
        ),

        # Controllo della velocità
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='velocity_control',
            name='velocity_control'
        ),

        # Controllo del drone
        Node(
            package='px4_offboard',
            executable='drone_control',
            name='drone_control',
            output='screen'
        ),

        # Rilevamento dell'ArUco Marker
        Node(
            package='px4_offboard',
            executable='detection',
            name='parallelepiped_marker_detector',
            output='screen'
        ),

        # Camera Publisher
        Node(
            package='px4_offboard',
            executable='camera_publisher_node',
            name='camera_publisher',
            output='screen'
        ),

        # Avvia RViz per la visualizzazione
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', os.path.join(package_dir, 'resource', 'visualize.rviz')]
        ),

        # Avvia QGroundControl
        ExecuteProcess(
            cmd=[os.path.expanduser('~/QGroundControl.AppImage')],
            output='screen'
        )
    ])

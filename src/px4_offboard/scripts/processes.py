#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import time

class ProcessesNode(Node):
    def __init__(self):
        super().__init__('processes')
        self.get_logger().info("Avviando i processi per PX4 e Gazebo...")

        commands = [
            "source /opt/ros/humble/setup.bash && source ~/aruco_drone_proj/install/setup.bash",
            "gnome-terminal -- bash -c 'MicroXRCEAgent udp4 -p 8888; exec bash'",
            "gnome-terminal -- bash -c 'cd ~/PX4-Autopilot && PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam; exec bash'",
            "sleep 5",
            "gnome-terminal -- bash -c 'ros2 run ros_gz_bridge parameter_bridge /rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image; exec bash'",
            "sleep 3",
            "gnome-terminal -- bash -c 'ros2 run px4_offboard detection; exec bash'",
            "gnome-terminal -- bash -c 'ros2 run px4_offboard camera_publisher_node; exec bash'",
            "gnome-terminal -- bash -c 'ros2 run px4_offboard visualizer; exec bash'",
            "gnome-terminal -- bash -c 'ros2 run px4_offboard drone_control; exec bash'",
            "gnome-terminal -- bash -c 'ros2 run px4_offboard velocity_control; exec bash'",
            "gnome-terminal -- bash -c 'ros2 run px4_offboard odometry_logger; exec bash'",
            "gnome-terminal -- bash -c 'cd ~ && ./QGroundControl.AppImage; exec bash'"
        ]

        for command in commands:
            subprocess.run(command, shell=True)
            time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = ProcessesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

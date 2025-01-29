#!/usr/bin/env python3

# Import the subprocess and time modules
import subprocess
import time

# List of commands to run
commands = [
    # Run the Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Run the PX4 SITL simulation with Ignition Gazebo (your custom world and drone)
    "cd ~/PX4-Autopilot && PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam",  # Avvia PX4 con Ignition Gazebo

    # Run the ros_gz_bridge parameter bridge
    "ros2 run ros_gz_bridge parameter_bridge /rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",  # Bridge ROS 2 <-> Gazebo
    
    # Run the cube and marker detection node
     "ros2 run px4_offboard detection"  # Lanciare il pacchetto per la detection 

    # Run the camera publisher node
    "ros2 run px4_offboard camera_publisher_node"
    
    # Run QGroundControl
    "cd ~ && ./QGroundControl.AppImage" # /home/martina/QGroundControl.AppImage

    
]

# Loop through each command in the list
for command in commands:
    # Each command is run in a new tab of the gnome-terminal
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    
    # Pause between each command
    time.sleep(1)



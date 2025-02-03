#!/usr/bin/env python3

import subprocess
import time

# Lista dei comandi da eseguire
commands = [
    # Avvia Micro XRCE-DDS Agent
    "MicroXRCEAgent udp4 -p 8888",

    # Avvia PX4 SITL con Ignition Gazebo
    "cd ~/PX4-Autopilot && PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam",

    # Avvia il ROS-Gazebo Bridge
    "ros2 run ros_gz_bridge parameter_bridge /rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",

    # Avvia il nodo di detection
    "ros2 run px4_offboard detection",

    # Avvia il nodo della camera
    "ros2 run px4_offboard camera_publisher_node",



    # Avvia il logger dell'odometria
    "ros2 run px4_offboard odometry_logger",

    # Avvia QGroundControl
    "cd ~ && ./QGroundControl.AppImage"
]

# Esegue ogni comando in una nuova scheda del terminale
for command in commands:
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])
    time.sleep(1)  # Pausa tra un comando e l'altro


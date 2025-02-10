# ArucoDrone: Marker Detection and Trajectory Tracking System

## Overview

**ArucoDrone** is a project developed for detecting **ArUco markers** using a drone controlled via **PX4** and **ROS2**. This project utilizes simulators like **Gazebo** and computer vision tools to enable the drone to:

- Identify an **ArUco marker** in a simulated environment  
- Analyze the content of the image within the marker  
- Follow a predefined trajectory  
- Dynamically visualize telemetry data through **Python plots**  

This repository is a derivative of the **Offboard example** by [Jaeyoung Lim](https://github.com/Jaeyoung-Lim/px4-offboard), with several added features.  

## Prerequisites

Ensure you have the following components installed:

- **PX4 Autopilot**  
- **ROS2 Humble**  
- **QGroundControl**  
- **Micro XRCE-DDS Agent**  
- **RViz**  
- **Ubuntu 22**  
- **Python 3.10**  
- **Gazebo** *(for drone simulation)*  
- **OpenCV** *(for computer vision)*  
- **Matplotlib & Pandas** *(for telemetry visualization)*

## Project Structure

```
ArucoDrone/
│── src/                         # Source code
│   ├── px4_offboard/
│   │   ├── scripts/
│   │   │   ├── detection.py       # ArUco marker detection
│   │   │   ├── drone_control.py   # Drone control functions
│   │   │   ├── waypoint_navigation.py # Waypoint trajectory execution
│   │   │   ├── visualizer.py      # Telemetry visualization
│   │   ├── ... other files ...
│── config/                      # Drone configuration and parameters
│── simulations/                 # Files for Gazebo simulation
│── README.md                    # Project documentation
│── requirements.txt              # Python dependencies
```

## Setup Steps

### Install PX4 Autopilot

To install PX4, run the following code:
```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.14
```
Run this script in a bash shell to install everything:
```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
You will now need to restart your computer before continuing.

### Install ROS2 Humble

Follow the steps [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) to install ROS2 Humble.

### Install Dependencies

```bash
pip3 install --user -U empy pyros-genmsg setuptools
pip3 install kconfiglib
pip install --user jsonschema
pip install --user jinja2
```

### Build Micro DDS
```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Setup Workspace
```bash
mkdir -p ~/ros2_px4_offboard_example_ws/src
cd ~/ros2_px4_offboard_example_ws/src
```

### Clone in Packages
```bash
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
git clone https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example.git
```

## Installation
1. Clone the repository:
   ```bash
   git clone <your-repo-url>
   cd <repo-name>
   ```
2. Follow the instructions to set up PX4 and ROS2.
3. Copy the `ROS2_PX4_Offboard_Example` into your workspace and prepare your configuration.

This will start:
- **`px4_msgs`** and **`px4_offboard`** package builds
- The necessary ROS2 environment setup
- The **offboard_velocity_control.launch.py** script to initiate the drone control system

### Drone Controls
- **W**: Up
- **S**: Down
- **A**: Yaw Left
- **D**: Yaw Right
- **Up Arrow**: Pitch Forward
- **Down Arrow**: Pitch Backward
- **Left Arrow**: Roll Left
- **Right Arrow**: Roll Right
- **Space**: Arm/Disarm

## Modifications to the World and Drone
We customized the simulation environment by modifying the drone type to include a camera. We added an ArUco marker supported by a parallelepiped to enhance its visibility from the camera.

## Approach Script
We implemented a script that initially positions the drone 6 meters away from the marker. The drone approaches to 1 meter from the marker to begin detection.

## Contributing
If you wish to contribute, feel free to fork the repository and submit a pull request.

## License
This project is distributed under the MIT License. See the LICENSE file for more details.


## ISTRUZIONI
1. le cartelle **arucotag** e **x500_mono_cam** una volta clonato il repository devono essere spostate in /PX4-Autopilot/Tools/simulation/gz/models
2. il file aruco.sdf va spostata in /PX4-Autopilot/Tools/simulation/gz/worlds

## Launching the Simulation
```bash
colcon build
source install/setup.bash
cd src
ros2 launch px4_offboard offboard_velocity_control.launch.py
```





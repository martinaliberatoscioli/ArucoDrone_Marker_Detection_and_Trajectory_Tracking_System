# ArucoDrone: Marker Detection and Trajectory Tracking System

## Overview

**ArucoDrone** is a project developed for detecting **ArUco markers** using a drone controlled via **PX4** and **ROS2**. This project utilizes **Gazebo** simulator and computer vision tools to enable the drone to:

- Detect an **ArUco marker** in the simulation
- Analysìze the content of the marker and publish an associated image when the marker with ID 42 is detected
- Follow a predefined trajectory and maintaining position
- Dynamically visualize real-time telemetry data through **Python plots**

## Project Structure

```
drone/
│── src/
|    │── px4_msgs                     # PX4 ROS2 Messages
│   ├── px4_offboard/
│   │   ├── scripts/
│   │   │   ├── detection.py               # ArUco marker detection
│   │   │   ├── camera_publisher_node.py   # Publishes images from the drone camera
│   │   │   ├── offboard.py                # Drone Offboard control
│   │   │   ├── odometry_logger.py         # Logs and plots drone odometry
│   │   │   ├── processes.py               # Automates launch of multiple ROS nodes
│   │   │   ├── visualizer.py              # Visualization of telemetry data
│   │   ├── ...
│── README.md                    # Project documentation

```

## Prerequisites & Installation Guide

### 1. Install PX4 Autopilot
PX4 is the flight control system required for the drone simulation.

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.14
```

Now, install the required dependencies:

```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

Reboot your computer after installation.

---

### 2. Install ROS2 Humble
ROS2 is used for communication between different drone components.

Follow the official installation steps for **ROS2 Humble**:
[ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Alternatively, you can install it using:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-dev-tools
```

To configure the ROS2 environment:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 3. Install Gazebo Garden 7.9.0
Gazebo is the simulation environment for the drone.

```bash
sudo apt install -y gazebo
```

You may also need the **Gazebo ROS2 bridge**:
```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

---

### 4. Install QGroundControl
QGroundControl is used for visualizing telemetry and controlling the drone.

Download the latest AppImage from the official website:
[QGroundControl Download](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html)

Then, make it executable:
```bash
chmod +x QGroundControl.AppImage
./QGroundControl.AppImage
```

---

### 5. Install Micro XRCE-DDS Agent
This is required for communication between PX4 and ROS2.

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

---

### 6. Install Python Dependencies
This project requires additional Python packages.

```bash
pip3 install --user -U empy pyros-genmsg setuptools kconfiglib jsonschema jinja2
pip3 install matplotlib pandas opencv-python numpy rclpy cv_bridge
```

For telemetry visualization, install:
```bash
pip3 install matplotlib pandas opencv-python
```

---

### 7. Setup the ROS2 Workspace
Create and configure a ROS2 workspace:

```bash
cd ~/drone/src
```

Now, clone the required ROS2 packages:

```bash
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
```

Then, build the workspace:
```bash
cd ~/drone
colcon build
source install/setup.bash
```

---

## Final Steps
After installing everything, you can launch the simulation with:

```bash
cd drone
colcon build
source install/setup.bash
cd src
ros2 launch px4_offboard offboard_velocity_control.launch.py
```

## Modifications to the World and Drone
We customized the simulation environment by modifying the drone type to include a camera. We added an ArUco marker supported by a parallelepiped to enhance its visibility from the camera.

## Approach Script
We implemented a script that initially positions the drone 6 meters away from the marker. The drone approaches to 1 meter from the marker to begin detection.

## Contributing
If you wish to contribute, feel free to fork the repository and submit a pull request.

## License
This project is distributed under the MIT License. See the LICENSE file for more details.

## Instructions
1. The folders **arucotag** and **x500_mono_cam** must be moved to `/PX4-Autopilot/Tools/simulation/gz/models` once the repository is cloned.
2. The file `aruco.sdf` must be moved to `/PX4-Autopilot/Tools/simulation/gz/worlds`.

## Launching the Simulation
```bash
cd drone
colcon build
source install/setup.bash
cd src
ros2 launch px4_offboard offboard_velocity_control.launch.py
```

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

## Modifications to the World and Drone
The simulation environment has been modified as follows:

1. The **drone model** has been customized to include a camera.
2. An **ArUco marker** has been added, placed on a parallelepiped to improve its visibility from the drone's camera.
3. The drone can detect the marker, process the associated image, and adjust its trajectory accordingly.

## **Approach and Detection Script**  

A set of scripts has been implemented to manage the drone's approach and marker detection process. The system follows these steps:  

1. **Positioning:** At startup, the drone is positioned **6 meters away** from the ArUco marker.  
2. **Approach Phase:** The drone autonomously moves **closer to 1 meter** from the marker before starting the detection.  
3. **Marker Detection:** When the **ArUco marker with ID 42** is detected, the system **publishes the associated image** (`Bologna.jpg`) to the `/processed_camera/image` topic.  
4. **Real-time Telemetry Display:** The system continuously logs **position data, altitude, and marker detection status**, which can be visualized in real-time.  

The detection and control system is built using multiple ROS2 nodes, each handling a specific aspect of the drone’s operation, including camera image processing, trajectory tracking, and telemetry visualization.

## Contributing
If you wish to contribute, feel free to fork the repository and submit a pull request.

## License
This project is distributed under the MIT License. See the LICENSE file for more details.

## **Setup Instructions**  

### **1 Move Rrquired Models**  

Before launching the simulation, the necessary models must be placed in the PX4 directories to ensure proper rendering of the ArUco marker and the modified drone model. Run the following commands:  

```bash
cp -r ~/drone/src/px4_offboard/arucotag ~/PX4-Autopilot/Tools/simulation/gz/models/
cp -r ~/drone/src/px4_offboard/x500_mono_cam ~/PX4-Autopilot/Tools/simulation/gz/models/
cp ~/drone/src/px4_offboard/aruco.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```
### **2. Description of the required Files**  

The following files are essential for the correct setup of the simulation environment:  

- **`arucotag`**: The 3D model of the **ArUco marker**, used for detection during the simulation.  
- **`x500_mono_cam`**: A **modified drone model equipped with an integrated camera**, enabling real-time image capture for marker detection.  
- **`aruco.sdf`**: The **simulation world file** containing the ArUco marker and defining its placement within the environment.  

These files must be correctly placed in the designated PX4 directories to ensure proper functionality of the system.

### **Modifying the Image Path**  

The script responsible for publishing the associated image when the **ArUco marker ID 42** is detected requires specifying the correct path to the image file.  

By default, the image path is set as:  

```python
self.bologna_image_path = "/home/martina/drone/src/px4_offboard/scripts/Bologna.jpg"
```
To ensure the script correctly locates the image, modify this path to match the actual location of **Bologna.jpg** on your system. 

## Final Step: Launching the Simulation
Once you have completed all the passages, you can launch the simulation running:
```bash
cd drone
colcon build
source install/setup.bash
cd src
ros2 launch px4_offboard offboard_velocity_control.launch.py
```

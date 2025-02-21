# ArucoDrone: Marker Detection and Trajectory Tracking System

## Overview

**ArucoDrone** is a project developed for detecting **ArUco markers** using a drone controlled via **PX4** and **ROS2**. This project utilizes **Gazebo** simulator and computer vision tools to enable the drone to:

- Detect an **ArUco marker** in the simulation
- Analyze the content of the marker and publish an associated image when the marker with ID 42 is detected
- Follow a predefined trajectory and maintaining position
- Dynamically visualize real-time telemetry data through **Python plots**

## Project Structure

```
ros_ws/
│── src/
│   │── px4_msgs                     # PX4 ROS2 Messages (to clone)
│   │── ros_gz                       # ROS2 pkgs (to clone)
│   │── px4_offboard/
│   │   ├── scripts/                  # Python scripts for control and visualization
│   │   │   ├── detection.py               # Detects ArUco markers in the camera feed
│   │   │   ├── camera_publisher_node.py   # Publishes camera images as ROS2 topics
│   │   │   ├── offboard.py                # Controls the drone in offboard mode
│   │   │   ├── odometry_logger.py         # Logs and visualizes odometry data
│   │   │   ├── processes.py               # Automates ROS node launches
│   │   │   ├── visualizer.py              # Displays real-time telemetry and position
│   │   ├── resource/                 # Resources like SDF models and visualizations
│   │   │   ├── aruco.sdf                   # SDF world file with ArUco marker
│   │   │   ├── detect_aruco.py             # Detects ArUco markers in Gazebo
│   │   │   ├── generate_aruco.py           # Generates ArUco markers
│   │   │   ├── visualize.rviz              # Rviz configuration for visualization
│   │   │   ├── arucotag/                    # ArUco tag 3D model
│   │   │   ├── x500_mono_cam/               # Drone model with a monocular camera
│   │   ├── launch/
│   │   │   ├── offboard_velocity_control.launch.py  # Launches the offboard node
│── README.md                    # Project documentation
│── LICENSE                       # Project license


```

## Prerequisites & Installation Guide

**Clone the Repository and Required Dependencies**

In your ROS2 workspace clone this repository

```bash
git clone https://github.com/martinaliberatoscioli/drone
```

### 1. Install PX4 Autopilot
PX4 is the flight control system required for the drone simulation.

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
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
cd ~/ros_ws/src
```
Now, clone in your local workspace the required **ROS2 packages**

```bash
git clone https://github.com/gazebosim/ros_gz.git
```
and the required **px4_msgs**:

```bash
git clone https://github.com/PX4/px4_msgs.git
```

Then, build the workspace:
```bash
cd ~/ros_ws
colcon build --symlink-install
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

## **Setup Instructions**  

### **1 Move Required Models**  

Before launching the simulation, the necessary models must be placed in the PX4 directories to ensure proper rendering of the ArUco marker and the modified drone model. Run the following commands:  

```bash
cp -r ~/ros_ws/px4_offboard/resource/arucotag ~/PX4-Autopilot/Tools/simulation/gz/models/
cp -r ~~/ros_ws/px4_offboard/resource/x500_mono_cam ~/PX4-Autopilot/Tools/simulation/gz/models/
cp ~~/ros_ws/px4_offboard/resource/aruco.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```
### **2. Description of the Resource Files**  

The **`resource/`** folder contains essential files required for the correct setup of the simulation environment and marker detection::  

- **`arucotag`**: The 3D model of the **ArUco marker**, used for detection during the simulation.  
- **`x500_mono_cam`**: A **modified drone model equipped with a monocular camera**, to capture and process real-time images of the ArUco marker. 
- **`aruco.sdf`**: The **simulation world file** containing the ArUco marker and defining its placement within the environment, including the position and environment of the ArUco marke.
- **`detect_aruco.py`**: Script responsible for detecting ArUco markers from the drone’s camera feed and processing the image data.
- **`generate_aruco.py`**: Generates ArUco marker patterns for use in the simulation environment.
- **`visualize.rviz`**: Configuration file for Rviz, allowing real-time visualization of the drone’s position and detected markers.

These files must be correctly placed in the designated PX4 directories to ensure proper functionality of the system.

### **Modifying the Image Path**  

The script responsible for publishing the associated image when the **ArUco marker ID 42** is detected requires specifying the correct path to the image file.  

By default, the image path is set as:  

```python
self.bologna_image_path = "/home/martina/ros_ws/src/px4_offboard/scripts/Bologna.jpg"
```
To ensure the script correctly locates the image, modify this path to match the actual location of **Bologna.jpg** on your system. 


## Scripts Description

### **1. visualizer.py**
Visualizes drone telemetry and trajectory in real-time:
- Subscribes to PX4 topics for **attitude**, **local position**, and **trajectory setpoints**.
- Publishes visual markers for:
  - `/px4_visualizer/vehicle_pose` (current drone position)
  - `/px4_visualizer/vehicle_velocity` (velocity vector visualization)
  - `/px4_visualizer/vehicle_path` (actual trajectory)
  - `/px4_visualizer/setpoint_path` (desired trajectory)

### **2. processes.py**
Automates launching all necessary components:
- Starts **Micro XRCE-DDS Agent** for PX4-ROS2 communication.
- Launches **PX4 SITL** with **x500_mono_cam** drone model in **Ignition Gazebo**.
- Runs **offboard control**, **odometry logging**, and optional **detection and camera nodes**.

### **3. offboard.py**
Controls the drone in **OFFBOARD mode**:
- Arms the drone and switches to OFFBOARD mode.
- Sends **position setpoints** to move the drone to a target location `(2.5, 2.0, -2.0)`.
- Monitors position and keeps the drone stable upon reaching the target.
- Uses **MAVLink commands** for arm/disarm and mode switching.

### **4. odometry_logger.py**
Logs and visualizes drone odometry data:
- Subscribes to **odometry and vehicle status** topics.
- Saves data to **CSV (`~/odometry_data.csv`)**.
- Displays **real-time graphs** for:
  - Position (X, Y, Z)
  - Velocity (X, Y, Z)
  - Orientation (quaternions)
- Only records data when the drone is **armed**.

### **5. detection.py**
Detects **parallelepipeds and ArUco markers**:
- Processes camera feed to find a **red parallelepiped** and **ArUco markers**.
- If **marker ID 42** is detected, **publishes an image** (`Bologna.jpg`).
- Publishes the **marker’s position** to `/aruco_pose`.

### **6. camera_publisher_node.py**
Manages the camera feed:
- Receives images from `/rgbd_camera/image`.
- Displays the feed in **real-time** using OpenCV.
- Publishes processed images to `/processed_camera/image`.

---
## Final Step: Launching the Simulation
Once you have completed all the passages, you can launch the simulation running:
```bash
cd ros_ws
colcon build
source install/setup.bash
cd src
ros2 launch px4_offboard offboard_velocity_control.launch.py
```
To see the associated image of the detected ArUco marker, open a new terminal and type:
```bash
rqt
```
## Contributing
If you wish to contribute, feel free to fork the repository and submit a pull request.

## License
This project is distributed under the MIT License. See the LICENSE file for more details.

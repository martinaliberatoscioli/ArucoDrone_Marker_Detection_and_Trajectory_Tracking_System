# ArucoDrone: Marker Detection System

## Overview

**ArucoDrone** is a project developed for the detection of ArUco markers using a drone controlled via PX4 and ROS2. By utilizing simulators like Gazebo and computer vision tools, this project demonstrates how a drone can identify and approach an ArUco marker in a simulated environment.

This repo is a derivative of Jaeyoung Lim's Offboard example https://github.com/Jaeyoung-Lim/px4-offboard

I've taken his example and added some functionality.

## Prerequisites

Ensure you have the following components installed:

- PX4 Autopilot
- ROS2 Humble
- QGroundControl
- Micro XRCE-DDS Agent
- RViz
- Ubuntu 22
- Python 3.10

## Setup Steps

### Install PX4 Autopilot

la sezione "Install PX4 Autopilot" del tuo README:

### Install PX4 Autopilot

To install PX4, run the following code:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.14
```
Run this script in a bash shell to install everything

```bash
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
You will now need to restart your computer before continuing.

### Install ROS2 Humble

To install ROS2 Humble, follow the steps [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

### Install Dependencies

Install Python dependencies as mentioned in the PX4 Docs with the following command:

```bash
pip3 install --user -U empy pyros-genmsg setuptools
```
Additionally, I found that without these packages installed, Gazebo has issues loading:

```bash
pip3 install kconfiglib
pip install --user jsonschema
pip install --user jinja2
```
### Build Micro DDS

As mentioned in the PX4 Docs, run the following commands to build MicroDDS on your machine:

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

This git repo is intended to be a ROS2 package that is cloned into a ROS2 workspace.

We're going to create a workspace in our home directory, and then clone this repo and the `px4_msgs` repo.

For more information on creating workspaces, see [here](https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-Your-Workspace.html).

Run the following code to create a workspace in your home directory:

```bash
mkdir -p ~/ros2_px4_offboard_example_ws/src
cd ~/ros2_px4_offboard_example_ws/src
```
### Clone in Packages 

We first will need the `px4_msgs` package. Our ROS2 nodes will rely on the message definitions in this package in order to communicate with PX4. Read [here](https://docs.px4.io/master/en/ros/ros2.html) for more information.

Be sure you're in the `src` directory of your workspace and then run the following command to clone the `px4_msgs` repo:

```bash
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
```

Once again be sure you are still in the src directory of your workspace. Run this code to clone in our example package

```bash
git clone https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example.git
```

## Installation

1. Clone the repository:
   ```bash
   git clone 
   cd 
2. Follow the instructions to set up PX4 and ROS2.

3. Copy the ROS2_PX4_Offboard_Example into your workspace and prepare your configuration.

## Running the System

After setting up the environment, you can start the system by running the following commands. This will launch several processes:

- **processes.py** in a new window
- **MicroDDS** in a new terminal window
- **Gazebo** in a second tab in the same terminal window
- **Gazebo GUI** in its own window
- **control.py** in a new window
- **RViz** in a new window

### Drone Controls

Use the following commands to control the drone:

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
If you wish to contribute to the project, feel free to fork the repository and submit a pull request.

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





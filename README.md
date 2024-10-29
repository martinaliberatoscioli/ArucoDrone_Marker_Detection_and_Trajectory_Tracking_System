# ArucoDrone: Marker Detection System

## Description

**ArucoDrone** is a project developed for the detection of ArUco markers using a drone controlled via PX4 and ROS2. By utilizing simulators like Gazebo and computer vision tools, this project demonstrates how a drone can identify and approach an ArUco marker in a simulated environment.

## Prerequisites

Ensure you have the following components installed:

- PX4 Autopilot
- ROS2 Humble
- QGroundControl
- Micro XRCE-DDS Agent
- RViz
- Ubuntu 22
- Python 3.10

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/repository-name.git
   cd repository-name
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

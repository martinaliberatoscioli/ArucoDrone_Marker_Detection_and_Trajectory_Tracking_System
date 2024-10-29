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

To install PX4, run the following code:

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b release/1.14


# Maze Navigation using Potential Field Method
**Autonomous robot navigation in a simulated maze — ROS 2 Final Project**

## 📋 Project Overview
This project implements an autonomous navigation system for a mobile robot within a Gazebo simulation. Using the **Artificial Potential Field (APF)** algorithm, the robot navigates from a start point to a goal while dynamically avoiding obstacles.

### 🛠 Technical Details
| Field | Details |
| :--- | :--- |
| **Robot** | TurtleBot3 Burger (Differential Drive) |
| **Algorithm** | Artificial Potential Field (APF) |
| **Start / Goal** | (0.5, 0.5) → (9.0, 9.0) |
| **Localisation** | Ground-truth odometry from Gazebo (Immune to drift) |
| **Students** | Haneen Awad • Nada Al-Saqa |
| **Instructor** | Dr. Marwan Radi |

---

## ⚙️ Prerequisites
Ensure you have the following packages installed on your ROS 2 Jazzy system:

| Package | Installation Command |
| :--- | :--- |
| **ROS 2 Jazzy** | `sudo apt install ros-jazzy-desktop` |
| **Gazebo Harmonic** | `sudo apt install gz-harmonic` |
| **TurtleBot3** | `sudo apt install ros-jazzy-turtlebot3 ros-jazzy-turtlebot3-gazebo` |
| **Bridge & Sim** | `sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim` |

---

## 🚀 Quick Start

### 1. Source and Build
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_project_ws
colcon build --symlink-install && source install/setup.bash
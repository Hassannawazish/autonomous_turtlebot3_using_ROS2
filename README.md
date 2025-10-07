# 🤖 Autonomous TurtleBot3 using ROS2
**Author:** Hassan Nawazish  
**Platform:** ROS 2 Humble (Ubuntu 20.04)  
**Simulation Tools:** Gazebo, RViz, Foxglove  

---

## 🧭 Overview

The goal of this project is to create an **autonomous robot** using **Camera**, **LiDAR**, **IMU**, and **GPS** sensors with **ROS 2 (Humble)**.  
The system is designed to **mimic the behavior of an autonomous vehicle**, featuring **SLAM (Simultaneous Localization and Mapping)** or **Visual SLAM** for real-time mapping and navigation.

The project includes visualization in **RViz**, simulation in **Gazebo**, and monitoring in **Foxglove**.

---

## 🧠 Features

- Sensor fusion using **Camera**, **LiDAR**, **IMU**, and **GPS**
- Integration with **SLAM** or **Visual SLAM**
- Real-time simulation in **Gazebo**
- Visualization in **RViz**
- Data streaming and monitoring with **Foxglove**
- Fully autonomous navigation using the **Nav2** stack

---

## ⚙️ Installation

### 1. Source ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash

### 2. Install turtlebot3 package in ROS2
```bash
$ sudo apt install ros-humble-turtlebot3*


This installs:

turtlebot3_description (URDF)

turtlebot3_bringup (real-world)

turtlebot3_gazebo (simulation)

turtlebot3_navigation2 (Nav2 stack)

$ export TURTLEBOT3_MODEL=burger

After selection we have to run the command given below to start the turtlebot3 to start the turtlebot2's Gazeebo simulation process in terminal 1. And it will be start always behind the other processes which we we do below in other programs.

$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

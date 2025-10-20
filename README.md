# 🤖 Autonomous TurtleBot3 using ROS2
**Platform:** ROS 2 Humble (Ubuntu 22.04)  
**Simulation Tools:** Gazebo, RViz, Foxglove  

---

## 🧭 Overview

The goal of this project is to create an **autonomous robot** using **Camera**, **LiDAR**, **IMU**, and **GPS** sensors with **ROS 2 (Humble)**.  
The system is designed to **mimic the behavior of an autonomous vehicle**, featuring **SLAM (Simultaneous Localization and Mapping)** or **Visual SLAM** for real-time mapping and navigation.
The simulation works perfectly but on hardware there can be some issue of robousness we have to change the conditions according to Lidar used.

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

### 1. Source ROS 2 Environment, Install turtlebot3 package and run turtlebot3 on Terminal 1
```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-turtlebot3*
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### This installs:
turtlebot3_description (URDF)
turtlebot3_bringup (real-world)
turtlebot3_gazebo (simulation)
turtlebot3_navigation2 (Nav2 stack)

After selection we have to run the command given above to start the turtlebot3 to start the turtlebot2's Gazeebo simulation process in terminal(1). And it will be start always behind the other processes which we we do below in other programs.

## Run

### 2. Running Autonomously controlled turtlebot3 using ROS2 in Terminal 2 using shell script
```bash
cd ~/ros2_ws/src/
chmod +x run.sh
./run.sh
```

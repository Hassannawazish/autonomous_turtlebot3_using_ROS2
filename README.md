# Autonomous Turtlebot3 using ROS2
The  goal is to create an autonomous robot using Camera, Lidar, IMU, and GPS using ros2. Visualize on rviz, simulate on gazeebo and show it on foxglove. It is to mimic the behavior of autonomous vehicle. It may include the SLAM or visual slam at well.  

## Install turtlebot3 package in ROS2

$ source /opt/ros/humble/setup.bash
$ sudo apt install ros-humble-turtlebot3*
This installs:

turtlebot3_description (URDF)

turtlebot3_bringup (real-world)

turtlebot3_gazebo (simulation)

turtlebot3_navigation2 (Nav2 stack)

$ export TURTLEBOT3_MODEL=burger

After selection we have to run the command given below to start the turtlebot3 to start the turtlebot2's Gazeebo simulation process in terminal 1. And it will be start always behind the other processes which we we do below in other programs.

$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

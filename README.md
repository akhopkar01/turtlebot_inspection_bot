# Turtlebot Inspection Robot
[![Build Status](https://travis-ci.org/kartikv97/Turtlebot_Inspection_Robot.svg?branch=master)](https://travis-ci.org/kartikv97/Turtlebot_Inspection_Robot)
[![Coverage Status](https://coveralls.io/repos/github/kartikv97/Turtlebot_Inspection_Robot/badge.svg?branch=master)](https://coveralls.io/github/kartikv97/Turtlebot_Inspection_Robot?branch=master)

---
## Authors
Currently pursuing M.Eng in Robotics from University of Maryland, College Park.
- Kartik Venkat - B.Eng in Electronics and Telecommunication Engineering from SIES Graduate School of Technology, Mumbai.
- Kushagra Agrawal - B.Tech in Mechanical Engineering from Manipal Institute of Technology.
- Aditya Khopkar - 

## Overview

In this project we are developing a real time anomaly detection robot based on TurtleBot 3. 
We will be creating a ROS package (ROS Melodic) and will demonstrate our implementation in a 
Gazebo Simulation environment. The robot will keep roaming around the map while searching for 
anomalies. Once an anomaly is detected, the robot will reach the detected location and report
the anomaly. 

## AIP Document
[![AIP](https://img.shields.io/badge/AIP-Click%20Here-red)](https://docs.google.com/spreadsheets/d/1gK6UU1C03G-Nt6Inuk5zHCRxUzo2bpcLRpkTf8MvC3I/edit?usp=sharing)


## Dependencies
```
1. ROS melodic
2. Catkin
3. Ubuntu 18.04 LTS
4. TurtleBot Gazebo
5. OpenCv
```
Install ROS melodic and setup catkin workspace by following this tutrial:
1. [Link to ROS tutorial!](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

## Standard install via command-line
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/kartikv97/TurtleBot_Gazebo_Walker.git
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make
```

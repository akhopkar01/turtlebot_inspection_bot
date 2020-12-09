# **Turtlebot Inspection Robot ROS Package**
[![Build Status](https://travis-ci.org/kartikv97/turtlebot_inspection_bot.svg?branch=master)](https://travis-ci.org/kartikv97/turtlebot_inspection_bot)
[![Coverage Status](https://coveralls.io/repos/github/kartikv97/turtlebot_inspection_bot/badge.svg?branch=master)](https://coveralls.io/github/kartikv97/turtlebot_inspection_bot?branch=master)

---
## Authors
* **Kartik Venkat :** M.Eng Robotics, UMD | B.Eng Electronics and Telecommunication Engineering, University of Mumbai.
* **Kushagra Agrawal :** M.Eng Robotics, UMD | B.Tech Mechanical Engineering, Manipal Institute of Technology.
* **Aditya Khopkar :** M.Eng Robotics, UMD | B.Eng Electronics Engineering, University of Mumbai 

## TODO
- Sprint Week 2
    - [X] Create worlds/anomalies.world 
    - [X] Stub implementation
    - [X] Unit tests
- Sprint Week 3
    - [ ] Update worlds/anomalies.world to encode color information
    - [ ] Complete both Node Implementation
    - [ ] Verify that all Tests pass
    - [ ] Verify atleast 80% Code Coverage

## Overview

In this project we are developing a real time anomaly detection robot based on TurtleBot 3. 
We will be creating a ROS package (ROS Melodic) and will demonstrate our implementation in a 
Gazebo Simulation environment. The robot will keep roaming around the map while searching for 
anomalies. Once an anomaly is detected, the robot will reach the detected location and report
the anomaly. 

## AIP Document
[![AIP](https://img.shields.io/badge/AIP-Click%20Here-red)](https://docs.google.com/spreadsheets/d/1gK6UU1C03G-Nt6Inuk5zHCRxUzo2bpcLRpkTf8MvC3I/edit?usp=sharing)
[![SprintDoc](https://img.shields.io/badge/SprintDoc-Click%20Here-red)](https://docs.google.com/document/d/1NFZc3CICtRCiKvu_DC-juLE--KWvMurhhtYTClnU67w/edit?usp=sharing)

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
## Visualize Simulation Environment in Gazebo
```
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make
roslaunch turtlebot_inspection_bot turtlebot_world.launch
```
## Run ROS Test(Work in progress)
```
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make
catkin_make tests              
```
**NOTE:** To be resolved in Phase 3 Implementation. (currently Failing due to the stub implementations and inactive ROS Nodes)[]
**Note:** Press **ctrl+c** in the terminal to stop the program.

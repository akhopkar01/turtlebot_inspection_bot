# **Turtlebot Inspection Robot ROS Package**
[![Build Status](https://travis-ci.org/kartikv97/turtlebot_inspection_bot.svg?branch=master)](https://travis-ci.org/kartikv97/turtlebot_inspection_bot)
[![Coverage Status](https://coveralls.io/repos/github/kartikv97/turtlebot_inspection_bot/badge.svg?branch=master)](https://coveralls.io/github/kartikv97/turtlebot_inspection_bot?branch=master)

---
## Overview

Anomalies in any workplace is considered unwanted and risky in nature. Such workplaces may include a pharamceutical warehouse, a logistic warehouse or even a packaging warehouse. The anomalies can disrupt the setting of the workplace or may be the cause of something much more risky depending on the nature of these anomalies. For example, an unwanted substance in the pharmaceutical warehouse may result in a severe scare. This could even have damaging ramifications to the industry. This is the basis of our motivation for thi project. <br>

In this project, we developed a real time inspection robot using TurtleBot 3. 
We have created an executable ROS package (ROS Melodic) which autonomously navigates the turtlebot in the environment while detecting anomalies by detecting the anomaly color. Thus, the project leverages the idea of greedily recognizing the anomaly color (in this case green) in an environment with red colored objects which resemble properly working objects/machines. In this version of the project, the robot can only recognize the color of the workplace objects. We demonstrate our implementation in a Gazebo Simulation environment with RVIZ. The robot when identifies an anomaly, suggests the coordinates of the anomaly with respect to the robot coordinate frame in real-time. <br>
![Fig1. Anomaly Detection](https://github.com/kartikv97/turtlebot_inspection_bot/blob/dev/media/anomaly%20detection.png)
We followed an Agile development process with TDD approach to develop the project in 3 sprints. This README provides a walk-through for our project with installation steps and execution steps.
![Gif. Product Demo](https://github.com/kartikv97/turtlebot_inspection_bot/blob/dev/media/DemoVideo.gif)

## Authors
* **Kartik Venkat :** M.Eng Robotics, UMD | B.Eng Electronics and Telecommunication Engineering, University of Mumbai.
* **Kushagra Agrawal :** M.Eng Robotics, UMD | B.Tech Mechanical Engineering, Manipal Institute of Technology.
* **Aditya Khopkar :** M.Eng Robotics, UMD | B.Eng Electronics Engineering, University of Mumbai 

### TODO
- Sprint Week 2
    - [X] Create worlds/anomalies.world 
    - [X] Stub implementation
    - [X] Unit tests
- Sprint Week 3
    - [X] Update worlds/anomalies.world to encode color information
    - [X] Complete both Node Implementation
    - [X] Verify that all Tests pass
    - [X] Verify atleast 80% Code Coverage

## AIP Document
[![AIP](https://img.shields.io/badge/AIP-Click%20Here-red)](https://docs.google.com/spreadsheets/d/1gK6UU1C03G-Nt6Inuk5zHCRxUzo2bpcLRpkTf8MvC3I/edit?usp=sharing)
[![SprintDoc](https://img.shields.io/badge/SprintDoc-Click%20Here-red)](https://docs.google.com/document/d/1NFZc3CICtRCiKvu_DC-juLE--KWvMurhhtYTClnU67w/edit?usp=sharing)

## Project Demonstration and Presentation
[![Presentation](https://img.shields.io/badge/Presentation-Click%20Here-red)](https://drive.google.com/file/d/1rKyKeaj7__AZYH1FggYsRepB-CGkGnkJ/view?usp=sharing)
[![DemoVideo](https://img.shields.io/badge/DemoVideo-Click%20Here-red)](https://drive.google.com/file/d/1lKAOqqY6jtvYOdxs0ZVOnK9ceASdSPxW/view?usp=sharing)
[![Slides](https://img.shields.io/badge/Slides-Click%20Here-red)](https://docs.google.com/presentation/d/1DKFuClv0wYsrBi0umdT-RRENfO7O2EHeqmnz29CQeoA/edit?usp=sharing)
You can access Project Live Technical Presentation, Project Demo Video and Project Slides from these tags, respectively

## Dependencies

### Direct Dependencies
1. [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
2. Ubuntu 18.04 LTS
3. [TURTLEBOT3](https://answers.ros.org/question/293514/turtlebot-installation-on-ros-melodic/)

### Other Dependencies
1. catkin: Comes default with ROS-melodic installation
2. Gazebo: Comes default with ROS-melodic installation
3. OpenCV 3.2.0: Comes default with Ubuntu-18.04 and ROS Melodic.

### Package Dependencies
1. cv_bridge
2. geometry_msgs
3. image_transport
4. move_base_msgs
5. roscpp
6. sensor_msgs
7. std_msgs

Install ROS melodic and setup catkin workspace by following this tutrial:
1. [Link to ROS tutorial!](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

## Step 1 : Standard install via command-line
Follow the following steps for comprehensive installation guide of the package:

```
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/kartikv97/turtlebot_inspection_bot.git
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```
## Step 2 : Visualize Simulation Environment in Gazebo
Gazebo and RVIZ packages are used by this package for visualization. Refer the following figure. After following Step 1, in the same terminal follow the following steps:
```
$ export TURTLEBOT3_MODEL=waffle_pi
$ roslaunch turtlebot_inspection_bot turtlebot_simulation.launch
```
![Fig2. Visualization Window](https://github.com/kartikv97/turtlebot_inspection_bot/blob/dev/media/visualization_ros.png)

### rosbag
This package is ```rosbag``` compliant. The bag file can be accessed from results/turtlebot_inspection_bot.bag. You may inspect the bag file by the command ```rosbag info results/*.bag```. The bag file has a 46 seconds long recorded simulation of the package. You may use the bag file to play the simulation results by operating the following commands in three terminals simultaneously:


**Terminal 1:**
```
$ roscore
```


**Terminal 2:**
```
$ cd ~/catkin_ws/src/turtlebot_inspection_bot
$ rosbag play results/*.bag
```


**Terminal 3:**
```
$ rqt_console
```

The package also come with record functionality for rosbag. i.e., you can record your own rosbag for the package by first building the project and launching it as follows:
```
$ roslaunch turtlebot_inspection_bot turtlebot_simulation.launch record:=true
```
The result can be accessed in the results the directory. 

## Run ROS Test
The package has Level 2 Unit Test compliance. You can run the ROS tests by following the following two options:


**Option 1**
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ catkin_make run_tests_turtlebot_inspection_bot              
```


**Option 2** 
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rostest turtlebot_inspection_bot test.launch
```
## Doxygen
To install doxygen run the following command:
```
$ sudo apt-get install doxygen
```
You can run the doxyfile in the package to generate files by the following command:
```
$ doxygen docs/Doxyfile
```
You can access the html file by
```
$ firefox docs/html/index.html
```
## Code Coverage (Locally)
To test the code coverage of the package locally you may follow,
```
$ cd ~/catkin_ws/build 
$ lcov --directory . --capture --output-file coverage.info
$ lcov --remove coverage.info '/opt/*' '/usr/*' '*/devel/*' '*test_*' '*_test*' --output-file coverage.info 
$ lcov --list coverage.info 
$ genhtml coverage.info --output-directory covout
```
This creates index.html file in build/covout, which can be accessed

## Future Work
In our succeeding versions, we intend to do the following:
1. Robust navigation algorithm for the ```mover``` node: Treat this problem as a reactive planning problem to make it robust for any uncertain environment.
2. Intelligent detection for the ```detector``` node: Employ a Deep Neural Network to detect known anomaly classes.
3. Randomize Environment: Randomize spawning of the anomalies randomly in the environment
4. Use transforms: Get the real world coordinates with respect to the world frame than the robot frame.

**Note:** Press **ctrl+c** in the terminal to stop the program.

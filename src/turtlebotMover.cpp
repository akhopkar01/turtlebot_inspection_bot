/**
 *
 * Copyright (c) 2020 Kartik Venkat Kushagra Agrawal Aditya Khopkar
 *
 * @section LICENSE
 *
 * MIT License
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file turtlebotMover.cpp
 *
 * @authors
 * Kartik Venkat kartik.venkat86@gmail.com \n
 * Kushagra Agrawal kushagraagrawal425@gmail.com  \n
 * Aditya Khopkar aadi0110@gmail.com  \n
 *
 * @version 1.0
 *
 * @section DESCRIPTION:
 * This is the implementation for the Turtlebot Mover node.
 */


#include <iostream>
#include <../include/mover.hpp>

#define _ANGLE_ 60

/*
 * @brief Constructor for turtlebotMover class.
 */
TurtlebotMover::TurtlebotMover() {
    pubVel = nh.advertise <geometry_msgs::Twist>
            ("/cmd_vel", 1000);

    subLaserScanner = nh.subscribe<sensor_msgs::LaserScan>
            ("/scan", 1000,
             &TurtlebotMover::scanEnvCallback, this);
    sub_odometry = nh.subscribe<nav_msgs::Odometry>("odom", 1, &TurtlebotMover::odomCallback,this);

    /*
     * @brief Initialize the linear and angular velocities
     * of the Turtlebot.
     */
    velMsg.linear.x = 0.0;
    velMsg.linear.y = 0.0;
    velMsg.linear.z = 0.0;
    velMsg.angular.x = 0.0;
    velMsg.angular.y = 0.0;
    velMsg.angular.z = 0.0;

    /*
     * @brief Publish the initial velocities for the Turtlebot.
     */
    pubVel.publish(velMsg);

    /*
     * @brief Initialize the private variables
     */
    isGoal = false;
    isObstacle = false;
    obstacleThresh = 1.0;
    newDirection = "Right";
}
/*
 *  @brief Callback service to get the live pose of the turtlebot
 *  from the environment
 *  @param msg: Pointer to messages from Odometry sensor.
 *  @return none.
 */
void TurtlebotMover::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    ROS_INFO("Got a transform! x = %f, y = %f",current_pose.x,current_pose.y);
}

/*
 * @brief Callback service to scan the environment for obstacles.
 * @param msg : Pointer to the messages from the LaserScan sensor.
 * @return none.
 */
void TurtlebotMover::scanEnvCallback(const sensor_msgs::LaserScan::
                                                        ConstPtr& msg) {
    auto sensorRanges = msg->ranges;
    for (size_t i = 0; i <= _ANGLE_; ++i) {
        if (sensorRanges[i] < obstacleThresh && sensorRanges[i + 299] < obstacleThresh) {
            isObstacle = true;
            return;
        }
    }
    isObstacle = false;
    return;
}
/*
 * @brief Check if the turtlebot has reached the goal.
 * @param current_pose: type Pose2D (stores current pose
 * of the turtlebot.)
 * @return bool value determining whether goal is reached or not.
 */
bool TurtlebotMover::isGoalReached(geometry_msgs::Pose2D current_pose) {
    if (current_pose.x > -1.0 && current_pose.x < 2.0 && current_pose.y > 1.0 && current_pose.y < 5.0 ) {
        ROS_INFO("Goal REACHED!!! x = %f, y = %f", current_pose.x, current_pose.y);
        return true;
    } else {
        return false;
    }
}
/*
 * @brief Check if an obstacle is present in the Turtlebot's path.
 * @param none.
 * @return isObstacle: bool indicating if obstacle is detected.
 */
bool TurtlebotMover::checkObstacle() {
    return isObstacle;
}

/*
 * @brief Set the isObstacle flag indicating that there is an
 * obstacle ahead.
 * @param obstacle: bool value
 */
void TurtlebotMover::setObstacle(bool obstacle) {
    isObstacle = obstacle;
}

/*
 * @brief Function that turns the TurtleBot according to the direction specified.
 * @param newDirection: std::string indicating new direction.
 * @return none.
 */
double TurtlebotMover::changeDirection(std::string newDirection) {
    double angularVelocity = 0.0;
    if(newDirection=="Right" || newDirection == "RIGHT" || newDirection=="R" || newDirection=="right") {
        angularVelocity = -0.8;
        return angularVelocity;
    } else if(newDirection=="Left" || newDirection == "LEFT" || newDirection=="L" || newDirection=="left") {
        angularVelocity = 0.8;
        return angularVelocity;
    } else {
        angularVelocity = 0.0;
        return angularVelocity;
    }
}

/*
 * @brief Function that moves the TurtleBot in the environment.
 * @param none.
 * @return none.
 */
void TurtlebotMover::moveRobot() {
    ros::Rate loop(10);

    while (ros::ok()) {
        isGoal = isGoalReached(current_pose);
        if (isGoal) {
            ROS_INFO_STREAM("Exiting Program.....");
            return;
        }
        if (checkObstacle()) {
            ROS_WARN_STREAM("Obstacle Detected Ahead!!, Turn Right");
            velMsg.linear.x = 0.0;
            velMsg.angular.z = changeDirection("Right");
        } else {
            ROS_INFO_STREAM("The path ahead is clear. Move Forward !!");
            velMsg.linear.x = 0.2;
            velMsg.angular.z = 0.0;
        }
        pubVel.publish(velMsg);
        ros::spinOnce();
        loop.sleep();
    }
}
/*
 * @brief Destructor for the TurtlebotMover class.
 * @param none.
 * @return none.
 */
TurtlebotMover::~TurtlebotMover() {
    /*
     * @brief Reset the linear and angular velocities
     * of the Turtlebot.
     */
    velMsg.linear.x = 0.0;
    velMsg.linear.y = 0.0;
    velMsg.linear.z = 0.0;
    velMsg.angular.x = 0.0;
    velMsg.angular.y = 0.0;
    velMsg.angular.z = 0.0;

    /*
     * @brief Publish the rested velocities for the Turtlebot.
     */
    pubVel.publish(velMsg);
}
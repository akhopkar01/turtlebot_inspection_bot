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

/*
 * @brief Constructor for turtlebotMover class.
 */
TurtlebotMover::TurtlebotMover() {
    pubVel = nh.advertise <geometry_msgs::Twist>
            ("/cmd_vel", 1000);

    subLaserScanner = nh.subscribe<sensor_msgs::LaserScan>
            ("/scan", 1000,
             &TurtlebotWalker::scanEnvCallback, this);
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
    isObstacle = false;
    obstacleThresh = 1.0;
    newDirection = "Right"
}


/*
 * @brief Callback service to scan the environment for obstacles.
 * @param msg : Pointer to the messages from the LaserScan sensor.
 * @return none.
 */
void TurtlebotMover::scanEnvCallback(const sensor_msgs::LaserScan::
                                                        ConstPtr& msg) {
    return;
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
 * @brief Function that changes the direction of the Turtlebot.
 * @param newDirection: std::string indicating new direction.
 * @return none.
 */
void TurtlebotMover::changeDirection(std::string newDirection) {
    return;
}

/*
 * @brief Function that moves the Turtlebot in the environment.
 * @param none.
 * @return none.
 */
void TurtlebotMover::moveRobot() {
    return;
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
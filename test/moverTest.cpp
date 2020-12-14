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
 * @file moverTest.cpp
 *
 * @authors
 * Kartik Venkat kartik.venkat86@gmail.com \n
 * Kushagra Agrawal kushagraagrawal425@gmail.com  \n
 * Aditya Khopkar aadi0110@gmail.com  \n
 *
 * @version 1.0
 *
 * @section DESCRIPTION:
 * This is the implementation for unit test for TuttlebotMover node..
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <mover.hpp>



/**
 * @ brief Test for checkObstacle method of the TurtlebotMover class.
 */
TEST(Turtlebotmover, checkObstacle) {
    TurtlebotMover turtlebot;
    bool obstacle = false;
    turtlebot.setObstacle(obstacle);
    EXPECT_FALSE(turtlebot.checkObstacle());
}
/**
 * @ brief Test for changeDirection method of the class.
 */
TEST(Turtlebotmover, changeDirection) {
    TurtlebotMover turtlebot;
    std::string newDirection = "Right";
    EXPECT_EQ(-0.8, turtlebot.changeDirection(newDirection));
    newDirection = "Left";
    EXPECT_EQ(+0.8, turtlebot.changeDirection(newDirection));
}
/**
 * @ brief Test for moveRobot method of the class.
 */
TEST(Turtlebotmover, moveRobot) {
    TurtlebotMover turtlebot;
    EXPECT_NO_FATAL_FAILURE(turtlebot.moveRobot(true));
    bool obstacle = true;
    turtlebot.setObstacle(obstacle);
    EXPECT_NO_FATAL_FAILURE(turtlebot.moveRobot(true));
}

/*
 * @brief Test to check goalReached method of the class.
 */
TEST(Sensing, GoalReached) {
    TurtlebotMover turtlebot;
    geometry_msgs::Pose2D current_pose;
    current_pose.x = 10;
    current_pose.y = 10;
    EXPECT_EQ(false, turtlebot.isGoalReached(current_pose));
    current_pose.x = 1;
    current_pose.y = 2;
    EXPECT_EQ(true, turtlebot.isGoalReached(current_pose));
}

/*
 * @brief LEVEL 2 ROS Test
 */
TEST(Sensing, LaserScan) {
TurtlebotMover turtlebot;
ros::NodeHandle n;
ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>(
    "/scan", 1000, &TurtlebotMover::scanEnvCallback, &turtlebot);
EXPECT_NO_FATAL_FAILURE(sub);
}

/*
 * @brief LEVEL 2 ROS Test
 */
TEST(Sensing, pubVel) {
    TurtlebotMover turtlebot;
    ros::NodeHandle n;

    // Initialize a Publisher that publishes velocity messages.
    geometry_msgs::Twist velMsg;
    ros::Publisher pubVel = n.advertise <geometry_msgs::Twist>
        ("/cmd_vel", 1000);
    velMsg.linear.x = 5.0;
    velMsg.angular.z = 7.0;
    pubVel.publish(velMsg);

    /* Initialize a Subscriber that subscribes to
       the published velocity messages.*/
    ros::Subscriber getVel = n.subscribe<geometry_msgs::Twist>
        ("/cmd_vel", 1000, &TurtlebotMover::velocityCallback, &turtlebot);
    ros::spinOnce();

    // Verify the published messages get updated.
    EXPECT_EQ(5.0, turtlebot.linX);
    EXPECT_EQ(7.0, turtlebot.angZ);
}

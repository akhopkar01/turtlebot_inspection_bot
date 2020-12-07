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
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <mover.hpp>

TurtlebotMover turtlebot;
/**
 * @ brief Test for checkObstacle method of the TurtlebotMover class.
 */
TEST(Turtlebotmover, checkObstacle) {
    bool obstacle = false;
    turtlebot.setObstacle(obstacle);
    EXPECT_FALSE(turtlebot.checkObstacle());
}
/**
 * @ brief Test for changeDirection method of the class.
 */
TEST(Turtlenotmover, changeDirection) {
    std::string newDirection = "Right";
    EXPECT_NO_FATAL_FAILURE(turtlebot.changeDirection(newDirection));
}
/**
 * @ brief Test for moveRobot method of the class.
 */
TEST(Turtlenotmover, moveRobot) {
EXPECT_NO_FATAL_FAILURE(turtlebot.moveRobot());
}

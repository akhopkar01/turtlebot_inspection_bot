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
 * @file detectorTest.cpp
 *
 * @authors
 * Kartik Venkat kartik.venkat86@gmail.com \n
 * Kushagra Agrawal kushagraagrawal425@gmail.com  \n
 * Aditya Khopkar aadi0110@gmail.com  \n
 *
 * @version 1.0
 *
 * @section DESCRIPTION:
 * This is the unit test implementation for anomaly detector node.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <detector.hpp>


/**
 * @brief Test to check the detectAnomaly method.
 */
TEST(checkDetector, anomaly) {
    ros::NodeHandle nh;
    AnomalyDetector anomalydetector(nh);
//    cv::Mat img2 = cv::imread("/home/kartik/catkin_ws/src/"
//                              "turtlebot_inspection_bot/testImg.png");
    cv::Mat img2 = cv::imread("./testImg.png");
    anomalydetector.cvImg_ = img2;
    EXPECT_NO_FATAL_FAILURE(anomalydetector.getImgPoints());
}
/**
 * @ brief Test to check the localizePoints method of the class.
 */
TEST(checkDetector, localizePoints) {
    ros::NodeHandle nh;
    AnomalyDetector anomalydetector(nh);
    EXPECT_NO_FATAL_FAILURE(anomalydetector.localizePoints());
}

/**
 * @ brief Test to check the localizePoints method of the class.
 */
TEST(checkDetector, detectAnomaly) {
    ros::NodeHandle nh;
    AnomalyDetector anomalydetector(nh);

//    cv::Mat img2 = cv::imread("/home/kartik/catkin_ws/"
//                              "src/turtlebot_inspection_bot/testImg.png");
    cv::Mat img2 = cv::imread("./testImg.png");
    anomalydetector.cvImg_ = img2;
    EXPECT_NO_FATAL_FAILURE(anomalydetector.getImgPoints());

    EXPECT_NO_FATAL_FAILURE(anomalydetector.detectAnomaly());
}

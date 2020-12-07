/**
 * MIT License

Copyright (c) 2020 Kartik Venkat Kushagra Agrawal Aditya Khopkar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 *
 * @file detectorTest.cpp
 * @authors
 * Kartik Venkat kartik.venkat86@gmail.com \n
 * Kushagra Agrawal kushagraagrawal425@gmail.com  \n
 * Aditya Khopkar aadi0110@gmail.com  \n
 *
 * @version 1.0
 *
 * @section LICENSE
 *
 * MIT License
 * @section DESCRIPTION:
 * This is the unit test for the anomaly detector class.
 */

#include <gtest/gtest.h>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <detector.hpp>

AnomalyDetector anomalydetector;
/**
 * @brief Test to check the detectAnomaly method.
 */
TEST(checkDetector, anomaly) {
    cv::Mat frame = cv::imread("../sample.jpg", 0);
    EXPECT_NO_FATAL_FAILURE(anomalydetector.detectAnomaly(frame));
}
/**
 * @ brief Test to check the localizePoints method of the class.
 */
TEST(checkDetector, localizePoints) {
    cv::Point2i pt(1,2);
    cv::Point3f new_pt = anomalydetector.localizePoints(pt);
    EXPECT_EQ(3,new_pt.x);
    EXPECT_EQ(4, new_pt.y);
    EXPECT_EQ(5, new_pt.z);
}

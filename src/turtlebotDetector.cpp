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
 * @file turtlebotDetector.cpp
 *
 * @authors
 * Kartik Venkat kartik.venkat86@gmail.com \n
 * Kushagra Agrawal kushagraagrawal425@gmail.com  \n
 * Aditya Khopkar aadi0110@gmail.com  \n
 *
 * @version 1.0
 *
 * @section DESCRIPTION:
 * This is the implementation for the Turtlebot Detector node.
 */

#include <detector.hpp>
#include "cv_bridge/cv_bridge.h"

/**
 * @brief: Detector class constructor definition
 * */
AnomalyDetector::AnomalyDetector(cv::Matx34f extP_, cv::Matx34f intP_) {
  imgCoords_{0, 0};
  worldCoords_{0.0f, 0.0f, 0.0f};
  P_ = intP_ * extP_;
  imgSub_ = nh_.subscribe("/camera/rgb/rgb_raw", 1,
                          &AnomalyDetector::imgCallback, this);
}

/**
 * @brief: Callback function definition
 * */
void AnomalyDetector::imgCallback(const 
                        sensor_msgs::Image::ConstPtr& msg) {
  cv_bridge::CvImagePtr cvPtr;
  try {
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cvImg_ = cvPtr->image;
    cv::waitKey(10);
  }
  catch(cv_bridge::Exception& exc) {
    ROS_ERROR_STREAM("CV Bridge Exception " << exc.what());
    return;
  }
}

/**
 * @brief: Detection method definiton
 * */
cv::Point2i AnomalyDetector::detectAnomaly(cv::Mat img) {
//   imgCoords_{0, 0};
  // Perform Image processing color recognition
  // Detect centroid of the recognition
  return imgCoords;
}

/**
 * @brief: get world coordinates definition 
 * */
cv::Point3f AnomalyDetector::localizePoints(cv::Point2i imgCoords) {
  // Perform geometric transformation

  return worldCoords;
}
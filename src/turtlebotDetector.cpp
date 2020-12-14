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
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <config.hpp>

#define COLOR_ cv::Scalar(0, 0, 255)

/**
 * @brief: Detector class constructor definition
 * */
AnomalyDetector::AnomalyDetector(ros::NodeHandle& nh) : it_(nh) {
  P_ = intP * extP;
  cv::namedWindow("Turtlebot Viewer");
  anomalyDetected_ = false;
  subImg_ = it_.subscribe("/camera/rgb/image_raw", 1,
                          &AnomalyDetector::imgCallback, this);
}

AnomalyDetector::~AnomalyDetector() { cv::destroyAllWindows(); }

/**
 * @brief: Detection definition
 * */
void AnomalyDetector::detectAnomaly() {
  getImgPoints();
  // If anomaly is detected
  if (anomalyDetected_) {
    cv::Point3f robotCoords = localizePoints();
    ROS_WARN_STREAM("Anomaly Detected at: " << robotCoords);
  } else {
    ROS_INFO_STREAM("Exploring..");
  }
  cv::imshow("Turtlebot Viewer", cvImg_);
  cv::waitKey(3);
}

/**
 * @brief: Callback function definition
 * */
void AnomalyDetector::imgCallback(const
                       sensor_msgs::Image::ConstPtr& msg) {
  cv_bridge::CvImagePtr cvPtr;
  try {
    // Convert ROS image to OpenCV image
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cvImg_ = cvPtr->image;
    // Check if image is empty
    if (cvImg_.empty()) {
      ROS_ERROR_STREAM("No callback received, Waiting..");
    } else {
      detectAnomaly();
    }
  }
  catch(cv_bridge::Exception& exc) {
    ROS_ERROR_STREAM("CV Bridge Exception " << exc.what());
    return;
  }
}

/**
 * @brief: Get image points from the image
 * */
void AnomalyDetector::getImgPoints() {
  imgCoords_ = cv::Point2i(0, 0);
  anomalyDetected_ = false;
  cv::Mat hsvImg;

  // Convert image to HSV
  cv::cvtColor(cvImg_, hsvImg, cv::COLOR_BGR2HSV);

  // Set threshold to detect green
  // Create a thresholded mask
  cv::Scalar greenLo(35, 40, 40);
  cv::Scalar greenHi(86, 255, 255);
  cv::inRange(hsvImg, greenLo, greenHi, maskImg_);

  // Morphological operations opening
  cv::erode(maskImg_, maskImg_,
          cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
  cv::dilate(maskImg_, maskImg_,
          cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

  // Morphological operations closing
  cv::dilate(maskImg_, maskImg_,
          cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
  cv::erode(maskImg_, maskImg_,
           cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

  // Find centroid of the image
  cv::Moments moments_ = cv::moments(maskImg_);
  double M01{moments_.m01}, M10{moments_.m10};
  double Area{moments_.m00};

  // Find contours to detect bounding box
  cv::Rect bounding_rect;
  std::vector<std::vector<cv::Point2i>> contours;
  cv::findContours(maskImg_, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

  // If considerable area
  if (Area >= 1000.f) {
    // Bounding Box
    bounding_rect = cv::boundingRect(contours[0]);
    int posX = M10 / Area;
    int posY = M01 / Area;
    imgCoords_.x = posX;
    imgCoords_.y = posY;
    cv::putText(cvImg_, "Anomaly", cv::Point2i(posX - 20, posY + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, COLOR_);
    cv::rectangle(cvImg_, bounding_rect, COLOR_);
    anomalyDetected_ = true;
  }
  return;
}

/**
 * @brief: get robot frame coordinates definition 
 * */
cv::Point3f AnomalyDetector::localizePoints() const {
  // Perform geometric transformation
  cv::Matx31f robotPoints;
  cv::Matx33f H{P_(0, 0), P_(0, 1), P_(0, 3),
                P_(1, 0), P_(1, 1), P_(1, 3),
                P_(2, 0), P_(2, 1), P_(2, 3)};
  cv::Matx31f pixel{static_cast<float>(imgCoords_.x),
                    static_cast<float>(imgCoords_.y),
                    1};
  robotPoints = H.inv() * pixel;
  float w = robotPoints(2);
  float u{robotPoints(0)}, v{robotPoints(1)};
  return {u/w, v/w, 0};
}

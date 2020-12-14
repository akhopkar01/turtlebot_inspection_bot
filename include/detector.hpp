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
 * @file detecto.hpp
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
 * This is the Anomaly detector node declaration
 */

#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <opencv2/opencv.hpp>

class AnomalyDetector {
 public:
  /**
   * @brief: Constructor for Anomaly Detector Node
   * @param: Extrinsic Parameter const reference
   * @param: Intrinsic Parameter const reference
   * @return: None
   * */
  explicit AnomalyDetector(ros::NodeHandle&);

  /**
   * @brief: Gateway function to detect anomalies
   * @param: None
   * @return: None
   * */
  void detectAnomaly(bool TEST = false);

  /**
   * @brief: Get Image from ROS Image to CV image using cv_bridge
   * @param: Image reference to const pointer
   * @return: None
   * */
  void imgCallback(const sensor_msgs::Image::ConstPtr& msg);

  /**
   * @brief: Detect anomalies using color mask technique
   * @param: Image frame CV
   * @return: Point in image frame
   * */
  void getImgPoints();

  /**
   * @brief: Convert the image coordinates to 3D world coordinates
   * @param: 2D Image coordinates
   * @return: 3D world coordinates
   * */
  cv::Point3f localizePoints() const;

  /**
   * @brief: Destructor
   * @param: None
   * @return: None
   * */
  ~AnomalyDetector();

  // Store converted CV images
  cv::Mat cvImg_;

 private:
  // ROS Node
  ros::NodeHandle nh_;
  // Create instance of image transport
  // Subscribe image
  image_transport::ImageTransport it_;
  image_transport::Subscriber subImg_;
  // Publish to coordinates to topic
  ros::Publisher pub_;
  // Store converted CV images and Mask images;
  cv::Mat maskImg_;
  // Image Coordinates
  cv::Point2i imgCoords_;
  // camera calibration
  cv::Matx34f P_;
  // Flag to check if anomaly is detected
  bool anomalyDetected_;
};

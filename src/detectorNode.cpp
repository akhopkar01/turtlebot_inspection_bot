
/**
 * @copyright (c) 2020, Kartik Venkat, Kushagra Agrawal, Aditya Khopkar
 *
 * @file main.cpp
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
 * This is the main file to run the Gazebo demo..
 */


#include <ros/ros.h>
#include <detector.hpp>
#include <opencv2/opencv.hpp>

/*
 * @brief main function
 * @param argc
 * @param argv
 * @return 0
 */

int main(int argc, char* argv[]) {
  // Create ROS node
  ros::init(argc, argv, "detector");
  ros::NodeHandle n;
  // Node instance
  AnomalyDetector detector(n);
  // Detection functionality
  while (ros::ok()) {
    // Checks if image is empty
    if (detector.cvImg_.empty()) {
      ROS_ERROR_STREAM("No callback received, Waiting..");
    }
    else {
      detector.getImgPoints();
      // If anomaly is detected
      if (detector.anomalyDetected_) {       
        cv::Point3f robotCoords = detector.localizePoints();
        ROS_WARN_STREAM("Anomaly Detected at: " << robotCoords);
      }
      else {
        ROS_INFO_STREAM("Exploring..");
      }
      cv::imshow("Turtlebot Viewer", detector.cvImg_);
      cv::waitKey(3);
    }
    ros::spinOnce();
  }
  return 0;
}

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
 * @return None
 */

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "detector");
  ros::NodeHandle n;
  AnomalyDetector detector(n);
  return 0;
}

/**
 * @copyright (c) 2020, Kartik Venkat, Kushagra Agrawal, Aditya Khopkar
 *
 * @file moverNode.cpp
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

#include "../include/mover.hpp"

/*
 * @brief main function
 * @param argc
 * @param argv
 * @return None
 */

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "mover");
    TurtlebotMover mover;
    mover.moveRobot();
    return 0;
}

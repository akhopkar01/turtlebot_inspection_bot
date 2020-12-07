cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_inspection_robot)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  move_base_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(detector include/detector.hpp)

add_executable(mover src/main.cpp include/mover.hpp src/turtlebotMover.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(mover
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )

#############
## Testing ##
#############



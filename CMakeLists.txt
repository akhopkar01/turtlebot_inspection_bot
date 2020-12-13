cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_inspection_bot)

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
  nav_msgs
)

find_package(OpenCV 3.2.0 REQUIRED)

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

add_executable(detector src/detectorNode.cpp src/turtlebotDetector.cpp)

add_executable(mover src/moverNode.cpp src/turtlebotMover.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(mover
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )
#### To be verified
#add_dependencies(demo beginner_tutorials_generate_messages_cpp)


#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
 find_package(rostest REQUIRED)
 add_rostest_gtest(testRobot test/main.cpp test/detectorTest.cpp test/moverTest.cpp)
 target_link_libraries(testRobot ${catkin_LIBRARIES})
 add_dependencies(testRobot ${catkin_EXPORTED_TARGETS})
endif()





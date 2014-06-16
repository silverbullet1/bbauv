cmake_minimum_required(VERSION 2.8.3)
project(explorer_dvl)

find_package(catkin REQUIRED COMPONENTS
  bbauv_msgs
  nav_msgs
  nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES explorer_dvl
  CATKIN_DEPENDS bbauv_msgs nav_msgs nodelet roscpp rospy sensor_msgs std_msgs tf
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(explorer_dvl
  src/node.cpp
)

target_link_libraries(explorer_dvl
  serial
  ${catkin_LIBRARIES}
)



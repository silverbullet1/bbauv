cmake_minimum_required(VERSION 2.8.3)
project(sparton_imu)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(GCC_VERSION VERSION_EQUAL 4.7)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS
  bbauv_msgs
  nav_msgs
  nodelet
  roscpp
  rospy
  sensor_msgs
  bbauv_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sparton_imu
  CATKIN_DEPENDS bbauv_msgs nav_msgs nodelet roscpp rospy sensor_msgs std_msgs tf
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(sparton_imu src/node.cpp)

target_link_libraries(sparton_imu
    serial
    ${catkin_LIBRARIES}
)


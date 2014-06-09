cmake_minimum_required(VERSION 2.8.3)
project(sensordri)

find_package(catkin REQUIRED COMPONENTS
  bbauv_msgs
  nav_msgs
  roscpp
  std_msgs
  nodelet
  tf
)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(GCC_VERSION VERSION_EQUAL 4.7)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sensordri
  CATKIN_DEPENDS bbauv_msgs nav_msgs roscpp std_msgs nodelet tf
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(imu src/imu/node.cpp)

link_directories(
    lib    
)

target_link_libraries(imu
    serial
    ${catkin_LIBRARIES}
)


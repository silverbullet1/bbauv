cmake_minimum_required(VERSION 2.8.3)
project(cpu_mon)

find_package(catkin REQUIRED COMPONENTS
  bbauv_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cpu_mon
  CATKIN_DEPENDS bbauv_msgs roscpp
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(NOT APPLE)
add_executable(cpu_temp src/main.cpp)
endif()

if(NOT APPLE)
target_link_libraries(cpu_temp
  sensors
  ${catkin_LIBRARIES}
)
endif()

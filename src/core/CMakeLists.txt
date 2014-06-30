cmake_minimum_required(VERSION 2.8.3)
project(core)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  bbauv_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  task.msg
  plan.msg
  missionPlanStamped.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  bbauv_msgs
  std_msgs
)

catkin_package(
  LIBRARIES core
  CATKIN_DEPENDS actionlib bbauv_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf message_runtime
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

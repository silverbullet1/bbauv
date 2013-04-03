cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# set the name to use for the executable
set (BINNAME WH_DVL)

# set the source files to use with the executable
set(SRCS ${SRCS} src/serial.cpp)
set(SRCS ${SRCS} src/timing.cpp)
set(SRCS ${SRCS} src/decoder.cpp)
set(SRCS ${SRCS} src/WH_core.cpp)
set(SRCS ${SRCS} src/WH_ros.cpp)
set(SRCS ${SRCS} src/WH_main.cpp)


#set the directories where include files can be found
include_directories (include)

# Build the executable that will be used to run this node
rosbuild_add_executable(${BINNAME} ${SRCS})

# Add dynamic reconfigure API
rosbuild_find_ros_package (dynamic_reconfigure)
include (${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg ()

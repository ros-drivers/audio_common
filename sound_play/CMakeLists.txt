cmake_minimum_required(VERSION 2.4.6)

# Include the ROS cmake rules
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# Sets up various things for the pytalker_listener package.  
# The argument to the "rospack" macro must be the name of the package.

set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init()
rosbuild_genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(test)


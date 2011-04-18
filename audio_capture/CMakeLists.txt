cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_add_boost_directories()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(PkgConfig)
pkg_check_modules(GST gstreamer-0.10)

include_directories(${GST_INCLUDE_DIRS})
rosbuild_add_executable(audio_server src/audio_server.cpp)
target_link_libraries(audio_server ${GST_LIBRARIES})

include_directories(${GST_INCLUDE_DIRS})
rosbuild_add_executable(audio_client src/audio_client.cpp)
target_link_libraries(audio_client ${GST_LIBRARIES})

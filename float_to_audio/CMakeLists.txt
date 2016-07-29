cmake_minimum_required(VERSION 2.8.3)

project(float_to_audio)

find_package(catkin REQUIRED COMPONENTS roscpp audio_common_msgs)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})

catkin_package()

add_executable(float_to_audio src/float_to_audio.cpp)
target_link_libraries(float_to_audio ${catkin_LIBRARIES} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(float_to_audio ${catkin_EXPORTED_TARGETS})

install(TARGETS float_to_audio
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

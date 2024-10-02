cmake_minimum_required(VERSION 2.8.3)

project(audio_to_float)

find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp audio_common_msgs)

find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)
pkg_check_modules(GSTAPP1.0 gstreamer-app-1.0 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GST1.0_INCLUDE_DIRS}
  ${GSTAPP1.0_INCLUDE_DIRS}
)

catkin_package()

add_executable(audio_to_float src/audio_to_float.cpp)
target_link_libraries(audio_to_float
  ${catkin_LIBRARIES}
  ${GST1.0_LIBRARIES}
  ${GSTAPP1.0_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(audio_to_float ${catkin_EXPORTED_TARGETS})

install(TARGETS audio_to_float
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

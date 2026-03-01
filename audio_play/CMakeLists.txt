cmake_minimum_required(VERSION 3.5)
project(audio_play)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

add_library(${PROJECT_NAME} SHARED
  src/audio_play_node.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${audio_common_msgs_INCLUDE_DIRS}
    ${GST1.0_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${rclcpp_LIBRARIES}
    ${rclcpp_components_LIBRARIES}
    ${audio_common_msgs_LIBRARIES}
  PRIVATE
    ${GST1.0_LIBRARIES}
    ${Boost_LIBRARIES}
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_play::AudioPlayNode"
  EXECUTABLE audio_play_node
)

install(TARGETS audio_play_node ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()

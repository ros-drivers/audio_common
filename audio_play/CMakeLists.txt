cmake_minimum_required(VERSION 3.5)
project(audio_play)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

target_include_directories(${PROJECT_NAME} PRIVATE ${GST1.0_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  audio_common_msgs
)

target_link_libraries(${PROJECT_NAME} ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})

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

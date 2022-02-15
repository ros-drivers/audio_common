cmake_minimum_required(VERSION 3.5)
project(audio_capture)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(rclcpp audio_common_msgs)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${GST1.0_INCLUDE_DIRS})

add_executable(audio_capture src/audio_capture.cpp)
ament_target_dependencies(audio_capture
  rclcpp
  audio_common_msgs
)
target_link_libraries(audio_capture ${GST1.0_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS audio_capture
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(sound_play_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
  "msg/SoundRequest.msg"
)

set(action_files
  "action/SoundRequest.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES
    action_msgs
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

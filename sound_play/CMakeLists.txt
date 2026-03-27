cmake_minimum_required(VERSION 3.5)
project(sound_play)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)

include_directories(
  include
  ${GST_INCLUDE_DIRS}
)

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
    audio_common_msgs
    builtin_interfaces
)

# Build C++ soundplay node
add_executable(soundplay_node_cpp src/soundplay_node.cpp)
ament_target_dependencies(soundplay_node_cpp
  rclcpp
  rclcpp_action
  diagnostic_msgs
  ament_index_cpp
)
target_link_libraries(soundplay_node_cpp ${GST_LIBRARIES})

# Need to link against the generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(soundplay_node_cpp "${cpp_typesupport_target}")

install(TARGETS soundplay_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python package manually to avoid conflicts with rosidl_generate_interfaces
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(PROGRAMS
  scripts/is_speaking.py
  scripts/playbuiltin.py
  scripts/play.py
  scripts/say.py
  scripts/shutup.py
  scripts/soundplay_node.py
  # scripts/test.py
  # scripts/test_actionlib_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY sounds
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

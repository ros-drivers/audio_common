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

include_directories(rclcpp audio_common_msgs action_msgs builtin_interfaces)

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

ament_python_install_package(${PROJECT_NAME})

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

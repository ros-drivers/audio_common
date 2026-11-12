cmake_minimum_required(VERSION 3.5)
project(sound_play)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(sound_play_msgs REQUIRED)

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
ament_export_include_directories(include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY sounds
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_auto_add_gtest(sound_play_test
    test/sound_play_mock_test.cpp
  )
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_soundplay_node test/test_soundplay_node.py TIMEOUT 60)
endif()


ament_export_dependencies(rosidl_default_runtime sound_play_msgs)

ament_package()

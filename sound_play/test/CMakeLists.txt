add_executable(test test.cpp)
target_link_libraries(test ${catkin_LIBRARIES})
add_dependencies(test sound_play_gencpp)


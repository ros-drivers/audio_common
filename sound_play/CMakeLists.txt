cmake_minimum_required(VERSION 2.8.3)

project(sound_play)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp)
add_message_files(DIRECTORY msg FILES SoundRequest.msg)
generate_messages()

include_directories(include ${catkin_INCLUDE_DIRS})

add_subdirectory(test)


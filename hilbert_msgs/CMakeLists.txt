cmake_minimum_required(VERSION 2.8.3)
project(hilbert_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES
  MapperInfo.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs)

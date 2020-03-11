cmake_minimum_required(VERSION 3.5.0)
project(twc_msgs)

add_compile_options(
  -std=c++14
  -Wall
  -Wextra
  -Wsuggest-override
  -Wconversion
  -Wsign-conversion)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  tesseract_msgs
  actionlib_msgs
)


add_action_files(
  DIRECTORY action
  FILES
    ProcessJob.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    tesseract_msgs
    actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    tesseract_msgs
    actionlib_msgs
)

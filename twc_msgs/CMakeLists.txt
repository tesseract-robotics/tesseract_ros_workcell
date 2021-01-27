cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version})

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

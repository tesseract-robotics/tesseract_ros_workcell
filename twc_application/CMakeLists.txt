cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  twc_msgs
  tesseract_msgs
  actionlib
  tesseract_rosutils
  tesseract_monitoring
)

catkin_package(
  CATKIN_DEPENDS
    twc_msgs
    tesseract_msgs
    actionlib
    tesseract_rosutils
  DEPENDS
    tesseract_motion_planners
    tesseract_visualization
    tesseract_command_language
    tesseract_environment
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/application_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_visualization
  tesseract::tesseract_command_language
  tesseract::tesseract_environment
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


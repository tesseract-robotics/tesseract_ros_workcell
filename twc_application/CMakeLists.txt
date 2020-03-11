cmake_minimum_required(VERSION 3.5.0)
project(twc_application)

find_package(tesseract_collision REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  twc_msgs
  twc_motion_planning
  tesseract_msgs
  actionlib
  tesseract_rosutils
)

catkin_package(
#  INCLUDE_DIRS
#    include
  LIBRARIES
    sanding_process_planner
  CATKIN_DEPENDS
    twc_motion_planning
    twc_msgs
    tesseract_msgs
    actionlib
    tesseract_rosutils
  DEPENDS
    tesseract_collision
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/application_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} tesseract::tesseract_collision_core ${YAML_CPP_LIBRARIES})

install(
  TARGETS
    ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


cmake_minimum_required(VERSION 3.5.0)
project(twc_motion_planning)

find_package(descartes_light)
find_package(descartes_samplers)
find_package(opw_kinematics REQUIRED)
find_package(tesseract REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_process_planners REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  tesseract_rosutils
  tesseract_monitoring
  twc_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    sanding_process_planner
  CATKIN_DEPENDS
    eigen_conversions
    tesseract_rosutils
    tesseract_monitoring
    twc_msgs
  DEPENDS
    descartes_light
    descartes_samplers
    tesseract
    tesseract_kinematics
    tesseract_monitoring
    tesseract_motion_planners
    tesseract_process_planners
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(process_definition_generator
  src/sanding_definition_generator.cpp)
add_dependencies(process_definition_generator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(process_definition_generator PUBLIC
  tesseract::tesseract_process_planners
  tesseract::tesseract_motion_planners_core)

add_library(sanding_process_planner
  src/sanding_process_planner.cpp)
add_dependencies(sanding_process_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(sanding_process_planner PUBLIC
  ${catkin_LIBRARIES}
  process_definition_generator
  descartes::descartes_light
  descartes::descartes_samplers
  tesseract::tesseract_kinematics_opw
  tesseract::tesseract_process_planners
  tesseract::tesseract_motion_planners_core
  tesseract::tesseract_motion_planners_ompl
  tesseract::tesseract_motion_planners_trajopt
  tesseract::tesseract_motion_planners_descartes)

add_library(planning_manager
  src/planning_manager.cpp)
add_dependencies(planning_manager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_manager PUBLIC
  ${catkin_LIBRARIES}
  sanding_process_planner)

add_executable(planning_manager_node
  src/planning_manager_node.cpp)
add_dependencies(planning_manager_node
  ${planning_manager_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_manager_node
  planning_manager
  ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
    process_definition_generator
    sanding_process_planner
    planning_manager
    planning_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


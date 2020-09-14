cmake_minimum_required(VERSION 3.5.0)
project(twc_motion_planning)

find_package(catkin REQUIRED COMPONENTS
  tesseract_monitoring
  tesseract_msgs
  tesseract_planning_server
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(cmake_common_scripts REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_collision REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_planning_server
  CATKIN_DEPENDS
    tesseract_msgs
    tesseract_monitoring
    tesseract_planning_server
  DEPENDS
    tesseract_command_language
    tesseract_process_managers
    tesseract_planning_server
)

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()

add_library(${PROJECT_NAME}_planning_server SHARED src/twc_planning_server.cpp)
target_link_libraries(${PROJECT_NAME}_planning_server PUBLIC tesseract::tesseract_collision_core tesseract::tesseract_command_language tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_planning_server PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_planning_server ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_planning_server PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_planning_server PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_planning_server SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_planning_server_node src/twc_planning_server_node.cpp)
target_link_libraries(${PROJECT_NAME}_planning_server_node PRIVATE ${PROJECT_NAME}_planning_server ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_planning_server_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_planning_server_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_planning_server_node PRIVATE VERSION 17)
target_include_directories(${PROJECT_NAME}_planning_server_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_planning_server_node SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  TARGETS ${PROJECT_NAME}_planning_server ${PROJECT_NAME}_planning_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


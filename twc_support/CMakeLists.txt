cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version})

find_package(catkin REQUIRED)

catkin_package()

install(
  DIRECTORY config launch meshes urdf CAD
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

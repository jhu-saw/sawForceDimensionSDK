cmake_minimum_required(VERSION 3.10)
project(saw_force_dimension_config VERSION 2.1.0)

find_package (catkin QUIET)
if (catkin_FOUND)
  catkin_package()
endif ()

install (DIRECTORY
  "${saw_force_dimension_config_SOURCE_DIR}"
  DESTINATION share/sawForceDimensionSDK
  COMPONENT sawForceDimensionSDK-Share
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "package.xml" EXCLUDE)

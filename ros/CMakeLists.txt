#
# (C) Copyright 2017-2024 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (force_dimension VERSION 2.1.0)

# first test for ROS1
find_package (catkin QUIET
  COMPONENTS
  cisst_ros_crtk
  roscpp
  roslib)

# ROS 1
if (catkin_FOUND)
  set (ROS_VERSION 1)
  catkin_package (CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)
  include_directories (${catkin_INCLUDE_DIRS})
  # for python CRTK client
  catkin_python_setup ()

else (catkin_FOUND)

  # look for ROS2
  find_package (ament_cmake QUIET)
  if (ament_cmake_FOUND)
    find_package (ament_cmake_python REQUIRED)
    set (ROS_VERSION 2)
    set (ROS2_depends
      rclcpp std_msgs std_srvs geometry_msgs sensor_msgs diagnostic_msgs
      tf2_ros tf2_msgs cisst_msgs crtk_msgs cisst_ros_bridge cisst_ros_crtk)
    foreach (_dep ${ROS2_depends})
      find_package (${_dep} REQUIRED)
    endforeach ()

  endif (ament_cmake_FOUND)
endif (catkin_FOUND)

# find cisst and make sure the required libraries have been compiled
set (REQUIRED_CISST_LIBRARIES
  cisstCommon
  cisstCommonQt
  cisstVector
  cisstVectorQt
  cisstOSAbstraction
  cisstMultiTask
  cisstMultiTaskQt
  cisstParameterTypes
  cisstQt)

find_package (cisst 1.2.1 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # set paths
  cisst_set_output_path ()

  # sawForceDimensionSDK has been compiled within cisst, we should find it automatically
  find_package (sawForceDimensionSDK 2.1.0)

  if (sawForceDimensionSDK_FOUND)

    # sawForceDimensionSDK configuration
    include_directories (${sawForceDimensionSDK_INCLUDE_DIR} ${force_dimension_SOURCE_DIR})
    link_directories (${sawForceDimensionSDK_LIBRARY_DIR})

    add_executable (force_dimension src/force_dimension.cpp)
    target_link_libraries (force_dimension
      ${sawForceDimensionSDK_LIBRARIES}
      ${catkin_LIBRARIES})

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (force_dimension ${REQUIRED_CISST_LIBRARIES})

    install (TARGETS force_dimension
      LIBRARY DESTINATION lib/${PROJECT_NAME}
      RUNTIME DESTINATION lib/${PROJECT_NAME}
      LIBRARY DESTINATION lib
      INCLUDES DESTINATION include)

    if (ROS_VERSION EQUAL 2)
      # Python
      add_subdirectory (python)
      install (PROGRAMS scripts/xyz-motions.py
        DESTINATION lib/${PROJECT_NAME})
      # C++
      ament_target_dependencies (
        force_dimension
        ${ROS2_depends}
        )
      ament_package ()
    endif ()

  else (sawForceDimensionSDK_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawForceDimensionSDK")
  endif (sawForceDimensionSDK_FOUND)
endif (cisst_FOUND_AS_REQUIRED)

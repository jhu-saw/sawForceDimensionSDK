#
# (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8.3)

project (force_dimension_ros_dvrk)

## find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              dvrk_robot
              geometry_msgs
              roscpp
              std_msgs
              roslib
              )

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

find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  include (${CISST_USE_FILE})

  # include_directories(include)
  include_directories (${catkin_INCLUDE_DIRS})

  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

  catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
                  CATKIN_DEPENDS saw_force_dimension_sdk geometry_msgs roscpp std_msgs dvrk_robot)

  # sawForceDimensionSDK has been compiled within cisst, we should find it automatically
  find_package (sawForceDimensionSDK    1.0.0 REQUIRED)
  find_package (sawRobotIO1394          1.4.0 REQUIRED)
  find_package (sawControllers          1.4.0 REQUIRED)
  find_package (sawConstraintController 1.1.0 REQUIRED)
  find_package (sawIntuitiveResearchKit 1.4.0 REQUIRED)

  include_directories (
    ${dvrk_robot_SOURCE_DIR}/include
    ${CATKIN_DEVEL_PREFIX}/include
    ${sawForceDimensionSDK_INCLUDE_DIR}
    ${sawRobotIO1394_INCLUDE_DIR}
    ${sawControllers_INCLUDE_DIR}
    ${sawConstraintController_INCLUDE_DIR}
    ${sawIntuitiveResearchKit_INCLUDE_DIR}
    )

  link_directories (
    ${sawForceDimensionSDK_LIBRARY_DIR}
    ${sawRobotIO1394_LIBRARY_DIR}
    ${sawControllers_LIBRARY_DIR}
    ${sawConstraintController_LIBRARY_DIR}
    ${sawIntuitiveResearchKit_LIBRARY_DIR}
    )

  add_executable (force_dimension_dvrk src/force_dimension_dvrk.cpp)
  target_link_libraries (
    force_dimension_dvrk
    ${sawForceDimensionSDK_LIBRARIES}
    ${sawIntuitiveResearchKit_LIBRARIES}
    ${sawRobotIO1394_LIBRARIES}
    ${sawControllers_LIBRARIES}
    ${sawConstraintController_LIBRARIES}
    ${catkin_LIBRARIES})

  cisst_target_link_libraries (force_dimension_dvrk ${REQUIRED_CISST_LIBRARIES})

endif (cisst_FOUND_AS_REQUIRED)

#
# (C) Copyright 2017-2023 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (force_dimension_ros VERSION 2.0.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              cisst_ros_crtk
              roscpp
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

find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  include (${CISST_USE_FILE})

  # include_directories(include)
  include_directories (${catkin_INCLUDE_DIRS})

  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

  catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
                  CATKIN_DEPENDS cisst_ros_crtk roscpp roslib)

  # sawForceDimensionSDK has been compiled within cisst, we should find it automatically
  find_package (sawForceDimensionSDK 2.0.0)

  if (sawForceDimensionSDK_FOUND)

    include_directories (${CATKIN_DEVEL_PREFIX}/include
                         ${sawForceDimensionSDK_INCLUDE_DIR})

    link_directories (${sawForceDimensionSDK_LIBRARY_DIR})

    add_executable (force_dimension src/force_dimension.cpp)
    target_link_libraries (force_dimension
                           ${sawForceDimensionSDK_LIBRARIES}
                           ${catkin_LIBRARIES})
    cisst_target_link_libraries (force_dimension ${REQUIRED_CISST_LIBRARIES})

    # for python CRTK client
    catkin_python_setup ()

  else (sawForceDimensionSDK_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawForceDimensionSDK")
  endif (sawForceDimensionSDK_FOUND)

endif (cisst_FOUND_AS_REQUIRED)

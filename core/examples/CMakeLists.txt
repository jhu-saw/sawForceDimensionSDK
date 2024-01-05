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
project (sawForceDimensionSDKExamples VERSION 2.1.0)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonQt
                              cisstVector
                              cisstVectorQt
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstMultiTaskQt
                              cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.2.0 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # sawForceDimensionSDK has been compiled within cisst, we should find it automatically
  find_package (sawForceDimensionSDK 2.0.0)

  if (sawForceDimensionSDK_FOUND AND CISST_HAS_QT)

    # sawForceDimensionSDK configuration
    include_directories (${sawForceDimensionSDK_INCLUDE_DIR})
    link_directories (${sawForceDimensionSDK_LIBRARY_DIR})

    add_executable (sawForceDimensionSDKQtExample main.cpp)
    set_target_properties (sawForceDimensionSDKQtExample PROPERTIES
                           COMPONENTS sawForceDimensionSDK-Examples
                           FOLDER "sawForceDimensionSDK")

    # link against non cisst libraries and saw components
    target_link_libraries (sawForceDimensionSDKQtExample
                           ${sawForceDimensionSDK_LIBRARIES}
                           ${sawForceDimensionSDKQt_LIBRARIES}
                           ${QT_LIBRARIES}
                           )

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (sawForceDimensionSDKQtExample ${REQUIRED_CISST_LIBRARIES})

    install (TARGETS sawForceDimensionSDKQtExample COMPONENT sawForceDimensionSDK-Examples
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

  else (sawForceDimensionSDK_FOUND AND CISST_HAS_QT)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawForceDimensionSDK and cisstQt")
  endif (sawForceDimensionSDK_FOUND AND CISST_HAS_QT)

endif (cisst_FOUND_AS_REQUIRED)

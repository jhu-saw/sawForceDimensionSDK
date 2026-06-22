#
# (C) Copyright 2014-2026 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt
)

# cisst
find_package (cisst REQUIRED COMPONENTS ${REQUIRED_CISST_LIBRARIES})

# for Qt generated header files
include_directories (${CMAKE_CURRENT_BINARY_DIR})
set (CMAKE_AUTOMOC ON)

add_library (sawForceDimensionSDKQt
             ${sawForceDimensionSDK_HEADER_DIR}/mtsForceDimensionQtWidget.h
             mtsForceDimensionQtWidget.cpp)
set_target_properties (sawForceDimensionSDKQt PROPERTIES
                       VERSION ${sawForceDimensionSDK_VERSION}
                       FOLDER "sawForceDimensionSDK")
target_link_libraries (sawForceDimensionSDKQt
                       ${cisst_LIBRARIES})

install (TARGETS sawForceDimensionSDKQt COMPONENT sawForceDimensionSDK-Qt
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib)

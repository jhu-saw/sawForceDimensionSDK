#
# (C) Copyright 2016-2018 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (sawForceDimensionSDK)

cmake_minimum_required (VERSION 2.8)

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.0.10 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  if (NOT CISST_HAS_JSON)
    message( FATAL_ERROR "sawForceDimensionSDK requires cisst to be built with JSON" )
  endif ()

  # catkin/ROS paths
  cisst_is_catkin_build (sawForceDimensionSDK_IS_CATKIN_BUILT)
  if (sawForceDimensionSDK_IS_CATKIN_BUILT)
    set (sawForceDimensionSDK_CONFIG_FILE_DIR
         "${CATKIN_DEVEL_PREFIX}/share/sawForceDimensionSDK/cmake")
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
    set (LIBRARY_OUTPUT_PATH    "${CATKIN_DEVEL_PREFIX}/lib")
  else ()
    set (sawForceDimensionSDK_CONFIG_FILE_DIR "${sawForceDimensionSDK_BINARY_DIR}")
  endif ()

  # this should be replaced by a find ForceDimensionSDK
  set (force_dimension_sdk_FOUND 0)
  if (WIN32)
    message ("this needs to be implemented for Windows")
    # Windows
    # set (force_dimension_sdk_DIR "C:/Program Files/Force_dimension_sdk/Passive Tracking SDK")
    # include_directories (${force_dimension_sdk_DIR}/include
    #                      ${force_dimension_sdk_DIR}/bin)
    # link_directories (${force_dimension_sdk_DIR}/bin)
    # set (force_dimension_sdk_LIBRARIES winmm fusionTrack64.lib)
    # set (force_dimension_sdk_FOUND 1)
    # Linux
  elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_path (force_dimension_sdk_DIR
               NAMES "dhdc.h"
               PATH_SUFFIXES "include")
    if (EXISTS "${force_dimension_sdk_DIR}/include/dhdc.h")
      set (force_dimension_sdk_FOUND 1)
      include_directories (${force_dimension_sdk_DIR}/include)
      # libraries
      link_directories (${force_dimension_sdk_DIR}/lib/release/lin-x86_64)
      set (force_dimension_sdk_LIBRARIES dhd drd usb-1.0)
      # copy the binary provided for fusionTrack in cisst/saw library directory
      find_library (force_dimension_sdk_LIBRARY_DHD
                    NAMES dhd
                    PATHS ${force_dimension_sdk_DIR}/lib/release/lin-x86_64-gcc)
      find_library (force_dimension_sdk_LIBRARY_DRD
                    NAMES drd
                    PATHS ${force_dimension_sdk_DIR}/lib/release/lin-x86_64-gcc)
      file (MAKE_DIRECTORY ${LIBRARY_OUTPUT_PATH})
      file (COPY ${force_dimension_sdk_LIBRARY_DHD} ${force_dimension_sdk_LIBRARY_DRD}
            DESTINATION ${LIBRARY_OUTPUT_PATH})
    endif ()
  else (WIN32)
    # Everything else
    message ("this needs to be implemented for Mac")
  endif (WIN32)

  # if we found the actracsys SDK
  if (force_dimension_sdk_FOUND)
    # create/configure file for find_package (sawForceDimensionSDK)
    set (sawForceDimensionSDK_INCLUDE_DIR
         "${sawForceDimensionSDK_SOURCE_DIR}/include"
         "${sawForceDimensionSDK_BINARY_DIR}/include")
    set (sawForceDimensionSDK_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}" "${force_dimension_sdk_DIR}/lib")
    set (sawForceDimensionSDK_LIBRARIES sawForceDimensionSDK ${force_dimension_sdk_LIBRARIES})

    # Set the version number
    set (sawForceDimensionSDK_VERSION_MAJOR "1")
    set (sawForceDimensionSDK_VERSION_MINOR "1")
    set (sawForceDimensionSDK_VERSION_PATCH "0")
    set (sawForceDimensionSDK_VERSION "${sawForceDimensionSDK_VERSION_MAJOR}.${sawForceDimensionSDK_VERSION_MINOR}.${sawForceDimensionSDK_VERSION_PATCH}")

    # Generate sawForceDimensionSDKRevision.h
    configure_file ("${sawForceDimensionSDK_SOURCE_DIR}/code/sawForceDimensionSDKRevision.h.in"
                    "${sawForceDimensionSDK_BINARY_DIR}/include/sawForceDimensionSDK/sawForceDimensionSDKRevision.h")

    include_directories (${sawForceDimensionSDK_INCLUDE_DIR})
    set (sawForceDimensionSDK_HEADER_DIR "${sawForceDimensionSDK_SOURCE_DIR}/include/sawForceDimensionSDK")

    add_library (sawForceDimensionSDK
                 ${sawForceDimensionSDK_HEADER_DIR}/sawForceDimensionSDKExport.h
                 code/mtsForceDimension.cpp
                 ${sawForceDimensionSDK_HEADER_DIR}/mtsForceDimension.h)
    set_property (TARGET sawForceDimensionSDK PROPERTY FOLDER "sawForceDimensionSDK")

    target_link_libraries (sawForceDimensionSDK
                           ${force_dimension_sdk_LIBRARY_DHD}
                           ${force_dimension_sdk_LIBRARY_DRD}
                           usb-1.0)
    cisst_target_link_libraries (sawForceDimensionSDK ${REQUIRED_CISST_LIBRARIES})

    # add Qt code
    if (CISST_HAS_QT)
      add_subdirectory (code/Qt)
      set (sawForceDimensionSDK_LIBRARIES ${sawForceDimensionSDK_LIBRARIES} ${sawForceDimensionSDKQt_LIBRARIES})
    endif (CISST_HAS_QT)

    set (sawForceDimensionSDK_CMAKE_CONFIG_FILE
         "${sawForceDimensionSDK_CONFIG_FILE_DIR}/sawForceDimensionSDKConfig.cmake")

    configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawForceDimensionSDKConfig.cmake.in"
                    "${sawForceDimensionSDK_CMAKE_CONFIG_FILE}"
                    @ONLY)

    cisst_add_config_version ("${sawForceDimensionSDK_CONFIG_FILE_DIR}/sawForceDimensionSDKConfigVersion.cmake"
                              VERSION ${sawForceDimensionSDK_VERSION}
                              DESTINATION ${CISST_CMAKE_INSTALL_SUFFIX}/saw
                              COMPONENT sawForceDimensionSDK)

    # Install target for headers and library
    install (DIRECTORY
             "${sawForceDimensionSDK_SOURCE_DIR}/include/sawForceDimensionSDK"
             DESTINATION include
             PATTERN .svn EXCLUDE)

    install (TARGETS sawForceDimensionSDK
             RUNTIME DESTINATION bin
             LIBRARY DESTINATION lib
             ARCHIVE DESTINATION lib)

    install (FILES "${sawForceDimensionSDK_BINARY_DIR}/sawForceDimensionSDKConfig.cmake"
             DESTINATION "${CISST_CMAKE_INSTALL_SUFFIX}/saw")

  else (force_dimension_sdk_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires the Force Dimension SDK.  Please set force_dimension_sdk_DIR using CMake, e.g. ccmake ${CMAKE_CURRENT_BINARY_DIR} or cmake-gui ${CMAKE_CURRENT_BINARY_DIR}")
  endif (force_dimension_sdk_FOUND)
else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)

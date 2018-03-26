##############################################################################
#                                                                            #
# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

#
# author: Nicola Piga <nicolapiga@gmail.com>
#

# require minimum cmake version
cmake_minimum_required(VERSION 2.8.9)

# produce the cmake var PROJECT_NAME
project(visual-tactile-localization-sim)

# mandatory use of ICUBcontrib
find_package(YARP REQUIRED)
find_package(ICUB)
find_package(ICUBcontrib REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers
include(YarpInstallationHelpers)
include(ICUBcontribHelpers)
#include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()

# set headers and sources
set(headers
  ${CMAKE_SOURCE_DIR}/headers/PointCloud.h
  ${CMAKE_SOURCE_DIR}/headers/filterCommand.h
  ${CMAKE_SOURCE_DIR}/headers/ArmController.h
  ${CMAKE_SOURCE_DIR}/headers/FingerController.h
  ${CMAKE_SOURCE_DIR}/headers/HandController.h
  ${CMAKE_SOURCE_DIR}/headers/ModelHelper.h
  )
set(sources
  ${CMAKE_SOURCE_DIR}/src/filterCommand.cpp
  ${CMAKE_SOURCE_DIR}/src/ArmController.cpp
  ${CMAKE_SOURCE_DIR}/src/FingerController.cpp  
  ${CMAKE_SOURCE_DIR}/src/HandController.cpp
  ${CMAKE_SOURCE_DIR}/src/ModelHelper.cpp
  ${CMAKE_SOURCE_DIR}/src/main.cpp)

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
add_executable(${PROJECT_NAME} ${headers} ${sources})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# add uninstall target
icubcontrib_add_uninstall_target()

# install application xml
file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})


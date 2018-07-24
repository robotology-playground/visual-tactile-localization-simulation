##############################################################################
#                                                                            #
# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

#
# author: Nicola Piga
#

set(EXE_TARGET_NAME vtl-demo)

# Dependencies
find_package(YARP REQUIRED COMPONENTS gsl)
find_package(ICUB REQUIRED)
find_package(IKIN_FWD_EXT REQUIRED)
find_package(HAND_CTRL_CMD REQUIRED)
find_package(GAZE_CTRL REQUIRED)

list(APPEND CMAKE_MODULE_PATH
  # ${YARP_MODULE_PATH}
  ${ICUB_MODULE_PATH})

# include(YarpInstallationHelpers)

# Source and header files
set(${EXE_TARGET_NAME}_HDR
 include/ArmController.h
 include/FilterCommand.h
 include/ModelHelper.h
 include/RotationTrajectoryGenerator.h
 include/TrajectoryGenerator.h
 ${IKIN_FWD_EXT_HDR}
 ${HAND_CTRL_CMD_HDR})

set(${EXE_TARGET_NAME}_SRC
 src/ArmController.cpp
 src/FilterCommand.cpp
 src/main.cpp
 src/ModelHelper.cpp
 src/RotationTrajectoryGenerator.cpp
 src/TrajectoryGenerator.cpp
 ${IKIN_FWD_EXT_SRC}
 ${HAND_CTRL_CMD_SRC})

# idl
set(${EXE_TARGET_NAME}_THRIFT_NAME
  visual_tactile_loc)
  
set(${EXE_TARGET_NAME}_THRIFT_HDR
  thrift/${${EXE_TARGET_NAME}_THRIFT_NAME}.thrift)

set(${EXE_TARGET_NAME}_THRIFT_MAKEFILE
  ${CMAKE_CURRENT_SOURCE_DIR}/thrift_${${EXE_TARGET_NAME}_THRIFT_NAME}_thrift.cmake)

set(${EXE_TARGET_NAME}_THRIFT_INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/thrift)

yarp_idl_to_dir(${${EXE_TARGET_NAME}_THRIFT_HDR} ${CMAKE_CURRENT_SOURCE_DIR})
include(${${EXE_TARGET_NAME}_THRIFT_MAKEFILE})
# the makefile included above set variables 'headers' and 'sources'
set(${EXE_TARGET_NAME}_THRIFT_SRC ${sources})

# Application target calls
add_executable(${EXE_TARGET_NAME}
  ${${EXE_TARGET_NAME}_HDR}
  ${${EXE_TARGET_NAME}_SRC}
  ${${EXE_TARGET_NAME}_THRIFT_SRC}
  ${GAZE_CTRL_SRC}
  )

target_include_directories(${EXE_TARGET_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${IKIN_FWD_EXT_INCLUDE}
  ${HAND_CTRL_CMD_INCLUDE}
  # ${YARP_INCLUDE_DIRS}
  ${ICUB_INCLUDE_DIRS}
  ${${EXE_TARGET_NAME}_THRIFT_INCLUDE}
  ${GAZE_CTRL_INCLUDE}
  )

target_link_libraries(${EXE_TARGET_NAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})

# Install calls
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

set(${EXE_TARGET_NAME}_CONF_SIM
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/simulation/demo_config.ini)
set(${EXE_TARGET_NAME}_CONF_SETUP
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/setup/demo_config.ini)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_SIM}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SIM})
yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_SETUP}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SETUP})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts}
  DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})

##############################################################################
#                                                                            #
# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

#
# author: Nicola Piga
#

set(EXE_TARGET_NAME vtl-hand-ctrl)

# Dependencies
find_package(YARP REQUIRED COMPONENTS gsl)
find_package(ICUB REQUIRED)
find_package(IKIN_FWD_EXT REQUIRED)

list(APPEND CMAKE_MODULE_PATH
  # ${YARP_MODULE_PATH}
  ${ICUB_MODULE_PATH})

# include(YarpInstallationHelpers)

# Source and header files
set(${EXE_TARGET_NAME}_HDR
  include/FingerController.h
  include/HandControlCommand.h
  include/HandController.h
  include/HandControlModule.h
  include/HandControlResponse.h
  ${IKIN_FWD_EXT_HDR})

set(${EXE_TARGET_NAME}_SRC
  src/FingerController.cpp
  src/HandControlCommand.cpp
  src/HandController.cpp
  src/HandControlModule.cpp
  src/HandControlResponse.cpp
  ${IKIN_FWD_EXT_SRC})

# Application target calls
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_SRC})

target_include_directories(${EXE_TARGET_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${IKIN_FWD_EXT_INCLUDE}
  # ${YARP_INCLUDE_DIRS}
  ${ICUB_INCLUDE_DIRS})

target_link_libraries(${EXE_TARGET_NAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})

# Configuration files
set(${EXE_TARGET_NAME}_CONF_SIM
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/simulation/hand-control_config.ini)
set(${EXE_TARGET_NAME}_CONF_SETUP
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/setup/hand-control_config.ini)

# Install calls
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_SIM}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SIM})

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_SETUP}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SETUP})

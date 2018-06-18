##############################################################################
#                                                                            #
# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

#
# author: Nicola Piga
#

set(EXE_TARGET_NAME vtl-gtruth-tracker)

# Dependencies
find_package(YARP REQUIRED)
find_package(ICUB)
find_package(OpenCV REQUIRED)
find_package(HEAD_KIN REQUIRED)

list(APPEND CMAKE_MODULE_PATH
  ${YARP_MODULE_PATH}
  ${ICUB_MODULE_PATH})

include(YarpInstallationHelpers)

# Source and header files
set(${EXE_TARGET_NAME}_HDR
  include/Tracker.h
  include/GazeController.h
  include/UcoBoardEstimator.h
  include/ArucoBoardEstimator.h
  include/CharucoBoardEstimator.h
  )

set(${EXE_TARGET_NAME}_SRC
  src/Tracker.cpp
  src/GazeController.cpp
  src/UcoBoardEstimator.cpp
  src/ArucoBoardEstimator.cpp
  src/CharucoBoardEstimator.cpp
  src/main.cpp
  ${HEAD_KIN_SRC}
  )

# Application target calls
add_executable(${EXE_TARGET_NAME}
  ${${EXE_TARGET_NAME}_HDR}
  ${${EXE_TARGET_NAME}_SRC}
  ${HEAD_KIN_SRC}
  )

target_include_directories(${EXE_TARGET_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include  
  ${YARP_INCLUDE_DIRS}
  ${ICUB_INCLUDE_DIRS}
  ${HEAD_KIN_INCLUDE}  
  )

target_link_libraries(${EXE_TARGET_NAME}
  ${YARP_LIBRARIES}
  ${ICUB_LIBRARIES}
  ${OpenCV_LIBS}  
  )

# Install calls
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

set(${EXE_TARGET_NAME}_CONF_SIM
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/simulation/gt_tracker_config.ini)
set(${EXE_TARGET_NAME}_CONF_SETUP
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/setup/gt_tracker_config.ini)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_SIM}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SIM})

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_SETUP}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SETUP})

set(${EXE_TARGET_NAME}_CAM_CALIB_DROID
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/calibration/cam_calib_droid)
set(${EXE_TARGET_NAME}_CAM_CALIB_ICUB
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/calibration/cam_calib_icub)
set(${EXE_TARGET_NAME}_CAM_CALIB_ICUB_640
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/calibration/cam_calib_icub_640_480)

yarp_install(FILES ${${EXE_TARGET_NAME}_CAM_CALIB_DROID}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SETUP}/calibration)
yarp_install(FILES ${${EXE_TARGET_NAME}_CAM_CALIB_ICUB}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SETUP}/calibration)
yarp_install(FILES ${${EXE_TARGET_NAME}_CAM_CALIB_ICUB_640}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SETUP}/calibration)

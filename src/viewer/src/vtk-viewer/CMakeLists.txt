##############################################################################
#                                                                            #
# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

#
# author: Nicola Piga
#

set(EXE_TARGET_NAME vtl-vtk-viewer)

# Dependencies
find_package(YARP REQUIRED COMPONENTS gsl)
find_package(ICUB REQUIRED)
find_package(VTK REQUIRED)
find_package(IKIN_FWD_EXT REQUIRED)

list(APPEND CMAKE_MODULE_PATH
  # ${YARP_MODULE_PATH}
  ${ICUB_MODULE_PATH})

# include(YarpInstallationHelpers)
include(${VTK_USE_FILE})

set(${EXE_TARGET_NAME}_HDR
  include/vtk3DAxes.h
  include/vtkFinger.h
  include/vtkHand.h
  include/handKinematics.h
 ${IKIN_FWD_EXT_HDR})

set(${EXE_TARGET_NAME}_SRC
  src/VtkViewer.cpp
  src/vtk3DAxes.cpp
  src/vtkFinger.cpp
  src/vtkHand.cpp
  src/handKinematics.cpp
 ${IKIN_FWD_EXT_SRC})

# Application target calls
add_executable(${EXE_TARGET_NAME}
  ${${EXE_TARGET_NAME}_HDR}
  ${${EXE_TARGET_NAME}_SRC})

target_include_directories(${EXE_TARGET_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  # ${YARP_INCLUDE_DIRS}
  ${ICUB_INCLUDE_DIRS}
  ${IKIN_FWD_EXT_INCLUDE})

target_link_libraries(${EXE_TARGET_NAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
if(VTK_LIBRARIES)
  target_link_libraries(${EXE_TARGET_NAME} ${VTK_LIBRARIES})
else()
  target_link_libraries(${EXE_TARGET_NAME} vtkHybrid vtkWidgets)
endif()

# Install calls
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

set(${EXE_TARGET_NAME}_CONF_SETUP
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/setup/vtk_viewer_config.ini)

set(${EXE_TARGET_NAME}_CONF_SIM
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/simulation/vtk_viewer_config.ini)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_SETUP}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SETUP})

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_SIM}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SIM})

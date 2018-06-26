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
find_package(YARP REQUIRED)
find_package(VTK REQUIRED)

list(APPEND CMAKE_MODULE_PATH
  ${YARP_MODULE_PATH}
  ${ICUB_MODULE_PATH})

include(YarpInstallationHelpers)
include(${VTK_USE_FILE})

# Application target calls
add_executable(${EXE_TARGET_NAME} src/VtkViewer.cpp)

target_include_directories(${EXE_TARGET_NAME} PRIVATE
  ${YARP_INCLUDE_DIRS}
  ${ICUB_INCLUDE_DIRS}
  )

target_link_libraries(${EXE_TARGET_NAME} ${YARP_LIBRARIES})
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

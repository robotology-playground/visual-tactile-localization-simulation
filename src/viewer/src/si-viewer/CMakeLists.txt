##############################################################################
#                                                                            #
# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

#
# author: Nicola Piga
#

set(EXE_TARGET_NAME vtl-si-viewer)

# Dependencies
find_package(YARP REQUIRED)
find_package(ICUB)
find_package(SuperimposeMesh REQUIRED)
find_package(HEAD_KIN REQUIRED)

list(APPEND CMAKE_MODULE_PATH
  ${YARP_MODULE_PATH}
  ${ICUB_MODULE_PATH})

include(YarpInstallationHelpers)

# Application target calls
add_executable(${EXE_TARGET_NAME}
  src/SuperimposeCameraViewer.cpp
  ${HEAD_KIN_SRC}
  )

target_include_directories(${EXE_TARGET_NAME} PRIVATE
  ${YARP_INCLUDE_DIRS}
  ${ICUB_INCLUDE_DIRS}
  ${HEAD_KIN_INCLUDE}  
  )

target_link_libraries(${EXE_TARGET_NAME}
  ${YARP_LIBRARIES}
  ${ICUB_LIBRARIES}
  SuperimposeMesh::SuperimposeMesh)

# Install calls
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

set(${EXE_TARGET_NAME}_CONF_SIM
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/simulation/si_viewer_config.ini)
set(${EXE_TARGET_NAME}_CONF_SETUP
  ${CMAKE_CURRENT_SOURCE_DIR}/conf/setup/si_viewer_config.ini)

yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_SIM}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SIM})
yarp_install(FILES ${${EXE_TARGET_NAME}_CONF_SETUP}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SETUP})

# Shader
set(SHADER_RESOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_background.vert
  ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model.vert
  ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_frame.vert
  ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_background.frag
  ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_model.frag
  ${CMAKE_CURRENT_SOURCE_DIR}/shader/shader_frame.frag
  )

yarp_install(FILES ${SHADER_RESOURCES}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SIM}/shader)
yarp_install(FILES ${SHADER_RESOURCES}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SETUP}/shader)

# Mesh
set(MESH_RESOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/mesh/box.obj
  ${CMAKE_CURRENT_SOURCE_DIR}/mesh/rice.obj
  )

yarp_install(FILES ${MESH_RESOURCES}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SIM}/mesh)
yarp_install(FILES ${MESH_RESOURCES}
  DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${VISUAL_TACTILE_CONTEXT_SETUP}/mesh)


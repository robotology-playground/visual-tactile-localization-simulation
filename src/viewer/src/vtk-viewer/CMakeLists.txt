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

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

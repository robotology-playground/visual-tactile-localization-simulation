##############################################################################
#                                                                            #
# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

#
# author: Nicola Piga
#

set(EXE_TARGET_NAME vtl-filter-point-cloud)

# Dependencies
find_package(YARP REQUIRED)
find_package(VTK REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(${VTK_USE_FILE})

# Source and header files
set(${EXE_TARGET_NAME}_SRC
 src/FilterPointCloud.cpp)

# Application target calls
add_executable(${EXE_TARGET_NAME}
  ${${EXE_TARGET_NAME}_SRC})

target_include_directories(${EXE_TARGET_NAME} PRIVATE
  ${YARP_INCLUDE_DIRS})

target_link_libraries(${EXE_TARGET_NAME} ${YARP_LIBRARIES})
if(VTK_LIBRARIES)
  target_link_libraries(${EXE_TARGET_NAME} ${VTK_LIBRARIES})
else()
  target_link_libraries(${EXE_TARGET_NAME} vtkHybrid vtkWidgets)
endif()

# Install calls
install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)


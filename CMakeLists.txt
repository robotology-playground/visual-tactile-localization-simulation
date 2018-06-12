##############################################################################
#                                                                            #
# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

#
# author: Nicola Piga
#

cmake_minimum_required(VERSION 3.0)

project(visual-tactile-localization
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

set(VISUAL_TACTILE_CONTEXT_SIM simVisualTactileLocalization)
set(VISUAL_TACTILE_CONTEXT_SETUP visualTactileLocalization)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# iCub-contrib project
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()
icubcontrib_add_uninstall_target()

add_subdirectory(src)

##############################################################################
#                                                                            #
# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        #
# All Rights Reserved.                                                       #
#                                                                            #
##############################################################################

#
# author: Nicola Piga
#

set(HAND_CTRL_CMD ${PROJECT_SOURCE_DIR}/src/hand-control)
set(HAND_CTRL_CMD_SRC
  ${HAND_CTRL_CMD}/src/HandControlCommand.cpp
  ${HAND_CTRL_CMD}/src/HandControlResponse.cpp)
set(HAND_CTRL_CMD_INCLUDE ${HAND_CTRL_CMD}/include)
set(HAND_CTRL_CMD_HDR
  ${HAND_CTRL_CMD_INCLUDE}/HandControlCommand.h
  ${HAND_CTRL_CMD_INCLUDE}/HandControlResponse.h)

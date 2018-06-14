/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

// yarp
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>

#ifndef GAZE_CONTROLLER_H
#define GAZE_CONTROLLER_H

class GazeController
{
    // driver
    yarp::dev::PolyDriver drv_gaze;

    // view
    yarp::dev::IGazeControl *igaze;

    // cartesian controller initial context
    int startup_cart_context;

public:
    bool configure();
    bool close();
};

#endif

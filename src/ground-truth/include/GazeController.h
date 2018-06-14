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
#include <yarp/os/ResourceFinder.h>
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

    // default trajectory time
    int eyes_traj_time;
    int neck_traj_time;

public:
    bool configure(const yarp::os::ResourceFinder &rf);
    bool close();
    bool setReference(const yarp::sig::Vector &fixation_point);
    bool getCameraPose(const std::string &eye_name,
                       yarp::sig::Vector &pos,
                       yarp::sig::Vector &att);
};

#endif

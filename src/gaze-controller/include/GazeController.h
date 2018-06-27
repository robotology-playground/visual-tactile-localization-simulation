/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

#ifndef GAZE_CONTROLLER_H
#define GAZE_CONTROLLER_H

// yarp
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>

class GazeController
{
    // driver
    yarp::dev::PolyDriver drv_gaze;

    // view
    yarp::dev::IGazeControl *igaze;

    // cartesian controller initial context
    int startup_cart_context;

    // default trajectory time
    double eyes_traj_time;
    double neck_traj_time;

public:
    bool configure(const yarp::os::ResourceFinder &rf,
                   const std::string &port_prefix);
    bool close();
    bool setReference(const yarp::sig::Vector &fixation_point);
    bool getCameraPose(const std::string &eye_name,
                       yarp::sig::Vector &pos,
                       yarp::sig::Vector &att);
    bool setTrajectoryTimes();
};

#endif

/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

//yarp
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>

#include <Tracker.h>

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    yarp::os::ResourceFinder rf;
    rf.setDefaultConfigFile("gt_tracker_config.ini");
    rf.configure(argc,argv);

    if (!yarp.checkNetwork())
    {
        yError() << "Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    Tracker tracker;
    return tracker.runModule(rf);
}

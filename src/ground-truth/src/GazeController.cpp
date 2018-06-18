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
#include <yarp/os/LogStream.h>

//
#include <GazeController.h>

bool GazeController::configure(const yarp::os::ResourceFinder &rf)
{
    yarp::os::Property prop;
    bool ok;

    // default trajectory times for neck and eyes
    neck_traj_time = 3.0;
    if (!rf.find("neckTrajTime").isNull())
    {
        yarp::os::Value time_v = rf.find("neckTrajTime");
        if (time_v.isDouble())
            neck_traj_time = time_v.asDouble();
    }
    yInfo() << "GazeController::configure"
            << "neck trajectory time is"
            << neck_traj_time;

    eyes_traj_time = 3.0;
    if (!rf.find("eyesTrajTime").isNull())
    {
        yarp::os::Value time_v = rf.find("eyesTrajTime");
        if (time_v.isDouble())
            eyes_traj_time = time_v.asDouble();
    }
    yInfo() << "GazeController::configure"
            << "eyes trajectory time is"
            << eyes_traj_time;

    /**
     * Drivers configuration
     */

    // prepare properties for the GazeController
    prop.put("device", "gazecontrollerclient");
    prop.put("remote", "/iKinGazeCtrl");
    prop.put("local", "/ground-truth-tracker/gazecontroller");

    // let's give the controller some time to warm up
    ok = false;
    double t0 = yarp::os::SystemClock::nowSystem();
    while (yarp::os::SystemClock::nowSystem() - t0 < 10.0)
    {
        // this might fail if controller
        // is not connected to solver yet
        if (drv_gaze.open(prop))
        {
            ok = true;
            break;
        }
        yarp::os::SystemClock::delaySystem(1.0);
    }
    if (!ok)
    {
        yError() << "GazeController::configure"
                 << "error: Unable to open the Gaze Controller driver";
        return false;
    }

    // try to retrieve the view
    ok = drv_gaze.view(igaze);
    if (!ok || igaze == 0)
    {
        yError() << "GazeController::configure"
                 << "error: Unable to retrieve the GazeController view";
        return false;
    }

    // store the initial context
    ok = igaze->storeContext(&startup_cart_context);
    if (!ok)
    {
        yError() << "GazeController::configure"
                 << "error: Unable to store the initial context";
        return false;
    }

    // set the default trajectory time
    igaze->setEyesTrajTime(eyes_traj_time);
    igaze->setNeckTrajTime(neck_traj_time);

    return true;
}

bool GazeController::close()
{
    bool ok;
    
    // stop any movement for safety
    ok = igaze->stopControl();
    if (!ok)
    {
        yError() << "GazeController::close"
                 << "error: Unable to stop the controller";
        yWarning() << "GazeController::close"
                   << "WARNING: the initial context will not be restored!";
        
        return false;
    }

    // restore initial context
    ok = igaze->restoreContext(startup_cart_context);
    if (!ok)
    {
        yError() << "GazeController::close"
                 << "error: unable to restore the initial context";
        
        return false;
    }

    // close the driver
    drv_gaze.close();

    return true;
}

bool GazeController::setReference(const yarp::sig::Vector &fixation_point)
{
    return igaze->lookAtFixationPoint(fixation_point);
}

bool GazeController::getCameraPose(const std::string &eye_name,
                                   yarp::sig::Vector &pos,
                                   yarp::sig::Vector &att)
{
    if ((eye_name != "right") && (eye_name != "left"))
        return false;

    if (eye_name == "right")
        return igaze->getRightEyePose(pos, att);
    else if(eye_name == "left")
        return igaze->getLeftEyePose(pos, att);
}
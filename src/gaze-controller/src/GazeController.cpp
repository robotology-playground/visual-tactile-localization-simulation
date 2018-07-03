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
#include <yarp/os/Bottle.h>

//
#include <GazeController.h>

bool GazeController::configure(const yarp::os::ResourceFinder &rf,
                               const std::string &port_prefix)
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
    prop.put("local", port_prefix + "/gazecontroller");

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

    // store the home fixation point
    home_fix.resize(3, 0.0);
    home_fix[0] = -0.3;

    return true;
}

bool GazeController::stop()
{
    return igaze->stopControl();
}

bool GazeController::clearEyes()
{
    return igaze->clearEyes();
}

bool GazeController::blockEyes(const double &vergence)
{
    return igaze->blockEyes(vergence);
}

bool GazeController::goHome()
{
    return igaze->lookAtFixationPointSync(home_fix);
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

bool GazeController::enableTrackingMode()
{
    return igaze->setTrackingMode(true);
}

bool GazeController::disableTrackingMode()
{
    return igaze->setTrackingMode(false);
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

bool GazeController::getCameraIntrinsics(const std::string eye_name,
                                         double &fx, double &fy,
                                         double &cx, double &cy)
{
    yarp::os::Bottle info;
    igaze->getInfo(info);
    std::string key = "camera_intrinsics_" + eye_name;

    if (info.find(key).isNull())
        return false;

    yarp::os::Bottle *list = info.find("camera_intrinsics_" + eye_name).asList();

    fx = list->get(0).asDouble();
    cx = list->get(2).asDouble();
    fy = list->get(5).asDouble();
    cy = list->get(6).asDouble();

    return true;
}

bool GazeController::setTrajectoryTimes()
{
    // set the default trajectory time
    if (!igaze->setEyesTrajTime(eyes_traj_time))
        return false;
    
    if (!igaze->setNeckTrajTime(neck_traj_time))
        return false;
}

bool GazeController::setHomeFixation(const yarp::sig::Vector &home)
{
    home_fix = home;
}

bool GazeController::isMotionDone(bool &done)
{
    return igaze->checkMotionDone(&done);
}

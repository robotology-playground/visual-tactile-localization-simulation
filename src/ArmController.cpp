/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#include "ArmController.h"

#include <cmath>

using namespace yarp::math;

bool ArmController::configure(const std::string &robot_name,
                              const std::string &arm_name)
{
    yarp::os::Property prop;
    bool ok;

    // store which arm
    this->arm_name = arm_name;

    /**
     * Drivers configuration
     */

    // prepare properties for the CartesianController
    prop.put("device", "cartesiancontrollerclient");
    prop.put("remote", "/" + robot_name + "/cartesianController/" + arm_name + "_arm");
    prop.put("local", "/" + arm_name + "_arm_controller/cartesian_client");

    // let's give the controller some time to warm up
    ok = false;
    double t0 = yarp::os::SystemClock::nowSystem();
    while (yarp::os::SystemClock::nowSystem() - t0 < 10.0)
    {
        // this might fail if controller
        // is not connected to solver yet
        if (drv_cart.open(prop))
        {
            ok = true;
            break;
        }
        yarp::os::SystemClock::delaySystem(1.0);
    }
    if (!ok)
    {
        yError() << "ArmController: Unable to open the Cartesian Controller driver"
                 << "for the"
                 << arm_name
                 << "arm.";
        return false;
    }

    // try to retrieve the view
    ok = drv_cart.view(icart);
    if (!ok || icart == 0)
    {
        yError() << "ArmController: Unable to retrieve the CartesianController view"
                 << "for the"
                 << arm_name
                 << "arm.";
        return false;
    }

    // prepare properties for the Encoders
    // these are required to retrieve fingers forward kinematics
    prop.put("device", "remote_controlboard");
    prop.put("remote", "/" + robot_name + "/" + arm_name + "_arm");
    prop.put("local", "/" + arm_name + "_arm_controller/encoder/arm");
    ok = drv_enc_arm.open(prop);
    if (!ok)
    {
        yError() << "ArmController: unable to open the Remote Control Board driver"
                 << "for the"
                 << arm_name
                 << "arm";
        return false;
    }

    // try to retrieve the view
    ok = drv_enc_arm.view(ienc_arm);
    if (!ok || ienc_arm == 0)
    {
        yError() << "ArmController: Unable to retrieve the Encoders view."
                 << "for the"
                 << arm_name
                 << "arm";
        return false;
    }

    /*
     *
     */

    /*
     * Cartesian Controller configuration
     */

    // store the current context so that
    // it can be restored when the controller closes
    icart->storeContext(&startup_cart_context);

    // set a default trajectory time
    icart->setTrajTime(5.0);

    // store home pose
    // wait until the pose is available
    while(!icart->getPose(home_pos, home_att))
        yarp::os::Time::yield();

    /*
     *
     */

    /**
     * Defaults
     */

    // hand attitude
    setHandAttitude(0, 0, 0);

    // this flag is true when a tip
    // is attached to the cartesian chain
    is_tip_attached = false;

    /*
     *
     */

    return true;
}

void ArmController::close()
{
    // stop the cartesian controller
    icart->stopControl();

    // restore the cartesian controller context
    icart->restoreContext(startup_cart_context);

    // close drivers
    drv_cart.close();
    drv_enc_arm.close();
}

yarp::dev::ICartesianControl* ArmController::cartesian()
{
    return icart;
}

void ArmController::setHandAttitude(const double &yaw = 0,
                                    const double &pitch = 0,
                                    const double &roll = 0)
{
    // given the reference frame convention for the hands of iCub
    // in order to place the right (left) hand in the standard configuration
    // it is required to have the x-axis attached to the center of the
    // palm pointing forward, the y-axis pointing downward (upward) and
    // the z-axis pointing leftward (rightward)
    //
    // one solution to obtain the final attitude w.r.t to the waist frame
    // is to compose a rotation of pi about the z-axis and a rotation
    // of -pi/2  about the x-axis

    yarp::sig::Vector axis_angle(4);
    yarp::sig::Matrix dcm;

    // rotation about z-axis
    axis_angle[0] = 0.0;
    axis_angle[1] = 0.0;
    axis_angle[2] = 1.0;
    axis_angle[3] = +M_PI;
    dcm = yarp::math::axis2dcm(axis_angle);

    // rotation about x-axis
    axis_angle[0] = 1.0;
    axis_angle[1] = 0.0;
    axis_angle[2] = 0.0;
    axis_angle[3] = -M_PI/2.0;
    dcm = dcm * yarp::math::axis2dcm(axis_angle);

    // additional rotations

    // rotation equivalent to a yaw rotation along the
    // z-axis of the robot waist
    axis_angle[0] = 0.0;
    axis_angle[1] = 0.0;
    axis_angle[2] = 1.0;
    axis_angle[3] = yaw * (M_PI/180);
    dcm = yarp::math::axis2dcm(axis_angle) * dcm;

    // pitch rotation along current z axis
    axis_angle[0] = 0.0;
    axis_angle[1] = 0.0;
    axis_angle[2] = 1.0;
    axis_angle[3] = pitch * (M_PI/180);
    dcm = dcm * yarp::math::axis2dcm(axis_angle);

    // roll rotation along current x axis
    axis_angle[0] = 1.0;
    axis_angle[1] = 0.0;
    axis_angle[2] = 0.0;
    axis_angle[3] = roll * (M_PI/180);
    dcm = dcm * yarp::math::axis2dcm(axis_angle);

    // store orientation
    hand_attitude = yarp::math::dcm2axis(dcm);
}

bool ArmController::attachFingerTip(const std::string& finger_name)
{
    bool ok;

    if (is_tip_attached)
    {
        // since a tip is already attached
        // first it is required to detach it
        return false;
    }

    // update tip status
    is_tip_attached = true;

    // get current value of encoders
    int n_encs;
    ok = ienc_arm->getAxes(&n_encs);
    if(!ok)
        return false;

    yarp::sig::Vector encs(n_encs);
    ok = ienc_arm->getEncoders(encs.data());
    if(!ok)
        return false;

    // get the transformation between the standard
    // effector and the desired finger
    yarp::sig::Vector joints;
    iCub::iKin::iCubFinger finger(arm_name + "_" + finger_name);
    ok = finger.getChainJoints(encs,joints);
    if (!ok)
        return false;
    yarp::sig::Matrix tip_frame = finger.getH((M_PI/180.0)*joints);

    // attach the tip taking into account only the positional part
    yarp::sig::Vector tip_x = tip_frame.getCol(3);
    yarp::sig::Matrix identity(3, 3);
    identity.eye();
    yarp::sig::Vector tip_a = yarp::math::dcm2axis(identity);
    ok = icart->attachTipFrame(tip_x, tip_a);
    if(!ok)
        return false;

    return true;
}

bool ArmController::detachFingerTip()
{
    bool ok;

    if (is_tip_attached)
    {
        ok = icart->removeTipFrame();

        if (!ok)
        {
            yError() << "ArmController::removeFingerFrame"
                     << "Error: unable to remove finger tip from the"
                     << arm_name << "arm chain";
            return false;
        }

        is_tip_attached = false;
    }

    return true;
}

bool ArmController::enableTorso()
{
    yarp::sig::Vector newDoF, curDoF;
    bool ok;

    // get the current DOFs
    ok = icart->getDOF(curDoF);
    if (!ok)
    {
        yError() << "ArmController::enableTorso"
                 << "Error: unable to get the current DOF configuration for the"
                 << arm_name << "arm chain";
        return false;
    }

    // enable torso
    newDoF = curDoF;
    newDoF[0] = 1;
    newDoF[1] = 1;
    newDoF[2] = 1;

    // set the new DOFs
    ok = icart->setDOF(newDoF, curDoF);
    if (!ok)
    {
        yError() << "ArmController::enableTorso"
                 << "Error: unable set the new DOF configuration for the"
                 << arm_name << "arm chain";
        return false;
    }

    return true;
}

bool ArmController::goHome()
{
    bool ok;

    // remove finger tip in case it is attached
    ok = detachFingerTip();
    if (!ok)
        return false;

    // store the context since the IK solver configuration
    // will be changed
    int current_context;
    ok = icart->storeContext(&current_context);
    if (!ok)
        return false;

    // since this function may be called for both
    // right and left arms it should be taken into account
    // the fact that different IK solutions for the torso
    // may be obtained
    // solution is to force the solution for the torso to
    // 0, 0, 0
    ok = icart->setLimits(0,0.0,0.0);
    ok &= icart->setLimits(1,0.0,0.0);
    ok &= icart->setLimits(2,0.0,0.0);
    if (!ok)
    {
        yError() << "ArmController::goHome"
                 << "Error: unable to force torso solution to 0 for the"
                 << arm_name << "arm";
        return false;
    }

    // restore home position
    ok = icart->goToPoseSync(home_pos, home_att);
    if (!ok)
    {
        yError() << "ArmController::goHome"
                 << "Error: unable to command home position for the"
                 << arm_name << "arm";
        return false;
    }
    icart->waitMotionDone(0.03, 5.0);

    // restore the context
    ok = icart->restoreContext(current_context);
    if (!ok)
    {
        yError() << "ArmController::goHome"
                 << "Error: unable to restore the previous context for the"
                 << arm_name << "arm";
        return false;
    }

    return true;
}

void ArmController::goToPos(const yarp::sig::Vector &pos)
{
    icart->goToPoseSync(pos, hand_attitude);
}

void ArmController::storeContext()
{
    icart->storeContext(&curr_cart_context);
}

void ArmController::restoreContext()
{
    icart->restoreContext(curr_cart_context);
}

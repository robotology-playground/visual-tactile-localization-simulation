/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#include "headers/ArmController.h"

#include <cmath>

using namespace yarp::math;

bool RightArmController::configure()
{
    return ArmController::configure("right");
}

bool LeftArmController::configure()
{
    return ArmController::configure("left");
}

bool ArmController::configure(const std::string &which_arm)
{
    yarp::os::Property prop;
    bool ok;

    // set default value for flag
    is_tip_attached = false;

    // store which arm
    this->which_arm = which_arm;
    
    // prepare properties for the CartesianController
    prop.put("device", "cartesiancontrollerclient");
    prop.put("remote", "/icubSim/cartesianController/" + which_arm + "_arm");
    prop.put("local", "/" + which_arm + "_arm_controller/cartesian_client");

    // let's give the controller some time to warm up
    // here use real time and not simulation time
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
		 << which_arm
		 << "arm.";
	return false;
    }

    // try to retrieve the view
    ok = drv_cart.view(icart);
    if (!ok || icart == 0)
    {
	yError() << "ArmController: Unable to retrieve the CartesianController view"
		 << "for the"
		 << which_arm
		 << "arm.";
	return false;
    }

    // store the current context so that
    // it can be restored when the controller closes
    icart->storeContext(&startup_cart_context);
	
    // use also the torso
    yarp::sig::Vector newDoF, curDoF;
    icart->getDOF(curDoF);
    newDoF = curDoF;

    newDoF[0] = 1;
    newDoF[1] = 1;
    newDoF[2] = 1;

    icart->setDOF(newDoF, curDoF);

    // set a default trajectory time
    icart->setTrajTime(1.0);

    // store home pose
    // wait until the pose is available
    while(!icart->getPose(home_pos, home_att))
	yarp::os::Time::yield();

    // configure default hand attitude
    setHandAttitude(0, 0, 0);

    // prepare properties for the Encoders
    // these are required to retrieve forward kinematics of the hand
    // without relying on the cartesian controller
    prop.put("device", "remote_controlboard");
    prop.put("remote", "/icubSim/" + which_arm + "_arm");
    prop.put("local", "/" + which_arm + "_arm_controller/encoder/arm");
    ok = drv_enc_arm.open(prop);
    if (!ok)
    {
	yError() << "ArmController: unable to open the Remote Control Board driver"
		 << "for the"
		 << which_arm
		 << "arm";
	return false;
    }

    prop.put("remote", "/icubSim/torso");
    prop.put("local", "/" + which_arm + "_arm_controller/encoder/torso");    
    ok = drv_enc_torso.open(prop);
    if (!ok)
    {
	yError() << "ArmController: unable to open the Remote Control Board driver"
		 << "for the torso.";
	return false;
    }

    // try to retrieve the views
    ok = drv_enc_arm.view(ienc_arm);
    if (!ok || ienc_arm == 0)
    {
	yError() << "ArmController: Unable to retrieve the Encoders view."
		 << "for the"
		 << which_arm
		 << "arm";
	return false;
    }

    ok = drv_enc_torso.view(ienc_torso);
    if (!ok || ienc_torso == 0)
    {
	yError() << "ArmController: Unable to retrieve the Encoders view."
		 << "for the torso";
	return false;
    }

    // instantiate arm chain
    arm_chain = iCub::iKin::iCubArm(which_arm);
    // limits update is not required to evaluate the forward kinematics
    // using angles from the encoders
    arm_chain.setAllConstraints(false);
    // torso can be moved in general so its links have to be released
    arm_chain.releaseLink(0);
    arm_chain.releaseLink(1);
    arm_chain.releaseLink(2);

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
    drv_enc_torso.close();
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
    // of -pi/2 (pi/2) about the x-axis (after the first rotation)

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
    if (which_arm == "right")
	axis_angle[3] = -M_PI/2.0;
    else
	axis_angle[3] = M_PI/2.0;
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

bool ArmController::getHandPose(yarp::sig::Vector& pos,
				yarp::sig::Matrix& rot)
{
    // get current value of encoders
    yarp::sig::Vector encs_torso(3);
    yarp::sig::Vector encs_arm(16);

    bool ok = ienc_arm->getEncoders(encs_arm.data());
    if(!ok)
	return false;

    ok = ienc_torso->getEncoders(encs_torso.data());
    if(!ok)
	return false;

    // fill in the vector of degrees of freedom
    yarp::sig::Vector joints_angles(arm_chain.getDOF());
    joints_angles[0] = encs_torso[2];
    joints_angles[1] = encs_torso[1];
    joints_angles[2] = encs_torso[0];
    joints_angles[3] = encs_arm[0];
    joints_angles[4] = encs_arm[1];
    joints_angles[5] = encs_arm[2];
    joints_angles[6] = encs_arm[3];
    joints_angles[7] = encs_arm[4];
    joints_angles[8] = encs_arm[5];    
    joints_angles[9] = encs_arm[6];

    // set the current values of the joints
    // iKin uses radians
    arm_chain.setAng((M_PI/180) * joints_angles);

    // get the transform from the robot root frame
    // to the frame attached to the plam of the hand
    yarp::sig::Matrix inertial_to_hand = arm_chain.getH();

    // extract position and rotation matrix	    
    pos = inertial_to_hand.getCol(3).subVector(0,2);
    rot = inertial_to_hand.submatrix(0, 2, 0, 2);

    return true;
}

bool ArmController::useFingerFrame(const std::string& finger_name)
{
    bool ok;

    if (is_tip_attached)
    {
	// since a tip is already attached
	// first it is required to detach it
	ok = removeFingerFrame();
	if (!ok)
	    return false;
    }
    else
    {
	is_tip_attached = true;
    }	
	
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
    iCub::iKin::iCubFinger finger(which_arm + "_" + finger_name);
    ok = finger.getChainJoints(encs,joints);
    if (!ok)
	return false;
    yarp::sig::Matrix tip_frame = finger.getH((M_PI/180.0)*joints);

    // attach the tip
    yarp::sig::Vector tip_x = tip_frame.getCol(3);
    yarp::sig::Vector tip_o = yarp::math::dcm2axis(tip_frame);
    ok = icart->attachTipFrame(tip_x,tip_o);
    if(!ok)
	return false;

    return true;
}

bool ArmController::removeFingerFrame()
{
    is_tip_attached = false;
    
    return icart->removeTipFrame();
}

bool ArmController::goHome()
{
    bool ok;
    
    // since this function may be called for both
    // right and left arms it should be taken into account
    // the fact that different IK solutions for the torso
    // may be obtained
    // solution is to force the solution for the torso to
    // 0, 0, 0

    // store the context
    int current_context;
    ok = icart->storeContext(&current_context);
    if (!ok)
	return false;

    // remove finger tip in case it is attached
    if (is_tip_attached)
    {	
	ok = removeFingerFrame();
	if (!ok)
	    return false;
    }
    
    // force the IK to use 0, 0, 0
    // as solution for the torso
    ok = icart->setLimits(0,0.0,0.0);
    ok &= icart->setLimits(1,0.0,0.0);
    ok &= icart->setLimits(2,0.0,0.0);
    if (!ok)
	return false;

    // restore home position
    ok = icart->goToPoseSync(home_pos, home_att);
    if (!ok)
	return false;
    ok = icart->waitMotionDone(0.03, 2);
    if (!ok)
	return false;

    // stop control
    ok = icart->stopControl();
    if (!ok)
	return false;
    
    // restore the context
    // this restore also the finger tip if it was attached
    ok = icart->restoreContext(current_context);
    if (!ok)
	return false;

    return true;
}

void ArmController::goToPos(const yarp::sig::Vector &pos)
{
    icart->goToPoseSync(pos, hand_attitude);
}

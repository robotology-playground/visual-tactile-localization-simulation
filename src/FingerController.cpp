/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#include "headers/FingerController.h"

using namespace yarp::math;

bool FingerController::configure(const std::string &hand_name,
				 const std::string &finger_name,
				 yarp::dev::IControlMode2 *imod,
				 yarp::dev::IPositionControl2 *ipos,				 
				 yarp::dev::IVelocityControl2 *ivel)
{
    bool ok;

    // store pointer to ControlMode2 instance
    this->imod = imod;

    // store pointer to PositionControl2 instance
    this->ipos = ipos;
    
    // store pointer to VelocityControl2 instance
    this->ivel = ivel;
    
    // store name of the finger
    this->finger_name = finger_name;

    // store name of the hand
    this->hand_name = hand_name;
    
    // initialize the finger
    finger = iCub::iKin::iCubFinger(hand_name + "_" + finger_name);

    // set the controlled joints depending on the finger name
    if (finger_name == "index")
    {
	ctl_joints.push_back(11);
	ctl_joints.push_back(12);
    }
    else if (finger_name == "middle")
    {
	ctl_joints.push_back(13);
	ctl_joints.push_back(14);	
    }
    else if (finger_name == "ring")
    {
	ctl_joints.push_back(15);
	// FIX ME :the forward kinematics of the ring finger is not available
	// using the forward kinematics of the index finger
	finger = iCub::iKin::iCubFinger(hand_name + "_index");
    }
    else
    {
	yError() << "FingerController:configure"
		 << "Error: finger"
		 << finger_name
		 << "is not valid or not supported";
	return false;
    }

    // get the current control modes for the controlled DoFs
    // FIX ME: not working with Gazebo
    // initial_modes.resize(ctl_joints.size());
    // ok = imod->getControlModes(ctl_joints.size(),
    // 		   		  ctl_joints.getFirst(),
    // 				  initial_modes.getFirst());
    
    // set the velocity control mode for the controlled DoFs
    ok = setControlMode(VOCAB_CM_VELOCITY);
    if (!ok)
    {
	yError() << "FingerController:configure"
		 << "Error: unable to set the velocity control"
		 << "mode for the joints of the"
		 << hand_name << finger_name
		 << "finger";
	
	return false;
    }

    // compose jacobian coupling matrix
    coupling.resize(3, ctl_joints.size());
    coupling = 0;
    if (finger_name == "index" || finger_name == "middle")
    {
	coupling[0][0] = 1;   // proximal joint velocity = velocity of first DoF
	coupling[1][1] = 0.5; // first distal joint velocity = half velocity of second DoF
	coupling[2][1] = 0.5; // second distal joint velocity = half velocity of second DoF
    }
    else if (finger_name == "ring")
    {
	// within ring only one DoF is available
	coupling = 1.0 / 3.0;
    }

    // extract the constant transformation between the hand
    // and the root frame of the finger once for all
    bool use_axis_angle = true;
    yarp::sig::Vector pose;
    pose = finger.Pose(0, use_axis_angle);
    finger_root_pos = pose.subVector(0,2);
    finger_root_att = yarp::math::axis2dcm(pose.subVector(3,6));
    finger_root_att = finger_root_att.submatrix(0, 2, 0, 2);

    // set default home joints position
    joints_home.resize(ctl_joints.size(), 0.0);

    return true;
}

bool FingerController::setControlMode(const int &mode)
{
    bool ok;

    // get current control modes first
    yarp::sig::VectorOf<int> modes(ctl_joints.size());
    ok = imod->getControlModes(ctl_joints.size(),
			       ctl_joints.getFirst(),
			       modes.getFirst());
    if (!ok)
    {
	yError() << "FingerController:setControlMode"
		 << "Error: unable to get current joints control modes for finger"
		 << hand_name << finger_name;
	return false;
    }

    // set only the control modes different from the desired one
    for (size_t i=0; i<modes.size(); i++)
    {
	if (modes[i] != mode)
	{
	    ok = imod->setControlMode(ctl_joints[i], mode);
	    if (!ok)
	    {
		yError() << "FingerController:setControlMode"
			 << "Error: unable to set control mode for one of the joint of the finger"
			 << hand_name << finger_name;
		return false;
	    }
	}
    }
    
    return true;
}

bool FingerController::close()
{
    bool ok;
    
    // stop motion
    ok = ivel->stop(ctl_joints.size(), ctl_joints.getFirst());
    if (!ok)
    {
	yError() << "FingerController::close"
		 << "WARNING: unable to stop joints motion for finger"
		 << hand_name << finger_name;
	return false;
    }

    // restore initial control mode
    // FIX ME: not working with Gazebo 
    // ok = imod->setControlModes(ctl_joints.size(),
    // 				  ctl_joints.getFirst(),
    // 				  initial_modes.getFirst());
    // if (!ok)
    // {
    // 	yError() << "FingerController:close"
    // 		 << "Error: unable to restore the initial control modes for finger"
    // 		 << hand_name << finger_name;
    // 	return false;
    // }
    
    return true;
}

void FingerController::setHomePosition(const yarp::sig::Vector &encoders)
{
    // extract values of controlled DoF
    for (size_t i=0; i<ctl_joints.size(); i++)
	joints_home[i] = encoders[ctl_joints[i]];
}

bool FingerController::updateFingerChain(const yarp::sig::Vector &encoders)
{
    bool ok;
    
    // get subset of joints related to the finger
    ok = finger.getChainJoints(encoders, joints);
    if (!ok)
    {
	yError() << "FingerController::updateFingerChain"
		 << "Error: unable to retrieve the finger's joint values for finger"
		 << hand_name << finger_name;
	return false;
    }

    // convert to radians
    joints = joints * (M_PI/180.0);

    // update chain
    finger.setAng(joints);

    return true;
}

bool FingerController::getJacobianFingerFrame(yarp::sig::Matrix &jacobian)
{
    // jacobian for linear velocity part
    yarp::sig::Matrix j_lin;

    // jacobian for angular velocity part
    yarp::sig::Matrix j_ang;

    // get the jacobian
    jacobian = finger.GeoJacobian();

    // neglect abduction if not the middle finger
    if (finger_name != "middle")
	jacobian.removeCols(0, 1);

    // the number of columns should be three
    if (jacobian.cols() != 3)
    {
	yError() << "FingerController::getJacobianFingerFrame"
		 << "Error: jacobian.cols() ="
		 << jacobian.cols()
		 << "instead of 3 for finger"
		 << hand_name << finger_name;
	
	return false;
    }

    // extract linear velocity part
    j_lin = jacobian.submatrix(0, 2, 0, 2);

    // express linear velocity in the root frame of the finger
    j_lin = finger_root_att.transposed() * j_lin;

    // the motion of the finger described w.r.t its root frame
    // is planar and velocities along the z axis are zeros
    // hence the third row of the linear velocity jacobian can be dropped
    j_lin = j_lin.removeRows(2, 1);

    // extract angular velocity part
    j_ang = jacobian.submatrix(3, 5, 0, 2);

    // express angular velocity in root frame of the finger
    j_ang = finger_root_att.transposed() * j_ang;

    // the motion of the finger described w.r.t its root frame
    // is planar and angular velocities is all along the z axis
    // hence first and second row of the angular velocity jacobian
    // can be dropped
    j_ang.removeRows(0, 2);

    // compose the linear velocity and angular velocity parts together
    jacobian.resize(3, 3);
    jacobian.setSubmatrix(j_lin, 0, 0);
    jacobian.setSubmatrix(j_ang, 2, 0);

    // take into account coupling
    jacobian = jacobian * coupling;
}

bool FingerController::getFingerTipPoseFingerFrame(yarp::sig::Vector &pose)
{
    // get the position of the finger tip
    yarp::sig::Vector finger_tip = finger.EndEffPosition();

    // evaluate the vector from the root frame of the finger
    // to the finger tip
    yarp::sig::Vector diff = finger_tip - finger_root_pos;

    // express it in the root frame of the finger
    diff = finger_root_att.transposed() * diff;

    // evaluate the sum of the controlled joints
    // representing the attitude of the planar chain
    double att = 0;

    // neglect abduction if not the middle finger
    if (finger_name != "middle")
	att = joints[1] + joints[2] + joints[3];
    else
	att = joints[0] + joints[1] + joints[2];
    
    pose.resize(3);
    pose[0] = diff[0];
    pose[1] = diff[1];
    pose[2] = att;
    
    return true;
}

bool FingerController::goHome()
{
    bool ok;
    
    // switch to position control
    ok = setControlMode(VOCAB_CM_POSITION);
    if (!ok)
    {
	yInfo() << "FingerController::goHome Error:"
		<< "unable to set Position control mode for finger"
		<< finger_name;

	return false;
    }
    
    // restore initial position of finger joints
    ok = ipos->positionMove(ctl_joints.size(),
			    ctl_joints.getFirst(),
			    joints_home.data());
    if (!ok)
    {
	yInfo() << "FingerController::goHome Error:"
		<< "unable to restore initial positions of joints of finger"
		<< finger_name;

	return false;
    }

    return true;
}

bool FingerController::setJointsVelocities(const yarp::sig::Vector &vels)
{
    bool ok;
    
    // switch to velocity control
    // FIX ME: it is better if the user calls this once outside of this function
    //         since this is used in "streaming" mode
    // ok = setControlMode(VOCAB_CM_VELOCITY);
    if (!ok)
    {
	yInfo() << "FingerController::setJointsVelocites Error:"
		<< "unable to set Velocity control mode for finger"
		<< finger_name;

	return false;
    }
    
    // convert velocities to deg/s
    yarp::sig::Vector vels_deg = vels * (180.0/M_PI);

    // issue velocity command
    return ivel->velocityMove(ctl_joints.size(), ctl_joints.getFirst(), vels_deg.data());
}

bool FingerController::moveFingerForward(const double &speed)
{
    // get the jacobian in the current configuration
    yarp::sig::Matrix jac;
    getJacobianFingerFrame(jac);

    // remove attitude part (i.e. third row)
    jac.removeRows(2, 1);

    // remove velocity along x part (i.e. first row)
    jac.removeRows(0, 1);
    
    // find joint velocities minimizing v_y - J_y * q_dot
    yarp::sig::Vector q_dot;
    yarp::sig::Vector vel(1, speed);
    
    q_dot = jac.transposed() *
	yarp::math::pinv(jac * jac.transposed()) * vel;

    // issue velocity command
    return setJointsVelocities(q_dot);
}

bool FingerController::stop()
{
    bool ok;
    
    // stop motion
    ok = ivel->stop(ctl_joints.size(), ctl_joints.getFirst());
    if (!ok)
	return false;

    return true;
}

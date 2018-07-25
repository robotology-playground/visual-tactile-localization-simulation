/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file handKinematics.cpp
 * @authors: Nicola Piga <nicola.piga@iit.it>
 */

//
#include <cmath>

#include "handKinematics.h"

using namespace yarp::math;

bool handKinematics::configure(const std::string &robot_name,
			       const std::string &hand_name,
			       const bool &use_analog,
                               const bool &use_analog_bounds,
			       const std::string &port_prefix)
{
    // encoders configuration
    this->use_analog = use_analog;
    this->use_analog_bounds = use_analog_bounds;

    // hand name
    this->hand_name = hand_name;

    // motor encoders
    yarp::os::Property prop_encoders;
    prop_encoders.put("device", "remote_controlboard");
    prop_encoders.put("remote", "/" + robot_name + "/" + hand_name + "_arm");
    prop_encoders.put("local", port_prefix + "/" + hand_name + "_hand_kinematics/motors_encoders");

    // try to open driver
    bool ok_drv = drv_arm.open(prop_encoders);
    if (!ok_drv)
    {
	yError() << "handKinematics::configure error:"
		 << "unable to open the Remote Control Board driver for the"
		 << hand_name
		 << "hand";
	return false;
    }

    prop_encoders.put("remote", "/" + robot_name + "/torso");
    prop_encoders.put("local", port_prefix + "/" + hand_name + "_hand_kinematics/torso");
    ok_drv = drv_torso.open(prop_encoders);
    if (!ok_drv)
    {
	yError() << "handKinematics::configure error:"
		 << "unable to open the Remote Control Board driver for the torso"
		 << "(required for" << hand_name << "hand)";
	return false;
    }

    // additional joints encoders for the fingers
    if (use_analog)
    {
	prop_encoders.put("device", "analogsensorclient");
	prop_encoders.put("remote", "/" + robot_name + "/" + hand_name + "_hand/analog:o");
	prop_encoders.put("local", port_prefix + "/" + hand_name + "_hand_kinematics/joints_encoders");
	ok_drv = drv_analog.open(prop_encoders);
	if (!ok_drv)
	{
	    yError() << "handKinematics::configure error:"
		     << "unable to open the Analog Sensor Client driver for the"
		     << hand_name
		     << "hand";
	    return false;
	}
    }

    // try to retrieve the views
    bool ok_view = drv_arm.view(ienc_arm);
    if (!ok_view || ienc_arm == 0)
    {
	yError() << "handKinematics::configure error:"
		 << "unable to retrieve the Encoders view for the"
		 << hand_name
		 << "hand";
	return false;
    }

    ok_view = drv_torso.view(ienc_torso);
    if (!ok_view || ienc_torso == 0)
    {
	yError() << "handKinematics::configure error:"
		 << "unable to retrieve the Encoders view for the torso"
		 << "(required for" << hand_name << "hand)";
	return false;
    }

    if (use_analog)
    {
        ok_view = drv_analog.view(ianalog);
        if (!ok_view || ianalog == 0)
        {
            yError() << "handKinematics::configure error:"
                     << "unable to retrieve the IAnalogSensor view for the"
                     << hand_name
                     << "hand";
        }

        ok_view = drv_arm.view(ilim);
        if (!ok_view || ilim == 0)
        {
            yError() << "handKinematics:configure error:"
                     << "unable to retrieve the IControlLimits view for the"
                     << hand_name
                     << "hand";
            return false;
        }
    }
    // instantiate forward kinematics for arm
    arm_kin = iCub::iKin::iCubArm(hand_name);

    // limits update is not required to evaluate the forward kinematics
    // using angles from the encoders
    arm_kin.setAllConstraints(false);
    // torso can be moved in general so its links have to be released
    arm_kin.releaseLink(0);
    arm_kin.releaseLink(1);
    arm_kin.releaseLink(2);

    // store fingers names
    fingers_names.push_back("thumb");
    fingers_names.push_back("index");
    fingers_names.push_back("middle");
    fingers_names.push_back("ring");
    fingers_names.push_back("little");

    // instantiate forward kinematics for fingers
    for (std::string &name : fingers_names)
	fingers_kin[name] = iCub::iKin::iCubFingerExt(hand_name + "_" + name);

    if (use_analog)
    {
        // configure finger limits if analogs are going to be used
        bool ok;
        std::deque<yarp::dev::IControlLimits*> lims;
        lims.push_back(ilim);
        for (std::string &name : fingers_names)
        {
            ok = fingers_kin[hand_name + "_" + name].alignJointsBounds(lims);
            if (!ok)
            {
                yError() << "handKinematics:configure error:"
                         << "cannot set joints bounds for finger"
                         << hand_name + name;
                return false;
            }
        }

        // setup analog bounds
        if (use_analog_bounds)
            setupAnalogBounds();
    }
    return true;
}

bool handKinematics::getEncoders(yarp::sig::Vector &arm_encs,
				 yarp::sig::Vector &torso_encs,
				 yarp::sig::Vector &fingers_analogs)
{
    // resize vectors
    arm_encs.resize(16);
    torso_encs.resize(3);
    fingers_analogs.resize(15);
    bool ok = ienc_arm->getEncoders(arm_encs.data());
    if(!ok)
	return false;
    ok = ienc_torso->getEncoders(torso_encs.data());
    if(!ok)
	return false;
    if (use_analog)
    {
	ok = ianalog->read(fingers_analogs);
	if(ok != yarp::dev::IAnalogSensor::AS_OK)
	    return false;
    }

    return true;
}

void handKinematics::getArmDofs(const yarp::sig::Vector &arm_encs,
				const yarp::sig::Vector &torso_encs,
				yarp::sig::Vector &arm_dofs)
{
    // these are the dofs as expected by the class
    // iCub::iKin::iCubArm
    arm_dofs.resize(arm_kin.getDOF());
    arm_dofs[0] = torso_encs[2];
    arm_dofs[1] = torso_encs[1];
    arm_dofs[2] = torso_encs[0];
    arm_dofs.setSubvector(3, arm_encs.subVector(0, 6));
}

bool handKinematics::getFingersLinks(fingersLinks &links)
{
    bool ok;

    // get current value of encoders
    yarp::sig::Vector arm_encs;
    yarp::sig::Vector torso_encs;
    yarp::sig::Vector fingers_analogs;
    ok = getEncoders(arm_encs, torso_encs, fingers_analogs);
    if (!ok)
	return false;

    // update the arm chain
    yarp::sig::Vector arm_dofs;
    getArmDofs(arm_encs, torso_encs, arm_dofs);
    arm_kin.setAng((M_PI/180.0) * arm_dofs);

    // get the current pose of the hand root frame
    yarp::sig::Matrix hand_root_pose = arm_kin.getH();

    links.clear();

    // add the palm
    std::vector<yarp::sig::Matrix> palm_pose;
    palm_pose.push_back(hand_root_pose);
    links["palm"] = palm_pose;

    // process all the fingers
    for (std::string &name : fingers_names)
    {
	// get finger forward kinematics
	iCub::iKin::iCubFingerExt &finger = fingers_kin[name];

	// get joints for the current finger
	yarp::sig::Vector finger_dofs;
	if (use_analog)
        {
            if (use_analog_bounds)
                finger.getChainJoints(arm_encs, fingers_analogs, finger_dofs, analog_bounds);
            else
                finger.getChainJoints(arm_encs, fingers_analogs, finger_dofs);
        }
	else
	    finger.getChainJoints(arm_encs, finger_dofs);

	// update che finger chain
	finger.setAng((M_PI/180.0) * finger_dofs);

	// prepare a vector of poses
	std::vector<yarp::sig::Matrix> poses;

	// first insert H0
	// in case the user is interested in the transform
	// between the hand root frame and the 0-th frame of the finger
	poses.push_back(hand_root_pose * finger.getH0());

	// handle middle finger carefully
	if (name == "middle")
	    poses.push_back(hand_root_pose * finger.getH0());

	// set the final index
	//
	// note that finger.getN() would return
	// thumb:  5 but one link is blocked -> final index 4 - 1 = 3
	// index:  4                         -> final_index       = 3
	// middle: 3                         -> final_index       = 2
	// ring:   4                         -> final_index       = 3
	// little: 4                         -> final_index       = 3
	//
	int final_index = 3;
	if (name == "middle")
	    final_index = 2;

	// process all links
	for (size_t i=0; i <= final_index; i++)
	{
	    if (i == 0 && name == "thumb")
		poses.push_back(hand_root_pose * finger.getH(1, true));
	    else
		poses.push_back(hand_root_pose * finger.getH(i));
	}

	links[name] = poses;
    }

    return true;
}

void handKinematics::close()
{
    // close remote control board driver
    drv_arm.close();
}

void handKinematics::setupAnalogBounds()
{
    // taken from real robot
    // by avering 50 trials
    // left hand only

    analog_bounds.resize(15, 2);
    if (hand_name == "left")
    {
        analog_bounds(0, 0) = 250.0; // not acquired
        analog_bounds(1, 0) = 250.0; // not acquire
        analog_bounds(2, 0) = 250.0; // not acquired
        analog_bounds(3, 0) = 243.22;
        analog_bounds(4, 0) = 216.44;
        analog_bounds(5, 0) = 230.44;
        analog_bounds(6, 0) = 244.46;
        analog_bounds(7, 0) = 214.38;
        analog_bounds(8, 0) = 247.98;
        analog_bounds(9, 0) = 229.56;
        analog_bounds(10, 0) = 208.84;
        analog_bounds(11, 0) = 231.54;
        analog_bounds(12, 0) = 249.20;
        analog_bounds(13, 0) = 212.52;
        analog_bounds(14, 0) = 227.62;
        analog_bounds(0, 1) = 0.0; // not acquired
        analog_bounds(1, 1) = 0.0; // not acquired
        analog_bounds(2, 1) = 0.0; // not acquired
        analog_bounds(3, 1) = 9.42;
        analog_bounds(4, 1) = 39.66;
        analog_bounds(5, 1) = 0.0;
        analog_bounds(6, 1) = 0.0;
        analog_bounds(7, 1) = 53.54;
        analog_bounds(8, 1) = 20.0;
        analog_bounds(9, 1) = 23.64;
        analog_bounds(10, 1) = 25.0;
        analog_bounds(11, 1) = 29.68;
        analog_bounds(12, 1) = 20.76;
        analog_bounds(13, 1) = 62.68;
        analog_bounds(14, 1) = 62.22;
    }
}

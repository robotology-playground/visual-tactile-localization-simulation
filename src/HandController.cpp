/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

// yarp
#include <yarp/os/Property.h>
#include <yarp/math/Math.h>

#include "headers/HandController.h"

#include <cmath>

using namespace yarp::math;

bool RightHandController::configure()
{
    return HandController::configure("right");
}

bool LeftHandController::configure()
{
    return HandController::configure("left");
}

bool HandController::configure(const std::string &hand_name)
{
    // store name of the hand
    this->hand_name = hand_name;
    
    // prepare properties for the Encoders
    yarp::os::Property prop;
    prop.put("device", "remote_controlboard");
    prop.put("remote", "/icubSim/" + hand_name + "_arm");
    prop.put("local", "/hand_controller/" + hand_name + "_arm/encoders");
    bool ok = drv_arm.open(prop);
    if (!ok)
    {
	yError() << "HandController::configure"
		 << "Error: unable to open the Remote Control Board driver";
	return false;
    }

    // try to retrieve the views
    ok = drv_arm.view(ienc_arm);
    if (!ok || ienc_arm == 0)
    {
	yError() << "HandController:configure"
		 << "Error: unable to retrieve the Encoders view";
	return false;
    }

    ok = drv_arm.view(ipos_arm);
    if (!ok || ipos_arm == 0)
    {
	yError() << "HandController:configure"
		 << "Error: unable to retrieve the PositionControl2 view";
	return false;
    }

    ok = drv_arm.view(ivel_arm);
    if (!ok || ivel_arm == 0)
    {
	yError() << "HandController:configure"
		 << "Error: unable to retrieve the VelocityControl2 view";
	return false;
    }

    ok = drv_arm.view(imod_arm);
    if (!ok || imod_arm == 0)
    {
	yError() << "HandController:configure"
		 << "Error: unable to retrieve the ControlMode2 view";
	return false;
    }

    // get the current encoder readings
    // required to set the home position of the fingers joints
    ok = false;
    double t0 = yarp::os::SystemClock::nowSystem();    
    yarp::sig::Vector joints;    
    while (yarp::os::SystemClock::nowSystem() - t0 < 10.0)
    {
	// this might fail if the gazebo pluging
	// exposing encoders is not yet ready
	if (getJoints(joints))
	{
	    ok = true;
	    break;
	}
	yarp::os::SystemClock::delaySystem(1.0);
    }
    
    // handle fingers
    fingers_names = {"index", "middle", "ring"};
    for (std::string finger_name : fingers_names)
    {
	// instantiate and configure fingers	
    	FingerController finger;
    	finger.configure(hand_name, finger_name, imod_arm, ipos_arm, ivel_arm);
    	fingers[finger_name] = finger;

	// set home position
	finger.setHomePosition(joints);

	// reset fingers contacts
	contacts[finger_name] = false;
    }
    
    return true;
}

bool HandController::close()
{    
    // close driver
    drv_arm.close();

    return true;
}

bool HandController::getJoints(yarp::sig::Vector &joints)
{
    bool ok;

    // extract joints for the whole arm
    int n_encs;
    ok = ienc_arm->getAxes(&n_encs);
    if (!ok)
    {
	yError() << "HandController::getJoints"
		 << "Error: unable to retrieve the number of controlled axes";
	return false;
    }
    joints.resize(n_encs);
    ok = ienc_arm->getEncoders(joints.data());
    if (!ok)
    {
	yError() << "HandController::getJoints"
		 << "Error: unable to retrieve the encoder readings";
	return false;
    }

    return true;
}

void HandController::resetFingersContacts()
{
    for (auto &contact : contacts)
    {
	bool &is_contact = contact.second;

	is_contact = false;
    }
}

bool HandController::moveFingersUntilContact(const double &speed,
					     const std::unordered_map<std::string, int> &number_contacts,
                                             bool &done)
{
    bool ok;
    
    // get current joints
    yarp::sig::Vector joints;
    getJoints(joints);

    for (std::string finger_name : fingers_names)
    {
	// if finger never reached contact
	if (!contacts[finger_name])
	{
	    // get the finger controller
	    FingerController &ctl = fingers[finger_name];

	    // try to update finger chain
	    ok = ctl.updateFingerChain(joints);
	    if (!ok)
		return false;
	    
	    // check if contact is reached now
	    if (number_contacts.at(finger_name) > 0 ||
		// this is because the ring and the pinky are coupled
		// and the pinky could touch before the ring finger
		finger_name == "ring" && number_contacts.at("pinky") > 0)
	    {   yInfo() << finger_name << number_contacts.at(finger_name);
		// stop the finger
		ok = ctl.stop();
		if (!ok)
		    return false;

		// remember that contact was reached
		contacts[finger_name] = true;
	    }
	    else
	    {
		// continue finger movements
		ok = ctl.moveFingerForward(speed);
		if (!ok)
		    return false;
	    }
	}
    }

    // check if all the contacts were reached
    done = true;
    for (auto &contact : contacts)
    {
	// get contact status
	bool &is_contact = contact.second;

	done &= is_contact;
    }

    return true;
}

bool HandController::restoreFingersPosition()
{
    bool ok;

    // command all the fingers
    for (std::string finger_name : fingers_names)
    {
	// get the finger controller
	FingerController &ctl = fingers[finger_name];

	// try to restore the initial position of the finger
	ok = ctl.goHome();
	if (!ok)
	    return false;
    }

    return true;
}

bool HandController::stopFingers()
{
    bool ok;
    
    for (std::string finger_name : fingers_names)
    {
	// get the finger controller
	FingerController &ctl = fingers[finger_name];

	// try to restore the initial position of the finger
	ok = ctl.stop();
	if (!ok)
	    return false;
    }

    return true;
}

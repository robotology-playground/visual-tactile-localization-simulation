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
#include <yarp/os/Vocab.h>

// std
#include <algorithm>

#include "headers/HandControlCommand.h"

HandControlCommand::HandControlCommand() : linear_forward_speed(0.0),
					   joint_restore_speed(0.0),
					   cmd(Command::Empty),
					   is_linear_forward_speed_set(false),
					   is_joint_restore_speed_set(false)
{
    // set the list of available fingers
    available_fingers = { "thumb", "index", "middle", "ring", "little" };
}

bool HandControlCommand::setCommandedHand(const std::string &hand_name)
{
    if (commanded_hand != "right" &&
	commanded_hand != "left")
	return false;
    
    commanded_hand = hand_name;

    return true;
}

bool HandControlCommand::setCommandedFinger(const std::string &finger_name)
{
    if (available_fingers.find(finger_name) == available_fingers.end())
	return false;

    commanded_fingers.insert(finger_name);
    
    return true;
}

bool HandControlCommand::setCommandedFingers(const std::vector<std::string> &fingers_names)
{
    for (const std::string &finger : fingers_names)
	if (!setCommandedFinger(finger))
	    return false;

    return true;
}

bool HandControlCommand::setFingersForwardSpeed(const double &speed)
{
    if (speed < 0)
	return false;
    
    linear_forward_speed = speed;
    is_linear_forward_speed_set = true;

    return true;
}

bool HandControlCommand::setFingersRestoreSpeed(const double &speed)
{
    if (speed < 0)
	return false;

    joint_restore_speed = speed;
    is_joint_restore_speed_set = true;

    return true;
}

void HandControlCommand::commandFingersApproach()
{
    cmd = Command::Approach;
}

void HandControlCommand::commandFingersFollow()
{
    cmd = Command::Follow;
}

void HandControlCommand::commandFingersRestore()
{
    cmd = Command::Restore;
}

void HandControlCommand::commandStop()
{
    cmd = Command::Stop;
}

void HandControlCommand::clear()
{
    commanded_hand.clear();
    commanded_fingers.clear();
    cmd = Command::Empty;
    is_linear_forward_speed_set = false;
    is_joint_restore_speed_set = false;
}

std::string HandControlCommand::getCommandedHand() const
{
    return commanded_hand;
}

void HandControlCommand::getCommandedFingers(std::set<std::string> &fingers_names) const
{
    fingers_names = commanded_fingers;
}

bool HandControlCommand::getForwardSpeed(double &speed) const
{
    if (!is_linear_forward_speed_set)
	return false;

    speed = linear_forward_speed;

    return true;
}

bool HandControlCommand::getRestoreSpeed(double &speed) const
{
    if (!is_joint_restore_speed_set)
	return false;

    speed = joint_restore_speed;

    return true;
}

Command HandControlCommand::getCommand() const
{
    return cmd;
}

bool HandControlCommand::read(yarp::os::ConnectionReader& connection)
{
    return !connection.isError();
}

bool HandControlCommand::write(yarp::os::ConnectionWriter& connection)
{
    return !connection.isError();    
}

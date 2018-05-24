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

#include "HandControlCommand.h"

HandControlCommand::HandControlCommand() : linear_forward_speed(0.0),
                                           joint_restore_speed(0.0),
                                           cmd(Command::Empty),
                                           is_linear_forward_speed_set(false),
                                           is_joint_restore_speed_set(false),
                                           available_fingers{ "thumb",  "index", "middle", "ring", "little"}
{
    // reset the commanded fingers
    resetCommandedFingers();
}

bool HandControlCommand::setCommandedHand(const std::string &hand_name)
{
    if (hand_name != "right" &&
        hand_name != "left")
        return false;

    commanded_hand = hand_name;

    return true;
}

bool HandControlCommand::setCommandedFinger(const std::string &finger_name)
{
    if (available_fingers.find(finger_name) == available_fingers.end())
        return false;

    commanded_fingers[finger_name] = true;

    return true;
}

void HandControlCommand::resetCommandedFingers()
{
    for (const std::string &finger : available_fingers)
        commanded_fingers[finger] = false;
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

void HandControlCommand::requestFingersApproachStatus()
{
    cmd = Command::ApproachStatus;
}

void HandControlCommand::requestFingersRestoreStatus()
{
    cmd = Command::RestoreStatus;
}

void HandControlCommand::probeContacts()
{
    cmd = Command::ProbeContacts;
}

void HandControlCommand::clear()
{
    commanded_hand.clear();
    resetCommandedFingers();
    cmd = Command::Empty;
    is_linear_forward_speed_set = false;
    is_joint_restore_speed_set = false;
}

std::string HandControlCommand::getCommandedHand() const
{
    return commanded_hand;
}

void HandControlCommand::getCommandedFingers(std::vector<std::string> &fingers) const
{
    fingers.clear();

    for (const std::string &finger : available_fingers)
        if(commanded_fingers.at(finger))
            fingers.push_back(finger);
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
    // clear the current object
    clear();

    // commanded hand
    int vocab_hand = connection.expectInt();
    if (vocab_hand == VOCAB4('R','I','G','H'))
        commanded_hand = "right";
    else if (vocab_hand == VOCAB4('L','E','F','T'))
        commanded_hand = "left";
    else
        return false;

    // commanded fingers
    for (const std::string &finger : available_fingers)
        commanded_fingers[finger] = connection.expectInt();

    // command
    cmd = static_cast<Command>(connection.expectInt());

    // speeds
    if (cmd == Command::Approach ||
        cmd == Command::Follow)
    {
        is_linear_forward_speed_set = true;
        linear_forward_speed = connection.expectDouble();
    }
    else if (cmd == Command::Restore)
    {
        is_joint_restore_speed_set = true;
        joint_restore_speed = connection.expectDouble();
    }

    return !connection.isError();
}

bool HandControlCommand::write(yarp::os::ConnectionWriter& connection)
{
    // commanded hand
    int vocab_hand;
    if (commanded_hand == "right")
        vocab_hand = VOCAB4('R','I','G','H');
    else if(commanded_hand == "left")
        vocab_hand = VOCAB4('L','E','F','T');
    else
        return false;
    connection.appendInt(vocab_hand);

    // commanded fingers
    for (const std::string &finger : available_fingers)
        connection.appendInt(commanded_fingers[finger]);

    // command
    connection.appendInt(static_cast<int>(cmd));

    // forward speed
    if (cmd == Command::Approach ||
        cmd == Command::Follow)
    {
        if (!is_linear_forward_speed_set)
            return false;
        else
            connection.appendDouble(linear_forward_speed);
    }
    else if (cmd == Command::Restore)
    {
        if (!is_joint_restore_speed_set)
            return false;
        else
            connection.appendDouble(joint_restore_speed);
    }

    return !connection.isError();
}

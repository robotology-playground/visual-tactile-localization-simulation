/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef HAND_CONTROL_COMMAND_H
#define HAND_CONTROL_COMMAND_H

// yarp
#include <yarp/os/Portable.h>

// std
#include <string>
#include <vector>
#include <set>
#include <unordered_map>

enum class Command { Empty = 0, Stop = 1, Approach = 2, Follow = 3, Restore = 4 };

class HandControlCommand : public yarp::os::Portable
{
private:
    /*
     * The string describing the commanded hand
     */
    std::string commanded_hand;

    /*
     * List of string containing the commanded fingers
     */
    std::unordered_map<std::string, bool> commanded_fingers;

    /*
     * The command requested to the controller
     */
    Command cmd;

    /*
     * The forward speed of the fingers
     */
    bool is_linear_forward_speed_set;
    double linear_forward_speed;

    /*
     * The joints velocity used in the restoring phase
     */
    bool is_joint_restore_speed_set;
    double joint_restore_speed;

    /*
     * List of available fingers
     */
    const std::set<std::string> available_fingers;

    /*
     * Set commanded finger
     * @param commanded_finger the name of the commanded finger
     * @return true/false on success/failure
     */
    bool setCommandedFinger(const std::string &finger_name);

    /*
     * Set all the fingers as not commanded
     */
    void resetCommandedFingers();

public:
    /*
     * Constructor
     */
    HandControlCommand();

    /*
     * Set commanded hand
     * @param commanded_hand the name of the commanded hand
     * @return true/false on success/failure
     */
    bool setCommandedHand(const std::string &hand_name);

    /*
     * Set commanded fingers
     * @param commanded_fingers the list of names of the commanded fingers
     * @return true/false on success/failure
     */
    bool setCommandedFingers(const std::vector<std::string> &fingers_names);

    /*
     * Set linear forward speed of fingers during approach/hold phases
     * The same speed is used for all the fingers
     * @param speed the value of the positive forward speed in m/s
     * @return true/false on success/failure
     */
    bool setFingersForwardSpeed(const double &speed);

    /*
     * Set the joints speed used restore phase.
     * The same speed is used for all the fingers
     * @param speed the value joint speed during restore in deg/s
     * @return true/false on success/failure
     */
    bool setFingersRestoreSpeed(const double &speed);

    /*
     * Request an approach phase in which the fingers move until contact
     * is detected for all of them
     */
    void commandFingersApproach();

    /*
     * Request activation of the controller that move fingers
     * in order to maintain contact with the object
     */
    void commandFingersFollow();

    /*
     * Request the restore of the initial configuration of the fingers
     */
    void commandFingersRestore();

    /*
     * Request the immediate stop of any movement of the fingers
     */
    void commandStop();

    /*
     * Clear the command
     */
    void clear();

    /*
     * Get the commanded hand
     */
    std::string getCommandedHand() const;

    /*
     * Get the commanded fingers
     */
    const std::unordered_map<std::string, bool>& getCommandedFingers() const;

    /*
     * Get the requested linear forward speed of the fingers
     */
    bool getForwardSpeed(double &speed) const;

    /*
     * Get the requested joint speed during restore phase
     */
    bool getRestoreSpeed(double &speed) const;

    /*
     * Get the requested command
     */
    Command getCommand() const;

    /*
     * Return true iff a HandControlCommand was received succesfully
     */
    bool read(yarp::os::ConnectionReader& connection) YARP_OVERRIDE;

    /*
     * Return true iff a HandControlCommand was sent succesfully
     */
    bool write(yarp::os::ConnectionWriter& connection) YARP_OVERRIDE;
};
#endif

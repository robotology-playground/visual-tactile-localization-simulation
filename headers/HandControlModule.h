/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#ifndef HAND_CONTROL_MODULE_H
#define HAND_CONTROL_MODULE_H

// yarp
#include <yarp/os/RFModule.h>

// icub-main
#include <iCub/skinDynLib/skinContactList.h>

// std
#include <string>

#include "headers/HandController.h"
#include "headers/HandControlCommand.h"

class HandControlModule : public yarp::os::RFModule
{
private:
    // hand controllers
    HandController hand;

    // name of the hand to be controlled
    std::string hand_name;

    // contact points port and storage
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> port_contacts;
    std::string port_contacts_name;

    // command port
    yarp::os::BufferedPort<HandControlCommand> port_cmd;
    std::string port_cmd_name;

    // current command
    Command current_command;

    // linear forward speed
    // to be used for commands Approach and Follow
    double linear_forward_speed;

    // joint restore speed
    // to be used for commands Restore
    double joint_restore_speed;

    // list of currently commanded fingers
    std::vector<std::string> commanded_fingers;
    
    // period
    double period;

   /*
    * Return the number of contacts detected for each finger tip
    * as a std::unorderd_map.
    * The key is the name of the finger, i.e. 'thumb', 'index',
    * 'middle', 'ring' or 'little'.
    */
    bool getNumberContacts(iCub::skinDynLib::skinContactList &skin_contact_list,
			   std::unordered_map<std::string, int> &number_contacts);
   /*
    * Process a command
    */
    void processCommand(HandControlCommand cmd);

   /*
    * Perform control according to the current command
    */
    void performControl();

   /*
    * Stop any ongoing finger movements
    */
    void stopControl();

public:
    /*
     * Configure the module.
     * @param rf a previously instantiated @see ResourceFinder
     */
    bool configure(yarp::os::ResourceFinder &rf) override;

    /*
     * Return the module period.
     */
    double getPeriod() override;

    /*
     * Define the behavior of this module.
     */
    bool updateModule() override;

    /*
     * Define the cleanup behavior.
     */
    bool close() override;
};

#endif

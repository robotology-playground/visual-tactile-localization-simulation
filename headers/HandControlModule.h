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
#include <yarp/os/RpcServer.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/PortReader.h>
#include <yarp/os/ConnectionReader.h>

// icub-main
#include <iCub/skinDynLib/skinContactList.h>

// std
#include <string>

#include "headers/HandController.h"
#include "headers/HandControlCommand.h"
#include "headers/HandControlResponse.h"

class HandControlModule : public yarp::os::RFModule, public yarp::os::PortReader
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
    std::string port_rpc_name;

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

    // status
    bool is_approach_done;
    bool is_restore_done;

    // rpc server
    yarp::os::RpcServer rpc_server;

    // mutex required to share data between
    // the RFModule thread and the PortReader callback
    yarp::os::Mutex mutex;

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
    void processCommand(const HandControlCommand &cmd,
			HandControlResponse &response);
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

    /*
     * Overriden read method of base class yarp::os::PortReader
     */
    bool read(yarp::os::ConnectionReader& connection) override;
};

#endif

/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga
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
#include <iCub/perception/springyFingers.h>

// std
#include <string>

#include "HandController.h"
#include "HandControlCommand.h"
#include "HandControlResponse.h"

class HandControlModule : public yarp::os::RFModule, public yarp::os::PortReader
{
private:
    // hand controllers
    HandController hand;

    // name of the hand to be controlled
    std::string hand_name;

    // contact points port and storage
    // this is used in simulation (GazeboYarpSkin plugin)
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> port_contacts_sim;
    // this is used with the real robot
    yarp::os::BufferedPort<yarp::sig::Vector> port_contacts;
    std::string port_contacts_name;
    bool use_simulated_contacts;

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

    // flags
    bool use_tactile_contacts;
    bool use_springy_contacts;

    // springy fingers for contact detection
    iCub::perception::SpringyFingersModel springy_fingers;
    yarp::sig::Vector springy_thres;

    // rpc server
    yarp::os::RpcServer rpc_server;

    // mutex required to share data between
    // the RFModule thread and the PortReader callback
    yarp::os::Mutex mutex;

   /*
    * Return the status of contacts detected for each finger tip
    * as a std::unorderd_map.
    *
    * This is to be used in simulation.
    *
    * The key is the name of the finger, i.e. 'thumb', 'index',
    * 'middle', 'ring' or 'little'.
    */
    void getContactsSim(std::unordered_map<std::string, bool> &fingers_contacts);
    void getContacts(std::unordered_map<std::string, bool> &fingers_contacts);
    void getContactsSpringy(std::unordered_map<std::string, bool> &fingers_contacts);

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

    bool loadListDouble(yarp::os::ResourceFinder &rf,
                        const std::string &key,
                        const int &size,
                        yarp::sig::Vector &list);
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

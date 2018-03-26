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

enum class Action { idle, open, approach };
enum class ApproachMode { once, continuous };

class HandControlModule : public yarp::os::RFModule
{
private:
    // hand controllers
    RightHandController right_hand;
    LeftHandController left_hand;

    // contact points port and storage
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> port_contacts;
    iCub::skinDynLib::skinContactList skin_contact_list;
    std::string port_contacts_name;

    // command port
    std::string port_cmd_name;
    
    // period
    double period;

    // flags
    bool are_contacts_available;

   /*
    * Return the number of contacts detected for each finger tip
    * for the specified hand as a std::map.
    * The key is the name of the finger, i.e. 'thumb', 'index',
    * 'middle', 'ring' or 'pinky'.
    */
    bool getNumberContacts(const std::string &which_hand,
			   std::unordered_map<std::string, int> &numberContacts);

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

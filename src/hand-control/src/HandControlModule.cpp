/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga
 */

// yarp
#include <yarp/os/ConnectionWriter.h>

#include "HandControlModule.h"

typedef std::map<iCub::skinDynLib::SkinPart, iCub::skinDynLib::skinContactList> skinPartMap;

void HandControlModule::getContactsSim(std::unordered_map<std::string, bool> &fingers_contacts)
{
    iCub::skinDynLib::skinContactList* skin_contact_list;
    iCub::skinDynLib::skinContactList empty_list;
    skin_contact_list = port_contacts_sim.read(false);
    if (skin_contact_list == YARP_NULLPTR)
        skin_contact_list = &empty_list;

    if (skin_contact_list->size() != 0)
    {
        // split contacts per SkinPart
        skinPartMap map = skin_contact_list->splitPerSkinPart();

        // take the right skinPart
        iCub::skinDynLib::SkinPart skinPart;
        if (hand_name == "right")
            skinPart = iCub::skinDynLib::SkinPart::SKIN_RIGHT_HAND;
        else
            skinPart = iCub::skinDynLib::SkinPart::SKIN_LEFT_HAND;

        // count contacts coming from finger tips only
        iCub::skinDynLib::skinContactList &list = map[skinPart];
        for (size_t i=0; i<list.size(); i++)
        {
            // need to verify if this contact was effectively produced
            // by taxels on the finger tips
            // in order to simplify things the Gazebo plugin only sends one
            // taxel id that is used to identify which finger is in contact
            std::vector<unsigned int> taxels_ids = list[i].getTaxelList();
            unsigned int taxel_id = taxels_ids[0];
            // taxels ids for finger tips are between 0 and 59
            if (taxel_id >= 0 && taxel_id < 12)
                fingers_contacts["index"] = true;
            else if (taxel_id >= 12 && taxel_id < 24)
                fingers_contacts["middle"] = true;
            else if (taxel_id >= 24 && taxel_id < 36)
                fingers_contacts["ring"] = true;
            else if (taxel_id >= 36 && taxel_id < 48)
                fingers_contacts["little"] = true;
            else if (taxel_id >= 48 && taxel_id < 60)
                fingers_contacts["thumb"] = true;
        }
    }
}

void HandControlModule::getContacts(std::unordered_map<std::string, bool> &fingers_contacts)
{
    // try to read skin data from the port
    yarp::sig::Vector *skin_data = port_contacts.read(false);
    if (skin_data == NULL)
	return;

    // size should be 192 for hand data
    if (skin_data->size() != 192)
	return;

    // finger tips taxels are in the range 0-59
    double thr = 0;
    for (size_t i=0; i<60; i++)
    {
	if ((*skin_data)[i] > thr)
	{
	    if (i >=0 && i < 12)
		fingers_contacts["index"] = true;
	    else if (i >= 12 && i < 24)
		fingers_contacts["middle"] = true;
	    else if (i >= 24 && i < 36)
		fingers_contacts["ring"] = true;
	    else if (i >= 36 && i < 48)
		fingers_contacts["little"] = true;
	    else if (i >= 48)
                fingers_contacts["thumb"] = true;
	}
    }
}

void HandControlModule::getContactsSpringy(std::unordered_map<std::string, bool> &fingers_contacts)
{
    // get the correct hand
    yarp::os::Value springy_output;
    yarp::os::Bottle *list;
    springy_fingers.getOutput(springy_output);
    list = springy_output.asList();

    for (size_t i=0; i<5; i++)
    {
        if (list->get(i).asDouble() > springy_thres[i])
        {
            if (i == 0)
                fingers_contacts["thumb"] = true;
            else if (i == 1)
                fingers_contacts["index"] = true;
            else if (i == 2)
                fingers_contacts["middle"] = true;
            else if (i == 3)
                fingers_contacts["ring"] = true;
            else if (i == 4)
                fingers_contacts["little"] = true;
        }
    }
}

void HandControlModule::processCommand(const HandControlCommand &cmd,
                                       HandControlResponse &response)
{
    // set default response
    response.clear();

    // extract command value
    Command command = cmd.getCommand();

    // check if the command is for this hand
    // (it should not happen)
    if (cmd.getCommandedHand() != hand_name)
    {
        // ignore this command
        return;
    }

    if (command == Command::Empty)
    {
        // nothing to do here
        return;
    }

    // perform actions common to
    // several commands
    if (command == Command::Approach ||
        command == Command::Follow ||
        command == Command::Restore ||
        command == Command::Stop ||
        command == Command::ProbeContacts ||
        command == Command::Idle ||
        command == Command::SwitchToPositionControl)
    {
        // change the current command
        current_command = command;

        // get commanded fingers
        cmd.getCommandedFingers(commanded_fingers);
    }

    switch(command)
    {

    case Command::ApproachStatus:
    {
        // set the status in the response
        response.setIsApproachDone(is_approach_done);

        break;
    }

    case Command::RestoreStatus:
    {
        // set the status in the response
        response.setIsRestoreDone(is_restore_done);

        break;
    }

    case Command::Approach:
    case Command::Follow:
    {
        // get requested speeds
        cmd.getForwardSpeed(linear_forward_speed);

        // remove pending contact points
        // from last session
        if (use_simulated_contacts)
        {
            while (port_contacts_sim.getPendingReads() > 0)
                port_contacts_sim.read(false);
        }
        else
        {
            while (port_contacts.getPendingReads() > 0)
                port_contacts.read(false);
        }

        if (command == Command::Approach)
        {
            // reset detected contacts within the
            // hand controller
            hand.resetFingersContacts();

            // reset status
            is_approach_done = false;
        }

        break;
    }

    case Command::Restore:
    {
        // get requested joints speeds
        cmd.getRestoreSpeed(joint_restore_speed);

        // reset status
        is_restore_done = false;

        break;
    }
    }
}

void HandControlModule::performControl()
{
    // get command safely
    mutex.lock();
    Command cmd = current_command;
    mutex.unlock();

    // switch according to the current command
    switch(cmd)
    {
    case Command::Empty:
    case Command::Idle:
    {
        // nothing to do here
        break;
    }

    case Command::ProbeContacts:
    {
        std::unordered_map<std::string, bool> fingers_contacts;
        fingers_contacts["index"] = false;
        fingers_contacts["middle"] = false;
        fingers_contacts["ring"] = false;
        fingers_contacts["little"] = false;
        fingers_contacts["thumb"] = false;

        if (use_simulated_contacts)
            getContactsSim(fingers_contacts);
        else
        {
            if (use_tactile_contacts)
                getContacts(fingers_contacts);
            if (use_springy_contacts)
                getContactsSpringy(fingers_contacts);
        }

        for (auto it = fingers_contacts.begin();
             it != fingers_contacts.end(); it++)
        {
            const std::string &finger_name = it->first;
            const bool &is_contact = it->second;
            if (is_contact)
                yInfo() << hand_name << finger_name
                        << ": contact detected";
        }

        break;
    }

    case Command::SwitchToPositionControl:
    {
        hand.switchToPositionControl(commanded_fingers);

        mutex.lock();
        current_command = Command::Idle;
        mutex.unlock();
    }

    case Command::Approach:
    case Command::Follow:
    {
        // extract contact informations
        std::unordered_map<std::string, bool> fingers_contacts;
        fingers_contacts["index"] = false;
        fingers_contacts["middle"] = false;
        fingers_contacts["ring"] = false;
        fingers_contacts["little"] = false;
        fingers_contacts["thumb"] = false;
        if (use_simulated_contacts)
            getContactsSim(fingers_contacts);
        else
        {
            if (use_tactile_contacts)
                getContacts(fingers_contacts);
            if (use_springy_contacts)
                getContactsSpringy(fingers_contacts);
        }

        // command fingers
        bool done = false;
        bool ok = false;
        if (cmd == Command::Approach)
        {
            ok = hand.moveFingersUntilContact(commanded_fingers,
                                              linear_forward_speed,
                                              fingers_contacts,
                                              done);
        }
        else if (cmd == Command::Follow)
        {
            ok = hand.moveFingersMaintainingContact(commanded_fingers,
                                                    linear_forward_speed,
                                                    fingers_contacts);
        }

        if (!ok)
        {
            // something went wrong
            // stop finger movements
            stopControl();

            // go in Idle
            mutex.lock();
            current_command = Command::Idle;
            mutex.unlock();

            return;
        }

        // in case of Approach
        // check if contact was reached for all the fingers
        if (cmd == Command::Approach)
        {
            if (done)
            {
                mutex.lock();

                // approach phase completed
                // go in Idle
                current_command = Command::Idle;

                // update flag
                is_approach_done = true;

                mutex.unlock();
            }
        }

        break;
    }

    case Command::Restore:
    {
        // issue finger restore command
        hand.restoreFingersPosition(commanded_fingers,
                                    joint_restore_speed);

        // go in WaitRestoreDone
        mutex.lock();
        current_command = Command::WaitRestoreDone;
        mutex.unlock();

        break;
    }

    case Command::WaitRestoreDone:
    {
        bool is_done;
        bool ok;
        ok = hand.isFingersRestoreDone(commanded_fingers,
                                       is_done);

        if (!ok)
        {
            // something went wrong
            // stop finger movements
            stopControl();

            // go in Idle
            mutex.lock();
            current_command = Command::Idle;
            mutex.unlock();

            return;
        }

        mutex.lock();

        // update flag
        is_restore_done = is_done;

        // go in Idle when done
        if (is_done)
            current_command = Command::Idle;

        mutex.unlock();

        break;
    }

    case Command::Stop:
    {
        stopControl();

        // go in Idle
        mutex.lock();
        current_command = Command::Idle;
        mutex.unlock();

        break;
    }
    }
}

void HandControlModule::stopControl()
{
    // stop any ongoing movement
    hand.stopFingers(commanded_fingers);
}

bool HandControlModule::loadListDouble(yarp::os::ResourceFinder &rf,
                                       const std::string &key,
                                       const int &size,
                                       yarp::sig::Vector &list)
{
    if (rf.find(key).isNull())
        return false;

    yarp::os::Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        return false;

    if (b->size() != size)
        return false;

    list.resize(size);
    for (size_t i=0; i<b->size(); i++)
    {
        yarp::os::Value item_v = b->get(i);
        if (item_v.isNull())
            return false;

        if (!item_v.isDouble())
        {
            list.clear();
            return false;
        }

        list[i] = item_v.asDouble();
    }
    return true;
}

bool HandControlModule::configure(yarp::os::ResourceFinder &rf)
{
    // get the name of the hand to be controlled
    hand_name = rf.find("handName").asString();
    if (rf.find("handName").isNull())
    {
        yError() << "HandControlModule::configure"
                 << "Error: cannot find parameter 'handName'"
                 << "in current configuration";
        return false;
    }

    // get the robot name
    std::string robot_name = rf.find("robotName").asString();
    if (rf.find("robotName").isNull())
        robot_name = "icub";

    // get use_simulated_contacts flag
    use_simulated_contacts = rf.find("useSimulatedContacts").asBool();
    if (rf.find("useSimulatedContacts").isNull())
        use_simulated_contacts = false;

    yarp::os::ResourceFinder inner_rf;
    inner_rf = rf.findNestedResourceFinder(hand_name.c_str());

    // get the period
    period = inner_rf.find("period").asDouble();
    if (inner_rf.find("period").isNull())
        period = 0.03;
    yInfo() << "HandControlModule: period is" << period;

    // get the name of the contact points port
    port_contacts_name = inner_rf.find("contactsInputPort").asString();
    if (inner_rf.find("contactsInputPort").isNull())
        port_contacts_name = "/hand-control/" + hand_name + "/contacts:i";
    yInfo() << "HandControlModule: contact points input port name is" << port_contacts_name;

    // get the name of rpc port
    port_rpc_name = inner_rf.find("rpcPort").asString();
    if (inner_rf.find("rpcPort").isNull())
        port_rpc_name = "/hand-control/" + hand_name + "/rpc:i";
    yInfo() << "HandControlModule: rpc port name is" << port_rpc_name;

    // check whether additional encoders are required
    bool use_analogs = inner_rf.find("useAnalogs").asBool();
    if(inner_rf.find("useAnalogs").isNull())
        use_analogs = false;

    // load parameters about contact detection
    use_tactile_contacts = inner_rf.find("useTactileContacts").asBool();
    if (inner_rf.find("useTactileContacts").isNull())
        use_tactile_contacts = true;
    use_springy_contacts = inner_rf.find("useSpringyContacts").asBool();
    if (inner_rf.find("useSpringyContacts").isNull())
        use_springy_contacts = false;
    if (use_springy_contacts)
    {
        if (!loadListDouble(inner_rf, "springyFingersThres",
                            5, springy_thres))
        {
            yError() << "HandControlModule: unable to load threshold for contact detetion"
                     << "with" << hand_name << "springy fingers";
            return false;
        }
    }

    bool ok;

    // open the contact points port
    if (use_simulated_contacts)
    {
        ok = port_contacts_sim.open(port_contacts_name);
        if (!ok)
        {
            yError() << "HandControlModule::configure"
                     << "Error: unable to open the simulated contacts port";
            return false;
        }
    }
    else
    {
        ok = port_contacts.open(port_contacts_name);
        if (!ok)
        {
            yError() << "HandControlModule::configure"
                     << "Error: unable to open the contacts port";
            return false;
        }

    }

    // configure springy fingers
    if (use_springy_contacts)
    {
        std::string springy_calib_path;
        if (!inner_rf.check("springyFingersCalib"))
        {
            yError() << "HandControlModule::configure"
                     << "Error: cannot load path containing the calibration file"
                     << "for" << hand_name << "springy fingers";
            return false;
        }
        springy_calib_path = inner_rf.findFile("springyFingersCalib");
        yarp::os::Property springy_prop;
        springy_prop.fromConfigFile(springy_calib_path.c_str());
        springy_prop.put("robot", robot_name.c_str());
        springy_prop.put("name", "vtl-hand-ctrl/" + hand_name + "/springy");
        springy_fingers.fromProperty(springy_prop);

        if (!springy_fingers.isCalibrated())
        {
            yError() << "HandControlModule::configure"
                     << "Error: cannot configure" << hand_name << "springy fingers";
            return false;
        }
    }

    // open the rpc server port
    ok = rpc_server.open(port_rpc_name);
    if (!ok)
    {
        yError() << "HandControlModule::configure"
                 << "Error: unable to open the rpc port";
        return false;
    }

    // configure callback for rpc
    rpc_server.setReader(*this);

    // configure hand the hand controller
    ok = hand.configure(rf, robot_name, hand_name, use_analogs);
    if (!ok)
    {
        yError() << "HandControlModule::configure"
                 << "Error: unable to configure the"
                 << hand_name
                 << "hand controller";
        return false;
    }

    // reset current command
    current_command = Command::Idle;

    // reset flags
    is_approach_done = false;
    is_restore_done = false;

    return true;
}

double HandControlModule::getPeriod()
{
    return period;
}

bool HandControlModule::updateModule()
{
    performControl();

    return true;
}

bool HandControlModule::close()
{
    // stop all movements for safety
    stopControl();

    // close hand controller
    hand.close();

    // close ports
    if (use_simulated_contacts)
        port_contacts_sim.close();
    else
        port_contacts.close();
    rpc_server.close();
}

bool HandControlModule::read(yarp::os::ConnectionReader& connection)
{
    // get command from the connection
    HandControlCommand hand_cmd;
    bool ok = hand_cmd.read(connection);
    if (!ok)
    {
        yError() << "HandControlModule::read"
                 << "Error: unable to read the hand control command"
                 << "from the incoming connection";
        return false;
    }

    // process the received commands
    HandControlResponse response;

    mutex.lock();

    processCommand(hand_cmd, response);

    mutex.unlock();

    // sends the response back
    yarp::os::ConnectionWriter* to_sender = connection.getWriter();
    if (to_sender == NULL)
    {
        yError() << "HandControlModule::read"
                 << "Error: unable to get a ConnectionWriter from the"
                 << "incoming connection";

        return false;
    }
    response.write(*to_sender);

    return true;
}

int main(int argc, char **argv)
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << "HandControlModule: cannot find YARP!";
        return 1;
    }

    // instantiate the resource finder
    yarp::os::ResourceFinder rf;
    rf.setDefaultConfigFile("hand-control_config.ini");
    rf.configure(argc,argv);

    // instantiate the localizer
    HandControlModule controller;

    // run the controller
    controller.runModule(rf);
}

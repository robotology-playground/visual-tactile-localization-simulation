/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

#include "headers/HandControlModule.h"

typedef std::map<iCub::skinDynLib::SkinPart, iCub::skinDynLib::skinContactList> skinPartMap;

bool HandControlModule::getNumberContacts(iCub::skinDynLib::skinContactList &skin_contact_list,
					  std::unordered_map<std::string, int> &number_contacts)
{
    // clear number of contacts for each finger
    int n_thumb = 0;
    int n_index = 0;
    int n_middle = 0;
    int n_ring = 0;
    int n_little = 0;

    if (skin_contact_list.size() != 0)
    {
	// split contacts per SkinPart
	skinPartMap map = skin_contact_list.splitPerSkinPart();

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
		n_index++;
	    else if (taxel_id >= 12 && taxel_id < 24)
		n_middle++;
	    else if (taxel_id >= 24 && taxel_id < 36)
		n_ring++;
	    else if (taxel_id >= 36 && taxel_id < 48)
		n_little++;
	    else if (taxel_id >= 48 && taxel_id < 60)
		n_thumb++;
	}
    }

    number_contacts["thumb"] = n_thumb;
    number_contacts["index"] = n_index;
    number_contacts["middle"] = n_middle;
    number_contacts["ring"] = n_ring;
    number_contacts["little"] = n_little;

    return true;
}

void HandControlModule::processCommand(HandControlCommand cmd)
{
    // extract command value
    current_command = cmd.getCommand();

    if (current_command == Command::Empty ||
	current_command == Command::Idle)
    {
	// nothing to do here
	return;
    }

    // check if the command is for this hand
    // (it should not happen)
    if (cmd.getCommandedHand() != hand_name)
    {
	// ignore this command
	return;
    }

    // get requested speeds if required
    if (current_command == Command::Approach ||
	current_command == Command::Follow)
    {
	cmd.getForwardSpeed(linear_forward_speed);

	if (current_command == Command::Approach)
	{
	    // reset detected contacts within the
	    // hand controller
	    hand.resetFingersContacts();

	    // remove pending contact points
	    // from last session
	    while (port_contacts.getPendingReads() > 0)
		port_contacts.read(false);
	}
    }
    else if(current_command == Command::Restore)
    {
	cmd.getRestoreSpeed(joint_restore_speed);
    }

    // get commanded fingers
    cmd.getCommandedFingers(commanded_fingers);
}

void HandControlModule::performControl()
{
    // switch according to the current command
    switch(current_command)
    {
    case Command::Empty:
    case Command::Idle:
    {
	// nothing to do here
	break;
    }

    case Command::Approach:
    case Command::Follow:
    {
	iCub::skinDynLib::skinContactList* list_ptr;
	iCub::skinDynLib::skinContactList empty_list;

	// read from contacts port
	list_ptr = port_contacts.read(false);
	if (list_ptr == YARP_NULLPTR)
	    list_ptr = &empty_list;

	// extract contact informations
	std::unordered_map<std::string, int> number_contacts;
	getNumberContacts(*list_ptr, number_contacts);

	// command fingers
	bool done = false;
	bool ok = false;
	if (current_command == Command::Approach)
	{
	    ok = hand.moveFingersUntilContact(commanded_fingers,
					      linear_forward_speed,
					      number_contacts,
					      done);
	}
	else if (current_command == Command::Follow)
	{
	    ok = hand.moveFingersMaintainingContact(commanded_fingers,
						    linear_forward_speed,
						    number_contacts);
	}

	if (!ok)
	{
	    // something went wrong
	    // stop finger movements
	    stopControl();

	    // go in Idle
	    current_command = Command::Idle;

	    return;
	}

	// in case of Approach
	// check if contact was reached for all the fingers
	if (current_command == Command::Approach)
	{
	    if (done)
	    {
		// approach phase completed
		// go in Idle
		current_command = Command::Idle;
	    }
	}

	break;
    }

    case Command::Restore:
    {
	hand.restoreFingersPosition(commanded_fingers,
				    joint_restore_speed);

	// go in Idle then
	current_command = Command::Idle;
	break;
    }

    case Command::Stop:
    {
	stopControl();
	break;
    }
    }
}

void HandControlModule::stopControl()
{
    // stop any ongoing movement
    hand.stopFingers(commanded_fingers);
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

    // get the name of input port
    port_cmd_name = inner_rf.find("inputPort").asString();
    if (inner_rf.find("inputPort").isNull())
	port_cmd_name = "/hand-control/" + hand_name + ":i";
    yInfo() << "HandControlModule: input port name is" << port_cmd_name;
    
    // open the contact points port
    bool ok = port_contacts.open(port_contacts_name);
    if (!ok)
    {
	yError() << "HandControlModule::configure"
		 << "Error: unable to open the contacts port";
	return false;
    }

    // open the input command port
    ok = port_cmd.open(port_cmd_name);
    if (!ok)
    {
	yError() << "HandControlModule::configure"
		 << "Error: unable to open the input command port";
	return false;
    }
    // set FIFO policy
    port_cmd.setStrict();

    // configure hand the hand controller
    ok = hand.configure(hand_name);
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
 	
    return true;
}

double HandControlModule::getPeriod()
{
    return period;
}

bool HandControlModule::updateModule()
{
    // try to receive a command from the input port
    HandControlCommand *cmd = port_cmd.read(false);
    if (cmd != YARP_NULLPTR)
	processCommand(*cmd);

    performControl();

    return true;
}

bool HandControlModule::close()
{
    // stop all movements for safety
    stopControl();

    // close ports
    port_contacts.close();
    port_cmd.close();
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
    rf.setDefaultConfigFile("hand_control_module_config.ini");
    rf.configure(argc,argv);

    // instantiate the localizer
    HandControlModule controller;

    // run the controller
    controller.runModule(rf);
}

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
                                          const std::string &which_hand,
					  std::unordered_map<std::string, int> &numberContacts)
{
    // split contacts per SkinPart
    skinPartMap map = skin_contact_list.splitPerSkinPart();

    // take the right skinPart
    iCub::skinDynLib::SkinPart skinPart;
    if (hand_name == "right")
	skinPart = iCub::skinDynLib::SkinPart::SKIN_RIGHT_HAND;
    else
	skinPart = iCub::skinDynLib::SkinPart::SKIN_LEFT_HAND;

    // clear number of contacts for each finger
    int n_thumb = 0;
    int n_index = 0;
    int n_middle = 0;
    int n_ring = 0;
    int n_little = 0;

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

    numberContacts["thumb"] = n_thumb;
    numberContacts["index"] = n_index;
    numberContacts["middle"] = n_middle;
    numberContacts["ring"] = n_ring;
    numberContacts["little"] = n_little;

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

    // get the requested command
    current_command = cmd.getCommand();

    // get requested speeds if required
    if (current_command == Command::Approach ||
	current_command == Command::Follow)
    {
	cmd.getForwardSpeed(linear_forward_speed);
    }
    else if(current_command == Command::Restore)
    {
	cmd.getRestoreSpeed(joint_restore_speed);
    }

    // get commanded fingers
    commanded_fingers = cmd.getCommandedFingers();
}

void HandControlModule::performControl()
{
    // switch according to the current command
    switch(current_command)
    {
    case Command::Empty:
    case Command::Idle:
	// nothing to do here
	break;

    case Command::Stop:
	stopControl();
	break;
    }
}

void HandControlModule::stopControl()
{
    // stop any ongoing movement
    hand.stopFingers();
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

    // get the period
    period = rf.find("period").asDouble();
    if (rf.find("period").isNull())
	period = 0.03;
    yInfo() << "HandControlModule: period is" << period;
    
    // get the name of the contact points port
    port_contacts_name = rf.find("contactsInputPort").asString();
    if (rf.find("contactsInputPort").isNull())
	port_contacts_name = "/hand-control/contacts:i";
    yInfo() << "HandControlModule: contact points input port name is" << port_contacts_name;

    // get the name of input port
    port_cmd_name = rf.find("inputPort").asString();
    if (rf.find("inputPort").isNull())
	port_cmd_name = "/hand-control:i";
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
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    // instantiate the localizer
    HandControlModule controller;

    // run the controller
    controller.runModule(rf);
}

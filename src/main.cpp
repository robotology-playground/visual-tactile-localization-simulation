/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @author: Nicola Piga <nicolapiga@gmail.com>
 */

// std
#include <string>
#include <map>
#include <unordered_map>

// yarp os
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/LogStream.h>

// yarp sig
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

// yarp math
#include <yarp/math/Math.h>

// yarp dev
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>

#include <cmath>

#include "headers/filterCommand.h"
#include "headers/ArmController.h"
#include "headers/ModelHelper.h"

using namespace yarp::math;

class VisTacLocSimModule: public yarp::os::RFModule
{
protected:
    // rpc server
    yarp::os::RpcServer rpc_port;

    // arm controllers
    RightArmController right_arm;
    LeftArmController left_arm;

    // mutexes required to share data between
    // the RFModule thread and the rpc thread
    yarp::os::Mutex mutex;
    yarp::os::Mutex mutex_contacts;

    // filter port
    yarp::os::BufferedPort<yarp::sig::FilterCommand> port_filter;

    // FrameTransformClient to read published poses
    yarp::dev::PolyDriver drv_transform_client;
    yarp::dev::IFrameTransform* tf_client;

    // last estimate published by the filter
    yarp::sig::Matrix estimate;
    bool is_estimate_available;

    // model helper class
    ModelHelper mod_helper;

    /*
     * Wait approximately for n seconds.
     * @param seconds the number of seconds to wait for
     */
    void waitSeconds(const int &seconds)
    {
	double t0 = yarp::os::Time::now();
	while (yarp::os::Time::now() - t0 < seconds)
	    yarp::os::Time::delay(1.0);
    }

    /*
     * Request visual localization to the filtering algorithm
     */
    bool localizeObject()
    {
	yarp::sig::FilterCommand &filter_data = port_filter.prepare();

	// clear the storage
	filter_data.clear();

	// set the command
	filter_data.setCommand(VOCAB2('O','N'));

	// set the tag
	filter_data.setTag(VOCAB3('V','I','S'));

	// send data to the filter
	port_filter.writeStrict();

	return true;
    }

    bool approachObject(const std::string &which_hand)
    {
	bool ok;

    	if (!is_estimate_available)
    	    return false;

	ArmController* arm;
	// HandController* hand;
	if (which_hand == "right")
	    arm = &right_arm;
	else
	    arm = &left_arm;
	// change effector to the middle finger
	ok = arm->useFingerFrame("middle");
        if (!ok)
	    return false;

    	mutex.lock();

    	// copy the current estimate of the object
    	yarp::sig::Matrix estimate = this->estimate;

    	mutex.unlock();

	// set the estimate within the model helper
	mod_helper.setModelPose(estimate);

	// set the hand yaw attitude according
	// to the estimate
	double yaw = mod_helper.evalApproachYawAttitude();
	arm->setHandAttitude(yaw * 180 / M_PI, 15, -90);

	// set the approaching position according
	// to the estimate
	yarp::sig::Vector pos(3, 0.0);
	mod_helper.evalApproachPosition(pos);

        // request pose to the cartesian interface
        arm->goToPos(pos);

        // wait for motion completion
        arm->cartesian()->waitMotionDone(0.04, 10.0);

	// reset contacts detected
	// hand->resetFingersContacts();

	// mutex_contacts.lock();
	// skin_contact_list.clear();
	// mutex_contacts.unlock();

	// move thumb opposition
	// and index, middle and ring until contact
	double t0 = yarp::os::Time::now();
	bool done = false;
	std::unordered_map<std::string, int> number_contacts;
	std::vector<std::string> finger_list = {"thumb", "index", "middle", "ring"};
	while (!done && (yarp::os::Time::now() - t0 < 15.0))
	{
    	    // mutex_contacts.lock();

	    // getNumberContacts(which_hand, number_contacts);
	    // skin_contact_list.clear();

	    // mutex_contacts.unlock();
	    
	    // ok = hand->moveFingersUntilContact(finger_list,
	    // 				       0.005,
	    // 				       number_contacts,
	    // 				       done);
	    if (!ok)
		return false;

    	    yarp::os::Time::delay(0.01);
	}
	// in case the contact was not reached for all the fingers
	// stop them and abort
	// if (!done)
	// {
	//     hand->stopFingers();

	//     return false;
	// }

	return true;
    }

    /*
     * Pushes left/right.
     * During pushing the pose of the object is estimated.
     */
    bool pushObject(const std::string &which_hand)
    {
	bool ok;

    	if (!is_estimate_available)
    	    return false;

	ArmController* arm;
	// HandController* hand;
	if (which_hand == "right")
	{
	    arm = &right_arm;
	    // hand = &right_hand;
	}
	else
	{
	    arm = &left_arm;
	    // hand = &left_hand;
	}

        // change effector to the middle finger
	ok = arm->useFingerFrame("middle");
	if (!ok)
	    return false;

	// get the current position of the hand
	yarp::sig::Vector pos;
	yarp::sig::Vector att;
	arm->cartesian()->getPose(pos, att);

        // final pose 
	pos[0] += 0.20;

        // store the current context because we are going
        // to change the trajectory time
        int context_id;
        arm->cartesian()->storeContext(&context_id);

        // set trajectory time
	double duration = 4.0;
	double traj_time = 4.0;
        arm->cartesian()->setTrajTime(traj_time);

        // // request pose to the cartesian interface
        arm->goToPos(pos);

	// enable filtering
	yarp::sig::FilterCommand &filter_data = port_filter.prepare();
	filter_data.clear();
	filter_data.setCommand(VOCAB2('O','N'));
	filter_data.setTag(VOCAB3('T','A','C'));
	port_filter.writeStrict();

	// perform pushing
        double t0 = yarp::os::Time::now();
    	double dt = 0.03;
    	bool done = false;
	std::unordered_map<std::string, int> number_contacts;
	std::vector<std::string> finger_list = {"index", "middle", "ring"};
    	while (!done && (yarp::os::Time::now() - t0 < duration))
    	{
	    // mutex_contacts.lock();
	    // getNumberContacts(which_hand, number_contacts);
	    // mutex_contacts.unlock();

	    // hand->moveFingersMaintainingContact(finger_list,
	    // 					0.005,
	    // 					number_contacts);

    	    arm->cartesian()->checkMotionDone(&done);
	}

	// stop filtering
	filter_data = port_filter.prepare();
	filter_data.clear();
	filter_data.setCommand(VOCAB3('O','F','F'));
	port_filter.writeStrict();

	// hand->stopFingers();

        // restore the context
        arm->cartesian()->restoreContext(context_id);

    	return true;
    }

public:
    bool configure(yarp::os::ResourceFinder &rf)
    {
	// open the filter port
	// TODO: take name from config
	bool ok = port_filter.open("/vis_tac_localization/filter:o");
	if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to open the filter port";
            return false;
        }

	// prepare properties for the FrameTransformClient
	yarp::os::Property propTfClient;
	propTfClient.put("device", "transformClient");
	propTfClient.put("local", "/vis_tac_localization/transformClient");
	propTfClient.put("remote", "/transformServer");

	// try to open the driver
	ok = drv_transform_client.open(propTfClient);
	if (!ok)
	{
	    yError() << "VisTacLocSimModule: unable to open the FrameTransformClient driver.";
	    return false;
	}

	// try to retrieve the view
	ok = drv_transform_client.view(tf_client);
	if (!ok || tf_client == 0)
	{
	    yError() << "VisTacLocSimModule: unable to retrieve the FrameTransformClient view.";
	    return false;
	}

	// open the rpc server
	// TODO: take name from config
        rpc_port.open("/service");
        attach(rpc_port);

	// set default value of flags
	is_estimate_available = false;

	// configure arm controllers
	ok = right_arm.configure();
        if (!ok)
	{
            yError() << "VisTacLocSimModule: unable to configure the right arm controller";
            return false;
	}

	ok = left_arm.configure();
        if (!ok)
	{
            yError() << "VisTacLocSimModule: unable to configure the left arm controller";
            return false;
	}

	// set default hands orientation
	right_arm.setHandAttitude(0, 15, -90);
	left_arm.setHandAttitude(0, 15, 0);

	// configure model helper
	mod_helper.setModelDimensions(0.24, 0.17, 0.037);

        return true;
    }

    bool interruptModule()
    {
        return true;
    }

    bool close()
    {
	// close arm controllers
	right_arm.close();
	left_arm.close();

	// close ports
        rpc_port.close();
	port_filter.close();
    }

    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
    {
	bool ok;
        std::string cmd = command.get(0).asString();
        if (cmd == "help")
        {
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- home-right");
            reply.addString("- home-left");
            reply.addString("- localize");
	    reply.addString("- approach-right");
	    reply.addString("- push-right");
            reply.addString("- quit");
        }
	else if (cmd == "home-right")
	{
	    // ok = right_hand.restoreFingersPosition();

	    // waitSeconds(5);

	    if (ok)
		ok &= right_arm.goHome();
		
	    if (ok)
		reply.addString("Go home done for right arm.");
	    else
		reply.addString("Go home failed for right arm.");
	}
	else if (cmd == "home-left")
	{
	    // ok = left_hand.restoreFingersPosition();

	    // waitSeconds(5);

	    if (ok)
		ok &= left_arm.goHome();
		
	    if (ok)
		reply.addString("Go home done for left arm.");
	    else
		reply.addString("Go home failed for left arm.");
	}
	else if (cmd == "localize")
	{
	    if (localizeObject())
	    	reply.addString("Localization using vision done.");
	    else
	    	reply.addString("Localization using vision failed.");
	}
	else if (cmd == "approach-right")
	{
	    if (approachObject("right"))
	    	reply.addString("Approaching phase done.");
	    else
	    	reply.addString("Approaching phase failed.");
	}
	else if (cmd == "push-right")
	{
	    if (pushObject("right"))
	    	reply.addString("Pushing with right hand done.");
	    else
	    	reply.addString("Pushing with right hand failed.");
	}
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);

        return true;
    }

    double getPeriod()
    {
        return 0.01;
    }

    bool updateModule()
    {
	if(isStopping())
	    return false;

	mutex.lock();

	// get current estimate from the filter
	// TODO: get source and target from configuration file
	std::string source = "/iCub/frame";
	std::string target = "/box_alt/estimate/frame";
	is_estimate_available = tf_client->getTransform(target, source, estimate);

	mutex.unlock();
	
        return true;
    }
};

int main()
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    VisTacLocSimModule mod;
    yarp::os::ResourceFinder rf;
    return mod.runModule(rf);
}

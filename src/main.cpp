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
#include <yarp/os/RpcClient.h>

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
#include "headers/HandControlCommand.h"
#include "headers/HandControlResponse.h"

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

    // filter port
    yarp::os::BufferedPort<yarp::sig::FilterCommand> port_filter;

    // hand controller modules ports
    yarp::os::RpcClient port_hand_right;
    yarp::os::RpcClient port_hand_left;

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
     * Request visual localization to the filtering algorithm.
     * @return true/false on success/failure
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

    /*
     * Wait until fingers approaching phase is done
     * @param which_hand which hand to ask for
     * @param timeout maximum time to wait for
     * @return true for success, false for failure and/or timeout expired
     */
    bool waitFingersApproachDone(const std::string &which_hand,
				 const double &period = 0.1,
				 const double &timeout = 0.0)
    {
	HandControlCommand hand_cmd;
	HandControlResponse response;
	bool done = false;
	bool ok;

        // pick the correct hand
	yarp::os::RpcClient* hand_port;
	if (which_hand == "right")
	    hand_port = &port_hand_right;
	else
	    hand_port = &port_hand_left;

	// store initial time
	double t0 = yarp::os::Time::now();

	// check until done or timeout expires
	while(!done)
	{
	    if (timeout != 0)
	    {
		if (yarp::os::Time::now() - t0 > timeout)
		{
		    // return false in case of timeout expired
		    return false;
		}
	    }

	    // clear messages
	    hand_cmd.clear();
	    response.clear();

	    // request for status
	    hand_cmd.setCommandedHand(which_hand);
	    hand_cmd.requestFingersApproachStatus();
	    hand_port->write(hand_cmd, response);

	    // check status
	    ok = response.isApproachDone(done);
	    if (!ok)
	    {
		yError() << "VisTacLocSimModule::waitFingersApproachDone"
			 << "Error: unable to get the status of the fingers"
			 << "phase from the hand control module";
		return false;
	    }
	    yarp::os::Time::delay(period);
	}

	return true;
    }

    /*
     * Perform approaching phase.
     * @param which_hand which hand to use
     * @return true/false on success/failure
     */
    bool approachObject(const std::string &which_hand)
    {
	bool ok;

	// check if the estimate is available
    	if (!is_estimate_available)
    	    return false;

        // copy the current estimate of the object
	mutex.lock();
	yarp::sig::Matrix estimate = this->estimate;
	mutex.unlock();

	// evaluate the desired hand pose
	// according to the current estimate
	mod_helper.setModelPose(estimate);
	double yaw = mod_helper.evalApproachYawAttitude();
	yarp::sig::Vector pos(3, 0.0);
	mod_helper.evalApproachPosition(pos);

	// pick the correct arm and hand
	ArmController* arm;
	yarp::os::RpcClient* hand_port;
	if (which_hand == "right")
	{
	    arm = &right_arm;
	    hand_port = &port_hand_right;
	}
	else
	{
	    arm = &left_arm;
	    hand_port = &port_hand_left;
	}

	// change effector to the middle finger
	ok = arm->useFingerFrame("middle");
        if (!ok)
	    return false;

	// set desired attitude
	arm->setHandAttitude(yaw * 180 / M_PI, 15, -90);

        // request pose to the cartesian interface
        arm->goToPos(pos);

        // wait for motion completion
        arm->cartesian()->waitMotionDone(0.03, 5.0);

	// move fingers towards the object
	std::vector<std::string> finger_list = {"thumb", "index", "middle", "ring"};
	HandControlCommand hand_cmd;
	HandControlResponse response;
	hand_cmd.setCommandedHand(which_hand);
	hand_cmd.setCommandedFingers(finger_list);
	hand_cmd.setFingersForwardSpeed(0.009);
	hand_cmd.commandFingersApproach();
	hand_port->write(hand_cmd, response);

	// wait for completion
	ok = waitFingersApproachDone(which_hand);
	if (!ok)
	    return false;

	return true;
    }

    /*
     * Pushes the object towards the robot using one of the hands.
     * @param which_hand which hand to use
     * @return true/false con success/failure
     */
    bool pushObject(const std::string &which_hand)
    {
	bool ok;

	// add some checks to stop this action
	// in case approaching has not been done yet

	// pick the correct arm and hand
	ArmController* arm;
	yarp::os::RpcClient* hand_port;
	if (which_hand == "right")
	{
	    arm = &right_arm;
	    hand_port = &port_hand_right;
	}
	else
	{
	    arm = &left_arm;
	    hand_port = &port_hand_left;
	}

        // change effector to the middle finger
	ok = arm->useFingerFrame("middle");
	if (!ok)
	    return false;

	// get the current position of the hand
	yarp::sig::Vector pos;
	yarp::sig::Vector att;
	arm->cartesian()->getPose(pos, att);

        // final position
	// TODO: this should be evaluated within
	// the model helper taking into account the geometry of the shelf
	// and should be used to perform closed loop control
	pos[0] += 0.20;

        // store the current context because we are going
        // to change the trajectory time
        int context_id;
        arm->cartesian()->storeContext(&context_id);

        // set trajectory time
	double duration = 4.0;
	double traj_time = 4.0;
        arm->cartesian()->setTrajTime(traj_time);

        // request pose to the cartesian interface
        arm->goToPos(pos);

	// enable filtering
	yarp::sig::FilterCommand &filter_data = port_filter.prepare();
	filter_data.clear();
	filter_data.setCommand(VOCAB2('O','N'));
	filter_data.setTag(VOCAB3('T','A','C'));
	port_filter.writeStrict();

	// enable fingers movements towards the object
	std::vector<std::string> finger_list = {"index", "middle", "ring"};
	HandControlCommand hand_cmd;
	HandControlResponse response;
	hand_cmd.setCommandedHand(which_hand);
	hand_cmd.setCommandedFingers(finger_list);
	hand_cmd.setFingersForwardSpeed(0.005);
	hand_cmd.commandFingersFollow();
	hand_port->write(hand_cmd, response);

	// perform pushing
	arm->cartesian()->waitMotionDone(0.03, duration);

	// stop fingers
	hand_cmd.commandStop();
	hand_port->write(hand_cmd, response);

	// stop filtering
	filter_data = port_filter.prepare();
	filter_data.clear();
	filter_data.setCommand(VOCAB3('O','F','F'));
	port_filter.writeStrict();

        // restore the context
        arm->cartesian()->restoreContext(context_id);

    	return true;
    }

    /*
     * Wait until fingers approaching phase is done
     * @param which_hand which hand to ask for
     * @param timeout maximum time to wait for
     * @return true for success, false for failure and/or timeout expired
     */
    bool waitFingersRestoreDone(const std::string &which_hand,
				const double &period = 0.1,
				const double &timeout = 0.0)
    {
	HandControlCommand hand_cmd;
	HandControlResponse response;
	bool done = false;
	bool ok;

        // pick the correct hand
	yarp::os::RpcClient* hand_port;
	if (which_hand == "right")
	    hand_port = &port_hand_right;
	else
	    hand_port = &port_hand_left;

	// store initial time
	double t0 = yarp::os::Time::now();

	// check until done or timeout expires
	while(!done)
	{
	    if (timeout != 0)
	    {
		if (yarp::os::Time::now() - t0 > timeout)
		{
		    // return false in case of timeout expired
		    return false;
		}
	    }

	    // clear messages
	    hand_cmd.clear();
	    response.clear();

	    // request for status
	    hand_cmd.setCommandedHand(which_hand);
	    hand_cmd.requestFingersRestoreStatus();
	    hand_port->write(hand_cmd, response);

	    // check status
	    ok = response.isRestoreDone(done);
	    if (!ok)
	    {
		yError() << "VisTacLocSimModule::waitFingersApproachDone"
			 << "Error: unable to get the status of the fingers"
			 << "restore phase from the hand control module";
		return false;
	    }
	    yarp::os::Time::delay(period);
	}

	return true;
    }

    /*
     * Restore the initial configuration of the fingers
     * @param which_hand which hand to use
     */
    void restoreHand(const std::string &which_hand)
    {
	// pick the correct arm and hand
	yarp::os::RpcClient* hand_port;
	if (which_hand == "right")
	    hand_port = &port_hand_right;
	else
	    hand_port = &port_hand_left;

	// issue restore command
	std::vector<std::string> finger_list = {"thumb", "index", "middle", "ring"};
	HandControlCommand hand_cmd;
	HandControlResponse response;
	hand_cmd.clear();
	hand_cmd.setCommandedHand(which_hand);
	hand_cmd.setCommandedFingers(finger_list);
	hand_cmd.setFingersRestoreSpeed(15.0);
	hand_cmd.commandFingersRestore();
	hand_port->write(hand_cmd, response);
    }

public:
    bool configure(yarp::os::ResourceFinder &rf)
    {
	// open ports
	bool ok = port_filter.open("/vis_tac_localization/filter:o");
	if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to open the filter port";
            return false;
        }

	ok = port_hand_right.open("/vis_tac_localization/hand-control/right/rpc:o");
	if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to open the right hand control module port";
            return false;
        }

	ok = port_hand_left.open("/vis_tac_localization/hand-control/left/rpc:o");
	if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to open the left hand control module port";
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
	    restoreHand("right");

	    ok = waitFingersRestoreDone("right");
	    
	    ok &= right_arm.goHome();
		
	    if (ok)
		reply.addString("Go home done for right arm.");
	    else
		reply.addString("Go home failed for right arm.");
	}
	else if (cmd == "home-left")
	{
	    restoreHand("left");

	    ok = waitFingersRestoreDone("left");

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

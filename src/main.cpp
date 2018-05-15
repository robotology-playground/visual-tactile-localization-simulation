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
#include "headers/TrajectoryGenerator.h"
#include "headers/RotationTrajectoryGenerator.h"

using namespace yarp::math;

enum class Status { Idle,
	            Localize,
	            MoveLeftUpward,
	            ArmApproach, WaitArmApproachDone,
                    FingersApproach, WaitFingersApproachDone,
	            PrepareRotation, PerformRotation,
	            PreparePush, PerformPush,
	            ArmRestore, WaitArmRestoreDone,
	            FingersRestore, WaitFingersRestoreDone,
	            Stop };

class VisTacLocSimModule: public yarp::os::RFModule
{
protected:
    // arm controllers
    ArmController right_arm;
    ArmController left_arm;

    // hand controller modules ports
    yarp::os::RpcClient port_hand_right;
    yarp::os::RpcClient port_hand_left;

    // filter port
    yarp::os::BufferedPort<yarp::sig::FilterCommand> port_filter;

    // last estimate published by the filter
    yarp::sig::Matrix estimate;
    bool is_estimate_available;

    // FrameTransformClient to read published poses
    yarp::dev::PolyDriver drv_transform_client;
    yarp::dev::IFrameTransform* tf_client;

    // model helper class
    ModelHelper mod_helper;

    // trajectory generator
    TrajectoryGenerator traj_gen;
    RotationTrajectoryGenerator rot_traj_gen;

    // default trajectory length
    double trajectory_duration;

    // rpc server
    yarp::os::RpcServer rpc_port;

    // mutexes required to share data between
    // the RFModule thread and the rpc thread
    yarp::os::Mutex mutex;

    // status
    Status status;
    Status previous_status;
    std::string current_hand;
    bool is_approach_done;
    bool is_timer_started;
    bool approach_corner;

    // last time
    // required to implement timeouts
    double last_time;

    /*
     * Send command to the filtering algorithm.
     * @param enable whether to enable or disable the filtering
     * @param type the type of filtering,
     *        i.e. 'visual' or 'tactile'
     * @return true/false on success/failure
     */
    bool sendCommandToFilter(const bool& enable, const std::string &type = "")
    {
	if (enable && type.empty())
	    return false;

	yarp::sig::FilterCommand &filter_cmd = port_filter.prepare();

	// clear the storage
	filter_cmd.clear();

	// enable filtering
	if (enable)
	    filter_cmd.enableFiltering();
	else
	    filter_cmd.disableFiltering();

	// enable the correct type of filtering
	if (enable)
	{
	    if (type == "visual")
		filter_cmd.enableVisualFiltering();
	    else if (type == "tactile")
		filter_cmd.enableTactileFiltering();
	}

	// send command to the filter
	port_filter.writeStrict();

	return true;
    }

    /*
     * Get an arm controller.
     * @param which_arm the required arm controller
     * @return a pointer to the arm controller in case of success,
     *         a null pointer in case of failure
     */
    ArmController* getArmController(const std::string &which_arm)
    {
	if (which_arm == "right")
	    return &right_arm;
	else if (which_arm == "left")
	    return &left_arm;
	else
	    return nullptr;
    }

    /*
     * Get a port connected to the hand controller module.
     * @param which_hand the required hand control module
     * @return true/false on success/failure
     */
    yarp::os::RpcClient* getHandPort(const std::string &which_hand)
    {
	if (which_hand == "right")
	    return &port_hand_right;
	else if (which_hand == "left")
	    return &port_hand_left;
	else
	    return nullptr;
    }

    /*
     * Check if arm motion is done.
     * @param which_arm which arm to ask the status of the motion for
     * @param is_done whether the arm motion is done or not
     * @return true for success, false for failure
     */
    bool checkArmMotionDone(const std::string &which_arm,
			    bool &is_done)
    {
	ArmController *arm = getArmController(which_arm);

	if (arm != nullptr)
	    return arm->cartesian()->checkMotionDone(&is_done);
	else
	    return false;
    }

    /*
     * Check if approaching phase with fingers is done.
     * @param which_hand which hand to ask for
     * @param is_done whether the approach phase is done or not
     * @return true for success, false for failure
     */
    bool checkFingersMotionDone(const std::string &which_hand,
				const std::string &motion_type,
				bool &is_done)
    {
        // pick the correct hand
	yarp::os::RpcClient* hand_port = getHandPort(which_hand);
	if (hand_port == nullptr)
	    return false;

	HandControlCommand hand_cmd;
	HandControlResponse response;

	// clear messages
	hand_cmd.clear();
	response.clear();

	// request for status
	hand_cmd.setCommandedHand(which_hand);
	if (motion_type == "fingers_approach")
	    hand_cmd.requestFingersApproachStatus();
	else if (motion_type == "fingers_restore")
	    hand_cmd.requestFingersRestoreStatus();
	else
	    return false;
	hand_port->write(hand_cmd, response);

	// check status
	bool ok;
	if (motion_type == "fingers_approach")
	    ok = response.isApproachDone(is_done);
	else if (motion_type == "fingers_restore")
	    ok = response.isRestoreDone(is_done);
	if (!ok)
	{
	    yError() << "VisTacLocSimModule::checkFingersMotionDone"
		     << "Error: unable to get the status of the fingers"
		     << "motion from the hand control module";
	    return false;
	}

	return true;
    }

    /*
     * Move the left arm upward with respect to its current position.
     */
    bool moveLeftArmUpward()
    {
	// get the current position of the arm
	yarp::sig::Vector pos;
	yarp::sig::Vector att;
	left_arm.cartesian()->getPose(pos, att);

	// shift position upward
	pos[2] += 0.1;

	// issue command
	left_arm.cartesian()->goToPose(pos, att);
    }

    /*
     * Perform approaching phase with the specified arm.
     * @param which_arm which arm to use
     * @param approach_corner whether to approach the corner of the object
     * @return true/false on success/failure
     */
    bool approachObjectWithArm(const std::string &which_arm,
			       const bool &approach_corner)
    {
	bool ok;

	// check if the estimate is available
    	if (!is_estimate_available)
    	    return false;

	// evaluate the desired hand pose
	// according to the current estimate
	mod_helper.setModelPose(estimate);
	double yaw = mod_helper.evalApproachYawAttitude();
	yarp::sig::Vector pos(3, 0.0);
	if (!approach_corner)
	    mod_helper.evalApproachPosition(pos);
	else
	    mod_helper.evalApproachPosition(pos, "right");

	// pick the correct arm
	ArmController* arm = getArmController(which_arm);
	if (arm == nullptr)
	    return false;

        // remove a previously attached finger tip
        ok = arm->detachFingerTip();
        if (!ok)
            return false;

	// change effector to the middle finger
	ok = arm->attachFingerTip("middle");
        if (!ok)
	    return false;

	// set desired attitude
	arm->setHandAttitude(yaw * 180 / M_PI, 15, -90);

        // request pose to the cartesian interface
        arm->goToPos(pos);

	return true;
    }

    /*
     * Perform approaching phase with the fingers of the specified hand.
     * @param which_hand which hand to use
     * @return true/false on success/failure
     */
    bool approachObjectWithFingers(const std::string &which_hand)
    {
	// pick the correct hand
	yarp::os::RpcClient* hand_port = getHandPort(which_hand);
	if (hand_port == nullptr)
	    return false;

        // move fingers towards the object
	std::vector<std::string> finger_list = {"thumb", "index", "middle", "ring"};
	HandControlCommand hand_cmd;
	HandControlResponse response;
	hand_cmd.setCommandedHand(which_hand);
	hand_cmd.setCommandedFingers(finger_list);
	hand_cmd.setFingersForwardSpeed(0.009);
	hand_cmd.commandFingersApproach();
	hand_port->write(hand_cmd, response);

	return true;
    }

    /*
     * Enable fingers following mode.
     * @param which_arm hand arm to use
     * @return true/false con success/failure
     */
    bool enableFingersFollowing(const std::string &which_hand)
    {
	// pick the correct hand
	yarp::os::RpcClient* hand_port = getHandPort(which_hand);
	if (hand_port == nullptr)
	    return false;

        // enable fingers movements towards the object
	std::vector<std::string> finger_list = {"index", "middle", "ring"};
	HandControlCommand hand_cmd;
	HandControlResponse response;
	hand_cmd.setCommandedHand(which_hand);
	hand_cmd.setCommandedFingers(finger_list);
	hand_cmd.setFingersForwardSpeed(0.005);
	hand_cmd.commandFingersFollow();
	hand_port->write(hand_cmd, response);

	return true;
    }

    /*
     * Configure cartesian controller for pushing phase.
     * @param which_arm which arm to use
     * @return true/false con success/failure
     */
    bool preparePushObject(const std::string &which_arm)
    {
	bool ok;

	// pick the correct arm
	ArmController* arm = getArmController(which_arm);
	if (arm == nullptr)
	    return false;

        // remove a previously attached finger tip
        ok = arm->detachFingerTip();
        if (!ok)
            return false;

        // change effector to the middle finger
	ok = arm->attachFingerTip("middle");
	if (!ok)
	    return false;

	// get the current position of the finger
	yarp::sig::Vector pos;
	yarp::sig::Vector att;
	arm->cartesian()->getPose(pos, att);

	// set final position
	yarp::sig::Vector pos_f(3, 0.0);
	pos_f = pos;
	pos_f[0] += 0.17;

	// configure trajectory generator
	traj_gen.setInitialPosition(pos);
	traj_gen.setFinalPosition(pos_f);
	traj_gen.setDuration(trajectory_duration);
	traj_gen.init();

        // store the current context because we are going
        // to change the trajectory time
        arm->storeContext();

        // set trajectory time
	// which determines the responsiveness
	// of the cartesian controller
	double traj_time = 0.6;
        arm->cartesian()->setTrajTime(traj_time);

    	return true;
    }

    bool prepareRotateObject(const std::string &which_arm)
    {
	// check if the estimate is available
	if (!is_estimate_available)
	    return false;

	bool ok;

	// pick the correct arm
	ArmController* arm = getArmController(which_arm);
	if (arm == nullptr)
	    return false;

        // remove a previously attached finger tip
        ok = arm->detachFingerTip();
        if (!ok)
            return false;

        // change effector to the middle finger
	ok = arm->attachFingerTip("middle");
	if (!ok)
	    return false;

	// get the current position of the finger
	yarp::sig::Vector finger_pos;
	yarp::sig::Vector attitude;
	arm->cartesian()->getPose(finger_pos, attitude);

	// get the current estimate of the center of the object
	yarp::sig::Vector object_center = estimate.getCol(3).subVector(0, 2);

	// configure the trajectory generator
	rot_traj_gen.setYawRate(-20 * M_PI / 180);
	rot_traj_gen.setObjectCenter(object_center);
	rot_traj_gen.setPushingPoint(finger_pos);

        // store the current context because we are going
        // to change the trajectory time
        arm->storeContext();

        // set trajectory time
	// which determines the responsiveness
	// of the cartesian controller
	double traj_time = 0.6;
        arm->cartesian()->setTrajTime(traj_time);
    }

    bool setArmLinearVelocity(const std::string &which_arm,
			      const yarp::sig::Vector &velocity)
    {
	// pick the correct arm
	ArmController* arm = getArmController(which_arm);
	if (arm == nullptr)
	    return false;

	// compose null attitude velocity
	yarp::sig::Vector att_dot(4, 0.0);

	return arm->cartesian()->setTaskVelocities(velocity, att_dot);
    }

    /*
     * Restore the initial configuration of the specified arm.
     * @param which_arm which hand to use
     */
    bool restoreArm(const std::string &which_arm)
    {
	// pick the correct arm
	ArmController* arm = getArmController(which_arm);
	if (arm == nullptr)
	    return false;

	// issue restore command
	arm->goHome();

	return true;
    }

    /*
     * Restore the initial configuration of the fingers of the specified hand.
     * @param which_hand which hand to use
     */
    bool restoreFingers(const std::string &which_hand)
    {
	// pick the correct hand
	yarp::os::RpcClient* hand_port = getHandPort(which_hand);
	if (hand_port == nullptr)
	    return false;

	// issue restore command
	std::vector<std::string> finger_list = {"thumb", "index", "middle", "ring"};
	HandControlCommand hand_cmd;
	HandControlResponse response;
	hand_cmd.clear();
	hand_cmd.setCommandedHand(which_hand);
	hand_cmd.setCommandedFingers(finger_list);
	hand_cmd.setFingersRestoreSpeed(25.0);
	hand_cmd.commandFingersRestore();
	hand_port->write(hand_cmd, response);

	return true;
    }

    bool restoreArmControllerContext(const std::string &which_arm)
    {
	// pick the correct arm
	ArmController* arm = getArmController(which_arm);
	if (arm == nullptr)
	    return false;

	// restore the context
	arm->restoreContext();

	return true;
    }

    /*
     * Stop control of the specified arm.
     * @param which_arm which arm to stop
     * @return true/false on success/failure
     */
    bool stopArm(const std::string &which_arm)
    {
	// pick the correct arm
	ArmController* arm = getArmController(which_arm);
	if (arm == nullptr)
	    return false;

	return arm->cartesian()->stopControl();
    }

    /*
     * Stop control of the fingers of the specified hand.
     * @param which_hand which hand to stop
     * @return true/false on success/failure
     */
    bool stopFingers(const std::string &which_hand)
    {
	// pick the correct hand
	yarp::os::RpcClient* hand_port = getHandPort(which_hand);
	if (hand_port == nullptr)
	    return false;

	// stop all the fingers
	std::vector<std::string> finger_list = {"thumb", "index", "middle", "ring"};
	HandControlCommand hand_cmd;
	HandControlResponse response;
	hand_cmd.setCommandedHand(which_hand);
	hand_cmd.setCommandedFingers(finger_list);
	hand_cmd.commandStop();
	hand_port->write(hand_cmd, response);

	return true;
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
	propTfClient.put("device", "FrameTransformClient");
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

	// configure arm controllers
	ok = right_arm.configure("icubSim", "right");
        if (!ok)
	{
            yError() << "VisTacLocSimModule: unable to configure the right arm controller";
            return false;
	}

	ok = left_arm.configure("icubSim", "left");
        if (!ok)
	{
            yError() << "VisTacLocSimModule: unable to configure the left arm controller";
            return false;
	}

	// set default hands orientation
	right_arm.setHandAttitude(0, 15, 0);
	left_arm.setHandAttitude(0, 15, 0);

	// enable torso on the right arm only
	right_arm.enableTorso();

	// enable tracking mode on the left arm
	// left_arm.cartesian()->setTrackingMode(true);
	// left_arm.cartesian()->setTrajTime(0.5);

	// configure model helper
	mod_helper.setModelDimensions(0.24, 0.17, 0.037);

	// set default value of flags
	is_estimate_available = false;
	is_approach_done = false;
	is_timer_started = false;

	// set default trajectory duration
	trajectory_duration = 4.0;

	// set default status
	status = Status::Idle;
	previous_status = Status::Idle;
	current_hand.clear();

	// open the rpc server
	// TODO: take name from config
        rpc_port.open("/service");
        attach(rpc_port);

        return true;
    }

    bool close()
    {
	// stop control of arms
	stopArm("right");
	stopArm("left");

	// stop control of fingers
	stopFingers("right");
	stopFingers("left");

	// in case pushing was initiated
	// the previous context of the cartesian controller
	// has to be restored
	if (status == Status::PreparePush ||
	    status == Status::PerformPush)
	{
	    // restore arm controller context
	    // that was changed in pushObject(curr_hand)
	    restoreArmControllerContext(current_hand);
	}

	// close arm controllers
	right_arm.close();
	left_arm.close();

	// close ports
        rpc_port.close();
	port_filter.close();
    }

    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
    {
	mutex.lock();

        std::string cmd = command.get(0).asString();
        if (cmd == "help")
        {
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString("Available commands:");
	    reply.addString("- move-left-upward");
            reply.addString("- home-right");
            reply.addString("- localize");
	    reply.addString("- approach-corner-with-right");
	    reply.addString("- approach-with-right");
	    reply.addString("- push-with-right");
	    reply.addString("- rotate-with-right");
	    reply.addString("- stop");
            reply.addString("- quit");
        }
	else if (cmd == "move-left-upward")
	{
	    if (status != Status::Idle)
		reply.addString("Wait for completion of the current phase!");
	    else
	    {
		previous_status = status;
		status = Status::MoveLeftUpward;

		reply.addString("Command issued.");
	    }
	}
	else if (cmd == "home-right")
	{
	    if (status != Status::Idle)
		reply.addString("Wait for completion of the current phase!");
	    else
	    {
		previous_status = status;
		status = Status::FingersRestore;
		current_hand = "right";

		reply.addString("Home right issued.");
	    }
	}
	else if (cmd == "home-left")
	{
	    if (status != Status::Idle)
		reply.addString("Wait for completion of the current phase!");
	    else
	    {
		previous_status = status;
		status = Status::FingersRestore;
		current_hand = "left";

		reply.addString("Home left issued.");
	    }
	}
	else if (cmd == "localize")
	{
	    if (status != Status::Idle)
		reply.addString("Wait for completion of the current phase!");
	    else
	    {
		previous_status = status;
		status = Status::Localize;

		reply.addString("Localization issued.");
	    }
	}
	else if (cmd == "approach-with-right"
		 || cmd == "approach-corner-with-right")
	{
	    if (status != Status::Idle)
		reply.addString("Wait for completion of the current phase!");
	    else
	    {
		previous_status = status;
		status = Status::ArmApproach;
		current_hand = "right";

		if (cmd == "approach-corner-with-right")
		    approach_corner = true;

		reply.addString("Approach with right-arm issued.");
	    }
	}
	else if (cmd == "push-with-right")
	{
	    if (status != Status::Idle)
		reply.addString("Wait for completion of the current phase!");
	    else
	    {
		previous_status = status;
		status = Status::PreparePush;
		current_hand = "right";

		reply.addString("Push with right-arm issued.");
	    }
	}
	else if (cmd == "rotate-with-right")
	{
	    if (status != Status::Idle)
		reply.addString("Wait for completion of the current phase!");
	    else
	    {
		previous_status = status;
		status = Status::PrepareRotation;
		current_hand = "right";

		reply.addString("Rotation with right-arm issued.");
	    }
	}
	else if (cmd == "stop")
	{
	    previous_status = status;
	    status = Status::Stop;

	    reply.addString("Stop issued.");
	}
        else
	{
	    mutex.unlock();

            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
	}

	mutex.unlock();

        return true;
    }

    double getPeriod()
    {
        return 0.02;
    }

    bool updateModule()
    {
	if(isStopping())
	    return false;

	mutex.lock();

	// get the current and previous status
	Status curr_status;
	Status prev_status;
	curr_status = status;
	prev_status = previous_status;

	// get the current hand
	std::string curr_hand = current_hand;

	mutex.unlock();

	// get current estimate from the filter
	std::string source = "/iCub/frame";
	std::string target = "/box_alt/estimate/frame";
	is_estimate_available = tf_client->getTransform(target, source, estimate);

	switch(curr_status)
	{
	case Status::Idle:
	{
	    // nothing to do here
	    break;
	}

	case Status::Localize:
	{
	    // issue localization
	    sendCommandToFilter(true, "visual");

	    // go back to Idle
	    mutex.lock();
	    status = Status::Idle;
	    mutex.unlock();

	    break;
	}

	case Status::MoveLeftUpward:
	{
	    // issue command
	    moveLeftArmUpward();

	    // go back to Idle
	    mutex.lock();
	    status = Status::Idle;
	    mutex.unlock();

	    break;
	}

	case Status::ArmApproach:
	{
	    // reset flag
	    is_approach_done = false;

	    if (curr_hand.empty())
	    {
		// this should not happen
		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();

		break;
	    }

	    // issue approach with arm
	    approachObjectWithArm(curr_hand, approach_corner);

	    // reset approach_corner flag
	    approach_corner = false;

	    // go to state WaitArmApproachDone
	    mutex.lock();
	    status = Status::WaitArmApproachDone;
	    mutex.unlock();

	    // reset timer
	    last_time = yarp::os::Time::now();

	    break;
	}

	case Status::WaitArmApproachDone:
	{
	    // timeout
	    double timeout = 5.0;

	    // check status
	    bool is_done = false;
	    bool ok = checkArmMotionDone(curr_hand, is_done);

	    // handle failure and timeout
	    if (!ok ||
		((yarp::os::Time::now() - last_time > timeout)))
	    {
		// stop control
		stopArm(curr_hand);

		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();
	    }

	    if (is_done)
	    {
		// approach completed
		yInfo() << "Arm approach done";

		// go to FingersApproach
		mutex.lock();
		status = Status::FingersApproach;
		mutex.unlock();
	    }

	    break;
	}

	case Status::FingersApproach:
	{
	    if (curr_hand.empty())
	    {
		// this should not happen
		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();

		break;
	    }

	    // issue approach with fingers
	    approachObjectWithFingers(curr_hand);

	    // go to state WaitFingersApproachDone
	    mutex.lock();
	    status = Status::WaitFingersApproachDone;
	    mutex.unlock();

	    // reset timer
	    last_time = yarp::os::Time::now();

	    break;
	}

	case Status::WaitFingersApproachDone:
	{
	    // timeout
	    double timeout = 10.0;

	    // check status
	    bool is_done = false;
	    bool ok = checkFingersMotionDone(curr_hand,
					     "fingers_approach",
					     is_done);
	    // handle failure and timeout
	    if (!ok ||
		((yarp::os::Time::now() - last_time > timeout)))
	    {
		// stop control
		stopFingers(curr_hand);

		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();
	    }

	    if (is_done)
	    {
		// approach completed
		yInfo() << "Fingers approach done";

		// go to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();

		// update flag
		is_approach_done = true;
	    }

	    break;
	}

	case Status::PreparePush:
	{
	    if (!is_approach_done)
	    {
		// push not possible
		// ignore this command

		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();

		break;
	    }

	    // reset flags
	    is_approach_done = false;
	    is_timer_started = false;

	    // prepare controller for push
	    preparePushObject(curr_hand);

	    // enable tactile filtering
	    sendCommandToFilter(true, "tactile");

	    // enable fingers following mode
	    enableFingersFollowing(curr_hand);

	    // go to state PerformPush
	    mutex.lock();
	    status = Status::PerformPush;
	    mutex.unlock();

	    break;
	}

	case Status::PerformPush:
	{
	    if (!is_timer_started)
	    {
		is_timer_started = true;

		// reset time
		last_time = yarp::os::Time::now();
	    }

	    // eval elapsed time
	    double elapsed = yarp::os::Time::now() - last_time;

	    // get current trajectory
	    yarp::sig::Vector pos(3, 0.0);
	    yarp::sig::Vector vel(3, 0.0);
	    traj_gen.getTrajectory(elapsed, pos, vel);

	    // issue velocity command
	    setArmLinearVelocity(curr_hand, vel);

	    // check for trajectory completion
	    if (elapsed > trajectory_duration)
	    {
		// issue zero velocities
		vel = 0;
		setArmLinearVelocity(curr_hand, vel);

		// stop fingers control
		stopFingers(curr_hand);

		// disable filtering
		sendCommandToFilter(false);

		// restore arm controller context
		// that was changed in preparePushObject(curr_hand)
		restoreArmControllerContext(curr_hand);

		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();
	    }

	    break;
	}

	case Status::PrepareRotation:
	{
	    if (curr_hand.empty())
	    {
		// this should not happen
		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();

		break;
	    }

	    // reset flags
	    is_approach_done = false;
	    is_timer_started = false;

	    // prepare controller for rotation
	    prepareRotateObject(curr_hand);

	    // enable tactile filtering
	    sendCommandToFilter(true, "tactile");

	    // enable fingers following mode
	    enableFingersFollowing(curr_hand);

	    // go to state PerformRotation
	    mutex.lock();
	    status = Status::PerformRotation;
	    mutex.unlock();

	    break;
	}

	case Status::PerformRotation:
	{
	    if (!is_timer_started)
	    {
		is_timer_started = true;

		// reset time
		last_time = yarp::os::Time::now();
	    }

	    // eval elapsed time
	    double elapsed = yarp::os::Time::now() - last_time;

	    // get current trajectory
	    yarp::sig::Vector vel(3, 0.0);
	    rot_traj_gen.getVelocity(elapsed, vel);

	    // issue velocity command
	    setArmLinearVelocity(curr_hand, vel);

	    // check for trajectory completion
	    if (elapsed > trajectory_duration)
	    {
		// issue zero velocities
		vel = 0;
		setArmLinearVelocity(curr_hand, vel);

		// stop fingers control
		stopFingers(curr_hand);

		// disable filtering
		sendCommandToFilter(false);

		// restore arm controller context
		// that was changed in prepareRotateObject(curr_hand)
		restoreArmControllerContext(curr_hand);

		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();
	    }

	    break;
	}

	case Status::FingersRestore:
	{
	    if (curr_hand.empty())
	    {
		// this should not happen
		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();

		break;
	    }

	    // issue fingers restore
	    restoreFingers(curr_hand);

	    // reset timer
	    last_time = yarp::os::Time::now();

	    // go to WaitFingersRestoreDone
	    mutex.lock();
	    status = Status::WaitFingersRestoreDone;
	    mutex.unlock();

	    break;
	}

	case Status::WaitFingersRestoreDone:
	{
	    // timeout
	    double timeout = 10.0;

	    // check status
	    bool is_done = false;
	    bool ok = checkFingersMotionDone(curr_hand,
					     "fingers_restore",
					     is_done);
	    // handle failure and timeout
	    if (!ok ||
		((yarp::os::Time::now() - last_time > timeout)))
	    {
		// stop control
		stopFingers(curr_hand);

		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();
	    }

	    if (is_done)
	    {
		// approach completed
		yInfo() << "Fingers restore done";

		// go to ArmRestore
		mutex.lock();
		status = Status::ArmRestore;
		mutex.unlock();
	    }

	    break;
	}

	case Status::ArmRestore:
	{
	    if (curr_hand.empty())
	    {
		// this should not happen
		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();

		break;
	    }

	    // issue arm restore
	    restoreArm(curr_hand);

	    // go to state WaitArmApproachDone
	    mutex.lock();
	    status = Status::WaitArmRestoreDone;
	    mutex.unlock();

	    // reset timer
	    last_time = yarp::os::Time::now();

	    break;
	}

	case Status::WaitArmRestoreDone:
	{
	    // timeout
	    double timeout = 5.0;

	    // check status
	    bool is_done = false;
	    bool ok = checkArmMotionDone(curr_hand, is_done);

	    // handle failure and timeout
	    if (!ok ||
		((yarp::os::Time::now() - last_time > timeout)))
	    {
		// stop control
		stopArm(curr_hand);

		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();
	    }

	    if (is_done)
	    {
		// restore completed
		yInfo() << "Arm restore done";

		// go back to Idle
		mutex.lock();
		status = Status::Idle;
		mutex.unlock();
	    }

	    break;
	}

	case Status::Stop:
	{
	    mutex.lock();

	    // stop control
	    stopArm("right");
	    stopArm("left");
	    stopFingers(curr_hand);

	    // disable filtering
	    sendCommandToFilter(false);

	    // in case pushing was initiated
	    // the previous context of the cartesian controller
	    // has to be restored
	    if (prev_status == Status::PreparePush ||
		prev_status == Status::PerformPush)
	    {
		// restore arm controller context
		// that was changed in pushObject(curr_hand)
		restoreArmControllerContext(curr_hand);
	    }

	    // reset flag
	    is_approach_done = false;

	    // go back to Idle
	    status = Status::Idle;

	    mutex.unlock();

	    break;
	}
	}
	
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

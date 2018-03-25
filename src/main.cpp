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

// icub-main
#include <iCub/skinDynLib/skinContactList.h>
#include <iCub/skinDynLib/common.h>
#include <iCub/iKin/iKinFwd.h>

#include "headers/PointCloud.h"
#include "headers/filterData.h"
#include "headers/ArmController.h"
#include "headers/HandController.h"
#include "headers/ModelHelper.h"

using namespace yarp::math;

typedef std::map<iCub::skinDynLib::SkinPart, iCub::skinDynLib::skinContactList> skinPartMap;

class VisTacLocSimModule: public yarp::os::RFModule
{
protected:
    // rpc server
    yarp::os::RpcServer rpc_port;

    // arm controllers
    RightArmController right_arm;
    LeftArmController left_arm;

    // hand controllers
    RightHandController right_hand;
    LeftHandController left_hand;

    // mutexes required to share data between
    // the RFModule thread and the rpc thread
    yarp::os::Mutex mutex;
    yarp::os::Mutex mutex_contacts;

    // point cloud port and storage
    yarp::os::BufferedPort<PointCloud> port_pc;
    PointCloud pc;

    // filter port
    yarp::os::BufferedPort<yarp::sig::FilterData> port_filter;

    // contact points port and storage
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> port_contacts;
    iCub::skinDynLib::skinContactList skin_contact_list;
    bool are_contacts_available;

    // FrameTransformClient to read published poses
    yarp::dev::PolyDriver drv_transform_client;
    yarp::dev::IFrameTransform* tf_client;

    // transformation from inertial to
    // the root link of the robot published by gazebo
    yarp::sig::Matrix inertial_to_robot;

    // last estimate published by the filter
    yarp::sig::Matrix estimate;
    bool is_estimate_available;

    // model helper class
    ModelHelper mod_helper;

    /**
     *  IEncoders
     */

    // PolyDriver required to access yarp::dev::IEncoders encoders
    yarp::dev::PolyDriver drv_right_arm;
    yarp::dev::PolyDriver drv_left_arm;
    yarp::dev::PolyDriver drv_torso;

    // pointers to yarp::dev::IEncoders view of the PolyDriver
    yarp::dev::IEncoders *ienc_right_arm;
    yarp::dev::IEncoders *ienc_left_arm;
    yarp::dev::IEncoders *ienc_torso;

    /*
     */

    /**
     *  iCub forward kinematics
     */

    iCub::iKin::iCubArm right_arm_kin;
    iCub::iKin::iCubArm left_arm_kin;

    // middle finger only for now
    // TODO: add the other fingers
    iCub::iKin::iCubFinger right_middle;
    iCub::iKin::iCubFinger left_middle;

    /*
     */

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
     * Return the last point cloud stored in
     * this->pc taking into account the pose of the root frame
     * of the robot attached to its waist.
     * This function is simulation-related and allows to obtain
     * point clouds expressed with respect to the robot root frame
     * as happens in the real setup.
     */
    bool getPointCloud(std::vector<yarp::sig::Vector> &pc_out)
    {
	mutex.lock();

	// check if there are points
	if (pc.size() == 0 )
	{
	    mutex.unlock();
	    return false;
	}

	// copy data to pc_out
	for (size_t i=0; i<pc.size(); i++)
	{
	    PointCloudItem item = pc[i];
	    yarp::sig::Vector point(3, 0.0);
	    point[0] = item.x;
	    point[1] = item.y;
	    point[2] = item.z;

	    pc_out.push_back(point);
	}

	mutex.unlock();

	// transform the points taking into account
	// the root link of the robot
	for (size_t i=0; i<pc_out.size(); i++)
	{
	    yarp::sig::Vector point(4, 0.0);
	    point.setSubvector(0, pc_out[i]);
	    point[3] = 1;

	    // transform the point so that
	    // it is relative to the orign of the robot root frame
	    // and expressed in the robot root frame
	    point = SE3inv(inertial_to_robot) * point;

	    pc_out[i] = point.subVector(0,2);
	}

	return true;
    }

    /*
     * Return the number of contacts detected for each finger tip
     * for the specified hand as a std::map.
     * The key is the name of the finger, i.e. 'thumb', 'index',
     * 'middle', 'ring' or 'pinky'.
     */
    bool getNumberContacts(const std::string &which_hand,
			   std::unordered_map<std::string, int> &numberContacts)
    {
	// split contacts per SkinPart
	skinPartMap map = skin_contact_list.splitPerSkinPart();

	// take the right skinPart
	iCub::skinDynLib::SkinPart skinPart;
	if (which_hand == "right")
	    skinPart = iCub::skinDynLib::SkinPart::SKIN_RIGHT_HAND;
	else
	    skinPart = iCub::skinDynLib::SkinPart::SKIN_LEFT_HAND;

	// clear number of contacts for each finger
	int n_thumb = 0;
	int n_index = 0;
	int n_middle = 0;
	int n_ring = 0;
	int n_pinky = 0;

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
		n_pinky++;
	    else if (taxel_id >= 48 && taxel_id < 60)
		n_thumb++;
    	}

	numberContacts["thumb"] = n_thumb;
	numberContacts["index"] = n_index;
	numberContacts["middle"] = n_middle;
	numberContacts["ring"] = n_ring;
	numberContacts["pinky"] = n_pinky;

	return true;
    }

    /*
     * Return the latest contact points received for specified hand.
     * Contact points produced by the Gazebo plugin are already
     * expressed with respect to the robot root frame.
     */
    bool getContactPoints(const std::string &which_hand,
			  std::vector<yarp::sig::Vector> &points)
    {
	// split contacts per SkinPart
	skinPartMap map = skin_contact_list.splitPerSkinPart();

	// take the right skinPart
	iCub::skinDynLib::SkinPart skinPart;
	if (which_hand == "right")
	    skinPart = iCub::skinDynLib::SkinPart::SKIN_RIGHT_HAND;
	else
	    skinPart = iCub::skinDynLib::SkinPart::SKIN_LEFT_HAND;

	// extract contacts coming from finger tips only
	iCub::skinDynLib::skinContactList &list = map[skinPart];
    	for (size_t i=0; i<list.size(); i++)
    	{
	    // extract the skin contact
    	    iCub::skinDynLib::skinContact &skin_contact = list[i];

	    // need to verify if this contact was effectively produced
	    // by taxels on the finger tips
	    // in order to simplify things the Gazebo plugin only sends one
	    // taxel id that is used to identify which finger is in contact
	    std::vector<unsigned int> taxels_ids = skin_contact.getTaxelList();
	    // taxels ids for finger tips are between 0 and 59
	    if (taxels_ids[0] >= 0 && taxels_ids[0] < 60)
		points.push_back(skin_contact.getGeoCenter());
    	}

	// clean the list once used
	skin_contact_list.clear();

    	return true;
    }

    /*
     * Perform object localization using the last
     * point cloud available.
     */
    bool localizeObject()
    {
    	// process cloud in chuncks of 10 points
    	// TODO: take n_points from config
    	int n_points = 10;

    	// get the last point cloud received
    	std::vector<yarp::sig::Vector> pc;
    	if (!getPointCloud(pc))
    	    return false;

    	// process the point cloud
    	for (size_t i=0; i+n_points <= pc.size(); i += n_points)
    	{
    	    // prepare to write
    	    yarp::sig::FilterData &filter_data = port_filter.prepare();

    	    // clear the storage
    	    filter_data.clear();

	    // set the command
	    filter_data.setCommand(VOCAB2('O','N'));

    	    // set the tag
    	    filter_data.setTag(VOCAB3('V','I','S'));

    	    // add measures
    	    for (size_t k=0; k<n_points; k++)
    		filter_data.addPoint(pc[i+k]);

    	    // add zero input
    	    yarp::sig::Vector zero(3, 0.0);
    	    filter_data.addInput(zero);

    	    // send data to the filter
    	    port_filter.writeStrict();

    	    // wait
    	    yarp::os::Time::delay(0.1);
    	}

	return true;
    }

    bool approachObject(const std::string &which_hand)
    {
	bool ok;

    	if (!is_estimate_available)
    	    return false;

	ArmController* arm;
	HandController* hand;
	if (which_hand == "right")
	{
	    arm = &right_arm;
	    hand = &right_hand;
	}
	else
	{
	    arm = &left_arm;
	    hand = &left_hand;
	}
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
	hand->resetFingersContacts();

	mutex_contacts.lock();
	skin_contact_list.clear();
	mutex_contacts.unlock();

	// move thumb opposition
	// and index, middle and ring until contact
	double t0 = yarp::os::Time::now();
	bool done = false;
	std::unordered_map<std::string, int> number_contacts;
	std::vector<std::string> finger_list = {"thumb", "index", "middle", "ring"};
	while (!done && (yarp::os::Time::now() - t0 < 15.0))
	{
    	    mutex_contacts.lock();

	    getNumberContacts(which_hand, number_contacts);
	    skin_contact_list.clear();

	    mutex_contacts.unlock();
	    
	    ok = hand->moveFingersUntilContact(finger_list,
					       0.005,
					       number_contacts,
					       done);
	    if (!ok)
		return false;

    	    yarp::os::Time::delay(0.01);
	}
	// in case the contact was not reached for all the fingers
	// stop them and abort
	if (!done)
	{
	    hand->stopFingers();

	    return false;
	}

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
	HandController* hand;
	if (which_hand == "right")
	{
	    arm = &right_arm;
	    hand = &right_hand;
	}
	else
	{
	    arm = &left_arm;
	    hand = &left_hand;
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
	double duration = 3.0;
	double traj_time = 3.0;
        arm->cartesian()->setTrajTime(traj_time);

        // // request pose to the cartesian interface
        arm->goToPos(pos);

        // filter while motion happens
        double t0 = yarp::os::Time::now();
    	double dt = 0.03;
    	bool done = false;
	std::unordered_map<std::string, int> number_contacts;
	std::vector<std::string> finger_list = {"index", "middle", "ring"};
    	while (!done && (yarp::os::Time::now() - t0 < duration))
    	{
	    mutex_contacts.lock();
	    getNumberContacts(which_hand, number_contacts);
	    mutex_contacts.unlock();

	    hand->moveFingersMaintainingContact(finger_list,
						0.005,
						number_contacts);

    	    arm->cartesian()->checkMotionDone(&done);

    	    // get velocity of the finger
    	    yarp::sig::Vector x_dot;
	    bool new_speed = getFingerVelocity(which_hand, "middle", x_dot);

    	    mutex_contacts.lock();

	    if(are_contacts_available)
	    {
		// extract contact points for the specified hand
		std::vector<yarp::sig::Vector> points;
		getContactPoints(which_hand, points);

		yarp::sig::FilterData &filter_data = port_filter.prepare();

		// clear the storage
		filter_data.clear();

		// set the command
		filter_data.setCommand(VOCAB2('O','N'));

		// set the tag
		filter_data.setTag(VOCAB3('T','A','C'));

		// add measures
		for (size_t i=0; i<points.size(); i++)
		    filter_data.addPoint(points[i]);

		// add input
		filter_data.addInput(x_dot);

		// send data to the filter
		port_filter.writeStrict();
	    }

	    skin_contact_list.clear();
    	    mutex_contacts.unlock();

    	    // wait
    	    yarp::os::Time::delay(dt);
    	}

	// stop filtering
	yarp::sig::FilterData &filter_data = port_filter.prepare();
	filter_data.clear();
	filter_data.setCommand(VOCAB3('O','F','F'));
	port_filter.writeStrict();

	hand->stopFingers();

        // restore the context
        arm->cartesian()->restoreContext(context_id);

    	return true;
    }

public:
    bool configure(yarp::os::ResourceFinder &rf)
    {
	// open the point cloud port
	// TODO: take name from config
	bool ok = port_pc.open("/vis_tac_localization/pc:i");
	if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to open the point cloud port";
            return false;
        }

	// open the filter port
	// TODO: take name from config
	ok = port_filter.open("/vis_tac_localization/filter:o");
	if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to open the filter port";
            return false;
        }

	// open the contacts port
	// TODO: take name from config
	ok = port_contacts.open("/vis_tac_localization/contacts:i");
	if (!ok)
        {
            yError() << "VisTacLocSimModule: unable to open the contacts port";
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

	// get the pose of the root frame of the robot
	// TODO: get source and target from configuration file
	inertial_to_robot.resize(4,4);
	std::string source = "/inertial";
	std::string target = "/iCub/frame";

	ok = false;
        double t0 = yarp::os::SystemClock::nowSystem();
        while (yarp::os::SystemClock::nowSystem() - t0 < 10.0)
        {
            // this might fail if the gazebo pluging
	    // publishing the pose is not started yet
            if (tf_client->getTransform(target, source, inertial_to_robot))
            {
                ok = true;
                break;
            }
	    yarp::os::SystemClock::delaySystem(1.0);
        }
        if (!ok)
	{
            yError() << "VisTacLocSimModule: unable to get the pose of the root link of the robot";
            return false;
	}

	// open the rpc server
	// TODO: take name from config
        rpc_port.open("/service");
        attach(rpc_port);

	// set default value of flags
	is_estimate_available = false;
	are_contacts_available = false;

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

	// configure hand controllers
	ok = right_hand.configure();
        if (!ok)
	{
            yError() << "VisTacLocSimModule: unable to configure the right hand controller";
            return false;
	}

	ok = left_hand.configure();
        if (!ok)
	{
            yError() << "VisTacLocSimModule: unable to configure the left hand controller";
            return false;
	}

	// configure model helper
	mod_helper.setModelDimensions(0.24, 0.17, 0.037);

	// prepare properties for the Encoders
	yarp::os::Property prop_encoders;
	prop_encoders.put("device", "remote_controlboard");
	prop_encoders.put("remote", "/icubSim/right_arm");
	prop_encoders.put("local", "/upf-localizer/encoders/right_arm");
	bool ok_drv = drv_right_arm.open(prop_encoders);
	if (!ok_drv)
	{
	    yError() << "LocalizerModule::configure error:"
		     << "unable to open the Remote Control Board driver for the right arm";
	    return false;
	}

	prop_encoders.put("remote", "/icubSim/left_arm");
	prop_encoders.put("local", "/upf-localizer/encoders/left_arm");
	ok_drv = drv_left_arm.open(prop_encoders);
	if (!ok_drv)
	{
	    yError() << "LocalizerModule::configure error:"
		     << "unable to open the Remote Control Board driver for the left arm";
	    return false;
	}

	prop_encoders.put("remote", "/icubSim/torso");
	prop_encoders.put("local", "/upf-localizer/encoders/torso");
	ok_drv = drv_torso.open(prop_encoders);
	if (!ok_drv)
	{
	    yError() << "LocalizerModule::configure error:"
		     << "unable to open the Remote Control Board driver for the torso";
	    return false;
	}

	// try to retrieve the views
	bool ok_view = drv_right_arm.view(ienc_right_arm);
	if (!ok_view || ienc_right_arm == 0)
	{
	    yError() << "LocalizerModule:configure error:"
		     << "unable to retrieve the Encoders view for the right arm";
	    return false;
	}
	ok_view = drv_left_arm.view(ienc_left_arm);
	if (!ok_view || ienc_left_arm == 0)
	{
	    yError() << "LocalizerModule:configure error:"
		     << "unable to retrieve the Encoders view for the left arm";
	    return false;
	}
	ok_view = drv_torso.view(ienc_torso);
	if (!ok_view || ienc_torso == 0)
	{
	    yError() << "LocalizerModule:configure error:"
		     << "unable to retrieve the Encoders view for the torso";
	    return false;
	}

	// configure forward kinematics
	right_arm_kin = iCub::iKin::iCubArm("right");
	left_arm_kin = iCub::iKin::iCubArm("left");
	right_middle = iCub::iKin::iCubFinger("right_middle");
	left_middle = iCub::iKin::iCubFinger("left_middle");

	// Limits update is not required to evaluate the forward kinematics
	// using angles from the encoders
	right_arm_kin.setAllConstraints(false);
	left_arm_kin.setAllConstraints(false);
	// Torso can be moved in general so its links have to be released
	right_arm_kin.releaseLink(0);
	right_arm_kin.releaseLink(1);
	right_arm_kin.releaseLink(2);
	left_arm_kin.releaseLink(0);
	left_arm_kin.releaseLink(1);
	left_arm_kin.releaseLink(2);

        return true;
    }

    bool getFingerVelocity(const std::string &hand_name,
			   const std::string &finger_name,
			   yarp::sig::Vector &finger_vel)
    {
	// choose between right and left hand
	// only middle finger implemented for now
	iCub::iKin::iCubArm *arm;
	iCub::iKin::iCubFinger *finger;
	yarp::dev::IEncoders *enc;
	if (hand_name == "right")
	{
	    arm = &right_arm_kin;
	    enc = ienc_right_arm;
	    finger = &right_middle;
	}
	else
	{
	    arm = &left_arm_kin;
	    enc = ienc_left_arm;
	    finger = &left_middle;
	}

	// get the encoders readings
	yarp::sig::Vector encs_arm(16);
	yarp::sig::Vector encs_torso(3);

	bool ok = enc->getEncoders(encs_arm.data());
	if(!ok)
	    return false;

	ok = ienc_torso->getEncoders(encs_torso.data());
	if(!ok)
	    return false;

	// fill in the vector of degrees of freedom
	yarp::sig::Vector arm_angles(arm->getDOF());
	yarp::sig::Vector finger_angles(finger->getDOF());
	arm_angles[0] = encs_torso[2];
	arm_angles[1] = encs_torso[1];
	arm_angles[2] = encs_torso[0];
	arm_angles.setSubvector(3, encs_arm.subVector(0, 6));
	finger->getChainJoints(encs_arm, finger_angles);

	// update the chain and the finger
	// (iKin uses radians)
	arm->setAng((M_PI/180) * arm_angles);
	finger->setAng((M_PI/180) * finger_angles);

	// get the geometric jacobian
	yarp::sig::Matrix jac = arm->GeoJacobian();

	// get the joints speeds
	yarp::sig::Vector speeds_torso(3);
	yarp::sig::Vector speeds_arm(16);

	ok = enc->getEncoderSpeeds(speeds_arm.data());
	if (!ok)
	    return false;

	ok = ienc_torso->getEncoderSpeeds(speeds_torso.data());
	if (!ok)
	    return false;

	// fill in the vector of degrees of freedom
	yarp::sig::Vector speeds(arm->getDOF());
	speeds[0] = speeds_torso[2];
	speeds[1] = speeds_torso[1];
	speeds[2] = speeds_torso[0];
	speeds.setSubvector(3, speeds_arm.subVector(0, 6));

	// evaluate the twist of the hand
	yarp::sig::Vector twist(6, 0.0);
	twist = jac * (M_PI / 180 * speeds);

	// get the current position of the finger
	// with respect to the center of the hand
	yarp::sig::Vector finger_pose = finger->EndEffPosition();
	// express it in the robot root frame
	finger_pose = (arm->getH()).submatrix(0, 2, 0, 2) * finger_pose;

	// evaluate the velocity of finger
	finger_vel = twist.subVector(0, 2) +
	    yarp::math::cross(twist.subVector(3, 5), finger_pose);

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
	port_pc.close();
	port_filter.close();
	port_contacts.close();

        return true;
    }

    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
    {
	bool ok;
        std::string cmd = command.get(0).asString();
        if (cmd == "help")
        {
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString("Available commands:");
	    reply.addString("- model-helper-test");
            reply.addString("- home-right");
            reply.addString("- home-left");
            reply.addString("- localize");
	    reply.addString("- approach-right");
	    reply.addString("- push-right");
            reply.addString("- quit");
        }
	else if (cmd == "model-helper-test")
	{
	    mutex.lock();

	    yarp::sig::Matrix rotation = estimate.submatrix(0, 2, 0, 2);
	    mod_helper.setModelAttitude(rotation);
	    
	    mutex.unlock();
            reply.addString("Done.");	    
	}
	else if (cmd == "home-right")
	{
	    ok = right_hand.restoreFingersPosition();

	    waitSeconds(5);

	    if (ok)
		ok &= right_arm.goHome();
		
	    if (ok)
		reply.addString("Go home done for right arm.");
	    else
		reply.addString("Go home failed for right arm.");
	}
	else if (cmd == "home-left")
	{
	    ok = left_hand.restoreFingersPosition();

	    waitSeconds(5);

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

	// read from the point cloud port
	PointCloud *new_pc = port_pc.read(false);
	// store a copy if new data available
	if (new_pc != NULL)
	    pc = *new_pc;

	// get current estimate from the filter
	// TODO: get source and target from configuration file
	std::string source = "/iCub/frame";
	std::string target = "/box_alt/estimate/frame";
	is_estimate_available = tf_client->getTransform(target, source, estimate);

	mutex.unlock();

	mutex_contacts.lock();

	iCub::skinDynLib::skinContactList *new_contacts = port_contacts.read(false);
	if (new_contacts != NULL && new_contacts->size() > 0)
	{
	    skin_contact_list = *new_contacts;
	    are_contacts_available = true;
	}
	else
	{
	    skin_contact_list.clear();
	    are_contacts_available = false;
	}

	mutex_contacts.unlock();

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

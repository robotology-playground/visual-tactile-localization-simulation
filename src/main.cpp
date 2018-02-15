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

// yarp os
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Vocab.h>

// yarp sig
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

// yarp math
#include <yarp/math/Math.h>
#include <yarp/math/FrameTransform.h>

// yarp dev
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>

// icub-main
#include <iCub/iKin/iKinFwd.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <iCub/iKin/iKinFwd.h>

#include <cmath>

#include "headers/PointCloud.h"
#include "headers/filterData.h"

using namespace yarp::math;

class VisTacLocSimModule: public yarp::os::RFModule
{
protected:
    // driver and context
    yarp::dev::PolyDriver drv_arm_cart_right;
    yarp::dev::PolyDriver drv_arm_enc_right;
    yarp::dev::PolyDriver drv_arm_enc_torso;    
    yarp::dev::ICartesianControl *iarm;
    yarp::dev::IEncoders *ienc_right;
    yarp::dev::IEncoders *ienc_torso;
    int startup_cart_context_id;
    
    // rpc server
    yarp::os::RpcServer rpc_port;

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

    // FrameTransformClient to read the
    // published poses 
    // 
    yarp::dev::PolyDriver drv_transform_client;
    yarp::dev::IFrameTransform* tf_client;

    // transformation from inertial to
    // the root link of the robot 
    yarp::sig::Matrix inertial_to_robot;

    // last estimate from the filter
    yarp::sig::Matrix estimate;
    bool is_estimate_available;

    // fixed hand orientation
    yarp::sig::Vector hand_orientation;

    // home pose
    yarp::sig::Vector home_pos;
    yarp::sig::Vector home_att;

    // chain for the right arm
    iCub::iKin::iCubArm right_arm_chain;    
    
    /*
     * This function evaluates the orientation of the right hand
     * required during a pushing phase
     * TODO: rename function
     * TODO: have a similar function for the left hand
     */
    void computeHandOrientation(yarp::sig::Vector& orientation)
    {
        // given the reference frame convention for the hands of iCub
        // it is required to have the x-axis attached to the center of the
        // palm pointing forward, the y-axis pointing downward and
        // the z-axis pointing lefttward
        //
        // one solution to obtain the final attitude w.r.t to the waist frame
        // is to compose a rotation of pi about the z-axis and a rotation
        // of -pi/2 about the x-axis (after the first rotation)

        yarp::sig::Vector axis_angle(4);
	yarp::sig::Matrix dcm;
	
        axis_angle[0] = 0.0;
        axis_angle[1] = 0.0;
        axis_angle[2] = 1.0;
        axis_angle[3] = +M_PI;
        dcm = yarp::math::axis2dcm(axis_angle);
	
        axis_angle[0] = 1.0;
        axis_angle[1] = 0.0;
        axis_angle[2] = 0.0;
        axis_angle[3] = -M_PI/2.0;
	dcm = dcm * yarp::math::axis2dcm(axis_angle);

	// add also a slight rotation about the the y-axis
	// (after the second rotation)
        axis_angle[0] = 0.0;
        axis_angle[1] = 1.0;
        axis_angle[2] = 0.0;
        axis_angle[3] = -15 * M_PI/180;
	dcm = dcm * yarp::math::axis2dcm(axis_angle);

        // convert back to axis
        orientation = yarp::math::dcm2axis(dcm);
    }

    /*
     * This function return the last point cloud stored in
     * this->pc taking into account the pose of the root frame
     * of the robot attached to its waist.
     * This function is simulation-related and allows to obtain
     * point clouds expressed with respect to the robot root frame
     * as happens in the real setup.
     */
    bool getPointCloud(std::vector<yarp::sig::Vector> &pc_out)
    {
	mutex.lock();

	// check if the pointer is valid
	// and if it contains data
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
     * This function return the latest contact points received
     * taking into account the pose of the frame attached to 
     * the palm of the hand. Contact points produced
     * by the skinManager are expressed with respect to that 
     * frame while the filter requires the point to be expressed
     * with respect to the robot root frame.
     */
    bool getContactPoints(std::vector<yarp::sig::Vector> &points)
    {
	// get current value of encoders
	yarp::sig::Vector encs_torso(3);
	yarp::sig::Vector encs_arm(16);

	bool ok = ienc_right->getEncoders(encs_arm.data());
	if(!ok)
	    return false;

	ok = ienc_torso->getEncoders(encs_torso.data());
	if(!ok)
	    return false;

	// fill in the vector of degrees of freedom
	yarp::sig::Vector joints_angles(right_arm_chain.getDOF());
	joints_angles[0] = encs_torso[2];
	joints_angles[1] = encs_torso[1];
	joints_angles[2] = encs_torso[0];
	joints_angles[3] = encs_arm[0];
	joints_angles[4] = encs_arm[1];
	joints_angles[5] = encs_arm[2];
	joints_angles[6] = encs_arm[3];
	joints_angles[7] = encs_arm[4];
	joints_angles[8] = encs_arm[5];    
	joints_angles[9] = encs_arm[6];

	// set the current values of the joints
	// iKin uses radians
	right_arm_chain.setAng((M_PI/180) * joints_angles);

	// get the transform from the robot root frame
	// to the frame attached to the plam of the hand
	yarp::sig::Matrix inertial_to_hand = right_arm_chain.getH();

	// transform all the contact points
	for (size_t i=0; i<skin_contact_list.size();  i++)
	{
	    const yarp::sig::Vector &skin_contact = skin_contact_list[i].getGeoCenter();
	    
	    yarp::sig::Vector point = inertial_to_hand.getCol(3).subVector(0,2) +
		inertial_to_hand.submatrix(0, 2, 0, 2) * skin_contact;
	    
	    points.push_back(point);
	}

	return true;
    }

    /*
     * This function restore the initial pose of the arm.
     */
    void goHome()
    {
	iarm->goToPoseSync(home_pos, home_att);
	iarm->waitMotionDone(0.03, 3);
    }
    
    /*
     * This function perform object localization using the last
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

    /*
     * This function attach a tip to the end effector
     * so that the reference for the cartesian controller
     * can be specified with respect to that tip
     */
    bool attachTipFrame(const std::string& finger_name)
    {
	bool ok;
	
	// get current value of encoders
	int nEncs;
	ok = ienc_right->getAxes(&nEncs);
	if(!ok)
	    return false;
	
	yarp::sig::Vector encs(nEncs);
	ok = ienc_right->getEncoders(encs.data());
	if(!ok)
	    return false;

	// get the transformation between the standard
	// effector and the desired finger
	yarp::sig::Vector joints;
	iCub::iKin::iCubFinger finger(finger_name);
	ok = finger.getChainJoints(encs,joints);
	if (!ok)
	    return false;
	yarp::sig::Matrix tip_frame = finger.getH((M_PI/180.0)*joints);

	// attach the tip
	yarp::sig::Vector tip_x = tip_frame.getCol(3);
	yarp::sig::Vector tip_o = yarp::math::dcm2axis(tip_frame);
	ok = iarm->attachTipFrame(tip_x,tip_o);
	if(!ok)
	    return false;

	return true;
    }

    /*
     * This function moves the right hand near the
     * localized object and then pushes left.
     * During pushing the pose of the object is estimaed.
     * TODO: handle push direction
     */
    bool pushObject()
    {
	if (!is_estimate_available)
	    return false;
	
	mutex.lock();

	// copy the current estimate of the object
	yarp::sig::Matrix estimate = this->estimate;
	
	mutex.unlock();

	// extract positional part of the estimate
	yarp::sig::Vector pos(3, 0.0);
	for (size_t i=0; i<3; i++)
	    pos[i] = estimate[i][3];

	// change effector to the middle finger
	if(!attachTipFrame("right_middle"))
	    return false;

	// approach object using a shifted position
	pos[0] -= 0.02;		
	pos[1] += 0.07;

        // request pose to the cartesian interface
        iarm->goToPoseSync(pos, hand_orientation);

        // wait for motion completion
        iarm->waitMotionDone(0.04, 3.0);

        // final pose in the negative waist y-direction
        pos[1] -= 0.2;

        // store the current context because we are going
        // to change the trajectory time
        int context_id;
        iarm->storeContext(&context_id);

        // set trajectory time
        iarm->setTrajTime(3.0);

        // request pose to the cartesian interface
        iarm->goToPoseSync(pos, hand_orientation);
	
        // filter while motion happens
        double t0 = yarp::os::Time::now();
	double dt = 0.03;
	bool done = false;
	yarp::sig::Vector input(3, 0.0);
	yarp::sig::Vector prev_vel(3, 0.0);	
	while (!done && (yarp::os::Time::now() - t0 < 3.0))
	{
	    iarm->checkMotionDone(&done);

	    // get velocity of the finger
	    yarp::sig::Vector x_dot;
	    yarp::sig::Vector att_dot;
	    bool new_speed = iarm->getTaskVelocities(x_dot, att_dot);

	    mutex_contacts.lock();

	    if (new_speed)
	    {
		// accumulate the contribution
		// due to the velocity of the finger
		input += prev_vel * dt;
		
		if(are_contacts_available)
		{
		    yarp::sig::FilterData &filter_data = port_filter.prepare();
	    
		    // clear the storage
		    filter_data.clear();

		    // set the tag
		    filter_data.setTag(VOCAB3('T','A','C'));

		    // add measures
		    std::vector<yarp::sig::Vector> points;
		    getContactPoints(points);
		    for (size_t i=0; i<points.size(); i++)
			filter_data.addPoint(points[i]);

		    // add input
		    // remove components on the z plane
		    input[2] = 0;
		    filter_data.addInput(input);

		    // reset input
		    input = 0;

		    // send data to the filter
		    port_filter.writeStrict();
		}
	    }
	    mutex_contacts.unlock();

	    // store velocity for the next iteration
	    if (new_speed)
		prev_vel = x_dot;
		
	    // wait
	    yarp::os::Time::delay(0.03);
	}

        // restore the context
        iarm->restoreContext(context_id);

	// detach tip frame
	iarm->removeTipFrame(); 

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

	// prepare properties for the Encoders
        yarp::os::Property prop_enc_right;
        prop_enc_right.put("device", "remote_controlboard");
        prop_enc_right.put("remote", "/icubSim/right_arm");
        prop_enc_right.put("local", "/vis_tac_localization/encoder/right_arm");
	ok = drv_arm_enc_right.open(prop_enc_right);
        if (!ok)
	{
            yError() << "VisTacLocSimModule: unable to open the Remote Control Board driver"
		     << "for the right arm";
            return false;
	}

	yarp::os::Property prop_enc_torso;
        prop_enc_torso.put("device", "remote_controlboard");
        prop_enc_torso.put("remote", "/icubSim/torso");
        prop_enc_torso.put("local", "/vis_tac_localization/encoder/torso");
	ok = drv_arm_enc_torso.open(prop_enc_torso);
        if (!ok)
	{
            yError() << "VisTacLocSimModule: unable to open the Remote Control Board driver"
		     << "for the torso.";
            return false;
	}

	// try to retrieve the views
        drv_arm_enc_right.view(ienc_right);
	if (!ok || ienc_right == 0)
	{
	    yError() << "VisTacLocSimModule: Unable to retrieve the Encoders view."
		     << "for the right arm";
	    return false;
	}

        drv_arm_enc_torso.view(ienc_torso);
	if (!ok || ienc_torso == 0)
	{
	    yError() << "VisTacLocSimModule: Unable to retrieve the Encoders view."
		     << "for the torso";
	    return false;
	}
	
	// prepare properties for the CartesianController
        yarp::os::Property prop_arm;
        prop_arm.put("device", "cartesiancontrollerclient");
        prop_arm.put("remote", "/icubSim/cartesianController/right_arm");
        prop_arm.put("local", "/vis_tac_localization/cartesian_client/right_arm");

        // let's give the controller some time to warm up
	// here use real time and not simulation time
	ok = false;
        t0 = yarp::os::SystemClock::nowSystem();
        while (yarp::os::SystemClock::nowSystem() - t0 < 10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drv_arm_cart_right.open(prop_arm))
            {
                ok = true;
                break;
            }
	    yarp::os::SystemClock::delaySystem(1.0);	    
        }
        if (!ok)
        {
            yError() << "VisTacLocSimModule: Unable to open the Cartesian Controller driver.";
            return false;
        }

	// try to retrieve the view
        drv_arm_cart_right.view(iarm);
	if (!ok || iarm == 0)
	{
	    yError() << "VisTacLocSimModule: Unable to retrieve the CartesianController view.";
	    return false;
	}

        // store the current context so that
        // it can be restored when the modules closes
        iarm->storeContext(&startup_cart_context_id);

        // set trajectory time
        iarm->setTrajTime(1.0);

	// store home pose
	while(!iarm->getPose(home_pos, home_att))
	    yarp::os::Time::yield();
	
        // use also the torso
        yarp::sig::Vector newDoF, curDoF;
        iarm->getDOF(curDoF);
        newDoF = curDoF;

        newDoF[0] = 1;
        newDoF[1] = 1;
        newDoF[2] = 1;

        iarm->setDOF(newDoF, curDoF);

	// instantiate arm chains
	right_arm_chain = iCub::iKin::iCubArm("right");
	// limits update is not required to evaluate the forward kinematics
	// using angles from the encoders
	right_arm_chain.setAllConstraints(false);
	// torso can be moved in general so its links have to be released
	right_arm_chain.releaseLink(0);
	right_arm_chain.releaseLink(1);
	right_arm_chain.releaseLink(2);

	// open the rpc server
	// TODO: take name from config
        rpc_port.open("/service");
        attach(rpc_port);

	// set default value of flags
	is_estimate_available = false;
	are_contacts_available = false;

	// compute orientation of right hand once for all
	computeHandOrientation(hand_orientation);

        return true;
    }

    bool interruptModule()
    {
        return true;
    }

    bool close()
    {
        // stop the cartesian controller for safety reason
        iarm->stopControl();

        // restore the cartesian controller context
        iarm->restoreContext(startup_cart_context_id);

	// close drivers
        drv_arm_cart_right.close();
        drv_arm_enc_right.close();	
	drv_transform_client.close();

	// close ports
        rpc_port.close();
	port_pc.close();
	port_filter.close();
	port_contacts.close();
	
        return true;
    }

    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
    {
        std::string cmd = command.get(0).asString();
        if (cmd == "help")
        {
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- home");	    
            reply.addString("- localize");
	    reply.addString("- push");
            reply.addString("- quit");	    
        }
	else if (cmd == "home")
	{
	    goHome();
	    reply.addString("Go home done.");	    
	}
	else if (cmd == "localize")
	{
	    if (localizeObject())
		reply.addString("Localization using vision done.");
	    else
		reply.addString("Localization using vision failed.");		
	}
	else if (cmd == "push")
	{	
	    if (pushObject())
		reply.addString("Pushing done.");
	    else
		reply.addString("Pushing failed.");		
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
	std::string target = "/mustard/estimate/frame";
	is_estimate_available = tf_client->getTransform(target, source, estimate);
	
	mutex.unlock();

	mutex_contacts.lock();
	
	iCub::skinDynLib::skinContactList *new_contacts = port_contacts.read(false);
	if (new_contacts != NULL && new_contacts->size() > 0)
	{
	    skin_contact_list = *new_contacts;
	    are_contacts_available = true;
	    std::vector<yarp::sig::Vector> ps;
	}
	else
	    are_contacts_available = false;

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


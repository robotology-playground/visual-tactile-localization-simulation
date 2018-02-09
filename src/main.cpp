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

// yarp
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/SystemClock.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/math/FrameTransform.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/PolyDriver.h>

#include <cmath>

#include "headers/PointCloud.h"
#include "headers/filterData.h"

using namespace yarp::math;

class VisTacLocSimModule: public yarp::os::RFModule
{
protected:
    // driver and context
    yarp::dev::PolyDriver drv_arm;
    yarp::dev::ICartesianControl *iarm;
    int startup_cart_context_id;    
    
    // rpc server
    yarp::os::RpcServer rpc_port;

    // mutex required to share data between
    // the RFModule thread and the rpc thread
    yarp::os::Mutex mutex;

    // point cloud port and storage
    yarp::os::BufferedPort<PointCloud> port_pc;
    PointCloud pc;

    // filter port
    yarp::os::BufferedPort<yarp::sig::FilterData> port_filter;

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
     * This function moves the right hand near the
     * localized object and then pushes left.
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

	// approach object using a shifted position
	pos[0] += 0.06;	
	pos[1] += 0.06;
	pos[2] -= 0.01;

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

        // wait for motion completion
        iarm->waitMotionDone(0.04, 3.0);

        // restore the context
        iarm->restoreContext(context_id);

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
	
	// prepare properties for the CartesianController
        yarp::os::Property prop_arm;
        prop_arm.put("device", "cartesiancontrollerclient");
        prop_arm.put("remote", "/icubSim/cartesianController/right_arm");
        prop_arm.put("local", "/cartesian_client/right_arm");

        // let's give the controller some time to warm up
	// here use real time and not simulation time
	ok = false;
        t0 = yarp::os::SystemClock::nowSystem();
        while (yarp::os::SystemClock::nowSystem() - t0 < 10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drv_arm.open(prop_arm))
            {
                ok = true;
                break;
            }
	    yarp::os::SystemClock::delaySystem(1.0);	    
        }
        if (!ok)
        {
            yError() << "Unable to open the Cartesian Controller driver.";
            return false;
        }

	// try to retrieve the view	
        drv_arm.view(iarm);
	if (!ok || iarm == 0)
	{
	    yError() << "Unable to retrieve the CartesianController view.";
	    return false;
	}

        // store the current context so that
        // it can be restored when the modules closes
        iarm->storeContext(&startup_cart_context_id);

        // set trajectory time
        iarm->setTrajTime(1.0);

        // use also the torso
        yarp::sig::Vector newDoF, curDoF;
        iarm->getDOF(curDoF);
        newDoF = curDoF;

        newDoF[0] = 1;
        newDoF[1] = 1;
        newDoF[2] = 1;

        iarm->setDOF(newDoF, curDoF);

	// open the rpc server
	// TODO: take name from config
        rpc_port.open("/service");
        attach(rpc_port);

	// set default value of flag
	is_estimate_available = false;

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
        drv_arm.close();
	drv_transform_client.close();

	// close ports
        rpc_port.close();
	port_pc.close();
	port_filter.close();
	
        return true;
    }

    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
    {
        std::string cmd = command.get(0).asString();
        if (cmd == "help")
        {
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- localize");
	    reply.addString("- push");
            reply.addString("- quit");	    
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


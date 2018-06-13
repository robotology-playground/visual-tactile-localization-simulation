/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>

// opencv
#include <opencv2/opencv.hpp>

//
#include <HeadKinematics.h>
#include <cstdlib>

class Tracker : public yarp::os::RFModule
{
private:
    // head kinematics
    headKinematics head_kin;
    
    // camera port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> image_input_port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> image_output_port;

    // selected eye name
    std::string eye_name;

    // period
    double period;

    // frame transform client
    yarp::dev::PolyDriver drv_transform_client;    
    yarp::dev::IFrameTransform* tf_client;
    std::string tf_source;
    std::string tf_target;

    // camera parameters
    double cam_fx;
    double cam_fy;
    double cam_cx;
    double cam_cy;

public:
    bool configure(yarp::os::ResourceFinder &rf) override
    {
        // get parameter from the configuration
        std::string robot_name = "icub";
        if (!rf.find("robotName").isNull())
            robot_name = rf.find("robotName").asString();
        
        eye_name = "left";
        if (!rf.find("eyeName").isNull())
            eye_name = rf.find("eyeName").asString();

        double cam_width = 320;
        if (!rf.find("camWidth").isNull())
            cam_width = rf.find("camWidth").asDouble();

        double cam_height = 240;
        if (!rf.find("camHeight").isNull())
            cam_height = rf.find("camHeight").asDouble();

        period = 0.03;
        if (!rf.find("period").isNull())
            period = rf.find("period").asDouble();

	tf_source = "/iCub/frame";
        if (!rf.find("tfSource").isNull())
            tf_source = rf.find("tfSource").asString();
        tf_target = "/ground_truth/frame";
        if (!rf.find("tfTarget").isNull())
            tf_target = rf.find("tfTarget").asString();

        // port prefix
        std::string port_prefix = "/gtruth_tracker/";
        port_prefix += eye_name + "_eye";
	
        // open port
        bool port_ok = image_input_port.open(port_prefix + ":i");
        if (!port_ok)
            return false;
        port_ok = image_output_port.open(port_prefix + ":o");
        if (!port_ok)
            return false;

        // head forward kinematics
        if (!head_kin.configure(robot_name, port_prefix))
            return false;

        // frame transform client
	yarp::os::Property propTfClient;
	propTfClient.put("device", "FrameTransformClient");
	propTfClient.put("local", port_prefix + "/transformClient");
	propTfClient.put("remote", "/transformServer");
	tf_client = nullptr;
	bool ok_drv = drv_transform_client.open(propTfClient);
	ok_drv = ok_drv && drv_transform_client.view(tf_client) && tf_client != nullptr;
	if (!ok_drv)
	    return false;

        // set intrinsic parameters
        double cam_fx;
        double cam_fy;
        double cam_cx;
        double cam_cy;
        if (eye_name == "left")
        {
            cam_fx = 219.057;
            cam_cx = 174.742;
            cam_fy = 219.028;
            cam_cy = 102.874;
        }
        else if (eye_name == "right")
        {
            cam_fx = 225.904;
            cam_cx = 157.858;
            cam_fy = 227.041;
            cam_cy = 113.51;
        }

        return true;
    }

    double getPeriod() override
    {
        return period;
    }

    bool getFrame(yarp::sig::ImageOf<yarp::sig::PixelRgb>* &yarp_image)
    {
	// try to read image from port
	yarp_image = image_input_port.read(false);

	if (yarp_image == NULL)
	    return false;

	return true;
    }

    bool updateModule() override
    {
	// get image from camera
	yarp::sig::ImageOf<yarp::sig::PixelRgb>* img_in;
	if (!getFrame(img_in))
	    return true;

        // get current pose of eyes
        yarp::sig::Vector left_eye_pose;
        yarp::sig::Vector right_eye_pose;
        yarp::sig::Vector eye_pose;
        head_kin.getEyesPose(left_eye_pose, right_eye_pose);
        eye_pose = (eye_name == "left") ? left_eye_pose : right_eye_pose;

        // prepare output image
        // yarp::sig::ImageOf<yarp::sig::PixelRgb> &img_out = image_output_port.prepare();
        // img_out = *img_in;
        // cv::Mat cv_img;
        // cv_img = cv::cvarrToMat(img_out.getIplImage());

        // send image
        image_output_port.write();

        return true;
    }

    bool close() override
    {
	// close ports
	image_input_port.close();
        image_output_port.close();        
        
	// close head kinematics
	head_kin.close();

        // close transform client
        drv_transform_client.close();        

        return true;
    }
};

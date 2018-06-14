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
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>

// opencv
#include <opencv2/opencv.hpp>

//
#include <Tracker.h>

bool Tracker::configure(yarp::os::ResourceFinder &rf)
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

    // set intrinsic parameters
    const yarp::os::ResourceFinder &rf_eye = rf.findNestedResourceFinder(eye_name.c_str());
    cam_fx = 219.057;
    cam_cx = 174.742;
    cam_fy = 219.028;
    cam_cy = 102.874;
    if (!rf_eye.find("camFx").isNull())
        cam_fx = rf_eye.find("camFx").asDouble();
    if (!rf_eye.find("camCx").isNull())
        cam_cx = rf_eye.find("camCx").asDouble();
    if (!rf_eye.find("camFy").asDouble())
        cam_fy = rf_eye.find("camFy").asDouble();
    if (!rf_eye.find("camCy").asDouble())
        cam_cy = rf_eye.find("camCy").asDouble();

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

    // gaze controller
    const yarp::os::ResourceFinder &rf_gaze = rf.findNestedResourceFinder("gazeController");
    if (!gaze_ctrl.configure(rf_gaze))
    {
        yError() << "Tracker::configure"
                 << "error: cannot configure the gaze controller";
        return false;
    }

    return true;
}

double Tracker::getPeriod()
{
    return period;
}

bool Tracker::getFrame(yarp::sig::ImageOf<yarp::sig::PixelRgb>* &yarp_image)
{
    // try to read image from port
    yarp_image = image_input_port.read(false);

    if (yarp_image == NULL)
        return false;

    return true;
}

bool Tracker::updateModule()
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

    // TODO: add aruco estimation here

    // TODO: add gaze tracking here

    // prepare output image
    // yarp::sig::ImageOf<yarp::sig::PixelRgb> &img_out = image_output_port.prepare();
    // img_out = *img_in;
    // cv::Mat cv_img;
    // cv_img = cv::cvarrToMat(img_out.getIplImage());

    // send image
    image_output_port.write();

    return true;
}

bool Tracker::close()
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

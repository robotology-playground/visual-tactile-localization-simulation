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

    // aruco board parameters
    const yarp::os::ResourceFinder &rf_aruco = rf.findNestedResourceFinder("arucoBoard");
    int n_x;
    int n_y;
    double side;
    double separation;
    if (!(rf_aruco.find("nX").isNull()))
        n_x = rf_aruco.find("nX").asInt();
    else
        return false;
    if (!(rf_aruco.find("nY").isNull()))
        n_y = rf_aruco.find("nY").asInt();
    else
        return false;
    if (!(rf_aruco.find("side").isNull()))
        side = rf_aruco.find("side").asDouble();
    else
        return false;
    if (!(rf_aruco.find("separation").isNull()))
        separation = rf_aruco.find("separation").asDouble();
    else
        return false;

    // camera parameters
    const yarp::os::ResourceFinder &rf_eye = rf.findNestedResourceFinder(eye_name.c_str());
    double cam_fx = 219.057;
    double cam_cx = 174.742;
    double cam_fy = 219.028;
    double cam_cy = 102.874;
    double cam_k1 = -0.374173;
    double cam_k2 = 0.205428;
    double cam_p1 = 0.00282356;
    double cam_p2 = 0.00270998;
    
    if (!rf_eye.find("camFx").isNull())
        cam_fx = rf_eye.find("camFx").asDouble();
    if (!rf_eye.find("camCx").isNull())
        cam_cx = rf_eye.find("camCx").asDouble();
    if (!rf_eye.find("camFy").isNull())
        cam_fy = rf_eye.find("camFy").asDouble();
    if (!rf_eye.find("camCy").isNull())
        cam_cy = rf_eye.find("camCy").asDouble();
    if (!rf_eye.find("camK1").isNull())
        cam_k1 = rf_eye.find("camK1").asDouble();
    if (!rf_eye.find("camK2").isNull())
        cam_k2 = rf_eye.find("camK2").asDouble();
    if (!rf_eye.find("camP1").isNull())
        cam_p1 = rf_eye.find("camP1").asDouble();
    if (!rf_eye.find("camP2").isNull())
        cam_p2 = rf_eye.find("camP2").asDouble();

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
    // const yarp::os::ResourceFinder &rf_gaze = rf.findNestedResourceFinder("gazeController");
    // if (!gaze_ctrl.configure(rf_gaze))
    // {
    //     yError() << "Tracker::configure"
    //              << "error: cannot configure the gaze controller";
    //     return false;
    // }

    // aruco board estimator    
    bool ok;
    ok = aruco_estimator.configure(n_x, n_y, side, separation,
                                   cam_fx, cam_fy, cam_cx, cam_cy,
                                   cam_k1, cam_k2, cam_p1, cam_p2);
    if (!ok)
    {
        yError() << "Tracker::configure"
                 << "error: cannot configure the aruco board";
        return false;
    }
    
    return true;
}

double Tracker::getPeriod()
{
    return period;
}

bool Tracker::getFrame(cv::Mat &frame)
{
    // try to read image from port
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *yarp_image = 
    yarp_image = image_input_port.read(false);

    if (yarp_image == NULL)
        return false;
    frame = cv::cvarrToMat(yarp_image->getIplImage(), true);

    return true;
}

bool evaluateEstimate(const cv::Mat &camera_pos, const cv::Mat &camera_att,
                      yarp::sig::Vector &estimate)
{
}

bool Tracker::updateModule()
{
    // get image from camera
    cv::Mat frame_in;
    if (!getFrame(frame_in))
        return true;

    // get current pose of eyes
    yarp::sig::Vector left_eye_pose;
    yarp::sig::Vector right_eye_pose;
    yarp::sig::Vector eye_pose;
    head_kin.getEyesPose(left_eye_pose, right_eye_pose);
    eye_pose = (eye_name == "left") ? left_eye_pose : right_eye_pose;

    // prepare output image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &img_out = image_output_port.prepare();
    cv::Mat frame_out;
    frame_out = cv::cvarrToMat(img_out.getIplImage());

    // aruco board pose estimation
    aruco_estimator.estimateBoardPose(frame_in, frame_out);
    
    // TODO: add gaze tracking here

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

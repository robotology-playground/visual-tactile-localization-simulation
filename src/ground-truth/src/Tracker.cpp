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
#include <ArucoBoardEstimator.h>
#include <CharucoBoardEstimator.h>

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

    // get board type
    std::string board_type;
    board_type = "aruco";
    if (!rf.find("boardType").isNull())
        board_type = rf.find("boardType").asString();

    // uco board parameters
    std::string uco_key = board_type + "Board";
    const yarp::os::ResourceFinder &rf_uco =
        rf.findNestedResourceFinder(uco_key.c_str());
    int n_x;
    int n_y;
    // for aruco boards
    // size1 := marker side (m)
    // size2 := marker separtion (m)
    //
    // for charuco boards
    // size1 := chess square side (m)
    // size2 := marker side (m)
    //
    double size1;
    double size2;
    if (!(rf_uco.find("nX").isNull()))
        n_x = rf_uco.find("nX").asInt();
    else
        return false;
    if (!(rf_uco.find("nY").isNull()))
        n_y = rf_uco.find("nY").asInt();
    else
        return false;
    if (!(rf_uco.find("size1").isNull()))
        size1 = rf_uco.find("size1").asDouble();
    else
        return false;
    if (!(rf_uco.find("size2").isNull()))
        size2 = rf_uco.find("size2").asDouble();
    else
        return false;

    // camera parameters file
    yarp::os::ResourceFinder rf_eye = rf.findNestedResourceFinder(eye_name.c_str());
    std::string cam_calib_path = rf_eye.findFile("camCalibPath");

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
    // if (!head_kin.configure(robot_name, port_prefix))
    //     return false;

    // frame transform client
    // yarp::os::Property propTfClient;
    // propTfClient.put("device", "FrameTransformClient");
    // propTfClient.put("local", port_prefix + "/transformClient");
    // propTfClient.put("remote", "/transformServer");
    // tf_client = nullptr;
    // bool ok_drv = drv_transform_client.open(propTfClient);
    // ok_drv = ok_drv && drv_transform_client.view(tf_client) && tf_client != nullptr;
    // if (!ok_drv)
    //     return false;

    // gaze controller
    // const yarp::os::ResourceFinder &rf_gaze = rf.findNestedResourceFinder("gazeController");
    // if (!gaze_ctrl.configure(rf_gaze))
    // {
    //     yError() << "Tracker::configure"
    //              << "error: cannot configure the gaze controller";
    //     return false;
    // }

    // aruco/charuco board estimator    
    if (board_type == "aruco")
        uco_estimator = std::unique_ptr<ArucoBoardEstimator>(
            new ArucoBoardEstimator());
    else if (board_type == "charuco")
        uco_estimator = std::unique_ptr<CharucoBoardEstimator>(
            new CharucoBoardEstimator());
    else
    {
        yError() << "Tracker::configure"
                 << "error: invalid board type" << board_type;
        return false;
    }

    bool ok;
    ok = uco_estimator->configure(n_x, n_y, size1, size2, cam_calib_path);
    if (!ok)
    {
        yError() << "Tracker::configure"
                 << "error: cannot configure the"
                 << board_type << "board";
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

bool evaluateEstimate(const cv::Mat &camera_pos, const cv::Mat &camera_att,
                      yarp::sig::Vector &estimate)
{
}

bool Tracker::updateModule()
{
    // get image from camera
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* img_in;
    if (!getFrame(img_in))
        return true;

    // get current pose of eyes
    // yarp::sig::Vector left_eye_pose;
    // yarp::sig::Vector right_eye_pose;
    // yarp::sig::Vector eye_pose;
    // head_kin.getEyesPose(left_eye_pose, right_eye_pose);
    // eye_pose = (eye_name == "left") ? left_eye_pose : right_eye_pose;

    // prepare input image
    cv::Mat frame_in;
    frame_in = cv::cvarrToMat(img_in->getIplImage());

    // prepare output image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> &img_out = image_output_port.prepare();
    img_out = *img_in;
    cv::Mat frame_out;
    frame_out = cv::cvarrToMat(img_out.getIplImage());

    // aruco board pose estimation
    uco_estimator->estimateBoardPose(frame_in, frame_out);
    
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

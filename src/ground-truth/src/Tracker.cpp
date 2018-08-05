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

using namespace yarp::math;

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

    publish_images = false;
    if (!rf.find("publishImages").isNull())
        publish_images = rf.find("publishImages").asBool();

    // port prefix
    std::string port_prefix = "/gtruth_tracker/";
    port_prefix += eye_name + "_eye";

    // get board type
    std::string board_type;
    board_type = "aruco";
    if (!rf.find("boardType").isNull())
        board_type = rf.find("boardType").asString();

    simulation_mode = false;
    if (!rf.find("simulation").isNull())
        simulation_mode = rf.find("simulation").asBool();

    if (simulation_mode)
    {
        sim_tf_source = "/iCub/frame";
        if (!rf.find("simTfSource").isNull())
            sim_tf_source = rf.find("simTfSource").asString();
        sim_tf_target = "/box_alt/frame";
        if (!rf.find("simTfTarget").isNull())
            sim_tf_target = rf.find("simTfTarget").asString();
    }
    filter_tf_source = "/iCub/frame";
    if (!rf.find("filterTfSource").isNull())
        filter_tf_source = rf.find("filterTfSource").asString();
    filter_tf_target = "/estimate/frame";
    if (!rf.find("filterTfTarget").isNull())
        filter_tf_target = rf.find("filterTfTarget").asString();

    tracking_source = "ground_truth";
    if (!rf.find("trackingSource").isNull())
        tracking_source = rf.find("trackingSource").asString();

    int n_x;
    int n_y;
    double size1;
    double size2;
    std::string cam_calib_path;
    if (!simulation_mode)
    {
        // uco board parameters
        std::string uco_key = board_type + "Board";
        const yarp::os::ResourceFinder &rf_uco =
            rf.findNestedResourceFinder(uco_key.c_str());
        if (!(rf_uco.find("nX").isNull()))
            n_x = rf_uco.find("nX").asInt();
        else
            return false;
        if (!(rf_uco.find("nY").isNull()))
            n_y = rf_uco.find("nY").asInt();
        else
            return false;
        // for aruco boards
        // size1 := marker side (m)
        // size2 := marker separtion (m)
        //
        // for charuco boards
        // size1 := chess square side (m)
        // size2 := marker side (m)
        //
        if (!(rf_uco.find("size1").isNull()))
            size1 = rf_uco.find("size1").asDouble();
        else
            return false;
        if (!(rf_uco.find("size2").isNull()))
            size2 = rf_uco.find("size2").asDouble();
        else
            return false;

        // object sizes
        const yarp::os::ResourceFinder &rf_object_size =
            rf.findNestedResourceFinder("objectSizes");
        obj_width = obj_depth = obj_height = 0.0;
        if (!(rf_object_size.find("width").isNull()))
            obj_width = rf_object_size.find("width").asDouble();
        if (!(rf_object_size.find("depth").isNull()))
            obj_depth = rf_object_size.find("depth").asDouble();
        if (!(rf_object_size.find("height").isNull()))
            obj_height = rf_object_size.find("height").asDouble();

        // camera parameters file
        yarp::os::ResourceFinder rf_eye = rf.findNestedResourceFinder(eye_name.c_str());
        cam_calib_path = rf_eye.findFile("camCalibPath");

        // open port
        bool port_ok = image_input_port.open(port_prefix + ":i");
        if (!port_ok)
            return false;
        if (publish_images)
        {
            port_ok = image_output_port.open(port_prefix + ":o");
            if (!port_ok)
                return false;
        }

        // head forward kinematics
        // if (!head_kin.configure(robot_name, port_prefix))
        //     return false;

    }

    // frame transform client
    yarp::os::Property propTfClient;
    propTfClient.put("device", "transformClient");
    propTfClient.put("local", port_prefix + "/transformClient");
    propTfClient.put("remote", "/transformServer");
    tf_client = nullptr;
    bool ok_drv = drv_transform_client.open(propTfClient);
    ok_drv = ok_drv && drv_transform_client.view(tf_client) && tf_client != nullptr;
    if (!ok_drv)
        return false;

    // gaze controller
    const yarp::os::ResourceFinder &rf_gaze = rf.findNestedResourceFinder("gazeController");
    if (!gaze_ctrl.configure(rf_gaze, port_prefix))
    {
        yError() << "Tracker::configure"
                 << "error: cannot configure the gaze controller";
        return false;
    }
    gaze_ctrl.storeContext();
    if (!gaze_ctrl.setTrajectoryTimes())
    {
        yError() << "Tracker::configure"
                 << "error: cannot set the trajectory times for the gaze controller";

        // close the gaze controller
        gaze_ctrl.close();

        return false;
    }

    if (!simulation_mode)
    {
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
        // override camera intrinsics using information
        // from the iKinGazeCtrl
        double fx, fy, cx, cy;
        gaze_ctrl.getCameraIntrinsics(eye_name, fx, fy, cx, cy);
        uco_estimator->setCameraIntrinsics(fx, fy, cx, cy);
    }

    // resize estimate matrix
    // est_pos.resize(3, 0.0);
    // est_att.resize(4, 0.0);
    yarp::sig::Matrix est_pose(4, 4);
    est_pose.zero();

    // reset flags
    is_estimate_available = false;
    status = Status::Idle;

    // start rpc server
    rpc_port.open("/vtl-gtruth-tracker/rpc");
    attach(rpc_port);

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

bool Tracker::evaluateEstimate(const cv::Mat &pos_wrt_cam, const cv::Mat &att_wrt_cam,
                               const yarp::sig::Vector &camera_pos,
                               const yarp::sig::Vector &camera_att,
                               yarp::sig::Matrix &est_pose)
{
    // transformation matrix
    // from robot root to camera
    yarp::sig::Matrix root_to_cam(4, 4);
    root_to_cam.zero();
    root_to_cam(3, 3) = 1.0;

    root_to_cam(0, 3) = camera_pos[0];
    root_to_cam(1, 3) = camera_pos[1];
    root_to_cam(2, 3) = camera_pos[2];

    root_to_cam.setSubmatrix(yarp::math::axis2dcm(camera_att).submatrix(0, 2, 0, 2),
                             0, 0);

    // transformation matrix
    // from camera to corner of the object
    yarp::sig::Matrix cam_to_obj(4,4);
    cam_to_obj.zero();
    cam_to_obj(3, 3) = 1.0;

    cam_to_obj(0, 3) = pos_wrt_cam.at<double>(0, 0);
    cam_to_obj(1, 3) = pos_wrt_cam.at<double>(1, 0);
    cam_to_obj(2, 3) = pos_wrt_cam.at<double>(2, 0);

    cv::Mat att_wrt_cam_matrix;
    cv::Rodrigues(att_wrt_cam, att_wrt_cam_matrix);
    yarp::sig::Matrix att_wrt_cam_yarp(3, 3);
    for (size_t i=0; i<3; i++)
        for (size_t j=0; j<3; j++)
            att_wrt_cam_yarp(i, j) = att_wrt_cam_matrix.at<double>(i, j);
    cam_to_obj.setSubmatrix(att_wrt_cam_yarp, 0, 0);

    // compose transformations
    est_pose = root_to_cam * cam_to_obj;

    // pick the center of the object
    yarp::sig::Vector corner_to_center(4, 0.0);
    yarp::sig::Vector est_pos_homog(4, 0.0);
    corner_to_center[0] = obj_width / 2.0;
    corner_to_center[1] = obj_depth / 2.0;
    corner_to_center[2] = -obj_height / 2.0;
    corner_to_center[3] = 1.0;
    est_pos_homog = est_pose * corner_to_center;
    est_pose.setCol(3, est_pos_homog);
}

bool Tracker::retrieveGroundTruthSim(yarp::sig::Matrix &pose)
{
    // Get the transform from the robot root frame
    // to the object frame provided by Gazebo
    if (!tf_client->getTransform(sim_tf_target, sim_tf_source, pose))
        return false;

    return true;
}

bool Tracker::retrieveFilterEstimate(yarp::sig::Matrix &pose)
{
    if (!tf_client->getTransform(filter_tf_target, filter_tf_source, pose))
        return false;

    return true;
}

void Tracker::publishEstimate()
{
    if (!is_estimate_available)
        return;

    // set a new transform
    tf_client->setTransform(tf_target, tf_source, est_pose);
}

void Tracker::getEstimate(yarp::sig::Vector &estimate)
{
    estimate.resize(6);

    // position
    estimate.setSubvector(0, est_pose.getCol(3).subVector(0, 2));

    // attitude
    estimate.setSubvector(3, yarp::math::dcm2rpy(est_pose.submatrix(0, 2, 0, 2)));
}

void Tracker::fixateWithEyes()
{
    // this function can be called once
    // or in streaming mode
    yarp::sig::Matrix *estimate;
    bool do_fixate = false;

    if ((tracking_source == "ground_truth") &&
        is_estimate_available)
    {
        estimate = &est_pose;
        do_fixate = true;
    }
    else if ((tracking_source == "filter") &&
             is_filter_est_available)
    {
        estimate = &filter_pose;
        do_fixate = true;
    }

    if (do_fixate)
    {
        yarp::sig::Vector fix_point;
        fix_point = estimate->getCol(3).subVector(0, 2);

        gaze_ctrl.setReference(fix_point);
    }
}

void Tracker::fixateWithEyesAndHold()
{
    // this function should be called once

    // set the fixation point
    // using the current estimate
    fixateWithEyes();

    // enable tracking mode
    gaze_ctrl.enableTrackingMode();
}

bool Tracker::updateModule()
{
    /*
     * Ground truth estimation
     */
    if (simulation_mode)
    {
        bool ok = retrieveGroundTruthSim(est_pose);

        if (ok)
            is_estimate_available = true;
    }
    else
    {
        // retrieve estimate from the filter
        bool ok = retrieveFilterEstimate(filter_pose);

        if (ok)
            is_filter_est_available = true;

        // get image from camera
        yarp::sig::ImageOf<yarp::sig::PixelRgb>* img_in;
        bool is_image = false;
        is_image = getFrame(img_in);

        cv::Mat frame_out;
        if (is_image)
        {
            // get current pose of eyes
            // yarp::sig::Vector left_eye_pose;
            // yarp::sig::Vector right_eye_pose;
            // yarp::sig::Vector eye_pose;
            // head_kin.getEyesPose(left_eye_pose, right_eye_pose);
            // eye_pose = (eye_name == "left") ? left_eye_pose : right_eye_pose;
            yarp::sig::Vector eye_pos;
            yarp::sig::Vector eye_att;
            gaze_ctrl.getCameraPose(eye_name, eye_pos, eye_att);

            // prepare input image
            cv::Mat frame_in;
            frame_in = cv::cvarrToMat(img_in->getIplImage());
            cv::cvtColor(frame_in, frame_in, cv::COLOR_RGB2BGR);

            if (publish_images)
            {
                // prepare output image
                yarp::sig::ImageOf<yarp::sig::PixelRgb> &img_out = image_output_port.prepare();
                img_out = *img_in;
                frame_out = cv::cvarrToMat(img_out.getIplImage());
            }

            // aruco board pose estimation
            cv::Mat pos_wrt_cam;
            cv::Mat att_wrt_cam;
            bool ok;
            ok = uco_estimator->estimateBoardPose(frame_in, frame_out,
                                                  pos_wrt_cam, att_wrt_cam,
                                                  publish_images);
            if (ok)
            {
                is_estimate_available = true;

                evaluateEstimate(pos_wrt_cam, att_wrt_cam,
                                 eye_pos, eye_att,
                                 est_pose);
            }

            if (publish_images)
            {
                // send image
                cv::cvtColor(frame_out, frame_out, cv::COLOR_BGR2RGB);
                image_output_port.write();
            }

        }

        // publish the last available estimate
        publishEstimate();

    }

    /*
     * Tracking with eyes
     */
    mutex.lock();
    Status st = status;
    mutex.unlock();
    switch (st)
    {
    case Status::Idle:
    {
        // nothing to do here
        break;
    }
    case Status::Hold:
    {
        // the tracker fixate at the object
        // using the current estimate
        // and enable tracking mode of the iKinGaze controller
        fixateWithEyesAndHold();

        // go back to idle
        mutex.lock();
        status = Status::Idle;
        mutex.unlock();

        break;
    }
    case Status::Track:
    {
        // the tracker continuously tracks
        // the object using the last estimate available
        fixateWithEyes();
        break;
    }
    }

    return true;
}

bool Tracker::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
{
    std::string cmd = command.get(0).asString();
    if (cmd == "help")
    {
        reply.addVocab(yarp::os::Vocab::encode("many"));
        reply.addString("Available commands:");
        reply.addString("- eyes-track");
        reply.addString("- eyes-fixate-and-hold");
        reply.addString("- eyes-stop");
        reply.addString("- quit");
    }
    else if (cmd == "eyes-track")
    {
        mutex.lock();

        // disable tracking mode of iKinGazeCtrl
        gaze_ctrl.disableTrackingMode();

        // unblock the eyes in case they were blocked
        gaze_ctrl.clearEyes();

        status = Status::Track;

        mutex.unlock();

        reply.addString("ok");
    }
    else if (cmd == "eyes-fixate-and-hold")
    {
        mutex.lock();

        // unblock the eyes in case they were blocked
        gaze_ctrl.clearEyes();

        status = Status::Hold;

        mutex.unlock();

        reply.addString("ok");
    }
    else if (cmd == "eyes-stop")
    {
        mutex.lock();

        gaze_ctrl.stop();
        gaze_ctrl.disableTrackingMode();

        status = Status::Idle;

        mutex.unlock();

        reply.addString("ok");
    }
    else
        // the father class already handles the "quit" command
        return RFModule::respond(command,reply);

    return true;
}

bool Tracker::close()
{
    // close ports
    image_input_port.close();
    image_output_port.close();

    // close head kinematics
    // head_kin.close();

    // close gaze controller
    gaze_ctrl.restoreContext();
    gaze_ctrl.close();

    // close transform client
    drv_transform_client.close();

    return true;
}

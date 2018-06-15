/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

#ifndef TRACKER_H
#define TRACKER_H

// yarp
#include <yarp/os/RFModule.h>
#include <yarp/sig/Image.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>

// opencv
#include <opencv2/opencv.hpp>

//
#include <HeadKinematics.h>
#include <Tracker.h>
#include <ArucoBoardEstimator.h>
#include <GazeController.h>

class Tracker : public yarp::os::RFModule
{
private:
    // head kinematics
    headKinematics head_kin;

    // gaze controller
    GazeController gaze_ctrl;

    // aruco board estimator
    ArucoBoardEstimator aruco_estimator;

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

public:
    bool configure(yarp::os::ResourceFinder &rf) override;
    double getPeriod() override;
    bool getFrame(cv::Mat &frame);
    bool updateModule() override;
    bool close() override;
};

#endif

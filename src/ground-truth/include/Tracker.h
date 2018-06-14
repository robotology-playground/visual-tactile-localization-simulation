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

#include <HeadKinematics.h>

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
    bool configure(yarp::os::ResourceFinder &rf) override;
    double getPeriod() override;
    bool getFrame(yarp::sig::ImageOf<yarp::sig::PixelRgb>* &yarp_image);
    bool updateModule() override;
    bool close() override;
};

#endif

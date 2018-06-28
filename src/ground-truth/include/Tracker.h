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
#include <UcoBoardEstimator.h>
#include <GazeController.h>

enum class Status { Idle, Hold, Track};

class Tracker : public yarp::os::RFModule
{
private:
    // head kinematics
    /* headKinematics head_kin; */

    // gaze controller
    GazeController gaze_ctrl;

    // aruco/charuco board estimator
    std::unique_ptr<UcoBoardEstimator> uco_estimator;

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

    // object sizes
    double obj_width;
    double obj_depth;
    double obj_height;

    // estimate
    yarp::sig::Matrix est_pose;
    /* yarp::sig::Vector est_pos; */
    /* yarp::sig::Vector est_att; */
    bool is_estimate_available;

    // status
    Status status;

    bool getFrame(yarp::sig::ImageOf<yarp::sig::PixelRgb>* &yarp_image);

    /*
     * Evaluate the estimate of the object w.r.t the root frame given
     * the estimated pose of the object w.r.t the camera and
     * the pose of the camera w.r.t the root frame
     *
     * @param pos_wrt_cam estimated position of the object w.r.t the camera
     * @param att_wrt_cam estimated attitude of the object w.r.t the camera 
     * @param camera_pos position of the camera w.r.t the root frame
     * @param camera_pos attitude of the camera w.r.t the root frame
     * @param est_pose estimated pose of the object w.r.t root frame
     */
    bool evaluateEstimate(const cv::Mat &pos_wrt_cam, const cv::Mat &att_wrt_cam,
                          const yarp::sig::Vector &camera_pos,
                          const yarp::sig::Vector &camera_att,
                          yarp::sig::Matrix est_pose);
    void publishEstimate();
    void trackObjectWithEyes();

public:
    bool configure(yarp::os::ResourceFinder &rf) override;
    double getPeriod() override;
    bool updateModule() override;
    bool close() override;
};

#endif

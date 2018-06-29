/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

#ifndef UCO_BOARD_ESTIMATOR
#define UCO_BOARD_ESTIMATOR

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class UcoBoardEstimator
{
protected:
    cv::Ptr<cv::aruco::Board> board;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params;
    cv::Mat cam_intrinsic;
    cv::Mat cam_distortion;

    // for visualization purposes
    double axis_length;

    bool readCameraParameters(std::string filename);    
public:
    virtual bool configure(const int &n_x, const int &n_y, const double &size1, const double &size2,
                           const std::string &cam_calib_path) = 0;
    virtual bool estimateBoardPose(const cv::Mat &img_in, cv::Mat &img_out,
                                   cv::Mat &pos, cv::Mat &att,
                                   const bool &draw_estimate = false) = 0;
    bool setCameraIntrinsics(const double &fx, const double &fy,
                             const double &cx, const double &cy);
};

#endif

/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

#ifndef CHARUCO_BOARD_ESTIMATOR
#define CHARUCO_BOARD_ESTIMATOR

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/charuco.hpp>

//
#include <UcoBoardEstimator.h>

class CharucoBoardEstimator : public UcoBoardEstimator
{
private:
    cv::Ptr<cv::aruco::CharucoBoard> charucoboard;
public:
    bool configure(const int &n_x, const int &n_y, const double &size1, const double &size2,
                   const std::string &cam_calib_path) override;
    bool estimateBoardPose(const cv::Mat &img_in, cv::Mat &img_out,
                           cv::Mat &pos, cv::Mat &att) override;
};

#endif

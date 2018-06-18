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

class ArucoBoardEstimator
{
private:
    cv::Ptr<cv::aruco::Board> board;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Mat camIntrinsic;
    cv::Mat camDistortion;
public:
    virtual bool configure(const int &n_x, const int &n_y, const double &size1, const double &size2,
                           cv::Mat &camMatrix, cv::Mat &distCoeffs) = 0;
    /*
     *
     */
    virtual void estimateBoardPose(const cv::Mat &img_in, cv::Mat &img_out) = 0;
};

#endif

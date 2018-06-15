/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Nicola Piga
 */

#ifndef ARUCO_BOARD_ESTIMATOR
#define ARUCO_BOARD_ESTIMATOR

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
    bool configure(const int &n_x, const int &n_y, const double &side, const double &separation,
                   const double &fx, const double &fy, const double &cx, const double &cy,
                   const double &k1, const double &k2, const double &p1, const double &p2);
    /*
     *
     */
    void estimateBoardPose(const cv::Mat &img_in, cv::Mat &img_out);
};

#endif

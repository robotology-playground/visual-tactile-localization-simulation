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
#include <yarp/os/LogStream.h>

//
#include <ArucoBoardEstimator.h>

bool ArucoBoardEstimator::configure(const int &n_x, const int &n_y, const double &side, const double &separation,
                                    const double &fx, const double &fy, const double &cx, const double &cy,
                                    const double &k1, const double &k2, const double &p1, const double &p2)
{
    if ((n_x <= 0) || (n_y <= 0) || (side <= 0) || (separation < 0))
        return false;
    
    // configure a standard Aruco dictionary
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // configure the board
    board = cv::aruco::GridBoard::create(n_x, n_y, side, separation, dictionary);

    // configure camera parameters
    camIntrinsic = cv::Mat::zeros(3,3, CV_64F);
    camIntrinsic.at<double>(0,0) = fx;
    camIntrinsic.at<double>(1,1) = fy;
    camIntrinsic.at<double>(0,2) = cx;
    camIntrinsic.at<double>(1,2) = cy;
    camIntrinsic.at<double>(2,2) = 1;

    camDistortion = cv::Mat::zeros(1, 4, CV_64F);    
    camDistortion.at<double>(0,0) = k1;
    camDistortion.at<double>(0,1) = k2;
    camDistortion.at<double>(0,2) = p1;
    camDistortion.at<double>(0,3) = p2;

    return true;
}

void ArucoBoardEstimator::estimateBoardPose(const cv::Mat &img_in, cv::Mat &img_out)
{
    // copy input image to output image
    img_in.copyTo(img_out);

    // perform marker detection
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(img_in, dictionary, corners, ids);
    
    // if at least one marker detected
    if (ids.size() > 0)
    {
        cv::aruco::drawDetectedMarkers(img_out, corners, ids);
        cv::Vec3d rvec, tvec;
        int valid = estimatePoseBoard(corners, ids, board,
                                      camIntrinsic, camDistortion,
                                      rvec, tvec);
        // if at least one board marker detected
        if(valid > 0)
            cv::aruco::drawAxis(img_out, camIntrinsic, camDistortion, rvec, tvec, 0.1);
    }
}

